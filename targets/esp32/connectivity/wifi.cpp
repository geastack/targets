#include "wifi.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#endif
#if __has_include("gea_embedded_app_config.h")
#include "gea_embedded_app_config.h"
#endif

#ifndef GEA_EMBEDDED_WIFI_SSID
#define GEA_EMBEDDED_WIFI_SSID ""
#endif

#ifndef GEA_EMBEDDED_WIFI_PASSWORD
#define GEA_EMBEDDED_WIFI_PASSWORD ""
#endif

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <string>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

#include "connectivity/wifi_task_stack.h"
#include "services/network_services.h"

#include "display.h"

namespace gea::targets::esp32::wifi {

namespace {

constexpr const char *kTag = "wifi";
constexpr int kScanMax = 20;
constexpr EventBits_t kConnectedBit = BIT0;
constexpr EventBits_t kFailBit = BIT1;

}  // namespace

class WifiStation final : public gea::framework::network::WifiDriver {
public:
	static WifiStation &instance()
	{
		static WifiStation station;
		return station;
	}

	bool init() override
	{
		// init() may be re-entered after a failed esp_wifi_init (the bring-up
		// worker retries); only create what a previous attempt didn't leave behind.
		if (!eventGroup_) eventGroup_ = xEventGroupCreate();
		// Keep credentials set via configure(); fall back to the build-time
		// defaults only when none were provided before this (deferred) bring-up.
		if (ssid_[0] == '\0') {
			std::snprintf(ssid_, sizeof(ssid_), "%s", GEA_EMBEDDED_WIFI_SSID);
			std::snprintf(password_, sizeof(password_), "%s", GEA_EMBEDDED_WIFI_PASSWORD);
		}

		ESP_ERROR_CHECK(esp_netif_init());
		// The default event loop may already exist when WiFi is brought up lazily
		// after boot; tolerate ESP_ERR_INVALID_STATE rather than aborting.
		{
			const esp_err_t loopErr = esp_event_loop_create_default();
			if (loopErr != ESP_OK && loopErr != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(loopErr);
		}
		if (!netif_) netif_ = esp_netif_create_default_wifi_sta();

		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#if CONFIG_ESP_WIFI_ENABLED
		// Use the smallest fixed DMA pools that leave room for BLE, the CO5300 SPI
		// bus, and the HTTP task on this S3. Hardware measurement after BLE showed
		// only 23,824 DMA bytes before WiFi: even a 3/8 RX, 2/1 TX/cache midpoint
		// associated but left 1,212 bytes, and display initialization failed. The
		// original slow OTA run also had a 6 KiB WiFi task and an 8 KiB HTTP task;
		// those stacks are now 8 KiB and 4 KiB respectively, so keep the viable
		// pool sizes and measure the corrected task configuration independently.
		// TCP OTA is the primary bulk receive path. Buy its receive depth with the
		// DYNAMIC frames, not the static ones: static RX frames are permanent
		// internal DMA allocations (1600 B each) taken from the same few tens of KiB
		// the display staging needs, while dynamic frames are allocated on demand
		// and land in PSRAM under CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP. Sixteen
		// dynamic frames match the widened TCP receive window; four static frames
		// are the minimum that still permits an 8-deep block-ack window.
		cfg.static_rx_buf_num = 4;
		cfg.dynamic_rx_buf_num = 16;
		cfg.rx_ba_win = 8;  // ESP-IDF requires <= 2 * static_rx_buf_num.
		cfg.static_tx_buf_num = 1;
		cfg.cache_tx_buf_num = 1;
		cfg.mgmt_sbuf_num = 6;
		cfg.ampdu_tx_enable = 0;
		// Give the blob's "wifi" task a stack it can authenticate on.
		applyTaskStackOverride(&cfg);
#endif  // CONFIG_ESP_WIFI_ENABLED — native RX-buf/osi tuning; remote (P4/C6) uses defaults
		// esp_wifi_init wants ~50 KB of DMA-capable internal RAM. The display's
		// elastic-RAM shrink (reserveInternal in ensureBringUp) frees it, but the
		// shrink is applied by the frame task and can lose the race against this
		// first allocation — a one-shot init then leaves WiFi down forever. Retry
		// with backoff so a transiently tight heap doesn't become permanent.
		ESP_LOGW(kTag, "pre-wifi-init heap: dma[free=%u largest=%u] int8[free=%u largest=%u]",
		    static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)),
		    static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)),
		    static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		    static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
		debugDmaFree_.store(
			heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
			std::memory_order_relaxed);
		debugDmaLargest_.store(
			heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
			std::memory_order_relaxed);
		// A failed esp_wifi_init can retain partial internal allocations. Retrying
		// against the same memory state compounds pressure and cannot succeed; the
		// outer bring-up scheduler is responsible for preparing memory first.
		const esp_err_t err = esp_wifi_init(&cfg);
		if (err != ESP_OK) {
			debugLastError_.store(static_cast<int>(err), std::memory_order_relaxed);
			debugState_.store(4, std::memory_order_relaxed);
			ESP_LOGE(kTag, "esp_wifi_init failed (%s); continuing without WiFi", esp_err_to_name(err));
			return false;
		}
		initialized_ = true;

		ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiStation::eventHandler, this, &evtAnyId_));
		ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiStation::eventHandler, this, &evtGotIp_));

		wifi_config_t wifiConfig = {};
		setCredentials(&wifiConfig, ssid_, password_);
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
		ESP_ERROR_CHECK(esp_wifi_start());
		started_ = true;
		// Disable modem power-save. The default WIFI_PS_MIN_MODEM sleeps the radio
		// between DTIM beacons; over a sustained multi-minute TLS upload the link
		// goes unreliable and the server's response is lost (the device streams all
		// 37 MB but never receives the /v1/files reply -> "upload HTTP -4"). An
		// always-on radio keeps the connection alive end-to-end. It does not cost
		// extra internal DMA RAM, so it is safe alongside the SD card's DMA reads.
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_ps(WIFI_PS_NONE));

		const EventBits_t bits = xEventGroupWaitBits(
			eventGroup_,
			kConnectedBit | kFailBit,
			pdFALSE,
			pdFALSE,
			pdMS_TO_TICKS(15000));

		if (bits & kConnectedBit) return true;
		debugState_.store(5, std::memory_order_relaxed);
		ESP_LOGE(kTag, "WiFi connection timed out");
		return false;
	}

	bool enabled() const override { return enabled_; }

	void setEnabled(bool enabled) override
	{
#ifdef GEA_EMBEDDED_WIFI_DISABLED
		(void)enabled;
		enabled_ = false;
		return;
#endif
		enabled_ = enabled;

		if (!enabled_) {
			if (!initialized_) return;
			if (scanning_) {
				ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_scan_stop());
				scanning_ = false;
			}
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());
			// Full deinit to RECLAIM WiFi's DMA-capable internal RAM — esp_wifi_stop
			// alone leaves the ~50-60 KB of buffers allocated. Tear down in reverse
			// of init() (deinit -> unregister handlers -> destroy netif -> delete
			// event group) so a later re-enable (ensureBringUp -> init()) re-creates
			// everything cleanly with no double-register/double-create.
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_deinit());
			if (evtAnyId_) {
				esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, evtAnyId_);
				evtAnyId_ = nullptr;
			}
			if (evtGotIp_) {
				esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, evtGotIp_);
				evtGotIp_ = nullptr;
			}
			if (netif_) {
				esp_netif_destroy_default_wifi(netif_);
				netif_ = nullptr;
			}
			if (eventGroup_) {
				vEventGroupDelete(eventGroup_);
				eventGroup_ = nullptr;
			}
			started_ = false;
			initialized_ = false;
			resetConnection();
			// Release the staging-budget reservation so the display grows its
			// staging buffers back now that WiFi's internal RAM is freed.
			gea::platform::display::Display::reserveInternal(0);
			return;
		}

		// Enable. First time: WiFi isn't initialized yet (opt-in — it's no longer
		// brought up at boot), so kick off the async internal-stack bring-up worker
		// and return immediately; the app polls connected(). Already initialized but
		// stopped: just restart the radio. Already running: nothing to do.
		if (!initialized_) {
			ensureBringUp();
			return;
		}
		if (started_) return;
		if (ssid_[0] != '\0') {
			wifi_config_t wifiConfig = {};
			setCredentials(&wifiConfig, ssid_, password_);
			ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
		}
		const esp_err_t err = esp_wifi_start();
		if (err == ESP_OK) started_ = true;
		else ESP_LOGW(kTag, "esp_wifi_start failed while enabling WiFi: %s", esp_err_to_name(err));
		if (started_ && ssid_[0] != '\0') ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
	}

	bool connected() const override
	{
		return connected_;
	}

	int rssi() override
	{
		wifi_ap_record_t ap = {};
		if (connected_ && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi_ = ap.rssi;
		return connected_ ? rssi_ : 0;
	}

	std::string ssid() override
	{
		wifi_ap_record_t ap = {};
		if (connected_ && esp_wifi_sta_get_ap_info(&ap) == ESP_OK && ap.ssid[0] != '\0') {
			std::snprintf(ssid_, sizeof(ssid_), "%s", reinterpret_cast<const char *>(ap.ssid));
		}
		return connected_ ? stringOrEmpty(ssid_) : std::string();
	}

	std::string ip() const override { return connected_ ? stringOrEmpty(ip_) : std::string("0.0.0.0"); }

	std::string mac() override
	{
		std::uint8_t macBytes[6] = {};
		esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, macBytes);
		if (err != ESP_OK) err = esp_read_mac(macBytes, ESP_MAC_WIFI_STA);
		if (err != ESP_OK) return std::string();
		std::snprintf(mac_, sizeof(mac_), "%02X:%02X:%02X:%02X:%02X:%02X",
			macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
		return stringOrEmpty(mac_);
	}

	void configure(const std::string &ssid, const std::string &password) override
	{
		const bool unchanged = ssid == ssid_ && password == password_;
		if (unchanged && initialized_ && connected_) {
			enabled_ = true;
			return;
		}
		debugState_.store(1, std::memory_order_relaxed);
		std::snprintf(ssid_, sizeof(ssid_), "%s", ssid.c_str());
		std::snprintf(password_, sizeof(password_), "%s", password.c_str());
		resetConnection();
		enabled_ = true;

		// WiFi not up yet (opt-in): bring it up lazily with these credentials.
		if (!initialized_) {
			ensureBringUp();
			return;
		}
		if (ssid_[0] == '\0') return;

		wifi_config_t wifiConfig = {};
		setCredentials(&wifiConfig, ssid.c_str(), password.c_str());
		if (!started_ && esp_wifi_start() == ESP_OK) started_ = true;
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_disconnect());
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wifiConfig));
		ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
	}

	void scan() override
	{
		if (!initialized_ || !enabled_ || scanning_) return;

		wifi_scan_config_t scanConfig = {};
		scanConfig.scan_type = WIFI_SCAN_TYPE_ACTIVE;
		scanConfig.scan_time.active.min = 100;
		scanConfig.scan_time.active.max = 300;

		if (esp_wifi_scan_start(&scanConfig, false) == ESP_OK) scanning_ = true;
	}

	bool scanning() const override { return scanning_; }
	int scanCount() const override { return scanCount_; }

	gea::framework::network::WifiNetwork networkAt(int index) const override
	{
		if (index < 0 || index >= scanCount_) return {};
		return {
			stringOrEmpty(scanSsids_[index]),
			scanRssi_[index],
			scanSecured_[index] != 0,
		};
	}

	std::vector<gea::framework::network::WifiNetwork> scanResults() const override
	{
		std::vector<gea::framework::network::WifiNetwork> results;
		results.reserve(scanCount_ > 0 ? static_cast<std::size_t>(scanCount_) : 0);
		for (int i = 0; i < scanCount_; i++) results.push_back(networkAt(i));
		return results;
	}

private:
	static void copyWifiString(std::uint8_t *destination, std::size_t destinationSize, const char *source)
	{
		std::memset(destination, 0, destinationSize);
		if (!source) return;

		std::size_t length = 0;
		while (length < destinationSize && source[length] != '\0') length++;
		std::memcpy(destination, source, length);
	}

	static void setCredentials(wifi_config_t *config, const char *ssid, const char *password)
	{
		copyWifiString(config->sta.ssid, sizeof(config->sta.ssid), ssid);
		copyWifiString(config->sta.password, sizeof(config->sta.password), password);
	}

	static std::string stringOrEmpty(const char *value)
	{
		return value ? std::string(value) : std::string();
	}

	static void eventHandler(void *arg, esp_event_base_t eventBase, std::int32_t eventId, void *eventData)
	{
		static_cast<WifiStation *>(arg)->handleEvent(eventBase, eventId, eventData);
	}

	void handleEvent(esp_event_base_t eventBase, std::int32_t eventId, void *eventData)
	{
		if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
			if (enabled_ && ssid_[0] != '\0') esp_wifi_connect();
		} else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
			if (enabled_) ESP_LOGW(kTag, "WiFi disconnected, retrying...");
			resetConnection();
			if (enabled_ && ssid_[0] != '\0') esp_wifi_connect();
		} else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
			auto *event = static_cast<ip_event_got_ip_t *>(eventData);
			ESP_LOGI(kTag, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
			connected_ = true;
			debugState_.store(6, std::memory_order_relaxed);
			std::snprintf(ip_, sizeof(ip_), IPSTR, IP2STR(&event->ip_info.ip));
			xEventGroupSetBits(eventGroup_, kConnectedBit);
			// Association can finish after init()'s bounded wait has returned. Start
			// the deferred OTA/diagnostics services on the actual address event too;
			// NetworkServices ignores this until Runtime has supplied its policy.
			gea::framework::services::NetworkServices::startDeferredServers();
		} else if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_SCAN_DONE) {
			handleScanDone();
		}
	}

	void resetConnection()
	{
		connected_ = false;
		rssi_ = 0;
		std::snprintf(ip_, sizeof(ip_), "0.0.0.0");
	}

	void handleScanDone()
	{
		std::uint16_t found = 0;
		if (esp_wifi_scan_get_ap_num(&found) != ESP_OK) {
			scanning_ = false;
			return;
		}

		std::uint16_t toRead = found > kScanMax ? kScanMax : found;
		if (toRead > 0) {
			std::uint16_t actual = toRead;
			if (esp_wifi_scan_get_ap_records(&actual, scanRecords_) != ESP_OK) {
				esp_wifi_clear_ap_list();
				scanning_ = false;
				return;
			}
			toRead = actual;
		}

		for (int i = 0; i < toRead; i++) {
			std::snprintf(scanSsids_[i], sizeof(scanSsids_[i]), "%s", reinterpret_cast<const char *>(scanRecords_[i].ssid));
			scanRssi_[i] = scanRecords_[i].rssi;
			scanSecured_[i] = scanRecords_[i].authmode == WIFI_AUTH_OPEN ? 0 : 1;
		}
		scanCount_ = toRead;
		scanning_ = false;
	}

	void ensureBringUp()
	{
#ifdef GEA_EMBEDDED_WIFI_DISABLED
		return;
#endif
		if (initialized_ || bringingUp_ || bringUpDeferred_) return;
		// Always reserve display staging RAM before allocating the transient
		// worker. App-level configure() runs during generated top-level code, before
		// the frame scheduler has necessarily applied a pending display resize; an
		// immediate worker could reach esp_wifi_init with only ~8 KB DMA free. The
		// configure path runs on the app/frame task, so apply the resize here before
		// deferring the stack-hungry native initialization to its worker.
		gea::platform::display::Display::reserveInternal(56 * 1024);
		gea::platform::display::Display::applyPendingInternalReserve();
		if (startBringUpWorker()) {
			// The generated app task has the top FreeRTOS priority. Yield once to the
			// equally prioritized worker so Wi-Fi claims the released DMA RAM before
			// mount() resumes and native UI allocations reuse it.
			taskYIELD();
			return;
		}
		bringUpDeferred_ = true;
		bringUpDeferFrames_ = 0;
		debugState_.store(3, std::memory_order_relaxed);
		scheduleDeferredBringUp();
	}

	// Create the transient internal-stack bring-up worker. esp_wifi_init +
	// esp_netif setup are stack-hungry (the original boot path ran them on the
	// 32 KB main task; 8 KB overflowed, 24 KB left no contiguous internal block
	// for esp_wifi_init's own driver-task stack), so 16 KB is the tuned margin,
	// freed as soon as bring-up ends. Returns false if the stack can't be
	// allocated, so the caller can free RAM and retry instead of giving up.
	bool startBringUpWorker()
	{
		bringingUp_ = true;
		// 10 KB (was 13/16 KB): this stack is held in internal RAM THROUGHOUT
		// esp_wifi_init, so it bounds the largest contiguous DMA block esp_wifi can
		// carve for its own buffers. On maps the static esf_buf pool now fits, but
		// the next esp_wifi alloc needs > the ~11 KB largest free block, which the
		// 16 KB worker caps. esp_wifi_init's call depth needs > 8 KB (8 KB
		// overflowed historically), so 10 KB keeps a 2 KB margin while returning
		// another 3 KB to the DMA heap for BLE + WiFi coexistence.
		const BaseType_t ok = xTaskCreateWithCaps(
		    &WifiStation::bringUpTrampoline, "wifi_up", 10240, this,
		    configMAX_PRIORITIES - 1, nullptr,
		    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		if (ok != pdPASS) {
			bringingUp_ = false;
			return false;
		}
		bringUpDeferred_ = false;
		debugState_.store(2, std::memory_order_relaxed);
		return true;
	}

	void scheduleDeferredBringUp()
	{
		if (!bringUpRetryTimer_) {
			const esp_timer_create_args_t args = {
				.callback = &WifiStation::deferredBringUpTimer,
				.arg = this,
				.dispatch_method = ESP_TIMER_TASK,
				.name = "wifi_retry",
				.skip_unhandled_events = true,
			};
			if (esp_timer_create(&args, &bringUpRetryTimer_) != ESP_OK) {
				ESP_LOGE(kTag, "wifi bring-up retry timer create failed");
				return;
			}
		}
		if (!esp_timer_is_active(bringUpRetryTimer_)) {
			// Keep ticking until retryDeferredBringUp() either creates the worker
			// or reaches its bounded timeout. A one-shot timer cannot safely rely on
			// re-arming itself while its callback is still considered active: the
			// active check can suppress the re-arm after the very first tick.
			esp_timer_start_periodic(bringUpRetryTimer_, 50 * 1000);
		}
	}

	static void deferredBringUpTimer(void *arg)
	{
		static_cast<WifiStation *>(arg)->retryDeferredBringUp();
	}

	// After a failed worker create armed the staging shrink, give the frame task
	// a few frames to apply it, then retry independently of application polling.
	// Gives up after roughly three seconds and releases the reservation.
	void retryDeferredBringUp()
	{
		if (!bringUpDeferred_ || initialized_ || bringingUp_) {
			if (bringUpRetryTimer_ && esp_timer_is_active(bringUpRetryTimer_)) {
				esp_timer_stop(bringUpRetryTimer_);
			}
			return;
		}
		bringUpDeferFrames_++;
		if (bringUpDeferFrames_ >= 3 && startBringUpWorker()) {
			if (bringUpRetryTimer_ && esp_timer_is_active(bringUpRetryTimer_)) {
				esp_timer_stop(bringUpRetryTimer_);
			}
			return;
		}
		if (bringUpDeferFrames_ > 60) {
			if (bringUpRetryTimer_ && esp_timer_is_active(bringUpRetryTimer_)) {
				esp_timer_stop(bringUpRetryTimer_);
			}
			bringUpDeferred_ = false;
			gea::platform::display::Display::reserveInternal(0);
			ESP_LOGE(kTag, "wifi bring-up worker create failed after staging shrink; WiFi unavailable");
			return;
		}
	}

	static void bringUpTrampoline(void *arg)
	{
		// Runs on this transient worker's internal-RAM stack: esp_wifi_init reads
		// calibration from NVS, which disables the flash cache (PSRAM unreachable),
		// so the bring-up must not run on the app's PSRAM stack. Deferred to here
		// so WiFi is only initialized when an app actually enables it.
		auto *self = static_cast<WifiStation *>(arg);
		// Elastic-RAM arbiter: free internal RAM for esp_wifi_init's DMA buffers by
		// shrinking the display's staging buffers first, then give the frame task a
		// few frames to apply the shrink before we allocate. Released on disable.
		gea::platform::display::Display::reserveInternal(56 * 1024);
		const bool connected = self->init();
		if (connected) gea::framework::services::NetworkServices::startDeferredServers();
		self->bringingUp_ = false;
		vTaskDeleteWithCaps(nullptr);
	}

public:
	int debugState() const { return debugState_.load(std::memory_order_relaxed); }
	int debugLastError() const { return debugLastError_.load(std::memory_order_relaxed); }
	std::uint32_t debugDmaFree() const { return debugDmaFree_.load(std::memory_order_relaxed); }
	std::uint32_t debugDmaLargest() const { return debugDmaLargest_.load(std::memory_order_relaxed); }


private:
	EventGroupHandle_t eventGroup_ = nullptr;
	bool connected_ = false;
#ifdef GEA_EMBEDDED_WIFI_DISABLED
	bool enabled_ = false;
#else
	bool enabled_ = true;
#endif
	bool initialized_ = false;
	bool bringingUp_ = false;
	// Deferred bring-up state: set when a worker create fails for lack of internal
	// RAM and we've armed a staging shrink to retry once the RAM frees up.
	bool bringUpDeferred_ = false;
	int bringUpDeferFrames_ = 0;
	esp_timer_handle_t bringUpRetryTimer_ = nullptr;
	std::atomic<int> debugState_{0};
	std::atomic<int> debugLastError_{0};
	std::atomic<std::uint32_t> debugDmaFree_{0};
	std::atomic<std::uint32_t> debugDmaLargest_{0};
	bool started_ = false;
	esp_netif_t *netif_ = nullptr;
	esp_event_handler_instance_t evtAnyId_ = nullptr;
	esp_event_handler_instance_t evtGotIp_ = nullptr;
	int rssi_ = 0;
	char ssid_[33] = "";
	char password_[65] = "";
	char ip_[16] = "0.0.0.0";
	char mac_[18] = "";
	bool scanning_ = false;
	int scanCount_ = 0;
	char scanSsids_[kScanMax][33]{};
	int scanRssi_[kScanMax]{};
	int scanSecured_[kScanMax]{};
	wifi_ap_record_t scanRecords_[kScanMax]{};
};

// Explicit driver registration. MUST be called from a TU that is always linked
// (the runtime boot path). Until 2026-05-28, this lived in an anonymous-
// namespace global-ctor pattern (`WifiDriverRegistration` + static instance),
// which compiled fine but the resulting archive entry was never pulled into
// the final ELF: ESP-IDF puts every source under `main/CMakeLists.txt` into
// one `libmain.a` archive, and three different files all named `wifi.cpp`
// (this one, lib/gea-embedded/wifi.cpp, lib/gea-embedded/host/wifi.cpp) all
// archived as `wifi.cpp.obj`. Symbol-driven archive linking only pulls an obj
// in when something external references its symbols; the static-init global
// here has no external references, so this obj was dropped, leaving
// `NullWifiDriver` as the active driver — `init()` returned false instantly,
// `enabled()` always reported false, `setEnabled()` was a no-op.
void registerDriver()
{
	::gea::framework::network::WifiAdapter::setDriver(&WifiStation::instance());
}

namespace wifi_debug {
int state() { return WifiStation::instance().debugState(); }
int lastError() { return WifiStation::instance().debugLastError(); }
std::uint32_t dmaFree() { return WifiStation::instance().debugDmaFree(); }
std::uint32_t dmaLargest() { return WifiStation::instance().debugDmaLargest(); }
}  // namespace wifi_debug

}  // namespace gea::targets::esp32::wifi
