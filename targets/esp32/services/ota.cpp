#include "services/ota.h"

#include "services/ota_image.h"

#include "display.h"  // display geometry for /screenshot, elastic RAM for staging
#include "services/device_control.h"

#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <strings.h>
#include <atomic>

#include "esp_app_format.h"
#include "esp_flash.h"
#include "esp_private/esp_clk.h"
#include "esp_memory_utils.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

namespace gea::framework::services {

namespace {

constexpr const char *kTag = "gea_esp32_ota";
constexpr int kServerPort = 8080;
// The handler streams through a 1 KiB stack buffer. A 3 KiB server task corrupted
// its FreeRTOS frame as esp_ota_begin entered the flash path on real hardware,
// resetting the connection after the client's first 128 KiB socket window. Four
// KiB is the smallest allocation that leaves room for that call chain beside
// the USB device-control task and both radio stacks.
//
// 8 KiB, not 4, since /screenshot landed here: capturing a frame replays the
// retained UI tree (renderRetainedSnapshotRgb565), whose stack cost scales with
// tree depth. The serial device-control task sizes itself at 8 KiB for exactly
// that call, and this task now makes the same one. The extra 4 KiB of internal
// DRAM is affordable only because the IPC task stacks gave back ~24 KiB.
constexpr int kServerStackSize = 8192;

// ESP-IDF only arms flash auto-suspend for an explicit allow-list of JEDEC ids
// (spi_flash_chip_gd.c / _winbond.c / _generic.c). Enabling the feature on a
// part that is NOT on that list is FATAL at boot: esp_flash_suspend_cmd_init
// returns ESP_ERR_NOT_SUPPORTED and esp_flash_init_default_chip propagates it.
// Report the id so the config can be decided from the hardware, not a guess.
constexpr std::uint32_t kSuspendQualifiedIds[] = {
	0xC84016, 0xC84017, 0xC84018, 0xC84319,  // GigaDevice (spi_flash_chip_gd.c)
};

bool flashSuspendQualified(std::uint32_t chipId)
{
	for (const std::uint32_t id : kSuspendQualifiedIds)
		if (id == chipId) return true;
	return false;
}

// The image session and the reboot timer moved to ota_image.cpp: staging an
// image is transport-agnostic and a BLE-only build has to link it without this
// HTTP server. The start diagnostics moved with them so ble_hid.cpp can report
// them in a build where no server exists; this file is their only writer.
using gea::targets::esp32::ota_debug::httpStartError;
using gea::targets::esp32::ota_debug::httpStarted;

class OtaHttpServer {
public:
	static OtaHttpServer &instance()
	{
		static OtaHttpServer server;
		return server;
	}

	bool start()
	{
		if (server_) return true;

		httpd_config_t config = HTTPD_DEFAULT_CONFIG();
		config.server_port = kServerPort;
		config.stack_size = kServerStackSize;
		// IDF's default cap is 8 and this server already registers 8 routes;
		// httpd_register_uri_handler fails silently past the cap, so every
		// route added later would 404 with nothing in the log to say why.
		config.max_uri_handlers = 16;

		const esp_err_t startError = httpd_start(&server_, &config);
		httpStartError.store(static_cast<int>(startError), std::memory_order_relaxed);
		if (startError != ESP_OK) {
			ESP_LOGE(kTag, "Failed to start OTA HTTP server: %s", esp_err_to_name(startError));
			server_ = nullptr;
			return false;
		}
		httpStarted.store(true, std::memory_order_relaxed);

		registerHandlers();
		ESP_LOGI(kTag, "OTA server listening on port %d", config.server_port);
		return true;
	}

private:
	OtaHttpServer() = default;

	static bool queryBool(const char *query, const char *key, bool fallback)
	{
		char value[12];
		if (!query || httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return fallback;
		return std::strcmp(value, "1") == 0
			|| strcasecmp(value, "true") == 0
			|| strcasecmp(value, "yes") == 0
			|| strcasecmp(value, "on") == 0;
	}

	static const esp_partition_t *findSlot(const char *slot)
	{
		if (!slot || slot[0] == '\0') return nullptr;
		return esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, slot);
	}

	static esp_err_t postHandler(httpd_req_t *req)
	{
		return instance().handlePost(req);
	}

	static esp_err_t statusHandler(httpd_req_t *req)
	{
		return instance().handleStatus(req);
	}

	static esp_err_t eraseHandler(httpd_req_t *req)
	{
		return instance().handleErase(req);
	}

	esp_err_t handlePost(httpd_req_t *req)
	{
		char query[128] = {};
		char slot[17] = {};
		const bool hasQuery = httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK;
		const bool hasSlot = hasQuery && httpd_query_key_value(query, "slot", slot, sizeof(slot)) == ESP_OK;
		const bool setBoot = queryBool(hasQuery ? query : nullptr, "boot", !hasSlot);
		const bool rebootAfter = queryBool(hasQuery ? query : nullptr, "reboot", !hasSlot);

		const esp_partition_t *updatePartition = hasSlot ? findSlot(slot) : esp_ota_get_next_update_partition(nullptr);
		if (!updatePartition) {
			httpd_resp_send_err(req, hasSlot ? HTTPD_404_NOT_FOUND : HTTPD_500_INTERNAL_SERVER_ERROR, hasSlot ? "OTA slot not found" : "No OTA partition found");
			return ESP_FAIL;
		}

		if (req->content_len > updatePartition->size) {
			httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE, "Image does not fit OTA slot");
			return ESP_FAIL;
		}

		const esp_partition_t *running = esp_ota_get_running_partition();
		if (running && running->address == updatePartition->address) {
			httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Cannot write the running partition over OTA");
			return ESP_FAIL;
		}

		ESP_LOGI(kTag, "OTA update started for %s (%d bytes)", updatePartition->label, req->content_len);

		// Erase the whole destination range ONCE, up front, instead of letting
		// esp_ota_write erase lazily under OTA_WITH_SEQUENTIAL_WRITES. Two things
		// were wrong with the lazy mode. It erases a 4 KiB SECTOR at a time (~45 ms
		// each on this part) — 938 of them for a 3.8 MB image, ~42 s of pure erase,
		// where esp_partition_erase_range over the same range issues 64 KiB BLOCK
		// erases (~3x less time per byte). And every one of those erases happened
		// INSIDE the receive loop with the flash cache disabled, so the socket was
		// not drained and lwIP's small receive window emptied and refilled around
		// each stall. Passing the real content length moves the erase out of the
		// transfer entirely; the client simply blocks on a full window while it runs.
		const std::size_t eraseLength = req->content_len > 0
			? static_cast<std::size_t>(req->content_len)
			: static_cast<std::size_t>(OTA_WITH_SEQUENTIAL_WRITES);
		const int64_t eraseStartUs = esp_timer_get_time();
		esp_ota_handle_t otaHandle;
		esp_err_t err = esp_ota_begin(updatePartition, eraseLength, &otaHandle);
		const int64_t eraseUs = esp_timer_get_time() - eraseStartUs;
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
			return ESP_FAIL;
		}
		ESP_LOGI(kTag, "OTA erase: %d bytes in %lld ms", req->content_len, eraseUs / 1000);

		const int64_t transferStartUs = esp_timer_get_time();
		const int total = receiveImage(req, otaHandle);
		if (total < 0) return ESP_FAIL;
		const int64_t transferUs = esp_timer_get_time() - transferStartUs;

		err = esp_ota_end(otaHandle);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "esp_ota_end failed: %s", esp_err_to_name(err));
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA validation failed");
			return ESP_FAIL;
		}

		if (setBoot) {
			err = esp_ota_set_boot_partition(updatePartition);
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
				return ESP_FAIL;
			}
		}

		// Report the phases separately: erase, socket receive, and flash write are
		// three different bottlenecks and a single aggregate rate hides which one
		// is binding.
		const int64_t totalUs = eraseUs + transferUs;
		ESP_LOGI(kTag,
			"OTA complete for %s! %d bytes in %lld ms (%lu B/s) "
			"[erase %lld ms, recv %lld ms, flash %lld ms, payload %lu B/s]",
			updatePartition->label,
			total,
			totalUs / 1000,
			static_cast<unsigned long>(totalUs > 0 ? (static_cast<int64_t>(total) * 1000000 / totalUs) : 0),
			eraseUs / 1000,
			recvUs_ / 1000,
			writeUs_ / 1000,
			static_cast<unsigned long>(transferUs > 0 ? (static_cast<int64_t>(total) * 1000000 / transferUs) : 0));
		httpd_resp_sendstr(req, rebootAfter ? "OTA OK, rebooting...\n" : "OTA OK, staged.\n");

		if (rebootAfter) {
			vTaskDelay(pdMS_TO_TICKS(500));
			esp_restart();
		}

		return ESP_OK;
	}

	// Staging for the receive loop. It MUST live in internal DRAM: the main flash
	// is hosted on SPI1, for which spi_flash_hal_supports_direct_write() is false,
	// so esp_flash_write only takes its direct path when esp_ptr_in_dram(buffer)
	// holds. A PSRAM staging buffer would instead be copied into flash through a
	// 32-byte stack bounce buffer, one bus acquisition and cache-disable window per
	// 32 bytes. PSRAM is plentiful on this board and completely unusable here.
	//
	// Receiving and writing run in sequence on purpose. A double-buffered writer
	// task was built and measured, and it bought nothing (30.6 s against 28.9 s for
	// this loop on the same image): esp_ota_write disables the flash cache, which
	// stops every flash-resident instruction on both cores — including lwIP and the
	// WiFi driver — so the socket cannot be drained while a write is in flight no
	// matter which task owns it. Overlap here needs the receive path in IRAM, not
	// another task.
	// The OTA staging buffer competes for DMA-capable internal DRAM with the
	// display's elastic flush staging -- and, since the diagnostics server landed,
	// with its socket too. What matters is not the buffer's SIZE but its REGION:
	// with no DMA-capable block free, the request falls back to RTC fast memory,
	// which is not DRAM, and esp_flash_write then copies the whole image through a
	// 32-byte bounce buffer. Measured on this board: flash phase 8.0 s from DRAM
	// vs 20.6 s from RTC fast, for the same 3.9 MB image. (Chunk size itself is
	// nearly free -- 4 KiB and 32 KiB writes clock the same 495-507 KB/s.)
	//
	// Borrow the RAM from the display for the duration of the transfer, exactly
	// as the WiFi bring-up does. The screen is static during an OTA, so a smaller
	// flush pipeline costs nothing visible, and the reservation is released on
	// every exit path.
	struct DisplayRamLoan {
		explicit DisplayRamLoan(std::size_t bytes)
		{
			if (bytes == 0) return;
			gea::platform::display::Display::reserveInternal(bytes);
			// The resize is applied by the frame task, not here on the HTTP task,
			// so give it a few frames to land before we try to allocate.
			vTaskDelay(pdMS_TO_TICKS(100));
			active_ = true;
		}
		~DisplayRamLoan()
		{
			if (active_) gea::platform::display::Display::reserveInternal(0);
		}
		DisplayRamLoan(const DisplayRamLoan &) = delete;
		DisplayRamLoan &operator=(const DisplayRamLoan &) = delete;

	private:
		bool active_ = false;
	};

	struct ReceiveStaging {
		std::uint8_t *data = nullptr;
		std::size_t size = 0;

		// MALLOC_CAP_DMA, not a bare MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT.
		// "Internal and 8-bit accessible" also matches RTC FAST memory, which the
		// heap hands out once DRAM runs short -- and RTC fast is not DRAM, so
		// esp_flash_write refuses its direct path and copies the whole image
		// through a 32-byte stack bounce buffer. Measured on this board: 517 KB/s
		// from DRAM vs 212 KB/s from RTC fast, i.e. the flash phase of a 3.9 MB
		// OTA went 7.6 s -> 20 s with no code change, purely because the buffer
		// landed in the wrong internal heap. A SMALLER DRAM buffer beats a larger
		// RTC one: the same measurement showed 4 KiB and 32 KiB writes running at
		// identical speed, so chunk size is nearly free while the source region is
		// worth 2.4x.
		explicit ReceiveStaging(std::size_t preferred)
		{
			for (std::size_t candidate = preferred; candidate >= 1024; candidate /= 2) {
				data = static_cast<std::uint8_t *>(
					heap_caps_malloc(candidate, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
				if (data) {
					size = candidate;
					return;
				}
			}
		}
		~ReceiveStaging() { heap_caps_free(data); }
		ReceiveStaging(const ReceiveStaging &) = delete;
		ReceiveStaging &operator=(const ReceiveStaging &) = delete;
	};

	int receiveImage(httpd_req_t *req, esp_ota_handle_t otaHandle)
	{
		char stackBuffer[1024];
		const DisplayRamLoan ramLoan(16 * 1024);
		ReceiveStaging staging(8192);
		char *const buffer = staging.data ? reinterpret_cast<char *>(staging.data) : stackBuffer;
		const std::size_t bufferSize = staging.data ? staging.size : sizeof(stackBuffer);
		// A staging buffer that shrank -- or worse, landed outside DRAM -- because
		// another subsystem took the DMA-capable heap first shows up as a slower
		// OTA, not as an allocation failure. Report the size actually obtained
		// alongside the DMA heap it came from, so the two are comparable after the
		// fact without another build.
		ESP_LOGI(kTag, "OTA staging buffer: %u bytes (internal-dma free=%u largest=%u)",
			static_cast<unsigned>(bufferSize),
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)),
			static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)));

		int received = 0;
		int total = 0;
		int loggedAt = 0;
		recvUs_ = 0;
		writeUs_ = 0;

		while (total < req->content_len) {
			const std::size_t remaining = static_cast<std::size_t>(req->content_len - total);
			const int64_t recvStartUs = esp_timer_get_time();
			received = httpd_req_recv(req, buffer, remaining < bufferSize ? remaining : bufferSize);
			recvUs_ += esp_timer_get_time() - recvStartUs;
			if (received == HTTPD_SOCK_ERR_TIMEOUT) {
				// A busy radio can temporarily drain lwIP's receive queue while the
				// peer is still connected. ESP-IDF documents this as retryable; the
				// old handler aborted an otherwise healthy upload on the first stall.
				continue;
			}
			if (received <= 0) break;
			const int64_t writeStartUs = esp_timer_get_time();
			const esp_err_t err = esp_ota_write(otaHandle, buffer, received);
			writeUs_ += esp_timer_get_time() - writeStartUs;
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
				esp_ota_abort(otaHandle);
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
				return -1;
			}
			total += received;
			if (total - loggedAt >= (512 * 1024)) {
				loggedAt = total;
				ESP_LOGI(kTag, "OTA progress: %d of %d bytes", total, req->content_len);
			}
		}

		if (total != req->content_len) {
			ESP_LOGE(kTag, "OTA receive error after %d of %d bytes", total, req->content_len);
			esp_ota_abort(otaHandle);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
			return -1;
		}

		return total;
	}

	int64_t recvUs_ = 0;
	int64_t writeUs_ = 0;

	esp_err_t handleStatus(httpd_req_t *req)
	{
		httpd_resp_set_type(req, "application/json");

		const esp_partition_t *running = esp_ota_get_running_partition();
		esp_app_desc_t desc;
		char chunk[256];

		httpd_resp_sendstr_chunk(req, "{\"running\":");
		if (running) {
			const char *version = "";
			if (esp_ota_get_partition_description(running, &desc) == ESP_OK) version = desc.version;
			snprintf(chunk, sizeof(chunk), "{\"label\":\"%s\",\"offset\":%lu,\"size\":%lu,\"version\":\"%s\"}",
				running->label, (unsigned long)running->address, (unsigned long)running->size, version);
			httpd_resp_sendstr_chunk(req, chunk);
		} else {
			httpd_resp_sendstr_chunk(req, "null");
		}

		httpd_resp_sendstr_chunk(req, ",\"partitions\":[");
		bool first = true;
		esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
		while (it) {
			const esp_partition_t *partition = esp_partition_get(it);
			if (partition) {
				const char *version = "";
				if (esp_ota_get_partition_description(partition, &desc) == ESP_OK) version = desc.version;
				snprintf(chunk, sizeof(chunk), "%s{\"label\":\"%s\",\"offset\":%lu,\"size\":%lu,\"version\":\"%s\"}",
					first ? "" : ",", partition->label, (unsigned long)partition->address, (unsigned long)partition->size, version);
				httpd_resp_sendstr_chunk(req, chunk);
				first = false;
			}
			it = esp_partition_next(it);
		}
		if (it) esp_partition_iterator_release(it);
		httpd_resp_sendstr_chunk(req, "]");

		// Flash identity. Decides whether CONFIG_SPI_FLASH_AUTO_SUSPEND can be
		// turned on for this board at all -- see kSuspendQualifiedIds above.
		std::uint32_t chipId = 0;
		if (esp_flash_read_id(nullptr, &chipId) == ESP_OK) {
			snprintf(chunk, sizeof(chunk),
				",\"flash\":{\"id\":\"0x%06lx\",\"suspendQualified\":%s}",
				(unsigned long)chipId, flashSuspendQualified(chipId) ? "true" : "false");
			httpd_resp_sendstr_chunk(req, chunk);
		}

		// Clock and internal-heap state. Both are inputs to flash throughput: the
		// SPI flash clock is derived from the CPU/APB clock, and a bench or OTA
		// staging buffer that cannot find contiguous internal DRAM silently falls
		// back to a smaller chunk. Reporting them turns "the flash got slower"
		// from a guess into a comparison.
		snprintf(chunk, sizeof(chunk),
			",\"clocks\":{\"cpuMhz\":%d},\"heap\":{\"internalFree\":%u,\"internalLargest\":%u,\"dmaLargest\":%u}",
			esp_clk_cpu_freq() / 1000000,
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
			static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
			static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)));
		httpd_resp_sendstr_chunk(req, chunk);
		httpd_resp_sendstr_chunk(req, "}");
		httpd_resp_sendstr_chunk(req, nullptr);
		return ESP_OK;
	}

	esp_err_t handleErase(httpd_req_t *req)
	{
		char query[64] = {};
		char slot[17] = {};
		if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK
			|| httpd_query_key_value(query, "slot", slot, sizeof(slot)) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing slot");
			return ESP_FAIL;
		}

		const esp_partition_t *partition = findSlot(slot);
		if (!partition) {
			httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "OTA slot not found");
			return ESP_FAIL;
		}

		const esp_partition_t *running = esp_ota_get_running_partition();
		if (running && running->address == partition->address) {
			httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Cannot erase the running partition");
			return ESP_FAIL;
		}

		ESP_LOGI(kTag, "Erasing OTA slot %s (%lu bytes)", partition->label, (unsigned long)partition->size);
		const esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Failed to erase %s: %s", partition->label, esp_err_to_name(err));
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erase failed");
			return ESP_FAIL;
		}

		httpd_resp_sendstr(req, "OTA slot erased.\n");
		return ESP_OK;
	}

	// Raw flash erase/program throughput on the INACTIVE OTA slot, with no radio
	// and no TCP in the path. This exists to answer one question with a
	// measurement instead of arithmetic: is the OTA "flash" phase sitting on the
	// SPI NOR floor, or on per-call overhead in esp_ota_write? It reports two
	// program rates -- one at the 8 KiB chunk the OTA receive loop uses, one at
	// 64 KiB. If they agree, the part itself is the limit.
	esp_err_t handleFlashBench(httpd_req_t *req)
	{
		const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
		if (!partition) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No spare OTA partition");
			return ESP_FAIL;
		}
		const esp_partition_t *running = esp_ota_get_running_partition();
		if (running && running->address == partition->address) {
			httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Refusing to benchmark the running partition");
			return ESP_FAIL;
		}

		char query[64] = {};
		char bytesValue[16] = {};
		std::size_t length = 1024 * 1024;
		if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
			&& httpd_query_key_value(query, "bytes", bytesValue, sizeof(bytesValue)) == ESP_OK) {
			const long requested = strtol(bytesValue, nullptr, 10);
			if (requested > 0) length = static_cast<std::size_t>(requested);
		}
		length &= ~static_cast<std::size_t>(0xFFF);  // whole 4 KiB sectors
		if (length == 0 || length > partition->size) length = partition->size;

		// Internal DRAM is scarce with both radios up, so take the largest
		// staging buffer that fits and report which size was used. MALLOC_CAP_DMA
		// for the same reason ReceiveStaging uses it: an RTC-fast buffer measures
		// the bounce path, not the flash. The address is reported so a slow result
		// can be attributed to the region rather than guessed at.
		// Borrow the display's elastic staging exactly as the real transfer does,
		// so the bench measures the conditions an OTA actually runs under rather
		// than the idle ones. Without it there is no 4 KiB DMA-capable block free
		// on this board at all and the bench simply fails.
		const DisplayRamLoan ramLoan(16 * 1024);
		std::size_t benchBuffer = 0;
		std::uint8_t *buffer = nullptr;
		for (std::size_t candidate = 32768; candidate >= 1024; candidate /= 2) {
			buffer = static_cast<std::uint8_t *>(
				heap_caps_malloc(candidate, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
			if (buffer) { benchBuffer = candidate; break; }
		}
		if (!buffer) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No internal DRAM for the bench buffer");
			return ESP_FAIL;
		}
		std::memset(buffer, 0xA5, benchBuffer);

		const int64_t eraseStartUs = esp_timer_get_time();
		esp_err_t err = esp_partition_erase_range(partition, 0, length);
		const int64_t eraseUs = esp_timer_get_time() - eraseStartUs;

		int64_t write8Us = 0;
		int64_t write64Us = 0;
		const std::size_t half = (length / 2) & ~static_cast<std::size_t>(0xFFF);
		if (err == ESP_OK) {
			const int64_t start = esp_timer_get_time();
			const std::size_t smallChunk = benchBuffer < 4096 ? benchBuffer : 4096;
			for (std::size_t offset = 0; offset < half && err == ESP_OK; offset += smallChunk)
				err = esp_partition_write(partition, offset, buffer, smallChunk);
			write8Us = esp_timer_get_time() - start;
		}
		if (err == ESP_OK) {
			const int64_t start = esp_timer_get_time();
			for (std::size_t offset = half; offset < length && err == ESP_OK; offset += benchBuffer)
				err = esp_partition_write(partition, offset, buffer, benchBuffer);
			write64Us = esp_timer_get_time() - start;
		}
		// Record where the buffer lived before releasing it: "internal" is not one
		// region, and only a DRAM source gets esp_flash_write's direct path.
		const std::uintptr_t bufferAddress = reinterpret_cast<std::uintptr_t>(buffer);
		const bool bufferInDram = esp_ptr_in_dram(buffer);
		heap_caps_free(buffer);

		if (err != ESP_OK) {
			ESP_LOGE(kTag, "flash bench failed: %s", esp_err_to_name(err));
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash bench failed");
			return ESP_FAIL;
		}

		// The slot now holds 0xA5, not an image. Say so: the caller must reflash
		// it before that slot can be booted.
		ESP_LOGW(kTag, "flash bench overwrote %s -- reflash before booting it", partition->label);

		const auto rate = [](std::size_t bytes, int64_t us) -> unsigned long {
			return us > 0 ? static_cast<unsigned long>((static_cast<int64_t>(bytes) * 1000000) / us / 1024) : 0;
		};
		char body[512];
		snprintf(body, sizeof(body),
			"{\"slot\":\"%s\",\"bytes\":%lu,"
			"\"eraseMs\":%lld,\"eraseKBs\":%lu,"
			"\"chunkBytes\":%lu,\"chunkAddr\":\"0x%08lx\",\"chunkInDram\":%s,"
			"\"program4kMs\":%lld,\"program4kKBs\":%lu,"
			"\"programBigMs\":%lld,\"programBigKBs\":%lu,"
			"\"combinedKBs\":%lu}\n",
			partition->label, (unsigned long)length,
			eraseUs / 1000, rate(length, eraseUs),
			(unsigned long)benchBuffer, (unsigned long)bufferAddress, bufferInDram ? "true" : "false",
			write8Us / 1000, rate(half, write8Us),
			write64Us / 1000, rate(length - half, write64Us),
			rate(length, eraseUs + write8Us + write64Us));
		httpd_resp_set_type(req, "application/json");
		httpd_resp_sendstr(req, body);
		return ESP_OK;
	}

	static esp_err_t flashBenchHandler(httpd_req_t *req)
	{
		return instance().handleFlashBench(req);
	}

	// Capture the live frame and return it as raw RGB565, the same
	// "rgb565-raw-v1" payload the serial SCREENSHOTBIN command emits, so the host
	// decoder is shared. Screenshots used to require the USB cable; the panel is
	// 410x502x2 = ~402 KiB, which is ~26 s over BLE but well under a second here.
#if GEA_PIXEL_STORAGE_PACKED
	// A packed (GRAY4/GRAY2) board's real framebuffer is a quarter/eighth the
	// size an RGB565 expansion of the same pixels would need. Capture that (what
	// the device can actually spare -- see the SCREENSHOTBIN fix this mirrors in
	// device_control.cpp) and expand to RGB565 one row at a time while chunking
	// to the socket. The wire payload stays rgb565-raw-v1 (shared with
	// SCREENSHOTBIN), so the host decoder needs no changes.
	esp_err_t handleScreenshot(httpd_req_t *req)
	{
		const int pixelCount = gea::platform::display::kWidth * gea::platform::display::kHeight;
		const std::size_t byteCount = static_cast<std::size_t>(
			gea::framework::graphics::pixel::packed::rowBytes(pixelCount));
		auto *snapshot = static_cast<std::uint8_t *>(
			heap_caps_malloc(byteCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!snapshot) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No PSRAM for the snapshot");
			return ESP_FAIL;
		}

		int width = 0;
		int height = 0;
		char appId[96] = {};
		const bool captured = gea::platform::esp32::services::captureScreenshotPacked(
			snapshot, static_cast<int>(byteCount), &width, &height, appId, sizeof(appId));
		if (!captured) {
			heap_caps_free(snapshot);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Display unavailable");
			return ESP_FAIL;
		}

		char widthHeader[16];
		char heightHeader[16];
		httpd_resp_set_type(req, "application/octet-stream");
		snprintf(widthHeader, sizeof(widthHeader), "%d", width);
		httpd_resp_set_hdr(req, "X-Gea-Width", widthHeader);
		snprintf(heightHeader, sizeof(heightHeader), "%d", height);
		httpd_resp_set_hdr(req, "X-Gea-Height", heightHeader);
		httpd_resp_set_hdr(req, "X-Gea-Encoding", "rgb565-raw-v1");
		httpd_resp_set_hdr(req, "X-Gea-App", appId);

		using gea::framework::graphics::pixel::packed;
		using gea::framework::graphics::pixel::fromNative;
		auto *rowRgb = static_cast<std::uint16_t *>(
			heap_caps_malloc(static_cast<std::size_t>(width) * sizeof(std::uint16_t), MALLOC_CAP_8BIT));
		if (!rowRgb) {
			heap_caps_free(snapshot);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory for the row buffer");
			return ESP_FAIL;
		}
		esp_err_t err = ESP_OK;
		for (int row = 0; row < height && err == ESP_OK; ++row) {
			for (int x = 0; x < width; ++x)
				rowRgb[x] = fromNative(packed::get(snapshot, row * width + x));
			err = httpd_resp_send_chunk(req, reinterpret_cast<const char *>(rowRgb),
			                            static_cast<std::size_t>(width) * sizeof(std::uint16_t));
		}
		if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
		heap_caps_free(rowRgb);
		heap_caps_free(snapshot);
		return err;
	}
#else
	esp_err_t handleScreenshot(httpd_req_t *req)
	{
		const int pixelCount = gea::platform::display::kWidth * gea::platform::display::kHeight;
		auto *snapshot = static_cast<std::uint16_t *>(
			heap_caps_malloc(static_cast<std::size_t>(pixelCount) * sizeof(std::uint16_t),
			                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!snapshot) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No PSRAM for the snapshot");
			return ESP_FAIL;
		}

		int width = 0;
		int height = 0;
		char appId[96] = {};
		const bool captured = gea::platform::esp32::services::captureScreenshotRgb565(
			snapshot, pixelCount, &width, &height, appId, sizeof(appId));
		if (!captured) {
			heap_caps_free(snapshot);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Display unavailable");
			return ESP_FAIL;
		}

		// httpd_resp_set_hdr stores the POINTER, not a copy, and the value must
		// stay alive until the response is sent -- so every header needs its own
		// buffer. Sharing one made both dimensions report the last value written.
		char widthHeader[16];
		char heightHeader[16];
		httpd_resp_set_type(req, "application/octet-stream");
		snprintf(widthHeader, sizeof(widthHeader), "%d", width);
		httpd_resp_set_hdr(req, "X-Gea-Width", widthHeader);
		snprintf(heightHeader, sizeof(heightHeader), "%d", height);
		httpd_resp_set_hdr(req, "X-Gea-Height", heightHeader);
		httpd_resp_set_hdr(req, "X-Gea-Encoding", "rgb565-raw-v1");
		httpd_resp_set_hdr(req, "X-Gea-App", appId);

		// Send in bounded chunks: one 402 KiB write would pin the whole frame in
		// the socket path at once.
		const auto *bytes = reinterpret_cast<const char *>(snapshot);
		const std::size_t total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height)
		                        * sizeof(std::uint16_t);
		esp_err_t err = ESP_OK;
		for (std::size_t sent = 0; sent < total && err == ESP_OK; sent += 8192) {
			const std::size_t span = (total - sent) < 8192 ? (total - sent) : 8192;
			err = httpd_resp_send_chunk(req, bytes + sent, span);
		}
		if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
		heap_caps_free(snapshot);
		return err;
	}
#endif  // GEA_PIXEL_STORAGE_PACKED

	static esp_err_t screenshotHandler(httpd_req_t *req)
	{
		return instance().handleScreenshot(req);
	}

	// POST /display/hbm?on=1|0 -- toggle the panel's high-brightness mode over
	// WiFi. Lives on this server because it is the one HTTP surface a cable-free
	// board has; it is device control, not OTA.
	esp_err_t handleDisplayHbm(httpd_req_t *req)
	{
		char query[32] = {};
		char onValue[8] = {};
		bool on = true;
		if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
			&& httpd_query_key_value(query, "on", onValue, sizeof(onValue)) == ESP_OK) {
			on = !(onValue[0] == '0' || onValue[0] == 'f' || onValue[0] == 'n');
		}
		const bool ok = gea::platform::display::Display::setHighBrightnessMode(on);
		char body[96];
		snprintf(body, sizeof(body), "{\"ok\":%s,\"hbm\":%s}\n", ok ? "true" : "false",
			gea::platform::display::Display::highBrightnessMode() ? "true" : "false");
		httpd_resp_set_type(req, "application/json");
		if (!ok) httpd_resp_set_status(req, "501 Not Implemented");
		httpd_resp_sendstr(req, body);
		return ESP_OK;
	}
	static esp_err_t displayHbmHandler(httpd_req_t *req) { return instance().handleDisplayHbm(req); }

	// POST /display/brightness?value=0-100 and /display/vsync?on=1|0 -- the
	// remaining display knobs, so every one of them answers on WiFi as well as
	// over USB (GEADEV BRIGHTNESS/HBM/VSYNC). Without a value each reports.
	esp_err_t handleDisplayBrightness(httpd_req_t *req)
	{
		using gea::platform::display::Display;
		char query[32] = {};
		char value[8] = {};
		if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
			&& httpd_query_key_value(query, "value", value, sizeof(value)) == ESP_OK) {
			int percent = atoi(value);
			if (percent < 0) percent = 0;
			if (percent > 100) percent = 100;
			Display::setBrightness(percent);
		}
		char body[64];
		snprintf(body, sizeof(body), "{\"ok\":true,\"brightness\":%d}\n", Display::brightness());
		httpd_resp_set_type(req, "application/json");
		httpd_resp_sendstr(req, body);
		return ESP_OK;
	}
	static esp_err_t displayBrightnessHandler(httpd_req_t *req) { return instance().handleDisplayBrightness(req); }

	esp_err_t handleDisplayVsync(httpd_req_t *req)
	{
		using gea::platform::display::Display;
		char query[32] = {};
		char onValue[8] = {};
		if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK
			&& httpd_query_key_value(query, "on", onValue, sizeof(onValue)) == ESP_OK) {
			Display::setVSync(!(onValue[0] == '0' || onValue[0] == 'f' || onValue[0] == 'n'));
		}
		char body[64];
		snprintf(body, sizeof(body), "{\"ok\":true,\"vsync\":%s}\n", Display::vsyncEnabled() ? "true" : "false");
		httpd_resp_set_type(req, "application/json");
		httpd_resp_sendstr(req, body);
		return ESP_OK;
	}
	static esp_err_t displayVsyncHandler(httpd_req_t *req) { return instance().handleDisplayVsync(req); }

	void registerHandlers()
	{
		httpd_uri_t otaUri = {};
		otaUri.uri = "/ota";
		otaUri.method = HTTP_POST;
		otaUri.handler = &OtaHttpServer::postHandler;
		httpd_register_uri_handler(server_, &otaUri);

		httpd_uri_t statusUri = {};
		statusUri.uri = "/ota/status";
		statusUri.method = HTTP_GET;
		statusUri.handler = &OtaHttpServer::statusHandler;
		httpd_register_uri_handler(server_, &statusUri);

		httpd_uri_t screenshotUri = {};
		screenshotUri.uri = "/screenshot";
		screenshotUri.method = HTTP_GET;
		screenshotUri.handler = &OtaHttpServer::screenshotHandler;
		httpd_register_uri_handler(server_, &screenshotUri);

		httpd_uri_t hbmUri = {};
		hbmUri.uri = "/display/hbm";
		hbmUri.method = HTTP_POST;
		hbmUri.handler = &OtaHttpServer::displayHbmHandler;
		httpd_register_uri_handler(server_, &hbmUri);

		httpd_uri_t brightnessUri = {};
		brightnessUri.uri = "/display/brightness";
		brightnessUri.method = HTTP_POST;
		brightnessUri.handler = &OtaHttpServer::displayBrightnessHandler;
		httpd_register_uri_handler(server_, &brightnessUri);

		httpd_uri_t vsyncUri = {};
		vsyncUri.uri = "/display/vsync";
		vsyncUri.method = HTTP_POST;
		vsyncUri.handler = &OtaHttpServer::displayVsyncHandler;
		httpd_register_uri_handler(server_, &vsyncUri);

		httpd_uri_t benchUri = {};
		benchUri.uri = "/ota/flashbench";
		benchUri.method = HTTP_POST;
		benchUri.handler = &OtaHttpServer::flashBenchHandler;
		httpd_register_uri_handler(server_, &benchUri);

		httpd_uri_t eraseUri = {};
		eraseUri.uri = "/ota/erase";
		eraseUri.method = HTTP_POST;
		eraseUri.handler = &OtaHttpServer::eraseHandler;
		httpd_register_uri_handler(server_, &eraseUri);
	}

	httpd_handle_t server_ = nullptr;
};

}  // namespace

bool OtaServer::start()
{
	return OtaHttpServer::instance().start();
}

}  // namespace gea::framework::services
