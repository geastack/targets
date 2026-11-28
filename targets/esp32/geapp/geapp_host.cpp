// SPDX-License-Identifier: GPL-3.0-only
#include "geapp_host.h"

#ifndef GEA_GEAPP_TRUST_ROOTS
#define GEA_GEAPP_TRUST_ROOTS ""
#endif

#include "geapp_exports.h"
#include "geapp_loader.h"
#include "platform/file_cache.h"
#include "display.h"
#ifndef GEA_EMBEDDED_WIFI_DISABLED
#include "wifi.h"
#endif
#include "program.h"
#include "services/frame_scheduler.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_private/panic_internal.h"
#if __XTENSA__
#include "xtensa_context.h"
#else
#include "riscv/rvruntime-frames.h"
#endif
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef GEA_GEAPP_ABI
#define GEA_GEAPP_ABI "unversioned"
#endif

// The board's side button returns a running program to the firmware's own.
#ifndef GEA_GEAPP_HOME_BUTTON_GPIO
#ifdef GEA_BOARD_LAUNCHER_BUTTON_GPIO
#define GEA_GEAPP_HOME_BUTTON_GPIO GEA_BOARD_LAUNCHER_BUTTON_GPIO
#else
#define GEA_GEAPP_HOME_BUTTON_GPIO -1
#endif
#endif

// The display backend's present-mode switch (targets/esp32/display.cpp); absent
// on a build without async present.
extern "C" void gea_display_set_async_present(bool on) __attribute__((weak));
extern "C" void gea_display_release_async_present() __attribute__((weak));

namespace gea::platform::geapp {

namespace program = gea::framework::program;
namespace graphics = gea::framework::graphics;

namespace {

constexpr char kTag[] = "geapp";

// The program entry points the runtime calls (program.h), by mangled name.
constexpr char kTopLevel[] = "_Z15__gea_top_levelv";
constexpr char kPrelude[] = "_Z31gea_plugin_cpp_register_preludev";
constexpr char kDrainMicrotasks[] = "_ZN3gea9framework3app9generated15drainMicrotasksEv";
constexpr char kReleaseGlobals[] = "gea_program_release_globals";
constexpr char kAssetLookup[] = "gea_embedded_asset_lookup";
constexpr char kAssetsPrepare[] = "gea_embedded_assets_prepare";
constexpr char kFontsEnsureLinked[] = "_ZN3gea9framework8graphics9generated12ensureLinkedEv";
constexpr char kLookupFont[] = "_ZN3gea9framework8graphics9generated10lookupFontEi";
constexpr char kLookupFontForFamily[] = "_ZN3gea9framework8graphics9generated19lookupFontForFamilyEii";
constexpr char kLookupFontFamily[] = "_ZN3gea9framework8graphics9generated16lookupFontFamilyEPKc";
constexpr char kLookupFontFamilyName[] = "_ZN3gea9framework8graphics9generated20lookupFontFamilyNameEi";
constexpr char kLookupRuntimeTtf[] = "_ZN3gea9framework8graphics9generated29lookupRuntimeTtfFontForFamilyEiPm";
constexpr char kRuntimeTtfCodepoints[] = "_ZN3gea9framework8graphics9generated28runtimeTtfDeclaredCodepointsEPi";

bool exists(const std::string &path)
{
	struct stat info{};
	return ::stat(path.c_str(), &info) == 0;
}

// Where a program's image is: a file, or a byte range of a .geapp package.
struct Image {
	std::string path;
	std::uint32_t offset = 0;
	std::uint32_t length = UINT32_MAX;
};

template <typename Function>
Function entry(Module *module, const char *name)
{
	return reinterpret_cast<Function>(symbol(module, name));
}

std::uintptr_t g_loadedBase = 0;

// The app whose code is running, kept across a reset: a panic or a watchdog
// inside an app restarts the chip, and Home names the app when it comes back
// instead of the user meeting a silent reboot. Set before the app's first
// constructor runs, cleared after its last byte is released, so a crash
// while it starts or exits is its own too.
//
// A panic also leaves its registers here (gea_geapp_panic_record, called from
// the framework's panic wrapper), with the program image's range, so the error
// can say where in the program it crashed: "pc program+0x1a2c".
struct Breadcrumb {
	std::uint32_t magic;
	char id[64];
	std::uint32_t base;  // the program image on the data bus, 0 before it is mapped
	std::uint32_t size;
	std::uint32_t codeOffset;
	std::uint32_t panicMagic;
	std::uint32_t pc;
	std::uint32_t ra;
	std::uint32_t cause;
	std::uint32_t address;
};
constexpr std::uint32_t kBreadcrumbMagic = 0x47454132;  // "GEA2"
constexpr std::uint32_t kPanicMagic = 0x50414e43;       // "PANC"
RTC_NOINIT_ATTR Breadcrumb g_breadcrumb;

void markRunning(const char *id)
{
	std::snprintf(g_breadcrumb.id, sizeof(g_breadcrumb.id), "%s", id ? id : "");
	g_breadcrumb.base = 0;
	g_breadcrumb.size = 0;
	g_breadcrumb.codeOffset = static_cast<std::uint32_t>(gea::platform::geapp::codeOffset());
	g_breadcrumb.panicMagic = 0;
	g_breadcrumb.magic = kBreadcrumbMagic;
}

void markImage(const gea::platform::geapp::Module *module)
{
	g_breadcrumb.base = static_cast<std::uint32_t>(gea::platform::geapp::base(module));
	g_breadcrumb.size = static_cast<std::uint32_t>(gea::platform::geapp::imageSize(module));
}

void clearRunning()
{
	g_breadcrumb.magic = 0;
	g_breadcrumb.panicMagic = 0;
	g_breadcrumb.id[0] = '\0';
}

// "program+0x1a2c" for an address in the program image (on either bus), else
// the raw address (firmware code or data).
std::string describeAddress(std::uint32_t address)
{
	char text[40];
	const Breadcrumb &b = g_breadcrumb;
	const auto inImage = [&](std::uint32_t a) { return b.base && a >= b.base && a - b.base < b.size; };
	if (inImage(address)) std::snprintf(text, sizeof(text), "program+0x%lx", static_cast<unsigned long>(address - b.base));
	else if (b.codeOffset && inImage(address - b.codeOffset))
		std::snprintf(text, sizeof(text), "program+0x%lx", static_cast<unsigned long>(address - b.codeOffset - b.base));
	else std::snprintf(text, sizeof(text), "0x%08lx", static_cast<unsigned long>(address));
	return text;
}

// At boot: if the last reset was a crash or a hang while an app ran, say so
// where Home shows errors.
void reportPreviousCrash()
{
	if (g_breadcrumb.magic != kBreadcrumbMagic) return;
	char id[sizeof(g_breadcrumb.id)];
	std::memcpy(id, g_breadcrumb.id, sizeof(id));
	id[sizeof(id) - 1] = '\0';
	const bool panicked = g_breadcrumb.panicMagic == kPanicMagic;
	clearRunning();
	const esp_reset_reason_t reason = esp_reset_reason();
	const char *what = nullptr;
	switch (reason) {
	case ESP_RST_PANIC: what = "crashed"; break;
	case ESP_RST_INT_WDT:
	case ESP_RST_TASK_WDT:
	case ESP_RST_WDT: what = "stopped responding"; break;
	default: return;  // a power cut or a deliberate restart: nobody's fault
	}
	if (!gea::geaos::isValidAppId(id)) return;
	std::string detail;
	if (reason == ESP_RST_PANIC && panicked) {
		char cause[48];
		std::snprintf(cause, sizeof(cause), ", cause %lu, address 0x%08lx", static_cast<unsigned long>(g_breadcrumb.cause),
		              static_cast<unsigned long>(g_breadcrumb.address));
		detail = " (pc " + describeAddress(g_breadcrumb.pc) + ", from " + describeAddress(g_breadcrumb.ra) + cause + ")";
	}
	ESP_LOGW(kTag, "%s %s%s; the device restarted into Home", id, what, detail.c_str());
	gea::geaos::Launcher::setLastError(std::string(id) + " " + what + " and was closed" + detail);
}

struct LoadedProgram {
	program::Hooks hooks;
	Module *module = nullptr;
};

class Host final : public program::Host {
public:
	const program::Hooks *load(const char *id, char *error, std::size_t errorSize) override
	{
		Image image;
		if (!prepare(id, image, error, errorSize)) return nullptr;
		Module *module = gea::platform::geapp::map(image.path.c_str(), image.offset, image.length, error, errorSize);
		if (!module) return nullptr;
		markRunning(id);
		markImage(module);
		gea::platform::geapp::construct(module);
		const program::Hooks *hooks = adopt(id, module, error, errorSize);
		if (!hooks) clearRunning();
		return hooks;
	}

	// The image is read and relocated on a task of its own (core 1) while the
	// launcher keeps animating its loading screen on the runtime's task; the
	// static constructors run in finishLoad, back on the runtime's task.
	bool beginLoad(const char *id, char *error, std::size_t errorSize) override
	{
		if (errorSize) error[0] = '\0';
		if (pending_.active) return false;
		if (!prepare(id, pending_.image, error, errorSize)) return false;
		std::snprintf(pending_.id, sizeof(pending_.id), "%s", id);
		pending_.module = nullptr;
		pending_.error[0] = '\0';
		pending_.done.store(false, std::memory_order_relaxed);
		pending_.active = true;
		// Above the runtime's priority: the display's render worker shares core 1
		// at that priority and busy-spins between raster bands, so while the
		// loading screen animates, a lower-priority reader (it was 5) got almost
		// no time and a launch took minutes. The reader mostly waits on the NAND,
		// so it costs the animation little.
		const UBaseType_t priority = uxTaskPriorityGet(nullptr) + 1;
		if (xTaskCreatePinnedToCore(mapTask, "geapp_map", kMapStackBytes, this, priority, nullptr, 1) != pdPASS) {
			pending_.active = false;
			return false;  // load() inline instead
		}
		return true;
	}

	bool loadDone() override { return !pending_.active || pending_.done.load(std::memory_order_acquire); }

	const program::Hooks *finishLoad(char *error, std::size_t errorSize) override
	{
		pending_.active = false;
		Module *module = pending_.module;
		pending_.module = nullptr;
		if (!module) {
			std::snprintf(error, errorSize, "%s", pending_.error);
			return nullptr;
		}
		markRunning(pending_.id);
		markImage(module);
		gea::platform::geapp::construct(module);
		const program::Hooks *hooks = adopt(pending_.id, module, error, errorSize);
		if (!hooks) clearRunning();
		return hooks;
	}

	void finalize(const program::Hooks *hooks) override
	{
		if (auto *loaded = programFor(hooks)) gea::platform::geapp::finalize(loaded->module);
	}

	void release(const program::Hooks *hooks) override
	{
		if (auto *loaded = programFor(hooks)) {
			gea::platform::geapp::release(loaded->module);
			delete loaded;
			g_loadedBase = 0;
			clearRunning();
		}
		ESP_LOGI(kTag, "after release: psram_free=%u internal_free=%u",
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
	}

	void configure(const char *id) override
	{
		// The firmware's own program (the launcher) repaints only what changed,
		// which needs the synchronous single-buffer present.
		const bool async = id && id[0] ? asyncPresent_ : false;
		if (gea_display_set_async_present) gea_display_set_async_present(async);
		// The previous app's present task, GDMA channel and second buffer go
		// with it: the launcher never presents asynchronously, and the next app
		// may need that internal RAM in one piece.
		if (!async && gea_display_release_async_present) gea_display_release_async_present();
	}

private:
	static constexpr std::uint32_t kMapStackBytes = 6144;  // ESP-IDF task stacks are bytes

	// Resolves `id` through the GeaOS registry, checks it was built for this
	// firmware, and finds its program image.
	bool prepare(const char *id, Image &image, char *error, std::size_t errorSize)
	{
		(void)gea::platform::storage::ensureMounted();
		std::optional<gea::geaos::InstalledApp> app = gea::geaos::Launcher::find(id);
		if (!app && gea::geaos::Launcher::rescan()) app = gea::geaos::Launcher::find(id);
		if (!app) {
			std::snprintf(error, errorSize, "%s is not installed", id);
			return false;
		}
		std::string reason;
		if (!gea::geaos::checkCompatible(app->manifest, firmwareAbi(), reason)) {
			std::snprintf(error, errorSize, "%s", reason.c_str());
			return false;
		}
		image = Image{};
		if (app->layout == gea::geaos::AppLayout::Package) {
			gea::geaos::PackageReader reader;
			if (!reader.open(gea::geaos::posixFs().open(app->location), reason)) {
				std::snprintf(error, errorSize, "%s: %s", id, reason.c_str());
				return false;
			}
			const gea::geaos::PackageEntry *entry = reader.find(gea::geaos::programEntryName(kTargetKey));
			if (!entry) {
				std::snprintf(error, errorSize, "%s has no program for %s", id, kTargetKey);
				return false;
			}
			image.path = app->location;
			image.offset = entry->offset;
			image.length = entry->size;
		} else {
			image.path = gea::geaos::folderProgramPath(gea::geaos::posixFs(), app->location, kTargetKey);
			if (image.path.empty()) {
				std::snprintf(error, errorSize, "%s has no program for %s", id, kTargetKey);
				return false;
			}
		}
		asyncPresent_ = app->manifest.present == gea::geaos::PresentMode::Async;
		ESP_LOGI(kTag, "launching %s from %s: psram_free=%u internal_free=%u", id, app->location.c_str(),
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
		return true;
	}

	// The program's entry points, once its constructors have run.
	const program::Hooks *adopt(const char *id, Module *module, char *error, std::size_t errorSize)
	{
		auto *loaded = new LoadedProgram();
		loaded->module = module;
		program::Hooks &hooks = loaded->hooks;
		hooks.topLevel = entry<void (*)()>(module, kTopLevel);
		hooks.registerPrelude = entry<void (*)()>(module, kPrelude);
		hooks.drainMicrotasks = entry<void (*)()>(module, kDrainMicrotasks);
		hooks.releaseGlobals = entry<void (*)()>(module, kReleaseGlobals);
		hooks.assetLookup = entry<bool (*)(const char *, const unsigned char **, unsigned long *)>(module, kAssetLookup);
		hooks.assetsPrepare = entry<bool (*)()>(module, kAssetsPrepare);
		hooks.fontsEnsureLinked = entry<void (*)()>(module, kFontsEnsureLinked);
		hooks.lookupFont = entry<const graphics::RasterizedFontData *(*)(int)>(module, kLookupFont);
		hooks.lookupFontForFamily = entry<const graphics::RasterizedFontData *(*)(int, int)>(module, kLookupFontForFamily);
		hooks.lookupFontFamily = entry<int (*)(const char *)>(module, kLookupFontFamily);
		hooks.lookupFontFamilyName = entry<const char *(*)(int)>(module, kLookupFontFamilyName);
		hooks.lookupRuntimeTtfFontForFamily = entry<const std::uint8_t *(*)(int, unsigned long *)>(module, kLookupRuntimeTtf);
		hooks.runtimeTtfDeclaredCodepoints = entry<const int *(*)(int *)>(module, kRuntimeTtfCodepoints);
		if (!hooks.topLevel || !hooks.drainMicrotasks) {
			std::snprintf(error, errorSize, "%s has no program entry (is it a gea .geapp?)", id);
			unload(module);
			delete loaded;
			return nullptr;
		}
		g_loadedBase = base(module);
		return &loaded->hooks;
	}

	static void mapTask(void *arg)
	{
		auto *self = static_cast<Host *>(arg);
		auto &pending = self->pending_;
		pending.module = gea::platform::geapp::map(pending.image.path.c_str(), pending.image.offset, pending.image.length,
		                                           pending.error, sizeof(pending.error));
		pending.done.store(true, std::memory_order_release);
		// Wake the runtime's loop in case nothing else is queued.
		gea::framework::events::Event wake{};
		wake.type = gea::framework::events::EventType::Timeout;
		(void)gea::framework::services::FrameScheduler::sendEvent(wake, 0);
		vTaskDelete(nullptr);
	}

	struct PendingLoad {
		bool active = false;
		std::atomic<bool> done{false};
		char id[64] = "";
		Image image;
		Module *module = nullptr;
		char error[160] = "";
	};
	PendingLoad pending_;

	static LoadedProgram *programFor(const program::Hooks *hooks)
	{
		// hooks is the first member of LoadedProgram.
		return hooks ? reinterpret_cast<LoadedProgram *>(const_cast<program::Hooks *>(hooks)) : nullptr;
	}

	bool asyncPresent_ = false;
};

Host g_host;
#if GEA_GEAPP_HOME_BUTTON_GPIO >= 0
TaskHandle_t g_buttonTask = nullptr;
#endif

// GeaOS's view of this firmware: launching an app asks the runtime to switch
// programs between frames (program::requestLaunch); a failure found while
// loading is reported through program::lastError().
class GeaosHost final : public gea::geaos::AppHost {
public:
	const char *targetKey() const override { return kTargetKey; }
	const char *abi() const override { return firmwareAbi(); }
	std::uint64_t maxProgramBytes() const override { return GEA_GEAPP_MAX_PROGRAM_BYTES; }
	const char *firmwareId() const override { return "home"; }
	const char *runningAppId() const override { return program::runningId(); }

	bool launch(const gea::geaos::InstalledApp &app, std::string &error) override
	{
		(void)error;
		program::requestLaunch(app.manifest.id.c_str());
		return true;
	}

	bool home(std::string &error) override
	{
		if (!program::loaded()) {
			error = "no app is running";
			return false;
		}
		program::requestLaunch("");
		return true;
	}

	// The whole program image -- code, data and the assets compiled into it
	// -- is in PSRAM by the time it runs, so the app keeps running without
	// its volume; only files it opens itself on that card fail, as any
	// removed file would. That makes every app here pinned, whatever its
	// manifest says.
	void volumeLost(const gea::geaos::InstalledApp &app) override
	{
		ESP_LOGW(kTag, "%s: its volume %s was removed; it keeps running from memory", app.manifest.id.c_str(), app.volumeId.c_str());
	}
	void volumeRestored(const gea::geaos::InstalledApp &app) override
	{
		ESP_LOGI(kTag, "%s: volume %s is back", app.manifest.id.c_str(), app.volumeId.c_str());
	}
};

GeaosHost g_geaosHost;

// What Settings can change on this board, and what it says about it.
class EspSystemServices final : public gea::geaos::SystemServices {
public:
	bool hasBrightness() const override { return true; }
	bool setBrightness(int percent) override
	{
		gea::platform::display::Display::setBrightness(percent);
		return true;
	}
	int brightness() const override { return static_cast<int>(gea::platform::display::Display::brightness()); }
#ifndef GEA_EMBEDDED_WIFI_DISABLED
	bool hasWifi() const override { return gea::framework::network::WifiAdapter::driver() != nullptr; }
	bool setWifiEnabled(bool on) override
	{
		if (!hasWifi()) return false;
		gea::framework::network::wifi().setEnabled(on);
		return true;
	}
	bool connectWifi(const std::string &ssid, const std::string &password) override
	{
		if (!hasWifi() || ssid.empty()) return false;
		gea::framework::network::wifi().configure(ssid, password);
		gea::framework::network::wifi().setEnabled(true);
		return true;
	}
	std::string wifiStatus() const override
	{
		if (!hasWifi()) return {};
		auto wifi = gea::framework::network::wifi();
		if (wifi.connected()) return "connected to " + wifi.ssid() + " (" + wifi.ip() + ")";
		return wifi.enabled() ? "connecting" : "off";
	}
#endif
	gea::geaos::About about() const override
	{
		gea::geaos::About about;
		about.firmware = "home";
		about.abi = firmwareAbi();
#ifdef GEA_EMBEDDED_BOARD_ID
		about.board = GEA_EMBEDDED_BOARD_ID;
#endif
		about.targetKey = kTargetKey;
		return about;
	}
};

EspSystemServices g_systemServices;

#if GEA_GEAPP_HOME_BUTTON_GPIO >= 0
void homeButtonTask(void *)
{
	const auto pin = static_cast<gpio_num_t>(GEA_GEAPP_HOME_BUTTON_GPIO);
	// A press is the pin held low for three consecutive samples, then released:
	// a single-sample glitch on the line (seen once ~11 s into a program, with
	// nobody near the button) must not tear the program down.
	int lowSamples = 0;
	bool pressed = false;
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(25));
		if (gpio_get_level(pin) == 0) {
			if (++lowSamples >= 3) pressed = true;
			continue;
		}
		lowSamples = 0;
		if (!pressed) continue;
		pressed = false;
		if (program::loaded()) {
			ESP_LOGI(kTag, "home button: returning to the firmware program");
			program::requestLaunch("");
		}
	}
}
#endif

}  // namespace

bool rescan()
{
	return pollVolumes();
}

std::string iconPath(const gea::geaos::InstalledApp &app)
{
	if (app.layout == gea::geaos::AppLayout::Folder) return app.iconPath;
	if (app.manifest.icon.empty()) return {};
	std::string volumeRoot;
	for (const auto &volume : gea::geaos::Launcher::volumes())
		if (volume.id == app.volumeId) volumeRoot = volume.root;
	if (volumeRoot.empty()) return {};
	const std::string dir = volumeRoot + "/apps/.icons";
	// Keyed by version: an updated package brings its new icon.
	const std::string path = dir + "/" + app.manifest.id + "-" + app.manifest.version.toString() + ".png";
	if (exists(path)) return path;
	gea::geaos::PackageReader reader;
	std::string data;
	std::string error;
	if (!reader.open(gea::geaos::posixFs().open(app.location), error) || !reader.read(app.manifest.icon, data, error)) {
		ESP_LOGW(kTag, "icon of %s: %s", app.manifest.id.c_str(), error.c_str());
		return {};
	}
	::mkdir(dir.c_str(), 0775);
	FILE *file = std::fopen(path.c_str(), "wb");
	if (!file) return {};
	const bool ok = std::fwrite(data.data(), 1, data.size(), file) == data.size();
	std::fclose(file);
	if (!ok) {
		::remove(path.c_str());
		return {};
	}
	return path;
}

unsigned long loadedProgramBase() { return g_loadedBase; }

const char *firmwareAbi()
{
	return GEA_GEAPP_ABI;
}

void installHost()
{
	if (program::host() == &g_host) return;
	program::setHost(&g_host);
	gea::geaos::Launcher::setHost(&g_geaosHost);
	if (gea_display_set_async_present) gea_display_set_async_present(false);
	// The C library makes its boot-time locks on the first clock read and
	// keeps them for good. Made here, at boot, they sit at the start of the
	// heap; made by an app (Sonora's first clock read) they landed in the
	// middle of the largest internal block and cut it from 38 KB to 24 KB for
	// every app after it.
	timeval now{};
	gettimeofday(&now, nullptr);
	timespec realtime{};
	clock_gettime(CLOCK_REALTIME, &realtime);
	ESP_LOGI(kTag, "program host ready: %u firmware exports, abi %s", static_cast<unsigned>(exportCount()), firmwareAbi());
	reportPreviousCrash();
	mountFlashVolume();
	// Settings live on the install volume; a board whose storage mounts
	// later (a card only) applies them when it comes (loadDeviceSettings
	// from its mount path).
	gea::geaos::setSystemServices(&g_systemServices);
	{
		// The developer-program roots this Home trusts (Home's
		// gea.targets.esp32.geapp.trustRoots); expiry against the system clock.
		gea::geaos::TrustPolicy policy;
		const std::string roots = GEA_GEAPP_TRUST_ROOTS;
		for (std::size_t start = 0; start < roots.size();) {
			const std::size_t comma = roots.find(',', start);
			const std::size_t end = comma == std::string::npos ? roots.size() : comma;
			if (end > start) policy.roots.push_back(roots.substr(start, end - start));
			start = end + 1;
		}
		policy.now = -1;
		gea::geaos::setTrustPolicy(std::move(policy));
	}
	if (!gea::geaos::loadDeviceSettings(gea::geaos::posixFs())) ESP_LOGI(kTag, "no volume for settings yet");
	startVolumeMonitor();
#if GEA_GEAPP_HOME_BUTTON_GPIO >= 0
	if (!g_buttonTask) {
		const auto pin = static_cast<gpio_num_t>(GEA_GEAPP_HOME_BUTTON_GPIO);
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << pin;
		config.mode = GPIO_MODE_INPUT;
		config.pull_up_en = GPIO_PULLUP_ENABLE;
		config.pull_down_en = GPIO_PULLDOWN_DISABLE;
		config.intr_type = GPIO_INTR_DISABLE;
		gpio_config(&config);
		xTaskCreate(&homeButtonTask, "geapp_home_btn", 3072, nullptr, 5, &g_buttonTask);
	}
#endif
}

}  // namespace gea::platform::geapp

// The framework's panic wrapper (targets/esp32/services/panic_hook.cpp) calls
// this before the reset: keep the registers for reportPreviousCrash. Only while
// a program runs, and nothing here may allocate, lock or log.
extern "C" void gea_geapp_panic_record(const panic_info_t *info)
{
	using gea::platform::geapp::g_breadcrumb;
	if (g_breadcrumb.magic != gea::platform::geapp::kBreadcrumbMagic || !info || !info->frame) return;
#if __XTENSA__
	const auto *frame = static_cast<const XtExcFrame *>(info->frame);
	const auto pc = static_cast<std::uint32_t>(frame->pc);
	g_breadcrumb.pc = pc;
	// A windowed call keeps the window increment in a0's top two bits; the
	// caller's region comes from the pc.
	g_breadcrumb.ra = (static_cast<std::uint32_t>(frame->a0) & 0x3fffffffu) | (pc & 0xc0000000u);
	g_breadcrumb.cause = static_cast<std::uint32_t>(frame->exccause);
	g_breadcrumb.address = static_cast<std::uint32_t>(frame->excvaddr);
#else
	const auto *frame = static_cast<const RvExcFrame *>(info->frame);
	g_breadcrumb.pc = static_cast<std::uint32_t>(frame->mepc);
	g_breadcrumb.ra = static_cast<std::uint32_t>(frame->ra);
	g_breadcrumb.cause = static_cast<std::uint32_t>(frame->mcause);
	g_breadcrumb.address = static_cast<std::uint32_t>(frame->mtval);
#endif
	g_breadcrumb.panicMagic = gea::platform::geapp::kPanicMagic;
}

