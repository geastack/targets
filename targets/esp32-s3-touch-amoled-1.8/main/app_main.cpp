#include "gea_embedded_app_config.h"

#include "apps/app_manager.h"
#include "connectivity/ble_hid.h"
#include "display.h"
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#include "host/display_orientation.h"
#endif

// Forward-declare here instead of through a header — there's already a
// `wifi.h` in `lib/gea-embedded/include/` defining WifiAdapter/WifiDriver,
// and adding a second `wifi.h` under `targets/esp32/connectivity/` shadows
// it in the include search path, breaking the WifiStation TU which needs
// the base class.
namespace gea::targets::esp32::wifi {
void registerDriver();
}
#include "memory_config.h"
#include "runtime.h"
#include "services/device_control.h"

#include <cstdlib>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char *kTag = "gea_embedded";

void runRuntime();

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
void logHeapProbe(const char *stage)
{
	ESP_LOGI(kTag,
		"heap probe [%s] internal_free=%u internal_largest=%u internal_min=%u psram_free=%u",
		stage ? stage : "?",
		static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
}
#endif

constexpr int kAppMainTaskStack = CONFIG_ESP_MAIN_TASK_STACK_SIZE;
// Inferred audio apps put rendering below codec workers. Preserve the board's
// graphics default for other apps and an explicit override when supplied.
#ifndef GEA_EMBEDDED_RUNTIME_TASK_PRIORITY
#if defined(GEA_EMBEDDED_APP_USES_AUDIO) && GEA_EMBEDDED_APP_USES_AUDIO
#define GEA_EMBEDDED_RUNTIME_TASK_PRIORITY 3
#else
#define GEA_EMBEDDED_RUNTIME_TASK_PRIORITY 5
#endif
#endif
constexpr UBaseType_t kRuntimeTaskPriority = GEA_EMBEDDED_RUNTIME_TASK_PRIORITY;

// Match the 2.06 target: keep flash-touching boot on the internal main stack,
// then optionally give the event loop its own stack and release main's stack.
#ifndef GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
#define GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES 0
#endif
#ifndef GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL
#define GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL 0
#endif
constexpr int kRuntimeTaskStackBytes = GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES;

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
void logCurrentTaskStack(const char *stage, int stackBytes)
{
	TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
	ESP_LOGI(kTag,
		"stack probe [%s] task=%s core=%d stack_arg=%d hwm=%u",
		stage ? stage : "?",
		currentTask ? pcTaskGetName(currentTask) : "?",
		static_cast<int>(xPortGetCoreID()),
		stackBytes,
		static_cast<unsigned>(uxTaskGetStackHighWaterMark(currentTask)));
}
#endif

gea::framework::RuntimeOptions runtimeOptions()
{
	gea::framework::RuntimeOptions options{};
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
	using OrientationState = gea::framework::display::detail::DisplayOrientationState;
	OrientationState::setSupportedOrientations(
		std::vector<std::string>{"portrait-primary", "landscape-primary"});
#if GEA_EMBEDDED_DISPLAY_DEFAULT_PORTRAIT_PRIMARY
	OrientationState::setOrientation("portrait-primary");
#else
	OrientationState::setOrientation("landscape-primary");
#endif
	options.width = OrientationState::width();
	options.height = OrientationState::height();
#elif GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
	using OrientationState = gea::framework::display::detail::DisplayOrientationState;
	OrientationState::setSupportedOrientations("landscape-primary");
	options.width = OrientationState::width();
	options.height = OrientationState::height();
#else
	options.width = gea::platform::display::kWidth;
	options.height = gea::platform::display::kHeight;
#endif
	return options;
}

void runRuntime()
{
	gea::framework::Runtime::run(runtimeOptions());
}

#if GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
void runtimeTask(void *)
{
	ESP_LOGI(kTag, "Gea runtime task started stack=%d %s", kRuntimeTaskStackBytes,
		GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL ? "external" : "internal");
	runRuntime();
	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking runtime task");
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
#endif

}  // namespace

// The app's stylesheet registration, emitted as a function (--cpp-prelude-symbol)
// rather than a file-scope global constructor. Static constructors run on the
// ROM's startup stack before FreeRTOS -- 8 KB on the ESP32-S31 -- and a CSS
// app's registration body alone takes a 3.5 KB frame, so it overran that stack
// into the heap. Weak: canvas-only builds emit no registration.
void gea_plugin_cpp_register_prelude() __attribute__((weak));

extern "C" void app_main(void)
{
	if (gea_plugin_cpp_register_prelude) gea_plugin_cpp_register_prelude();
	// Force the targets/esp32/apps/app_manager.cpp TU to be linked in. That
	// TU's only side effect is a file-scope `PlatformRegistration` whose
	// constructor calls AppManager::setPlatform(...). With -Wl,--gc-sections
	// and the main component built as a static archive, nothing else
	// references launcherPlatform(), so the linker drops the entire object
	// file — the static ctor never runs, AppManager::platform_ stays null,
	// and AppManager::startLauncherButtonTask() / returnRunningAppToLauncher
	// / ... all become silent no-ops. Result: the BOOT button task is never
	// created and no `launcher_button` log lines are ever emitted.
	(void)gea::platform::esp32::apps::launcherPlatform();

	// Same static-archive trap as `launcherPlatform()` above. Three different
	// `wifi.cpp` files all archive as `wifi.cpp.obj` in `libmain.a`; symbol-
	// driven archive linking pulls in `lib/gea-embedded/wifi.cpp` (WifiAdapter
	// + NullWifiDriver) and `lib/gea-embedded/host/wifi.cpp` (WifiBackend
	// shim) because external code references them, but the ESP32 driver's
	// only side effect was an anonymous-namespace static-init global with no
	// external references — so the obj was dropped, leaving NullWifiDriver
	// (whose enabled()/setEnabled() are silent no-ops) as the active driver.
	// `wifi().init()` returned false in <1ms at boot ("WiFi failed; OTA not
	// available") and the Settings panel WiFi toggle did nothing.
	//
	// Note: wifi.cpp itself wraps `wifi_init_config_t.osi_funcs` to bump the
	// closed-source wifi blob's hardcoded 3584-byte task stack to 8192,
	// avoiding a stack overflow during WPA2 auth on this firmware.
	//
	// GEA_EMBEDDED_WIFI_DISABLED skips driver registration entirely, leaving the
	// no-op NullWifiDriver active (no esp_wifi_init/start, no WiFi DMA/ISR bus
	// traffic). Used to isolate whether WiFi PSRAM/cache-bus contention is what
	// starves the direct-PSRAM SPI flush FIFO.
#ifndef GEA_EMBEDDED_WIFI_DISABLED
	gea::targets::esp32::wifi::registerDriver();
#endif

	// Same static-archive trap as `launcherPlatform()` and `wifi::registerDriver()`
	// above. Reference the HID driver explicitly so its TU is pulled in and
	// `HidServer` replaces the board-independent `NullBleDriver`. Originally
	// gated on `GEA_EMBEDDED_APP_USES_BLE`, but `geatsc analyze` only detects
	// BLE usage when the import literal is `from 'gea-embedded'` — the
	// internal `runtime-surface` re-export chain that `gea-embedded`'s
	// Settings panel uses (`import { BLE, ... } from '../../runtime-surface'`)
	// gets a false negative, the macro never lands, the driver never
	// registers, and the BLUETOOTH toggle is a silent no-op (NullBleDriver's
	// `setEnabled` ignores its argument; `enabled()` always returns false).
	// We can't just define the macro unconditionally either — it also gates
	// `gea_init`'s SPIRAM-vs-DRAM placement in `app_runner.cpp`, and forcing
	// DRAM there fails (64 KB stack > 32 KB largest contiguous block at
	// init time). So: register the driver here unconditionally, leave the
	// macro narrow.
#ifndef GEA_EMBEDDED_BLE_DISABLED
	gea::targets::esp32::ble::registerHidDriver();
#endif

#if !GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
	vTaskPrioritySet(nullptr, kRuntimeTaskPriority);
#endif
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
	logCurrentTaskStack("main_task:before_runtime", kAppMainTaskStack);
#endif
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	logHeapProbe("app_main:before_device_control_task");
#endif
	gea::platform::esp32::services::startDeviceControlTask();
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	logHeapProbe("app_main:before_runtime");
#endif
#if GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
	// Boot includes SPIFFS/NVS and must finish before entering a PSRAM stack.
	if (!gea::framework::Runtime::boot(runtimeOptions())) {
		ESP_LOGE(kTag, "Gea bring-up failed; the UI will not start (the app's own tasks keep running)");
		return;
	}
#if !GEA_EMBEDDED_NO_DISPLAY
	TaskHandle_t runtime = nullptr;
	const BaseType_t created = xTaskCreatePinnedToCoreWithCaps(
		&runtimeTask, "gea_runtime", kRuntimeTaskStackBytes, nullptr,
		kRuntimeTaskPriority, &runtime, xPortGetCoreID(),
		GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
		                                         : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
	if (created != pdPASS) {
		ESP_LOGE(kTag, "Failed to allocate %d-byte Gea runtime stack", kRuntimeTaskStackBytes);
		std::abort();
	}
	ESP_LOGI(kTag, "Bootstrap complete; Gea runtime task=%p stack=%d", runtime, kRuntimeTaskStackBytes);
#endif
	// Returning releases the internal main stack; the runtime task owns the loop.
#else
	runRuntime();

	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking main task");
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
#endif
}
