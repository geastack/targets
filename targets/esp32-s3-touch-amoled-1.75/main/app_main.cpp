#include "gea_embedded_app_config.h"

#include "apps/app_manager.h"
#include "connectivity/ble_hid.h"
#include "display.h"

// Forward-declare here instead of through a header — there's already a
// `wifi.h` in `lib/gea-embedded/include/` defining WifiAdapter/WifiDriver,
// and adding a second `wifi.h` under `targets/esp32/connectivity/` shadows
// it in the include search path, breaking the WifiStation TU which needs
// the base class.
namespace gea::targets::esp32::wifi {
void registerDriver();
}
namespace gea::targets::esp32::gps {
void registerDriver();
}
#include "memory_config.h"
#include "runtime.h"
#include "services/device_control.h"

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
#include "esp_heap_caps.h"
#endif
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
// An app whose own realtime tasks must outrank the render loop (audio DSP, a USB
// isochronous host) sets gea.defines.GEA_EMBEDDED_RUNTIME_TASK_PRIORITY to place
// this below them; the default is what this board needs on its own.
#ifndef GEA_EMBEDDED_RUNTIME_TASK_PRIORITY
#define GEA_EMBEDDED_RUNTIME_TASK_PRIORITY 5
#endif
constexpr UBaseType_t kRuntimeTaskPriority = GEA_EMBEDDED_RUNTIME_TASK_PRIORITY;

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

void runRuntime()
{
	gea::framework::RuntimeOptions options{};
	options.width = gea::platform::display::kWidth;
	options.height = gea::platform::display::kHeight;
	gea::framework::Runtime::run(options);
}

}  // namespace

extern "C" void app_main(void)
{
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
	gea::targets::esp32::gps::registerDriver();

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
	gea::targets::esp32::ble::registerHidDriver();

	vTaskPrioritySet(nullptr, kRuntimeTaskPriority);
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
	runRuntime();

	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking main task");
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
