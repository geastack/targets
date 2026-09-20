#include "gea_embedded_app_config.h"
#include "sdkconfig.h"

#if !defined(GEA_EMBEDDED_DIRECT_CANVAS_CONTEXT) || !GEA_EMBEDDED_DIRECT_CANVAS_CONTEXT
#define GEA_EMBEDDED_FULL_BOOT_SERVICES 1
#else
#define GEA_EMBEDDED_FULL_BOOT_SERVICES 0
#endif

#if GEA_EMBEDDED_FULL_BOOT_SERVICES
#include "apps/app_manager.h"
#if CONFIG_BT_ENABLED && defined(GEA_EMBEDDED_APP_USES_BLE)
#include "connectivity/ble_hid.h"
#endif
#endif
#include "display.h"
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#include "host/display_orientation.h"
#endif

#if GEA_EMBEDDED_FULL_BOOT_SERVICES
// Forward-declare here instead of through a header — there's already a
// `wifi.h` in `lib/gea-embedded/include/` defining WifiAdapter/WifiDriver,
// and adding a second `wifi.h` under `targets/esp32/connectivity/` shadows
// it in the include search path, breaking the WifiStation TU which needs
// the base class.
namespace gea::targets::esp32::wifi {
void registerDriver();
}
#endif
#include "memory_config.h"
#include "runtime.h"
#if GEA_EMBEDDED_FULL_BOOT_SERVICES
#include "services/device_control.h"
#endif

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
#include <cstdlib>

#include "esp_heap_caps.h"
#endif
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if GEA_EMBEDDED_FULL_BOOT_SERVICES
namespace gea::platform::storage {
// Defined in sdcard_mount.cpp; registers the microSD mount provider so the
// persistent tile/file cache works. Called explicitly from app_main both to run
// it and to force the TU to link (it has no other external references and would
// otherwise be dropped by -Wl,--gc-sections, like launcherPlatform/wifi above).
void installSdCardFileCache();
}  // namespace gea::platform::storage
#endif

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
// The runtime/render loop runs here. Under TE-sync it must not be preempted mid-flush by
// other core-0 tasks (esp_timer @22, the frame ticker @23, touch polling, …) — a preemption
// that lands near the TE deadline makes the frame miss its VBlank and drop, capping the
// steady locked rate ~2 fps below the panel TE. It blocks on the TE between frames, so it
// yields the core to those tasks in the gap; raising it above them only protects the active
// frame. 24 = top of the FreeRTOS band (configMAX_PRIORITIES-1), above the @23 frame ticker.
// An app whose own realtime tasks must outrank the render loop (audio DSP, a USB
// isochronous host) sets gea.defines.GEA_EMBEDDED_RUNTIME_TASK_PRIORITY to place
// this below them; the default is what this board needs on its own.
#ifndef GEA_EMBEDDED_RUNTIME_TASK_PRIORITY
#define GEA_EMBEDDED_RUNTIME_TASK_PRIORITY 24
#endif
constexpr UBaseType_t kRuntimeTaskPriority = GEA_EMBEDDED_RUNTIME_TASK_PRIORITY;

// Where the runtime/render loop's stack lives.
//
// By default it is main_task's own stack: app_main does the whole bring-up and
// then calls runRuntime() without returning, so CONFIG_ESP_MAIN_TASK_STACK_SIZE
// has to cover the deepest TSX frame and pointer-down re-render, and those bytes
// are internal DRAM for the life of the device. That is the right default -- one
// task, one stack, nothing to configure.
//
// It is the wrong default for an app whose own data must live in internal DRAM:
// this board's pedalboard app needs 213,472 bytes of it for its neural-amp
// arenas out of ~240 KiB, and a 20 KiB render stack it cannot move is the
// difference between the model loading and not. Setting
// GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES to a non-zero value runs the loop on its
// own task instead, and app_main then RETURNS -- which deletes main_task and
// hands its stack back to the heap, so the app's own bring-up (the
// gea_app_native_boot hook, where that app allocates its arenas) is the only
// thing main_task's stack still has to cover.
// GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL=1 additionally places that stack in
// PSRAM, which takes it off the internal budget entirely. The constraint that
// comes with it: a task on an external stack must never perform a flash
// operation, because a flash write disables the cache and ESP-IDF asserts
// esp_task_stack_is_sane_cache_disabled() for exactly that case. Rendering does
// not (embedded assets and fonts are cache-mapped reads), but an app that writes
// NVS, SPIFFS or an OTA slot from inside an event handler must keep that work on
// a task of its own. Needs CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM, which this
// board's defaults already set.
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
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
	// The panel is natively portrait and the CO5300 has no usable XY-swap
	// register, so landscape is the software path in targets/esp32/display.cpp:
	// the Canvas writes logical landscape pixels straight into native-order
	// framebuffer rows. It only engages when the orientation state agrees --
	// TouchRuntime::transformTouchToLogical reads the same state to rotate
	// controller coordinates, so without this the display would rotate and the
	// touch input would not. An app asks for it from its manifest's
	// `gea.defines`, which reach the whole native build:
	// SOFTWARE_LANDSCAPE_PRIMARY=1 and ROTATE_LANDSCAPE=1 (the rotated Canvas
	// bind is compiled out without the second), plus all four dimensions --
	// this board declares none, so display.h otherwise defaults them to portrait
	// 410x502: DISPLAY_WIDTH=502, DISPLAY_HEIGHT=410, DISPLAY_NATIVE_WIDTH=410,
	// DISPLAY_NATIVE_HEIGHT=502. The static_asserts in display.cpp check that
	// pair against the native panel.
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
	ESP_LOGI(kTag, "Gea runtime task started stack=%d %s hwm=%u", kRuntimeTaskStackBytes,
		GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL ? "external" : "internal",
		static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
	runRuntime();
	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking runtime task");
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
#endif

}  // namespace

extern "C" void app_main(void)
{
#if GEA_EMBEDDED_FULL_BOOT_SERVICES
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
	// We can't just define the analyzer macro unconditionally either — it also gates
	// `gea_init`'s SPIRAM-vs-DRAM placement in `app_runner.cpp`, and forcing
	// DRAM there fails (64 KB stack > 32 KB largest contiguous block at
	// init time). The board build's independently resolved BLE capability controls
	// whether this driver is linked and registered; the analyzer macro stays narrow.
#if CONFIG_BT_ENABLED && defined(GEA_EMBEDDED_APP_USES_BLE)
	gea::targets::esp32::ble::registerHidDriver();
#ifdef GEA_EMBEDDED_BLE_OTA
	gea::targets::esp32::ble::startOtaServer();
#endif
#endif

	// Register the microSD mount provider so the persistent file cache (maps OSM
	// tiles, recordings, GEADEV PUSH) actually works. Lazy: the card isn't touched
	// until the first file op. Without this, ensureMounted() is always false and
	// every tile re-downloads.
	gea::platform::storage::installSdCardFileCache();
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
#if GEA_EMBEDDED_FULL_BOOT_SERVICES
	gea::platform::esp32::services::startDeviceControlTask();
#endif
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	logHeapProbe("app_main:before_runtime");
#endif
#if GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
	// Run the WHOLE bring-up HERE, on main_task, before the loop moves to a task
	// whose stack may be external. Not just the app's native boot hook: the
	// framework's own bring-up reads and writes flash too -- StorageService::init
	// mounts SPIFFS, the localStorage restore and the radios go through NVS -- and
	// a flash operation from an external stack aborts in
	// esp_task_stack_is_sane_cache_disabled. Hoisting only the app hook left
	// exactly that crash: the SPIFFS mount inside Runtime::run asserted on the
	// PSRAM-stacked loop task, on every boot, about nine seconds in. Bring-up on
	// main_task is also what makes the arrangement worth anything -- the app
	// claims its internal RAM while main_task's stack is the only one committed,
	// and returning below hands that stack back. Runtime::run() on the loop task
	// finds this already done and goes straight to receiving events.
	// A failed bring-up is NOT fatal here. On a board whose app owns hardware of
	// its own -- an audio engine, a USB host -- that hardware is already running
	// by now, and aborting because the display could not get its DMA staging
	// would take a working instrument down with the screen. Log it, skip the loop
	// task, and let app_main return: the app's own tasks keep running, and its
	// maintenance path stays reachable.
	if (!gea::framework::Runtime::boot(runtimeOptions())) {
		ESP_LOGE(kTag, "Gea bring-up failed; the UI will not start (the app's own tasks keep running)");
		return;
	}

#if GEA_EMBEDDED_NO_DISPLAY
	// The runtime task exists to run the frame loop, and a board with no panel
	// and no canvas has no frames: boot() above never created an event queue, so
	// the loop would have nothing to wait on and would spin a core flat out,
	// starving every lower-priority task behind it. boot() is the whole of the
	// bring-up here, so there is no second task to create, and its stack is
	// never allocated.
	ESP_LOGI(kTag, "Bootstrap complete; no display on this board, so no Gea runtime loop");
#else
	// Same core main_task ran on: the priority comment above is about not being
	// preempted by the other core-0 tasks between TE deadlines, which only holds
	// if the loop stays there.
	TaskHandle_t runtime = nullptr;
	const BaseType_t created = xTaskCreatePinnedToCoreWithCaps(
		&runtimeTask,
		"gea_runtime",
		kRuntimeTaskStackBytes,
		nullptr,
		kRuntimeTaskPriority,
		&runtime,
		xPortGetCoreID(),
		GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
		                                         : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
	if (created != pdPASS) {
		ESP_LOGE(kTag, "Failed to allocate %d-byte Gea runtime stack", kRuntimeTaskStackBytes);
		std::abort();
	}
	// Returning deletes main_task and returns its stack to the heap. Nothing
	// below this point runs, which is the point: the app's bring-up is the only
	// thing CONFIG_ESP_MAIN_TASK_STACK_SIZE now has to cover.
	ESP_LOGI(kTag, "Bootstrap complete; Gea runtime task=%p stack=%d", runtime, kRuntimeTaskStackBytes);
#endif
#else
	runRuntime();

	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking main task");
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
#endif
}
