#include "gea_embedded_app_config.h"

#include "apps/app_manager.h"
#include "board.h"
#include "buttons.h"
#include "connectivity/ble_hid.h"
#include "display.h"
#include "memory_config.h"
#include "runtime.h"
#include "services/device_control.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <cstdlib>

namespace gea::targets::esp32::wifi {
void registerDriver();
}
namespace gea::platform::storage {
void installSdCardFileCache();
}

namespace {

constexpr char kTag[] = "gea_lilygo_t5";

#ifndef GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
#define GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES (32 * 1024)
#endif

constexpr int kRuntimeTaskStackBytes = GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES;

[[noreturn]] void runGeaRuntime()
{
	// Unlike the M5Paper, this board has no discrete power-rail GPIOs to latch:
	// the ED047TC1 rails come up inside the display engine (epd_poweron via the
	// CFG shift register), so display init owns power sequencing.
	gea::platform::lilygo_t5::buttons::init();
	gea::platform::storage::installSdCardFileCache();

	// These explicit references keep registration-only translation units alive
	// when the component archive is linked with --gc-sections.
	(void)gea::platform::esp32::apps::launcherPlatform();
#ifndef GEA_EMBEDDED_WIFI_DISABLED
	gea::targets::esp32::wifi::registerDriver();
#endif
#ifndef GEA_EMBEDDED_BLE_DISABLED
	gea::targets::esp32::ble::registerHidDriver();
#endif

	gea::platform::esp32::services::startDeviceControlTask();

	gea::framework::RuntimeOptions options{};
	options.width = gea::platform::display::kWidth;
	options.height = gea::platform::display::kHeight;
	gea::framework::Runtime::run(options);

	ESP_LOGE(kTag, "Runtime returned unexpectedly; parking runtime task");
	for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

void runtimeTask(void *)
{
	ESP_LOGI(kTag, "Gea runtime task started stack=%d hwm=%u", kRuntimeTaskStackBytes,
	         static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
	runGeaRuntime();
}

}  // namespace

extern "C" void app_main(void)
{
	// Keep the ROM-startup bootstrap stack small, then allocate the real Gea
	// runtime stack from the large contiguous internal block that ESP-IDF frees
	// after startup. Returning deletes the bootstrap task and reclaims its stack.
	TaskHandle_t runtime = nullptr;
	const BaseType_t created = xTaskCreateWithCaps(
		&runtimeTask,
		"gea_runtime",
		kRuntimeTaskStackBytes,
		nullptr,
		5,
		&runtime,
		MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (created != pdPASS) {
		ESP_LOGE(kTag, "Failed to allocate %d-byte Gea runtime stack", kRuntimeTaskStackBytes);
		std::abort();
	}
	ESP_LOGI(kTag, "Bootstrap complete; Gea runtime task=%p stack=%d", runtime, kRuntimeTaskStackBytes);
}
