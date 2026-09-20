#include "gea_embedded_app_config.h"

#include "apps/app_manager.h"
#include "board.h"
#include "buttons.h"
#include "connectivity/ble_hid.h"
#include "display.h"
#include "memory_config.h"
#include "runtime.h"
#include "services/device_control.h"

#include "driver/gpio.h"
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

constexpr char kTag[] = "gea_m5paper";

#ifndef GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES
#define GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES (32 * 1024)
#endif

constexpr int kRuntimeTaskStackBytes = GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES;

void enablePowerRails()
{
	gpio_config_t config = {};
	config.pin_bit_mask = (1ULL << gea::platform::board::power.mainPower) |
	                      (1ULL << gea::platform::board::power.externalPower) |
	                      (1ULL << gea::platform::board::power.epdPower);
	config.mode = GPIO_MODE_OUTPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&config));
	// GPIO2 is the battery soft latch. It must go high before the user releases
	// the power button. GPIO5 and GPIO23 enable expansion and EPD power.
	gpio_set_level(gea::platform::board::power.mainPower, 1);
	gpio_set_level(gea::platform::board::power.externalPower, 1);
	gpio_set_level(gea::platform::board::power.epdPower, 1);
}

[[noreturn]] void runGeaRuntime()
{
	enablePowerRails();
	gea::platform::m5paper::buttons::init();
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
	// ESP-IDF creates app_main before it releases the ROM startup-stack D/IRAM
	// region. Keep that bootstrap stack small, then allocate the real Gea stack
	// here, after heap_caps_enable_nonos_stack_heaps() has exposed the large
	// contiguous internal block. Returning deletes the bootstrap task and gives
	// its stack back to the heap.
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
