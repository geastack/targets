#include "apps/app_manager.h"
#include "board.h"
#include "input.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::apps {

namespace {

constexpr char kTag[] = "launcher_button";
constexpr gpio_num_t kButton = gea::platform::board::buttons.power;
constexpr int kDebounceMs = 50;
constexpr int kPollMs = 25;
constexpr int kLongPressMs = 800;

class LauncherButton {
public:
	static LauncherButton &instance()
	{
		static LauncherButton button;
		return button;
	}

	void start()
	{
		if (task_ || !gea::framework::apps::AppManager::shouldEnableLauncherButton()) return;
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << kButton;
		config.mode = GPIO_MODE_INPUT;
		config.pull_up_en = GPIO_PULLUP_DISABLE;  // GPIO38 has the board pull-up.
		config.pull_down_en = GPIO_PULLDOWN_DISABLE;
		config.intr_type = GPIO_INTR_DISABLE;
		if (gpio_config(&config) != ESP_OK) return;
		if (xTaskCreate(taskMain, "launcher_button", 4096, this, 10, &task_) != pdPASS) task_ = nullptr;
	}

private:
	void run()
	{
		for (;;) {
			if (gpio_get_level(kButton) == 0) {
				vTaskDelay(pdMS_TO_TICKS(kDebounceMs));
				if (gpio_get_level(kButton) == 0) {
					const TickType_t pressedAt = xTaskGetTickCount();
					bool longHandled = false;
					while (gpio_get_level(kButton) == 0) {
						if (!longHandled && xTaskGetTickCount() - pressedAt >= pdMS_TO_TICKS(kLongPressMs)) {
							const bool returned = gea::framework::apps::AppManager::returnRunningAppToLauncher(
								"M5Paper power button long press");
							if (!returned) gea::framework::apps::AppManager::queueSettingsToggle();
							longHandled = true;
						}
						vTaskDelay(pdMS_TO_TICKS(kPollMs));
					}
					if (!longHandled) {
					const bool returned = gea::framework::apps::AppManager::returnRunningAppToLauncher(
						"M5Paper power button press");
						if (!returned) gea::framework::input::pressBackButton();
					}
				}
			}
			vTaskDelay(pdMS_TO_TICKS(kPollMs));
		}
	}

	static void taskMain(void *arg) { static_cast<LauncherButton *>(arg)->run(); }
	TaskHandle_t task_ = nullptr;
};

}  // namespace

void startLauncherButtonTask()
{
	LauncherButton::instance().start();
	ESP_LOGI(kTag, "M5Paper launcher button configured on GPIO%d", static_cast<int>(kButton));
}

}  // namespace gea::platform::esp32::apps
