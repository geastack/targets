#include "apps.h"
#include "apps/app_manager.h"
#include "input.h"
#include "memory_config.h"
#include "board.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
#include "esp_heap_caps.h"
#endif
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::apps {

namespace {

constexpr const char *kTag = "launcher_button";
constexpr gpio_num_t kGpio = gea::platform::board::launcherButton.pin;
constexpr int kActiveLevel = gea::platform::board::launcherButton.activeLevel;
// A board with no button leaves the pin GPIO_NUM_NC (-1); `1ULL << kGpio` on
// that constant is a negative shift the compiler rejects even though start()
// returns before using it, so the mask is formed behind a branch.
constexpr std::uint64_t pinMask(gpio_num_t pin)
{
	return pin < 0 ? 0ULL : (1ULL << static_cast<unsigned>(pin));
}
constexpr int kDebounceMs = 50;
constexpr int kPollMs = 25;
constexpr int kLongPressMs = 800;
// ESP-IDF task stack sizes are bytes. The BOOT-button task only polls GPIO and
// posts launcher events; keep its internal-RAM footprint small and verify with
// the high-water probes below.
// Hardware high-water logging showed only ~788 bytes used from the old 4096-byte
// allocation. Keep over 2.5x that measured peak and return 2 KiB of scarce
// internal RAM to radio/display recovery paths.
constexpr std::uint32_t kTaskStackBytes = 2048;
constexpr UBaseType_t kTaskPriority = 10;

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
void logInternalHeap(const char *stage)
{
	multi_heap_info_t info{};
	heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	ESP_LOGI(kTag,
		"heap probe [%s] internal_free=%u internal_largest=%u internal_min=%u free_blocks=%u alloc_blocks=%u",
		stage ? stage : "?",
		static_cast<unsigned>(info.total_free_bytes),
		static_cast<unsigned>(info.largest_free_block),
		static_cast<unsigned>(info.minimum_free_bytes),
		static_cast<unsigned>(info.free_blocks),
		static_cast<unsigned>(info.allocated_blocks));
}
#endif

class LauncherButton {
public:
	static LauncherButton &instance()
	{
		static LauncherButton button;
		return button;
	}

	void start()
	{
		if (task_) return;
		if (kGpio == GPIO_NUM_NC) {
			ESP_LOGI(kTag, "no launcher button configured");
			return;
		}
		if (!gea::framework::apps::AppManager::shouldEnableLauncherButton()) {
			ESP_LOGW(kTag, "BOOT launcher button disabled by AppManager policy");
			return;
		}

		// GPIO0 on ESP32-S3 is a strapping pin and may be left muxed to a
		// non-GPIO function (USB-serial JTAG, RTC GPIO, etc.) by the bootloader
		// or a prior reset cause. gpio_config alone does not detach those
		// alternates, so the pin can read as floating even with pull_up_en set.
		// Resetting first forces the pad back to a clean GPIO matrix entry.
		gpio_reset_pin(kGpio);

		gpio_config_t config = {
			.pin_bit_mask = pinMask(kGpio),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE,
		};
		const esp_err_t err = gpio_config(&config);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Failed to configure BOOT launcher button on GPIO0: %s", esp_err_to_name(err));
			return;
		}

		const BaseType_t ok = xTaskCreate(
			&LauncherButton::taskMain,
			"launcher_button",
			kTaskStackBytes,
			this,
			kTaskPriority,
			&task_);
		if (ok != pdPASS) {
			task_ = nullptr;
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
			logInternalHeap("launcher_button:create_failed");
#endif
			ESP_LOGE(kTag,
				"Failed to start BOOT launcher button task (stack_bytes=%u)",
				static_cast<unsigned>(kTaskStackBytes));
			return;
		}
		ESP_LOGI(kTag,
			"BOOT launcher button task started on GPIO%d (priority=%u, stack_bytes=%u, hwm=%u, level=%d)",
			static_cast<int>(kGpio),
			static_cast<unsigned>(kTaskPriority),
			static_cast<unsigned>(kTaskStackBytes),
			static_cast<unsigned>(uxTaskGetStackHighWaterMark(task_)),
			gpio_get_level(kGpio));
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		logInternalHeap("launcher_button:after_start");
#endif
	}

private:
	LauncherButton() = default;

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
	void logStackProbe(const char *stage) const
	{
		ESP_LOGI(kTag,
			"stack probe [%s] task=launcher_button stack_bytes=%u hwm=%u",
			stage ? stage : "?",
			static_cast<unsigned>(kTaskStackBytes),
			static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
	}
#endif

	void run()
	{
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		logStackProbe("launcher_button:task_start");
#endif
		while (true) {
			if (gpio_get_level(kGpio) == kActiveLevel) {
				vTaskDelay(pdMS_TO_TICKS(kDebounceMs));
				if (gpio_get_level(kGpio) == kActiveLevel) {
					ESP_LOGI(kTag, "BOOT pressed");
					TickType_t pressedAt = xTaskGetTickCount();
					bool longPressHandled = false;
					while (gpio_get_level(kGpio) == kActiveLevel) {
						if (!longPressHandled && xTaskGetTickCount() - pressedAt >= pdMS_TO_TICKS(kLongPressMs)) {
							const bool returned = gea::framework::apps::AppManager::returnRunningAppToLauncher("BOOT button long press");
							ESP_LOGI(kTag, "BOOT long press: returnRunningAppToLauncher=%d", returned ? 1 : 0);
							if (!returned) {
								const bool toggled = gea::framework::apps::AppManager::queueSettingsToggle();
								ESP_LOGI(kTag, "BOOT long press: queueSettingsToggle=%d", toggled ? 1 : 0);
							}
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
							logStackProbe("launcher_button:after_long_press");
#endif
							longPressHandled = true;
						}
						vTaskDelay(pdMS_TO_TICKS(kPollMs));
					}
					if (!longPressHandled) {
						const bool returned = gea::framework::apps::AppManager::returnRunningAppToLauncher("BOOT button press");
						ESP_LOGI(kTag, "BOOT short press: returnRunningAppToLauncher=%d", returned ? 1 : 0);
						if (!returned) {
							// At the launcher (no sub-app to return to). Set the back-button
							// flag so the launcher's rAF loop dismisses Settings if it's
							// open. Polled via `Input.consumeBackButton()`.
							gea::framework::input::pressBackButton();
							ESP_LOGI(kTag, "BOOT short press: back-button flag set");
						}
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
						logStackProbe("launcher_button:after_short_press");
#endif
					}
				}
			}

			vTaskDelay(pdMS_TO_TICKS(kPollMs));
		}
	}

	static void taskMain(void *arg)
	{
		static_cast<LauncherButton *>(arg)->run();
	}

	TaskHandle_t task_ = nullptr;
};

}  // namespace

void startLauncherButtonTask()
{
	LauncherButton::instance().start();
}

}  // namespace gea::platform::esp32::apps
