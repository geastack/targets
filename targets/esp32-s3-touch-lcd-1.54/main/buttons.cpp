// BOOT/PLUS/PWR: GPIO0/4/5, active low. BOOT and PLUS repeat as
// ArrowUp/ArrowDown; PWR sends Enter. Holding PWR drops the battery latch.
#include "buttons.h"

#include "board.h"
#include "input.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::waveshare_lcd154::buttons {

namespace {

constexpr const char *kTag = "lcd154_buttons";
constexpr gpio_num_t kUpPin = gea::platform::board::buttons.key1;
constexpr gpio_num_t kDownPin = gea::platform::board::buttons.key2;
constexpr gpio_num_t kPowerPin = GPIO_NUM_5;
constexpr int kUpKeyCode = 38;    // ArrowUp
constexpr int kDownKeyCode = 40;  // ArrowDown
constexpr int kPollMs = 20;
constexpr int kDebouncePolls = 2;       // 2 consecutive matching reads = stable
constexpr int kRepeatDelayMs = 450;     // hold this long before auto-repeat
constexpr int kRepeatIntervalMs = 200;  // then one press per interval
constexpr std::uint32_t kTaskStackBytes = 3072;
constexpr UBaseType_t kTaskPriority = 10;

struct ButtonState {
	gpio_num_t pin;
	int keyCode;
	bool pressed = false;
	int stablePolls = 0;
	std::int64_t pressedAtUs = 0;
	std::int64_t lastRepeatUs = 0;
};

void pollButton(ButtonState &button)
{
	const bool rawPressed = gpio_get_level(button.pin) == 0;
	if (rawPressed != button.pressed) {
		if (++button.stablePolls < kDebouncePolls) return;
		button.stablePolls = 0;
		button.pressed = rawPressed;
		if (rawPressed) {
			const std::int64_t now = esp_timer_get_time();
			button.pressedAtUs = now;
			button.lastRepeatUs = now;
			gea::framework::input::queueKeyDown(button.keyCode);
		}
		return;
	}
	button.stablePolls = 0;
	if (!button.pressed) return;
	const std::int64_t now = esp_timer_get_time();
	if (now - button.pressedAtUs < static_cast<std::int64_t>(kRepeatDelayMs) * 1000) return;
	if (now - button.lastRepeatUs < static_cast<std::int64_t>(kRepeatIntervalMs) * 1000) return;
	button.lastRepeatUs = now;
	gea::framework::input::queueKeyDown(button.keyCode);
}

void buttonsTask(void *)
{
	ButtonState up{kUpPin, kUpKeyCode};
	ButtonState down{kDownPin, kDownKeyCode};
	ButtonState power{kPowerPin, 13};
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(kPollMs));
		pollButton(up);
		pollButton(down);
		pollButton(power);
		if (power.pressed && esp_timer_get_time() - power.pressedAtUs >= 2000000) {
			gpio_set_level(gea::platform::board::power.enable, 0);
		}
	}
}

}  // namespace

void startButtonsTask()
{
	gpio_config_t conf = {};
	conf.intr_type = GPIO_INTR_DISABLE;
	conf.mode = GPIO_MODE_INPUT;
	conf.pin_bit_mask = (1ULL << kUpPin) | (1ULL << kDownPin) | (1ULL << kPowerPin);
	conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	conf.pull_up_en = GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&conf));

	if (xTaskCreatePinnedToCore(buttonsTask, "lcd154_buttons", kTaskStackBytes, nullptr, kTaskPriority, nullptr, 1) != pdPASS) {
		ESP_LOGE(kTag, "failed to start buttons task");
		return;
	}
	ESP_LOGI(kTag, "buttons ready: KEY1(GPIO%d)=ArrowUp KEY2(GPIO%d)=ArrowDown",
	         static_cast<int>(kUpPin), static_cast<int>(kDownPin));
}

}  // namespace gea::platform::waveshare_lcd154::buttons
