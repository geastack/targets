// The M5Stack StickS3 has two user buttons on the face, KEY1 (GPIO11) and
// KEY2 (GPIO12), both active low with pull-ups. They are the board's only
// GPIO input (the side power button talks to the M5PM1 PMIC, and power
// on/off/download-mode chords are handled in PMIC hardware), so deliver them
// to apps as web keydown events — KEY1 = ArrowUp (38), KEY2 = ArrowDown (40)
// — via the same queue the runtime frame loop drains. Held buttons
// auto-repeat.
#include "buttons.h"

#include "board.h"
#include "input.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32_s3_sticks3::buttons {

namespace {

constexpr const char *kTag = "sticks3_buttons";
constexpr gpio_num_t kUpPin = gea::platform::board::buttons.key1;
constexpr gpio_num_t kDownPin = gea::platform::board::buttons.key2;
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
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(kPollMs));
		pollButton(up);
		pollButton(down);
	}
}

}  // namespace

void startButtonsTask()
{
	gpio_config_t conf = {};
	conf.intr_type = GPIO_INTR_DISABLE;
	conf.mode = GPIO_MODE_INPUT;
	conf.pin_bit_mask = (1ULL << kUpPin) | (1ULL << kDownPin);
	conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	conf.pull_up_en = GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&conf));

	if (xTaskCreatePinnedToCore(buttonsTask, "sticks3_buttons", kTaskStackBytes, nullptr, kTaskPriority, nullptr, 1) != pdPASS) {
		ESP_LOGE(kTag, "failed to start buttons task");
		return;
	}
	ESP_LOGI(kTag, "buttons ready: KEY1(GPIO%d)=ArrowUp KEY2(GPIO%d)=ArrowDown",
	         static_cast<int>(kUpPin), static_cast<int>(kDownPin));
}

}  // namespace gea::platform::esp32_s3_sticks3::buttons
