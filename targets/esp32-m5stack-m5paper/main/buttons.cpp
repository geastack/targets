#include "buttons.h"

#include "board.h"
#include "input.h"

#include <cstdint>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::m5paper::buttons {

namespace {

constexpr char kTag[] = "m5paper_buttons";
constexpr int kPollMs = 20;
constexpr int kDebouncePolls = 2;
constexpr int kRepeatDelayMs = 450;
constexpr int kRepeatIntervalMs = 160;
constexpr int kPowerOffHoldMs = 2500;
bool initialized = false;
std::uint32_t nextPollMs = 0;

struct ButtonState {
	gpio_num_t pin;
	int keyCode;
	bool repeat;
	bool pressed = false;
	int stablePolls = 0;
	std::int64_t pressedAtUs = 0;
	std::int64_t lastRepeatUs = 0;
};

ButtonState left{gea::platform::board::buttons.left, 37, true};   // ArrowLeft
ButtonState power{gea::platform::board::buttons.power, 0, false}; // reserved for launcher/power
ButtonState right{gea::platform::board::buttons.right, 39, true}; // ArrowRight

[[noreturn]] void powerOff()
{
	ESP_LOGI(kTag, "power button held: shutting down");
	while (gpio_get_level(gea::platform::board::buttons.power) == 0) vTaskDelay(pdMS_TO_TICKS(kPollMs));
	gpio_set_level(gea::platform::board::power.epdPower, 0);
	gpio_set_level(gea::platform::board::power.externalPower, 0);
	vTaskDelay(pdMS_TO_TICKS(20));
	gpio_set_level(gea::platform::board::power.mainPower, 0);

	// USB can keep the ESP32 alive even after the battery latch drops. In that
	// case sleep until the same hardware power button is pressed again.
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_sleep_enable_ext0_wakeup(gea::platform::board::buttons.power, 0));
	esp_deep_sleep_start();
}

void pollKey(ButtonState &button)
{
	const bool rawPressed = gpio_get_level(button.pin) == 0;
	if (rawPressed != button.pressed) {
		if (++button.stablePolls < kDebouncePolls) return;
		button.stablePolls = 0;
		button.pressed = rawPressed;
		if (rawPressed) {
			button.pressedAtUs = esp_timer_get_time();
			button.lastRepeatUs = button.pressedAtUs;
			if (button.keyCode) gea::framework::input::queueKeyDown(button.keyCode);
		}
		return;
	}
	button.stablePolls = 0;
	if (!button.pressed) return;
	const std::int64_t now = esp_timer_get_time();
	if (button.pin == gea::platform::board::buttons.power &&
	    now - button.pressedAtUs >= static_cast<std::int64_t>(kPowerOffHoldMs) * 1000) powerOff();
	if (!button.repeat || now - button.pressedAtUs < static_cast<std::int64_t>(kRepeatDelayMs) * 1000 ||
	    now - button.lastRepeatUs < static_cast<std::int64_t>(kRepeatIntervalMs) * 1000) return;
	button.lastRepeatUs = now;
	if (button.keyCode) gea::framework::input::queueKeyDown(button.keyCode);
}

}  // namespace

void init()
{
	if (initialized) return;
	gpio_config_t config = {};
	config.pin_bit_mask = (1ULL << gea::platform::board::buttons.left) |
	                      (1ULL << gea::platform::board::buttons.power) |
	                      (1ULL << gea::platform::board::buttons.right);
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;  // GPIO34-39 are input-only; board supplies pull-ups.
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&config));
	initialized = true;
}

void poll(int nowMs)
{
	if (!initialized) return;
	const auto now = static_cast<std::uint32_t>(nowMs);
	if (nextPollMs != 0 && static_cast<std::int32_t>(now - nextPollMs) < 0) return;
	nextPollMs = now + kPollMs;
	pollKey(left);
	pollKey(power);
	pollKey(right);
}

}  // namespace gea::platform::m5paper::buttons
