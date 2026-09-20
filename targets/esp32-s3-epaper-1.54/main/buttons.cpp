// The non-touch ESP32-S3-ePaper-1.54 has two user buttons, both active low
// with pull-ups (vendor button_bsp): BOOT on GPIO0 and PWR on GPIO18. They are
// the board's only physical input, so deliver them to apps as web keydown
// events — BOOT = ArrowUp (38), PWR = ArrowDown (40) — via the same queue the
// runtime frame loop drains for rotary input. Held buttons auto-repeat.
//
// GPIO0 note: launcher_button.cpp owns BOOT on launcher builds; on single-app
// builds its AppManager policy disables that task, so polling it here is safe.
#include "buttons.h"

#include "board.h"
#include "input.h"

#include <cstdint>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32_s3_epaper::buttons {

namespace {

constexpr const char *kTag = "epaper_buttons";
constexpr gpio_num_t kUpPin = GPIO_NUM_0;     // BOOT
constexpr gpio_num_t kDownPin = GPIO_NUM_18;  // PWR
constexpr int kUpKeyCode = 38;    // ArrowUp
constexpr int kDownKeyCode = 40;  // ArrowDown
constexpr int kPollMs = 20;
constexpr int kDebouncePolls = 2;        // 2 consecutive matching reads = stable
constexpr int kRepeatDelayMs = 450;      // hold this long before auto-repeat
constexpr int kRepeatIntervalMs = 200;   // then one press per interval
constexpr int kPowerOffHoldMs = 2500;    // PWR held this long = power off
constexpr std::uint32_t kTaskStackBytes = 3072;
constexpr UBaseType_t kTaskPriority = 10;

struct ButtonState {
	gpio_num_t pin;
	int keyCode;
	bool autoRepeat;
	bool pressed = false;
	int stablePolls = 0;
	std::int64_t pressedAtUs = 0;
	std::int64_t lastRepeatUs = 0;
};

// Vendor power-off semantics (board_power_bsp::EnableDeepLowPowerMode): rails
// off, battery latch (GPIO17) dropped — on battery the board dies right here,
// the e-paper keeps showing the last frame. On USB the chip stays powered, so
// emulate "off" with deep sleep; either button (ANY_LOW) wakes into a fresh
// boot. On battery, power-on is the hardware path: hold PWR until the
// app_main latch takes over (~1s).
[[noreturn]] void powerOff()
{
	ESP_LOGI(kTag, "PWR long-press: powering off");
	// Wait for PWR release so the ext1 ANY_LOW wake doesn't fire instantly.
	while (gpio_get_level(kDownPin) == 0) vTaskDelay(pdMS_TO_TICKS(kPollMs));
	vTaskDelay(pdMS_TO_TICKS(100));

	gpio_set_level(gea::platform::board::power.epdPower, 1);    // active low: off
	gpio_set_level(gea::platform::board::power.audioPower, 1);  // active low: off
	gpio_set_level(gea::platform::board::power.vbatPower, 0);   // battery latch off

	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
	const std::uint64_t wakeMask = (1ULL << kUpPin) | (1ULL << kDownPin);
	ESP_ERROR_CHECK_WITHOUT_ABORT(esp_sleep_enable_ext1_wakeup_io(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW));
	ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_gpio_pulldown_dis(kUpPin));
	ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_gpio_pullup_en(kUpPin));
	ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_gpio_pulldown_dis(kDownPin));
	ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_gpio_pullup_en(kDownPin));
	// Hold the dropped battery latch through deep sleep so a USB-powered "off"
	// doesn't relatch the battery rail.
	ESP_ERROR_CHECK_WITHOUT_ABORT(rtc_gpio_hold_en(gea::platform::board::power.vbatPower));
	esp_deep_sleep_start();
}

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
	if (!button.autoRepeat) {
		// PWR: held past the threshold powers the board off (no auto-repeat —
		// a repeat ramp that ends in a surprise power-off would be hostile).
		if (now - button.pressedAtUs >= static_cast<std::int64_t>(kPowerOffHoldMs) * 1000) powerOff();
		return;
	}
	if (now - button.pressedAtUs < static_cast<std::int64_t>(kRepeatDelayMs) * 1000) return;
	if (now - button.lastRepeatUs < static_cast<std::int64_t>(kRepeatIntervalMs) * 1000) return;
	button.lastRepeatUs = now;
	gea::framework::input::queueKeyDown(button.keyCode);
}

void buttonsTask(void *)
{
	ButtonState up{kUpPin, kUpKeyCode, /*autoRepeat=*/true};
	ButtonState down{kDownPin, kDownKeyCode, /*autoRepeat=*/false};
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(kPollMs));
		pollButton(up);
		pollButton(down);
	}
}

}  // namespace

void startButtonsTask()
{
	// GPIO0 is a strapping pin — reset it before reconfiguring as plain input.
	gpio_reset_pin(kUpPin);
	gpio_config_t conf = {};
	conf.intr_type = GPIO_INTR_DISABLE;
	conf.mode = GPIO_MODE_INPUT;
	conf.pin_bit_mask = (1ULL << kUpPin) | (1ULL << kDownPin);
	conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	conf.pull_up_en = GPIO_PULLUP_ENABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&conf));

	if (xTaskCreatePinnedToCore(buttonsTask, "epd_buttons", kTaskStackBytes, nullptr, kTaskPriority, nullptr, 1) != pdPASS) {
		ESP_LOGE(kTag, "failed to start buttons task");
		return;
	}
	ESP_LOGI(kTag, "buttons ready: BOOT(GPIO%d)=ArrowUp PWR(GPIO%d)=ArrowDown",
	         static_cast<int>(kUpPin), static_cast<int>(kDownPin));
}

}  // namespace gea::platform::esp32_s3_epaper::buttons
