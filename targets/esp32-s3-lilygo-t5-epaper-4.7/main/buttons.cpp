#include "buttons.h"

#include "board.h"

#include "driver/gpio.h"
#include "esp_log.h"

namespace gea::platform::lilygo_t5::buttons {

namespace {

constexpr char kTag[] = "lilygo_t5_buttons";
bool initialized = false;

}  // namespace

void init()
{
	if (initialized) return;
	// The single physical button (shared with the launcher/back handler in
	// launcher_button.cpp) is an input with an internal pull-up. Paging in the
	// reader is driven by the GT911 touch panel, so this file stays minimal.
	gpio_config_t config = {};
	config.pin_bit_mask = 1ULL << gea::platform::board::buttons.power;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&config));
	initialized = true;
	ESP_LOGI(kTag, "button init on GPIO%d", static_cast<int>(gea::platform::board::buttons.power));
}

void poll(int)
{
	// launcher_button.cpp owns the button semantics on its own task; nothing to
	// sample on the frame loop.
}

}  // namespace gea::platform::lilygo_t5::buttons
