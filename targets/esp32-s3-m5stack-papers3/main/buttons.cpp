#include "buttons.h"

#include "board.h"

#include "driver/gpio.h"
#include "esp_log.h"

namespace gea::platform::papers3::buttons {

namespace {

constexpr char kTag[] = "papers3_buttons";
bool initialized = false;

}  // namespace

void init()
{
	if (initialized) return;
	// No user button on this board (see board.h). Configuring a pin here would
	// mean touching GPIO0, which is a strapping pin — don't.
	if (gea::platform::board::buttons.power == GPIO_NUM_NC) {
		initialized = true;
		ESP_LOGI(kTag, "no user button on this board; input is touch only");
		return;
	}
	// The single physical button (shared with the launcher/back handler in
	// launcher_button.cpp) is an input with an internal pull-up. Paging in the
	// reader is driven by the GT911 touch panel, so this file stays minimal.
	gpio_config_t config = {};
	// `& 63` keeps the shift count valid for the compiler in the GPIO_NUM_NC
	// (-1) case above, which is unreachable here but still type-checked.
	config.pin_bit_mask = 1ULL << (static_cast<int>(gea::platform::board::buttons.power) & 63);
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

}  // namespace gea::platform::papers3::buttons
