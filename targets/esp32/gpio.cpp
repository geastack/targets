// ESP32 digital pins and a WS2812 strand.
//
// The LED half drives WS2812 timing through RMT directly rather than pulling in
// the managed `led_strip` component: this file is compiled into every ESP32
// board, and a new managed dependency would land in the dependency lock of all
// of them to light one diode on the boards that have one.

#include "gpio.h"

#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

namespace {

constexpr const char *kTag = "gpio";

bool validPin(int pin)
{
	return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(pin));
}

// WS2812 is a single-wire protocol where a bit is a fixed-length pulse and only
// the high/low split says whether it is a 0 or a 1. At the 10 MHz resolution
// below one tick is 100 ns, so these are the datasheet's T0H/T0L/T1H/T1L.
constexpr std::uint32_t kLedResolutionHz = 10 * 1000 * 1000;
constexpr int kMaxPixels = 64;
// A WS2812 latches the frame it has just clocked in when the line stays low for
// longer than a bit slot -- 50us in the datasheet, 300 here because the cost is
// nothing and a strand that latches early shows the previous frame's colours.
constexpr std::uint32_t kLedLatchUs = 300;

rmt_channel_handle_t gLedChannel = nullptr;
rmt_encoder_handle_t gLedEncoder = nullptr;
int gLedPin = -1;
int gLedCount = 0;
// GRB, which is the order WS2812 clocks in — not RGB.
std::uint8_t gLedBytes[kMaxPixels * 3];

void releaseLed()
{
	// Deleting the channel hands the pin back with whatever level the last RMT
	// symbol left on it, and a floating data line reads as noise to the strand:
	// enough to latch a garbage frame the moment the channel goes away. Parking
	// the pin as a low output first makes the line rest where the protocol says
	// idle is, so a strand driven on another pin meanwhile stays dark.
	const int parkPin = gLedPin;

	if (gLedEncoder) {
		rmt_del_encoder(gLedEncoder);
		gLedEncoder = nullptr;
	}
	if (gLedChannel) {
		rmt_disable(gLedChannel);
		rmt_del_channel(gLedChannel);
		gLedChannel = nullptr;
	}
	gLedPin = -1;
	gLedCount = 0;

	if (parkPin >= 0) {
		const gpio_num_t num = static_cast<gpio_num_t>(parkPin);
		gpio_reset_pin(num);
		gpio_set_direction(num, GPIO_MODE_OUTPUT);
		gpio_set_level(num, 0);
	}
}

}  // namespace

namespace gea::platform::gpio {


bool Gpio::configureOutput(int pin)
{
	if (!validPin(pin)) return false;
	const gpio_num_t num = static_cast<gpio_num_t>(pin);
	gpio_reset_pin(num);
	gpio_config_t config = {};
	config.pin_bit_mask = 1ULL << pin;
	config.mode = GPIO_MODE_OUTPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	return gpio_config(&config) == ESP_OK;
}

bool Gpio::configureInput(int pin, bool pull_up)
{
	if (!validPin(pin)) return false;
	const gpio_num_t num = static_cast<gpio_num_t>(pin);
	// A strapping pin (GPIO0 among them) can still be muxed to a boot function;
	// resetting forces it back to a plain GPIO matrix entry, which is why
	// launcher_button.cpp does the same before configuring BOOT.
	gpio_reset_pin(num);
	gpio_config_t config = {};
	config.pin_bit_mask = 1ULL << pin;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	config.pull_down_en = pull_up ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	return gpio_config(&config) == ESP_OK;
}

bool Gpio::write(int pin, bool level)
{
	if (!validPin(pin)) return false;
	return gpio_set_level(static_cast<gpio_num_t>(pin), level ? 1 : 0) == ESP_OK;
}

bool Gpio::read(int pin)
{
	if (!validPin(pin)) return false;
	return gpio_get_level(static_cast<gpio_num_t>(pin)) != 0;
}

bool AddressableLed::attach(int pin, int count)
{
	if (!validPin(pin) || count <= 0 || count > kMaxPixels) return false;
	if (gLedChannel && gLedPin == pin && gLedCount == count) return true;
	releaseLed();

	rmt_tx_channel_config_t channel = {};
	channel.gpio_num = static_cast<gpio_num_t>(pin);
	channel.clk_src = RMT_CLK_SRC_DEFAULT;
	channel.resolution_hz = kLedResolutionHz;
	channel.mem_block_symbols = 64;
	channel.trans_queue_depth = 4;
	if (rmt_new_tx_channel(&channel, &gLedChannel) != ESP_OK) {
		ESP_LOGE(kTag, "RMT channel for the LED on GPIO%d could not be created", pin);
		gLedChannel = nullptr;
		return false;
	}

	rmt_bytes_encoder_config_t encoder = {};
	encoder.bit0.level0 = 1;
	encoder.bit0.duration0 = 3;  // T0H 0.3us
	encoder.bit0.level1 = 0;
	encoder.bit0.duration1 = 9;  // T0L 0.9us
	encoder.bit1.level0 = 1;
	encoder.bit1.duration0 = 9;  // T1H 0.9us
	encoder.bit1.level1 = 0;
	encoder.bit1.duration1 = 3;  // T1L 0.3us
	encoder.flags.msb_first = 1;
	if (rmt_new_bytes_encoder(&encoder, &gLedEncoder) != ESP_OK) {
		ESP_LOGE(kTag, "RMT encoder for the LED on GPIO%d could not be created", pin);
		releaseLed();
		return false;
	}
	if (rmt_enable(gLedChannel) != ESP_OK) {
		ESP_LOGE(kTag, "RMT channel for the LED on GPIO%d could not be enabled", pin);
		releaseLed();
		return false;
	}

	gLedPin = pin;
	gLedCount = count;
	std::memset(gLedBytes, 0, sizeof(gLedBytes));
	return true;
}

bool AddressableLed::setPixel(int index, int r, int g, int b)
{
	if (index < 0 || index >= gLedCount) return false;
	const auto clamp = [](int value) -> std::uint8_t {
		if (value < 0) return 0;
		if (value > 255) return 255;
		return static_cast<std::uint8_t>(value);
	};
	// GRB, not RGB.
	gLedBytes[index * 3 + 0] = clamp(g);
	gLedBytes[index * 3 + 1] = clamp(r);
	gLedBytes[index * 3 + 2] = clamp(b);
	return true;
}

bool AddressableLed::show()
{
	if (!gLedChannel || !gLedEncoder || gLedCount <= 0) return false;
	rmt_transmit_config_t tx = {};
	tx.loop_count = 0;
	if (rmt_transmit(gLedChannel, gLedEncoder, gLedBytes, static_cast<std::size_t>(gLedCount) * 3, &tx) != ESP_OK) {
		return false;
	}
	// The strand latches on a long low, so the next transmit must not begin
	// until this one has drained; waiting here also makes show() mean "the LED
	// is now showing this" rather than "the write was queued".
	if (rmt_tx_wait_all_done(gLedChannel, 100) != ESP_OK) return false;
	// Drained is not latched. Hold the idle low for the reset period before
	// returning, so the colours are on the strand by the time show() does --
	// including when the caller's next move is to drive a different pin, which
	// tears this channel down.
	esp_rom_delay_us(kLedLatchUs);
	return true;
}

void AddressableLed::detach() { releaseLed(); }

bool AddressableLed::set(int pin, int r, int g, int b)
{
	// attach() returns early when the pin and length already match, so a caller
	// that drives the LED every frame re-allocates nothing; a caller that moves
	// to a different pin gets the previous channel released first.
	if (!attach(pin, 1)) return false;
	if (!setPixel(0, r, g, b)) return false;
	return show();
}

bool AddressableLed::off(int pin) { return set(pin, 0, 0, 0); }

}  // namespace gea::platform::gpio

// Link anchor. Nothing in the framework or in `main` calls anything in this
// file — the only caller is the app's own geatsc archive, which the linker
// reaches after every ESP-IDF component archive has gone past. Left to itself
// the archive linker therefore takes gpio.cpp.obj during the final
// main/gea_framework rescan, by which point esp_driver_rmt (and the gdma/soc
// archives behind it) are behind the cursor and every rmt_* call is undefined.
//
// The gea_framework component names this symbol in a `-u` link flag, which
// makes the linker treat it as undefined from the start and take the object
// while the driver archives are still ahead. `extern "C"` so that flag spells
// a stable name rather than a C++ mangling.
extern "C" void gea_gpio_link_bindings(void) {}
