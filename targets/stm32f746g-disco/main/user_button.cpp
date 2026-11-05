#include "user_button.h"

#include "gpio_pins.h"
#include "stm32f7xx.h"

namespace gea::stm32::user_button {

namespace {

constexpr std::uint16_t kPin = gpio::pins({11});
constexpr std::uint64_t kDebounceUs = 30'000;

bool gStable = false;
bool gLast = false;
std::uint64_t gChangedUs = 0;

}  // namespace

void init() {
	gpio::configure(GPIOI, kPin, {gpio::Mode::Input, gpio::Pull::None});
}

bool pollPressed(std::uint64_t nowUs) {
	const bool level = gpio::read(GPIOI, kPin);
	if (level != gLast) {
		gLast = level;
		gChangedUs = nowUs;
		return false;
	}

	if (level == gStable || nowUs - gChangedUs < kDebounceUs) return false;

	gStable = level;
	return level;
}

}  // namespace gea::stm32::user_button
