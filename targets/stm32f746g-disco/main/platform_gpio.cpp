// gea's Gpio on the Discovery. Pins are numbered port * 16 + pin, so PA0 is 0
// and the green user LED (PI1) is 129. Pins the board itself drives (SDRAM,
// QSPI, LTDC, touch I2C, the console UART, SWD) refuse configuration and
// writes; any pin can be read.

#include "gpio.h"

#include "gpio_pins.h"
#include "stm32f7xx.h"

#include <array>
#include <cstdint>

namespace {

namespace pins = gea::stm32::gpio;

constexpr int kPortCount = 11;  // A..K

// Per port, the pins owned by board peripherals (see system_init.cpp, lcd.cpp,
// touch_panel.cpp, console.cpp).
constexpr std::array<std::uint16_t, kPortCount> kReserved = {
	pins::pins({9, 13, 14}),                                         // A: USART1 TX, SWDIO, SWCLK
	pins::pins({2, 6, 7}),                                           // B: QSPI CLK/NCS, USART1 RX
	pins::pins({3}),                                                 // C: SDRAM
	pins::pins({0, 1, 8, 9, 10, 11, 12, 13, 14, 15}),                // D: SDRAM, QSPI
	pins::pins({0, 1, 2, 4, 7, 8, 9, 10, 11, 12, 13, 14, 15}),       // E: SDRAM, QSPI, LTDC
	pins::pins({0, 1, 2, 3, 4, 5, 11, 12, 13, 14, 15}),              // F: SDRAM
	pins::pins({0, 1, 4, 5, 8, 12, 15}),                             // G: SDRAM, LTDC
	pins::pins({3, 5, 7, 8}),                                        // H: SDRAM, touch I2C
	pins::pins({9, 10, 11, 12, 13, 14, 15}),                         // I: LTDC, button, LCD enable, touch INT
	pins::pins({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15}),  // J: LTDC
	pins::pins({0, 1, 2, 3, 4, 5, 6, 7}),                            // K: LTDC, backlight
};

struct Pin {
	GPIO_TypeDef *port = nullptr;
	std::uint16_t mask = 0;
	bool reserved = true;
};

Pin pinFor(int number) {
	if (number < 0 || number >= kPortCount * 16) return {};

	const int port = number / 16;
	const auto mask = static_cast<std::uint16_t>(1U << (number % 16));
	auto *base = reinterpret_cast<GPIO_TypeDef *>(GPIOA_BASE + static_cast<std::uint32_t>(port) * (GPIOB_BASE - GPIOA_BASE));
	return {base, mask, (kReserved[static_cast<std::size_t>(port)] & mask) != 0};
}

}  // namespace

namespace gea::platform::gpio {

bool Gpio::configureOutput(int pin) {
	const Pin target = pinFor(pin);
	if (target.reserved) return false;

	pins::configure(target.port, target.mask, {pins::Mode::Output, pins::Pull::None, pins::Speed::Low});
	return true;
}

bool Gpio::configureInput(int pin, bool pull_up) {
	const Pin target = pinFor(pin);
	if (target.reserved) return false;

	pins::configure(target.port, target.mask, {pins::Mode::Input, pull_up ? pins::Pull::Up : pins::Pull::None});
	return true;
}

bool Gpio::write(int pin, bool level) {
	const Pin target = pinFor(pin);
	if (target.reserved) return false;

	pins::write(target.port, target.mask, level);
	return true;
}

bool Gpio::read(int pin) {
	const Pin target = pinFor(pin);
	return target.port && pins::read(target.port, target.mask);
}

// No addressable LED strip on this board.
bool AddressableLed::set(int, int, int, int) { return false; }
bool AddressableLed::off(int) { return false; }
bool AddressableLed::attach(int, int) { return false; }
bool AddressableLed::setPixel(int, int, int, int) { return false; }
bool AddressableLed::show() { return false; }
void AddressableLed::detach() {}

}  // namespace gea::platform::gpio
