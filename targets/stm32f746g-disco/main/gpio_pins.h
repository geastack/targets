#pragma once

#include "stm32f7xx.h"

#include <cstdint>
#include <initializer_list>

// Register-level pin setup. Every write is read-modify-write per pin, so
// peripherals that share a port (SDRAM and QSPI on GPIOD/GPIOE) never clobber
// each other's configuration.
namespace gea::stm32::gpio {

enum class Mode : std::uint32_t { Input = 0, Output = 1, Alternate = 2, Analog = 3 };
enum class Pull : std::uint32_t { None = 0, Up = 1, Down = 2 };
enum class Speed : std::uint32_t { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };
enum class Drive : std::uint32_t { PushPull = 0, OpenDrain = 1 };

struct PinConfig {
	Mode mode = Mode::Input;
	Pull pull = Pull::None;
	Speed speed = Speed::Low;
	Drive drive = Drive::PushPull;
	std::uint32_t alternate = 0;
};

inline void setField(volatile std::uint32_t &reg, int index, int width, std::uint32_t value) {
	const std::uint32_t mask = (1U << width) - 1U;
	const int shift = index * width;
	reg = (reg & ~(mask << shift)) | ((value & mask) << shift);
}

inline void configure(GPIO_TypeDef *port, std::uint16_t pins, const PinConfig &config) {
	for (int pin = 0; pin < 16; ++pin) {
		if (!(pins & (1U << pin))) continue;

		setField(port->AFR[pin / 8], pin % 8, 4, config.alternate);
		setField(port->OSPEEDR, pin, 2, static_cast<std::uint32_t>(config.speed));
		setField(port->OTYPER, pin, 1, static_cast<std::uint32_t>(config.drive));
		setField(port->PUPDR, pin, 2, static_cast<std::uint32_t>(config.pull));
		setField(port->MODER, pin, 2, static_cast<std::uint32_t>(config.mode));
	}
}

inline void write(GPIO_TypeDef *port, std::uint16_t pins, bool high) {
	port->BSRR = high ? pins : static_cast<std::uint32_t>(pins) << 16;
}

inline bool read(GPIO_TypeDef *port, std::uint16_t pin) {
	return (port->IDR & pin) != 0;
}

inline constexpr std::uint16_t pins(std::initializer_list<int> numbers) {
	std::uint16_t mask = 0;
	for (int number : numbers) mask |= static_cast<std::uint16_t>(1U << number);
	return mask;
}

}  // namespace gea::stm32::gpio
