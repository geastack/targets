#pragma once

#include "stm32f7xx.h"

#include <cstddef>
#include <cstdint>

// Blocking register-level I2C master with short timeouts, so a wedged bus
// costs a few milliseconds of a frame instead of stalling the loop.
namespace gea::stm32 {

class I2cBus {
public:
	I2cBus(I2C_TypeDef *instance, std::uint8_t address) : instance_(instance), address_(address) {}

	void init(std::uint32_t timing);
	// One write transfer of `length` bytes (register address included).
	bool write(const std::uint8_t *data, std::size_t length);
	bool writeRegister(std::uint8_t reg, std::uint8_t value);
	// Writes `address` (a register address of any width), then reads `length` bytes.
	bool read(const std::uint8_t *address, std::size_t addressLength, std::uint8_t *data, std::size_t length);
	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length);

private:
	void start(std::uint32_t length, bool read, bool autoEnd);
	bool waitFor(std::uint32_t flag);
	bool finish();

	I2C_TypeDef *instance_;
	std::uint8_t address_;
};

}  // namespace gea::stm32
