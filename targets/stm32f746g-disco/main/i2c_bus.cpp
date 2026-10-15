#include "i2c_bus.h"

#include "clock.h"

namespace gea::stm32 {

namespace {

constexpr std::uint64_t kTimeoutUs = 5'000;

}  // namespace

void I2cBus::init(std::uint32_t timing) {
	instance_->CR1 = 0;
	instance_->TIMINGR = timing;
	instance_->CR1 = I2C_CR1_PE;
}

bool I2cBus::waitFor(std::uint32_t flag) {
	const std::uint64_t deadline = clock::micros() + kTimeoutUs;
	while (!(instance_->ISR & flag)) {
		if (instance_->ISR & I2C_ISR_NACKF || clock::micros() > deadline) {
			instance_->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
			instance_->CR1 = 0;
			instance_->CR1 = I2C_CR1_PE;
			return false;
		}
	}
	return true;
}

void I2cBus::start(std::uint32_t length, bool read, bool autoEnd) {
	instance_->CR2 = (static_cast<std::uint32_t>(address_) << 1) | (length << I2C_CR2_NBYTES_Pos) | (read ? I2C_CR2_RD_WRN : 0) |
	                 (autoEnd ? I2C_CR2_AUTOEND : 0) | I2C_CR2_START;
}

bool I2cBus::finish() {
	if (!waitFor(I2C_ISR_STOPF)) return false;

	instance_->ICR = I2C_ICR_STOPCF;
	return true;
}

bool I2cBus::write(const std::uint8_t *data, std::size_t length) {
	start(static_cast<std::uint32_t>(length), false, true);
	for (std::size_t i = 0; i < length; ++i) {
		if (!waitFor(I2C_ISR_TXIS)) return false;

		instance_->TXDR = data[i];
	}
	return finish();
}

bool I2cBus::writeRegister(std::uint8_t reg, std::uint8_t value) {
	const std::uint8_t bytes[] = {reg, value};
	return write(bytes, sizeof(bytes));
}

bool I2cBus::read(const std::uint8_t *address, std::size_t addressLength, std::uint8_t *data, std::size_t length) {
	start(static_cast<std::uint32_t>(addressLength), false, false);
	for (std::size_t i = 0; i < addressLength; ++i) {
		if (!waitFor(I2C_ISR_TXIS)) return false;

		instance_->TXDR = address[i];
	}
	if (!waitFor(I2C_ISR_TC)) return false;

	start(static_cast<std::uint32_t>(length), true, true);
	for (std::size_t i = 0; i < length; ++i) {
		if (!waitFor(I2C_ISR_RXNE)) return false;

		data[i] = static_cast<std::uint8_t>(instance_->RXDR);
	}
	return finish();
}

bool I2cBus::readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) {
	return read(&reg, 1, data, length);
}

}  // namespace gea::stm32
