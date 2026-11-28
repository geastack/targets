#include "touch_panel.h"

#include "board.h"
#include "clock.h"
#include "gpio_pins.h"
#include "i2c_bus.h"
#include "lcd.h"

#include <algorithm>

namespace gea::stm32::touch_panel {

namespace {

class Bus final : public chips::ft5336::RegisterBus {
public:
	I2cBus i2c{I2C3, chips::ft5336::kI2cAddress};

	bool writeRegister(std::uint8_t reg, std::uint8_t value) override {
		return i2c.writeRegister(reg, value);
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override {
		return i2c.readRegisters(reg, &value, 1);
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override {
		return i2c.readRegisters(reg, data, length);
	}
};

// CTP_RST is wired to NRST, so the controller restarts with the MCU and only
// answers ~200 ms after reset (ST's ft5336_Init waits the same).
constexpr std::uint64_t kControllerBootUs = 200'000;

Bus gBus;
chips::ft5336::ControllerCore gController;

}  // namespace

bool init() {
	const gpio::PinConfig i2c{gpio::Mode::Alternate, gpio::Pull::None, gpio::Speed::High, gpio::Drive::OpenDrain, 4};
	gpio::configure(GPIOH, gpio::pins({7, 8}), i2c);

	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	(void)RCC->APB1ENR;

	gBus.i2c.init(board::kTouch.i2cTiming);
	while (clock::micros() < kControllerBootUs) {}

	return gController.configure(gBus);
}

bool read(Sample &sample) {
	chips::ft5336::MultiTouchSample raw{};
	sample.count = 0;
	if (!gController.readMulti(gBus, raw)) return false;

	for (int i = 0; i < raw.count; ++i) {
		const int x = board::kTouch.swapXY ? raw.y[i] : raw.x[i];
		const int y = board::kTouch.swapXY ? raw.x[i] : raw.y[i];
		sample.id[i] = raw.id[i];
		sample.x[i] = std::clamp(x, 0, lcd::kWidth - 1);
		sample.y[i] = std::clamp(y, 0, lcd::kHeight - 1);
	}
	sample.count = raw.count;
	return true;
}

}  // namespace gea::stm32::touch_panel
