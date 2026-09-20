#include "board.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace board = gea::platform::board;

namespace {

i2c_inst_t *i2cBus() {
	return board::i2c.bus == 1 ? i2c1 : i2c0;
}

class PicoI2cRegisterBus final : public gea::chips::qmi8658::RegisterBus,
                                 public gea::chips::ft6336::RegisterBus,
                                 public gea::chips::pcf85063::RegisterBus {
public:
	explicit PicoI2cRegisterBus(std::uint8_t address) : address_(address) {}

	bool writeRegister(std::uint8_t reg, std::uint8_t value) override {
		const std::uint8_t data[] = {reg, value};
		return i2c_write_blocking(i2cBus(), address_, data, 2, false) == 2;
	}

	bool writeRegisters(std::uint8_t reg, const std::uint8_t *data, std::size_t length) override {
		if (!data || length == 0 || length > 31) return false;
		std::uint8_t buffer[32]{};
		buffer[0] = reg;
		for (std::size_t i = 0; i < length; i++) buffer[i + 1] = data[i];
		const int expected = static_cast<int>(length + 1);
		return i2c_write_blocking(i2cBus(), address_, buffer, expected, false) == expected;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override {
		return readRegisters(reg, &value, 1);
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override {
		if (!data || length == 0) return false;
		if (i2c_write_blocking(i2cBus(), address_, &reg, 1, true) != 1) return false;
		return i2c_read_blocking(i2cBus(), address_, data, static_cast<int>(length), false) == static_cast<int>(length);
	}

private:
	std::uint8_t address_;
};

class PicoDelay final : public gea::chips::qmi8658::Delay {
public:
	void milliseconds(int ms) override {
		sleep_ms(ms);
	}
};

class PicoPowerIo final : public gea::chips::eta6098::PowerIo {
public:
	bool setPowerHold(bool enabled) override {
		gpio_put(board::power.powerHold, enabled ? 1 : 0);
		return true;
	}

	bool readPowerKey(bool &pressed) override {
		pressed = gpio_get(board::power.powerKey) == 0;
		return true;
	}

	bool readBatteryAdcRaw(int &raw) override {
		adc_select_input(board::power.batteryAdcChannel);
		raw = static_cast<int>(adc_read());
		return true;
	}
};

void initPowerPins() {
	gpio_init(board::power.powerHold);
	gpio_set_dir(board::power.powerHold, GPIO_OUT);
	gpio_put(board::power.powerHold, 1);

	gpio_init(board::power.powerKey);
	gpio_set_dir(board::power.powerKey, GPIO_IN);
	gpio_pull_up(board::power.powerKey);

	adc_init();
	adc_gpio_init(board::power.batteryAdc);
	adc_select_input(board::power.batteryAdcChannel);
}

void initI2c() {
	i2c_init(i2cBus(), board::i2c.frequencyHz);
	gpio_set_function(board::i2c.sda, GPIO_FUNC_I2C);
	gpio_set_function(board::i2c.scl, GPIO_FUNC_I2C);
	gpio_pull_up(board::i2c.sda);
	gpio_pull_up(board::i2c.scl);
}

void resetTouch() {
	gpio_init(board::touch.reset);
	gpio_set_dir(board::touch.reset, GPIO_OUT);
	gpio_put(board::touch.reset, 1);
	sleep_ms(10);
	gpio_put(board::touch.reset, 0);
	sleep_ms(10);
	gpio_put(board::touch.reset, 1);
	sleep_ms(300);

	gpio_init(board::touch.interrupt);
	gpio_set_dir(board::touch.interrupt, GPIO_IN);
	gpio_pull_up(board::touch.interrupt);
}

bool i2cProbe(std::uint8_t address) {
	std::uint8_t value = 0;
	return i2c_read_blocking(i2cBus(), address, &value, 1, false) >= 0;
}

void printI2cProbe(const char *name, std::uint8_t address) {
	std::printf("%s 0x%02x: %s\n", name, address, i2cProbe(address) ? "present" : "not found");
}

void probeRtc() {
	PicoI2cRegisterBus bus(board::rtc.address);
	gea::chips::pcf85063::RealTimeClock rtc(bus);
	const bool ok = rtc.begin();
	gea::chips::pcf85063::DateTime now{};
	const bool read = ok && rtc.readDateTime(now);
	std::printf("pcf85063: begin=%d read=%d %04d-%02d-%02d %02d:%02d:%02d\n",
	            ok ? 1 : 0,
	            read ? 1 : 0,
	            now.year,
	            now.month,
	            now.day,
	            now.hour,
	            now.minute,
	            now.second);
}

void probeImu() {
	PicoI2cRegisterBus bus(board::imu.address);
	PicoDelay delay;
	gea::chips::qmi8658::Driver imu(bus, delay);
	const bool ok = imu.begin();
	gea::chips::qmi8658::Sample sample{};
	const bool read = ok && imu.readSample(sample);
	std::printf("qmi8658: begin=%d whoami=0x%02x read=%d accel=(%.3f, %.3f, %.3f)\n",
	            ok ? 1 : 0,
	            imu.lastWhoAmI(),
	            read ? 1 : 0,
	            sample.ax,
	            sample.ay,
	            sample.az);
}

void probeTouch() {
	resetTouch();
	PicoI2cRegisterBus bus(board::touch.address);
	gea::chips::ft6336::ControllerCore touch;
	std::uint8_t chipId = 0;
	const bool idOk = touch.readChipId(bus, chipId);
	const bool cfgOk = idOk && touch.configure(bus);
	gea::chips::ft6336::TouchSample sample{};
	const bool touching = cfgOk && touch.read(bus, sample);
	std::printf("ft6336: id_ok=%d id=0x%02x configure=%d touching=%d x=%d y=%d\n",
	            idOk ? 1 : 0,
	            chipId,
	            cfgOk ? 1 : 0,
	            touching ? 1 : 0,
	            sample.x,
	            sample.y);
}

void printDisplayCore() {
	std::size_t initCount = 0;
	(void)gea::chips::rm690b0::CommandSet::initCommands(initCount);
	const auto window = gea::chips::rm690b0::CommandSet::addressWindow(0, 0, board::display.width - 1, board::display.height - 1);
	std::printf("rm690b0: %dx%d x_gap=%d init_commands=%u CASET=%02x %02x %02x %02x\n",
	            board::display.width,
	            board::display.height,
	            board::display.panelXGap,
	            static_cast<unsigned>(initCount),
	            window.columns[0],
	            window.columns[1],
	            window.columns[2],
	            window.columns[3]);
}

}  // namespace

int main() {
	stdio_init_all();
	sleep_ms(1500);

	std::printf("\nGea RP2350 Touch AMOLED 2.41 bring-up\n");
	std::printf("flash=%u psram=%u\n", board::storage.flashBytes, board::storage.psramBytes);

	initPowerPins();
	PicoPowerIo powerIo;
	gea::chips::eta6098::PowerManagementUnit power(powerIo, board::power.battery);
	power.begin();
	std::printf("eta6098: battery=%.1fmV percent=%d key=%d\n",
	            power.batteryVoltageMv(),
	            power.batteryPercent(),
	            power.powerKeyPressed() ? 1 : 0);

	initI2c();
	printI2cProbe("pcf85063", board::rtc.address);
	printI2cProbe("qmi8658", board::imu.address);
	printI2cProbe("ft6336", board::touch.address);
	probeRtc();
	probeImu();
	probeTouch();
	printDisplayCore();

	while (true) {
		sleep_ms(1000);
	}
}
