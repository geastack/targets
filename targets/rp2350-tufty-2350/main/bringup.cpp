#include "board.h"
#include "rp2350_panel.h"

#include "pixel.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iterator>

namespace board = gea::platform::board;
namespace pixel = gea::framework::graphics::pixel;

namespace {

i2c_inst_t *i2cBus()
{
	return board::i2c.bus == 1 ? i2c1 : i2c0;
}

class PicoI2cRegisterBus final : public gea::chips::pcf85063::RegisterBus {
public:
	explicit PicoI2cRegisterBus(std::uint8_t address) : address_(address) {}

	bool writeRegister(std::uint8_t reg, std::uint8_t value) override
	{
		const std::uint8_t data[] = {reg, value};
		return i2c_write_blocking(i2cBus(), address_, data, 2, false) == 2;
	}

	bool writeRegisters(std::uint8_t reg, const std::uint8_t *data, std::size_t length) override
	{
		if (!data || length == 0 || length > 31) return false;
		std::uint8_t buffer[32]{};
		buffer[0] = reg;
		for (std::size_t i = 0; i < length; i++) buffer[i + 1] = data[i];
		const int expected = static_cast<int>(length + 1);
		return i2c_write_blocking(i2cBus(), address_, buffer, expected, false) == expected;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override
	{
		return readRegisters(reg, &value, 1);
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override
	{
		if (!data || length == 0) return false;
		if (i2c_write_blocking(i2cBus(), address_, &reg, 1, true) != 1) return false;
		return i2c_read_blocking(i2cBus(), address_, data, static_cast<int>(length), false) == static_cast<int>(length);
	}

private:
	std::uint8_t address_;
};

void initPower()
{
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

void initI2c()
{
	i2c_init(i2cBus(), board::i2c.frequencyHz);
	gpio_set_function(board::i2c.sda, GPIO_FUNC_I2C);
	gpio_set_function(board::i2c.scl, GPIO_FUNC_I2C);
	gpio_pull_up(board::i2c.sda);
	gpio_pull_up(board::i2c.scl);
}

bool i2cProbe(std::uint8_t address)
{
	std::uint8_t value = 0;
	return i2c_read_blocking(i2cBus(), address, &value, 1, false) >= 0;
}

void probeRtc()
{
	PicoI2cRegisterBus bus(board::rtc.address);
	gea::chips::pcf85063::RealTimeClock rtc(bus);
	const bool ok = rtc.begin();
	gea::chips::pcf85063::DateTime now{};
	const bool read = ok && rtc.readDateTime(now);
	std::printf("pcf85063: present=%d begin=%d read=%d %04d-%02d-%02d %02d:%02d:%02d\n",
	            i2cProbe(board::rtc.address) ? 1 : 0,
	            ok ? 1 : 0,
	            read ? 1 : 0,
	            now.year,
	            now.month,
	            now.day,
	            now.hour,
	            now.minute,
	            now.second);
}

void initButtons()
{
	for (const auto &button : board::buttons) {
		gpio_init(button.pin);
		gpio_set_dir(button.pin, GPIO_IN);
		if (button.pullUp) gpio_pull_up(button.pin);
	}
}

void printButtons()
{
	std::printf("buttons:");
	for (const auto &button : board::buttons) {
		const bool down = button.activeLow ? gpio_get(button.pin) == 0 : gpio_get(button.pin) != 0;
		std::printf(" gp%d=%d", button.pin, down ? 1 : 0);
	}
	std::printf("\n");
}

void colorBars(std::uint16_t *pixels, int width, int height, int originX, int originY, void *)
{
	static constexpr std::uint16_t colors[] = {
	    0xf800,
	    0xffe0,
	    0x07e0,
	    0x07ff,
	    0x001f,
	    0xf81f,
	    0xffff,
	    0x0000,
	};
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			const int absoluteX = originX + x;
			const int bar = (absoluteX * static_cast<int>(std::size(colors))) / gea::rp2350::kPanelWidth;
			pixels[y * width + x] = pixel::fromRgb565(colors[bar]);
		}
		(void)originY;
	}
}

}  // namespace

int main()
{
	stdio_init_all();
	sleep_ms(1500);

	std::printf("\nGea RP2350 Tufty 2350 bring-up\n");
	std::printf("flash=%u psram=%u display=%dx%d\n",
	            board::storage.flashBytes,
	            board::storage.psramBytes,
	            board::display.width,
	            board::display.height);

	initPower();
	initI2c();
	initButtons();
	probeRtc();

	const int batteryRaw = adc_read();
	const float batteryMv = static_cast<float>(batteryRaw) * 3300.0f * 2.0f / 4095.0f;
	std::printf("battery raw=%d mv=%.1f\n", batteryRaw, batteryMv);

	gea::rp2350::panelInit();
	gea::rp2350::panelStreamRect(0,
	                             0,
	                             gea::rp2350::kPanelWidth - 1,
	                             gea::rp2350::kPanelHeight - 1,
	                             colorBars,
	                             nullptr);

	while (true) {
		printButtons();
		sleep_ms(1000);
	}
}
