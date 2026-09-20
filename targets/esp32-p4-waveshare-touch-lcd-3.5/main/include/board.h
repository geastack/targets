#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

// Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5: ST7796 panel on a standard 1-data-line
// SPI bus (SPI2), 8-bit command/parameter, DC + CS + RST GPIOs, PWM backlight.
// The panel's SDO is wired to LCD_MISO (schematic: GPIO22, otherwise unused) —
// no TE pin is brought out, so scan-position feedback comes from reading the
// ST7796 GSCAN (0x45) register over MISO instead.
struct St7796DisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t mosi;
	gpio_num_t miso;
	gpio_num_t sclk;
	gpio_num_t cs;
	gpio_num_t dc;
	gpio_num_t reset;
	gpio_num_t backlight;
};

// FT5x06-family capacitive touch (FT6336U on this board) over the shared I2C bus.
struct Ft5x06TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

struct Es8311AudioConfig {
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
};

class Esp32P4WaveshareTouchLcd35 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_7,
		.scl = GPIO_NUM_8,
	};

	static constexpr St7796DisplayConfig display{
		.spiHost = SPI2_HOST,
		.mosi = GPIO_NUM_20,
		.miso = GPIO_NUM_22,
		.sclk = GPIO_NUM_21,
		.cs = GPIO_NUM_23,
		.dc = GPIO_NUM_26,
		.reset = GPIO_NUM_27,
		.backlight = GPIO_NUM_28,
	};

	static constexpr Ft5x06TouchConfig touch{
		.reset = GPIO_NUM_29,
		.interrupt = GPIO_NUM_50,
	};

	static constexpr Es8311AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_13,
		.bclk = GPIO_NUM_12,
		.ws = GPIO_NUM_10,
		.dout = GPIO_NUM_9,
		.din = GPIO_NUM_11,
		.powerAmplifier = GPIO_NUM_53,
	};
};

using Board = Esp32P4WaveshareTouchLcd35;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;

}  // namespace gea::platform::board
