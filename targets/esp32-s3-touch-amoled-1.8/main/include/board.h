#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct QspiDisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t pclk;
	gpio_num_t data0;
	gpio_num_t data1;
	gpio_num_t data2;
	gpio_num_t data3;
	gpio_num_t reset;
	gpio_num_t te; // VBlank/tearing-effect line; GPIO_NUM_NC = vsync disabled
};

struct Ft3168TouchConfig {
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

class Esp32S3TouchAmoled18 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_15,
		.scl = GPIO_NUM_14,
	};

	static constexpr QspiDisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_12,
		.pclk = GPIO_NUM_11,
		.data0 = GPIO_NUM_4,
		.data1 = GPIO_NUM_5,
		.data2 = GPIO_NUM_6,
		.data3 = GPIO_NUM_7,
		.reset = GPIO_NUM_NC,
		.te = GPIO_NUM_NC,
	};

	static constexpr Ft3168TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_21,
	};

	static constexpr Es8311AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_16,
		.bclk = GPIO_NUM_9,
		.ws = GPIO_NUM_45,
		.dout = GPIO_NUM_8,
		.din = GPIO_NUM_10,
		.powerAmplifier = GPIO_NUM_46,
	};
};

using Board = Esp32S3TouchAmoled18;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;

}  // namespace gea::platform::board
