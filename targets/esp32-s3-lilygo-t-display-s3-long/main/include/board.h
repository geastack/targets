#pragma once

#include "driver/gpio.h"
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
	gpio_num_t te;
};

struct TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

class LilygoTDisplayS3Long {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_15,
		.scl = GPIO_NUM_10,
	};

	static constexpr QspiDisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_12,
		.pclk = GPIO_NUM_17,
		.data0 = GPIO_NUM_13,
		.data1 = GPIO_NUM_18,
		.data2 = GPIO_NUM_21,
		.data3 = GPIO_NUM_14,
		.reset = GPIO_NUM_16,
		.te = GPIO_NUM_NC,
	};

	static constexpr TouchConfig touch{
		.reset = GPIO_NUM_2,
		.interrupt = GPIO_NUM_11,
	};
};

using Board = LilygoTDisplayS3Long;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;

}  // namespace gea::platform::board
