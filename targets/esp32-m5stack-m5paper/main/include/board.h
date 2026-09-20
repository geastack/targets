#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct It8951DisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t busy;
	gpio_num_t sclk;
	gpio_num_t mosi;
	gpio_num_t miso;
};

struct Gt911TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

struct ButtonConfig {
	gpio_num_t left;
	gpio_num_t power;
	gpio_num_t right;
};

struct PowerRailConfig {
	gpio_num_t mainPower;
	gpio_num_t externalPower;
	gpio_num_t epdPower;
	gpio_num_t batteryAdc;
};

struct SdSpiStorageConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t sclk;
	gpio_num_t mosi;
	gpio_num_t miso;
};

class M5StackM5Paper {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_21,
		.scl = GPIO_NUM_22,
	};

	static constexpr It8951DisplayConfig display{
		.spiHost = SPI3_HOST,
		.cs = GPIO_NUM_15,
		.busy = GPIO_NUM_27,
		.sclk = GPIO_NUM_14,
		.mosi = GPIO_NUM_12,
		.miso = GPIO_NUM_13,
	};

	static constexpr Gt911TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_36,
	};

	static constexpr ButtonConfig buttons{
		.left = GPIO_NUM_37,
		.power = GPIO_NUM_38,
		.right = GPIO_NUM_39,
	};

	static constexpr PowerRailConfig power{
		.mainPower = GPIO_NUM_2,
		.externalPower = GPIO_NUM_5,
		.epdPower = GPIO_NUM_23,
		.batteryAdc = GPIO_NUM_35,
	};

	static constexpr SdSpiStorageConfig storage{
		.spiHost = SPI3_HOST,
		.cs = GPIO_NUM_4,
		.sclk = GPIO_NUM_14,
		.mosi = GPIO_NUM_12,
		.miso = GPIO_NUM_13,
	};
};

using Board = M5StackM5Paper;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto buttons = Board::buttons;
inline constexpr auto power = Board::power;
inline constexpr auto storage = Board::storage;

}  // namespace gea::platform::board
