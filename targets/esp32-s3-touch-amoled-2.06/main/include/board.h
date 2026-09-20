#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct Co5300DisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t pclk;
	gpio_num_t data0;
	gpio_num_t data1;
	gpio_num_t data2;
	gpio_num_t data3;
	gpio_num_t reset;
	// Tearing-Effect line out of the CO5300 (TEON/0x35 is enabled in the panel
	// init). The runtime waits for its VBlank edge before flushing, so the QSPI
	// write into GRAM never overlaps the panel's scanout (vsync). GPIO_NUM_NC
	// disables the wait on boards that don't route TE.
	gpio_num_t te;
};

struct Ft3168TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

struct Es8311AudioConfig {
	// ESP-IDF 6.0 removed the legacy I2S driver and its i2s_port_t enum; the new
	// channel API's i2s_chan_config_t.id is a plain int (I2S_NUM_AUTO/_0 are now
	// int macros). Keep this an int so I2S_NUM_AUTO assigns cleanly.
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
};

struct SdMmcConfig {
	gpio_num_t clk;
	gpio_num_t cmd;
	gpio_num_t data0;
};

struct LauncherButtonConfig {
	gpio_num_t pin;
	int activeLevel;
};

class Esp32S3TouchAmoled206 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_15,
		.scl = GPIO_NUM_14,
	};

	static constexpr Co5300DisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_12,
		.pclk = GPIO_NUM_11,
		.data0 = GPIO_NUM_4,
		.data1 = GPIO_NUM_5,
		.data2 = GPIO_NUM_6,
		.data3 = GPIO_NUM_7,
		.reset = GPIO_NUM_8,
		.te = GPIO_NUM_13,
	};

	static constexpr Ft3168TouchConfig touch{
		.reset = GPIO_NUM_9,
		.interrupt = GPIO_NUM_38,
	};

	static constexpr Es8311AudioConfig audio{
		// I2S_NUM_AUTO ("any free controller"), spelled as its value so the
		// board description does not have to include driver/i2s_types.h --
		// which would make every soundless app carry esp_driver_i2s to
		// describe pins it never drives. es8311.cpp, the only consumer,
		// static_asserts the two agree where that header is in scope.
		.i2sPort = -1,
		.mclk = GPIO_NUM_16,
		.bclk = GPIO_NUM_41,
		.ws = GPIO_NUM_45,
		.dout = GPIO_NUM_40,
		.din = GPIO_NUM_42,
		.powerAmplifier = GPIO_NUM_46,
	};
};

using Board = Esp32S3TouchAmoled206;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr SdMmcConfig storage{
	.clk = GPIO_NUM_2,
	.cmd = GPIO_NUM_1,
	.data0 = GPIO_NUM_3,
};
inline constexpr LauncherButtonConfig launcherButton{
	.pin = GPIO_NUM_0,
	.activeLevel = 0,
};

}  // namespace gea::platform::board
