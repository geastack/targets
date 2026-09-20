#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

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
	gpio_num_t te; // VBlank/tearing-effect line; GPIO_NUM_NC = vsync disabled
};

struct Cst9217TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
	bool swapXy;
	bool mirrorX;
	bool mirrorY;
};

struct Lc76gGpsConfig {
	uart_port_t uartPort;
	gpio_num_t tx;
	gpio_num_t rx;
	gpio_num_t reset;
	int baud;
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

class Esp32S3TouchAmoled175 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_15,
		.scl = GPIO_NUM_14,
	};

	static constexpr Co5300DisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_12,
		.pclk = GPIO_NUM_38,
		.data0 = GPIO_NUM_4,
		.data1 = GPIO_NUM_5,
		.data2 = GPIO_NUM_6,
		.data3 = GPIO_NUM_7,
		.reset = GPIO_NUM_39,
		.te = GPIO_NUM_NC,
	};

	static constexpr Cst9217TouchConfig touch{
		.reset = GPIO_NUM_40,
		.interrupt = GPIO_NUM_11,
		.swapXy = true,
		.mirrorX = false,
		.mirrorY = true,
	};

	static constexpr Es8311AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_42,
		.bclk = GPIO_NUM_9,
		.ws = GPIO_NUM_45,
		.dout = GPIO_NUM_8,
		.din = GPIO_NUM_10,
		.powerAmplifier = GPIO_NUM_46,
	};

	static constexpr Lc76gGpsConfig gps{
		.uartPort = UART_NUM_1,
		.tx = GPIO_NUM_18,
		.rx = GPIO_NUM_17,
		.reset = GPIO_NUM_16,
		.baud = 9600,
	};
};

using Board = Esp32S3TouchAmoled175;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr auto gps = Board::gps;

}  // namespace gea::platform::board
