#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

// SSD1681-class 1.54" 200x200 e-paper on SPI2. CS/DC are driven manually
// (the controller interleaves single-byte commands with parameter bytes),
// BUSY is polled high-active.
struct EpaperDisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t dc;
	gpio_num_t sclk;
	gpio_num_t mosi;
	gpio_num_t reset;
	gpio_num_t busy;
};

struct Ft6336TouchConfig {
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

// Switchable power rails. EPD and audio rails are ACTIVE LOW (drive 0 to
// power the peripheral), the battery rail is active high — matches the
// vendor BSP's board_power_bsp.
struct PowerRailConfig {
	gpio_num_t epdPower;    // active low
	gpio_num_t audioPower;  // active low
	gpio_num_t vbatPower;   // active high (battery operation only)
};

struct SdMmcStorageConfig {
	gpio_num_t clk;
	gpio_num_t cmd;
	gpio_num_t d0;
};

class Esp32S3Epaper154 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_47,
		.scl = GPIO_NUM_48,
	};

	static constexpr EpaperDisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_11,
		.dc = GPIO_NUM_10,
		.sclk = GPIO_NUM_12,
		.mosi = GPIO_NUM_13,
		.reset = GPIO_NUM_9,
		.busy = GPIO_NUM_8,
	};

	static constexpr Ft6336TouchConfig touch{
		.reset = GPIO_NUM_7,
		.interrupt = GPIO_NUM_21,
	};

	static constexpr Es8311AudioConfig audio{
		// I2S_NUM_AUTO ("any free controller"), spelled as its value so the
		// board description does not have to include driver/i2s_types.h --
		// which would make every soundless app carry esp_driver_i2s to
		// describe pins it never drives, and fail to compile outright when the
		// active app has no audio and the component is therefore not linked.
		// Mirrors esp32-s3-touch-amoled-2.06, where es8311.cpp static_asserts
		// the two agree where that header is in scope.
		.i2sPort = -1,
		.mclk = GPIO_NUM_14,
		.bclk = GPIO_NUM_15,
		.ws = GPIO_NUM_38,
		.dout = GPIO_NUM_45,
		.din = GPIO_NUM_16,
		.powerAmplifier = GPIO_NUM_46,
	};

	static constexpr PowerRailConfig power{
		.epdPower = GPIO_NUM_6,
		.audioPower = GPIO_NUM_42,
		.vbatPower = GPIO_NUM_17,
	};

	static constexpr SdMmcStorageConfig storage{
		.clk = GPIO_NUM_39,
		.cmd = GPIO_NUM_41,
		.d0 = GPIO_NUM_40,
	};
};

using Board = Esp32S3Epaper154;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr auto power = Board::power;

}  // namespace gea::platform::board
