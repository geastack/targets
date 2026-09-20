#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// LilyGo T5 4.7" E-Paper S3 (ED047TC1, 960x540 16-level grayscale).
//
// The e-paper panel itself is driven by the vendored `epd_driver` component
// (parallel bus + CFG shift register + CKV), whose pins live in that
// component's `ed047tc1.h`/`utilities.h` (D0-D7 = {8,1,2,3,4,5,6,7}, CKH=41,
// STH=40, CKV=38, CFG_DATA=13/CFG_CLK=12/CFG_STR=0). This header only carries
// the peripherals the shared gea target files consume: I2C, touch, the user
// button, the battery ADC, and the microSD SPI bus.

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct Gt911TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

struct ButtonConfig {
	gpio_num_t power;  // single user/BOOT-adjacent button; used for launcher/back
};

struct PowerRailConfig {
	gpio_num_t batteryAdc;
};

struct SdSpiStorageConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t sclk;
	gpio_num_t mosi;
	gpio_num_t miso;
};

class LilygoT5Epaper47 {
public:
	// NOTE: on this board revision GPIO 17/18 are e-paper DATA pins (D5/D6), so
	// the old touch-I2C pins would collide with the display. Touch is deferred;
	// point this at free GPIOs so the (non-fatal) GT911 probe can't fight the
	// panel. Real touch-bus pins are a follow-up.
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_10,
		.scl = GPIO_NUM_13,
	};

	static constexpr Gt911TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_47,
	};

	static constexpr ButtonConfig buttons{
		.power = GPIO_NUM_21,
	};

	static constexpr PowerRailConfig power{
		.batteryAdc = GPIO_NUM_14,  // ADC2 on the S3; battery divider
	};

	// microSD lives on its own SPI bus (the display is parallel, not SPI).
	static constexpr SdSpiStorageConfig storage{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_42,
		.sclk = GPIO_NUM_11,
		.mosi = GPIO_NUM_15,
		.miso = GPIO_NUM_16,
	};
};

using Board = LilygoT5Epaper47;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto touch = Board::touch;
inline constexpr auto buttons = Board::buttons;
inline constexpr auto power = Board::power;
inline constexpr auto storage = Board::storage;

}  // namespace gea::platform::board
