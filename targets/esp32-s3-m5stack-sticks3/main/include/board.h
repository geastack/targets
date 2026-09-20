#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"
#include "driver/spi_master.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

// M5Stack StickS3: ST7789V2 1.14" 135x240 IPS panel on a standard 1-data-line
// SPI bus (SPI2), 8-bit command/parameter, DC + CS + RST GPIOs, PWM backlight.
// The panel GRAM is 240x320; the glass sits at column offset 52, row offset 40
// (see display.cpp).
struct St7789DisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t mosi;
	gpio_num_t sclk;
	gpio_num_t cs;
	gpio_num_t dc;
	gpio_num_t reset;
	gpio_num_t backlight;
};

// Two user buttons on the face (KEY1/KEY2), active low. The side power button
// is wired to the M5PM1 PMIC, not a GPIO.
struct ButtonConfig {
	gpio_num_t key1;
	gpio_num_t key2;
};

// ES8311 mono codec + AW8737 amplifier. The amplifier enable is driven by the
// M5PM1 PMIC (PYG3 "speaker pulse"), not an ESP GPIO, so powerAmplifier is NC
// here; enabling the speaker requires PMIC support (not wired up yet).
struct Es8311AudioConfig {
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
};

class M5StackStickS3 {
public:
	// Shared bus: M5PM1 PMIC (0x6e), BMI270 IMU (0x68).
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_47,
		.scl = GPIO_NUM_48,
	};

	static constexpr St7789DisplayConfig display{
		.spiHost = SPI2_HOST,
		.mosi = GPIO_NUM_39,
		.sclk = GPIO_NUM_40,
		.cs = GPIO_NUM_41,
		.dc = GPIO_NUM_45,
		.reset = GPIO_NUM_21,
		.backlight = GPIO_NUM_38,
	};

	static constexpr ButtonConfig buttons{
		.key1 = GPIO_NUM_11,
		.key2 = GPIO_NUM_12,
	};

	static constexpr Es8311AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_18,
		.bclk = GPIO_NUM_17,
		.ws = GPIO_NUM_15,
		.dout = GPIO_NUM_14,
		.din = GPIO_NUM_16,
		.powerAmplifier = GPIO_NUM_NC,
	};
};

using Board = M5StackStickS3;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto buttons = Board::buttons;
inline constexpr auto audio = Board::audio;

}  // namespace gea::platform::board
