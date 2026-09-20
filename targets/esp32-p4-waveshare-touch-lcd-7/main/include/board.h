#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct WaveshareP4DisplayConfig {
	gpio_num_t reset;
	gpio_num_t backlight;
};

struct Gt911TouchConfig {
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

class Esp32P4WaveshareTouchLcd7 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_7,
		.scl = GPIO_NUM_8,
	};

	static constexpr WaveshareP4DisplayConfig display{
		.reset = GPIO_NUM_27,
		.backlight = GPIO_NUM_26,
	};

	static constexpr Gt911TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_NC,
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

using Board = Esp32P4WaveshareTouchLcd7;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;

}  // namespace gea::platform::board
