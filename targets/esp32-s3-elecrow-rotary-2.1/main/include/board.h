#pragma once

#include "driver/gpio.h"

#include <cstdint>

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct St7701RgbDisplayConfig {
	gpio_num_t spiCs;
	gpio_num_t spiSck;
	gpio_num_t spiSda;
	gpio_num_t de;
	gpio_num_t vsync;
	gpio_num_t hsync;
	gpio_num_t pclk;
	gpio_num_t r0;
	gpio_num_t r1;
	gpio_num_t r2;
	gpio_num_t r3;
	gpio_num_t r4;
	gpio_num_t g0;
	gpio_num_t g1;
	gpio_num_t g2;
	gpio_num_t g3;
	gpio_num_t g4;
	gpio_num_t g5;
	gpio_num_t b0;
	gpio_num_t b1;
	gpio_num_t b2;
	gpio_num_t b3;
	gpio_num_t b4;
	gpio_num_t backlight;
	int pclkHz;
	int hsyncFrontPorch;
	int hsyncPulseWidth;
	int hsyncBackPorch;
	int vsyncFrontPorch;
	int vsyncPulseWidth;
	int vsyncBackPorch;
	bool pclkActiveNeg;
	bool bgr;
};

struct Pcf8574Config {
	std::uint8_t address;
	int touchResetBit;
	int touchInterruptBit;
	int lcdPowerBit;
	int lcdResetBit;
	int encoderButtonBit;
};

struct Cst8xxTouchConfig {
	std::uint8_t address;
	int maxX;
	int maxY;
};

struct RotaryEncoderConfig {
	gpio_num_t phaseA;
	gpio_num_t phaseB;
};

class Esp32S3ElecrowRotary21 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_38,
		.scl = GPIO_NUM_39,
	};

	static constexpr St7701RgbDisplayConfig display{
		.spiCs = GPIO_NUM_16,
		.spiSck = GPIO_NUM_2,
		.spiSda = GPIO_NUM_1,
		.de = GPIO_NUM_40,
		.vsync = GPIO_NUM_7,
		.hsync = GPIO_NUM_15,
		.pclk = GPIO_NUM_41,
		.r0 = GPIO_NUM_46,
		.r1 = GPIO_NUM_3,
		.r2 = GPIO_NUM_8,
		.r3 = GPIO_NUM_18,
		.r4 = GPIO_NUM_17,
		.g0 = GPIO_NUM_14,
		.g1 = GPIO_NUM_13,
		.g2 = GPIO_NUM_12,
		.g3 = GPIO_NUM_11,
		.g4 = GPIO_NUM_10,
		.g5 = GPIO_NUM_9,
		.b0 = GPIO_NUM_5,
		.b1 = GPIO_NUM_45,
		.b2 = GPIO_NUM_48,
		.b3 = GPIO_NUM_47,
		.b4 = GPIO_NUM_21,
		.backlight = GPIO_NUM_6,
		.pclkHz = 12 * 1000 * 1000,
		.hsyncFrontPorch = 10,
		.hsyncPulseWidth = 4,
		.hsyncBackPorch = 20,
		.vsyncFrontPorch = 10,
		.vsyncPulseWidth = 4,
		.vsyncBackPorch = 20,
		.pclkActiveNeg = false,
		.bgr = true,
	};

	static constexpr Pcf8574Config expander{
		.address = 0x21,
		.touchResetBit = 0,
		.touchInterruptBit = 2,
		.lcdPowerBit = 3,
		.lcdResetBit = 4,
		.encoderButtonBit = 5,
	};

	static constexpr Cst8xxTouchConfig touch{
		.address = 0x15,
		.maxX = 480,
		.maxY = 480,
	};

	static constexpr RotaryEncoderConfig rotary{
		.phaseA = GPIO_NUM_42,
		.phaseB = GPIO_NUM_4,
	};
};

using Board = Esp32S3ElecrowRotary21;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto expander = Board::expander;
inline constexpr auto touch = Board::touch;
inline constexpr auto rotary = Board::rotary;

}  // namespace gea::platform::board
