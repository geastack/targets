#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct Tab5DisplayConfig {
	gpio_num_t reset;
	gpio_num_t backlight;
};

struct Tab5TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
};

struct Tab5AudioConfig {
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
};

struct Tab5CameraConfig {
	gpio_num_t mclk;
	gpio_num_t sda;
	gpio_num_t scl;
	gpio_num_t reset;
};

struct Tab5MicroSdConfig {
	gpio_num_t clk;
	gpio_num_t cmd;
	gpio_num_t dat0;
	gpio_num_t dat1;
	gpio_num_t dat2;
	gpio_num_t dat3;
};

struct Tab5Rs485Config {
	gpio_num_t rx;
	gpio_num_t tx;
	gpio_num_t direction;
};

class Esp32P4M5StackTab5 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_31,
		.scl = GPIO_NUM_32,
	};

	static constexpr Tab5DisplayConfig display{
		.reset = GPIO_NUM_NC,
		.backlight = GPIO_NUM_22,
	};

	static constexpr Tab5TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_23,
	};

	static constexpr Tab5AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_30,
		.bclk = GPIO_NUM_27,
		.ws = GPIO_NUM_29,
		.dout = GPIO_NUM_26,
		.din = GPIO_NUM_28,
		.powerAmplifier = GPIO_NUM_NC,
	};

	static constexpr Tab5CameraConfig camera{
		.mclk = GPIO_NUM_36,
		.sda = GPIO_NUM_31,
		.scl = GPIO_NUM_32,
		.reset = GPIO_NUM_NC,
	};

	static constexpr Tab5MicroSdConfig microSd{
		.clk = GPIO_NUM_43,
		.cmd = GPIO_NUM_44,
		.dat0 = GPIO_NUM_39,
		.dat1 = GPIO_NUM_40,
		.dat2 = GPIO_NUM_41,
		.dat3 = GPIO_NUM_42,
	};

	static constexpr Tab5Rs485Config rs485{
		.rx = GPIO_NUM_21,
		.tx = GPIO_NUM_20,
		.direction = GPIO_NUM_34,
	};

	static constexpr const char *mainSoc = "ESP32-P4NRW32";
	static constexpr const char *wirelessSoc = "ESP32-C6-MINI-1U";
	static constexpr const char *displayController = "ST7123/ST7121 or ILI9881C";
	static constexpr const char *touchController = "ST7123/ST7121 integrated touch or GT911";
	static constexpr const char *cameraSensor = "SC2356";
	static constexpr const char *audioCodec = "ES8388";
	static constexpr const char *micCodec = "ES7210";
	static constexpr const char *speakerAmplifier = "NS4150B";
	static constexpr const char *imu = "BMI270";
	static constexpr const char *rtc = "RX8130CE";
	static constexpr const char *powerMonitor = "INA226";
	static constexpr const char *charger = "IP2326";
	static constexpr const char *ioExpander = "PI4IOE5V6408";
	static constexpr const char *rs485Transceiver = "SIT3088";
};

using Board = Esp32P4M5StackTab5;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr auto camera = Board::camera;
inline constexpr auto microSd = Board::microSd;
inline constexpr auto rs485 = Board::rs485;

}  // namespace gea::platform::board
