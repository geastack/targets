#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// Waveshare ESP32-S3-Touch-AMOLED-1.75C: the 1.75's 466x466 CO5300 AMOLED with
// a CST9217 touch layer, on a 32 MB board without the GPS module. Pin
// assignments: Waveshare's BSP (Waveshare-ESP32-components,
// bsp/esp32_s3_touch_amoled_1_75c/include/bsp/esp32_s3_touch_amoled_1_75c.h).
//
// It is not the 1.75 rewired with fewer parts: the panel and touch resets moved
// to GPIO1/GPIO2 (the 1.75's TF-card pins -- this board has no card slot) and
// the codec master clock to GPIO16.
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
	gpio_num_t te;
};

struct Cst9217TouchConfig {
	gpio_num_t reset;
	gpio_num_t interrupt;
	bool swapXy;
	bool mirrorX;
	bool mirrorY;
};

struct Es8311AudioConfig {
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
	// The two microphones are on an ES7210 ADC (I2C 0x40) sharing the I2S
	// bus; the ES8311's own input is not wired to a microphone.
	int es7210Address;
};

struct LauncherButtonConfig {
	gpio_num_t pin;
	int activeLevel;
};

class Esp32S3TouchAmoled175C {
public:
	// Touch, AXP2101, QMI8658, PCF85063, ES8311 and ES7210 share one bus.
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
		.reset = GPIO_NUM_1,
		.te = GPIO_NUM_NC,
	};

	// Orientation from the vendor BSP (waveshare/esp32_s3_touch_amoled_1_75c
	// 3.0.0, bsp_display_new_with_handles): no swap, both axes mirrored.
	static constexpr Cst9217TouchConfig touch{
		.reset = GPIO_NUM_2,
		.interrupt = GPIO_NUM_11,
		.swapXy = false,
		.mirrorX = true,
		.mirrorY = true,
	};

	static constexpr Es8311AudioConfig audio{
		// I2S_NUM_AUTO, spelled as its value; see the AMOLED 2.06 board.h.
		.i2sPort = -1,
		.mclk = GPIO_NUM_16,
		.bclk = GPIO_NUM_9,
		.ws = GPIO_NUM_45,
		.dout = GPIO_NUM_8,
		.din = GPIO_NUM_10,
		.powerAmplifier = GPIO_NUM_46,
		.es7210Address = 0x40,
	};
};

using Board = Esp32S3TouchAmoled175C;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr LauncherButtonConfig launcherButton{
	.pin = GPIO_NUM_0,
	.activeLevel = 0,
};

}  // namespace gea::platform::board
