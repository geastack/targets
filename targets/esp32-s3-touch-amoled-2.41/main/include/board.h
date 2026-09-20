#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// Waveshare ESP32-S3-Touch-AMOLED-2.41, hardware V1.
//
// V1 and V2 are not interchangeable: they differ in which signals are native
// GPIOs and which sit behind the board's TCA9554 I2C expander. On V1 the panel
// reset is GPIO21 and the touch interrupt is on the expander; V2 swaps both.
// These are V1's values. Getting this wrong is silent: with GPIO21 configured
// as the tearing-effect input the panel never leaves reset, every QSPI write
// still reports success, and the screen simply stays dark.
//
// The panel IC is an RM69080 driven through the SH8601 command set -- the
// vendor's example calls esp_lcd_new_panel_sh8601() for it, because the two
// QSPI init sequences are compatible.

namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct Sh8601DisplayConfig {
	spi_host_device_t spiHost;
	gpio_num_t cs;
	gpio_num_t pclk;
	gpio_num_t data0;
	gpio_num_t data1;
	gpio_num_t data2;
	gpio_num_t data3;
	// Panel reset is wired to expander pin EXIO0 rather than to a native GPIO,
	// so the panel driver gets GPIO_NUM_NC and relies on power-on reset. The
	// vendor's own example passes GPIO_NUM_NC here too and pulses the expander
	// line separately before panel init.
	gpio_num_t reset;
	// Tearing-Effect line from the panel. The runtime waits for its VBlank edge
	// before flushing so a QSPI write into GRAM never overlaps scanout.
	gpio_num_t te;
};

struct Ft6336TouchConfig {
	// Touch reset is on expander pin EXIO1, likewise unreachable as a GPIO.
	gpio_num_t reset;
	gpio_num_t interrupt;
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

class Esp32S3TouchAmoled241 {
public:
	// One bus carries the touch controller, the PCF85063 RTC, the QMI8658 IMU
	// and the TCA9554 expander.
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_47,
		.scl = GPIO_NUM_48,
	};

	static constexpr Sh8601DisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_9,
		.pclk = GPIO_NUM_10,
		.data0 = GPIO_NUM_11,
		.data1 = GPIO_NUM_12,
		.data2 = GPIO_NUM_13,
		.data3 = GPIO_NUM_14,
		.reset = GPIO_NUM_21,
		.te = GPIO_NUM_NC,
	};

	// V1 wires TP_RESET to GPIO3 and leaves TP_INT on expander pin EXIO2, which
	// is not reachable as a GPIO -- so the controller is polled rather than
	// interrupt-driven.
	static constexpr Ft6336TouchConfig touch{
		.reset = GPIO_NUM_3,
		.interrupt = GPIO_NUM_NC,
	};
};

using Board = Esp32S3TouchAmoled241;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
// The TF slot is wired for SDMMC 1-bit as well as SPI; the runtime uses the
// SDMMC pins.
inline constexpr SdMmcConfig storage{
	.clk = GPIO_NUM_4,
	.cmd = GPIO_NUM_5,
	.data0 = GPIO_NUM_6,
};
inline constexpr LauncherButtonConfig launcherButton{
	.pin = GPIO_NUM_0,
	.activeLevel = 0,
};

}  // namespace gea::platform::board
