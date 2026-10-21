#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// Waveshare ESP32-S3-Touch-LCD-3.49: a 172x640 IPS bar on an AXS15231B QSPI
// controller. Pin assignments: the vendor's ESP-IDF examples
// (Examples/ESP-IDF/*/main/user_config.h and codec_board/board_cfg.txt,
// board S3_LCD_3_49).
//
// The board has two I2C buses. The touch half of the AXS15231B answers at 0x3b
// on its own pair (GPIO17/18); everything else -- the QMI8658 IMU, PCF85063
// RTC, ES8311 DAC, ES7210 microphone ADC and TCA9554 expander -- shares the
// GPIO47/48 bus, which is the runtime's primary bus.
namespace gea::platform::board {

struct I2cBusConfig {
	gpio_num_t sda;
	gpio_num_t scl;
};

struct QspiDisplayConfig {
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

struct TouchConfig {
	I2cBusConfig bus;
	gpio_num_t reset;
	// The TCA9554's open-drain INT, which falls when any expander input --
	// the touch controller's INT among them -- changes.
	gpio_num_t interrupt;
	// The expander pin the touch controller's own INT lands on.
	int expanderInterruptPin;
};

struct Es8311AudioConfig {
	int i2sPort;
	gpio_num_t mclk;
	gpio_num_t bclk;
	gpio_num_t ws;
	gpio_num_t dout;
	gpio_num_t din;
	gpio_num_t powerAmplifier;
	// The four microphones are on an ES7210 ADC (I2C 0x40) sharing the I2S
	// bus; the ES8311's own input is not wired to a microphone.
	int es7210Address;
};

// TCA9554 pin numbers (0-based, the chip's own), -1 where a line is not
// routed through it.
struct IoExpanderConfig {
	int initialOutputs;
	int address;
	int initialDirections;
	int powerAmplifier;
};

struct SdMmcConfig {
	gpio_num_t clk;
	gpio_num_t cmd;
	gpio_num_t data0;
};

struct BatteryConfig {
	// VBAT through a 1:3 divider into ADC1.
	gpio_num_t sense;
};

struct LauncherButtonConfig {
	gpio_num_t pin;
	int activeLevel;
};

class Esp32S3TouchLcd349 {
public:
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_47,
		.scl = GPIO_NUM_48,
	};

	// V2 board. Backlight PWM is GPIO42, active low, gated by the expander's
	// BL_EN (GEA_EMBEDDED_AXS15231B_BACKLIGHT_* in main/CMakeLists.txt). The
	// panel reset is the expander's LCD_RST, so no GPIO reset here; GPIO21 is
	// the tearing-effect line, which the runtime does not use.
	static constexpr QspiDisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_9,
		.pclk = GPIO_NUM_10,
		.data0 = GPIO_NUM_11,
		.data1 = GPIO_NUM_12,
		.data2 = GPIO_NUM_13,
		.data3 = GPIO_NUM_14,
		.reset = GPIO_NUM_NC,
		.te = GPIO_NUM_NC,
	};

	// Touch reset is not routed. Touch INT reaches the ESP32 through the
	// expander: EXIO0, signalled on the expander's INT at GPIO8.
	static constexpr TouchConfig touch{
		.bus = {.sda = GPIO_NUM_17, .scl = GPIO_NUM_18},
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_8,
		.expanderInterruptPin = 0,
	};

	static constexpr Es8311AudioConfig audio{
		// I2S_NUM_AUTO, spelled as its value; see the AMOLED 2.06 board.h.
		.i2sPort = -1,
		.mclk = GPIO_NUM_7,
		.bclk = GPIO_NUM_15,
		.ws = GPIO_NUM_46,
		.dout = GPIO_NUM_45,
		.din = GPIO_NUM_6,
		// The speaker amplifier is enabled through the TCA9554 (expander below).
		.powerAmplifier = GPIO_NUM_NC,
		.es7210Address = 0x40,
	};
};

using Board = Esp32S3TouchLcd349;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
// TCA9554 at 0x20 on the primary bus (V2, waveshareteam
// ESP32-S3-Touch-LCD-3.49-V2 user_config.h): EXIO0 touch INT, EXIO1 BL_EN,
// EXIO2/3 IMU INT1/2, EXIO4 RTC INT, EXIO5 LCD_RST, EXIO6 SYS_EN, EXIO7 the
// speaker amplifier. BL_EN, LCD_RST and the amplifier are outputs; BL_EN and
// LCD_RST start high so the panel stays lit and out of reset, the amplifier
// starts off and follows the speaker.
inline constexpr IoExpanderConfig expander{
	.initialOutputs = 0x22,
	.address = 0x20,
	.initialDirections = 0x5D,
	.powerAmplifier = 7,
};
// TF slot, SDMMC 1-bit (vendor Examples/ESP-IDF/04_SD_Card).
inline constexpr SdMmcConfig storage{
	.clk = GPIO_NUM_41,
	.cmd = GPIO_NUM_39,
	.data0 = GPIO_NUM_40,
};
inline constexpr BatteryConfig battery{.sense = GPIO_NUM_4};
// BOOT. The app-facing buttons (BOOT and PWR on GPIO16) are declared in
// targets.json.
inline constexpr LauncherButtonConfig launcherButton{
	.pin = GPIO_NUM_0,
	.activeLevel = 0,
};

}  // namespace gea::platform::board
