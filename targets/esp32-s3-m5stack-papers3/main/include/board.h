#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// M5Stack PaperS3 (M5PaperS3) — ESP32-S3R8, 16 MB flash / 8 MB octal PSRAM,
// 4.7" 960x540 grayscale e-paper (ED047TC1 class) wired DIRECTLY to the SoC.
//
// Unlike the original M5Paper there is no IT8951 controller: the panel is
// driven by our components/papers3_epd (i80 parallel bus + gate/source driver
// timing). Those panel pins live in that component, not here. This header
// carries only the peripherals the shared gea target sources consume.
//
// Every pin below is taken from upstream PaperBoy (gitlab.com/zephray/paperboy)
// running on this exact board — main/msg/msg.h, main/main.c, main/touch.c and
// main/battery.c — so they are hardware-verified rather than inferred from docs.

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
	gpio_num_t power;  // launcher/back button
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

class M5PaperS3 {
public:
	// GT911 touch bus (PaperBoy touch.c: SDA 41, SCL 42, responds at 0x5D).
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_41,
		.scl = GPIO_NUM_42,
	};

	// No reset line is driven; the controller is released by the panel rails.
	static constexpr Gt911TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_48,
	};

	// This board has NO user GPIO button: power/reset/download are owned by a
	// PMS150G companion MCU, not by the ESP32. Launcher and back are reached
	// through touch.
	//
	// ⛔ Do NOT "borrow" GPIO0 here. It is the boot strapping pin AND an RTC
	// pad, so its pad configuration survives a core reset — binding it as a
	// button (the inherited launcher_button.cpp disables the pull-up) leaves it
	// floating low at the next reset and the ROM comes up in DOWNLOAD mode
	// instead of running the app. That wedged this board once already; only a
	// full power cycle clears the RTC pad state.
	//
	// GPIO_NUM_NC means "absent" — buttons.cpp and launcher_button.cpp skip all
	// GPIO configuration when they see it.
	static constexpr ButtonConfig buttons{
		.power = GPIO_NUM_NC,
	};

	// Battery divider on GPIO3 = ADC1_CH2 (PaperBoy battery.c). ADC1, so unlike
	// the LilyGo T5's ADC2 divider it stays readable while Wi-Fi is up.
	static constexpr PowerRailConfig power{
		.batteryAdc = GPIO_NUM_3,
	};

	// microSD on its own SPI bus — the panel is parallel, not SPI.
	static constexpr SdSpiStorageConfig storage{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_47,
		.sclk = GPIO_NUM_39,
		.mosi = GPIO_NUM_38,
		.miso = GPIO_NUM_40,
	};
};

using Board = M5PaperS3;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto touch = Board::touch;
inline constexpr auto buttons = Board::buttons;
inline constexpr auto power = Board::power;
inline constexpr auto storage = Board::storage;

}  // namespace gea::platform::board
