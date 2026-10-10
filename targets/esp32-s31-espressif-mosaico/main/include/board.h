#pragma once

#include "driver/gpio.h"
#include "driver/i2s_types.h"
#include "driver/spi_master.h"

// Espressif ESP-Mosaico, hardware V1.2 (V1.1 shares the mapping).
//
// ESP32-S31 (dual-core RISC-V, 320 MHz) with 16 MB in-package octal PSRAM,
// 16 MB quad NOR flash, a 480x480 CO5300 QSPI AMOLED with a CST9220 touch
// controller, ES8311 codec + NS4150B amplifier, BMI270 IMU, two BMM150
// magnetometers, a BQ27220 fuel gauge and 128 MB SPI NAND.
//
// Pins come from the board's own BSP (esp-mosaico-bsp, bsp/esp_mosaico.h) and
// the esp-dev-kits V1.2 user guide. V1.0 is NOT interchangeable: it swaps the
// LCD clock and reset (GPIO44/42) and puts the onboard I2C on GPIO0/1. Getting
// the swap wrong is silent -- every QSPI write reports success into a panel
// still held in reset.
//
// Everything on the panel side sits behind VCC_3V3, a high-side switch whose
// enable (GPIO60) is ACTIVE LOW. Nothing on that rail answers until the board
// power code has opened it; see board_power.cpp.

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
	gpio_num_t te;  // VBlank/tearing-effect line; GPIO_NUM_NC = vsync disabled
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
};

struct PowerConfig {
	// VCC_3V3 high-side switch enable, active low.
	gpio_num_t vcc3v3Enable;
	// Power-off request to the SAM8108 controller, open drain, asserted low.
	// Released (high impedance) for as long as the board should stay on.
	gpio_num_t powerOffRequest;
};

// The 1 Gbit SPI NAND (GD5F1GM7UEYIGR) on SPI3, pins per the esp-dev-kits V1.2
// user guide. main/nand_mount.cpp mounts it as the board's file storage.
struct NandConfig {
	spi_host_device_t spiHost;
	gpio_num_t clk;
	gpio_num_t mosi;
	gpio_num_t miso;
	gpio_num_t cs;
	gpio_num_t wp;
	gpio_num_t hold;
};

struct LauncherButtonConfig {
	gpio_num_t pin;
	int activeLevel;
};

class EspMosaicoV12 {
public:
	// Onboard bus: touch (0x5A), ES8311 (0x19), BMI270 (0x69), BMM150 (0x11,
	// 0x12) and BQ27220 (0x55). The expansion slots have their own bus on
	// GPIO0/GPIO1, which nothing here drives.
	static constexpr I2cBusConfig i2c{
		.sda = GPIO_NUM_56,
		.scl = GPIO_NUM_3,
	};

	// SPI3 carries the NAND; SPI2 is the panel's.
	static constexpr Co5300DisplayConfig display{
		.spiHost = SPI2_HOST,
		.cs = GPIO_NUM_50,
		.pclk = GPIO_NUM_42,
		.data0 = GPIO_NUM_36,
		.data1 = GPIO_NUM_51,
		.data2 = GPIO_NUM_35,
		.data3 = GPIO_NUM_9,
		.reset = GPIO_NUM_44,
		// The panel drives TE on GPIO43 (TEON in the init sequence). An app that
		// calls Display.setVSync(true) frames off it, locked to the ~60 Hz scanout.
		.te = GPIO_NUM_43,
	};

	// Touch reset is not wired; the controller comes out of power-on reset
	// with the VCC_3V3 rail. The panel and touch share orientation, and the
	// BSP applies no base swap or mirror to either.
	static constexpr Cst9217TouchConfig touch{
		.reset = GPIO_NUM_NC,
		.interrupt = GPIO_NUM_6,
		.swapXy = false,
		.mirrorX = false,
		.mirrorY = false,
	};

	// The codec's I2S pins double as right-slot expansion pins; a module
	// there and audio cannot be used at the same time.
	static constexpr Es8311AudioConfig audio{
		.i2sPort = I2S_NUM_AUTO,
		.mclk = GPIO_NUM_54,
		.bclk = GPIO_NUM_37,
		.ws = GPIO_NUM_49,
		.dout = GPIO_NUM_52,
		.din = GPIO_NUM_40,
		.powerAmplifier = GPIO_NUM_45,
	};

	static constexpr NandConfig nand{
		.spiHost = SPI3_HOST,
		.clk = GPIO_NUM_20,
		.mosi = GPIO_NUM_21,
		.miso = GPIO_NUM_22,
		.cs = GPIO_NUM_23,
		.wp = GPIO_NUM_25,
		.hold = GPIO_NUM_24,
	};

	static constexpr PowerConfig power{
		.vcc3v3Enable = GPIO_NUM_60,
		.powerOffRequest = GPIO_NUM_57,
	};
};

using Board = EspMosaicoV12;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto audio = Board::audio;
inline constexpr auto power = Board::power;
inline constexpr auto nand = Board::nand;

// The "AI" button on the side, active low. BOOT (GPIO61) stays the ROM's
// download strap and is left alone.
inline constexpr LauncherButtonConfig launcherButton{
	.pin = GPIO_NUM_7,
	.activeLevel = 0,
};

// Opens VCC_3V3 (soft-started) and releases the power-off request. Safe to
// call more than once; display init calls it through prepareDisplayPanel().
void powerOnRails();

}  // namespace gea::platform::board
