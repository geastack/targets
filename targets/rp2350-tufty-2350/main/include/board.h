#pragma once

#include <array>
#include <cstdint>

#include "rtc/pcf85063/pcf85063.h"

namespace gea::platform::board {

struct I2cBusConfig {
	int bus;
	int sda;
	int scl;
	std::uint32_t frequencyHz;
};

struct St7789ParallelDisplayConfig {
	int pio;
	int stateMachine;
	int cs;
	int dc;
	int wr;
	int rd;
	int data0;
	int backlight;
	int vsync;
	int width;
	int height;
};

struct Pcf85063RtcConfig {
	std::uint8_t address;
	std::uint32_t frequencyHz;
};

struct PowerConfig {
	int powerKey;
	int powerHold;
	int batteryAdc;
	int batteryAdcChannel;
};

struct StorageConfig {
	std::uint32_t flashBytes;
	std::uint32_t psramBytes;
};

struct ButtonConfig {
	int pin;
	int keyCode;
	bool activeLow;
	bool pullUp;
	// When true, the button auto-repeats its keydown while held (typematic
	// repeat) instead of firing once per press — used for the scroll buttons.
	bool repeat = false;
};

class PimoroniTufty2350 {
public:
	static constexpr int kKeyCodeButtonA = 65;
	static constexpr int kKeyCodeButtonB = 66;
	static constexpr int kKeyCodeButtonC = 67;
	static constexpr int kKeyCodeArrowUp = 38;
	static constexpr int kKeyCodeArrowDown = 40;
	static constexpr int kKeyCodeEscape = 27;

	static constexpr I2cBusConfig i2c{
	    .bus = 0,
	    .sda = 4,
	    .scl = 5,
	    .frequencyHz = 400000,
	};

	static constexpr St7789ParallelDisplayConfig display{
	    .pio = 1,
	    .stateMachine = 0,
	    .cs = 27,
	    .dc = 28,
	    .wr = 30,
	    .rd = 31,
	    .data0 = 32,
	    .backlight = 26,
	    .vsync = 21,
	    .width = 320,
	    .height = 240,
	};

	static constexpr Pcf85063RtcConfig rtc{
	    .address = gea::chips::pcf85063::kI2cAddress,
	    .frequencyHz = gea::chips::pcf85063::kI2cFrequencyHz,
	};

	static constexpr PowerConfig power{
	    .powerKey = 14,
	    .powerHold = 41,
	    .batteryAdc = 40,
	    .batteryAdcChannel = 0,
	};

	static constexpr StorageConfig storage{
	    .flashBytes = 16U * 1024U * 1024U,
	    .psramBytes = 8U * 1024U * 1024U,
	};

	static constexpr std::array<ButtonConfig, 6> buttons{{
	    {.pin = 6, .keyCode = kKeyCodeArrowDown, .activeLow = true, .pullUp = true, .repeat = true},
	    {.pin = 7, .keyCode = kKeyCodeButtonA, .activeLow = true, .pullUp = true},
	    {.pin = 9, .keyCode = kKeyCodeButtonB, .activeLow = true, .pullUp = true},
	    {.pin = 10, .keyCode = kKeyCodeButtonC, .activeLow = true, .pullUp = true},
	    {.pin = 11, .keyCode = kKeyCodeArrowUp, .activeLow = true, .pullUp = true, .repeat = true},
	    {.pin = 22, .keyCode = kKeyCodeEscape, .activeLow = true, .pullUp = true},
	}};
};

using Board = PimoroniTufty2350;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto rtc = Board::rtc;
inline constexpr auto power = Board::power;
inline constexpr auto storage = Board::storage;
inline constexpr auto buttons = Board::buttons;

}  // namespace gea::platform::board
