#pragma once

#include <cstdint>

#include "displays/rm690b0/rm690b0.h"
#include "power/eta6098/eta6098.h"
#include "rtc/pcf85063/pcf85063.h"
#include "touch/ft6336/ft6336.h"
#include "imu/qmi8658/qmi8658.h"

namespace gea::platform::board {

struct I2cBusConfig {
	int bus;
	int sda;
	int scl;
	std::uint32_t frequencyHz;
};

struct Rm690b0DisplayConfig {
	int pio;
	int dataStateMachine;
	int commandStateMachine;
	int cs;
	int sclk;
	int data0;
	int data1;
	int data2;
	int data3;
	int powerEnable;
	int reset;
	int width;
	int height;
	int panelXGap;
	int panelYGap;
};

struct Ft6336TouchConfig {
	int reset;
	int interrupt;
	std::uint8_t address;
	std::uint32_t frequencyHz;
};

struct Qmi8658ImuConfig {
	int interrupt;
	std::uint8_t address;
	std::uint32_t frequencyHz;
};

struct Pcf85063RtcConfig {
	std::uint8_t address;
	std::uint32_t frequencyHz;
};

struct Eta6098PowerConfig {
	int powerKey;
	int powerHold;
	int batteryAdc;
	int batteryAdcChannel;
	gea::chips::eta6098::BatteryAdcConfig battery;
};

struct StorageConfig {
	std::uint32_t flashBytes;
	std::uint32_t psramBytes;
};

class WaveshareRp2350TouchAmoled241 {
public:
	static constexpr I2cBusConfig i2c{
	    .bus = 1,
	    .sda = 6,
	    .scl = 7,
	    .frequencyHz = 400000,
	};

	static constexpr Rm690b0DisplayConfig display{
	    .pio = 0,
	    .dataStateMachine = 0,
	    .commandStateMachine = 1,
	    .cs = 9,
	    .sclk = 10,
	    .data0 = 11,
	    .data1 = 12,
	    .data2 = 13,
	    .data3 = 14,
	    .powerEnable = -1,
	    .reset = 15,
	    .width = gea::chips::rm690b0::kWidth,
	    .height = gea::chips::rm690b0::kHeight,
	    .panelXGap = gea::chips::rm690b0::kPanelXGap,
	    .panelYGap = gea::chips::rm690b0::kPanelYGap,
	};

	static constexpr Ft6336TouchConfig touch{
	    .reset = 5,
	    .interrupt = 4,
	    .address = gea::chips::ft6336::kI2cAddress,
	    .frequencyHz = gea::chips::ft6336::kI2cFrequencyHz,
	};

	static constexpr Qmi8658ImuConfig imu{
	    .interrupt = 8,
	    .address = gea::chips::qmi8658::kI2cAddress,
	    .frequencyHz = gea::chips::qmi8658::kI2cFrequencyHz,
	};

	static constexpr Pcf85063RtcConfig rtc{
	    .address = gea::chips::pcf85063::kI2cAddress,
	    .frequencyHz = gea::chips::pcf85063::kI2cFrequencyHz,
	};

	static constexpr Eta6098PowerConfig power{
	    .powerKey = 24,
	    .powerHold = 25,
	    .batteryAdc = 40,
	    .batteryAdcChannel = 0,
	    .battery = {},
	};

	static constexpr StorageConfig storage{
	    .flashBytes = 16U * 1024U * 1024U,
	    .psramBytes = 2U * 1024U * 1024U,
	};
};

using Board = WaveshareRp2350TouchAmoled241;

inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto imu = Board::imu;
inline constexpr auto rtc = Board::rtc;
inline constexpr auto power = Board::power;
inline constexpr auto storage = Board::storage;

}  // namespace gea::platform::board
