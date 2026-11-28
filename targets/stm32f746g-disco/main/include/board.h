#pragma once

#include <cstdint>

// STM32F746G-Discovery (MB1191). Values follow ST's STM32CubeF7 BSP for this
// board; the clock runs at 200 MHz, the fastest ST validates with the SDRAM.
namespace gea::stm32::board {

inline constexpr std::uint32_t kHseHz = 25'000'000;
inline constexpr std::uint32_t kSysClockHz = 200'000'000;
inline constexpr std::uint32_t kApb1Hz = kSysClockHz / 4;
inline constexpr std::uint32_t kApb2Hz = kSysClockHz / 2;
inline constexpr std::uint32_t kApb1TimerHz = kApb1Hz * 2;

inline constexpr std::uint32_t kSdramBase = 0xC0000000;
inline constexpr std::uint32_t kSdramBytes = 8U << 20;

// RK043FN48H 4.3" RGB panel on the LTDC, 9.6 MHz pixel clock.
struct DisplayConfig {
	int width = 480;
	int height = 272;
	int hsync = 41;
	int hbp = 13;
	int hfp = 32;
	int vsync = 10;
	int vbp = 2;
	int vfp = 2;
};
inline constexpr DisplayConfig kDisplay{};

// FT5336 on I2C3 (PH7 SCL, PH8 SDA), shared with the WM8994 codec. The
// controller's X axis is the panel's short edge.
struct TouchConfig {
	std::uint32_t i2cTiming = 0x5033050C;  // 400 kHz fast mode from a 50 MHz PCLK1
	bool swapXY = true;
};
inline constexpr TouchConfig kTouch{};

// ST-LINK/V2-1 virtual COM port on USART1 (PA9 TX, PB7 RX).
inline constexpr std::uint32_t kConsoleBaud = 115200;

}  // namespace gea::stm32::board
