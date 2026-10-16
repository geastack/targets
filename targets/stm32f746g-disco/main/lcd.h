#pragma once

#include <cstdint>

// RK043FN48H on the LTDC. The CPU draws into the back buffer; the LTDC scans
// the front buffer, which only DMA2D writes. Both are RGB565 in SDRAM.
namespace gea::stm32::lcd {

inline constexpr int kWidth = 480;
inline constexpr int kHeight = 272;

void init();
std::uint16_t *backBuffer();
std::uint16_t *frontBuffer();

// Blocks until the beam leaves the active area: ~0.8 ms of blanking follow.
void waitForBlanking();
// Copies a back-buffer rectangle to the front buffer with DMA2D, returning
// before the copy finishes.
void present(int x, int y, int w, int h);
void fill(std::uint16_t *buffer, int x, int y, int w, int h, std::uint16_t color);

void setBacklight(bool on);
// 0..255, applied at the next vertical blanking as layer alpha over black.
void setLayerAlpha(std::uint8_t alpha);

}  // namespace gea::stm32::lcd
