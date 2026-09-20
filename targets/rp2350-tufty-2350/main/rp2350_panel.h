#pragma once

#include <cstdint>

#ifndef GEA_RP2350_PANEL_SCALE
#define GEA_RP2350_PANEL_SCALE 1
#endif

namespace gea::rp2350 {

inline constexpr int kNativePanelWidth = 320;
inline constexpr int kNativePanelHeight = 240;
inline constexpr int kPanelScale = GEA_RP2350_PANEL_SCALE;
static_assert(kPanelScale >= 1, "GEA_RP2350_PANEL_SCALE must be at least 1");
static_assert(kNativePanelWidth % kPanelScale == 0, "panel scale must divide native width");
static_assert(kNativePanelHeight % kPanelScale == 0, "panel scale must divide native height");
inline constexpr int kPanelWidth = kNativePanelWidth / kPanelScale;
inline constexpr int kPanelHeight = kNativePanelHeight / kPanelScale;

struct PanelFlushStats {
	std::int64_t totalUs = 0;
	std::int64_t setWindowUs = 0;
	std::int64_t copyUs = 0;
	std::int64_t txWaitUs = 0;
	int callCount = 0;
	int chunkCount = 0;
	int pixelCount = 0;
};

using PanelStreamRasterFn = void (*)(std::uint16_t *pixels,
                                     int width,
                                     int height,
                                     int originX,
                                     int originY,
                                     void *user);

bool panelInit();
void panelSetBrightness(int percent);
void panelSetFlushConfig(int rows, int depth);
void panelFlushRect(const std::uint16_t *pixels, int stridePixels, int x0, int y0, int x1, int y1);
void panelStreamRect(int x0, int y0, int x1, int y1, PanelStreamRasterFn raster, void *user);
void panelClear(std::uint16_t color);
PanelFlushStats panelFlushStatsRead();
void panelFlushStatsReset();

}  // namespace gea::rp2350
