#pragma once

#include <cstdint>

namespace gea::rp2350 {

inline constexpr int kPanelWidth = 450;
inline constexpr int kPanelHeight = 600;

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
// DCS vertical scroll (VSCRDEF/VSCSAD): the panel scans out GRAM row
// (startRow + displayRow) % kPanelHeight, so a scroll becomes a 2-byte
// register write instead of a full-GRAM re-stream. startRow 0 = identity.
void panelSetVerticalScroll(int startRow);
void panelFlushRect(const std::uint16_t *pixels, int stridePixels, int x0, int y0, int x1, int y1);
// Streams full-width GRAM rows starting at gramY0 from up to two contiguous
// full-width framebuffer row spans, DMA'd directly from PSRAM (no SRAM chunk
// bounce, no CPU copies). Cleans the XIP cache first so pending CPU writes are
// visible to the DMA read. Used by the software scroll register: a circularly
// remapped full-width flush is exactly two contiguous spans in display order.
void panelFlushFullWidthSpans(const std::uint16_t *span0, int rows0,
                              const std::uint16_t *span1, int rows1,
                              int gramY0);
void panelStreamRect(int x0, int y0, int x1, int y1, PanelStreamRasterFn raster, void *user);
void panelClear(std::uint16_t color);
PanelFlushStats panelFlushStatsRead();
void panelFlushStatsReset();

}  // namespace gea::rp2350
