// Headless display backend: the board has no panel.
//
// gea's Display is canvas-backed on every target — the panel driver is what
// sits *after* the canvas, not under it. That is why the native test host runs
// the whole framework, apps and all, with no display hardware at all. This file
// is that same arrangement for an ESP32 board: a real canvas in PSRAM, every
// drawing call rasterized into it, and a flush that transmits nothing.
//
// So a headless board is not a degraded board. Timers, input, Wi-Fi, BLE, the
// reactive tree and the app all behave exactly as they do on a panel target; the
// pixels simply stay in RAM, where a screenshot over OTA can still read them.
//
// Selected by composing a board with no `chips.display` — the CLI then emits
// GEA_EMBEDDED_DISPLAY_HEADLESS=1 and compiles this instead of display.cpp plus
// a panel driver. GEA_EMBEDDED_DISPLAY_WIDTH/HEIGHT come from the definition's
// `canvas` block rather than from a panel's geometry.

#include "display.h"

#include "canvas.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char *kTag = "display.headless";

using Pixel = gea::framework::graphics::pixel::native_t;

gea::framework::graphics::Canvas gCanvas;
Pixel *gPixels = nullptr;
Pixel *gPresented = nullptr;
std::uint8_t gAlpha = 255;
int gBrightness = 100;
bool gVSync = false;

std::int64_t gFlushTotalUs = 0;
int gFlushCalls = 0;
int gFlushPixels = 0;
std::uint32_t gOdometerCalls = 0;
std::uint64_t gOdometerPixels = 0;

constexpr int kWidth = gea::platform::display::kWidth;
constexpr int kHeight = gea::platform::display::kHeight;
constexpr std::size_t kPixelCount = static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);

// The offscreen surface is the only allocation a headless board makes for
// display, and it is deliberately PSRAM: internal DRAM on an S3 is the scarce
// pool that Wi-Fi, BLE and DMA compete for, and nothing here is ever DMA'd.
Pixel *allocateSurface()
{
	void *buffer = heap_caps_calloc(kPixelCount, sizeof(Pixel), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!buffer) buffer = heap_caps_calloc(kPixelCount, sizeof(Pixel), MALLOC_CAP_8BIT);
	return static_cast<Pixel *>(buffer);
}

// Mirror the drawn region into the "presented" copy. A panel target would be
// transmitting these rows over QSPI; here the copy is what makes
// copySnapshotRgb565(presented=true) answer with the frame the app finished,
// rather than one caught mid-draw.
void promote(int x0, int y0, int x1, int y1)
{
	if (!gPixels || !gPresented) return;
	x0 = std::max(x0, 0);
	y0 = std::max(y0, 0);
	x1 = std::min(x1, kWidth - 1);
	y1 = std::min(y1, kHeight - 1);
	if (x0 > x1 || y0 > y1) return;
	const std::size_t run = static_cast<std::size_t>(x1 - x0 + 1);
	for (int y = y0; y <= y1; ++y) {
		const std::size_t offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(kWidth) + static_cast<std::size_t>(x0);
		std::memcpy(gPresented + offset, gPixels + offset, run * sizeof(Pixel));
	}
	gFlushPixels += static_cast<int>(run) * (y1 - y0 + 1);
	gOdometerPixels += static_cast<std::uint64_t>(run) * static_cast<std::uint64_t>(y1 - y0 + 1);
}

}  // namespace

namespace gea::platform::display {

bool Display::init()
{
	if (gPixels) return true;
	gPixels = allocateSurface();
	gPresented = allocateSurface();
	if (!gPixels || !gPresented) {
		ESP_LOGE(kTag, "offscreen surface allocation failed (%dx%d)", kWidth, kHeight);
		return false;
	}
	gCanvas.bindPixels(gPixels, kWidth, kHeight);
	gCanvas.resetClip();
	gCanvas.setGlobalAlpha(255);
	ESP_LOGI(kTag, "headless display: %dx%d offscreen canvas, no panel attached", kWidth, kHeight);
	return true;
}

bool Display::start() { return gPixels != nullptr; }

gea::framework::graphics::Canvas *Display::canvas() { return &gCanvas; }

// No panel means no scanout buffer and no rotation to one: every query about
// panel-direct rendering is simply false here, and the UI composes offscreen.
bool Display::framebufferIsPanelDirect() { return false; }
bool Display::panelScanoutSurface(uint16_t **, int *, int *) { return false; }
bool Display::panelDirectTarget(int, int, int, int, uint16_t **, int *, int *, int *, int *, int *, int *, int *, int *) { return false; }
void Display::flipPanelToBack() {}
void Display::flipPanelToBack(uint16_t *) {}

bool Display::copySnapshotRgb565(uint16_t *dst, int pixel_capacity, int *width, int *height, bool presented)
{
	if (!dst || !gPixels) return false;
	if (pixel_capacity < static_cast<int>(kPixelCount)) return false;
	const Pixel *source = presented ? gPresented : gPixels;
	// native_t on this target is RGB565 stored BYTE-SWAPPED, because that is the
	// order the panel wants on the wire — canvas.h's own helpers call
	// fromRgb565() "undo any panel byte-swap". The snapshot contract is plain
	// RGB565, so the swap has to be undone here exactly as the panel backend
	// does it. A raw memcpy compiles, runs, and returns an image with every hue
	// wrong: red and blue swapped and green shifted.
	for (std::size_t i = 0; i < kPixelCount; ++i) {
		dst[i] = gea::framework::graphics::pixel::toRgb565(source[i]);
	}
	if (width) *width = kWidth;
	if (height) *height = kHeight;
	return true;
}

bool Display::copySnapshotPacked(uint8_t *, int, int *, int *, bool) { return false; }

int Display::countNonBlackPixels(bool presented)
{
	const Pixel *source = presented ? gPresented : gPixels;
	if (!source) return 0;
	int count = 0;
	for (std::size_t i = 0; i < kPixelCount; ++i) {
		if (source[i] != 0) count++;
	}
	return count;
}

void Display::clear()
{
	gCanvas.clear(0);
	flush();
}

void Display::clearNoFlush() { gCanvas.clear(0); }

// There is no console to print to: a headless board's diagnostics go out the
// serial/OTA log, which the caller already has.
void Display::print(const char *) {}

void Display::flush()
{
	const std::int64_t started = esp_timer_get_time();
	gFlushCalls++;
	gOdometerCalls++;
	int x0 = 0;
	int y0 = 0;
	int x1 = -1;
	int y1 = -1;
	if (gCanvas.dirty(&x0, &y0, &x1, &y1) && x0 <= x1 && y0 <= y1) {
		promote(x0, y0, x1, y1);
		gCanvas.resetDirty();
	}
	gFlushTotalUs += esp_timer_get_time() - started;
}

void Display::flushRects(const DisplayFlushRect *rects, int count, bool)
{
	if (!rects || count <= 0) return;
	const std::int64_t started = esp_timer_get_time();
	gFlushCalls++;
	gOdometerCalls++;
	for (int i = 0; i < count; ++i) promote(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1);
	gCanvas.resetDirty();
	gFlushTotalUs += esp_timer_get_time() - started;
}

// The fused path exists to overlap rasterization with DMA. With no DMA to
// overlap, rasterize straight into the surface and promote it — same pixels,
// none of the chunking.
void Display::flushRectsRasterized(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, bool)
{
	if (!rects || count <= 0 || !raster) return;
	for (int i = 0; i < count; ++i) {
		streamRect(rects[i].x0, rects[i].y0, rects[i].x1 - rects[i].x0 + 1, rects[i].y1 - rects[i].y0 + 1, raster, user);
	}
	rebindCanvasToFramebuffer();
}

void Display::rebindCanvasToFramebuffer()
{
	if (!gPixels) return;
	gCanvas.bindPixels(gPixels, kWidth, kHeight);
}

bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user)
{
	if (!raster || w <= 0 || h <= 0 || !gPixels) return false;
	int x0 = std::max(x, 0);
	int y0 = std::max(y, 0);
	int x1 = std::min(x + w - 1, kWidth - 1);
	int y1 = std::min(y + h - 1, kHeight - 1);
	if (x0 > x1 || y0 > y1) return true;

	const int width = x1 - x0 + 1;
	const int height = y1 - y0 + 1;
	// Raster straight into the surface rows: the destination is already a
	// contiguous, correctly sized region, so no staging buffer is needed.
	gea::framework::graphics::Canvas region;
	region.bindPixels(gPixels + static_cast<std::size_t>(y0) * static_cast<std::size_t>(kWidth) + static_cast<std::size_t>(x0),
	                  width, height, kWidth);
	raster(region.pixels(), width, height, x0, y0, user);
	promote(x0, y0, x1, y1);
	gFlushCalls++;
	gOdometerCalls++;
	gCanvas.resetDirty();
	return true;
}

bool Display::present(const DisplayPresentCommand *commands, int command_count)
{
	if (!commands || command_count <= 0) return false;
	for (int i = 0; i < command_count; ++i) {
		const auto &command = commands[i];
		switch (command.type) {
			case DisplayPresentCommandType::Clear:
				gCanvas.clear(command.clear.color);
				break;
			case DisplayPresentCommandType::FillRectRgb565:
				gCanvas.setGlobalAlpha(command.fillRectRgb565.alpha);
				gCanvas.fillRect(command.fillRectRgb565.x, command.fillRectRgb565.y,
				                 command.fillRectRgb565.w, command.fillRectRgb565.h,
				                 command.fillRectRgb565.color);
				break;
			case DisplayPresentCommandType::StrokeRectRgb565:
				gCanvas.setGlobalAlpha(command.strokeRectRgb565.alpha);
				gCanvas.strokeRect(command.strokeRectRgb565.x, command.strokeRectRgb565.y,
				                   command.strokeRectRgb565.w, command.strokeRectRgb565.h,
				                   command.strokeRectRgb565.color);
				break;
			case DisplayPresentCommandType::FillTriangleRgb565:
				gCanvas.setGlobalAlpha(command.fillTriangleRgb565.alpha);
				gCanvas.fillTriangle(command.fillTriangleRgb565.x0, command.fillTriangleRgb565.y0,
				                     command.fillTriangleRgb565.x1, command.fillTriangleRgb565.y1,
				                     command.fillTriangleRgb565.x2, command.fillTriangleRgb565.y2,
				                     command.fillTriangleRgb565.color);
				break;
			case DisplayPresentCommandType::FillTrianglesRgb565:
				gCanvas.setGlobalAlpha(command.fillTrianglesRgb565.alpha);
				gCanvas.fillTrianglesOpaqueOccluded(command.fillTrianglesRgb565.entries,
				                                    command.fillTrianglesRgb565.count, 0, 0);
				break;
			case DisplayPresentCommandType::FillCircleRgb565:
				gCanvas.setGlobalAlpha(command.fillCircleRgb565.alpha);
				gCanvas.fillCircle(command.fillCircleRgb565.x, command.fillCircleRgb565.y,
				                   command.fillCircleRgb565.radius, command.fillCircleRgb565.color);
				break;
			case DisplayPresentCommandType::StrokeCircleRgb565:
				gCanvas.setGlobalAlpha(command.strokeCircleRgb565.alpha);
				gCanvas.strokeCircle(command.strokeCircleRgb565.x, command.strokeCircleRgb565.y,
				                     command.strokeCircleRgb565.radius, command.strokeCircleRgb565.color);
				break;
			case DisplayPresentCommandType::FillCirclesRgb565:
				gCanvas.setGlobalAlpha(command.fillCirclesRgb565.alpha);
				gCanvas.fillCirclesRgb565(command.fillCirclesRgb565.xs, command.fillCirclesRgb565.ys,
				                          command.fillCirclesRgb565.count, command.fillCirclesRgb565.radius,
				                          command.fillCirclesRgb565.colors);
				break;
			case DisplayPresentCommandType::DrawImage:
				gCanvas.setGlobalAlpha(command.drawImage.alpha);
				gCanvas.drawImage(command.drawImage.pixels, command.drawImage.alphaPixels,
				                  command.drawImage.srcWidth, command.drawImage.srcHeight,
				                  command.drawImage.x, command.drawImage.y);
				break;
			case DisplayPresentCommandType::DrawImageScaled:
				gCanvas.setGlobalAlpha(command.drawImageScaled.alpha);
				gCanvas.drawImage(command.drawImageScaled.pixels, command.drawImageScaled.alphaPixels,
				                  command.drawImageScaled.srcWidth, command.drawImageScaled.srcHeight,
				                  command.drawImageScaled.x, command.drawImageScaled.y,
				                  command.drawImageScaled.w, command.drawImageScaled.h);
				break;
			case DisplayPresentCommandType::DrawImageRotated90CW:
				gCanvas.setGlobalAlpha(command.drawImageRotated90CW.alpha);
				gCanvas.drawImageRotated90CW(command.drawImageRotated90CW.pixels, command.drawImageRotated90CW.alphaPixels,
				                             command.drawImageRotated90CW.srcWidth, command.drawImageRotated90CW.srcHeight,
				                             command.drawImageRotated90CW.x, command.drawImageRotated90CW.y,
				                             command.drawImageRotated90CW.w, command.drawImageRotated90CW.h);
				break;
			case DisplayPresentCommandType::DrawImageTiledX:
				gCanvas.setGlobalAlpha(command.drawImageTiledX.alpha);
				gCanvas.drawImageTiledX(command.drawImageTiledX.pixels, command.drawImageTiledX.alphaPixels,
				                        command.drawImageTiledX.srcWidth, command.drawImageTiledX.srcHeight,
				                        command.drawImageTiledX.x, command.drawImageTiledX.y,
				                        command.drawImageTiledX.w);
				break;
			case DisplayPresentCommandType::FillText:
				gCanvas.setGlobalAlpha(command.fillText.alpha);
				gCanvas.drawText(command.fillText.text, command.fillText.x, command.fillText.y,
				                 command.fillText.color, command.fillText.scale);
				break;
		}
	}
	gCanvas.setGlobalAlpha(255);
	flush();
	return true;
}

// Flush tuning describes a DMA pipeline this board does not have. The getters
// still answer, because callers use them to size their own buffers.
void Display::setFlushConfig(int, int) {}
int Display::flushChunkRows() { return kHeight; }
int Display::flushQueueDepth() { return 1; }
int Display::flushBufferBytes() { return 0; }
void Display::setPresentScale(int) {}
void Display::reserveInternal(std::size_t) {}
void Display::applyPendingInternalReserve() {}

void Display::pushClip(int x, int y, int w, int h) { gCanvas.pushClip(x, y, w, h); }
void Display::popClip() { gCanvas.popClip(); }
void Display::resetClip() { gCanvas.resetClip(); }
void Display::clip(int *x0, int *y0, int *x1, int *y1) { gCanvas.currentClip(x0, y0, x1, y1); }

void Display::setAlpha(uint8_t alpha)
{
	gAlpha = alpha;
	gCanvas.setGlobalAlpha(alpha);
}
uint8_t Display::alpha() { return gAlpha; }
// setAA/aa are NOT defined here: canvas.cpp already owns them for every target,
// since antialiasing is a canvas setting and not a panel one.

// No backlight and no panel controller: brightness is remembered so an app that
// round-trips the value sees what it set, and changes nothing.
int Display::brightness() { return gBrightness; }
void Display::setBrightness(int brightness_percent) { gBrightness = brightness_percent; }
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }

// No TE line to sync to, so vsync is accepted and ignored rather than blocking
// the frame scheduler on a signal that will never arrive.
void Display::setVSync(bool on) { gVSync = on; }
bool Display::vsyncEnabled() { return gVSync; }
void Display::vsyncWaitForFrame() {}
void Display::invalidate() {}

void Display::fillRect(int x, int y, int w, int h, Pixel color) { gCanvas.fillRect(x, y, w, h, color); }
void Display::scrollRect(int x, int y, int w, int h, int dx, int dy) { gCanvas.scrollRect(x, y, w, h, dx, dy); }
void Display::resetScrollRegion() { gCanvas.setScrollRegion(0, 0, 0); }
void Display::strokeRect(int x, int y, int w, int h, Pixel color) { gCanvas.strokeRect(x, y, w, h, color); }
void Display::fillCircle(int cx, int cy, int r, Pixel color) { gCanvas.fillCircle(cx, cy, r, color); }
void Display::strokeCircle(int cx, int cy, int r, Pixel color) { gCanvas.strokeCircle(cx, cy, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, Pixel color) { gCanvas.drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int cx, int cy, int r, int start_deg, int end_deg, Pixel color) { gCanvas.drawArc(cx, cy, r, start_deg, end_deg, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, Pixel color) { gCanvas.fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, Pixel color, float scale) { gCanvas.drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, Pixel color, int font_id) { gCanvas.drawTextFont(text, x, y, color, font_id); }
void Display::drawTextFontFamily(const char *text, int x, int y, Pixel color, int family_id, int size_px)
{
	gCanvas.drawTextFontFamily(text, x, y, color, family_id, size_px);
}
void Display::setPixel(int x, int y, Pixel color) { gCanvas.fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, Pixel color)
{
	gCanvas.fillRoundedRect(x, y, w, h, tl, tr, br, bl, color);
}
void Display::fillRoundedRectBoxesRgb565(const int16_t *xs, const int16_t *ys, int count,
                                         int w, int h, int tl, int tr, int br, int bl,
                                         const Pixel *colors)
{
	gCanvas.fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lw, Pixel color)
{
	gCanvas.strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lw, color);
}
void Display::blitImage(const Pixel *src, const uint8_t *alpha, int src_w, int src_h, int dx, int dy)
{
	gCanvas.drawImage(src, alpha, src_w, src_h, dx, dy);
}
void Display::blitImageScaled(const Pixel *src, const uint8_t *alpha, int src_w, int src_h, int dx, int dy, int dst_w, int dst_h)
{
	gCanvas.drawImage(src, alpha, src_w, src_h, dx, dy, dst_w, dst_h);
}

// The world overlay is a panel-scroll trick (a wide bitmap scrolled under the
// UI by reprogramming the scanout window). Without a panel there is nothing to
// reprogram.
void Display::setWorldOverlay(const uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}

void Display::flushStatsRead(int64_t *total_us, int *call_count, int *pixel_count)
{
	if (total_us) *total_us = gFlushTotalUs;
	if (call_count) *call_count = gFlushCalls;
	if (pixel_count) *pixel_count = gFlushPixels;
}

DisplayFlushPerfStats Display::flushPerfStatsRead()
{
	DisplayFlushPerfStats stats;
	stats.totalUs = gFlushTotalUs;
	stats.copyUs = gFlushTotalUs;
	stats.callCount = gFlushCalls;
	stats.pixelCount = gFlushPixels;
	return stats;
}

void Display::flushOdometerRead(uint32_t &calls, uint64_t &pixels)
{
	calls = gOdometerCalls;
	pixels = gOdometerPixels;
}

void Display::flushStatsReset()
{
	gFlushTotalUs = 0;
	gFlushCalls = 0;
	gFlushPixels = 0;
}

// Diagnostics for paths this backend does not have. Zeroing the outputs is the
// honest answer: GEADEV reads them as counters, and a stale or invented number
// would read as a panel doing work that never happened.
void Display::presentPathDebug(int *calls, int *direct, int *general, int *rejected,
                               int *tileShapeFailKind, int *tileShapeFailType, int *tileShapeFailCount)
{
	if (calls) *calls = 0;
	if (direct) *direct = 0;
	if (general) *general = 0;
	if (rejected) *rejected = 0;
	if (tileShapeFailKind) *tileShapeFailKind = 0;
	if (tileShapeFailType) *tileShapeFailType = 0;
	if (tileShapeFailCount) *tileShapeFailCount = 0;
}

void Display::landFrameDebug(int *total, int *align, int *raster, int *text, int *flip)
{
	if (total) *total = 0;
	if (align) *align = 0;
	if (raster) *raster = 0;
	if (text) *text = 0;
	if (flip) *flip = 0;
}

void Display::landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior)
{
	if (detect) *detect = 0;
	if (kick) *kick = 0;
	if (strips) *strips = 0;
	if (wait) *wait = 0;
	if (interior) *interior = 0;
}

const char *Display::flushStageName() { return "idle"; }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return DisplayFlushStageDetail{}; }

}  // namespace gea::platform::display
