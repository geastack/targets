// gea's Display on the STM32F746G-Discovery: the canvas draws into the LTDC
// back buffer in SDRAM, and a flush has DMA2D copy the dirty rectangles into
// the front buffer the LTDC scans. The CPU never reads the front buffer, so
// screenshots and the presented snapshot come from the back buffer, which
// always holds the last complete frame after a flush.

#include "display.h"

#include "canvas.h"
#include "clock.h"
#include "lcd.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

using namespace gea::stm32;

namespace {

using Pixel = gea::framework::graphics::pixel::native_t;

constexpr int kWidth = gea::platform::display::kWidth;
constexpr int kHeight = gea::platform::display::kHeight;
constexpr std::size_t kPixelCount = static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);

static_assert(kWidth == lcd::kWidth && kHeight == lcd::kHeight, "GEA_EMBEDDED_DISPLAY_* must match the panel");

gea::framework::graphics::Canvas gCanvas;
bool gReady = false;
std::uint8_t gAlpha = 255;
int gBrightness = 100;
bool gVSync = false;

std::int64_t gFlushTotalUs = 0;
int gFlushCalls = 0;
int gFlushPixels = 0;
std::uint32_t gOdometerCalls = 0;
std::uint64_t gOdometerPixels = 0;

Pixel *backBuffer() {
	return reinterpret_cast<Pixel *>(lcd::backBuffer());
}

void presentRect(int x0, int y0, int x1, int y1) {
	x0 = std::max(x0, 0);
	y0 = std::max(y0, 0);
	x1 = std::min(x1, kWidth - 1);
	y1 = std::min(y1, kHeight - 1);
	if (x0 > x1 || y0 > y1) return;

	const int w = x1 - x0 + 1;
	const int h = y1 - y0 + 1;
	lcd::present(x0, y0, w, h);
	gFlushPixels += w * h;
	gOdometerPixels += static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
}

// Every flush entry point funnels through here: one optional wait for the
// panel's vertical blanking, then the rectangles, then the bookkeeping.
template <typename EachRect>
void flushTimed(EachRect eachRect) {
	const std::uint64_t started = clock::micros();
	if (gVSync) lcd::waitForBlanking();

	eachRect();
	gCanvas.resetDirty();
	gFlushCalls++;
	gOdometerCalls++;
	gFlushTotalUs += static_cast<std::int64_t>(clock::micros() - started);
}

}  // namespace

namespace gea::platform::display {

bool Display::init() {
	if (gReady) return true;

	lcd::init();
	gCanvas.bindPixels(backBuffer(), kWidth, kHeight);
	gCanvas.resetClip();
	gCanvas.setGlobalAlpha(255);
	gReady = true;
	return true;
}

bool Display::start() { return init(); }

gea::framework::graphics::Canvas *Display::canvas() { return &gCanvas; }

// The canvas surface is not the scanned buffer: a flush has to copy it there.
bool Display::framebufferIsPanelDirect() { return false; }
bool Display::panelScanoutSurface(uint16_t **, int *, int *) { return false; }
bool Display::panelDirectTarget(int, int, int, int, uint16_t **, int *, int *, int *, int *, int *, int *, int *, int *) { return false; }
void Display::flipPanelToBack() {}
void Display::flipPanelToBack(uint16_t *) {}

bool Display::copySnapshotRgb565(uint16_t *dst, int pixel_capacity, int *width, int *height, bool) {
	if (!dst || !gReady || pixel_capacity < static_cast<int>(kPixelCount)) return false;

	const Pixel *source = backBuffer();
	for (std::size_t i = 0; i < kPixelCount; ++i) dst[i] = gea::framework::graphics::pixel::toRgb565(source[i]);

	if (width) *width = kWidth;
	if (height) *height = kHeight;
	return true;
}

bool Display::copySnapshotPacked(uint8_t *, int, int *, int *, bool) { return false; }

int Display::countNonBlackPixels(bool) {
	if (!gReady) return 0;

	const Pixel *source = backBuffer();
	return static_cast<int>(std::count_if(source, source + kPixelCount, [](Pixel pixel) { return pixel != 0; }));
}

void Display::clear() {
	gCanvas.clear(0);
	flush();
}

void Display::clearNoFlush() { gCanvas.clear(0); }

// Diagnostics go to the serial console, not the panel.
void Display::print(const char *) {}

void Display::flush() {
	int x0 = 0;
	int y0 = 0;
	int x1 = -1;
	int y1 = -1;
	if (!gCanvas.dirty(&x0, &y0, &x1, &y1) || x0 > x1 || y0 > y1) return;

	flushTimed([&] { presentRect(x0, y0, x1, y1); });
}

void Display::flushRects(const DisplayFlushRect *rects, int count, bool) {
	if (!rects || count <= 0) return;

	flushTimed([&] {
		for (int i = 0; i < count; ++i) presentRect(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1);
	});
}

// The fused path overlaps rasterization with a panel transfer. Here the whole
// back buffer is addressable, so rasterize straight into it and present.
void Display::flushRectsRasterized(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, bool) {
	if (!rects || count <= 0 || !raster) return;

	for (int i = 0; i < count; ++i) {
		streamRect(rects[i].x0, rects[i].y0, rects[i].x1 - rects[i].x0 + 1, rects[i].y1 - rects[i].y0 + 1, raster, user);
	}
	rebindCanvasToFramebuffer();
}

void Display::rebindCanvasToFramebuffer() {
	if (gReady) gCanvas.bindPixels(backBuffer(), kWidth, kHeight);
}

bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) {
	if (!raster || w <= 0 || h <= 0 || !gReady) return false;

	const int x0 = std::max(x, 0);
	const int y0 = std::max(y, 0);
	const int x1 = std::min(x + w - 1, kWidth - 1);
	const int y1 = std::min(y + h - 1, kHeight - 1);
	if (x0 > x1 || y0 > y1) return true;

	gea::framework::graphics::Canvas region;
	region.bindPixels(backBuffer() + static_cast<std::size_t>(y0) * kWidth + x0, x1 - x0 + 1, y1 - y0 + 1, kWidth);
	raster(region.pixels(), x1 - x0 + 1, y1 - y0 + 1, x0, y0, user);
	flushTimed([&] { presentRect(x0, y0, x1, y1); });
	return true;
}

bool Display::present(const DisplayPresentCommand *commands, int command_count) {
	if (!commands || command_count <= 0) return false;

	for (int i = 0; i < command_count; ++i) {
		const auto &command = commands[i];
		switch (command.type) {
			case DisplayPresentCommandType::Clear:
				gCanvas.clear(command.clear.color);
				break;
			case DisplayPresentCommandType::FillRectRgb565:
				gCanvas.setGlobalAlpha(command.fillRectRgb565.alpha);
				gCanvas.fillRect(command.fillRectRgb565.x, command.fillRectRgb565.y, command.fillRectRgb565.w, command.fillRectRgb565.h,
				                 command.fillRectRgb565.color);
				break;
			case DisplayPresentCommandType::StrokeRectRgb565:
				gCanvas.setGlobalAlpha(command.strokeRectRgb565.alpha);
				gCanvas.strokeRect(command.strokeRectRgb565.x, command.strokeRectRgb565.y, command.strokeRectRgb565.w, command.strokeRectRgb565.h,
				                   command.strokeRectRgb565.color);
				break;
			case DisplayPresentCommandType::FillTriangleRgb565:
				gCanvas.setGlobalAlpha(command.fillTriangleRgb565.alpha);
				gCanvas.fillTriangle(command.fillTriangleRgb565.x0, command.fillTriangleRgb565.y0, command.fillTriangleRgb565.x1,
				                     command.fillTriangleRgb565.y1, command.fillTriangleRgb565.x2, command.fillTriangleRgb565.y2,
				                     command.fillTriangleRgb565.color);
				break;
			case DisplayPresentCommandType::FillTrianglesRgb565:
				gCanvas.setGlobalAlpha(command.fillTrianglesRgb565.alpha);
				gCanvas.fillTrianglesOpaqueOccluded(command.fillTrianglesRgb565.entries, command.fillTrianglesRgb565.count, 0, 0);
				break;
			case DisplayPresentCommandType::FillCircleRgb565:
				gCanvas.setGlobalAlpha(command.fillCircleRgb565.alpha);
				gCanvas.fillCircle(command.fillCircleRgb565.x, command.fillCircleRgb565.y, command.fillCircleRgb565.radius,
				                   command.fillCircleRgb565.color);
				break;
			case DisplayPresentCommandType::StrokeCircleRgb565:
				gCanvas.setGlobalAlpha(command.strokeCircleRgb565.alpha);
				gCanvas.strokeCircle(command.strokeCircleRgb565.x, command.strokeCircleRgb565.y, command.strokeCircleRgb565.radius,
				                     command.strokeCircleRgb565.color);
				break;
			case DisplayPresentCommandType::FillCirclesRgb565:
				gCanvas.setGlobalAlpha(command.fillCirclesRgb565.alpha);
				gCanvas.fillCirclesRgb565(command.fillCirclesRgb565.xs, command.fillCirclesRgb565.ys, command.fillCirclesRgb565.count,
				                          command.fillCirclesRgb565.radius, command.fillCirclesRgb565.colors);
				break;
			case DisplayPresentCommandType::DrawImage:
				gCanvas.setGlobalAlpha(command.drawImage.alpha);
				gCanvas.drawImage(command.drawImage.pixels, command.drawImage.alphaPixels, command.drawImage.srcWidth, command.drawImage.srcHeight,
				                  command.drawImage.x, command.drawImage.y);
				break;
			case DisplayPresentCommandType::DrawImageScaled:
				gCanvas.setGlobalAlpha(command.drawImageScaled.alpha);
				gCanvas.drawImage(command.drawImageScaled.pixels, command.drawImageScaled.alphaPixels, command.drawImageScaled.srcWidth,
				                  command.drawImageScaled.srcHeight, command.drawImageScaled.x, command.drawImageScaled.y, command.drawImageScaled.w,
				                  command.drawImageScaled.h);
				break;
			case DisplayPresentCommandType::DrawImageRotated90CW:
				gCanvas.setGlobalAlpha(command.drawImageRotated90CW.alpha);
				gCanvas.drawImageRotated90CW(command.drawImageRotated90CW.pixels, command.drawImageRotated90CW.alphaPixels,
				                             command.drawImageRotated90CW.srcWidth, command.drawImageRotated90CW.srcHeight, command.drawImageRotated90CW.x,
				                             command.drawImageRotated90CW.y, command.drawImageRotated90CW.w, command.drawImageRotated90CW.h);
				break;
			case DisplayPresentCommandType::DrawImageTiledX:
				gCanvas.setGlobalAlpha(command.drawImageTiledX.alpha);
				gCanvas.drawImageTiledX(command.drawImageTiledX.pixels, command.drawImageTiledX.alphaPixels, command.drawImageTiledX.srcWidth,
				                        command.drawImageTiledX.srcHeight, command.drawImageTiledX.x, command.drawImageTiledX.y, command.drawImageTiledX.w);
				break;
			case DisplayPresentCommandType::FillText:
				// The baked font when the context set one, as display_present.h does.
				gCanvas.setGlobalAlpha(command.fillText.alpha);
				if (command.fillText.fontFamilyId >= 0)
					gCanvas.drawTextFontFamily(command.fillText.text, command.fillText.x, command.fillText.y, command.fillText.color,
					                           command.fillText.fontFamilyId, command.fillText.fontSizePx);
				else
					gCanvas.drawText(command.fillText.text, command.fillText.x, command.fillText.y, command.fillText.color, command.fillText.scale);
				break;
		}
	}
	gCanvas.setGlobalAlpha(255);
	flush();
	return true;
}

// There is no chunked transfer pipeline to tune; the getters describe the one
// full-frame copy so callers sizing their own buffers get a sane answer.
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

void Display::setAlpha(uint8_t alpha) {
	gAlpha = alpha;
	gCanvas.setGlobalAlpha(alpha);
}
uint8_t Display::alpha() { return gAlpha; }

// The backlight is on/off only; dimming blends the layer toward the LTDC's
// black background instead.
int Display::brightness() { return gBrightness; }
void Display::setBrightness(int brightness_percent) {
	gBrightness = std::clamp(brightness_percent, 0, 100);
	lcd::setBacklight(gBrightness > 0);
	lcd::setLayerAlpha(static_cast<std::uint8_t>(gBrightness * 255 / 100));
}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }

// VSync makes every flush start in the panel's vertical blanking.
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
void Display::drawTextFontFamily(const char *text, int x, int y, Pixel color, int family_id, int size_px) {
	gCanvas.drawTextFontFamily(text, x, y, color, family_id, size_px);
}
void Display::setPixel(int x, int y, Pixel color) { gCanvas.fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, Pixel color) {
	gCanvas.fillRoundedRect(x, y, w, h, tl, tr, br, bl, color);
}
void Display::fillRoundedRectBoxesRgb565(const int16_t *xs, const int16_t *ys, int count, int w, int h, int tl, int tr, int br, int bl, const Pixel *colors) {
	gCanvas.fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lw, Pixel color) {
	gCanvas.strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lw, color);
}
void Display::blitImage(const Pixel *src, const uint8_t *alpha, int src_w, int src_h, int dx, int dy) {
	gCanvas.drawImage(src, alpha, src_w, src_h, dx, dy);
}
void Display::blitImageScaled(const Pixel *src, const uint8_t *alpha, int src_w, int src_h, int dx, int dy, int dst_w, int dst_h) {
	gCanvas.drawImage(src, alpha, src_w, src_h, dx, dy, dst_w, dst_h);
}

// The world overlay reprograms a panel scanout window; the LTDC layer here is
// fixed full-screen.
void Display::setWorldOverlay(const uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}

void Display::flushStatsRead(int64_t *total_us, int *call_count, int *pixel_count) {
	if (total_us) *total_us = gFlushTotalUs;
	if (call_count) *call_count = gFlushCalls;
	if (pixel_count) *pixel_count = gFlushPixels;
}

DisplayFlushPerfStats Display::flushPerfStatsRead() {
	DisplayFlushPerfStats stats;
	stats.totalUs = gFlushTotalUs;
	stats.copyUs = gFlushTotalUs;
	stats.callCount = gFlushCalls;
	stats.pixelCount = gFlushPixels;
	return stats;
}

void Display::flushOdometerRead(uint32_t &calls, uint64_t &pixels) {
	calls = gOdometerCalls;
	pixels = gOdometerPixels;
}

void Display::flushStatsReset() {
	gFlushTotalUs = 0;
	gFlushCalls = 0;
	gFlushPixels = 0;
}

// Diagnostics for present paths this backend does not have: zero, not
// invented numbers.
void Display::presentPathDebug(int *calls, int *direct, int *general, int *rejected, int *tileShapeFailKind, int *tileShapeFailType,
                               int *tileShapeFailCount) {
	for (int *out : {calls, direct, general, rejected, tileShapeFailKind, tileShapeFailType, tileShapeFailCount}) {
		if (out) *out = 0;
	}
}

void Display::landFrameDebug(int *total, int *align, int *raster, int *text, int *flip) {
	for (int *out : {total, align, raster, text, flip}) {
		if (out) *out = 0;
	}
}

void Display::landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior) {
	for (int *out : {detect, kick, strips, wait, interior}) {
		if (out) *out = 0;
	}
}

const char *Display::flushStageName() { return "idle"; }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return DisplayFlushStageDetail{}; }

}  // namespace gea::platform::display
