#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "host/display_orientation.h"
#include "pixel.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern "C" {
#include "epd_driver.h"

// From components/epd_driver/gea_epd.c (not part of the vendored epd_driver.h
// API): balanced multipass recovery cycles for a charge-imbalanced panel, and
// a region-masked variant for a localized burn-in.
extern "C" void gea_epd_repair(int cycles, int fields_per_phase);
extern "C" void gea_epd_repair_region(int cycles, int fields_per_phase, int byte_lo, int byte_hi);
extern "C" void gea_epd_wash(int cycles, int fields_per_phase);
extern "C" void gea_epd_recovery_forever(void);
extern "C" void gea_epd_diagnostic_forever(void);
extern "C" void gea_epd_set_vcom_mv(unsigned mv);
extern "C" void gea_epd_enable_experimental(void);
extern "C" void gea_epd_first_light_ritual(void);
extern "C" void gea_epd_draw_full_flash(uint8_t *data);
}

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;

namespace gea::platform::lilygo_t5::display {

namespace {

constexpr char kTag[] = "lilygo_t5_display";

// Physical panel is 960x540 landscape (ED047TC1). Apps (the e-reader) run
// portrait 540x960, so the Canvas framebuffer is portrait and we rotate it into
// the landscape epd framebuffer at flush time (the raw panel has no controller-
// side rotation, unlike the M5Paper's IT8951).
constexpr int kControllerWidth = EPD_WIDTH;    // 960
constexpr int kControllerHeight = EPD_HEIGHT;   // 540
constexpr int kPortraitWidth = 540;
constexpr int kPortraitHeight = 960;
constexpr int kPortraitStride = (kPortraitWidth + 1) / 2;        // 270 bytes/row
constexpr int kPortraitPackedBytes = kPortraitStride * kPortraitHeight;
constexpr int kEpdStride = kControllerWidth / 2;                 // 480 bytes/row
constexpr int kEpdBytes = kEpdStride * kControllerHeight;        // full landscape 4bpp

// ED047TC1 4-bit brightness: 0x0 = black, 0xF = white. gea GRAY4 native values
// use the same polarity (0 = black, 15 = white), so no inversion is needed.
// Flip this if the panel comes up as a negative image.
constexpr bool kInvertGray = false;

// Reset accumulated e-paper ghosting with a full clear+redraw every N frames.
constexpr int kDefaultFullRefreshEvery = 8;

// ---- PANEL SAFETY CONFIG (per-panel; see the safety layer in gea_epd.c) -----
// The first panel was destroyed by sustained saturation DC. Replacement-panel
// protocol, in order:
//   1. Set kPanelVcomMv to the NEW panel's OWN factory calibration (the first
//      panel's was 1560 — NEVER reuse another panel's value). 0 keeps every
//      scan NOP-locked: the build boots but cannot drive the glass.
//   2. Flash with kFirstLightConfirmed=false: only the first-light ritual
//      runs (a gentle dark band across the center). Visually confirm the band
//      DARKENED. If it got lighter, the polarity is inverted — stop.
//   3. Only then set kFirstLightConfirmed=true and reflash for app rendering.
// Recovery/repair/diagnostic waveforms additionally require
// gea_epd_enable_experimental() and hard-stop after 30 minutes.
// CURRENT (first, damaged) panel: 1560 is THIS panel's own factory value and
// its polarity was visually confirmed during bring-up. When the REPLACEMENT
// panel is installed, reset these to 0 / false and follow the protocol above.
constexpr int kPanelVcomMv = 1560;
constexpr bool kFirstLightConfirmed = true;
// Boot straight into the (safety-gated) recovery treatment loop instead of
// app rendering. Recovery sessions hard-stop after 30 minutes (experimental
// wall-clock cap) — reboot or power-cycle to run another session.
constexpr bool kRecoveryOnlyBoot = true;

class DisplayBackend {
public:
	static DisplayBackend &instance()
	{
		static DisplayBackend backend;
		return backend;
	}

	bool init()
	{
		if (initialized_) return true;
		mutex_ = xSemaphoreCreateMutex();
		if (!mutex_) return false;

		frameBuffer_ = static_cast<std::uint8_t *>(
			heap_caps_calloc(kPortraitPackedBytes, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		epdFb_ = static_cast<std::uint8_t *>(
			heap_caps_aligned_calloc(16, 1, kEpdBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!frameBuffer_ || !epdFb_) {
			ESP_LOGE(kTag, "framebuffer allocation failed (PSRAM required)");
			return false;
		}

		// Bring up the ED047TC1 parallel driver behind the safety interlocks.
		epd_init();
		if (kPanelVcomMv <= 0) {
			// No panel calibration configured: refuse to drive, forever. The
			// safety layer NOPs every scan anyway; this loop just makes the
			// state unmissable instead of silently rendering nothing.
			for (;;) {
				ESP_LOGE(kTag, "PANEL LOCKED: set kPanelVcomMv to THIS panel's own factory VCOM (see safety config)");
				vTaskDelay(pdMS_TO_TICKS(60000));
			}
		}
		gea_epd_set_vcom_mv(static_cast<unsigned>(kPanelVcomMv));
		if (!kFirstLightConfirmed) {
			gea_epd_first_light_ritual();
			for (;;) {
				ESP_LOGW(kTag, "FIRST-LIGHT MODE: confirm the center band darkened, then set kFirstLightConfirmed=true");
				vTaskDelay(pdMS_TO_TICKS(60000));
			}
		}
		if (kRecoveryOnlyBoot) {
			ESP_LOGI(kTag, "RECOVERY build (safety-gated): treatment/rest loop, 30-min session cap, reboot for another session");
			gea_epd_enable_experimental();
			gea_epd_recovery_forever();  // parks itself (rails off) at the session cap
		}
		epd_poweron();
		epd_clear();
		epd_poweroff();

		bindCanvas();
		if (xTaskCreatePinnedToCore(refreshTaskEntry, "lilygo_t5_epd", 4096, this, 6, &refreshTask_, 1) != pdPASS) {
			ESP_LOGE(kTag, "failed to start refresh task");
			return false;
		}
		initialized_ = true;
		ESP_LOGI(kTag, "LilyGo T5 ready %dx%d portrait, 16-gray, epd=%d bytes",
		         kPortraitWidth, kPortraitHeight, kEpdBytes);
		return true;
	}

	bool start() { return init(); }

	gea::framework::graphics::Canvas *canvas()
	{
		if (!initialized_) init();
		return &canvas_;
	}

	bool framebufferIsPanelDirect() const { return false; }

	void applyOrientation(gea::framework::display::DisplayOrientation)
	{
		if (!init()) return;
		take();
		bindCanvasLocked();
		std::memset(frameBuffer_, 0, kPortraitPackedBytes);
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		forceFull_.store(true);
		give();
	}

	void clear()
	{
		clearNoFlush();
		flush();
	}

	void clearNoFlush()
	{
		if (!init()) return;
		take();
		std::memset(frameBuffer_, 0, kPortraitPackedBytes);
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		forceFull_.store(true);
		give();
	}

	void flush()
	{
		if (!init()) return;
		int x0 = 0, y0 = 0, x1 = logicalWidth() - 1, y1 = logicalHeight() - 1;
		if (!forceFull_.load() && !canvas_.dirty(&x0, &y0, &x1, &y1)) return;
		canvas_.resetDirty();
		requestRefresh();
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		if (!rects || count <= 0 || !init()) return;
		canvas_.resetDirty();
		requestRefresh();
	}

	bool streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user)
	{
		if (!raster || w <= 0 || h <= 0 || !init()) return false;
		const int x0 = std::max(0, x);
		const int y0 = std::max(0, y);
		const int x1 = std::min(logicalWidth() - 1, x + w - 1);
		const int y1 = std::min(logicalHeight() - 1, y + h - 1);
		if (x0 > x1 || y0 > y1) return true;
		const int width = x1 - x0 + 1;
		static pixel::native_t rowbuf[kPortraitWidth];
		for (int row = y0; row <= y1; ++row) {
			raster(rowbuf, width, 1, x0, row, user);
			pixel::packed::writeUnpacked(frameBuffer_ + static_cast<std::size_t>(row) * kPortraitStride, x0, rowbuf, width);
		}
		canvas_.markDirty(x0, y0, x1, y1);
		flush();
		return true;
	}

	// Accept the direct-canvas present batch and rasterize it straight into the
	// GRAY4 framebuffer (native_t colors are already gray4 at compile time), then
	// refresh. Declining would send these to an off-screen composed buffer the
	// single-framebuffer e-paper backend never blits back — so canvas apps
	// (bouncing-balls' fillCirclesRgb565) would draw nothing.
	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int count)
	{
		using T = platform_display::DisplayPresentCommandType;
		if (!commands || count <= 0 || !init()) return false;
		take();
		// A stale clip from an earlier op would drop everything outside it (the
		// label at 6,23 survives; balls in the corner don't). Draw the whole batch
		// against the full canvas.
		canvas_.resetClip();
		for (int i = 0; i < count; ++i) {
			const auto &c = commands[i];
			switch (c.type) {
			case T::Clear:
				canvas_.setGlobalAlpha(255);
				// e-paper is white paper: clear to white (gray4 15) so content has
				// real contrast, regardless of the app's canvas clear color.
				canvas_.clear(static_cast<pixel::native_t>(15));
				break;
			case T::FillRectRgb565:
				canvas_.setGlobalAlpha(c.fillRectRgb565.alpha);
				canvas_.fillRect(c.fillRectRgb565.x, c.fillRectRgb565.y, c.fillRectRgb565.w, c.fillRectRgb565.h, c.fillRectRgb565.color);
				break;
			case T::StrokeRectRgb565:
				canvas_.setGlobalAlpha(c.strokeRectRgb565.alpha);
				canvas_.strokeRect(c.strokeRectRgb565.x, c.strokeRectRgb565.y, c.strokeRectRgb565.w, c.strokeRectRgb565.h, c.strokeRectRgb565.color);
				break;
			case T::FillTriangleRgb565:
				canvas_.setGlobalAlpha(c.fillTriangleRgb565.alpha);
				canvas_.fillTriangle(c.fillTriangleRgb565.x0, c.fillTriangleRgb565.y0, c.fillTriangleRgb565.x1, c.fillTriangleRgb565.y1,
				                     c.fillTriangleRgb565.x2, c.fillTriangleRgb565.y2, c.fillTriangleRgb565.color);
				break;
			case T::FillCircleRgb565:
				canvas_.setGlobalAlpha(c.fillCircleRgb565.alpha);
				canvas_.fillCircle(c.fillCircleRgb565.x, c.fillCircleRgb565.y, c.fillCircleRgb565.radius, c.fillCircleRgb565.color);
				break;
			case T::StrokeCircleRgb565:
				canvas_.setGlobalAlpha(c.strokeCircleRgb565.alpha);
				canvas_.strokeCircle(c.strokeCircleRgb565.x, c.strokeCircleRgb565.y, c.strokeCircleRgb565.radius, c.strokeCircleRgb565.color);
				break;
			case T::FillCirclesRgb565:
				canvas_.setGlobalAlpha(c.fillCirclesRgb565.alpha);
				canvas_.fillCirclesRgb565(c.fillCirclesRgb565.xs, c.fillCirclesRgb565.ys, c.fillCirclesRgb565.count,
				                          c.fillCirclesRgb565.radius, c.fillCirclesRgb565.colors);
				break;
			case T::FillTrianglesRgb565:
				canvas_.setGlobalAlpha(c.fillTrianglesRgb565.alpha);
				for (int t = 0; t < c.fillTrianglesRgb565.count; ++t) {
					const auto &e = c.fillTrianglesRgb565.entries[t];
					canvas_.fillTriangle(e.x0, e.y0, e.x1, e.y1, e.x2, e.y2, e.color);
				}
				break;
			case T::DrawImage:
				canvas_.setGlobalAlpha(c.drawImage.alpha);
				canvas_.drawImage(c.drawImage.pixels, c.drawImage.alphaPixels, c.drawImage.srcWidth, c.drawImage.srcHeight, c.drawImage.x, c.drawImage.y);
				break;
			case T::DrawImageScaled:
				canvas_.setGlobalAlpha(c.drawImageScaled.alpha);
				canvas_.drawImage(c.drawImageScaled.pixels, c.drawImageScaled.alphaPixels, c.drawImageScaled.srcWidth, c.drawImageScaled.srcHeight,
				                  c.drawImageScaled.x, c.drawImageScaled.y, c.drawImageScaled.w, c.drawImageScaled.h);
				break;
			case T::DrawImageRotated90CW:
				canvas_.setGlobalAlpha(c.drawImageRotated90CW.alpha);
				canvas_.drawImageRotated90CW(c.drawImageRotated90CW.pixels, c.drawImageRotated90CW.alphaPixels, c.drawImageRotated90CW.srcWidth, c.drawImageRotated90CW.srcHeight,
				                             c.drawImageRotated90CW.x, c.drawImageRotated90CW.y, c.drawImageRotated90CW.w, c.drawImageRotated90CW.h);
				break;
			case T::DrawImageTiledX:
				canvas_.setGlobalAlpha(c.drawImageTiledX.alpha);
				canvas_.drawImageTiledX(c.drawImageTiledX.pixels, c.drawImageTiledX.alphaPixels, c.drawImageTiledX.srcWidth, c.drawImageTiledX.srcHeight,
				                        c.drawImageTiledX.x, c.drawImageTiledX.y, c.drawImageTiledX.w);
				break;
			case T::FillText:
				canvas_.setGlobalAlpha(255);
				if (c.fillText.fontFamilyId >= 0)
					canvas_.drawTextFontFamily(c.fillText.text, c.fillText.x, c.fillText.y, c.fillText.color, c.fillText.fontFamilyId, c.fillText.fontSizePx);
				else
					canvas_.drawText(c.fillText.text, c.fillText.x, c.fillText.y, c.fillText.color, c.fillText.scale);
				break;
			}
		}
		canvas_.setGlobalAlpha(255);
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		give();
		requestRefresh();
		return true;
	}

	void setFlushConfig(int, int) {}
	int flushChunkRows() const { return logicalHeight(); }
	int flushQueueDepth() const { return 1; }
	int flushBufferBytes() const { return kEpdBytes; }

	bool copySnapshotRgb565(std::uint16_t *dst, int capacity, int *width, int *height)
	{
		const int pixels = kPortraitWidth * kPortraitHeight;
		if (!dst || capacity < pixels || !frameBuffer_) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		for (int i = 0; i < pixels; ++i)
			dst[i] = pixel::gray4ToRgb565(pixel::packed::get(frameBuffer_, i));
		return true;
	}

	// Packed (native GRAY4) snapshot -- frameBuffer_ is already the exact flat
	// packed pixel stream (kPortraitPackedBytes = kPortraitStride*kPortraitHeight,
	// and kPortraitStride == pixel::packed::rowBytes(kPortraitWidth) since
	// kPortraitWidth is even), so this is a straight copy: no per-pixel unpack/
	// expand to RGB565, a quarter the allocation copySnapshotRgb565 needs.
	bool copySnapshotPacked(std::uint8_t *dst, int byteCapacity, int *width, int *height)
	{
		if (!dst || byteCapacity < kPortraitPackedBytes || !frameBuffer_) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		std::memcpy(dst, frameBuffer_, static_cast<std::size_t>(kPortraitPackedBytes));
		return true;
	}

	int countNonBlackPixels() const
	{
		int count = 0;
		const int pixels = kPortraitWidth * kPortraitHeight;
		for (int i = 0; frameBuffer_ && i < pixels; ++i)
			if (pixel::packed::get(frameBuffer_, i) != 0) ++count;
		return count;
	}

	void setBrightness(int) {}
	int brightness() const { return 100; }
	platform_display::DisplayFlushPerfStats flushStats() const { return stats_; }
	void flushStatsReset() { stats_ = {}; }
	const char *flushStageName() const { return refreshInFlight_.load() ? "epd-refresh" : "idle"; }

	std::uint16_t *backgroundCache(int *capacity) { return scratch(backgroundCache_, backgroundAttempted_, capacity); }
	std::uint16_t *backdropCache(int *capacity) { return scratch(backdropCache_, backdropAttempted_, capacity); }

	void setRefreshConfig(int fullEvery, int hardCap, int)
	{
		if (fullEvery >= 0) fullRefreshEvery_ = fullEvery;
		(void)hardCap;
		ESP_LOGI(kTag, "refresh config fullEvery=%d", fullRefreshEvery_);
	}

	void requestFullRefresh()
	{
		if (!init()) return;
		forceFull_.store(true);
		requestRefresh();
	}

	void setQualityGrayscale(bool enabled) { qualityGrayscale_.store(enabled); }
	void setFullOnCover(bool enabled) { fullOnCover_.store(enabled); }
	void rebindCanvasToFramebuffer() { bindCanvas(); }

private:
	DisplayBackend() = default;

	int logicalWidth() const { return orientation_detail::DisplayOrientationState::width(); }
	int logicalHeight() const { return orientation_detail::DisplayOrientationState::height(); }

	void take() { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
	void give() { if (mutex_) xSemaphoreGive(mutex_); }

	void bindCanvas() { take(); bindCanvasLocked(); give(); }
	void bindCanvasLocked()
	{
		canvas_.bindPixels(frameBuffer_, kPortraitWidth, kPortraitHeight, kPortraitWidth);
	}

	std::uint16_t *scratch(std::uint16_t *&buffer, bool &attempted, int *capacity)
	{
		const int pixels = kPortraitWidth * kPortraitHeight;
		if (!attempted) {
			attempted = true;
			buffer = static_cast<std::uint16_t *>(
				heap_caps_malloc(pixels * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		}
		if (capacity) *capacity = buffer ? pixels : 0;
		return buffer;
	}

	void requestRefresh()
	{
		pendingSeq_.fetch_add(1, std::memory_order_release);
		if (refreshTask_) xTaskNotifyGive(refreshTask_);
	}

	// Rotate the portrait GRAY4 framebuffer (540x960, gea nibble order: even x =
	// high nibble) into the landscape epd framebuffer (960x540, epd nibble order:
	// even x = low nibble). 90 degrees clockwise: physical(px,py) = (ly, 539-lx).
	// The map is a bijection over every physical pixel, so no pre-clear is needed.
	void rotatePortraitToLandscape()
	{
		for (int ly = 0; ly < kPortraitHeight; ++ly) {
			const std::uint8_t *srow = frameBuffer_ + static_cast<std::size_t>(ly) * kPortraitStride;
			const int px = ly;                       // physical x
			const int pxHiNibble = px & 1;           // odd physical x -> high nibble
			std::uint8_t *pcol = epdFb_ + (px >> 1); // column base in landscape buffer
			for (int lx = 0; lx < kPortraitWidth; ++lx) {
				const std::uint8_t b = srow[lx >> 1];
				std::uint8_t v = (lx & 1) ? (b & 0x0F) : (b >> 4);  // gea gray 0..15
				if (kInvertGray) v = static_cast<std::uint8_t>(15 - v);
				const int py = (kPortraitWidth - 1) - lx;           // 539 - lx
				std::uint8_t *d = pcol + static_cast<std::size_t>(py) * kEpdStride;
				*d = pxHiNibble ? static_cast<std::uint8_t>((*d & 0x0F) | (v << 4))
				                : static_cast<std::uint8_t>((*d & 0xF0) | v);
			}
		}
	}

	static void refreshTaskEntry(void *arg) { static_cast<DisplayBackend *>(arg)->refreshLoop(); }

	void refreshLoop()
	{
		std::uint32_t transmitted = 0;
		for (;;) {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			const std::uint32_t seq = pendingSeq_.load(std::memory_order_acquire);
			if (seq == transmitted) continue;
			transmitted = seq;

			refreshInFlight_.store(true);
			const std::int64_t started = esp_timer_get_time();

			take();
			rotatePortraitToLandscape();
			give();

			// STAGE 0 bring-up path: correctness over speed. Each present is a
			// full-panel 16-level grayscale draw (the "quality" half of the
			// eventual dual-mode engine). The fast per-pixel-state 60fps path
			// replaces this branch next.
			const bool full = forceFull_.exchange(false) ||
			                  (fullRefreshEvery_ > 0 && partialsSinceFull_ >= fullRefreshEvery_);
			epd_poweron();
			if (full) {
				// Classic flashing full refresh (shake -> negative -> white ->
				// image): the standard anti-ghosting sequence.
				gea_epd_draw_full_flash(epdFb_);
				partialsSinceFull_ = 0;
			} else {
				epd_draw_grayscale_image(epd_full_screen(), epdFb_);
			}
			epd_poweroff();
			if (!full) ++partialsSinceFull_;

			const std::int64_t elapsed = esp_timer_get_time() - started;
			stats_.callCount++;
			stats_.txUs += elapsed;
			stats_.chunkCount++;
			refreshInFlight_.store(false);
			ESP_LOGI(kTag, "%s present %lldms", full ? "full" : "draw",
			         static_cast<long long>(elapsed / 1000));
		}
	}

	SemaphoreHandle_t mutex_ = nullptr;
	TaskHandle_t refreshTask_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	std::uint8_t *frameBuffer_ = nullptr;  // portrait GRAY4 (Canvas target)
	std::uint8_t *epdFb_ = nullptr;        // landscape 4bpp (epd_draw target)
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	std::atomic<std::uint32_t> pendingSeq_{0};
	std::atomic<bool> refreshInFlight_{false};
	std::atomic<bool> forceFull_{true};
	std::atomic<bool> qualityGrayscale_{true};
	std::atomic<bool> fullOnCover_{false};
	int partialsSinceFull_ = 0;
	int fullRefreshEvery_ = kDefaultFullRefreshEvery;
	bool backgroundAttempted_ = false;
	bool backdropAttempted_ = false;
	bool initialized_ = false;
	platform_display::DisplayFlushPerfStats stats_ = {};
};

}  // namespace

}  // namespace gea::platform::lilygo_t5::display

extern "C" void gea_epaper_set_refresh_config(
	int fullEvery, int hardCap, int fastWindowMs,
	const std::uint8_t *, int, const std::uint8_t *, int)
{
	(void)fastWindowMs;
	gea::platform::lilygo_t5::display::DisplayBackend::instance().setRefreshConfig(fullEvery, hardCap, fastWindowMs);
}

extern "C" void gea_epaper_full_refresh(void)
{
	gea::platform::lilygo_t5::display::DisplayBackend::instance().requestFullRefresh();
}

extern "C" void gea_epaper_set_grayscale(int enabled)
{
	gea::platform::lilygo_t5::display::DisplayBackend::instance().setQualityGrayscale(enabled != 0);
}

extern "C" void gea_epaper_set_full_on_cover(int enabled)
{
	gea::platform::lilygo_t5::display::DisplayBackend::instance().setFullOnCover(enabled != 0);
}

extern "C" std::uint16_t *gea_bg_cache(int *capacity)
{
	return gea::platform::lilygo_t5::display::DisplayBackend::instance().backgroundCache(capacity);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *capacity)
{
	return gea::platform::lilygo_t5::display::DisplayBackend::instance().backdropCache(capacity);
}

namespace gea::platform::display {

namespace {

using Backend = gea::platform::lilygo_t5::display::DisplayBackend;
gea::framework::graphics::Canvas *drawingCanvas() { return Backend::instance().canvas(); }

}  // namespace

bool Display::init() { return Backend::instance().init(); }
bool Display::start() { return Backend::instance().start(); }
gea::framework::graphics::Canvas *Display::canvas() { return Backend::instance().canvas(); }
bool Display::framebufferIsPanelDirect() { return false; }
void Display::clear() { Backend::instance().clear(); }
void Display::clearNoFlush() { Backend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { Backend::instance().flush(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { Backend::instance().flushRects(rects, count); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return Backend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int count) { return Backend::instance().presentCommands(commands, count); }
void Display::rebindCanvasToFramebuffer() { Backend::instance().rebindCanvasToFramebuffer(); }
void Display::setFlushConfig(int rows, int depth) { Backend::instance().setFlushConfig(rows, depth); }
void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return Backend::instance().flushChunkRows(); }
int Display::flushQueueDepth() { return Backend::instance().flushQueueDepth(); }
int Display::flushBufferBytes() { return Backend::instance().flushBufferBytes(); }
void Display::pushClip(int x, int y, int w, int h) { if (auto *c = drawingCanvas()) c->pushClip(x, y, w, h); }
void Display::popClip() { if (auto *c = drawingCanvas()) c->popClip(); }
void Display::resetClip() { if (auto *c = drawingCanvas()) c->resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { if (auto *c = drawingCanvas()) c->setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return drawingCanvas() ? drawingCanvas()->globalAlpha() : 255; }
int Display::brightness() { return Backend::instance().brightness(); }
void Display::setBrightness(int value) { Backend::instance().setBrightness(value); }
void Display::setVSync(bool) {}
bool Display::vsyncEnabled() { return false; }
void Display::vsyncWaitForFrame() {}
void Display::invalidate() {}
void Display::clip(int *x0, int *y0, int *x1, int *y1) { if (auto *c = drawingCanvas()) c->currentClip(x0, y0, x1, y1); }
void Display::fillRect(int x, int y, int w, int h, pixel::native_t color) { if (auto *c = drawingCanvas()) c->fillRect(x, y, w, h, color); }
void Display::scrollRect(int x, int y, int w, int h, int dx, int dy) { if (auto *c = drawingCanvas()) c->scrollRect(x, y, w, h, dx, dy); }
void Display::resetScrollRegion() {}
void Display::strokeRect(int x, int y, int w, int h, pixel::native_t color) { if (auto *c = drawingCanvas()) c->strokeRect(x, y, w, h, color); }
void Display::fillCircle(int x, int y, int r, pixel::native_t color) { if (auto *c = drawingCanvas()) c->fillCircle(x, y, r, color); }
void Display::strokeCircle(int x, int y, int r, pixel::native_t color) { if (auto *c = drawingCanvas()) c->strokeCircle(x, y, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, pixel::native_t color) { if (auto *c = drawingCanvas()) c->drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int x, int y, int r, int start, int end, pixel::native_t color) { if (auto *c = drawingCanvas()) c->drawArc(x, y, r, start, end, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, pixel::native_t color) { if (auto *c = drawingCanvas()) c->fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, pixel::native_t color, float scale) { if (auto *c = drawingCanvas()) c->drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, pixel::native_t color, int font) { if (auto *c = drawingCanvas()) c->drawTextFont(text, x, y, color, font); }
void Display::drawTextFontFamily(const char *text, int x, int y, pixel::native_t color, int family, int size) { if (auto *c = drawingCanvas()) c->drawTextFontFamily(text, x, y, color, family, size); }
void Display::setPixel(int x, int y, pixel::native_t color) { if (auto *c = drawingCanvas()) c->fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, pixel::native_t color) { if (auto *c = drawingCanvas()) c->fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count, int w, int h, int tl, int tr, int br, int bl, const pixel::native_t *colors) { if (auto *c = drawingCanvas()) c->fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors); }
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lw, pixel::native_t color) { if (auto *c = drawingCanvas()) c->strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lw, color); }
void Display::blitImage(const pixel::native_t *src, const std::uint8_t *alpha, int sw, int sh, int x, int y) { if (auto *c = drawingCanvas()) c->drawImage(src, alpha, sw, sh, x, y); }
void Display::blitImageScaled(const pixel::native_t *src, const std::uint8_t *alpha, int sw, int sh, int x, int y, int w, int h) { if (auto *c = drawingCanvas()) c->drawImage(src, alpha, sw, sh, x, y, w, h); }
void Display::setWorldOverlay(const std::uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}
void Display::flushStatsRead(std::int64_t *us, int *calls, int *pixels)
{
	const auto stats = Backend::instance().flushStats();
	if (us) *us = stats.totalUs;
	if (calls) *calls = stats.callCount;
	if (pixels) *pixels = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead() { return Backend::instance().flushStats(); }
void Display::flushStatsReset() { Backend::instance().flushStatsReset(); }
const char *Display::flushStageName() { return Backend::instance().flushStageName(); }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return {}; }
bool Display::copySnapshotRgb565(std::uint16_t *dst, int capacity, int *w, int *h, bool) { return Backend::instance().copySnapshotRgb565(dst, capacity, w, h); }
bool Display::copySnapshotPacked(std::uint8_t *dst, int byteCapacity, int *w, int *h, bool) { return Backend::instance().copySnapshotPacked(dst, byteCapacity, w, h); }
int Display::countNonBlackPixels(bool) { return Backend::instance().countNonBlackPixels(); }

// The canvas draws directly into the packed 4bpp framebuffer, so the presented
// copy IS the live frame; a retained-snapshot re-replay would read packed
// storage as unpacked and lie.
extern "C" bool geaDisplaySnapshotPrefersPresented() { return true; }

void applyOrientation(gea::framework::display::DisplayOrientation orientation) { Backend::instance().applyOrientation(orientation); }
bool autoRotateAllowed() { return false; }

}  // namespace gea::platform::display
