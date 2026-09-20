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
#include "papers3_epd.h"
}

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;

namespace gea::platform::papers3::display {

namespace {

constexpr char kTag[] = "papers3_display";

// Physical panel is 960x540 LANDSCAPE (ED047TC1 class), wired straight to the
// SoC — no controller-side rotation (unlike the original M5Paper's IT8951).
//
// Apps + the framework render LOGICAL PORTRAIT 540x960. We render LANDSCAPE-
// NATIVE: the Canvas is bound in its rotation-aware mode straight to the physical
// 960x540 GRAY4 buffer (bindPixelsRotatedLandscape), so a logical pixel (lx,ly)
// lands directly at physical (px=ly, py=539-lx). There is NO per-frame transpose
// pass — the Canvas IS the panel buffer. The 90° map here is authoritative and
// matches the packing the panel driver reads (high nibble = left/even pixel).
constexpr int kControllerWidth = PAPERS3_EPD_WIDTH;    // 960 (physical width)
constexpr int kControllerHeight = PAPERS3_EPD_HEIGHT;  // 540 (physical height)
constexpr int kPortraitWidth = 540;                    // logical width
constexpr int kPortraitHeight = 960;                   // logical height
constexpr int kEpdStride = kControllerWidth / 2;                 // 480 bytes/row
constexpr int kEpdBytes = kEpdStride * kControllerHeight;        // 960x540 4bpp

// ED047TC1 4-bit brightness: 0x0 = black, 0xF = white. gea GRAY4 native values
// use the same polarity (0 = black, 15 = white), so no inversion is needed and
// the Canvas gray values go straight to the panel unchanged (the old transpose's
// kInvertGray=false is now implicit — nothing rewrites the levels).

// Reset accumulated e-paper ghosting periodically. Apps normally override this
// (the e-reader uses 10); zero disables cadence-based full refreshes.
constexpr int kDefaultFullRefreshEvery = 10;

// NOTE ON PANEL SAFETY. The protection that matters lives in
// components/papers3_epd: a pixel stops being driven the moment it reaches its
// target level (NOP-at-target), drive per pixel is bounded by construction, and a
// drive-duty limiter forces a rails-off rest if a caller refreshes continuously.

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

		// The Canvas renders rotated straight into a physical landscape GRAY4
		// buffer. The panel gets a second physical buffer as an immutable scan
		// snapshot: a quality refresh takes up to two 13-field cycles, and letting the runtime draw
		// the next page into the buffer while those fields are being generated mixes
		// two targets in one waveform. Worse, the driver's final shadow commit would
		// then claim that the mixed scan showed the newest live buffer, poisoning
		// every later delta. A 253 KiB snapshot is cheaper than holding the render
		// mutex for the entire blocking panel scan and keeps touch/render responsive.
		epdFb_ = static_cast<std::uint8_t *>(
			heap_caps_aligned_calloc(16, 1, kEpdBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		scanFb_ = static_cast<std::uint8_t *>(
			heap_caps_aligned_alloc(16, kEpdBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!epdFb_ || !scanFb_) {
			ESP_LOGE(kTag, "framebuffer allocation failed (two PSRAM GRAY4 buffers required)");
			return false;
		}
		std::memset(scanFb_, 0xff, kEpdBytes);
		std::memset(dirtyRows_, 0, sizeof(dirtyRows_));
		dirtyCoversScreen_ = false;

		// Bring up the direct-drive panel and flash it to a clean white.
		papers3_epd_init();
		// TREATMENT-ONLY boot: no app rendering — balanced deghosting drive
		// with rails-off rests, 30-min session cap (reboot for another
		// session). Set false to restore normal rendering.
		constexpr bool kTreatmentOnlyBoot = false;
		if (kTreatmentOnlyBoot) {
			ESP_LOGI(kTag, "TREATMENT build: deghosting loop, no app rendering, 30-min session cap");
			papers3_epd_treatment_forever();  // never returns
		}
		papers3_epd_clear();

		bindCanvas();
		if (xTaskCreatePinnedToCore(refreshTaskEntry, "papers3_epd", 4096, this, 6, &refreshTask_, 1) != pdPASS) {
			ESP_LOGE(kTag, "failed to start refresh task");
			return false;
		}
		initialized_ = true;
		ESP_LOGI(kTag, "M5PaperS3 ready %dx%d portrait (landscape-native %dx%d), GRAY4 canvas / calibrated 4-level panel, epd=%d bytes",
		         kPortraitWidth, kPortraitHeight, kControllerWidth, kControllerHeight, kEpdBytes);
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
		canvas_.clear(static_cast<pixel::native_t>(15));
		markAllRowsLocked();
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
		canvas_.clear(static_cast<pixel::native_t>(15));
		markAllRowsLocked();
		forceFull_.store(true);
		give();
	}

	void flush()
	{
		if (!init()) return;
		take();
		accumulateCanvasDirtyLocked();
		canvas_.resetDirty();
		const bool needsRefresh = anyDirtyRows_ || forceFull_.load();
		give();
		if (needsRefresh) requestRefresh();
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		if (!rects || count <= 0 || !init()) return;
		take();
		for (int i = 0; i < count; ++i)
			markLogicalRectLocked(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1);
		canvas_.resetDirty();
		give();
		requestRefresh();
	}

	bool streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user)
	{
		if (!raster || w <= 0 || h <= 0 || !init()) return false;
		const int x0 = std::max(0, x);
		const int y0 = std::max(0, y);
		const int x1 = std::min(kPortraitWidth - 1, x + w - 1);
		const int y1 = std::min(kPortraitHeight - 1, y + h - 1);
		if (x0 > x1 || y0 > y1) return true;
		const int width = x1 - x0 + 1;
		static pixel::native_t rowbuf[kPortraitWidth];
		take();
		for (int row = y0; row <= y1; ++row) {
			raster(rowbuf, width, 1, x0, row, user);
			for (int i = 0; i < width; ++i)
				epdSetLogical(x0 + i, row, static_cast<std::uint8_t>(rowbuf[i]));
		}
		markLogicalRectLocked(x0, y0, x1, y1);
		give();
		requestRefresh();
		return true;
	}

	// Accept the direct-canvas present batch and rasterize it straight into the
	// GRAY4 framebuffer (native_t colors are already gray4 at compile time), then
	// refresh. The Canvas is bound rotated, so every draw lands landscape-native.
	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int count)
	{
		using T = platform_display::DisplayPresentCommandType;
		if (!commands || count <= 0 || !init()) return false;
		take();
		canvas_.resetClip();
		for (int i = 0; i < count; ++i) {
			const auto &c = commands[i];
			switch (c.type) {
			case T::Clear:
				canvas_.setGlobalAlpha(255);
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
		// A canvas-app present repaints the whole logical surface; mark every
		// physical row and let the per-pixel delta drive skip what did not change.
		markAllRowsLocked();
		give();
		requestRefresh();
		return true;
	}

	void setFlushConfig(int, int) {}
	int flushChunkRows() const { return kPortraitHeight; }
	int flushQueueDepth() const { return 1; }
	int flushBufferBytes() const { return kEpdBytes; }

	// Read the physical landscape buffer back out as a LOGICAL portrait RGB565
	// snapshot (inverse of the render rotation): logical (lx,ly) <- physical
	// (px=ly, py=539-lx).
	bool copySnapshotRgb565(std::uint16_t *dst, int capacity, int *width, int *height)
	{
		const int pixels = kPortraitWidth * kPortraitHeight;
		if (!dst || capacity < pixels || !epdFb_) return false;
		if (width) *width = kPortraitWidth;
		if (height) *height = kPortraitHeight;
		for (int ly = 0; ly < kPortraitHeight; ++ly) {
			for (int lx = 0; lx < kPortraitWidth; ++lx) {
				const std::uint8_t v = epdGetLogical(lx, ly);
				dst[static_cast<std::size_t>(ly) * kPortraitWidth + lx] = pixel::gray4ToRgb565(v);
			}
		}
		return true;
	}

	// Same LOGICAL-portrait snapshot as copySnapshotRgb565, but packed 2 px/byte
	// (this board's native GRAY4 storage) instead of expanded to RGB565 -- 259,200
	// bytes for the 540x960 panel vs 1,036,800 expanded. Caller (device_control's
	// screenshot commands) expands to RGB565 per-pixel while it encodes, so the
	// wire format is unchanged; only the device-side allocation shrinks. Written
	// as a flat row-major pixel stream (index ly*kPortraitWidth+lx, no per-row
	// padding -- kPortraitWidth is even so that never bites), through the same
	// epdGetLogical rotation the RGB565 path uses (physical px=ly, py=539-lx).
	bool copySnapshotPacked(std::uint8_t *dst, int byteCapacity, int *width, int *height)
	{
		const int pixels = kPortraitWidth * kPortraitHeight;
		const int neededBytes = pixel::packed::rowBytes(pixels);
		if (!dst || byteCapacity < neededBytes || !epdFb_) return false;
		if (width) *width = kPortraitWidth;
		if (height) *height = kPortraitHeight;
		for (int ly = 0; ly < kPortraitHeight; ++ly) {
			for (int lx = 0; lx < kPortraitWidth; ++lx) {
				const std::uint8_t v = epdGetLogical(lx, ly);
				pixel::packed::set(dst, ly * kPortraitWidth + lx, v);
			}
		}
		return true;
	}

	int countNonBlackPixels() const
	{
		int count = 0;
		for (int i = 0; epdFb_ && i < kEpdBytes; ++i) {
			const std::uint8_t b = epdFb_[i];
			if (b >> 4) ++count;
			if (b & 0x0F) ++count;
		}
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
		if (fullEvery >= 0) fullRefreshEvery_.store(fullEvery);
		if (hardCap >= 0) fullRefreshHardCap_.store(hardCap);
		ESP_LOGI(kTag, "refresh config fullEvery=%d hardCap=%d",
		         fullRefreshEvery_.load(), fullRefreshHardCap_.load());
	}

	void requestFullRefresh()
	{
		if (!init()) return;
		forceFull_.store(true);
		requestRefresh();
	}

	void setQualityGrayscale(bool enabled)
	{
		// A waveform-mode boundary gets a known-white baseline. Promote the next
		// real frame rather than immediately repainting stale content.
		if (qualityGrayscale_.exchange(enabled) != enabled) forceFull_.store(true);
		ESP_LOGI(kTag, "grayscale mode=%s",
		         enabled ? "calibrated 4-level high" : "calibrated 4-level fast");
	}
	void setFullOnCover(bool enabled) { fullOnCover_.store(enabled); }
	void rebindCanvasToFramebuffer() { bindCanvas(); }

private:
	DisplayBackend() = default;

	int logicalWidth() const { return kPortraitWidth; }
	int logicalHeight() const { return kPortraitHeight; }

	void take() { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
	void give() { if (mutex_) xSemaphoreGive(mutex_); }

	void bindCanvas() { take(); bindCanvasLocked(); give(); }
	void bindCanvasLocked()
	{
		// Rotation-aware bind: the Canvas presents logical portrait 540x960 but
		// writes rotated into the physical landscape buffer. Every draw lands in
		// its final panel location with no separate transpose pass.
		canvas_.bindPixelsRotatedLandscape(epdFb_, kPortraitWidth, kPortraitHeight);
	}

	// Direct rotated write of a LOGICAL pixel into the physical buffer (used by the
	// streamRect raster path, which does not go through the Canvas). Mirrors the
	// Canvas rotation exactly: physical (px=ly, py=539-lx), high nibble = even px.
	void epdSetLogical(int lx, int ly, std::uint8_t v)
	{
		const int px = ly;
		const int py = (kPortraitWidth - 1) - lx;
		std::uint8_t *b = epdFb_ + static_cast<std::size_t>(py) * kEpdStride + (px >> 1);
		if (px & 1) *b = static_cast<std::uint8_t>((*b & 0xF0) | (v & 0x0F));
		else *b = static_cast<std::uint8_t>((*b & 0x0F) | ((v & 0x0F) << 4));
	}

	std::uint8_t epdGetLogical(int lx, int ly) const
	{
		const int px = ly;
		const int py = (kPortraitWidth - 1) - lx;
		const std::uint8_t b = epdFb_[static_cast<std::size_t>(py) * kEpdStride + (px >> 1)];
		return (px & 1) ? static_cast<std::uint8_t>(b & 0x0F) : static_cast<std::uint8_t>(b >> 4);
	}

	// Map a LOGICAL-portrait dirty rect to the PHYSICAL rows it touches and mark
	// them. logical x range [lx0,lx1] -> physical rows [539-lx1, 539-lx0]; the
	// logical y range maps to physical columns, which the driver drives whole-row
	// (unchanged pixels NOP), so it does not narrow the row set. Caller holds mutex.
	void markLogicalRectLocked(int lx0, int ly0, int lx1, int ly1)
	{
		if (lx0 < 0) lx0 = 0;
		if (ly0 < 0) ly0 = 0;
		if (lx1 > kPortraitWidth - 1) lx1 = kPortraitWidth - 1;
		if (ly1 > kPortraitHeight - 1) ly1 = kPortraitHeight - 1;
		if (lx0 > lx1 || ly0 > ly1) return;
		int py0 = (kControllerHeight - 1) - lx1;
		int py1 = (kControllerHeight - 1) - lx0;
		if (py0 < 0) py0 = 0;
		if (py1 > kControllerHeight - 1) py1 = kControllerHeight - 1;
		for (int py = py0; py <= py1; ++py) dirtyRows_[py] = 1;
		anyDirtyRows_ = true;
		if (lx0 == 0 && ly0 == 0 && lx1 == kPortraitWidth - 1 && ly1 == kPortraitHeight - 1)
			dirtyCoversScreen_ = true;
	}

	void markAllRowsLocked()
	{
		std::memset(dirtyRows_, 1, sizeof(dirtyRows_));
		anyDirtyRows_ = true;
		dirtyCoversScreen_ = true;
	}

	bool coversScreenLocked() const { return dirtyCoversScreen_; }

	// Pull the engine's per-rect dirty regions (or its overall bounds) off the
	// Canvas and fold them into the physical dirty-row set. Caller holds mutex.
	void accumulateCanvasDirtyLocked()
	{
		gea::framework::graphics::CanvasDirtyRect rects[gea::framework::graphics::Canvas::kMaxDirtyRects];
		const int n = canvas_.dirtyRects(rects, gea::framework::graphics::Canvas::kMaxDirtyRects);
		if (n > 0) {
			for (int i = 0; i < n; ++i)
				markLogicalRectLocked(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1);
			return;
		}
		int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
		if (canvas_.dirty(&x0, &y0, &x1, &y1)) markLogicalRectLocked(x0, y0, x1, y1);
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

	static void refreshTaskEntry(void *arg) { static_cast<DisplayBackend *>(arg)->refreshLoop(); }

	void refreshLoop()
	{
		std::uint32_t transmitted = 0;
		for (;;) {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			const std::uint32_t seq = pendingSeq_.load(std::memory_order_acquire);
			const bool newFrame = (seq != transmitted);
			if (!newFrame) continue;
			transmitted = seq;

			refreshInFlight_.store(true);
			const std::int64_t started = esp_timer_get_time();

			// Both modes use the calibrated four-state physical transition model.
			// Reading mode (quality=true, fullOnCover=false) deliberately uses the
			// 7-pass FAST table for ordinary page turns and periodically clears the
			// panel. QUALITY (fullOnCover=true) still flashes full page redraws; its
			// smaller partials use the 13-pass HIGH table.
			const bool quality = qualityGrayscale_.load();
			const bool highPartial = quality && fullOnCover_.load();
			bool coversScreen = false;

			// Clear the engine's dirty accumulator after snapshotting this frame.
			take();
			coversScreen = coversScreenLocked();
			// From this point until the panel powers off, every field and the
			// driver's shadow commit must observe exactly this same frame.
			std::memcpy(scanFb_, epdFb_, kEpdBytes);
			std::memset(dirtyRows_, 0, sizeof(dirtyRows_));
			anyDirtyRows_ = false;
			dirtyCoversScreen_ = false;
			give();

			const bool forced = forceFull_.exchange(false);
			const int fullEvery = fullRefreshEvery_.load();
			const int hardCap = fullRefreshHardCap_.load();
			const bool cadenceFull = quality &&
			                         ((fullEvery > 0 && partialsSinceFull_ >= fullEvery) ||
			                          (hardCap > 0 && partialsSinceFull_ >= hardCap));
			const bool coverFull = quality && coversScreen && fullOnCover_.load();
			const bool full = forced || cadenceFull || coverFull;

			papers3_epd_power_on();
			if (full) {
				// Blocking flash-to-white + calibrated full redraw.
				papers3_epd_refresh(scanFb_);
				partialsSinceFull_ = 0;
			} else if (highPartial) {
				// HIGH partial: changed old ink is erased to white, then non-white
				// targets are painted with the calibrated PaperS3 waveform. Unchanged
				// pixels emit NOP throughout.
				papers3_epd_draw_gray4(scanFb_);
				++partialsSinceFull_;
			} else {
				papers3_epd_draw_fast_gray4(scanFb_);
				++partialsSinceFull_;
			}
			papers3_epd_power_off();

			const std::int64_t elapsed = esp_timer_get_time() - started;
			stats_.callCount++;
			stats_.txUs += elapsed;
			stats_.chunkCount++;
			refreshInFlight_.store(false);
			const char *kind = full ? "full" : (highPartial ? "high4" : "fast4");
			ESP_LOGI(kTag, "%s present %lldms (partials=%d)",
			         kind,
			         static_cast<long long>(elapsed / 1000), partialsSinceFull_);
		}
	}

	SemaphoreHandle_t mutex_ = nullptr;
	TaskHandle_t refreshTask_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	std::uint8_t *epdFb_ = nullptr;          // physical landscape GRAY4 (Canvas target)
	std::uint8_t *scanFb_ = nullptr;         // immutable physical GRAY4 panel target
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	// Physical rows dirtied by the engine since the last drive; drained by the
	// refresh task. Protected by mutex_.
	std::uint8_t dirtyRows_[kControllerHeight] = {};
	bool anyDirtyRows_ = false;
	bool dirtyCoversScreen_ = false;
	std::atomic<std::uint32_t> pendingSeq_{0};
	std::atomic<bool> refreshInFlight_{false};
	std::atomic<bool> forceFull_{true};
	// Default = calibrated FAST; apps that want cleaner text opt into HIGH.
	std::atomic<bool> qualityGrayscale_{false};
	std::atomic<bool> fullOnCover_{false};
	int partialsSinceFull_ = 0;
	std::atomic<int> fullRefreshEvery_{kDefaultFullRefreshEvery};
	std::atomic<int> fullRefreshHardCap_{0};
	bool backgroundAttempted_ = false;
	bool backdropAttempted_ = false;
	bool initialized_ = false;
	platform_display::DisplayFlushPerfStats stats_ = {};
};

}  // namespace

}  // namespace gea::platform::papers3::display

extern "C" void gea_epaper_set_refresh_config(
	int fullEvery, int hardCap, int fastWindowMs,
	const std::uint8_t *, int, const std::uint8_t *, int)
{
	(void)fastWindowMs;
	gea::platform::papers3::display::DisplayBackend::instance().setRefreshConfig(fullEvery, hardCap, fastWindowMs);
}

extern "C" void gea_epaper_full_refresh(void)
{
	gea::platform::papers3::display::DisplayBackend::instance().requestFullRefresh();
}

extern "C" void gea_epaper_set_grayscale(int enabled)
{
	gea::platform::papers3::display::DisplayBackend::instance().setQualityGrayscale(enabled != 0);
}

extern "C" void gea_epaper_set_full_on_cover(int enabled)
{
	gea::platform::papers3::display::DisplayBackend::instance().setFullOnCover(enabled != 0);
}

extern "C" std::uint16_t *gea_bg_cache(int *capacity)
{
	return gea::platform::papers3::display::DisplayBackend::instance().backgroundCache(capacity);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *capacity)
{
	return gea::platform::papers3::display::DisplayBackend::instance().backdropCache(capacity);
}

namespace gea::platform::display {

namespace {

using Backend = gea::platform::papers3::display::DisplayBackend;
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
