#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "host/display_orientation.h"
#include "pixel.h"
#include "tab5_drivers.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <utility>

#include "driver/ledc.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_async_memcpy.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_private/esp_cache_private.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;
namespace present = gea::framework::display_present;

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1);
extern "C" void gea_render_parallel_wait();

namespace gea::platform::esp32_p4_m5stack_tab5::display {

namespace {

constexpr const char *kTag = "m5stack_tab5_display";
constexpr int kNativeWidth = platform_display::kNativeWidth;
constexpr int kNativeHeight = platform_display::kNativeHeight;
constexpr int kMaxLogicalWidth = kNativeHeight > kNativeWidth ? kNativeHeight : kNativeWidth;
constexpr int kMaxLogicalHeight = kNativeHeight;
constexpr int kMaxPixels = kNativeWidth * kNativeHeight;
constexpr int kFlushRowsDefault = 64;
constexpr int kDsiLaneCount = 2;
constexpr float kDsiLaneBitrateMbps = 1000.0f;
constexpr float kDpiClockMhz = 80.0f;
constexpr pixel::Format kFramebufferPixelFormat = pixel::Format::Rgb565;
constexpr pixel::Format kDsiOutputPixelFormat = pixel::Format::Rgb565;
constexpr int kPanelBitsPerPixel = pixel::bitsPerPixel(kDsiOutputPixelFormat);
// Two DPI scanout framebuffers. Native portrait UI renders into the non-scanned
// buffer, syncs touched row bands, then swaps the DPI framebuffer index; full-screen
// direct producers (camera/tile paths) use the same back-buffer flip model.
constexpr int kDpiFrameBufferCount = 3;
constexpr int kDsiPhyLdoChannel = 3;
constexpr int kDsiPhyLdoVoltageMv = 2500;
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_1;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_1;
constexpr int kBacklightLedcMaxDuty = 1023;
constexpr int kMaxPresentRects = 16;
constexpr int kMaxFlushRects = 64;
constexpr int kCircleDeltaMax = 96;
constexpr int kCircleDeltaMaxShift = 16;
constexpr int kCircleSpanCacheMaxRadius = 256;
constexpr int kCpuPanelMapFlushMaxPixels = 4096;
constexpr int kOwnedFrameBufferPixels = kMaxPixels;
#ifndef GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS
#define GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS 96
#endif
#ifndef GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS
#define GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS 65536
#endif
constexpr int kPresentParallelMinRows = GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS;
constexpr int kPresentParallelMinPixels = GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS;
constexpr std::uint32_t kAsyncPresentStackBytes = 12288;

struct PanelCoord {
	int x = 0;
	int y = 0;
};

struct PresentRasterBand {
	std::uint16_t *base = nullptr;
	int stridePixels = 0;
	int regionX0 = 0;
	int regionWidth = 0;
	int logicalY0 = 0;
	int displayHeight = 0;
	int maxRows = 1;
	const present::Frame *frame = nullptr;
};

void rasterPresentBandRows(PresentRasterBand &job, int row0, int row1)
{
	if (!job.base || !job.frame || job.regionWidth <= 0 || job.maxRows <= 0 || row0 > row1) return;
	for (int r = row0; r <= row1; r += job.maxRows) {
		int rows = job.maxRows;
		if (r + rows > row1 + 1) rows = row1 - r + 1;
		std::uint16_t *target = job.base + static_cast<std::size_t>(r) * job.stridePixels;
		present::rasterFrameRowsStrided(target,
		                                job.stridePixels,
		                                job.regionX0,
		                                job.regionWidth,
		                                job.logicalY0 + r,
		                                rows,
		                                job.displayHeight,
		                                *job.frame);
	}
}

void rasterPresentBandThunk(void *ctx, int y0, int y1)
{
	auto *job = static_cast<PresentRasterBand *>(ctx);
	if (!job) return;
	rasterPresentBandRows(*job, y0, y1);
}

constexpr lcd_color_format_t lcdColorFormat(pixel::Format format)
{
	switch (format) {
	case pixel::Format::Rgb565:
		return LCD_COLOR_FMT_RGB565;
	case pixel::Format::Rgb888:
		return LCD_COLOR_FMT_RGB888;
	case pixel::Format::Rgba8888:
	case pixel::Format::Argb8888:
		break;
	}
	return LCD_COLOR_FMT_RGB565;
}

PanelCoord panelForSourcePixel(int sourceX, int sourceY)
{
	using gea::framework::display::DisplayOrientation;
	switch (orientation_detail::DisplayOrientationState::orientation()) {
	case DisplayOrientation::PortraitSecondary:
		return {kNativeWidth - 1 - sourceX, kNativeHeight - 1 - sourceY};
	case DisplayOrientation::LandscapePrimary:
		return {sourceY, kNativeHeight - 1 - sourceX};
	case DisplayOrientation::LandscapeSecondary:
		return {kNativeWidth - 1 - sourceY, sourceX};
	case DisplayOrientation::PortraitPrimary:
	default:
		return {sourceX, sourceY};
	}
}

present::Rect clampLogicalRect(present::Rect rect, int width, int height)
{
	if (width <= 0 || height <= 0) return {};
	rect = present::clampAndAlign(rect, width, height);
	if (!present::valid(rect)) return {};
	return rect;
}

int integerSqrt(int n)
{
	if (n <= 0) return 0;
	int x = n;
	int y = (x + 1) / 2;
	while (y < x) {
		x = y;
		y = (x + n / x) / 2;
	}
	return x;
}

present::Rect physicalBoundsForLogicalRect(const present::Rect &logical)
{
	if (!present::valid(logical)) return {};
	const PanelCoord corners[] = {
		panelForSourcePixel(logical.x0, logical.y0),
		panelForSourcePixel(logical.x1, logical.y0),
		panelForSourcePixel(logical.x0, logical.y1),
		panelForSourcePixel(logical.x1, logical.y1),
	};
	present::Rect out{corners[0].x, corners[0].y, corners[0].x, corners[0].y};
	for (const PanelCoord &corner : corners) {
		if (corner.x < out.x0) out.x0 = corner.x;
		if (corner.y < out.y0) out.y0 = corner.y;
		if (corner.x > out.x1) out.x1 = corner.x;
		if (corner.y > out.y1) out.y1 = corner.y;
	}
	return present::clampAndAlign(out, kNativeWidth, kNativeHeight);
}

long long rectArea64(const present::Rect &rect)
{
	if (!present::valid(rect)) return 0;
	return static_cast<long long>(rect.x1 - rect.x0 + 1) * (rect.y1 - rect.y0 + 1);
}

}  // namespace

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
		if (!mutex_) {
			ESP_LOGE(kTag, "failed to allocate display mutex");
			return false;
		}

		// 128-byte aligned so the PPA can target it as an output picture (the
		// camera's direct-present scales straight into this framebuffer). PPA
		// output to PSRAM requires the buffer base + size aligned to the cache
		// line; round the size up to match.
		{
			const std::size_t fbBytes =
				(static_cast<std::size_t>(kOwnedFrameBufferPixels) * sizeof(std::uint16_t) + 127) & ~static_cast<std::size_t>(127);
			ownedFrameBuffer_ = static_cast<std::uint16_t *>(
				heap_caps_aligned_alloc(128, fbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
			if (ownedFrameBuffer_) std::memset(ownedFrameBuffer_, 0, fbBytes);
		}
		if (!ownedFrameBuffer_) {
			ESP_LOGE(kTag, "display owned framebuffer alloc failed");
			return false;
		}

		if (!initPanel()) {
			ESP_LOGE(kTag, "panel init failed");
			return false;
		}
		bindCanvas();
		if (frameBuffer_) {
			std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		}
		initialized_ = true;
		ESP_LOGI(kTag, "display ready native=%dx%d logical=%dx%d", kNativeWidth, kNativeHeight, logicalWidth(), logicalHeight());
		return true;
	}

	bool start() { return init(); }

	gea::framework::graphics::Canvas *canvas()
	{
		waitForAsyncPresent();
		prepareDrawingTarget();
		return &replayCanvas();
	}

	// Public accessor: is canvas() the panel scanout buffer (no logical->panel
	// rotation)? Used by the camera direct-present to skip the rotate-flush.
	bool framebufferIsPanelDirect() const { return usingPanelFrameBufferDirectly(); }
	bool snapshotPrefersPresented() const { return panelDirectActive_ && panelFrameBuffers_[scannedFbIndex_] != nullptr; }

	// The portrait scanout buffer the camera can PPA straight into for its preview
	// rect. Valid whenever the panel framebuffer is the un-rotated portrait scanout
	// (logical coords == panel coords), independent of where the UI composes — so it
	// stays available even though the UI now composes off-screen into ownedFrameBuffer_.
	bool panelScanoutSurface(std::uint16_t **outBuffer, int *outW, int *outH) const
	{
		using gea::framework::display::DisplayOrientation;
		if (!panelFrameBuffer_ ||
		    orientation_detail::DisplayOrientationState::orientation() != DisplayOrientation::PortraitPrimary ||
		    logicalStride_ != kNativeWidth) {
			return false;
		}
		if (outBuffer) *outBuffer = panelFrameBuffer_;
		if (outW) *outW = kNativeWidth;
		if (outH) *outH = kNativeHeight;
		return true;
	}

	// One-pass camera→panel target (see Display::panelDirectTarget). The panel scanout
	// buffer always exists, so this works in ANY orientation: the camera composes its own
	// sensor rotation with the orientation's rotation steps and scales+rotates straight
	// into the panel, eliminating the off-screen landscape canvas + second rotate flush.
	bool panelDirectTarget(int logicalX, int logicalY, int logicalW, int logicalH,
	                       std::uint16_t **outBuf, int *outBufW, int *outBufH,
	                       int *outPanelX, int *outPanelY, int *outPanelW, int *outPanelH,
	                       int *outRotSteps, int *outFlip) const
	{
		if (!panelFrameBuffer_) return false;
		present::Rect logical{logicalX, logicalY, logicalX + logicalW - 1, logicalY + logicalH - 1};
		logical = clampLogicalRect(logical, logicalWidth(), logicalHeight());
		if (!present::valid(logical)) return false;
		const present::Rect panel = physicalBoundsForLogicalRect(logical);
		if (!present::valid(panel)) return false;
		const int panelW = panel.x1 - panel.x0 + 1;
		const int panelH = panel.y1 - panel.y0 + 1;
		// Tear-free double-buffer path: only when the producer covers the WHOLE panel
		// (e.g. a full-screen camera) and a second framebuffer exists. Then it writes the
		// non-scanned (back) buffer and flips it in at vblank — nothing it overwrites is
		// being scanned. A sub-rect producer (windowed) can't double-buffer without also
		// carrying the surrounding UI into the back buffer, so it writes the live buffer.
		const bool coversPanel = panel.x0 == 0 && panel.y0 == 0 && panelW == kNativeWidth && panelH == kNativeHeight;
		const bool canFlip = coversPanel && panelFrameBuffers_[1] != nullptr;
		if (outBuf) {
			std::uint16_t *target = panelFrameBuffer_;
			if (canFlip) {
				const int writable = const_cast<DisplayBackend *>(this)->selectWritablePanelFrameBufferLocked(true);
				if (writable >= 0) target = panelFrameBuffers_[writable];
			}
			*outBuf = target;
		}
		if (outFlip) *outFlip = canFlip ? 1 : 0;
		if (outBufW) *outBufW = kNativeWidth;
		if (outBufH) *outBufH = kNativeHeight;
		if (outPanelX) *outPanelX = panel.x0;
		if (outPanelY) *outPanelY = panel.y0;
		if (outPanelW) *outPanelW = panelW;
		if (outPanelH) *outPanelH = panelH;
		if (outRotSteps) {
			switch (ppaRotationForCurrentOrientation()) {
			case PPA_SRM_ROTATION_ANGLE_90: *outRotSteps = 1; break;
			case PPA_SRM_ROTATION_ANGLE_180: *outRotSteps = 2; break;
			case PPA_SRM_ROTATION_ANGLE_270: *outRotSteps = 3; break;
			default: *outRotSteps = 0; break;
			}
		}
		return true;
	}

	// Flip the just-written back buffer in at the next vblank (tear-free). The producer
	// PPA-wrote a writable panel framebuffer (returned by panelDirectTarget with
	// outFlip=1); draw_bitmap with a pointer inside an internal fb does no copy, it cache-
	// syncs the rows and swaps cur_fb_index, which the DSI DMA picks up at the frame restart.
	void flipPanelToBack()
	{
		if (!panel_ || panelFrameBuffers_[1] == nullptr) return;
		const int back = selectWritablePanelFrameBufferLocked(true);
		if (back < 0) return;
		if (flipPanelToBufferLocked(panelFrameBuffers_[back], true)) {
			resetPanelBackBufferCoherenceLocked();
			lastFlipUs_ = esp_timer_get_time();
		}
	}

	void flipPanelToBack(std::uint16_t *buffer)
	{
		if (!panel_ || panelFrameBuffers_[1] == nullptr || !buffer) return;
		const int index = panelFrameBufferIndex(buffer);
		if (index < 0 || index == scannedFbIndex_) return;
		if (flipPanelToBufferLocked(buffer, true)) {
			resetPanelBackBufferCoherenceLocked();
			lastFlipUs_ = esp_timer_get_time();
			panelDirectActive_ = true;
			previousPresentValid_ = false;
			frameBufferCoherent_ = false;
		}
	}

	gea::framework::graphics::Canvas *drawingCanvas()
	{
		waitForAsyncPresent();
		prepareDrawingTarget();
		return &replayCanvas();
	}

	void applyOrientation()
	{
		take();
		bindCanvasLocked();
		if (!frameBuffer_) {
			give();
			return;
		}
		resetPanelBackBufferCoherenceLocked();
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		previousPresentValid_ = false;
		needsFullPhysicalFlush_ = true;
		frameBufferCoherent_ = true;
		give();
		ESP_LOGI(kTag, "orientation=%s logical=%dx%d",
		         orientation_detail::DisplayOrientationState::orientationString().c_str(),
		         logicalWidth(), logicalHeight());
	}

	void clear()
	{
		clearNoFlush();
		flush();
	}

	void clearNoFlush()
	{
		if (!frameBuffer_) init();
		take();
		bindCanvasLocked();
		resetPanelBackBufferCoherenceLocked();
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		previousPresentValid_ = false;
		needsFullPhysicalFlush_ = true;
		frameBufferCoherent_ = true;
		give();
	}

	// Restore the draw canvas binding to the framebuffer. The engine calls
	// Display::rebindCanvasToFramebuffer() after the fused rasterized-flush path
	// rebinds the canvas onto a chunk buffer; this board has no rasterized flush,
	// so it is idempotent here — but the symbol must exist and rebinding is the
	// correct, safe restore.
	void rebindCanvasToFramebuffer()
	{
		take();
		bindCanvasLocked();
		give();
	}

	void flush()
	{
		if (!init()) return;
		if (asyncPresentInFlight_.load(std::memory_order_acquire)) return;
		// While the panel-direct tile path owns the screen (full-frame flips of
		// the scanout back buffer), the composed logical canvas is STALE — a
		// flush here would paint it over the fresh tiles, alternating
		// fresh/stale frames (white flicker while panning). The transition out
		// of panel-direct re-arms a full flush explicitly.
		if (panelDirectActive_) return;
		pathExternalFlush_++;
		const bool fullPhysicalFlush = needsFullPhysicalFlush_;
		if (!fullPhysicalFlush && !canvas_.dirty(nullptr, nullptr, nullptr, nullptr)) return;
		const std::int64_t started = esp_timer_get_time();
		present::Rect dirtyRects[kMaxFlushRects];
		int dirtyCount = 0;
		if (fullPhysicalFlush) {
			dirtyRects[0] = {0, 0, logicalWidth() - 1, logicalHeight() - 1};
			dirtyCount = 1;
			ESP_LOGI(kTag, "first physical flush covers full logical frame %dx%d", logicalWidth(), logicalHeight());
		} else {
			gea::framework::graphics::CanvasDirtyRect canvasRects[gea::framework::graphics::Canvas::kMaxDirtyRects];
			const int canvasDirtyCount = canvas_.dirtyRects(canvasRects, gea::framework::graphics::Canvas::kMaxDirtyRects);
			for (int i = 0; i < canvasDirtyCount; ++i) {
				appendFlushRect(dirtyRects,
				                &dirtyCount,
				                kMaxFlushRects,
				                {canvasRects[i].x0, canvasRects[i].y0, canvasRects[i].x1, canvasRects[i].y1});
			}
		}
		if (!fullPhysicalFlush && dirtyCount <= 0) {
			int x0 = 0;
			int y0 = 0;
			int x1 = -1;
			int y1 = -1;
			if (canvas_.dirty(&x0, &y0, &x1, &y1)) {
				appendFlushRect(dirtyRects, &dirtyCount, kMaxFlushRects, {x0, y0, x1, y1});
			}
		}
		bool ok = true;
		flushStats_.callCount++;
		ok = flushPreparedLogicalRects(dirtyRects, dirtyCount);
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) {
			canvas_.resetDirty();
			if (fullPhysicalFlush) needsFullPhysicalFlush_ = false;
		}
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		waitForAsyncPresent();
		if (!rects || count <= 0) return;
		if (!init()) return;
		if (needsFullPhysicalFlush_) {
			const std::int64_t started = esp_timer_get_time();
			flushStats_.callCount++;
			ESP_LOGI(kTag, "first physical flush covers full logical frame %dx%d", logicalWidth(), logicalHeight());
			present::Rect fullRect{0, 0, logicalWidth() - 1, logicalHeight() - 1};
			const bool ok = flushPreparedLogicalRects(&fullRect, 1);
			flushStats_.totalUs += esp_timer_get_time() - started;
			if (ok) {
				canvas_.resetDirty();
				needsFullPhysicalFlush_ = false;
			}
			return;
		}
		const std::int64_t started = esp_timer_get_time();
		bool ok = true;
		flushStats_.callCount++;
		present::Rect dirtyRects[kMaxFlushRects];
		int dirtyCount = 0;
		for (int i = 0; i < count; ++i) {
			const auto &rect = rects[i];
			appendFlushRectExact(dirtyRects, &dirtyCount, kMaxFlushRects, {rect.x0, rect.y0, rect.x1, rect.y1});
		}
		ok = flushPreparedLogicalRects(dirtyRects, dirtyCount);
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) canvas_.resetDirty();
	}

	bool streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user)
	{
		waitForAsyncPresent();
		if (!raster || w <= 0 || h <= 0) return false;
		if (!init()) return false;
		prepareDrawingTarget();
		const int x0 = std::max(0, x);
		const int y0 = std::max(0, y);
		const int x1 = std::min(logicalWidth() - 1, x + w - 1);
		const int y1 = std::min(logicalHeight() - 1, y + h - 1);
		if (x0 > x1 || y0 > y1) return true;

		const int width = x1 - x0 + 1;
		const int maxRows = std::max(1, kFlushRowsDefault);
		for (int row = y0; row <= y1; row += maxRows) {
			int rows = maxRows;
			if (row + rows > y1 + 1) rows = y1 - row + 1;
			if (x0 == 0 && width == logicalStride_) {
				raster(frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_, width, rows, x0, row, user);
			} else {
				for (int copyRow = 0; copyRow < rows; ++copyRow) {
					raster(frameBuffer_ + static_cast<std::size_t>(row + copyRow) * logicalStride_ + x0,
					       width,
					       1,
					       x0,
					       row + copyRow,
					       user);
				}
			}
		}
		canvas_.markDirty(x0, y0, x1, y1);
		frameBufferCoherent_ = true;
		flush();
		return true;
	}

	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int commandCount)
	{
		if (!commands || commandCount <= 0 || !init()) return false;
		presentCalls_++;
		present::Frame current;
		if (!present::extractFrame(commands, commandCount, current)) {
			pathRejected_++;
			return false;
		}
		if (!present::frameHasOpaqueBase(current, logicalWidth(), logicalHeight())) {
			pathRejected_++;
			return false;
		}
		if (!isFastClearFillCirclesFrame(current) && !frameBufferCoherent_) {
			previousPresentValid_ = false;
			needsFullPhysicalFlush_ = true;
		}
		const bool tileShape = framePpaTileEligible(current);
		if (!tileShape && (scrollOriginX_ != 0 || scrollOriginY_ != 0)) {
			// A non-tile frame can't run on the wrapped surface: linearize by
			// forcing a full rebuild at origin (0,0).
			scrollOriginX_ = 0;
			scrollOriginY_ = 0;
			panelSyncedForTorus_ = false;
			previousPresentValid_ = false;
			needsFullPhysicalFlush_ = true;
		}
		// PANEL-DIRECT TILE PATH: raster the whole tile frame straight into the
		// NON-SCANNED panel buffer and flip it in at vblank. The default present
		// composes into ownedFrameBuffer_ then PPA-copies owned->panel — and that
		// 0-degree full-screen PPA copy measures a constant ~43ms (the PPA moves
		// ~86MB/s against the DSI-scanned PSRAM), capping every present at ~20fps
		// no matter how little was rastered. Direct raster (~15ms with the
		// memcpy/nearest blits) + draw_bitmap's internal row sync (~11ms) + a
		// tear-free flip ≈ 26ms — pan, pinch, and load all land at ~38fps.
		{
			static bool gateLogged = false;
			if (!gateLogged) {
				gateLogged = true;
				ESP_LOGI(kTag, "panel-direct gate: tile=%d fb1=%d orient=%d lw=%d lh=%d nw=%d nh=%d",
				         tileShape ? 1 : 0, panelFrameBuffers_[1] != nullptr ? 1 : 0,
				         static_cast<int>(orientation_detail::DisplayOrientationState::orientation()),
				         logicalWidth(), logicalHeight(), kNativeWidth, kNativeHeight);
			}
		}
		if (tileShape && panelFrameBuffers_[1] != nullptr &&
		    orientation_detail::DisplayOrientationState::orientation() ==
		        gea::framework::display::DisplayOrientation::PortraitPrimary &&
		    logicalWidth() == kNativeWidth && logicalHeight() == kNativeHeight) {
			scrollOriginX_ = 0;
			scrollOriginY_ = 0;
			panelSyncedForTorus_ = false;
			const std::int64_t started = esp_timer_get_time();
			flushStats_.callCount++;
			// Scan alignment: the previous flip latches at the next frame restart.
			// If it was recent, wait for one refresh boundary before writing the
			// buffer it released — otherwise the base fill paints the LIVE frame
			// (white flashes while panning).
			const int backIndex = selectWritablePanelFrameBufferLocked(true);
			if (backIndex < 0) return false;
			std::uint16_t *back = panelFrameBuffers_[backIndex];
			const int height = logicalHeight();
			const int width = logicalWidth();
			const int maxRows = std::max(1, kFlushRowsDefault);
			for (int row = 0; row < height; row += maxRows) {
				int rows = maxRows;
				if (row + rows > height) rows = height - row;
				// Occlusion: when the band is fully covered by opaque tile blits,
				// skip the base fill (command 0) — at ~100MB/s PSRAM bandwidth the
				// fill's 1.8MB of throwaway writes per frame cost ~12ms alone.
				std::size_t firstCommand = 0;
				{
					// Interval-union over x of commands spanning ALL of this band.
					int covered = 0;
					int spans[16][2];
					int spanCount = 0;
					for (std::size_t i = 1; i < current.commands.size() && spanCount < 16; ++i) {
						const present::Command &c = current.commands[i];
						const present::Rect b = present::commandBounds(c, width, height);
						if (!present::valid(b) || b.y0 > row || b.y1 < row + rows - 1) continue;
						spans[spanCount][0] = std::max(0, b.x0);
						spans[spanCount][1] = std::min(width - 1, b.x1);
						spanCount++;
					}
					// Sort the few spans by start (insertion sort) and union them.
					for (int a = 1; a < spanCount; ++a) {
						for (int b2 = a; b2 > 0 && spans[b2][0] < spans[b2 - 1][0]; --b2) {
							std::swap(spans[b2][0], spans[b2 - 1][0]);
							std::swap(spans[b2][1], spans[b2 - 1][1]);
						}
					}
					int reach = 0;
					for (int a = 0; a < spanCount; ++a) {
						if (spans[a][0] > reach) break;
						if (spans[a][1] + 1 > reach) reach = spans[a][1] + 1;
					}
					covered = reach >= width ? 1 : 0;
					if (covered) firstCommand = 1;
				}
				std::uint16_t *target = back + static_cast<std::size_t>(row) * kNativeWidth;
				const std::int64_t rasterStartUs = esp_timer_get_time();
				gea::framework::graphics::Canvas bandCanvas;
				bandCanvas.bindPixels(target, kNativeWidth, rows, kNativeWidth);
				for (std::size_t i = firstCommand; i < current.commands.size(); ++i) {
					const present::Command &cmd = current.commands[i];
					// Band pre-skip: without it every command rasters against every
					// 64-row chunk — ~25 sidebar text commands x 20 chunks walked
					// glyph loops 500x per frame (+110ms measured).
					if (cmd.type != gea::platform::display::DisplayPresentCommandType::Clear) {
						const present::Rect bounds = present::commandBounds(cmd, width, height);
						if (!present::valid(bounds) || bounds.y1 < row || bounds.y0 > row + rows - 1) continue;
					}
					present::rasterCommandRows(bandCanvas, target, 0, kNativeWidth, row, rows, height, cmd);
				}
				flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;
			}
			take();
			const std::int64_t flipStartUs = esp_timer_get_time();
			// Drain a stale refresh grant so the post-flip wait (next frame's
			// pre-raster) observes a boundary that happened AFTER this flip.
			drainRefreshDoneLocked();
			flipPanelToBack();
			lastFlipUs_ = esp_timer_get_time();
			flushStats_.copyUs += lastFlipUs_ - flipStartUs;
			give();
			flushStats_.totalUs += esp_timer_get_time() - started;
			flushStats_.chunkCount++;
			flushStats_.pixelCount += logicalWidth() * height;
			static int directHits = 0;
			directHits++;
			pathDirect_++;
			if ((directHits & 15) == 1) {
				ESP_LOGI(kTag, "panel-direct tile frame #%d total=%lldus", directHits,
				         static_cast<long long>(esp_timer_get_time() - started));
			}
			// Mark the mode: the composed (owned) framebuffer is now STALE. The
			// canvas flush is suppressed while active; the transition out
			// re-arms a full composed repaint.
			panelDirectActive_ = true;
			previousPresentValid_ = false;
			frameBufferCoherent_ = false;
			canvas_.resetDirty();
			return true;
		}
		// LANDSCAPE PANEL-NATIVE PRESENT: the panel is portrait-native and the
		// PPA's 90-degree rotation costs ~150ms/frame (measured) — no pipelining
		// can hide that. Instead, tiles are PRE-ROTATED once at decode (worker
		// core), every command's geometry is transformed into panel space, and
		// the frame rasters DIRECTLY into the portrait back scanout buffer like
		// the portrait panel-direct path. Sidebar text draws via the rotated
		// glyph blitter in a full-buffer pass.
		{
			using gea::framework::display::DisplayOrientation;
			const DisplayOrientation orient = orientation_detail::DisplayOrientationState::orientation();
			const bool landscape = orient == DisplayOrientation::LandscapePrimary;
			{
				static bool landGateLogged = false;
				if (!landGateLogged && landscape) {
					landGateLogged = true;
					ESP_LOGI(kTag, "landscape gate: tile=%d fb1=%d lw=%d lh=%d nW=%d nH=%d",
					         tileShape ? 1 : 0, panelFrameBuffers_[1] != nullptr ? 1 : 0, logicalWidth(), logicalHeight(),
					         kNativeWidth, kNativeHeight);
				}
			}
			if (tileShape && landscape && panelFrameBuffers_[1] != nullptr &&
			    logicalWidth() == kNativeHeight && logicalHeight() == kNativeWidth) {
				scrollOriginX_ = 0;
				scrollOriginY_ = 0;
				panelSyncedForTorus_ = false;
				const std::int64_t started = esp_timer_get_time();
				flushStats_.callCount++;
				// Transform geometry to panel space IN PLACE (landscape-primary:
				// panel = {srcY, kNativeHeight - 1 - srcX}); FillText keeps its
				// landscape anchor for the rotated text pass.
				using Type = gea::platform::display::DisplayPresentCommandType;
				for (present::Command &cmd : current.commands) {
					if (cmd.type == Type::Clear || cmd.type == Type::FillText) continue;
					const int lx = cmd.x;
					const int ly = cmd.y;
					const int lw2 = cmd.w;
					const int lh2 = cmd.h;
					cmd.x = ly;
					cmd.y = kNativeHeight - lx - lw2;
					cmd.w = lh2;
					cmd.h = lw2;
					if (cmd.type == Type::DrawImage || cmd.type == Type::DrawImageScaled) {
						const int sw = cmd.srcWidth;
						cmd.srcWidth = cmd.srcHeight;
						cmd.srcHeight = sw;
					}
				}
				// Scan alignment (same as the portrait path).
				{
					const std::int64_t alignStart = esp_timer_get_time();
					waitForReleasedPanelBufferLocked();
					landAlignUs_ = static_cast<int>(esp_timer_get_time() - alignStart);
				}
				const int backIndex = selectWritablePanelFrameBufferLocked(true);
				if (backIndex < 0) return false;
				std::uint16_t *back = panelFrameBuffers_[backIndex];
				const int height = kNativeHeight;
				const int width = kNativeWidth;
				// PAN FAST PATH: when every image command moved by ONE common
				// translation since the previous frame, copy the overlap from the
				// front buffer (frame N-1) and raster only the exposed strips and
				// the moved small rects (pins/labels). A full-screen re-raster is
				// ~40ms; the copy is ~15ms for the map region.
				// DISABLED pending debugging: v1 had a copy-direction defect that
				// wiped tiles and pathological dirty-raster cost (212-378ms). The
				// full raster below is correct at ~76ms.
				constexpr bool kPanFastPathEnabled = true;
				bool panMode = false;
				int pdx = 0, pdy = 0;
				present::Rect dirtyRects[64];
				int dirtyCount = 0;
				const std::int64_t panDetectStart = esp_timer_get_time();
				const std::uint32_t railHash = railContentHash(current, kNativeWidth, kNativeHeight);
				if (kPanFastPathEnabled && !landPrevValid_[backIndex]) {
					panBailReason_ = 1;
					panBailCounts_[1]++;
				}
				if (kPanFastPathEnabled && landPrevValid_[backIndex]) {
					bool haveDelta = false;
					bool consistent = true;
					int imgPairs = 0;
					for (const present::Command &cmd : current.commands) {
						if (cmd.type != Type::DrawImage && cmd.type != Type::DrawImageScaled) continue;
						for (int j = 0; j < landPrevCount_[backIndex]; ++j) {
							const LandPrevCmd &prev = landPrev_[backIndex][j];
							if (prev.pixels != cmd.pixels || prev.type != static_cast<std::int16_t>(cmd.type)) continue;
							// Same image at a different scale (fallback ancestors swap
							// span levels while tiles stream): not a translation pair —
							// the dirty phase repaints it — but no reason to bail.
							if (prev.w != cmd.w || prev.h != cmd.h) break;
							const int dx = cmd.x - prev.x;
							const int dy = cmd.y - prev.y;
							if (!haveDelta) { pdx = dx; pdy = dy; haveDelta = true; }
							else if (dx != pdx || dy != pdy) consistent = false;
							imgPairs++;
							break;
						}
						if (!consistent) break;
					}
					panMode = haveDelta && consistent && imgPairs >= 4 &&
					          railHash == landPrevRailHash_[backIndex] &&
					          pdx > -440 && pdx < 440 && pdy > -440 && pdy < 440;
					if (!panMode) {
						int reason = 2;
						if (!haveDelta) reason = 2;
						else if (!consistent) reason = 3;
						else if (imgPairs < 4) reason = 4;
						else reason = 5;
						panBailReason_ = reason;
						panBailCounts_[reason]++;
					}
					if (panMode) {
						// Small-rect dirties: previous + current fills/text in the map
						// region (pins, labels) so movers repaint and ghosts clear.
						auto addDirty = [&](present::Rect r) {
							// Each fresh tile is ~1.3ms of honest raster; only bail to a
							// full re-raster when the dirty set rivals the whole screen.
							if (dirtyCount >= 60) { panMode = false; panBailReason_ = 6; panBailCounts_[6]++; return; }
							if (!present::valid(r)) return;
							dirtyRects[dirtyCount++] = r;
						};
						for (int j = 0; j < landPrevCount_[backIndex] && panMode; ++j) {
							const LandPrevCmd &prev = landPrev_[backIndex][j];
							if (prev.pixels) continue;
							addDirty(present::Rect{prev.x, prev.y, prev.x + prev.w - 1, prev.y + prev.h - 1});
						}
						for (std::size_t ci = 0; ci < current.commands.size(); ++ci) {
							const present::Command &cmd = current.commands[ci];
							if (!panMode) break;
							if (ci == 0) continue;  // base fill participates in strip rasters only
							if (cmd.type == Type::Clear) continue;
							if (cmd.type == Type::DrawImage || cmd.type == Type::DrawImageScaled) {
								// A draw is clean only when its previous instance moved by
								// exactly the pan delta at the same scale; everything else
								// (fresh tile, rescaled ancestor) repaints, including the
								// previous footprint so stale pixels clear.
								bool clean = false;
								for (int j = 0; j < landPrevCount_[backIndex]; ++j) {
									const LandPrevCmd &prev = landPrev_[backIndex][j];
									if (prev.pixels != cmd.pixels) continue;
									if (prev.w == cmd.w && prev.h == cmd.h &&
									    cmd.x - prev.x == pdx && cmd.y - prev.y == pdy) {
										clean = true;
									} else {
										addDirty(present::Rect{prev.x, prev.y, prev.x + prev.w - 1, prev.y + prev.h - 1});
									}
									break;
								}
								if (!clean) addDirty(present::commandBounds(cmd, width, height));
								continue;
							}
							if (cmd.type == Type::FillText) {
								// Rail text only; rail rows aren't copied or touched in
								// pan mode, so nothing to do.
								continue;
							}
							const present::Rect b = present::commandBounds(cmd, width, height);
							// Rail-region fills (panel rows >= kRailPanelRow) stay put.
							if (b.y0 >= kRailPanelRow) continue;
							addDirty(b);
						}
					}
				}
				panDetectUs_ = static_cast<int>(esp_timer_get_time() - panDetectStart);
				if (panMode) {
					// NO COPY: the back buffer already holds this buffer's previous
					// frame (N-2); the strips below cover the N-2 -> N exposure.
					panKickUs_ = 0;
					// Exposed strips (disjoint from the copied block) raster while
					// the DMA streams; interior rects wait for completion below.
					present::Rect stripRects[4];
					int stripCount = 0;
					if (pdy > 0) stripRects[stripCount++] = present::Rect{0, 0, width - 1, pdy - 1};
					if (pdy < 0) stripRects[stripCount++] = present::Rect{0, kRailPanelRow + pdy, width - 1, kRailPanelRow - 1};
					if (pdx > 0) stripRects[stripCount++] = present::Rect{0, 0, pdx - 1, kRailPanelRow - 1};
					if (pdx < 0) stripRects[stripCount++] = present::Rect{width + pdx, 0, width - 1, kRailPanelRow - 1};
					const std::int64_t rasterStartUs = esp_timer_get_time();
					for (int sIdx = 0; sIdx < stripCount; ++sIdx) {
						present::Rect rect = stripRects[sIdx];
						if (rect.y0 < 0) rect.y0 = 0;
						if (rect.x0 < 0) rect.x0 = 0;
						if (rect.y1 >= kRailPanelRow) rect.y1 = kRailPanelRow - 1;
						if (rect.x1 >= width) rect.x1 = width - 1;
						if (!present::valid(rect)) continue;
						const int rows = rect.y1 - rect.y0 + 1;
						std::uint16_t *target = back + static_cast<std::size_t>(rect.y0) * width;
						gea::framework::graphics::Canvas bandCanvas;
						bandCanvas.bindPixels(target, width, rows, width);
						bandCanvas.pushClip(rect.x0, 0, rect.x1 - rect.x0 + 1, rows);
						for (std::size_t i = 0; i < current.commands.size(); ++i) {
							const present::Command &cmd = current.commands[i];
							if (cmd.type == Type::FillText) continue;
							if (cmd.type != Type::Clear) {
								const present::Rect bounds = present::commandBounds(cmd, width, height);
								if (!present::valid(bounds) || bounds.y1 < rect.y0 || bounds.y0 > rect.y1 ||
								    bounds.x1 < rect.x0 || bounds.x0 > rect.x1) continue;
							}
							present::rasterCommandRows(bandCanvas, target, 0, width, rect.y0, rows, height, cmd);
						}
						bandCanvas.resetClip();
					}
					panStripUs_ = static_cast<int>(esp_timer_get_time() - rasterStartUs);
					panWaitUs_ = 0;
					const std::int64_t panInteriorStart = esp_timer_get_time();
					// Interior dirty rects (pins, fresh tiles) over the copied area.
					for (int d = 0; d < dirtyCount; ++d) {
						present::Rect rect = dirtyRects[d];
						if (rect.y0 < 0) rect.y0 = 0;
						if (rect.x0 < 0) rect.x0 = 0;
						if (rect.y1 >= kRailPanelRow) rect.y1 = kRailPanelRow - 1;
						if (rect.x1 >= width) rect.x1 = width - 1;
						if (!present::valid(rect)) continue;
						const int rows = rect.y1 - rect.y0 + 1;
						std::uint16_t *target = back + static_cast<std::size_t>(rect.y0) * width;
						gea::framework::graphics::Canvas bandCanvas;
						bandCanvas.bindPixels(target, width, rows, width);
						bandCanvas.pushClip(rect.x0, 0, rect.x1 - rect.x0 + 1, rows);
						for (std::size_t i = 0; i < current.commands.size(); ++i) {
							const present::Command &cmd = current.commands[i];
							if (cmd.type == Type::FillText) continue;
							if (cmd.type != Type::Clear) {
								const present::Rect bounds = present::commandBounds(cmd, width, height);
								if (!present::valid(bounds) || bounds.y1 < rect.y0 || bounds.y0 > rect.y1 ||
								    bounds.x1 < rect.x0 || bounds.x0 > rect.x1) continue;
							}
							present::rasterCommandRows(bandCanvas, target, 0, width, rect.y0, rows, height, cmd);
						}
						bandCanvas.resetClip();
					}
					flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;
					panInteriorUs_ = static_cast<int>(esp_timer_get_time() - panInteriorStart);
				}
				// Snapshot this frame's commands into THIS buffer's slot for the
				// next time it becomes the back buffer (two frames from now).
				landPrevCount_[backIndex] = 0;
				landPrevValid_[backIndex] = false;
				bool snapshotOverflow = false;
				for (std::size_t ci = 0; ci < current.commands.size(); ++ci) {
					const present::Command &cmd = current.commands[ci];
					if (landPrevCount_[backIndex] >= kLandPrevMax) { snapshotOverflow = true; break; }
					if (ci == 0) continue;  // base fill: never a dirty source
					if (cmd.type == Type::Clear || cmd.type == Type::FillText) continue;
					const bool isImage = cmd.type == Type::DrawImage || cmd.type == Type::DrawImageScaled;
					const present::Rect b = present::commandBounds(cmd, width, height);
					if (!isImage && present::valid(b) && b.y0 >= kRailPanelRow) continue;  // static rail fills
					LandPrevCmd &slot = landPrev_[backIndex][landPrevCount_[backIndex]];
					slot.pixels = isImage ? cmd.pixels : nullptr;
					slot.type = static_cast<std::int16_t>(cmd.type);
					slot.x = cmd.x;
					slot.y = cmd.y;
					slot.w = cmd.w;
					slot.h = cmd.h;
					landPrevCount_[backIndex]++;
				}
				if (!snapshotOverflow && landPrevCount_[backIndex] > 0) landPrevValid_[backIndex] = true;
				landPrevRailHash_[backIndex] = railHash;
				const int maxRows = std::max(1, kFlushRowsDefault);
				if (!panMode)
				for (int row = 0; row < height; row += maxRows) {
					int rows = maxRows;
					if (row + rows > height) rows = height - row;
					// Occlusion: skip the base fill when opaque blits cover the band.
					std::size_t firstCommand = 0;
					{
						int spans[16][2];
						int spanCount = 0;
						for (std::size_t i = 1; i < current.commands.size() && spanCount < 16; ++i) {
							const present::Command &c = current.commands[i];
							if (c.type == Type::FillText) continue;
							const present::Rect b = present::commandBounds(c, width, height);
							if (!present::valid(b) || b.y0 > row || b.y1 < row + rows - 1) continue;
							spans[spanCount][0] = std::max(0, b.x0);
							spans[spanCount][1] = std::min(width - 1, b.x1);
							spanCount++;
						}
						for (int a = 1; a < spanCount; ++a) {
							for (int b2 = a; b2 > 0 && spans[b2][0] < spans[b2 - 1][0]; --b2) {
								std::swap(spans[b2][0], spans[b2 - 1][0]);
								std::swap(spans[b2][1], spans[b2 - 1][1]);
							}
						}
						int reach = 0;
						for (int a = 0; a < spanCount; ++a) {
							if (spans[a][0] > reach) break;
							if (spans[a][1] + 1 > reach) reach = spans[a][1] + 1;
						}
						if (reach >= width) firstCommand = 1;
					}
					std::uint16_t *target = back + static_cast<std::size_t>(row) * kNativeWidth;
					const std::int64_t rasterStartUs = esp_timer_get_time();
					gea::framework::graphics::Canvas bandCanvas;
					bandCanvas.bindPixels(target, kNativeWidth, rows, kNativeWidth);
					for (std::size_t i = firstCommand; i < current.commands.size(); ++i) {
						const present::Command &cmd = current.commands[i];
						if (cmd.type == Type::FillText) continue;  // rotated full-buffer pass below
						if (cmd.type != Type::Clear) {
							const present::Rect bounds = present::commandBounds(cmd, width, height);
							if (!present::valid(bounds) || bounds.y1 < row || bounds.y0 > row + rows - 1) continue;
						}
						present::rasterCommandRows(bandCanvas, target, 0, kNativeWidth, row, rows, height, cmd);
					}
					flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;
				}
				landRasterUs_ = static_cast<int>(esp_timer_get_time() - started) - landAlignUs_;
				landPanMode_ = panMode ? 1 : 0;
				if (panMode) landPanFrames_++; else landFullFrames_++;
				// Sidebar text: rotated glyph blits over the full back buffer
				// (landscape anchors; the blitter does the panel transform per pixel).
				if (!panMode) {
					const std::int64_t textStartUs = esp_timer_get_time();
					gea::framework::graphics::Canvas fullCanvas;
					fullCanvas.bindPixels(back, kNativeWidth, kNativeHeight, kNativeWidth);
					for (const present::Command &cmd : current.commands) {
						if (cmd.type != Type::FillText || cmd.fontFamilyId < 0) continue;
						fullCanvas.drawTextFontFamilyRotated90(cmd.text.c_str(), cmd.x, cmd.y, cmd.color,
						                                       cmd.fontFamilyId, cmd.fontSizePx);
					}
					flushStats_.rasterUs += esp_timer_get_time() - textStartUs;
					landTextUs_ = static_cast<int>(esp_timer_get_time() - textStartUs);
				}
				take();
				const std::int64_t flipStartUs = esp_timer_get_time();
				drainRefreshDoneLocked();
				flipPanelToBack();
				lastFlipUs_ = esp_timer_get_time();
				flushStats_.copyUs += lastFlipUs_ - flipStartUs;
				landFlipUs_ = static_cast<int>(lastFlipUs_ - flipStartUs);
				give();
				landTotalUs_ = static_cast<int>(esp_timer_get_time() - started);
				flushStats_.totalUs += esp_timer_get_time() - started;
				flushStats_.chunkCount++;
				flushStats_.pixelCount += width * height;
				static int landHits = 0;
				landHits++;
				pathDirect_++;
				if ((landHits & 15) == 1) {
					ESP_LOGI(kTag, "landscape panel-native frame #%d total=%lldus", landHits,
					         static_cast<long long>(esp_timer_get_time() - started));
				}
				panelDirectActive_ = true;
				previousPresentValid_ = false;
				frameBufferCoherent_ = false;
				canvas_.resetDirty();
				return true;
			}
		}
		if (panelDirectActive_) {
			// A frame that ISN'T panel-direct is taking over: the scanout holds
			// tile content the composed path knows nothing about. Force a full
			// composed repaint + flush.
			panelDirectActive_ = false;
			previousPresentValid_ = false;
			needsFullPhysicalFlush_ = true;
			resetPanelBackBufferCoherenceLocked();
			for (int i = 0; i < kDpiFrameBufferCount; ++i) landPrevValid_[i] = false;
		}
			constexpr bool kUsePanelBufferLocalCirclePrev = false;
			if (kUsePanelBufferLocalCirclePrev &&
			    !asyncPresentInFlight_.load(std::memory_order_acquire) &&
			    isFastClearFillCirclesFrame(current) &&
			    canUseNativePanelBackBufferPath()) {
				int backIndex = -1;
				if (prepareNativePanelBackBufferForCircleFrame(&backIndex)) {
					const present::Frame *previousForBuffer =
						(backIndex >= 0 && backIndex < kDpiFrameBufferCount && panelCirclePrevValid_[backIndex] &&
						 isFastClearFillCirclesFrame(panelCirclePrevFrame_[backIndex]) && !needsFullPhysicalFlush_)
							? &panelCirclePrevFrame_[backIndex]
							: nullptr;
					const bool ok = presentClearFillCirclesFrame(current, previousForBuffer);
					if (ok) {
						if (backIndex >= 0 && backIndex < kDpiFrameBufferCount) {
							panelCirclePrevFrame_[backIndex] = std::move(current);
							panelCirclePrevValid_[backIndex] = true;
						}
						previousPresentValid_ = false;
						needsFullPhysicalFlush_ = false;
						frameBufferCoherent_ = true;
						canvas_.resetDirty();
					}
					return ok;
				}
			}
		invalidatePanelCircleFrames();
		{
			using gea::framework::display::DisplayOrientation;
			using Type = gea::platform::display::DisplayPresentCommandType;
			if (tileShape) {
				prepareDrawingTarget();
				const bool rotatedMode = !(orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary &&
				                           usingPanelFrameBufferDirectly());
				int shiftDx = 0;
				int shiftDy = 0;
				const int absLimitX = logicalWidth() / 2;
				const int absLimitY = logicalHeight() / 2;
				const bool panPreconditions = rotatedMode && frameBuffer_ == ownedFrameBuffer_ &&
				                              previousPresentValid_ && !needsFullPhysicalFlush_ && frameBufferCoherent_;
				const bool panShiftFound = panPreconditions && detectUniformShift(previousPresentFrame_, current, &shiftDx, &shiftDy);
					if (panShiftFound &&
					    (shiftDx < 0 ? -shiftDx : shiftDx) < absLimitX && (shiftDy < 0 ? -shiftDy : shiftDy) < absLimitY) {
						// PAN FAST PATH: the frame is the previous one uniformly shifted.
						// Move the torus origin, raster ONLY the exposed strips and any
						// commands without a shifted twin, then rebuild the panel with
						// 1-4 big PPA rotate ops. ~30-80k rastered pixels instead of 921k.
						static int panHits = 0;
						panHits++;
						const std::int64_t rasterUs0 = flushStats_.rasterUs;
						const std::int64_t copyUs0 = flushStats_.copyUs;
						const std::int64_t txUs0 = flushStats_.txUs;
						const std::int64_t started = esp_timer_get_time();
						flushStats_.callCount++;
						scrollOriginX_ = wrapCoord(scrollOriginX_ - shiftDx, logicalWidth());
						scrollOriginY_ = wrapCoord(scrollOriginY_ - shiftDy, logicalHeight());
						present::Rect rects[kMaxPresentRects];
						int rectCount = 0;
						if (shiftDx > 0)
							present::addRect(rects, &rectCount, kMaxPresentRects, {0, 0, shiftDx - 1, logicalHeight() - 1}, logicalWidth(), logicalHeight());
						else if (shiftDx < 0)
							present::addRect(rects, &rectCount, kMaxPresentRects, {logicalWidth() + shiftDx, 0, logicalWidth() - 1, logicalHeight() - 1},
							                 logicalWidth(), logicalHeight());
						if (shiftDy > 0)
							present::addRect(rects, &rectCount, kMaxPresentRects, {0, 0, logicalWidth() - 1, shiftDy - 1}, logicalWidth(), logicalHeight());
						else if (shiftDy < 0)
							present::addRect(rects, &rectCount, kMaxPresentRects, {0, logicalHeight() + shiftDy, logicalWidth() - 1, logicalHeight() - 1},
							                 logicalWidth(), logicalHeight());
						for (std::size_t i = 1; i < current.commands.size(); ++i) {
							const present::Command &c = current.commands[i];
							bool matchedCmd = false;
							for (std::size_t j = 1; j < previousPresentFrame_.commands.size() && !matchedCmd; ++j) {
								const present::Command &p = previousPresentFrame_.commands[j];
								matchedCmd = p.type == c.type && p.pixels == c.pixels && p.srcWidth == c.srcWidth &&
								             p.srcHeight == c.srcHeight &&
								             (c.type != Type::DrawImageScaled || (p.w == c.w && p.h == c.h)) &&
								             p.x + shiftDx == c.x && p.y + shiftDy == c.y;
							}
							if (!matchedCmd)
								present::addCommandBounds(c, rects, &rectCount, kMaxPresentRects, logicalWidth(), logicalHeight());
						}
						bool ok = true;
						for (int i = 0; i < rectCount && ok; ++i) {
							present::Rect region = clampLogicalRect(rects[i], logicalWidth(), logicalHeight());
							if (!present::valid(region)) continue;
							rasterPresentRegion(current, region);
							TorusPart parts[4];
							const int partCount = splitLogicalRectToTorus(region, parts);
							const std::int64_t syncStartUs = esp_timer_get_time();
							for (int p = 0; p < partCount && ok; ++p) ok = syncTorusPartToMemory(parts[p]);
							flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
						}
						if (ok && !panelSyncedForTorus_) {
							ok = syncPanelFrameBufferRectToMemory({0, 0, kNativeWidth - 1, kNativeHeight - 1});
							panelSyncedForTorus_ = true;
						}
						if (ok) ok = flushFullPanelFromTorus();
						flushStats_.totalUs += esp_timer_get_time() - started;
						if ((panHits & 15) == 1) {
							ESP_LOGI(kTag, "torus pan #%d dx=%d dy=%d rects=%d total=%lldus raster=%lldus sync=%lldus ppa=%lldus",
							         panHits, shiftDx, shiftDy, rectCount,
							         static_cast<long long>(esp_timer_get_time() - started),
							         static_cast<long long>(flushStats_.rasterUs - rasterUs0),
							         static_cast<long long>(flushStats_.copyUs - copyUs0),
							         static_cast<long long>(flushStats_.txUs - txUs0));
						}
						if (ok) {
							previousPresentFrame_ = std::move(current);
							previousPresentValid_ = true;
							frameBufferCoherent_ = true;
							canvas_.resetDirty();
						} else {
							scrollOriginX_ = 0;
							scrollOriginY_ = 0;
							panelSyncedForTorus_ = false;
							previousPresentValid_ = false;
							needsFullPhysicalFlush_ = true;
						}
						return ok;
					}
				}
			}
		if (!asyncPresentInFlight_.load(std::memory_order_acquire) &&
		    isFastClearFillCirclesFrame(current)) {
			prepareDrawingTarget(false);
			const bool ok = presentClearFillCirclesFrame(current, &previousPresentFrame_);
			if (ok) {
				previousPresentFrame_ = std::move(current);
				previousPresentValid_ = true;
				needsFullPhysicalFlush_ = false;
				canvas_.resetDirty();
			}
			return ok;
		}
		const bool forceFullPanelBackRepaint = shouldForceFullPanelBackRepaint(current);
		if (forceFullPanelBackRepaint && scheduleAsyncFullPanelBackRepaint(std::move(current))) {
			return true;
		}
		if (!waitForAsyncPresent()) return false;
		present::Rect regions[kMaxPresentRects];
		int regionCount = 0;
		if (forceFullPanelBackRepaint) {
			regions[0] = {0, 0, logicalWidth() - 1, logicalHeight() - 1};
			regionCount = 1;
		} else if (previousPresentValid_) {
			regionCount = present::dirtyRects(&previousPresentFrame_,
			                                  current,
			                                  regions,
			                                  kMaxPresentRects,
			                                  logicalWidth(),
			                                  logicalHeight());
		} else {
			regions[0] = {0, 0, logicalWidth() - 1, logicalHeight() - 1};
			regionCount = 1;
		}
		if (regionCount <= 0) {
			previousPresentFrame_ = std::move(current);
			previousPresentValid_ = true;
			return true;
		}
		prepareDrawingTarget(!regionsCoverLogicalFrame(regions, regionCount));

		const std::int64_t started = esp_timer_get_time();
		bool ok = true;
		pathGeneral_++;
		flushStats_.callCount++;
		// MEASURED LOSS, kept dark: per-tile PPA blits cost MORE than the CPU
		// raster (flush 53-89ms -> 84-182ms on the maps tile grid) because the
		// IDF PPA driver cache-syncs the full 1.8MB output framebuffer around
		// EVERY transaction — ~30 small ops pay ~30 full-buffer msyncs. The PPA
		// only wins as one big op per region (like the rotate flush). Re-try if
		// the driver ever grows a no-sync/explicit-sync mode.
		constexpr bool kUsePpaTileBlits = false;
		const bool ppaTiles = kUsePpaTileBlits && framePpaTileEligible(current);
		present::Rect flushedRects[kMaxPresentRects];
		int flushedCount = 0;
		for (int i = 0; i < regionCount; ++i) {
			present::Rect region = clampLogicalRect(regions[i], logicalWidth(), logicalHeight());
			if (!present::valid(region)) continue;
			bool rastered = false;
			if (ppaTiles) {
				const std::int64_t ppaStartUs = esp_timer_get_time();
				rastered = ppaRasterTileRegion(current, region);
				flushStats_.rasterUs += esp_timer_get_time() - ppaStartUs;
			}
			if (!rastered) rasterPresentRegion(current, region);
			present::addRect(flushedRects, &flushedCount, kMaxPresentRects, region, logicalWidth(), logicalHeight());
		}
		ok = flushPreparedLogicalRects(flushedRects, flushedCount);
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) {
			previousPresentFrame_ = std::move(current);
			previousPresentValid_ = true;
			needsFullPhysicalFlush_ = false;
			frameBufferCoherent_ = true;
			canvas_.resetDirty();
		}
		return ok;
		}

	bool shouldForceFullPanelBackRepaint(const present::Frame &current) const
	{
		if (!canUseNativePanelBackBufferPath() || scrollOriginX_ != 0 || scrollOriginY_ != 0) return false;
		if (!present::frameHasOpaqueBase(current, logicalWidth(), logicalHeight())) return false;
		if (!isFastClearFillCirclesFrame(current) || current.commands[1].circlesCount < 256) return false;
		return true;
	}

	// A frame whose base layer is [Clear-or-full-fill, FillCircles]. Commands AFTER
	// the circle batch (FPS text, a HUD rect, ...) are overlays drawn on top — they
	// ride the incremental path via a per-frame overlay re-raster, so they no longer
	// disqualify it. Direct-canvas ctx.clear() currently emits a full-screen fill
	// rect, so accepting both base forms keeps generated apps on this path.
	bool isFastClearFillCirclesFrame(const present::Frame &frame) const
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		if (frame.commands.size() < 2) return false;
		const present::Command &base = frame.commands[0];
		const present::Command &circles = frame.commands[1];
		const bool clearBase = base.type == Type::Clear;
		const bool fillBase = base.type == Type::FillRectRgb565 &&
		                      base.alpha == 255 &&
		                      base.x <= 0 &&
		                      base.y <= 0 &&
		                      base.x + base.w >= logicalWidth() &&
		                      base.y + base.h >= logicalHeight();
		if ((!clearBase && !fillBase) || circles.type != Type::FillCirclesRgb565) return false;
		if (circles.alpha != 255 || circles.radius <= 0 || circles.circlesCount < 0) return false;
		return circles.hasCircleEntries();
	}

	// The incremental circle path erases + redraws every ball (2x scattered writes
	// over the whole framebuffer — cache-thrashing) to save the cache writeback on
	// the untouched area. That trade only pays when the touched area is small; for a
	// dense/full-screen ball field the general chunked rasterizer is faster (measured
	// 21.6 vs 18.2fps for 1000 balls). So gate the incremental path on the would-be
	// dirty area, mirroring the amoled's geometric band-collapse decision.
	bool incrementalCirclePathBeneficial(const present::Frame &previous, const present::Frame &current) const
	{
		if (!isFastClearFillCirclesFrame(previous)) return false;
		if (previous.commands[0].color != current.commands[0].color) return false;
		present::Rect dirtyRects[kMaxFlushRects];
		int dirtyCount = 0;
		addCommandDirtyRectsTight(previous.commands[1], dirtyRects, &dirtyCount, kMaxFlushRects);
		addCommandDirtyRectsTight(current.commands[1], dirtyRects, &dirtyCount, kMaxFlushRects);
		for (std::size_t i = 2; i < current.commands.size(); ++i)
			addCommandDirtyRectsTight(current.commands[i], dirtyRects, &dirtyCount, kMaxFlushRects);
		for (std::size_t i = 2; i < previous.commands.size(); ++i)
			addCommandDirtyRectsTight(previous.commands[i], dirtyRects, &dirtyCount, kMaxFlushRects);
		if (dirtyCount <= 0) return true;  // nothing moved — trivially cheap
		const long long dirtyArea = rectAreaSum(dirtyRects, dirtyCount);
		const long long screenArea = static_cast<long long>(logicalWidth()) * logicalHeight();
		return dirtyArea < screenArea;
	}

	bool incrementalCirclePathBeneficial(const present::Frame &current) const
	{
		if (!previousPresentValid_ || needsFullPhysicalFlush_) return false;
		return incrementalCirclePathBeneficial(previousPresentFrame_, current);
	}

	bool smallCircleDeltaEligible(const present::Frame &previous, const present::Frame &current) const
	{
		if (!isFastClearFillCirclesFrame(previous) || !isFastClearFillCirclesFrame(current)) return false;
		const present::Command &oldCircles = previous.commands[1];
		const present::Command &newCircles = current.commands[1];
		return oldCircles.radius == newCircles.radius &&
		       oldCircles.circlesCount <= kCircleDeltaMax &&
		       newCircles.circlesCount <= kCircleDeltaMax;
	}

	bool presentClearFillCirclesFrame(const present::Frame &current, const present::Frame *previousForBuffer)
	{
		const present::Command &clear = current.commands[0];
		const present::Command &circles = current.commands[1];
		const bool previousFast =
			previousForBuffer &&
			isFastClearFillCirclesFrame(*previousForBuffer) &&
			previousForBuffer->commands[0].color == clear.color;
		const bool smallDelta = previousFast && smallCircleDeltaEligible(*previousForBuffer, current);
		const bool canIncremental =
			previousFast &&
			(smallDelta || incrementalCirclePathBeneficial(*previousForBuffer, current));
		constexpr bool kUseIncrementalCircleDirty = false;
		const bool incremental = kUseIncrementalCircleDirty && canIncremental && !needsFullPhysicalFlush_;

		// Overlays = every command past the circle batch (e.g. the FPS text). They
		// sit on top of the balls and don't move with them, so their box — current
		// AND previous (to erase the stale overlay) — is repainted independently and
		// re-composited from scratch. Empty when the frame is pure [Clear, FillCircles].
		present::Rect overlayRects[kMaxFlushRects];
		int overlayCount = 0;
		const bool scalarCircleDelta = incremental && scalarCircleOverlayDeltaEligible(*previousForBuffer, current);
		for (std::size_t i = 2; i < current.commands.size(); ++i) {
			if (incremental) {
				if (scalarCircleDelta && isScalarCircleOverlayCommand(current.commands[i])) continue;
			} else if (isCircleOverlayCommand(current.commands[i])) {
				continue;
			}
			addCommandDirtyRectsTight(current.commands[i], overlayRects, &overlayCount, kMaxFlushRects);
		}
		if (incremental)
			for (std::size_t i = 2; i < previousForBuffer->commands.size(); ++i) {
				if (scalarCircleDelta && isScalarCircleOverlayCommand(previousForBuffer->commands[i])) continue;
				addCommandDirtyRectsTight(previousForBuffer->commands[i], overlayRects, &overlayCount, kMaxFlushRects);
			}

		present::Rect dirtyRects[kMaxFlushRects];
		int dirtyCount = 0;
		if (!incremental) {
			dirtyRects[0] = present::fullScreen(logicalWidth(), logicalHeight());
			dirtyCount = 1;
		}

		const std::int64_t started = esp_timer_get_time();
		flushStats_.callCount++;
		const std::int64_t rasterStartUs = esp_timer_get_time();
		if (incremental) {
			const bool deltaDrawn = drawCircleBatchDeltaRaw(previousForBuffer->commands[1],
			                                                circles,
			                                                clear.color,
			                                                dirtyRects,
			                                                &dirtyCount,
			                                                kMaxFlushRects);
			if (!deltaDrawn) {
				addCommandDirtyRectsTight(previousForBuffer->commands[1], dirtyRects, &dirtyCount, kMaxFlushRects);
				addCommandDirtyRectsTight(circles, dirtyRects, &dirtyCount, kMaxFlushRects);
				drawCircleBatchRaw(previousForBuffer->commands[1], clear.color, true);
				drawCircleBatchRaw(circles, 0, false);
			}
			if (scalarCircleDelta)
				drawScalarCircleOverlayDelta(*previousForBuffer, current, clear.color, dirtyRects, &dirtyCount, kMaxFlushRects);
		} else {
			clearFrameBufferRows(0, logicalHeight() - 1, clear.color);
			drawCircleBatchRaw(circles, 0, false);
			drawScalarCircleOverlaysRaw(current);
		}
		// Composite overlays: re-raster their box from the full frame (clear + the
		// balls behind them + the overlay itself), so the old overlay is erased and
		// the new one lands on top of whatever balls sit behind it.
		for (int i = 0; i < overlayCount; ++i) {
			rasterPresentRegion(current, overlayRects[i]);
			if (incremental)
				addRectTight(dirtyRects, &dirtyCount, kMaxFlushRects, overlayRects[i], logicalWidth(), logicalHeight());
		}
		flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;

		if (dirtyCount <= 0) return true;
		const bool ok = flushPreparedLogicalRects(dirtyRects, dirtyCount);
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) frameBufferCoherent_ = true;
		return ok;
	}

	present::Rect circleBatchBounds(const present::Command &command) const
	{
		present::Rect bounds{};
		const auto *circles = command.circleEntries();
		if (!circles || command.circlesCount <= 0) return bounds;
		for (int i = 0; i < command.circlesCount; ++i) {
			bounds = present::unite(bounds,
			                        present::circleBounds(static_cast<int>(circles[i].x),
			                                              static_cast<int>(circles[i].y),
			                                              command.radius));
		}
		return bounds;
	}

	void addCommandDirtyRectsTight(const present::Command &command,
	                               present::Rect *rects,
	                               int *count,
	                               int capacity) const
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		if (command.type == Type::FillCirclesRgb565) {
			const auto *circles = command.circleEntries();
			if (!circles || command.circlesCount <= 0) return;
			for (int i = 0; i < command.circlesCount; ++i) {
				addRectTight(rects,
				             count,
				             capacity,
				             present::circleBounds(static_cast<int>(circles[i].x),
				                                   static_cast<int>(circles[i].y),
				                                   command.radius),
				             logicalWidth(),
				             logicalHeight());
			}
			return;
		}
		addRectTight(rects,
		             count,
		             capacity,
		             present::commandBounds(command, logicalWidth(), logicalHeight()),
		             logicalWidth(),
		             logicalHeight());
	}

	void clearFrameBufferRows(int y0, int y1, std::uint16_t color)
	{
		if (!frameBuffer_) return;
		y0 = std::max(0, y0);
		y1 = std::min(logicalHeight() - 1, y1);
		if (y0 > y1) return;
		const int width = logicalWidth();
		const int rows = y1 - y0 + 1;
		std::uint16_t *dst = frameBuffer_ + static_cast<std::size_t>(y0) * logicalStride_;
		if (logicalStride_ == width) {
			const std::size_t pixels = static_cast<std::size_t>(width) * rows;
			if (color == 0) std::memset(dst, 0, pixels * sizeof(std::uint16_t));
			else std::fill_n(dst, pixels, color);
			return;
		}
		for (int y = y0; y <= y1; ++y) {
			std::uint16_t *row = frameBuffer_ + static_cast<std::size_t>(y) * logicalStride_;
			if (color == 0) std::memset(row, 0, static_cast<std::size_t>(width) * sizeof(std::uint16_t));
			else std::fill_n(row, width, color);
		}
	}

	int circleHalfWidth(int r, int dy)
	{
		if (r <= 0) return 0;
		const int ady = dy < 0 ? -dy : dy;
		if (ady > r) return 0;
		if (r <= kCircleSpanCacheMaxRadius) {
			if (circleSpanCacheRadius_ != r) {
				for (int y = 0; y <= r; ++y) {
					circleSpanHalfWidths_[y] = integerSqrt(r * r - y * y);
				}
				circleSpanCacheRadius_ = r;
			}
			return circleSpanHalfWidths_[ady];
		}
		return integerSqrt(r * r - dy * dy);
	}

	void fillFrameBufferSpanRaw(int y, int x0, int x1, std::uint16_t color)
	{
		if (!frameBuffer_ || y < 0 || y >= logicalHeight()) return;
		if (x0 < 0) x0 = 0;
		if (x1 >= logicalWidth()) x1 = logicalWidth() - 1;
		if (x0 > x1) return;
		std::uint16_t *dst = frameBuffer_ + static_cast<std::size_t>(y) * logicalStride_ + x0;
		const int count = x1 - x0 + 1;
		if (color == 0) std::memset(dst, 0, static_cast<std::size_t>(count) * sizeof(std::uint16_t));
		else std::fill_n(dst, count, color);
	}

	void fillFrameBufferColumnRaw(int x, int y0, int y1, std::uint16_t color)
	{
		if (!frameBuffer_ || x < 0 || x >= logicalWidth()) return;
		if (y0 < 0) y0 = 0;
		if (y1 >= logicalHeight()) y1 = logicalHeight() - 1;
		if (y0 > y1) return;
		std::uint16_t *dst = frameBuffer_ + static_cast<std::size_t>(y0) * logicalStride_ + x;
		const int stride = logicalStride_;
		for (int y = y0; y <= y1; ++y) {
			*dst = color;
			dst += stride;
		}
	}

	void drawCircleRaw(int cx, int cy, int r, std::uint16_t color)
	{
		if (!frameBuffer_ || r <= 0) return;
		int dy0 = -r;
		int dy1 = r;
		if (cy + dy0 < 0) dy0 = -cy;
		if (cy + dy1 >= logicalHeight()) dy1 = logicalHeight() - 1 - cy;
		if (dy0 > dy1) return;
		for (int dy = dy0; dy <= dy1; ++dy) {
			const int halfWidth = circleHalfWidth(r, dy);
			fillFrameBufferSpanRaw(cy + dy, cx - halfWidth, cx + halfWidth, color);
		}
	}

	void drawCircleDifferenceRaw(int cx, int cy, int r, int otherCx, int otherCy, int otherR, std::uint16_t color)
	{
		if (!frameBuffer_ || r <= 0) return;
		int y0 = cy - r;
		int y1 = cy + r;
		if (y0 < 0) y0 = 0;
		if (y1 >= logicalHeight()) y1 = logicalHeight() - 1;
		if (y0 > y1) return;
		for (int y = y0; y <= y1; ++y) {
			const int dy = y - cy;
			const int halfWidth = circleHalfWidth(r, dy);
			int x0 = cx - halfWidth;
			int x1 = cx + halfWidth;
			const int ody = y - otherCy;
			const int aody = ody < 0 ? -ody : ody;
			if (otherR <= 0 || aody > otherR) {
				fillFrameBufferSpanRaw(y, x0, x1, color);
				continue;
			}
			const int otherHalfWidth = circleHalfWidth(otherR, ody);
			const int ox0 = otherCx - otherHalfWidth;
			const int ox1 = otherCx + otherHalfWidth;
			if (x0 < ox0) fillFrameBufferSpanRaw(y, x0, std::min(x1, ox0 - 1), color);
			if (x1 > ox1) fillFrameBufferSpanRaw(y, std::max(x0, ox1 + 1), x1, color);
		}
	}

	void drawCircleDifferenceRaw(int cx, int cy, int otherCx, int otherCy, int r, std::uint16_t color)
	{
		drawCircleDifferenceRaw(cx, cy, r, otherCx, otherCy, r, color);
	}

	bool drawHorizontalCircleShiftDeltaRaw(int oldCx,
	                                       int oldCy,
	                                       int newCx,
	                                       int newCy,
	                                       int r,
	                                       std::uint16_t clearColor,
	                                       std::uint16_t newColor)
	{
		if (!frameBuffer_ || r <= 0 || oldCy != newCy) return false;
		const int dx = newCx - oldCx;
		const int adx = dx < 0 ? -dx : dx;
		if (adx <= 0 || adx > kCircleDeltaMaxShift) return false;
		if (dx > 0) {
			for (int i = 0; i < dx; ++i) {
				const int clearX = oldCx - r + i;
				const int clearHalfHeight = circleHalfWidth(r, clearX - oldCx);
				fillFrameBufferColumnRaw(clearX, oldCy - clearHalfHeight, oldCy + clearHalfHeight, clearColor);
				const int drawX = oldCx + r + 1 + i;
				const int drawHalfHeight = circleHalfWidth(r, drawX - newCx);
				fillFrameBufferColumnRaw(drawX, newCy - drawHalfHeight, newCy + drawHalfHeight, newColor);
			}
		} else {
			for (int i = 0; i < adx; ++i) {
				const int clearX = oldCx + r - i;
				const int clearHalfHeight = circleHalfWidth(r, clearX - oldCx);
				fillFrameBufferColumnRaw(clearX, oldCy - clearHalfHeight, oldCy + clearHalfHeight, clearColor);
				const int drawX = oldCx - r - 1 - i;
				const int drawHalfHeight = circleHalfWidth(r, drawX - newCx);
				fillFrameBufferColumnRaw(drawX, newCy - drawHalfHeight, newCy + drawHalfHeight, newColor);
			}
		}
		return true;
	}

	bool appendCircleShiftDirtyRects(int oldCx,
	                                 int oldCy,
	                                 int newCx,
	                                 int newCy,
	                                 int r,
	                                 present::Rect *rects,
	                                 int *count,
	                                 int capacity) const
	{
		const int dx = newCx - oldCx;
		const int dy = newCy - oldCy;
		const int adx = dx < 0 ? -dx : dx;
		const int ady = dy < 0 ? -dy : dy;
		if (dx == 0 && dy == 0) return true;
		if (dy == 0 && adx <= kCircleDeltaMaxShift) {
			if (dx > 0) {
				addRectTight(rects, count, capacity, {oldCx - r, oldCy - r, oldCx - r + dx - 1, oldCy + r}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {oldCx + r + 1, oldCy - r, oldCx + r + dx, oldCy + r}, logicalWidth(), logicalHeight());
			} else {
				addRectTight(rects, count, capacity, {oldCx + r + dx + 1, oldCy - r, oldCx + r, oldCy + r}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {oldCx + dx - r, oldCy - r, oldCx - r - 1, oldCy + r}, logicalWidth(), logicalHeight());
			}
			return true;
		}
		if (dx == 0 && ady <= kCircleDeltaMaxShift) {
			if (dy > 0) {
				addRectTight(rects, count, capacity, {oldCx - r, oldCy - r, oldCx + r, oldCy - r + dy - 1}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {oldCx - r, oldCy + r + 1, oldCx + r, oldCy + r + dy}, logicalWidth(), logicalHeight());
			} else {
				addRectTight(rects, count, capacity, {oldCx - r, oldCy + r + dy + 1, oldCx + r, oldCy + r}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {oldCx - r, oldCy + dy - r, oldCx + r, oldCy - r - 1}, logicalWidth(), logicalHeight());
			}
			return true;
		}
		return false;
	}

	bool isScalarCircleOverlayCommand(const present::Command &command) const
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		return command.type == Type::FillCircleRgb565 && command.alpha == 255 && command.radius > 0;
	}

	bool isCircleOverlayCommand(const present::Command &command) const
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		if (isScalarCircleOverlayCommand(command)) return true;
		return command.type == Type::FillCirclesRgb565 &&
		       command.alpha == 255 &&
		       command.radius > 0 &&
		       command.circlesCount > 0 &&
		       command.hasCircleEntries();
	}

	int collectScalarCircleOverlayIndices(const present::Frame &frame, int *indices, int capacity) const
	{
		if (!indices || capacity <= 0) return 0;
		int count = 0;
		for (std::size_t i = 2; i < frame.commands.size(); ++i) {
			if (!isScalarCircleOverlayCommand(frame.commands[i])) continue;
			if (count >= capacity) return capacity + 1;
			indices[count++] = static_cast<int>(i);
		}
		return count;
	}

	bool scalarCircleOverlayDeltaEligible(const present::Frame &previous, const present::Frame &current) const
	{
		int previousIndices[kCircleDeltaMax];
		int currentIndices[kCircleDeltaMax];
		const int previousCount = collectScalarCircleOverlayIndices(previous, previousIndices, kCircleDeltaMax);
		const int currentCount = collectScalarCircleOverlayIndices(current, currentIndices, kCircleDeltaMax);
		return previousCount <= kCircleDeltaMax && currentCount <= kCircleDeltaMax;
	}

	void appendCircleDeltaDirtyRects(int oldCx,
	                                 int oldCy,
	                                 int oldR,
	                                 int newCx,
	                                 int newCy,
	                                 int newR,
	                                 present::Rect *rects,
	                                 int *count,
	                                 int capacity) const
	{
		if (oldCx == newCx && oldCy == newCy && oldR == newR) return;
		if (oldR == newR &&
		    appendCircleShiftDirtyRects(oldCx, oldCy, newCx, newCy, oldR, rects, count, capacity)) {
			return;
		}
		const int dx = newCx - oldCx;
		const int dy = newCy - oldCy;
		const int dr = newR - oldR;
		const int adx = dx < 0 ? -dx : dx;
		const int ady = dy < 0 ? -dy : dy;
		const int adr = dr < 0 ? -dr : dr;
		if (adx + adr <= kCircleDeltaMaxShift && ady + adr <= kCircleDeltaMaxShift) {
			const int ux0 = std::min(oldCx - oldR, newCx - newR);
			const int uy0 = std::min(oldCy - oldR, newCy - newR);
			const int ux1 = std::max(oldCx + oldR, newCx + newR);
			const int uy1 = std::max(oldCy + oldR, newCy + newR);
			const int sideW = adx + adr;
			const int sideH = ady + adr;
			if (sideW > 0) {
				addRectTight(rects, count, capacity, {ux0, uy0, ux0 + sideW - 1, uy1}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {ux1 - sideW + 1, uy0, ux1, uy1}, logicalWidth(), logicalHeight());
			}
			if (sideH > 0) {
				addRectTight(rects, count, capacity, {ux0, uy0, ux1, uy0 + sideH - 1}, logicalWidth(), logicalHeight());
				addRectTight(rects, count, capacity, {ux0, uy1 - sideH + 1, ux1, uy1}, logicalWidth(), logicalHeight());
			}
			return;
		}
		addRectTight(rects, count, capacity, present::circleBounds(oldCx, oldCy, oldR), logicalWidth(), logicalHeight());
		addRectTight(rects, count, capacity, present::circleBounds(newCx, newCy, newR), logicalWidth(), logicalHeight());
	}

	void drawScalarCircleOverlaysRaw(const present::Frame &current)
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		for (std::size_t i = 2; i < current.commands.size(); ++i) {
			const present::Command &command = current.commands[i];
			if (isScalarCircleOverlayCommand(command)) {
				drawCircleRaw(command.x, command.y, command.radius, command.color);
				continue;
			}
			if (command.type == Type::FillCirclesRgb565 && command.alpha == 255) {
				drawCircleBatchRaw(command, 0, false);
			}
		}
	}

	void drawScalarCircleOverlayDelta(const present::Frame &previous,
	                                  const present::Frame &current,
	                                  std::uint16_t clearColor,
	                                  present::Rect *dirtyRects,
	                                  int *dirtyCount,
	                                  int dirtyCapacity)
	{
		int previousIndices[kCircleDeltaMax];
		int currentIndices[kCircleDeltaMax];
		const int previousCount = collectScalarCircleOverlayIndices(previous, previousIndices, kCircleDeltaMax);
		const int currentCount = collectScalarCircleOverlayIndices(current, currentIndices, kCircleDeltaMax);
		if (previousCount > kCircleDeltaMax || currentCount > kCircleDeltaMax) return;

		int oldToNew[kCircleDeltaMax];
		int newToOld[kCircleDeltaMax];
		std::int8_t oldMode[kCircleDeltaMax];
		for (int i = 0; i < kCircleDeltaMax; ++i) {
			oldToNew[i] = -1;
			newToOld[i] = -1;
			oldMode[i] = 0;
		}

		for (int i = 0; i < previousCount; ++i) {
			const present::Command &oldCircle = previous.commands[previousIndices[i]];
			int best = -1;
			int bestDistance = 0x7fffffff;
			for (int j = 0; j < currentCount; ++j) {
				if (newToOld[j] >= 0) continue;
				const present::Command &newCircle = current.commands[currentIndices[j]];
				if (newCircle.color != oldCircle.color) continue;
				const int dx = newCircle.x - oldCircle.x;
				const int dy = newCircle.y - oldCircle.y;
				const int dr = newCircle.radius - oldCircle.radius;
				const int distance = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy) + (dr < 0 ? -dr : dr);
				if (distance < bestDistance) {
					bestDistance = distance;
					best = j;
				}
			}
			if (best >= 0) {
				oldToNew[i] = best;
				newToOld[best] = i;
			}
		}

		for (int i = 0; i < previousCount; ++i) {
			const present::Command &oldCircle = previous.commands[previousIndices[i]];
			const int j = oldToNew[i];
			if (j < 0) {
				addRectTight(dirtyRects,
				             dirtyCount,
				             dirtyCapacity,
				             present::circleBounds(oldCircle.x, oldCircle.y, oldCircle.radius),
				             logicalWidth(),
				             logicalHeight());
				drawCircleRaw(oldCircle.x, oldCircle.y, oldCircle.radius, clearColor);
				continue;
			}
			const present::Command &newCircle = current.commands[currentIndices[j]];
			if (oldCircle.x == newCircle.x && oldCircle.y == newCircle.y && oldCircle.radius == newCircle.radius) {
				oldMode[i] = 1;
				continue;
			}
			oldMode[i] = 2;
			appendCircleDeltaDirtyRects(oldCircle.x,
			                            oldCircle.y,
			                            oldCircle.radius,
			                            newCircle.x,
			                            newCircle.y,
			                            newCircle.radius,
			                            dirtyRects,
			                            dirtyCount,
			                            dirtyCapacity);
			drawCircleDifferenceRaw(oldCircle.x,
			                        oldCircle.y,
			                        oldCircle.radius,
			                        newCircle.x,
			                        newCircle.y,
			                        newCircle.radius,
			                        clearColor);
		}

		for (int j = 0; j < currentCount; ++j) {
			const present::Command &newCircle = current.commands[currentIndices[j]];
			const int i = newToOld[j];
			if (i < 0) {
				addRectTight(dirtyRects,
				             dirtyCount,
				             dirtyCapacity,
				             present::circleBounds(newCircle.x, newCircle.y, newCircle.radius),
				             logicalWidth(),
				             logicalHeight());
				drawCircleRaw(newCircle.x, newCircle.y, newCircle.radius, newCircle.color);
				continue;
			}
			if (oldMode[i] == 1) continue;
			const present::Command &oldCircle = previous.commands[previousIndices[i]];
			drawCircleDifferenceRaw(newCircle.x,
			                        newCircle.y,
			                        newCircle.radius,
			                        oldCircle.x,
			                        oldCircle.y,
			                        oldCircle.radius,
			                        newCircle.color);
		}
	}

	bool drawCircleBatchDeltaRaw(const present::Command &previous,
	                             const present::Command &current,
	                             std::uint16_t clearColor,
	                             present::Rect *dirtyRects,
	                             int *dirtyCount,
	                             int dirtyCapacity)
	{
		if (!frameBuffer_ || previous.radius <= 0 || current.radius != previous.radius) return false;
		if (previous.circlesCount < 0 || current.circlesCount < 0 ||
		    previous.circlesCount > kCircleDeltaMax || current.circlesCount > kCircleDeltaMax) {
			return false;
		}
		const auto *oldCircles = previous.circleEntries();
		const auto *newCircles = current.circleEntries();
		if ((previous.circlesCount > 0 && !oldCircles) || (current.circlesCount > 0 && !newCircles)) return false;

		int oldToNew[kCircleDeltaMax];
		int newToOld[kCircleDeltaMax];
		std::int8_t oldMode[kCircleDeltaMax];
		for (int i = 0; i < kCircleDeltaMax; ++i) {
			oldToNew[i] = -1;
			newToOld[i] = -1;
			oldMode[i] = 0;
		}

		for (int i = 0; i < previous.circlesCount; ++i) {
			const auto &oldCircle = oldCircles[i];
			int best = -1;
			int bestDistance = 0x7fffffff;
			for (int j = 0; j < current.circlesCount; ++j) {
				if (newToOld[j] >= 0) continue;
				const auto &newCircle = newCircles[j];
				if (newCircle.color != oldCircle.color) continue;
				const int dx = static_cast<int>(newCircle.x) - static_cast<int>(oldCircle.x);
				const int dy = static_cast<int>(newCircle.y) - static_cast<int>(oldCircle.y);
				const int distance = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
				if (distance < bestDistance) {
					bestDistance = distance;
					best = j;
				}
			}
			if (best >= 0) {
				oldToNew[i] = best;
				newToOld[best] = i;
			}
		}

		const int r = current.radius;
		for (int i = 0; i < previous.circlesCount; ++i) {
			const auto &oldCircle = oldCircles[i];
			const int oldCx = static_cast<int>(oldCircle.x);
			const int oldCy = static_cast<int>(oldCircle.y);
			const int j = oldToNew[i];
			if (j < 0) {
				addRectTight(dirtyRects, dirtyCount, dirtyCapacity, present::circleBounds(oldCx, oldCy, r), logicalWidth(), logicalHeight());
				drawCircleRaw(oldCx, oldCy, r, clearColor);
				continue;
			}
			const auto &newCircle = newCircles[j];
			const int newCx = static_cast<int>(newCircle.x);
			const int newCy = static_cast<int>(newCircle.y);
			if (oldCx == newCx && oldCy == newCy) {
				oldMode[i] = 1;
				continue;
			}
			if (appendCircleShiftDirtyRects(oldCx, oldCy, newCx, newCy, r, dirtyRects, dirtyCount, dirtyCapacity)) {
				if (drawHorizontalCircleShiftDeltaRaw(oldCx,
				                                      oldCy,
				                                      newCx,
				                                      newCy,
				                                      r,
				                                      clearColor,
				                                      newCircle.color)) {
					oldMode[i] = 4;
				} else {
					oldMode[i] = 2;
					drawCircleDifferenceRaw(oldCx, oldCy, newCx, newCy, r, clearColor);
				}
			} else {
				oldMode[i] = 3;
				addRectTight(dirtyRects, dirtyCount, dirtyCapacity, present::circleBounds(oldCx, oldCy, r), logicalWidth(), logicalHeight());
				addRectTight(dirtyRects, dirtyCount, dirtyCapacity, present::circleBounds(newCx, newCy, r), logicalWidth(), logicalHeight());
				drawCircleRaw(oldCx, oldCy, r, clearColor);
			}
		}

		for (int j = 0; j < current.circlesCount; ++j) {
			const auto &newCircle = newCircles[j];
			const int newCx = static_cast<int>(newCircle.x);
			const int newCy = static_cast<int>(newCircle.y);
			const int i = newToOld[j];
			if (i < 0) {
				addRectTight(dirtyRects, dirtyCount, dirtyCapacity, present::circleBounds(newCx, newCy, r), logicalWidth(), logicalHeight());
				drawCircleRaw(newCx, newCy, r, newCircle.color);
				continue;
			}
			if (oldMode[i] == 1 || oldMode[i] == 4) continue;
			const auto &oldCircle = oldCircles[i];
			if (oldMode[i] == 2)
				drawCircleDifferenceRaw(newCx, newCy, static_cast<int>(oldCircle.x), static_cast<int>(oldCircle.y), r, newCircle.color);
			else
				drawCircleRaw(newCx, newCy, r, newCircle.color);
		}
		return true;
	}

	void drawCircleBatchRaw(const present::Command &command, std::uint16_t overrideColor, bool useOverrideColor)
	{
		if (!frameBuffer_ || command.radius <= 0 || command.circlesCount <= 0) return;
		const auto *circles = command.circleEntries();
		if (!circles) return;
		const int width = logicalWidth();
		const int height = logicalHeight();
		const int r = command.radius;
		for (int i = 0; i < command.circlesCount; ++i) {
			const int cx = static_cast<int>(circles[i].x);
			const int cy = static_cast<int>(circles[i].y);
			const std::uint16_t color = useOverrideColor ? overrideColor : circles[i].color;
			int dy0 = -r;
			int dy1 = r;
			if (cy + dy0 < 0) dy0 = -cy;
			if (cy + dy1 >= height) dy1 = height - 1 - cy;
			if (dy0 > dy1) continue;
			for (int dy = dy0; dy <= dy1; ++dy) {
				const int halfWidth = circleHalfWidth(r, dy);
				int x0 = cx - halfWidth;
				int x1 = cx + halfWidth;
				if (x0 < 0) x0 = 0;
				if (x1 >= width) x1 = width - 1;
				if (x0 > x1) continue;
				std::uint16_t *dst = frameBuffer_ + static_cast<std::size_t>(cy + dy) * logicalStride_ + x0;
				const int count = x1 - x0 + 1;
				if (color == 0) std::memset(dst, 0, static_cast<std::size_t>(count) * sizeof(std::uint16_t));
				else std::fill_n(dst, count, color);
			}
		}
	}

	void setFlushConfig(int chunkRows, int queueDepth)
	{
		if (chunkRows > 0) flushRows_ = std::min(chunkRows, kFlushRowsDefault);
		if (queueDepth > 0) flushDepth_ = queueDepth;
	}

	int flushChunkRows() const { return flushRows_; }
	int flushQueueDepth() const { return flushDepth_; }
	int flushBufferBytes() const { return 0; }

	bool copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height)
	{
		const int lw = logicalWidth();
		const int lh = logicalHeight();
		if (!dst || pixelCapacity < lw * lh) return false;
		if (width) *width = lw;
		if (height) *height = lh;
		// Panel-direct modes raster straight into the scanout buffers and leave
		// the composed framebuffer stale — snapshot what the GLASS shows by
		// reading the scanned panel buffer through the inverse orientation map.
		if (panelDirectActive_ && panelFrameBuffers_[scannedFbIndex_]) {
			const std::uint16_t *panel = panelFrameBuffers_[scannedFbIndex_];
			for (int ly = 0; ly < lh; ++ly) {
				std::uint16_t *out = dst + static_cast<std::size_t>(ly) * lw;
				for (int lx = 0; lx < lw; ++lx) {
					const PanelCoord pc = panelForSourcePixel(lx, ly);
					out[lx] = panel[static_cast<std::size_t>(pc.y) * kNativeWidth + pc.x];
				}
			}
			return true;
		}
		for (int row = 0; row < lh; ++row) {
			std::memcpy(dst + static_cast<std::size_t>(row) * lw,
			            frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_,
			            static_cast<std::size_t>(lw) * sizeof(std::uint16_t));
		}
		return true;
	}

	int countNonBlackPixels() const
	{
		if (!frameBuffer_) return 0;
		int count = 0;
		for (int y = 0; y < logicalHeight(); ++y) {
			const std::uint16_t *row = frameBuffer_ + static_cast<std::size_t>(y) * logicalStride_;
			for (int x = 0; x < logicalWidth(); ++x) {
				if (pixel::toRgb565(row[x]) != 0) ++count;
			}
		}
		return count;
	}

	void setBrightness(int brightness)
	{
		if (brightness < 0) brightness = 0;
		if (brightness > 100) brightness = 100;
		brightness_ = brightness;
		if (brightnessInitialized_) updateBacklightDuty();
	}

	int brightness() const { return brightness_; }

	platform_display::DisplayFlushPerfStats flushStats() const { return flushStats_; }
	void presentPathDebug(int *calls, int *direct, int *general, int *rejected, int *tsKind, int *tsType, int *tsCount) const
	{
		*calls = presentCalls_;
		*direct = pathDirect_;
		*general = pathGeneral_;
		*rejected = pathRejected_;
		*tsKind = gTileShapeFailKind;
		*tsType = gTileShapeFailType;
		*tsCount = gTileShapeFailCount;
	}
	void landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior) const
	{
		*detect = panBailCounts_[1] + panBailCounts_[2] * 1000 + panBailCounts_[3] * 1000000;
		*kick = panBailCounts_[4] + panBailCounts_[5] * 1000 + panBailCounts_[6] * 1000000;
		*strips = panStripUs_;
		*wait = panBailReason_;
		*interior = panInteriorUs_ & 0xffffff;
	}
	void landFrameDebug(int *total, int *align, int *raster, int *text, int *flip) const
	{
		*total = landTotalUs_;
		*align = landAlignUs_;
		*raster = landRasterUs_;
		*text = landTextUs_;
		*flip = (landFlipUs_ & 0xffff) | (landPanMode_ << 16) | ((landPanFrames_ & 0xff) << 17) | ((landFullFrames_ & 0x7f) << 25);
	}
	void flushStatsReset() { flushStats_ = {}; }

	const char *flushStageName() const { return "idle"; }
	int flushStageChunk() const { return 0; }
	platform_display::DisplayFlushStageDetail flushStageDetail() const { return {}; }
	std::uint16_t *backgroundCache(int *capPx) { return fullScreenScratch(backgroundCache_, backgroundCacheAttempted_, capPx, "background"); }
	std::uint16_t *backdropCache(int *capPx) { return fullScreenScratch(backdropCache_, backdropCacheAttempted_, capPx, "backdrop"); }
	void setRenderCore(int core)
	{
		renderCoreId_ = core;
		if (frameBuffer_) workerCanvas_.bindPixels(frameBuffer_, logicalWidth(), logicalHeight(), logicalStride_);
	}

	void absorbWorkerDirty()
	{
		int x0 = 0;
		int y0 = 0;
		int x1 = -1;
		int y1 = -1;
		if (workerCanvas_.dirty(&x0, &y0, &x1, &y1)) canvas_.markDirty(x0, y0, x1, y1);
		workerCanvas_.resetDirty();
	}

private:
	DisplayBackend() = default;

	int logicalWidth() const { return orientation_detail::DisplayOrientationState::width(); }
	int logicalHeight() const { return orientation_detail::DisplayOrientationState::height(); }
	bool onWorkerCore() const { return renderCoreId_ >= 0 && xPortGetCoreID() != renderCoreId_; }
	bool onAsyncPresentWorker() const { return asyncPresentTask_ && xTaskGetCurrentTaskHandle() == asyncPresentTask_; }
	gea::framework::graphics::Canvas &replayCanvas() { return onWorkerCore() ? workerCanvas_ : canvas_; }

	static void asyncPresentTaskEntry(void *arg)
	{
		static_cast<DisplayBackend *>(arg)->asyncPresentTaskLoop();
	}

	void asyncPresentTaskLoop()
	{
		for (;;) {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			const bool ok = presentFullPanelBackRepaint(std::move(asyncPresentFrame_));
			asyncPresentOk_ = ok;
			asyncPresentInFlight_.store(false, std::memory_order_release);
			if (asyncPresentDoneSem_) xSemaphoreGive(asyncPresentDoneSem_);
		}
	}

	bool ensureAsyncPresentTask()
	{
		if (asyncPresentTask_) return true;
		if (!asyncPresentDoneSem_) asyncPresentDoneSem_ = xSemaphoreCreateBinary();
		if (!asyncPresentDoneSem_) return false;
		const BaseType_t appCore = xPortGetCoreID();
		const BaseType_t workerCore = (appCore == 0) ? 1 : 0;
		const UBaseType_t priority = uxTaskPriorityGet(nullptr);
		const BaseType_t created = xTaskCreatePinnedToCore(asyncPresentTaskEntry,
		                                                   "gea_present",
		                                                   kAsyncPresentStackBytes,
		                                                   this,
		                                                   priority,
		                                                   &asyncPresentTask_,
		                                                   workerCore);
		if (created != pdPASS || !asyncPresentTask_) {
			ESP_LOGE(kTag, "async present task creation failed: %d", static_cast<int>(created));
		}
		return created == pdPASS && asyncPresentTask_;
	}

	bool waitForAsyncPresent()
	{
		if (onAsyncPresentWorker()) return asyncPresentOk_;
		while (asyncPresentInFlight_.load(std::memory_order_acquire)) {
			if (asyncPresentDoneSem_)
				xSemaphoreTake(asyncPresentDoneSem_, pdMS_TO_TICKS(100));
			else
				taskYIELD();
		}
		return asyncPresentOk_;
	}

	bool scheduleAsyncFullPanelBackRepaint(present::Frame &&current)
	{
		waitForAsyncPresent();
		if (!ensureAsyncPresentTask()) {
			return presentFullPanelBackRepaint(std::move(current));
		}
		if (asyncPresentDoneSem_) xSemaphoreTake(asyncPresentDoneSem_, 0);
		asyncPresentFrame_ = std::move(current);
		asyncPresentOk_ = true;
		asyncPresentInFlight_.store(true, std::memory_order_release);
		xTaskNotifyGive(asyncPresentTask_);
		return true;
	}

	bool presentFullPanelBackRepaint(present::Frame &&current)
	{
		present::Rect region{0, 0, logicalWidth() - 1, logicalHeight() - 1};
		prepareDrawingTarget(false);
		const std::int64_t started = esp_timer_get_time();
		pathGeneral_++;
		flushStats_.callCount++;
		rasterPresentRegion(current, region);
		const bool ok = flushPreparedLogicalRects(&region, 1);
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) {
			previousPresentFrame_ = std::move(current);
			previousPresentValid_ = true;
			needsFullPhysicalFlush_ = false;
			frameBufferCoherent_ = true;
			canvas_.resetDirty();
		}
		return ok;
	}

	std::uint16_t *fullScreenScratch(std::uint16_t *&buffer, bool &attempted, int *capPx, const char *label)
	{
		if (!attempted) {
			attempted = true;
			buffer = static_cast<std::uint16_t *>(
				heap_caps_malloc(static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t),
				                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
			if (buffer) {
				ESP_LOGI(kTag, "allocated %s cache %.1f KiB", label, (kMaxPixels * sizeof(std::uint16_t)) / 1024.0);
			} else {
				ESP_LOGW(kTag, "failed to allocate %s cache (%zu bytes)",
				         label,
				         static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
			}
		}
		if (capPx) *capPx = buffer ? kMaxPixels : 0;
		return buffer;
	}

	void bindCanvas()
	{
		take();
		bindCanvasLocked();
		give();
	}

	void bindCanvasLocked()
	{
		const int width = logicalWidth();
		const int height = logicalHeight();
		const int stride = width;
		std::uint16_t *targetFrameBuffer = preferredFrameBuffer();
		if (!targetFrameBuffer) return;
		if (frameBuffer_ != targetFrameBuffer) frameBuffer_ = targetFrameBuffer;
		if (canvas_.pixels() == frameBuffer_ && canvas_.width() == width && canvas_.height() == height && logicalStride_ == stride) return;
		logicalStride_ = stride;
		canvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
		workerCanvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
	}

	std::uint16_t *preferredFrameBuffer()
	{
		using gea::framework::display::DisplayOrientation;
		if (orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary &&
		    panelFrameBuffers_[1] != nullptr) {
			drainRefreshDoneLocked();
			const int current = panelFrameBufferIndex(frameBuffer_);
			if (current >= 0 &&
			    panelFrameBuffers_[current] &&
			    current != scannedFbIndex_ &&
			    current != pendingReleasedFbIndex_) {
				return frameBuffer_;
			}
			const int writable = selectWritablePanelFrameBufferLocked(false);
			if (writable >= 0) return panelFrameBuffers_[writable];
			for (int i = 0; i < kDpiFrameBufferCount; ++i) {
				if (panelFrameBuffers_[i] && i != scannedFbIndex_) return panelFrameBuffers_[i];
			}
		}
		return ownedFrameBuffer_;
	}

	void prepareDrawingTarget(bool preservePanelBackBuffer = true)
	{
		if (!frameBuffer_) init();
		take();
		bindCanvasLocked();
		if (preservePanelBackBuffer)
			preparePanelBackBufferForDrawingLocked();
		else
			discardPanelBackBufferCoherenceForFullOverwriteLocked();
		give();
	}

	bool prepareNativePanelBackBufferForCircleFrame(int *outIndex)
	{
		if (outIndex) *outIndex = -1;
		if (!frameBuffer_) init();
		take();
		if (!canUseNativePanelBackBufferPath()) {
			give();
			return false;
		}
		const int index = selectWritablePanelFrameBufferLocked(true);
		if (index < 0 || !panelFrameBuffers_[index]) {
			give();
			return false;
		}
		frameBuffer_ = panelFrameBuffers_[index];
		logicalStride_ = kNativeWidth;
		canvas_.bindPixels(frameBuffer_, logicalWidth(), logicalHeight(), logicalStride_);
		workerCanvas_.bindPixels(frameBuffer_, logicalWidth(), logicalHeight(), logicalStride_);
		pendingPanelBackSyncCount_ = 0;
		clearPanelBufferStaleLocked(index);
		if (outIndex) *outIndex = index;
		give();
		return true;
	}

	bool regionsCoverLogicalFrame(const present::Rect *rects, int count) const
	{
		if (!rects || count <= 0) return false;
		for (int i = 0; i < count; ++i) {
			const present::Rect rect = clampLogicalRect(rects[i], logicalWidth(), logicalHeight());
			if (present::valid(rect) &&
			    rect.x0 <= 0 &&
			    rect.y0 <= 0 &&
			    rect.x1 >= logicalWidth() - 1 &&
			    rect.y1 >= logicalHeight() - 1) {
				return true;
			}
		}
		return false;
	}

	bool usingPanelFrameBufferDirectly() const
	{
		using gea::framework::display::DisplayOrientation;
		return frameBuffer_ && panelFrameBuffer_ && frameBuffer_ == panelFrameBuffer_ &&
		       logicalStride_ == kNativeWidth &&
		       orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary;
	}

	bool usingNativePanelBackBufferForDrawingLocked() const
	{
		const int index = panelFrameBufferIndex(frameBuffer_);
		return selectedNativePanelFrameBufferLocked() &&
		       index != pendingReleasedFbIndex_;
	}

	bool selectedNativePanelFrameBufferLocked() const
	{
		const int index = panelFrameBufferIndex(frameBuffer_);
		return frameBuffer_ && canUseNativePanelBackBufferPath() &&
		       logicalStride_ == kNativeWidth &&
		       index >= 0 &&
		       index != scannedFbIndex_;
	}

	bool canUseNativePanelBackBufferPath() const
	{
		using gea::framework::display::DisplayOrientation;
		return panelFrameBuffers_[1] != nullptr &&
		       logicalWidth() == kNativeWidth &&
		       logicalHeight() == kNativeHeight &&
		       orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary;
	}

	int panelFrameBufferIndex(const std::uint16_t *buffer) const
	{
		for (int i = 0; i < kDpiFrameBufferCount; ++i) {
			if (buffer && buffer == panelFrameBuffers_[i]) return i;
		}
		return -1;
	}

	void drainRefreshDoneLocked()
	{
		if (!refreshDoneSem_) {
			pendingReleasedFbIndex_ = -1;
			return;
		}
		while (xSemaphoreTake(refreshDoneSem_, 0) == pdTRUE) {
			pendingReleasedFbIndex_ = -1;
		}
	}

	void waitForReleasedPanelBufferLocked()
	{
		if (pendingReleasedFbIndex_ < 0) {
			drainRefreshDoneLocked();
			return;
		}
		if (refreshDoneSem_) xSemaphoreTake(refreshDoneSem_, pdMS_TO_TICKS(40));
		pendingReleasedFbIndex_ = -1;
	}

	int selectWritablePanelFrameBufferLocked(bool waitIfNeeded)
	{
		drainRefreshDoneLocked();
		for (int i = 0; i < kDpiFrameBufferCount; ++i) {
			if (!panelFrameBuffers_[i]) continue;
			if (i == scannedFbIndex_ || i == pendingReleasedFbIndex_) continue;
			return i;
		}
		if (waitIfNeeded) {
			waitForReleasedPanelBufferLocked();
			for (int i = 0; i < kDpiFrameBufferCount; ++i) {
				if (!panelFrameBuffers_[i]) continue;
				if (i == scannedFbIndex_ || i == pendingReleasedFbIndex_) continue;
				return i;
			}
		}
		return -1;
	}

	bool flipPanelToBufferLocked(std::uint16_t *buffer, bool syncFullFrame)
	{
		if (!panel_ || !buffer) return false;
		const int index = panelFrameBufferIndex(buffer);
		if (index < 0) return false;
		waitForReleasedPanelBufferLocked();
		const int previousScanIndex = scannedFbIndex_;
		const int xEnd = syncFullFrame ? kNativeWidth : 1;
		const int yEnd = syncFullFrame ? kNativeHeight : 1;
		const esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, 0, 0, xEnd, yEnd, buffer);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "DPI panel framebuffer swap failed: %s", esp_err_to_name(err));
			return false;
		}
		scannedFbIndex_ = index;
		panelFrameBuffer_ = panelFrameBuffers_[index];
		pendingReleasedFbIndex_ = previousScanIndex != index ? previousScanIndex : -1;
		return true;
	}

	void resetPanelBackBufferCoherenceLocked()
	{
		pendingPanelBackSyncCount_ = 0;
		for (int i = 0; i < kDpiFrameBufferCount; ++i) {
			panelBufferStaleCount_[i] = 0;
			panelBufferStaleFull_[i] = false;
		}
		invalidatePanelCircleFrames();
	}

	void invalidatePanelCircleFrames()
	{
		for (int i = 0; i < kDpiFrameBufferCount; ++i) panelCirclePrevValid_[i] = false;
	}

	void clearPanelBufferStaleLocked(int index)
	{
		if (index < 0 || index >= kDpiFrameBufferCount) return;
		panelBufferStaleCount_[index] = 0;
		panelBufferStaleFull_[index] = false;
	}

	void appendPanelBufferStaleRectLocked(int index, present::Rect rect)
	{
		if (index < 0 || index >= kDpiFrameBufferCount || !panelFrameBuffers_[index]) return;
		if (panelBufferStaleFull_[index]) return;
		rect = present::clampAndAlign(rect, kNativeWidth, kNativeHeight);
		if (!present::valid(rect)) return;
		if (rect.x0 <= 0 && rect.y0 <= 0 && rect.x1 >= kNativeWidth - 1 && rect.y1 >= kNativeHeight - 1) {
			panelBufferStaleCount_[index] = 0;
			panelBufferStaleFull_[index] = true;
			return;
		}
		addRectTight(panelBufferStaleRects_[index],
		             &panelBufferStaleCount_[index],
		             kMaxFlushRects,
		             rect,
		             kNativeWidth,
		             kNativeHeight);
	}

	void markPanelBuffersStaleAfterFlipLocked(const present::Rect *rects, int count)
	{
		clearPanelBufferStaleLocked(scannedFbIndex_);
		if (!rects || count <= 0) return;
		for (int i = 0; i < kDpiFrameBufferCount; ++i) {
			if (i == scannedFbIndex_ || !panelFrameBuffers_[i]) continue;
			for (int r = 0; r < count; ++r) appendPanelBufferStaleRectLocked(i, rects[r]);
		}
	}

	void appendPanelBackSyncRectLocked(present::Rect rect)
	{
		rect = present::clampAndAlign(rect, kNativeWidth, kNativeHeight);
		if (!present::valid(rect)) return;
		addRectTight(pendingPanelBackSyncRects_,
		             &pendingPanelBackSyncCount_,
		             kMaxFlushRects,
		             rect,
		             kNativeWidth,
		             kNativeHeight);
	}

	void copyPanelRectPixels(std::uint16_t *dst, const std::uint16_t *src, present::Rect rect) const
	{
		if (!dst || !src) return;
		rect = present::clampAndAlign(rect, kNativeWidth, kNativeHeight);
		if (!present::valid(rect)) return;
		const std::size_t bytes = static_cast<std::size_t>(rect.x1 - rect.x0 + 1) * sizeof(std::uint16_t);
		for (int y = rect.y0; y <= rect.y1; ++y) {
			std::memcpy(dst + static_cast<std::size_t>(y) * kNativeWidth + rect.x0,
			            src + static_cast<std::size_t>(y) * kNativeWidth + rect.x0,
			            bytes);
		}
	}

	void preparePanelBackBufferForDrawingLocked()
	{
		if (!selectedNativePanelFrameBufferLocked()) {
			resetPanelBackBufferCoherenceLocked();
			return;
		}
		const int selectedIndex = panelFrameBufferIndex(frameBuffer_);
		if (selectedIndex == pendingReleasedFbIndex_) waitForReleasedPanelBufferLocked();
		if (!usingNativePanelBackBufferForDrawingLocked()) {
			resetPanelBackBufferCoherenceLocked();
			return;
		}
		const bool needsFullCoherence = panelBufferStaleFull_[selectedIndex];
		const int staleCount = panelBufferStaleCount_[selectedIndex];
		if (!needsFullCoherence && staleCount <= 0) return;

		const std::int64_t copyStartUs = esp_timer_get_time();
		if (needsFullCoherence) {
			const present::Rect full{0, 0, kNativeWidth - 1, kNativeHeight - 1};
			copyPanelRectPixels(frameBuffer_, panelFrameBuffer_, full);
			appendPanelBackSyncRectLocked(full);
		} else {
			for (int i = 0; i < staleCount; ++i) {
				const present::Rect rect = present::clampAndAlign(panelBufferStaleRects_[selectedIndex][i], kNativeWidth, kNativeHeight);
				if (!present::valid(rect)) continue;
				copyPanelRectPixels(frameBuffer_, panelFrameBuffer_, rect);
				appendPanelBackSyncRectLocked(rect);
			}
		}
		flushStats_.txUs += esp_timer_get_time() - copyStartUs;
		clearPanelBufferStaleLocked(selectedIndex);
	}

	void discardPanelBackBufferCoherenceForFullOverwriteLocked()
	{
		if (!selectedNativePanelFrameBufferLocked()) {
			resetPanelBackBufferCoherenceLocked();
			return;
		}
		const int selectedIndex = panelFrameBufferIndex(frameBuffer_);
		if (selectedIndex == pendingReleasedFbIndex_) waitForReleasedPanelBufferLocked();
		pendingPanelBackSyncCount_ = 0;
		clearPanelBufferStaleLocked(selectedIndex);
	}

	void markPanelBackBuffersStaleLocked(const present::Rect *rects, int count)
	{
		markPanelBuffersStaleAfterFlipLocked(rects, count);
	}

	bool copyLogicalRectPixelsToPanelCpu(present::Rect logicalRect)
	{
		if (!frameBuffer_ || !panelFrameBuffer_) return false;
		logicalRect = clampLogicalRect(logicalRect, logicalWidth(), logicalHeight());
		if (!present::valid(logicalRect)) return true;

		using gea::framework::display::DisplayOrientation;
		switch (orientation_detail::DisplayOrientationState::orientation()) {
		case DisplayOrientation::PortraitSecondary:
			for (int sy = logicalRect.y0; sy <= logicalRect.y1; ++sy) {
				const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(sy) * logicalStride_ + logicalRect.x1;
				std::uint16_t *dst = panelFrameBuffer_ +
					static_cast<std::size_t>(kNativeHeight - 1 - sy) * kNativeWidth +
					(kNativeWidth - 1 - logicalRect.x1);
				for (int sx = logicalRect.x1; sx >= logicalRect.x0; --sx) {
					*dst++ = *src--;
				}
			}
			break;
		case DisplayOrientation::LandscapePrimary:
			for (int sx = logicalRect.x0; sx <= logicalRect.x1; ++sx) {
				const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(logicalRect.y0) * logicalStride_ + sx;
				std::uint16_t *dst = panelFrameBuffer_ +
					static_cast<std::size_t>(kNativeHeight - 1 - sx) * kNativeWidth +
					logicalRect.y0;
				for (int sy = logicalRect.y0; sy <= logicalRect.y1; ++sy) {
					*dst++ = *src;
					src += logicalStride_;
				}
			}
			break;
		case DisplayOrientation::LandscapeSecondary:
			for (int sx = logicalRect.x0; sx <= logicalRect.x1; ++sx) {
				const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(logicalRect.y1) * logicalStride_ + sx;
				std::uint16_t *dst = panelFrameBuffer_ +
					static_cast<std::size_t>(sx) * kNativeWidth +
					(kNativeWidth - 1 - logicalRect.y1);
				for (int sy = logicalRect.y1; sy >= logicalRect.y0; --sy) {
					*dst++ = *src;
					src -= logicalStride_;
				}
			}
			break;
		case DisplayOrientation::PortraitPrimary:
		default:
			for (int sy = logicalRect.y0; sy <= logicalRect.y1; ++sy) {
				const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(sy) * logicalStride_ + logicalRect.x0;
				std::uint16_t *dst = panelFrameBuffer_ + static_cast<std::size_t>(sy) * kNativeWidth + logicalRect.x0;
				std::memcpy(dst, src, static_cast<std::size_t>(logicalRect.x1 - logicalRect.x0 + 1) * sizeof(std::uint16_t));
			}
			break;
		}
		return true;
	}

	bool copyLogicalRectToPanelCpu(present::Rect logicalRect, present::Rect panelRect)
	{
		panelRect = present::clampAndAlign(panelRect, kNativeWidth, kNativeHeight);
		if (!present::valid(panelRect)) return true;
		const std::int64_t copyStartUs = esp_timer_get_time();
		const bool copied = copyLogicalRectPixelsToPanelCpu(logicalRect);
		flushStats_.txUs += esp_timer_get_time() - copyStartUs;
		if (!copied) return false;

		const std::int64_t syncStartUs = esp_timer_get_time();
		const bool ok = syncPanelFrameBufferRectToMemory(panelRect);
		flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
		return ok;
	}

	bool flushPreparedLogicalRects(present::Rect *rects, int count)
	{
		if (!rects || count <= 0) return true;
		bool panelBackHandled = false;
		const bool panelBackOk = flushPanelBackBufferLogicalRects(rects, count, &panelBackHandled);
		if (!panelBackOk || panelBackHandled) return panelBackOk;

		bool handled = false;
		const bool batchOk = flushSmallCpuMappedLogicalRects(rects, count, &handled);
		if (!batchOk || handled) return batchOk;
		bool ok = true;
		for (int i = 0; i < count; ++i) {
			if (!flushNativeLogicalRect(rects[i])) {
				ok = false;
				break;
			}
		}
		return ok;
	}

	bool flushPanelBackBufferLogicalRects(const present::Rect *rects, int count, bool *handled)
	{
		if (handled) *handled = false;
		if (!rects || count <= 0) return true;

		take();
		if (!usingNativePanelBackBufferForDrawingLocked() || scrollOriginX_ != 0 || scrollOriginY_ != 0) {
			give();
			return true;
		}
		if (handled) *handled = true;
		if (!panel_) {
			give();
			return false;
		}

		present::Rect syncRects[kMaxFlushRects];
		int syncCount = 0;
		for (int i = 0; i < pendingPanelBackSyncCount_; ++i) {
			addRectTight(syncRects, &syncCount, kMaxFlushRects, pendingPanelBackSyncRects_[i], kNativeWidth, kNativeHeight);
		}
		for (int i = 0; i < count; ++i) {
			present::Rect rect = present::clampAndAlign(rects[i], kNativeWidth, kNativeHeight);
			if (!present::valid(rect)) continue;
			addRectTight(syncRects, &syncCount, kMaxFlushRects, rect, kNativeWidth, kNativeHeight);
		}

		bool ok = true;
		if (syncCount > 0) {
			const std::int64_t syncStartUs = esp_timer_get_time();
			if (shouldSyncWholeCache(syncRects, syncCount))
				ok = syncWholeDataCacheToMemory();
			else if (shouldSyncRectsIndividually(syncRects, syncCount))
				ok = syncFrameBufferRectsToMemory(frameBuffer_, syncRects, syncCount, "DPI panel back framebuffer");
			else
				ok = syncFrameBufferRowBandsToMemory(frameBuffer_, syncRects, syncCount, "DPI panel back framebuffer");
			flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
		}
		if (ok) {
			const std::int64_t flipStartUs = esp_timer_get_time();
			drainRefreshDoneLocked();
			ok = flipPanelToBufferLocked(frameBuffer_, false);
			lastFlipUs_ = esp_timer_get_time();
			flushStats_.copyUs += lastFlipUs_ - flipStartUs;
		}
		if (ok) {
			markPanelBackBuffersStaleLocked(rects, count);
			pendingPanelBackSyncCount_ = 0;
			flushStats_.chunkCount++;
			for (int i = 0; i < count; ++i) flushStats_.pixelCount += present::area(rects[i]);
		}
		give();
		return ok;
	}

	bool flushSmallCpuMappedLogicalRects(present::Rect *rects, int count, bool *handled)
	{
		if (handled) *handled = false;
		if (!rects || count < 2) return true;

		present::Rect logicalRects[kMaxFlushRects];
		present::Rect panelRects[kMaxFlushRects];
		int preparedCount = 0;
		for (int i = 0; i < count && preparedCount < kMaxFlushRects; ++i) {
			present::Rect logicalRect = clampLogicalRect(rects[i], logicalWidth(), logicalHeight());
			if (!present::valid(logicalRect)) continue;
			const int logicalPixels = (logicalRect.x1 - logicalRect.x0 + 1) * (logicalRect.y1 - logicalRect.y0 + 1);
			if (logicalPixels > kCpuPanelMapFlushMaxPixels) return true;
			const present::Rect panelRect = present::clampAndAlign(physicalBoundsForLogicalRect(logicalRect), kNativeWidth, kNativeHeight);
			if (!present::valid(panelRect)) continue;
			logicalRects[preparedCount] = logicalRect;
			panelRects[preparedCount] = panelRect;
			++preparedCount;
		}
		if (preparedCount < 2) return true;

		take();
		bindCanvasLocked();
		if (!panel_) {
			give();
			if (handled) *handled = true;
			return false;
		}
		if (frameBuffer_ != ownedFrameBuffer_ || scrollOriginX_ != 0 || scrollOriginY_ != 0) {
			give();
			return true;
		}

		if (handled) *handled = true;
		bool ok = true;
		const std::int64_t copyStartUs = esp_timer_get_time();
		for (int i = 0; i < preparedCount; ++i) {
			if (!copyLogicalRectPixelsToPanelCpu(logicalRects[i])) {
				ok = false;
				break;
			}
		}
		flushStats_.txUs += esp_timer_get_time() - copyStartUs;

		if (ok) {
			const std::int64_t syncStartUs = esp_timer_get_time();
			ok = syncPanelFrameBufferRowBandsToMemory(panelRects, preparedCount);
			flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
		}
		if (ok) {
			for (int i = 0; i < preparedCount; ++i) {
				flushStats_.chunkCount++;
				flushStats_.pixelCount += present::area(panelRects[i]);
			}
		}
		give();
		return ok;
	}

	void appendFlushRect(present::Rect *rects, int *count, int capacity, present::Rect rect) const
	{
		rect = clampLogicalRect(rect, logicalWidth(), logicalHeight());
		if (!present::valid(rect)) return;
		present::addRect(rects, count, capacity, rect, logicalWidth(), logicalHeight());
	}

	void appendFlushRectExact(present::Rect *rects, int *count, int capacity, present::Rect rect) const
	{
		if (!rects || !count || capacity <= 0) return;
		rect = clampLogicalRect(rect, logicalWidth(), logicalHeight());
		if (!present::valid(rect)) return;
		if (*count < capacity) {
			rects[(*count)++] = rect;
			return;
		}
		present::addRect(rects, count, capacity, rect, logicalWidth(), logicalHeight());
	}

	void addRectTight(present::Rect *rects, int *count, int capacity, present::Rect rect, int width, int height) const
	{
		if (!rects || !count || capacity <= 0) return;
		rect = present::clampAndAlign(rect, width, height);
		if (!present::valid(rect)) return;
		const long long newArea = rectArea64(rect);
		for (int i = 0; i < *count; ++i) {
			const present::Rect merged = present::unite(rects[i], rect);
			if (rectArea64(merged) <= rectArea64(rects[i]) + newArea) {
				rects[i] = merged;
				return;
			}
		}
		if (*count < capacity) {
			rects[(*count)++] = rect;
			return;
		}
		int best = 0;
		long long bestCost = 0x7fffffffffffffffLL;
		for (int i = 0; i < *count; ++i) {
			const present::Rect merged = present::unite(rects[i], rect);
			const long long cost = rectArea64(merged) - rectArea64(rects[i]);
			if (cost < bestCost) {
				bestCost = cost;
				best = i;
			}
		}
		rects[best] = present::unite(rects[best], rect);
	}

	long long rectAreaSum(const present::Rect *rects, int count) const
	{
		if (!rects || count <= 0) return 0;
		long long area = 0;
		for (int i = 0; i < count; ++i) area += rectArea64(rects[i]);
		return area;
	}

	long long rowBandAreaForRects(const present::Rect *rects, int count) const
	{
		if (!rects || count <= 0) return 0;
		present::Rect rowBands[kMaxFlushRects];
		int rowBandCount = 0;
		for (int i = 0; i < count; ++i) {
			const present::Rect rect = present::clampAndAlign(rects[i], kNativeWidth, kNativeHeight);
			if (!present::valid(rect)) continue;
			present::addRect(rowBands,
			                 &rowBandCount,
			                 kMaxFlushRects,
			                 {0, rect.y0, kNativeWidth - 1, rect.y1},
			                 kNativeWidth,
			                 kNativeHeight);
		}
		return rectAreaSum(rowBands, rowBandCount);
	}

	bool syncFrameBufferRectToMemory(present::Rect logicalRect)
	{
		return syncBufferRectToMemory(frameBuffer_, logicalStride_, logicalWidth(), logicalHeight(), logicalRect, "DPI framebuffer");
	}

	bool syncBufferRectToMemory(std::uint16_t *buffer,
	                            int stridePixels,
	                            int widthPixels,
	                            int heightPixels,
	                            present::Rect rect,
	                            const char *label)
	{
		if (!buffer) return false;
		rect = present::clampAndAlign(rect, widthPixels, heightPixels);
		if (!present::valid(rect)) return true;
		const int width = rect.x1 - rect.x0 + 1;
		if (width <= 0) return true;
		const std::size_t bytes = static_cast<std::size_t>(width) * sizeof(std::uint16_t);
		constexpr int kFlags = ESP_CACHE_MSYNC_FLAG_DIR_C2M |
		                       ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
		                       ESP_CACHE_MSYNC_FLAG_UNALIGNED;
		for (int y = rect.y0; y <= rect.y1; ++y) {
			void *row = buffer + static_cast<std::size_t>(y) * stridePixels + rect.x0;
			const esp_err_t err = esp_cache_msync(row, bytes, kFlags);
			if (err == ESP_OK) continue;
			if (!cacheSyncWarned_) {
				cacheSyncWarned_ = true;
				ESP_LOGE(kTag, "%s cache writeback failed: %s", label, esp_err_to_name(err));
			}
			return false;
		}
		return true;
	}

	bool syncBufferRectToMemoryFast(std::uint16_t *buffer,
	                                int stridePixels,
	                                int widthPixels,
	                                int heightPixels,
	                                present::Rect rect,
	                                const char *label)
	{
		if (!buffer) return false;
		rect = present::clampAndAlign(rect, widthPixels, heightPixels);
		if (!present::valid(rect)) return true;
		const int width = rect.x1 - rect.x0 + 1;
		if (width <= 0) return true;
		const std::size_t bytes = static_cast<std::size_t>(width) * sizeof(std::uint16_t);
		bool ok = true;
		esp_cache_sync_ops_enter_critical_section();
		for (int y = rect.y0; y <= rect.y1; ++y) {
			void *row = buffer + static_cast<std::size_t>(y) * stridePixels + rect.x0;
			if (cache_hal_writeback_addr(static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(row)),
			                             static_cast<std::uint32_t>(bytes))) {
				continue;
			}
			ok = false;
			break;
		}
		esp_cache_sync_ops_exit_critical_section();
		if (ok) return true;
		if (!cacheSyncWarned_) {
			cacheSyncWarned_ = true;
			ESP_LOGE(kTag, "%s fast cache writeback failed", label ? label : "framebuffer");
		}
		return false;
	}

	bool syncPanelFrameBufferRectToMemory(present::Rect panelRect)
	{
		return syncBufferRectToMemory(panelFrameBuffer_, kNativeWidth, kNativeWidth, kNativeHeight, panelRect, "DPI panel framebuffer");
	}

	bool syncFrameBufferRectsToMemory(std::uint16_t *buffer,
	                                  const present::Rect *panelRects,
	                                  int count,
	                                  const char *label)
	{
		if (!panelRects || count <= 0) return true;
		for (int i = 0; i < count; ++i) {
			const present::Rect rect = present::clampAndAlign(panelRects[i], kNativeWidth, kNativeHeight);
			if (!present::valid(rect)) continue;
			if (!syncBufferRectToMemoryFast(buffer, kNativeWidth, kNativeWidth, kNativeHeight, rect, label)) return false;
		}
		return true;
	}

	bool syncWholeDataCacheToMemory()
	{
		esp_cache_sync_ops_enter_critical_section();
		cache_ll_writeback_all(CACHE_LL_LEVEL_ALL, CACHE_TYPE_DATA, CACHE_LL_ID_ALL);
		esp_cache_sync_ops_exit_critical_section();
		return true;
	}

	bool syncFrameBufferFullRowsToMemory(std::uint16_t *buffer, int y0, int y1, const char *label)
	{
		if (!buffer) return false;
		if (y0 < 0) y0 = 0;
		if (y1 >= kNativeHeight) y1 = kNativeHeight - 1;
		if (y0 > y1) return true;
		void *start = buffer + static_cast<std::size_t>(y0) * kNativeWidth;
		const std::size_t rows = static_cast<std::size_t>(y1 - y0 + 1);
		const std::size_t bytes = rows * kNativeWidth * sizeof(std::uint16_t);
		constexpr int kFlags = ESP_CACHE_MSYNC_FLAG_DIR_C2M |
		                       ESP_CACHE_MSYNC_FLAG_TYPE_DATA |
		                       ESP_CACHE_MSYNC_FLAG_UNALIGNED;
		const esp_err_t err = esp_cache_msync(start, bytes, kFlags);
		if (err == ESP_OK) return true;
		if (!cacheSyncWarned_) {
			cacheSyncWarned_ = true;
			ESP_LOGE(kTag, "%s row-band cache writeback failed: %s", label ? label : "framebuffer", esp_err_to_name(err));
		}
		return false;
	}

	bool syncFrameBufferRowBandsToMemory(std::uint16_t *buffer, const present::Rect *panelRects, int count, const char *label)
	{
		if (!panelRects || count <= 0) return true;
		present::Rect rowBands[kMaxFlushRects];
		int rowBandCount = 0;
		for (int i = 0; i < count; ++i) {
			const present::Rect panelRect = present::clampAndAlign(panelRects[i], kNativeWidth, kNativeHeight);
			if (!present::valid(panelRect)) continue;
			present::addRect(rowBands,
			                 &rowBandCount,
			                 kMaxFlushRects,
			                 {0, panelRect.y0, kNativeWidth - 1, panelRect.y1},
			                 kNativeWidth,
			                 kNativeHeight);
		}
		for (int i = 0; i < rowBandCount; ++i) {
			if (!syncFrameBufferFullRowsToMemory(buffer, rowBands[i].y0, rowBands[i].y1, label)) return false;
		}
		return true;
	}

	bool shouldSyncRectsIndividually(const present::Rect *rects, int count) const
	{
		if (!rects || count <= 1) return false;
		const long long rectArea = rectAreaSum(rects, count);
		if (rectArea <= 0) return false;
		const long long screenArea = static_cast<long long>(kNativeWidth) * kNativeHeight;
		if (rectArea >= screenArea) return false;
		const long long rowBandArea = rowBandAreaForRects(rects, count);
		return rowBandArea * 10 > rectArea * 11;
	}

	bool shouldSyncWholeCache(const present::Rect *rects, int count) const
	{
		if (!rects || count < 8) return false;
		const long long rectArea = rectAreaSum(rects, count);
		if (rectArea <= 0) return false;
		const long long screenArea = static_cast<long long>(kNativeWidth) * kNativeHeight;
		return rectArea * 4 < screenArea;
	}

	bool syncPanelFrameBufferFullRowsToMemory(int y0, int y1)
	{
		return syncFrameBufferFullRowsToMemory(panelFrameBuffer_, y0, y1, "DPI panel framebuffer");
	}

	bool syncPanelFrameBufferRowBandsToMemory(const present::Rect *panelRects, int count)
	{
		return syncFrameBufferRowBandsToMemory(panelFrameBuffer_, panelRects, count, "DPI panel framebuffer");
	}

	bool initPpaSrm()
	{
		if (ppaSrmClient_) return true;
		ppa_client_config_t config{};
		config.oper_type = PPA_OPERATION_SRM;
		config.max_pending_trans_num = 1;
		config.data_burst_length = PPA_DATA_BURST_LENGTH_128;
		const esp_err_t err = ppa_register_client(&config, &ppaSrmClient_);
		if (err != ESP_OK) {
			if (!ppaWarned_) {
				ppaWarned_ = true;
				ESP_LOGE(kTag, "PPA SRM client registration failed: %s", esp_err_to_name(err));
			}
			return false;
		}
		ESP_LOGI(kTag, "PPA SRM rotation ready");
		return true;
	}

	ppa_srm_rotation_angle_t ppaRotationForCurrentOrientation() const
	{
		using gea::framework::display::DisplayOrientation;
		switch (orientation_detail::DisplayOrientationState::orientation()) {
		case DisplayOrientation::PortraitSecondary:
			return PPA_SRM_ROTATION_ANGLE_180;
		case DisplayOrientation::LandscapePrimary:
			return PPA_SRM_ROTATION_ANGLE_90;
		case DisplayOrientation::LandscapeSecondary:
			return PPA_SRM_ROTATION_ANGLE_270;
		case DisplayOrientation::PortraitPrimary:
		default:
			return PPA_SRM_ROTATION_ANGLE_0;
		}
	}

	bool rotateLogicalRectToPanelWithPpa(present::Rect logicalRect, present::Rect panelRect)
	{
		if (!frameBuffer_ || !panelFrameBuffer_) return false;
		logicalRect = clampLogicalRect(logicalRect, logicalWidth(), logicalHeight());
		panelRect = present::clampAndAlign(panelRect, kNativeWidth, kNativeHeight);
		if (!present::valid(logicalRect) || !present::valid(panelRect)) return true;
		if (!initPpaSrm()) return false;

		const int logicalRectWidth = logicalRect.x1 - logicalRect.x0 + 1;
		const int logicalRectHeight = logicalRect.y1 - logicalRect.y0 + 1;
		const std::int64_t syncStartUs = esp_timer_get_time();
		if (!syncFrameBufferRectToMemory(logicalRect) || !syncPanelFrameBufferRectToMemory(panelRect)) {
			flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
			return false;
		}
		flushStats_.copyUs += esp_timer_get_time() - syncStartUs;

		ppa_srm_oper_config_t config{};
		config.in.buffer = frameBuffer_;
		config.in.pic_w = static_cast<std::uint32_t>(logicalWidth());
		config.in.pic_h = static_cast<std::uint32_t>(logicalHeight());
		config.in.block_w = static_cast<std::uint32_t>(logicalRectWidth);
		config.in.block_h = static_cast<std::uint32_t>(logicalRectHeight);
		config.in.block_offset_x = static_cast<std::uint32_t>(logicalRect.x0);
		config.in.block_offset_y = static_cast<std::uint32_t>(logicalRect.y0);
		config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		config.out.buffer = panelFrameBuffer_;
		config.out.buffer_size = static_cast<std::uint32_t>(kMaxPixels * sizeof(std::uint16_t));
		config.out.pic_w = kNativeWidth;
		config.out.pic_h = kNativeHeight;
		config.out.block_offset_x = static_cast<std::uint32_t>(panelRect.x0);
		config.out.block_offset_y = static_cast<std::uint32_t>(panelRect.y0);
		config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		config.rotation_angle = ppaRotationForCurrentOrientation();
		config.scale_x = 1.0f;
		config.scale_y = 1.0f;
		config.mode = PPA_TRANS_MODE_BLOCKING;

		const std::int64_t ppaStartUs = esp_timer_get_time();
		const esp_err_t err = ppa_do_scale_rotate_mirror(ppaSrmClient_, &config);
		flushStats_.txUs += esp_timer_get_time() - ppaStartUs;
		if (err != ESP_OK) {
			if (!ppaWarned_) {
				ppaWarned_ = true;
				ESP_LOGE(kTag, "PPA SRM present failed: %s", esp_err_to_name(err));
			}
			return false;
		}
		return true;
	}

	bool enableDsiPhyPower()
	{
		if (dsiPhyPower_) return true;
		esp_ldo_channel_config_t ldoConfig{};
		ldoConfig.chan_id = kDsiPhyLdoChannel;
		ldoConfig.voltage_mv = kDsiPhyLdoVoltageMv;
		const esp_err_t err = esp_ldo_acquire_channel(&ldoConfig, &dsiPhyPower_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "DSI PHY LDO enable failed: %s", esp_err_to_name(err));
			return false;
		}
		ESP_LOGI(kTag, "DSI PHY powered on LDO%d %dmV", kDsiPhyLdoChannel, kDsiPhyLdoVoltageMv);
		return true;
	}

	bool initBacklight()
	{
		if (brightnessInitialized_) return true;

		ledc_timer_config_t timerConfig{};
		timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
		timerConfig.timer_num = kBacklightLedcTimer;
		timerConfig.freq_hz = 5000;
		timerConfig.clk_cfg = LEDC_AUTO_CLK;
		esp_err_t err = ledc_timer_config(&timerConfig);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "backlight LEDC timer init failed: %s", esp_err_to_name(err));
			return false;
		}

		ledc_channel_config_t channelConfig{};
		channelConfig.gpio_num = gea::platform::board::display.backlight;
		channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		channelConfig.channel = kBacklightLedcChannel;
		channelConfig.timer_sel = kBacklightLedcTimer;
		channelConfig.duty = 0;
		channelConfig.hpoint = 0;
		err = ledc_channel_config(&channelConfig);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "backlight LEDC channel init failed: %s", esp_err_to_name(err));
			return false;
		}

		brightnessInitialized_ = true;
		updateBacklightDuty();
		return true;
	}

	void updateBacklightDuty()
	{
		const std::uint32_t duty = static_cast<std::uint32_t>((brightness_ * kBacklightLedcMaxDuty) / 100);
		ledc_set_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel, duty);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel);
	}

	bool initPanel()
	{
		if (panel_) return true;

		if (!enableDsiPhyPower()) return false;

		gea::platform::tab5::LcdPanel panelInfo;
		if (!gea::platform::tab5::initDisplayPanel(kDpiFrameBufferCount, &panelInfo)) {
			ESP_LOGE(kTag, "Tab5 LCD init failed");
			return false;
		}
		const char *controllerName = gea::platform::tab5::displayControllerName(panelInfo.controller);

		mipiDsiBus_ = panelInfo.dsiBus;
		io_ = panelInfo.io;
		panel_ = panelInfo.panel;
		if (!panel_) {
			ESP_LOGE(kTag, "Tab5 driver did not expose an LCD panel handle");
			return false;
		}

		if (!initBacklight()) return false;

		void *fb0 = nullptr;
		void *fb1 = nullptr;
		void *fb2 = nullptr;
		esp_err_t err = esp_lcd_dpi_panel_get_frame_buffer(panel_, kDpiFrameBufferCount, &fb0, &fb1, &fb2);
		if (err == ESP_OK && fb0) {
			panelFrameBuffers_[0] = static_cast<std::uint16_t *>(fb0);
			panelFrameBuffers_[1] = static_cast<std::uint16_t *>(fb1);  // null when num_fbs == 1
			panelFrameBuffers_[2] = static_cast<std::uint16_t *>(fb2);  // null when num_fbs < 3
			scannedFbIndex_ = 0;  // the driver scans fb0 first
			pendingReleasedFbIndex_ = -1;
			panelFrameBuffer_ = panelFrameBuffers_[0];
			const std::size_t panelBytes = static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t);
			for (int i = 0; i < kDpiFrameBufferCount; ++i) {
				if (!panelFrameBuffers_[i]) continue;
				std::memset(panelFrameBuffers_[i], 0, panelBytes);
				syncFrameBufferFullRowsToMemory(panelFrameBuffers_[i], 0, kNativeHeight - 1, "DPI panel framebuffer");
			}
			resetPanelBackBufferCoherenceLocked();
			ESP_LOGI(kTag, "using DPI panel framebuffers fb0=%p fb1=%p fb2=%p",
			         panelFrameBuffers_[0],
			         panelFrameBuffers_[1],
			         panelFrameBuffers_[2]);
		} else {
			ESP_LOGE(kTag, "DPI panel framebuffer unavailable: %s", esp_err_to_name(err));
			return false;
		}
		// Refresh-boundary signal for scan-aligned double-buffer flips: a
		// draw_bitmap buffer swap only latches at the next frame restart, so a
		// producer must not write the just-released buffer until one refresh
		// completes (else its base fill lands on the still-scanned frame —
		// visible as white flashes while panning).
		refreshDoneSem_ = xSemaphoreCreateBinary();
		if (refreshDoneSem_) {
			esp_lcd_dpi_panel_event_callbacks_t cbs{};
			cbs.on_refresh_done = [](esp_lcd_panel_handle_t, esp_lcd_dpi_panel_event_data_t *, void *ctx) -> bool {
				BaseType_t woken = pdFALSE;
				xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(ctx), &woken);
				return woken == pdTRUE;
			};
			if (esp_lcd_dpi_panel_register_event_callbacks(panel_, &cbs, refreshDoneSem_) != ESP_OK) {
				ESP_LOGW(kTag, "refresh-done callback unavailable; panel-direct flips stay unaligned");
				vSemaphoreDelete(refreshDoneSem_);
				refreshDoneSem_ = nullptr;
			}
		}
		err = esp_lcd_panel_disp_on_off(panel_, true);
		if (err == ESP_ERR_NOT_SUPPORTED) {
			ESP_LOGW(kTag, "panel display-on hook unsupported by DPI panel; continuing after DSI init table");
		} else if (err != ESP_OK) {
			ESP_LOGE(kTag, "panel on failed: %s", esp_err_to_name(err));
			return false;
		}

		// One-shot GDMA bandwidth probe: can the AXI DMA move a full frame
		// between PSRAM buffers materially faster than the PPA's measured
		// ~86MB/s or the CPU's ~120MB/s? Decides whether map pans should be a
		// DMA content-shift + strip raster. The buffers get fully overwritten
		// by the first present, so the copy is harmless.
		if (panelFrameBuffers_[1] && ownedFrameBuffer_) {
			async_memcpy_config_t mcpConfig{};
			mcpConfig.backlog = 4;
			mcpConfig.dma_burst_size = 64;
			async_memcpy_handle_t mcp = nullptr;
			if (esp_async_memcpy_install_gdma_axi(&mcpConfig, &mcp) == ESP_OK) {
				SemaphoreHandle_t done = xSemaphoreCreateBinary();
				auto benchCb = [](async_memcpy_handle_t, async_memcpy_event_t *, void *ctx) -> bool {
					BaseType_t woken = pdFALSE;
					xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(ctx), &woken);
					return woken == pdTRUE;
				};
				const std::size_t benchBytes = static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t);
				const std::int64_t benchStart = esp_timer_get_time();
				if (done &&
				    esp_async_memcpy(mcp, panelFrameBuffers_[1], ownedFrameBuffer_, benchBytes, benchCb, done) == ESP_OK &&
				    xSemaphoreTake(done, pdMS_TO_TICKS(500)) == pdTRUE) {
					const std::int64_t benchUs = esp_timer_get_time() - benchStart;
					ESP_LOGI(kTag, "GDMA bench: %u bytes in %lld us = %.1f MB/s", static_cast<unsigned>(benchBytes),
					         static_cast<long long>(benchUs), benchUs > 0 ? benchBytes / static_cast<double>(benchUs) : 0.0);
				} else {
					ESP_LOGW(kTag, "GDMA bench failed");
				}
				if (done) vSemaphoreDelete(done);
				// Keep the channel: the landscape pan fast path uses it to
				// shift-copy the map region (~15.5ms DMA, overlapped with the
				// CPU strip raster) instead of a ~22ms CPU memcpy.
				mcp_ = mcp;
				mcpDone_ = xSemaphoreCreateBinary();
			} else {
				ESP_LOGW(kTag, "GDMA AXI async-memcpy unavailable");
			}
		}
		ESP_LOGI(kTag,
		         "Tab5 %s DSI panel ready native=%dx%d logical=%dx%d fb=%s out=%s panel_bpp=%d dpi_fbs=%d",
		         controllerName,
		         kNativeWidth,
		         kNativeHeight,
		         logicalWidth(),
		         logicalHeight(),
		         pixel::name(kFramebufferPixelFormat),
		         pixel::name(kDsiOutputPixelFormat),
		         kPanelBitsPerPixel,
		         kDpiFrameBufferCount);
		return true;
	}

	bool flushNativeLogicalRect(present::Rect logicalRect)
	{
		logicalRect = clampLogicalRect(logicalRect, logicalWidth(), logicalHeight());
		if (!present::valid(logicalRect)) return true;
		const present::Rect panelRect = physicalBoundsForLogicalRect(logicalRect);
		if (!present::valid(panelRect)) return true;
		using gea::framework::display::DisplayOrientation;
		const bool directRows =
			orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary;

		take();
		bindCanvasLocked();
		if (!panel_) {
			give();
			return false;
		}
		bool ok = true;
		const int panelWidth = panelRect.x1 - panelRect.x0 + 1;
		const int panelHeight = panelRect.y1 - panelRect.y0 + 1;
		if (directRows && usingPanelFrameBufferDirectly()) {
			const std::int64_t syncStartUs = esp_timer_get_time();
			// NOTE: this full-frame cache writeback (~40ms for 720x1280) is the dominant
			// per-frame cost. Splitting it across both cores was measured to be a net loss
			// (21.6->19.4fps, flush 40->44ms): it is PSRAM-write-bandwidth-bound, so two
			// cores just contend for the same bus while adding handshake latency. The 40ms
			// is a hard bandwidth floor for a full-screen redraw on this DSI panel.
			ok = syncFrameBufferRectToMemory(logicalRect);
			flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
			flushStats_.chunkCount++;
			flushStats_.pixelCount += panelWidth * panelHeight;
			give();
			return ok;
		}

		if (frameBuffer_ != ownedFrameBuffer_) {
			ESP_LOGE(kTag, "rotated flush requires the owned framebuffer");
			give();
			return false;
		}
		if (scrollOriginX_ != 0 || scrollOriginY_ != 0) {
			// Torus mode: the logical rect's pixels live at wrapped framebuffer
			// locations — sync + rotate each part separately.
			TorusPart parts[4];
			const int count = splitLogicalRectToTorus(logicalRect, parts);
			ok = true;
			for (int i = 0; i < count && ok; ++i) {
				const TorusPart &part = parts[i];
				const std::int64_t syncStartUs = esp_timer_get_time();
				ok = syncTorusPartToMemory(part);
				flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
				if (!ok) break;
				const int w = part.torus.x1 - part.torus.x0 + 1;
				const int h = part.torus.y1 - part.torus.y0 + 1;
				const present::Rect partLogical{part.logicalX, part.logicalY, part.logicalX + w - 1, part.logicalY + h - 1};
				ok = rotateTorusPartToPanelWithPpa(part, physicalBoundsForLogicalRect(partLogical));
			}
			if (ok) {
				flushStats_.chunkCount++;
				flushStats_.pixelCount += panelWidth * panelHeight;
			}
			give();
			return ok;
		}
		const int logicalPixels = (logicalRect.x1 - logicalRect.x0 + 1) * (logicalRect.y1 - logicalRect.y0 + 1);
		if (logicalPixels <= kCpuPanelMapFlushMaxPixels) {
			ok = copyLogicalRectToPanelCpu(logicalRect, panelRect);
		} else {
			ok = rotateLogicalRectToPanelWithPpa(logicalRect, panelRect);
		}
		if (ok) {
			flushStats_.chunkCount++;
			flushStats_.pixelCount += panelWidth * panelHeight;
		}
		give();
		return ok;
	}

	// ---- Scroll-origin (torus) panning -------------------------------------
	// A pan repaints the whole screen even though 80-90% of the pixels are the
	// SAME content at a shifted address — and a full-screen repaint is bus-
	// bound (~9MB of PSRAM traffic ≈ 60-90ms). Instead, when a frame is the
	// previous frame uniformly shifted by (dx,dy), move the framebuffer's READ
	// ORIGIN (treating it as a wrapping scroll surface) and raster only the
	// newly exposed strips. The PPA rotate pass — which runs anyway — assembles
	// the wrapped quadrants into the panel buffer. Pan frames drop from ~921k
	// rastered pixels to the strip area (~30-80k).

	static int wrapCoord(int v, int size)
	{
		int m = v % size;
		return m < 0 ? m + size : m;
	}

	// A logical-space rect mapped onto the torus: `torus` is where the pixels
	// live in the framebuffer; logicalX/logicalY are the content coordinates of
	// its top-left corner.
	struct TorusPart {
		present::Rect torus;
		int logicalX;
		int logicalY;
	};

	int splitLogicalRectToTorus(const present::Rect &logical, TorusPart out[4]) const
	{
		const int w = logicalWidth();
		const int h = logicalHeight();
		const int rectW = logical.x1 - logical.x0 + 1;
		const int rectH = logical.y1 - logical.y0 + 1;
		if (rectW <= 0 || rectH <= 0) return 0;
		const int startX = wrapCoord(logical.x0 + scrollOriginX_, w);
		const int startY = wrapCoord(logical.y0 + scrollOriginY_, h);
		const int xw0 = std::min(rectW, w - startX);
		const int yh0 = std::min(rectH, h - startY);
		struct Seg {
			int torusStart;
			int logicalStart;
			int size;
		};
		const Seg xs[2] = {{startX, logical.x0, xw0}, {0, logical.x0 + xw0, rectW - xw0}};
		const Seg ys[2] = {{startY, logical.y0, yh0}, {0, logical.y0 + yh0, rectH - yh0}};
		int count = 0;
		for (const Seg &sy : ys) {
			if (sy.size <= 0) continue;
			for (const Seg &sx : xs) {
				if (sx.size <= 0) continue;
				out[count].torus = {sx.torusStart, sy.torusStart, sx.torusStart + sx.size - 1, sy.torusStart + sy.size - 1};
				out[count].logicalX = sx.logicalStart;
				out[count].logicalY = sy.logicalStart;
				count++;
			}
		}
		return count;
	}

	// Write back one torus part's framebuffer rows to PSRAM (so the PPA's DMA
	// reads see them).
	bool syncTorusPartToMemory(const TorusPart &part)
	{
		return syncBufferRectToMemory(frameBuffer_, logicalStride_, logicalStride_, logicalHeight(), part.torus,
		                              "torus framebuffer part");
	}

	// True when `current` equals `previous` with every image command shifted by
	// one uniform integer (dx,dy) (matched by pixel pointer + draw size) and an
	// identical opaque base. Unmatched commands (edge tiles entering the view)
	// are fine — the caller rasters their bounds. At least 2/3 must match so a
	// zoom (where sizes change) never false-positives.
	static bool detectUniformShift(const present::Frame &previous, const present::Frame &current, int *dx, int *dy)
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		if (previous.commands.size() < 2 || current.commands.size() < 2) return false;
		if (!present::commandsEqual(previous.commands[0], current.commands[0])) return false;
		bool have = false;
		int sx = 0;
		int sy = 0;
		int matched = 0;
		int images = 0;
		for (std::size_t i = 1; i < current.commands.size(); ++i) {
			const present::Command &c = current.commands[i];
			if (c.type != Type::DrawImage && c.type != Type::DrawImageScaled) return false;
			images++;
			for (std::size_t j = 1; j < previous.commands.size(); ++j) {
				const present::Command &p = previous.commands[j];
				if (p.type != c.type || p.pixels != c.pixels) continue;
				if (p.srcWidth != c.srcWidth || p.srcHeight != c.srcHeight) continue;
				if (c.type == Type::DrawImageScaled && (p.w != c.w || p.h != c.h)) continue;
				const int candX = c.x - p.x;
				const int candY = c.y - p.y;
				if (!have) {
					have = true;
					sx = candX;
					sy = candY;
				} else if (candX != sx || candY != sy) {
					return false;
				}
				matched++;
				break;
			}
		}
		if (!have || (sx == 0 && sy == 0)) return false;
		if (matched * 3 < images * 2) return false;
		*dx = sx;
		*dy = sy;
		return true;
	}

	// Rotate the full torus surface into the panel buffer: one PPA op per
	// wrapped quadrant (1, 2, or 4 big ops — efficient, unlike per-tile ops).
	// Sources must already be written back (strips are synced at raster time;
	// everything else was synced when it was last rastered).
	bool flushFullPanelFromTorus()
	{
		const present::Rect full{0, 0, logicalWidth() - 1, logicalHeight() - 1};
		TorusPart parts[4];
		const int count = splitLogicalRectToTorus(full, parts);
		take();
		bindCanvasLocked();
		if (!panel_ || frameBuffer_ != ownedFrameBuffer_) {
			give();
			return false;
		}
		bool ok = true;
		for (int i = 0; i < count && ok; ++i) {
			const TorusPart &part = parts[i];
			const int w = part.torus.x1 - part.torus.x0 + 1;
			const int h = part.torus.y1 - part.torus.y0 + 1;
			const present::Rect logicalRect{part.logicalX, part.logicalY, part.logicalX + w - 1, part.logicalY + h - 1};
			const present::Rect panelRect = physicalBoundsForLogicalRect(logicalRect);
			if (!present::valid(panelRect)) continue;
			ok = rotateTorusPartToPanelWithPpa(part, panelRect);
		}
		if (ok) {
			flushStats_.chunkCount++;
			flushStats_.pixelCount += logicalWidth() * logicalHeight();
		}
		give();
		return ok;
	}

	// Like rotateLogicalRectToPanelWithPpa, but the source block sits at the
	// part's TORUS coordinates while the panel destination comes from its
	// LOGICAL rect. No source cache sync here — the caller guarantees it.
	bool rotateTorusPartToPanelWithPpa(const TorusPart &part, present::Rect panelRect)
	{
		if (!frameBuffer_ || !panelFrameBuffer_) return false;
		panelRect = present::clampAndAlign(panelRect, kNativeWidth, kNativeHeight);
		if (!present::valid(panelRect)) return true;
		if (!initPpaSrm()) return false;

		ppa_srm_oper_config_t config{};
		config.in.buffer = frameBuffer_;
		config.in.pic_w = static_cast<std::uint32_t>(logicalWidth());
		config.in.pic_h = static_cast<std::uint32_t>(logicalHeight());
		config.in.block_w = static_cast<std::uint32_t>(part.torus.x1 - part.torus.x0 + 1);
		config.in.block_h = static_cast<std::uint32_t>(part.torus.y1 - part.torus.y0 + 1);
		config.in.block_offset_x = static_cast<std::uint32_t>(part.torus.x0);
		config.in.block_offset_y = static_cast<std::uint32_t>(part.torus.y0);
		config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		config.out.buffer = panelFrameBuffer_;
		config.out.buffer_size = static_cast<std::uint32_t>(kMaxPixels * sizeof(std::uint16_t));
		config.out.pic_w = kNativeWidth;
		config.out.pic_h = kNativeHeight;
		config.out.block_offset_x = static_cast<std::uint32_t>(panelRect.x0);
		config.out.block_offset_y = static_cast<std::uint32_t>(panelRect.y0);
		config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		config.rotation_angle = ppaRotationForCurrentOrientation();
		config.scale_x = 1.0f;
		config.scale_y = 1.0f;
		config.mode = PPA_TRANS_MODE_BLOCKING;

		const std::int64_t ppaStartUs = esp_timer_get_time();
		const esp_err_t err = ppa_do_scale_rotate_mirror(ppaSrmClient_, &config);
		flushStats_.txUs += esp_timer_get_time() - ppaStartUs;
		return err == ESP_OK;
	}

	// True when the frame is an opaque base followed by raster-safe commands —
	// originally OPAQUE IMAGE BLITS only (the tile grid), now also opaque
	// fills and text (the in-canvas sidebar): the panel-direct path does a
	// FULL re-raster in painter's order every frame, so any command the chunk
	// rasterizer supports is z-order-safe. Images must be opaque + aligned.
	static inline int gTileShapeFailKind = 0;
	static inline int gTileShapeFailType = 0;
	static inline int gTileShapeFailCount = 0;

	// Landscape pan fast path: previous frame's panel-space commands, used to
	// detect a pure translation and to dirty the moved small rects (pins).
	struct LandPrevCmd {
		const std::uint16_t *pixels;
		std::int16_t type;
		int x, y, w, h;
	};
	// Panel row where the (static) sidebar begins in landscape: the rail
	// occupies landscape x < 220 -> panel rows >= kNativeHeight - 220.
	static constexpr int kRailPanelRow = kNativeHeight - 220;
	static constexpr int kLandPrevMax = 192;
	// Last landscape frame's phase timings (never reset; GEADEV diagnostics).
	int landTotalUs_ = 0;
	int landAlignUs_ = 0;
	int landRasterUs_ = 0;
	int landTextUs_ = 0;
	int landFlipUs_ = 0;
	int landPanMode_ = 0;
	int landPanFrames_ = 0;
	int landFullFrames_ = 0;
	async_memcpy_handle_t mcp_ = nullptr;
	SemaphoreHandle_t mcpDone_ = nullptr;
	// Pan-path phase timings (us) for GEADEV LANDPAN.
	int panDetectUs_ = 0;
	int panKickUs_ = 0;
	int panStripUs_ = 0;
	int panWaitUs_ = 0;
	int panInteriorUs_ = 0;
	int panDmaErr_ = -1;
	// Why the last non-pan landscape frame bailed: 1=no snapshot, 2=no delta,
	// 3=inconsistent deltas, 4=too few pairs, 5=delta too large, 6=dirty overflow.
	int panBailReason_ = 0;
	int panBailCounts_[7] = {0, 0, 0, 0, 0, 0, 0};
	// One snapshot per scanout buffer: the pan path rasters against the
	// content ALREADY in the back buffer (frame N-2 for that buffer), so the
	// delta must be measured against that frame's command set.
	LandPrevCmd landPrev_[kDpiFrameBufferCount][kLandPrevMax];
	int landPrevCount_[kDpiFrameBufferCount] = {};
	bool landPrevValid_[kDpiFrameBufferCount] = {};
	// Hash of the rail-region commands (fills below kRailPanelRow + all text):
	// the pan path skips rail rows entirely, so ANY rail change must force a
	// full raster or sidebar state changes (search keyboard, zoom label)
	// never reach the glass.
	std::uint32_t landPrevRailHash_[kDpiFrameBufferCount] = {};

	static std::uint32_t railContentHash(const present::Frame &frame, int width, int height)
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		std::uint32_t h = 2166136261u;
		auto mix = [&h](std::uint32_t v) {
			h ^= v;
			h *= 16777619u;
		};
		for (const present::Command &cmd : frame.commands) {
			if (cmd.type == Type::FillText) {
				mix(static_cast<std::uint32_t>(cmd.x * 31 + cmd.y));
				mix(cmd.color);
				for (char c : cmd.text) mix(static_cast<std::uint32_t>(c));
				continue;
			}
			if (cmd.type != Type::FillRectRgb565) continue;
			const present::Rect b = present::commandBounds(cmd, width, height);
			if (!present::valid(b) || b.y0 < height - 220) continue;
			mix(static_cast<std::uint32_t>(cmd.x * 31 + cmd.y));
			mix(static_cast<std::uint32_t>(cmd.w * 31 + cmd.h));
			mix(cmd.color);
		}
		return h;
	}

	static bool framePpaTileEligible(const present::Frame &frame)
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		auto fail = [&](int kind, int type) {
			gTileShapeFailKind = kind;
			gTileShapeFailType = type;
			gTileShapeFailCount++;
			return false;
		};
		if (frame.commands.size() < 2) return fail(1, static_cast<int>(frame.commands.size()));
		for (std::size_t i = 1; i < frame.commands.size(); ++i) {
			const present::Command &cmd = frame.commands[i];
			if (cmd.type == Type::FillRectRgb565 || cmd.type == Type::FillText) {
				if (cmd.alpha != 255) return fail(2, static_cast<int>(cmd.type));
				continue;
			}
			if (cmd.type != Type::DrawImage && cmd.type != Type::DrawImageScaled) return fail(3, static_cast<int>(cmd.type));
			if (cmd.alphaPixels || cmd.alpha != 255 || !cmd.pixels) return fail(4, static_cast<int>(cmd.type));
			if (cmd.srcWidth <= 0 || cmd.srcHeight <= 0) return fail(5, static_cast<int>(cmd.type));
			if ((reinterpret_cast<std::uintptr_t>(cmd.pixels) & 63) != 0) return fail(6, static_cast<int>(cmd.type));
		}
		return true;
	}

	// Raster an eligible tile frame for one region with the PPA doing the pixel
	// hauling: the CPU only fills the opaque base; every tile is a hardware DMA
	// blit (with scaling) straight into the logical framebuffer. The CPU raster
	// costs ~70-100ms for a full 720x1280 composite (PSRAM bandwidth through
	// the cache); the PPA does the same haul in ~15ms with the CPU free.
	// Returns false (caller falls back to the CPU raster) when a SCALED blit
	// only partially intersects the region — PPA block offsets can't express
	// that clip exactly — or when a PPA op is rejected.
	bool ppaRasterTileRegion(const present::Frame &frame, const present::Rect &region)
	{
		if (!frameBuffer_ || !initPpaSrm()) return false;
		using Type = gea::platform::display::DisplayPresentCommandType;
		for (std::size_t i = 1; i < frame.commands.size(); ++i) {
			const present::Command &cmd = frame.commands[i];
			if (cmd.type != Type::DrawImageScaled) continue;
			if (cmd.w == cmd.srcWidth && cmd.h == cmd.srcHeight) continue;
			const present::Rect dst = present::rectFromBox(cmd.x, cmd.y, cmd.w, cmd.h);
			const bool outside = dst.x1 < region.x0 || dst.x0 > region.x1 || dst.y1 < region.y0 || dst.y0 > region.y1;
			const bool inside = dst.x0 >= region.x0 && dst.y0 >= region.y0 && dst.x1 <= region.x1 && dst.y1 <= region.y1;
			if (!outside && !inside) return false;
			const float sx = static_cast<float>(cmd.w) / cmd.srcWidth;
			const float sy = static_cast<float>(cmd.h) / cmd.srcHeight;
			// PPA SRM scale range is [1/16, 256); stay comfortably inside.
			if (sx < 0.07f || sy < 0.07f || sx >= 250.0f || sy >= 250.0f) return false;
		}

		// 1. CPU: opaque base fill across the region (Clear or full-screen fill).
		const std::uint16_t fillColor = frame.commands.front().color;
		const int rowWidth = region.x1 - region.x0 + 1;
		for (int y = region.y0; y <= region.y1; ++y) {
			std::uint16_t *row = frameBuffer_ + static_cast<std::size_t>(y) * logicalStride_ + region.x0;
			std::fill_n(row, rowWidth, fillColor);
		}
		// 2. Write the fill back to PSRAM so the PPA's writes compose on top.
		if (!syncFrameBufferRectToMemory(region)) return false;

		// 3. The blits, in painter's order (ancestor fallbacks before exact tiles).
		for (std::size_t i = 1; i < frame.commands.size(); ++i) {
			const present::Command &cmd = frame.commands[i];
			const int dstW = cmd.type == Type::DrawImageScaled ? cmd.w : cmd.srcWidth;
			const int dstH = cmd.type == Type::DrawImageScaled ? cmd.h : cmd.srcHeight;
			const bool scaled = dstW != cmd.srcWidth || dstH != cmd.srcHeight;
			const present::Rect dst = present::rectFromBox(cmd.x, cmd.y, dstW, dstH);
			const present::Rect inter{std::max(dst.x0, region.x0), std::max(dst.y0, region.y0),
			                          std::min(dst.x1, region.x1), std::min(dst.y1, region.y1)};
			if (!present::valid(inter)) continue;

			ppa_srm_oper_config_t config{};
			config.in.buffer = const_cast<void *>(static_cast<const void *>(cmd.pixels));
			config.in.pic_w = static_cast<std::uint32_t>(cmd.srcWidth);
			config.in.pic_h = static_cast<std::uint32_t>(cmd.srcHeight);
			if (scaled) {
				config.in.block_w = static_cast<std::uint32_t>(cmd.srcWidth);
				config.in.block_h = static_cast<std::uint32_t>(cmd.srcHeight);
				config.out.block_offset_x = static_cast<std::uint32_t>(dst.x0);
				config.out.block_offset_y = static_cast<std::uint32_t>(dst.y0);
				config.scale_x = static_cast<float>(dstW) / cmd.srcWidth;
				config.scale_y = static_cast<float>(dstH) / cmd.srcHeight;
			} else {
				config.in.block_w = static_cast<std::uint32_t>(inter.x1 - inter.x0 + 1);
				config.in.block_h = static_cast<std::uint32_t>(inter.y1 - inter.y0 + 1);
				config.in.block_offset_x = static_cast<std::uint32_t>(inter.x0 - dst.x0);
				config.in.block_offset_y = static_cast<std::uint32_t>(inter.y0 - dst.y0);
				config.out.block_offset_x = static_cast<std::uint32_t>(inter.x0);
				config.out.block_offset_y = static_cast<std::uint32_t>(inter.y0);
				config.scale_x = 1.0f;
				config.scale_y = 1.0f;
			}
			config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
			config.out.buffer = frameBuffer_;
			config.out.buffer_size = static_cast<std::uint32_t>(kMaxPixels * sizeof(std::uint16_t));
			config.out.pic_w = static_cast<std::uint32_t>(logicalStride_);
			config.out.pic_h = static_cast<std::uint32_t>(logicalHeight());
			config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
			config.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
			config.mode = PPA_TRANS_MODE_BLOCKING;
			if (ppa_do_scale_rotate_mirror(ppaSrmClient_, &config) != ESP_OK) return false;
		}

		// 4. Drop the CPU's now-stale cached copy of the PPA-written rows.
		// Writeback-then-invalidate over a cache-line-aligned span so neighbor
		// lines never lose dirty data (M2C demands aligned ranges).
		{
			std::uint8_t *start = reinterpret_cast<std::uint8_t *>(frameBuffer_ + static_cast<std::size_t>(region.y0) * logicalStride_);
			const std::size_t len = static_cast<std::size_t>(region.y1 - region.y0 + 1) * logicalStride_ * sizeof(std::uint16_t);
			const std::uintptr_t spanStart = reinterpret_cast<std::uintptr_t>(start) & ~static_cast<std::uintptr_t>(127);
			const std::uintptr_t spanEnd = (reinterpret_cast<std::uintptr_t>(start) + len + 127) & ~static_cast<std::uintptr_t>(127);
			esp_cache_msync(reinterpret_cast<void *>(spanStart), spanEnd - spanStart,
			                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
			esp_cache_msync(reinterpret_cast<void *>(spanStart), spanEnd - spanStart,
			                ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
		}
		return true;
	}

	void rasterPresentRegion(const present::Frame &frame, const present::Rect &region)
	{
		if (!present::valid(region)) return;
		// Torus-aware: raster the LOGICAL region into its (possibly wrapped)
		// framebuffer location(s). With origin (0,0) this is exactly one part at
		// the identity position — the historical behavior.
		TorusPart parts[4];
		const int count = splitLogicalRectToTorus(region, parts);
		const int height = logicalHeight();
		const int maxRows = std::max(1, flushRows_);
		for (int p = 0; p < count; ++p) {
			const TorusPart &part = parts[p];
			const int width = part.torus.x1 - part.torus.x0 + 1;
			const int partRows = part.torus.y1 - part.torus.y0 + 1;
			PresentRasterBand job{
				frameBuffer_ + static_cast<std::size_t>(part.torus.y0) * logicalStride_ + part.torus.x0,
				logicalStride_,
				part.logicalX,
				width,
				part.logicalY,
				height,
				maxRows,
				&frame,
			};
			const std::int64_t rasterStartUs = esp_timer_get_time();
			bool parallel = false;
			if (!onAsyncPresentWorker() &&
			    partRows >= kPresentParallelMinRows &&
			    width * partRows >= kPresentParallelMinPixels) {
				const int mainRows = partRows / 2;
				if (mainRows > 0 && mainRows < partRows &&
				    gea_render_parallel_submit(rasterPresentBandThunk, &job, mainRows, partRows - 1)) {
					rasterPresentBandRows(job, 0, mainRows - 1);
					gea_render_parallel_wait();
					parallel = true;
				}
			}
			if (!parallel) rasterPresentBandRows(job, 0, partRows - 1);
			flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;
		}
	}

	void take()
	{
		if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
	}

	void give()
	{
		if (mutex_) xSemaphoreGive(mutex_);
	}

	SemaphoreHandle_t mutex_ = nullptr;
	esp_lcd_panel_handle_t panel_ = nullptr;
	esp_lcd_panel_io_handle_t io_ = nullptr;
	esp_lcd_dsi_bus_handle_t mipiDsiBus_ = nullptr;
	esp_ldo_channel_handle_t dsiPhyPower_ = nullptr;
	ppa_client_handle_t ppaSrmClient_ = nullptr;
	std::uint16_t *frameBuffer_ = nullptr;
	std::uint16_t *ownedFrameBuffer_ = nullptr;
	std::uint16_t *panelFrameBuffer_ = nullptr;  // the buffer the DSI is currently scanning
	std::uint16_t *panelFrameBuffers_[kDpiFrameBufferCount] = {};  // DPI scanout buffers (num_fbs)
	int pendingReleasedFbIndex_ = -1;
	int scannedFbIndex_ = 0;  // which of panelFrameBuffers_ the DSI scans now (driver default 0)
	present::Rect panelBufferStaleRects_[kDpiFrameBufferCount][kMaxFlushRects] = {};
	int panelBufferStaleCount_[kDpiFrameBufferCount] = {};
	bool panelBufferStaleFull_[kDpiFrameBufferCount] = {};
	present::Rect pendingPanelBackSyncRects_[kMaxFlushRects] = {};
	int pendingPanelBackSyncCount_ = 0;
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	// Scroll origin of the logical framebuffer when it acts as a wrapping
	// (torus) scroll surface for tile-grid pans; (0,0) = plain linear layout.
	int scrollOriginX_ = 0;
	int scrollOriginY_ = 0;
	// Refresh-boundary semaphore (given at 60Hz by the DPI on_refresh_done ISR)
	// + the moment of the last double-buffer flip, for scan-aligned writes.
	SemaphoreHandle_t refreshDoneSem_ = nullptr;
	std::int64_t lastFlipUs_ = 0;
	// Landscape pipelined present: the CPU rasters frame N+1 into a compose
	// ping-pong while the PPA rotates frame N into the panel back buffer —
	// throughput becomes the rotate alone (~43ms) instead of raster+rotate.
	std::uint16_t *composeBuffers_[2] = {nullptr, nullptr};
	int composeIndex_ = 0;
	SemaphoreHandle_t ppaDoneSem_ = nullptr;
	bool ppaInFlight_ = false;
	bool ppaCallbackRegistered_ = false;
	// The panel buffer is only ever DMA-written in torus mode; one defensive
	// writeback when entering the mode, then skipped.
	bool panelSyncedForTorus_ = false;
	// Panel-direct tile mode: full-frame flips own the scanout; the composed
	// logical canvas is stale, so its flush is suppressed until transition out.
	bool panelDirectActive_ = false;
	// Present-path diagnostics.
	int presentCalls_ = 0;
	int pathDirect_ = 0;
	int pathGeneral_ = 0;
	int pathExternalFlush_ = 0;
	int pathRejected_ = 0;
	int circleSpanCacheRadius_ = -1;
	int circleSpanHalfWidths_[kCircleSpanCacheMaxRadius + 1] = {};
	gea::framework::graphics::Canvas canvas_;
	gea::framework::graphics::Canvas workerCanvas_;
	TaskHandle_t asyncPresentTask_ = nullptr;
	SemaphoreHandle_t asyncPresentDoneSem_ = nullptr;
	std::atomic<bool> asyncPresentInFlight_{false};
	present::Frame asyncPresentFrame_{};
	bool asyncPresentOk_ = true;
	int logicalStride_ = kMaxLogicalWidth;
	int renderCoreId_ = -1;
	int flushRows_ = kFlushRowsDefault;
	int flushDepth_ = 1;
	int brightness_ = 100;
	bool brightnessInitialized_ = false;
	bool initialized_ = false;
	bool frameBufferCoherent_ = true;
	bool cacheSyncWarned_ = false;
	bool ppaWarned_ = false;
	bool backgroundCacheAttempted_ = false;
	bool backdropCacheAttempted_ = false;
	platform_display::DisplayFlushPerfStats flushStats_{};
	present::Frame previousPresentFrame_{};
	present::Frame panelCirclePrevFrame_[kDpiFrameBufferCount]{};
	bool panelCirclePrevValid_[kDpiFrameBufferCount] = {};
	bool previousPresentValid_ = false;
	bool needsFullPhysicalFlush_ = true;
};

}  // namespace gea::platform::esp32_p4_m5stack_tab5::display

extern "C" std::uint16_t *gea_bg_cache(int *cap_px)
{
	return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().backgroundCache(cap_px);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().backdropCache(cap_px);
}

namespace {

struct P4RenderWorker {
	TaskHandle_t task = nullptr;
	void (*volatile fn)(void *, int, int) = nullptr;
	void *volatile ctx = nullptr;
	volatile int y0 = 0;
	volatile int y1 = 0;
	std::atomic<std::uint32_t> jobSeq{0};
	std::atomic<bool> done{true};
	bool attempted = false;
};

P4RenderWorker gRenderWorker;
constexpr std::uint32_t kRenderWorkerStackBytes = 12288;

void renderWorkerTask(void *)
{
	// MUST start at 0 (jobSeq's initial value), NOT jobSeq.load(): submit() creates
	// this task and increments jobSeq immediately after, so loading here usually
	// reads 1 and swallows job 1 (the caller times out and the first parallel band
	// is silently dropped). Same race fixed on the elecrow in d22bfc5f9.
	std::uint32_t seen = 0;
	std::int64_t idleSince = esp_timer_get_time();
	for (;;) {
		while (gRenderWorker.jobSeq.load(std::memory_order_acquire) == seen) {
			if (esp_timer_get_time() - idleSince > 4000) {
				ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
				idleSince = esp_timer_get_time();
			}
		}
		seen = gRenderWorker.jobSeq.load(std::memory_order_acquire);
		void (*fn)(void *, int, int) = gRenderWorker.fn;
		if (fn) fn(gRenderWorker.ctx, gRenderWorker.y0, gRenderWorker.y1);
		gRenderWorker.done.store(true, std::memory_order_release);
		idleSince = esp_timer_get_time();
	}
}

}  // namespace

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	if (!gRenderWorker.attempted) {
		gRenderWorker.attempted = true;
		const BaseType_t renderCore = xPortGetCoreID();
		const BaseType_t workerCore = (renderCore == 0) ? 1 : 0;
		const UBaseType_t priority = uxTaskPriorityGet(nullptr);
		const BaseType_t created = xTaskCreatePinnedToCore(renderWorkerTask,
		                                                   "gea_rwrk",
		                                                   kRenderWorkerStackBytes,
		                                                   nullptr,
		                                                   priority,
		                                                   &gRenderWorker.task,
		                                                   workerCore);
		if (created != pdPASS || !gRenderWorker.task) {
			multi_heap_info_t info{};
			heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
			ESP_LOGE("m5stack_tab5_display",
			         "render worker creation failed: create=%d stack=%u internal_free=%u internal_largest=%u",
			         static_cast<int>(created),
			         static_cast<unsigned>(kRenderWorkerStackBytes),
			         static_cast<unsigned>(info.total_free_bytes),
			         static_cast<unsigned>(info.largest_free_block));
		}
	}
	if (!gRenderWorker.task) return false;
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().setRenderCore(static_cast<int>(xPortGetCoreID()));
	gRenderWorker.fn = fn;
	gRenderWorker.ctx = ctx;
	gRenderWorker.y0 = y0;
	gRenderWorker.y1 = y1;
	gRenderWorker.done.store(false, std::memory_order_release);
	gRenderWorker.jobSeq.fetch_add(1, std::memory_order_release);
	xTaskNotifyGive(gRenderWorker.task);
	return true;
}

extern "C" void gea_render_parallel_wait()
{
	if (!gRenderWorker.task) return;
	const std::int64_t deadline = esp_timer_get_time() + 100000;
	while (!gRenderWorker.done.load(std::memory_order_acquire)) {
		if (esp_timer_get_time() > deadline) {
			ESP_LOGW("m5stack_tab5_display", "render worker wait timed out");
			break;
		}
	}
}

extern "C" void gea_render_parallel_merge_dirty()
{
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().absorbWorkerDirty();
}

extern "C" int gea_current_render_core()
{
	return static_cast<int>(xPortGetCoreID());
}

namespace gea::platform::display {

namespace {

gea::framework::graphics::Canvas *drawingCanvas()
{
	return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().drawingCanvas();
}

}  // namespace

bool Display::init() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().init(); }
bool Display::start() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().start(); }
gea::framework::graphics::Canvas *Display::canvas() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().canvas(); }
bool Display::framebufferIsPanelDirect() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().framebufferIsPanelDirect(); }
bool Display::panelScanoutSurface(uint16_t **outBuffer, int *outW, int *outH) { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().panelScanoutSurface(outBuffer, outW, outH); }
bool Display::panelDirectTarget(int logicalX, int logicalY, int logicalW, int logicalH,
                                uint16_t **outBuf, int *outBufW, int *outBufH,
                                int *outPanelX, int *outPanelY, int *outPanelW, int *outPanelH,
                                int *outRotSteps, int *outFlip) {
	return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().panelDirectTarget(
	    logicalX, logicalY, logicalW, logicalH, outBuf, outBufW, outBufH, outPanelX, outPanelY, outPanelW, outPanelH, outRotSteps, outFlip);
}
void Display::flipPanelToBack() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flipPanelToBack(); }
void Display::flipPanelToBack(uint16_t *buffer) { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flipPanelToBack(buffer); }
void Display::clear() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().clear(); }
void Display::clearNoFlush() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flush(); }
void Display::rebindCanvasToFramebuffer() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().rebindCanvasToFramebuffer(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushRects(rects, count); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int command_count) { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().presentCommands(commands, command_count); }
void Display::presentPathDebug(int *calls, int *direct, int *general, int *rejected, int *tileShapeFailKind, int *tileShapeFailType, int *tileShapeFailCount)
{
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().presentPathDebug(
	    calls, direct, general, rejected, tileShapeFailKind, tileShapeFailType, tileShapeFailCount);
}
void Display::landFrameDebug(int *total, int *align, int *raster, int *text, int *flip)
{
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().landFrameDebug(total, align, raster, text, flip);
}
void Display::landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior)
{
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().landPanDebug(detect, kick, strips, wait, interior);
}
void Display::setFlushConfig(int chunk_rows, int queue_depth) { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().setFlushConfig(chunk_rows, queue_depth); }
void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushChunkRows(); }
int Display::flushQueueDepth() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushQueueDepth(); }
int Display::flushBufferBytes() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushBufferBytes(); }
void Display::pushClip(int x, int y, int w, int h) { if (auto *c = drawingCanvas()) c->pushClip(x, y, w, h); }
void Display::popClip() { if (auto *c = drawingCanvas()) c->popClip(); }
void Display::resetClip() { if (auto *c = drawingCanvas()) c->resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { if (auto *c = drawingCanvas()) c->setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return drawingCanvas() ? drawingCanvas()->globalAlpha() : 255; }
int Display::brightness() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().brightness(); }
void Display::setBrightness(int brightness_percent) { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().setBrightness(brightness_percent); }
// Tearing sync (TE/VBlank): not implemented on this target.
void Display::setVSync(bool) {}
void Display::invalidate() {}
bool Display::vsyncEnabled() { return false; }
void Display::vsyncWaitForFrame() {}
void Display::clip(int *x0, int *y0, int *x1, int *y1)
{
	if (auto *c = drawingCanvas()) c->currentClip(x0, y0, x1, y1);
}
void Display::fillRect(int x, int y, int w, int h, std::uint16_t color) { if (auto *c = drawingCanvas()) c->fillRect(x, y, w, h, color); }
void Display::scrollRect(int x, int y, int w, int h, int dx, int dy) { if (auto *c = drawingCanvas()) c->scrollRect(x, y, w, h, dx, dy); }
void Display::resetScrollRegion() {}
void Display::strokeRect(int x, int y, int w, int h, std::uint16_t color) { if (auto *c = drawingCanvas()) c->strokeRect(x, y, w, h, color); }
void Display::fillCircle(int cx, int cy, int r, std::uint16_t color) { if (auto *c = drawingCanvas()) c->fillCircle(cx, cy, r, color); }
void Display::strokeCircle(int cx, int cy, int r, std::uint16_t color) { if (auto *c = drawingCanvas()) c->strokeCircle(cx, cy, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, std::uint16_t color) { if (auto *c = drawingCanvas()) c->drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int cx, int cy, int r, int start_deg, int end_deg, std::uint16_t color) { if (auto *c = drawingCanvas()) c->drawArc(cx, cy, r, start_deg, end_deg, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, std::uint16_t color) { if (auto *c = drawingCanvas()) c->fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, std::uint16_t color, float scale) { if (auto *c = drawingCanvas()) c->drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, std::uint16_t color, int font_id) { if (auto *c = drawingCanvas()) c->drawTextFont(text, x, y, color, font_id); }
void Display::drawTextFontFamily(const char *text, int x, int y, std::uint16_t color, int family_id, int size_px) { if (auto *c = drawingCanvas()) c->drawTextFontFamily(text, x, y, color, family_id, size_px); }
void Display::setPixel(int x, int y, std::uint16_t color) { if (auto *c = drawingCanvas()) c->fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, std::uint16_t color) { if (auto *c = drawingCanvas()) c->fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count, int w, int h, int tl, int tr, int br, int bl, const std::uint16_t *colors) { if (auto *c = drawingCanvas()) c->fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors); }
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lw, std::uint16_t color) { if (auto *c = drawingCanvas()) c->strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lw, color); }
void Display::blitImage(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int src_w, int src_h, int dx, int dy) { if (auto *c = drawingCanvas()) c->drawImage(src, alpha, src_w, src_h, dx, dy); }
void Display::blitImageScaled(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int src_w, int src_h, int dx, int dy, int dst_w, int dst_h) { if (auto *c = drawingCanvas()) c->drawImage(src, alpha, src_w, src_h, dx, dy, dst_w, dst_h); }
void Display::setWorldOverlay(const std::uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}
void Display::flushStatsRead(std::int64_t *total_us, int *call_count, int *pixel_count)
{
	const auto stats = gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStats();
	if (total_us) *total_us = stats.totalUs;
	if (call_count) *call_count = stats.callCount;
	if (pixel_count) *pixel_count = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStats(); }
void Display::flushStatsReset() { gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStatsReset(); }
const char *Display::flushStageName() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStageName(); }
int Display::flushStageChunk() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStageChunk(); }
DisplayFlushStageDetail Display::flushStageDetail() { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().flushStageDetail(); }
bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixel_capacity, int *width, int *height, bool) { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().copySnapshotRgb565(dst, pixel_capacity, width, height); }
int Display::countNonBlackPixels(bool) { return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().countNonBlackPixels(); }

extern "C" bool geaDisplaySnapshotPrefersPresented()
{
	return gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().snapshotPrefersPresented();
}

void applyOrientation(gea::framework::display::DisplayOrientation)
{
	gea::platform::esp32_p4_m5stack_tab5::display::DisplayBackend::instance().applyOrientation();
}

bool autoRotateAllowed()
{
	return false;
}

}  // namespace gea::platform::display
