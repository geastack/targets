#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "host/display_orientation.h"
#include "i2c.h"
#include "pixel.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;
namespace present = gea::framework::display_present;

namespace gea::platform::esp32_s3_sticks3::display {

namespace {

constexpr const char *kTag = "sticks3_display";
constexpr int kNativeWidth = platform_display::kNativeWidth;    // 135
constexpr int kNativeHeight = platform_display::kNativeHeight;  // 240
constexpr int kMaxLogicalWidth = kNativeHeight > kNativeWidth ? kNativeHeight : kNativeWidth;
constexpr int kMaxPixels = kNativeWidth * kNativeHeight;
constexpr int kFlushRowsDefault = 64;
// ST7789V2 SPI panel (M5Stack StickS3): 135x240 on a 240x320 GRAM, SPI mode 0,
// 8-bit command/parameter, color-inverted, RGB element order. 80 MHz pixel
// clock — over the ST7789V2's spec'd ~62 MHz write cycle, but overclocking
// ST7789 glass to 80 MHz is common (its ST7796 sibling runs 80 here) and
// this panel was verified visually clean at 80 MHz on-device. It does NOT
// raise fps over 40 MHz: bouncing-balls-jsx is rasterization-bound (~11 ms
// display-list replay/frame → ~71 fps), and the panel-push task already
// hides the SPI wire, so the clock isn't on the critical path. Kept at 80
// for latency headroom. (An earlier "80 MHz blanks the panel" reading was a
// false attribution — the glass was dark because the M5PM1 L3B rail was
// unpowered, not the clock.) Glass window sits at GRAM offset (52,40) in the
// panel's native portrait addressing; esp_lcd_panel_set_gap carries the
// offset so draw_bitmap coords stay 0-based. Framebuffer is panel-endian, so
// staging is a straight memcpy.
constexpr int kSpiPclkHz = 80 * 1000 * 1000;
constexpr int kPanelBitsPerPixel = 16;
constexpr int kSpiTransQueueDepth = 8;
// Four staging bands cover a whole 240-row frame (4 x 64-row chunks), so a
// full-screen flush queues every chunk and returns with the ~13 ms of wire
// time riding under the NEXT frame's rasterization instead of gating the
// frame loop (the panel-side cap is then the wire rate, ~76 Hz).
constexpr int kStagingDepth = 4;
constexpr int kGapPortraitPrimaryX = 52;
constexpr int kGapPortraitPrimaryY = 40;
// MADCTL MX+MY flips addressing across the 240x320 GRAM; the 135-column glass
// splits the 105 spare columns 52/53, so the mirrored gap is one column over.
constexpr int kGapPortraitSecondaryX = 53;
constexpr int kGapPortraitSecondaryY = 40;
// Flush rotation modes: landscape logical frames are rotated into the panel's
// portrait scan order at flush time instead of via MADCTL (which would make
// GRAM writes perpendicular to the gate scan and shear/tear on motion).
constexpr int kFlushRotationNone = 0;
constexpr int kFlushRotationLandscapePrimary = 1;
constexpr int kFlushRotationLandscapeSecondary = 2;
// Internal-RAM bounce band for CPU-composed flushes. Bands are at most
// kNativeWidth pixels wide in BOTH orientations: the portrait linear path
// pushes full-width (135 px) rows, and the landscape path rotates into
// panel-order bands whose width is the panel's 135-px scan line.
constexpr int kStagingBytes = kNativeWidth * kFlushRowsDefault * static_cast<int>(sizeof(std::uint16_t));
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_1;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_1;
constexpr int kBacklightLedcMaxDuty = 1023;
constexpr int kMaxPresentRects = 16;
constexpr int kOwnedFrameBufferPixels = kMaxPixels;

present::Rect clampLogicalRect(present::Rect rect, int width, int height)
{
	if (width <= 0 || height <= 0) return {};
	rect = present::clampAndAlign(rect, width, height);
	if (!present::valid(rect)) return {};
	return rect;
}

int integerSqrt(int value)
{
	if (value <= 0) return 0;
	int result = 0;
	int bit = 1 << 14;
	while (bit > value) bit >>= 2;
	while (bit != 0) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}
		bit >>= 2;
	}
	return result;
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

		{
			const std::size_t fbBytes = static_cast<std::size_t>(kOwnedFrameBufferPixels) * sizeof(std::uint16_t);
			ownedFrameBuffer_ = static_cast<std::uint16_t *>(
				heap_caps_malloc(fbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
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
		if (!frameBuffer_) init();
		return &replayCanvas();
	}

	// A push panel never exposes its own scanout buffer.
	bool framebufferIsPanelDirect() const { return false; }

	gea::framework::graphics::Canvas *drawingCanvas()
	{
		if (!frameBuffer_) init();
		return &replayCanvas();
	}

	void applyOrientation()
	{
		take();
		// MADCTL/gap commands must not interleave with a band on the wire.
		waitPanelPushIdle();
		bindCanvasLocked();
		applyPanelOrientation();
		if (!frameBuffer_) {
			give();
			return;
		}
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		previousPresentValid_ = false;
		needsFullPhysicalFlush_ = true;
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
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		previousPresentValid_ = false;
		needsFullPhysicalFlush_ = true;
		give();
	}

	void flush()
	{
		if (!init()) return;
		const bool fullPhysicalFlush = needsFullPhysicalFlush_;
		if (!fullPhysicalFlush && !canvas_.dirty(nullptr, nullptr, nullptr, nullptr)) return;
		const std::int64_t started = esp_timer_get_time();
		gea::framework::graphics::CanvasDirtyRect dirtyRects[gea::framework::graphics::Canvas::kMaxDirtyRects];
		int dirtyCount = 0;
		if (fullPhysicalFlush) {
			dirtyRects[0] = {0, 0, logicalWidth() - 1, logicalHeight() - 1};
			dirtyCount = 1;
			ESP_LOGI(kTag, "first physical flush covers full logical frame %dx%d", logicalWidth(), logicalHeight());
		} else {
			dirtyCount = canvas_.dirtyRects(dirtyRects, gea::framework::graphics::Canvas::kMaxDirtyRects);
		}
		if (!fullPhysicalFlush && dirtyCount <= 0) {
			int x0 = 0;
			int y0 = 0;
			int x1 = -1;
			int y1 = -1;
			if (canvas_.dirty(&x0, &y0, &x1, &y1)) {
				dirtyRects[0] = {x0, y0, x1, y1};
				dirtyCount = 1;
			}
		}
		bool ok = true;
		flushStats_.callCount++;
		for (int i = 0; i < dirtyCount; ++i) {
			const auto &rect = dirtyRects[i];
			if (!flushNativeLogicalRect({rect.x0, rect.y0, rect.x1, rect.y1})) {
				ok = false;
				break;
			}
		}
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) {
			canvas_.resetDirty();
			if (fullPhysicalFlush) needsFullPhysicalFlush_ = false;
		}
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		if (!rects || count <= 0) return;
		if (!init()) return;
		if (needsFullPhysicalFlush_) {
			const std::int64_t started = esp_timer_get_time();
			flushStats_.callCount++;
			ESP_LOGI(kTag, "first physical flush covers full logical frame %dx%d", logicalWidth(), logicalHeight());
			const bool ok = flushNativeLogicalRect({0, 0, logicalWidth() - 1, logicalHeight() - 1});
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
		for (int i = 0; i < count; ++i) {
			const auto &rect = rects[i];
			if (!flushNativeLogicalRect({rect.x0, rect.y0, rect.x1, rect.y1})) {
				ok = false;
				break;
			}
		}
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) canvas_.resetDirty();
	}

	bool streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user)
	{
		if (!raster || w <= 0 || h <= 0) return false;
		if (!init()) return false;
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
		flush();
		return true;
	}

	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int commandCount)
	{
		if (!commands || commandCount <= 0 || !init()) return false;
		present::Frame current;
		if (!present::extractFrame(commands, commandCount, current)) return false;
		if (!present::frameHasOpaqueBase(current, logicalWidth(), logicalHeight())) return false;
		if (isFastClearFillCirclesFrame(current) && incrementalCirclePathBeneficial(current)) {
			const bool ok = presentClearFillCirclesFrame(current);
			if (ok) {
				previousPresentFrame_ = std::move(current);
				previousPresentValid_ = true;
				needsFullPhysicalFlush_ = false;
				canvas_.resetDirty();
			}
			return ok;
		}
		present::Rect regions[kMaxPresentRects];
		int regionCount = 0;
		if (previousPresentValid_) {
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

		const std::int64_t started = esp_timer_get_time();
		bool ok = true;
		flushStats_.callCount++;
		for (int i = 0; i < regionCount; ++i) {
			present::Rect region = clampLogicalRect(regions[i], logicalWidth(), logicalHeight());
			if (!present::valid(region)) continue;
			rasterPresentRegion(current, region);
			ok = flushNativeLogicalRect(region);
			if (!ok) break;
		}
		flushStats_.totalUs += esp_timer_get_time() - started;
		if (ok) {
			previousPresentFrame_ = std::move(current);
			previousPresentValid_ = true;
			needsFullPhysicalFlush_ = false;
			canvas_.resetDirty();
		}
		return ok;
	}

	// A frame whose base layer is [Clear, FillCircles]. Commands AFTER the circle
	// batch (FPS text, a HUD rect, ...) are overlays drawn on top — they ride the
	// incremental path via a per-frame overlay re-raster.
	static bool isFastClearFillCirclesFrame(const present::Frame &frame)
	{
		using Type = gea::platform::display::DisplayPresentCommandType;
		if (frame.commands.size() < 2) return false;
		const present::Command &clear = frame.commands[0];
		const present::Command &circles = frame.commands[1];
		if (clear.type != Type::Clear || circles.type != Type::FillCirclesRgb565) return false;
		if (circles.alpha != 255 || circles.radius <= 0 || circles.circlesCount < 0) return false;
		return circles.hasCircleEntries();
	}

	// The incremental circle path erases + redraws every ball to save the
	// writeback of the untouched area; only worth it when the touched area is
	// small relative to the screen.
	bool incrementalCirclePathBeneficial(const present::Frame &current) const
	{
		if (!previousPresentValid_ || needsFullPhysicalFlush_) return false;
		if (!isFastClearFillCirclesFrame(previousPresentFrame_)) return false;
		if (previousPresentFrame_.commands[0].color != current.commands[0].color) return false;
		present::Rect dirty = present::unite(circleBatchBounds(previousPresentFrame_.commands[1]),
		                                     circleBatchBounds(current.commands[1]));
		for (std::size_t i = 2; i < current.commands.size(); ++i)
			dirty = present::unite(dirty, present::commandBounds(current.commands[i], logicalWidth(), logicalHeight()));
		for (std::size_t i = 2; i < previousPresentFrame_.commands.size(); ++i)
			dirty = present::unite(dirty, present::commandBounds(previousPresentFrame_.commands[i], logicalWidth(), logicalHeight()));
		dirty = clampLogicalRect(dirty, logicalWidth(), logicalHeight());
		if (!present::valid(dirty)) return true;  // nothing moved — trivially cheap
		const long long dirtyArea = static_cast<long long>(dirty.x1 - dirty.x0 + 1) * (dirty.y1 - dirty.y0 + 1);
		const long long screenArea = static_cast<long long>(logicalWidth()) * logicalHeight();
		return dirtyArea * 3 < screenArea;  // only when < ~1/3 of the screen is touched
	}

	bool presentClearFillCirclesFrame(const present::Frame &current)
	{
		const present::Command &clear = current.commands[0];
		const present::Command &circles = current.commands[1];
		const bool canIncremental =
			previousPresentValid_ &&
			isFastClearFillCirclesFrame(previousPresentFrame_) &&
			previousPresentFrame_.commands[0].color == clear.color;

		// Overlays = every command past the circle batch (e.g. the FPS text).
		// Their box — current AND previous (to erase the stale overlay) — is
		// repainted independently and re-composited from scratch.
		present::Rect overlayDirty{};
		for (std::size_t i = 2; i < current.commands.size(); ++i)
			overlayDirty = present::unite(overlayDirty, present::commandBounds(current.commands[i], logicalWidth(), logicalHeight()));
		if (canIncremental)
			for (std::size_t i = 2; i < previousPresentFrame_.commands.size(); ++i)
				overlayDirty = present::unite(overlayDirty, present::commandBounds(previousPresentFrame_.commands[i], logicalWidth(), logicalHeight()));
		overlayDirty = clampLogicalRect(overlayDirty, logicalWidth(), logicalHeight());

		present::Rect dirty = present::fullScreen(logicalWidth(), logicalHeight());
		if (canIncremental && !needsFullPhysicalFlush_) {
			dirty = present::unite(circleBatchBounds(previousPresentFrame_.commands[1]), circleBatchBounds(circles));
			if (present::valid(overlayDirty)) dirty = present::unite(dirty, overlayDirty);
			dirty = clampLogicalRect(dirty, logicalWidth(), logicalHeight());
			if (!present::valid(dirty)) return true;
		}

		const std::int64_t started = esp_timer_get_time();
		flushStats_.callCount++;
		const std::int64_t rasterStartUs = esp_timer_get_time();
		if (canIncremental && !needsFullPhysicalFlush_) {
			drawCircleBatchRaw(previousPresentFrame_.commands[1], clear.color, true);
		} else {
			clearFrameBufferRows(0, logicalHeight() - 1, clear.color);
		}
		drawCircleBatchRaw(circles, 0, false);
		// Composite overlays: re-raster their box from the full frame so the old
		// overlay is erased and the new one lands on top of whatever balls sit
		// behind it.
		if (present::valid(overlayDirty)) rasterPresentRegion(current, overlayDirty);
		flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;

		const bool ok = flushNativeLogicalRect(dirty);
		flushStats_.totalUs += esp_timer_get_time() - started;
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
				const int halfWidth = integerSqrt(r * r - dy * dy);
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
	int flushBufferBytes() const { return kStagingBytes * kStagingDepth; }

	bool copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height)
	{
		if (!dst || pixelCapacity < logicalWidth() * logicalHeight()) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		for (int row = 0; row < logicalHeight(); ++row) {
			std::memcpy(dst + static_cast<std::size_t>(row) * logicalWidth(),
			            frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_,
			            static_cast<std::size_t>(logicalWidth()) * sizeof(std::uint16_t));
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

	// Restore the draw canvas to the framebuffer after a caller (e.g. the render
	// layer's retained-snapshot path) temporarily bound it onto another buffer.
	void rebindCanvasToFramebuffer()
	{
		take();
		bindCanvasLocked();
		give();
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
	gea::framework::graphics::Canvas &replayCanvas() { return onWorkerCore() ? workerCanvas_ : canvas_; }

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
		if (!ownedFrameBuffer_) return;
		if (frameBuffer_ != ownedFrameBuffer_) frameBuffer_ = ownedFrameBuffer_;
		if (canvas_.pixels() == frameBuffer_ && canvas_.width() == width && canvas_.height() == height && logicalStride_ == stride) return;
		logicalStride_ = stride;
		canvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
		workerCanvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
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
		const std::uint32_t duty = static_cast<std::uint32_t>((kBacklightLedcMaxDuty * brightness_) / 100);
		const esp_err_t setErr = ledc_set_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel, duty);
		const esp_err_t updateErr = ledc_update_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel);
		if (setErr != ESP_OK || updateErr != ESP_OK) {
			ESP_LOGE(kTag, "backlight duty update failed: set=%s update=%s",
			         esp_err_to_name(setErr),
			         esp_err_to_name(updateErr));
		}
	}

	static bool IRAM_ATTR onColorTransDone(esp_lcd_panel_io_handle_t,
	                                       esp_lcd_panel_io_event_data_t *,
	                                       void *userCtx)
	{
		auto *self = static_cast<DisplayBackend *>(userCtx);
		BaseType_t woken = pdFALSE;
		if (self && self->flushDoneSem_) xSemaphoreGiveFromISR(self->flushDoneSem_, &woken);
		return woken == pdTRUE;
	}

	// The ST7789's gate driver always scans the physical (portrait) lines —
	// MADCTL only remaps GRAM *addressing*, not the refresh scan. Writing
	// landscape full-width bands through a swapped MADCTL would advance
	// PERPENDICULAR to the panel's scan and shear on motion, so landscape
	// orientations keep the panel in PORTRAIT addressing and the flush path
	// rotates each band into scan order instead (flushRotatedLogicalRect);
	// only portrait orientations use MADCTL. The GRAM gap moves with the
	// mirror (the 240x320 GRAM window shifts across the 135x240 glass).
	void applyPanelOrientation()
	{
		if (!panel_) return;
		using gea::framework::display::DisplayOrientation;
		bool mirrorX = false;
		bool mirrorY = false;
		int gapX = kGapPortraitPrimaryX;
		int gapY = kGapPortraitPrimaryY;
		flushRotation_ = kFlushRotationNone;
		switch (orientation_detail::DisplayOrientationState::orientation()) {
		case DisplayOrientation::LandscapePrimary:
			// Portrait addressing + rotate-on-flush:
			// panel x = logical y, panel y = (panelH-1) - logical x.
			flushRotation_ = kFlushRotationLandscapePrimary;
			break;
		case DisplayOrientation::LandscapeSecondary:
			// panel x = (panelW-1) - logical y, panel y = logical x.
			flushRotation_ = kFlushRotationLandscapeSecondary;
			break;
		case DisplayOrientation::PortraitSecondary:
			mirrorX = true;
			mirrorY = true;
			gapX = kGapPortraitSecondaryX;
			gapY = kGapPortraitSecondaryY;
			break;
		case DisplayOrientation::PortraitPrimary:
		default:
			break;
		}
		esp_lcd_panel_swap_xy(panel_, false);
		esp_lcd_panel_mirror(panel_, mirrorX, mirrorY);
		esp_lcd_panel_set_gap(panel_, gapX, gapY);
	}

	// Drive the M5PM1 PMIC's GPIO2 high — the "L3B" rail that powers the LCD
	// panel + backlight on the StickS3. The exact register sequence is M5GFX's
	// board_M5StickS3 bring-up ("PM1_G2 -- L3B Enable, LCD Power On"): make
	// GPIO2 a push-pull output driven high, then disable the PMIC's I2C idle
	// sleep so it keeps answering. On a cold boot this rail is OFF, so without
	// it the panel and backlight are unpowered and the glass stays dark no
	// matter what the SPI side does (ST7789 never ACKs, so flushes still
	// "succeed"). M5PM1 @ 0x6E, 100 kHz.
	bool powerOnLcdRail()
	{
		constexpr std::uint8_t kAddr = 0x6E;
		constexpr std::uint8_t kGpio2 = 1 << 2;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus unavailable; cannot power LCD rail");
			return false;
		}
		auto busHandle = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
		i2c_device_config_t devCfg = {};
		devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		devCfg.device_address = kAddr;
		devCfg.scl_speed_hz = 100000;
		i2c_master_dev_handle_t dev = nullptr;
		esp_err_t err = i2c_master_bus_add_device(busHandle, &devCfg, &dev);
		if (err != ESP_OK || !dev) {
			ESP_LOGE(kTag, "M5PM1 add_device failed: %s", esp_err_to_name(err));
			return false;
		}

		constexpr int kTimeoutMs = 100;
		auto write8 = [&](std::uint8_t reg, std::uint8_t val) {
			const std::uint8_t buf[2] = {reg, val};
			return i2c_master_transmit(dev, buf, sizeof(buf), kTimeoutMs) == ESP_OK;
		};
		auto read8 = [&](std::uint8_t reg, std::uint8_t &val) {
			return i2c_master_transmit_receive(dev, &reg, 1, &val, 1, kTimeoutMs) == ESP_OK;
		};
		auto bitOff = [&](std::uint8_t reg, std::uint8_t mask) {
			std::uint8_t v = 0;
			return read8(reg, v) && write8(reg, static_cast<std::uint8_t>(v & ~mask));
		};
		auto bitOn = [&](std::uint8_t reg, std::uint8_t mask) {
			std::uint8_t v = 0;
			return read8(reg, v) && write8(reg, static_cast<std::uint8_t>(v | mask));
		};

		bool ok = true;
		ok = bitOff(0x16, kGpio2) && ok;  // gpio2 = gpio function (not alternate)
		ok = bitOn(0x10, kGpio2) && ok;   // gpio2 = output
		ok = bitOff(0x13, kGpio2) && ok;  // gpio2 = push-pull
		ok = bitOn(0x11, kGpio2) && ok;   // gpio2 = HIGH -> LCD/backlight rail on
		ok = write8(0x09, 0x00) && ok;    // disable PMIC I2C idle sleep
		i2c_master_bus_rm_device(dev);
		if (!ok) {
			ESP_LOGE(kTag, "M5PM1 LCD-rail enable had I2C failures");
			return false;
		}
		// Let the rail settle before resetting/initializing the panel (M5GFX
		// waits 100 ms here too).
		vTaskDelay(pdMS_TO_TICKS(100));
		ESP_LOGI(kTag, "M5PM1 L3B rail enabled (LCD power on)");
		return true;
	}

	bool initPanel()
	{
		if (panel_) return true;
		// LCD + backlight power rail FIRST — the panel is unpowered until this
		// runs, so backlight PWM and SPI writes are no-ops before it.
		powerOnLcdRail();
		if (!initBacklight()) return false;

		// Rotating internal-RAM staging bands, allocated FIRST so the ring depth
		// can adapt to how much DMA-capable internal RAM is actually free. Each
		// flush copies a full-width band of the PSRAM framebuffer here, then DMAs
		// it over SPI; with multiple bands the memcpy of the next band runs while
		// the previous one is still on the wire. A connectivity-heavy app
		// (WiFi+BLE+audio) reserves a large slice of internal RAM, leaving too
		// little for the full 4-band ring — rather than kill the panel we degrade
		// to as many bands as fit (min 1). Fewer bands just means less
		// rasterize/DMA overlap (lower peak fps), not a dead display: the right
		// trade for a diagnostics screen.
		stagingDepth_ = 0;
		for (int i = 0; i < kStagingDepth; ++i) {
			stagingPixels_[i] = static_cast<std::uint16_t *>(
				heap_caps_malloc(static_cast<std::size_t>(kStagingBytes), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (!stagingPixels_[i]) break;
			stagingDepth_ = i + 1;
		}
		if (stagingDepth_ == 0) {
			ESP_LOGE(kTag, "display staging band alloc (%d bytes) failed", kStagingBytes);
			return false;
		}
		if (stagingDepth_ < kStagingDepth) {
			ESP_LOGW(kTag, "display staging ring reduced to %d/%d bands (%d bytes each) under RAM pressure",
				stagingDepth_, kStagingDepth, kStagingBytes);
		}

		// DMA-completion handshake for the push task (one transfer on the wire at
		// a time), plus the staging ring's producer/consumer semaphores sized to
		// the actual band count.
		flushDoneSem_ = xSemaphoreCreateBinary();
		slotFree_ = xSemaphoreCreateCounting(stagingDepth_, stagingDepth_);
		slotReady_ = xSemaphoreCreateCounting(stagingDepth_, 0);
		if (!flushDoneSem_ || !slotFree_ || !slotReady_) {
			ESP_LOGE(kTag, "failed to allocate flush semaphores");
			return false;
		}

		spi_bus_config_t busConfig = {};
		busConfig.mosi_io_num = gea::platform::board::display.mosi;
		busConfig.miso_io_num = GPIO_NUM_NC;
		busConfig.sclk_io_num = gea::platform::board::display.sclk;
		busConfig.quadwp_io_num = GPIO_NUM_NC;
		busConfig.quadhd_io_num = GPIO_NUM_NC;
		busConfig.max_transfer_sz = kStagingBytes;
		esp_err_t err = spi_bus_initialize(gea::platform::board::display.spiHost, &busConfig, SPI_DMA_CH_AUTO);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_lcd_panel_io_spi_config_t ioConfig = {};
		ioConfig.cs_gpio_num = gea::platform::board::display.cs;
		ioConfig.dc_gpio_num = gea::platform::board::display.dc;
		ioConfig.spi_mode = 0;
		ioConfig.pclk_hz = kSpiPclkHz;
		ioConfig.trans_queue_depth = kSpiTransQueueDepth;
		ioConfig.lcd_cmd_bits = 8;
		ioConfig.lcd_param_bits = 8;
		ioConfig.on_color_trans_done = &DisplayBackend::onColorTransDone;
		ioConfig.user_ctx = this;
		err = esp_lcd_new_panel_io_spi(
			static_cast<esp_lcd_spi_bus_handle_t>(gea::platform::board::display.spiHost), &ioConfig, &io_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI panel IO init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_lcd_panel_dev_config_t panelConfig = {};
		panelConfig.reset_gpio_num = gea::platform::board::display.reset;
		panelConfig.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
		panelConfig.bits_per_pixel = kPanelBitsPerPixel;
		err = esp_lcd_new_panel_st7789(io_, &panelConfig, &panel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "ST7789 panel create failed: %s", esp_err_to_name(err));
			return false;
		}
		err = esp_lcd_panel_reset(panel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "panel reset failed: %s", esp_err_to_name(err));
			return false;
		}
		err = esp_lcd_panel_init(panel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "panel init failed: %s", esp_err_to_name(err));
			return false;
		}
		// The StickS3 glass runs color-inverted (M5GFX: invert = true).
		esp_lcd_panel_invert_color(panel_, true);
		applyPanelOrientation();
		err = esp_lcd_panel_disp_on_off(panel_, true);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "panel on failed: %s", esp_err_to_name(err));
			return false;
		}
		// Panel push worker on the core opposite the app frame task (core 0);
		// created after panel init so bring-up commands never interleave with
		// a queued band.
		if (xTaskCreatePinnedToCore(&DisplayBackend::panelPushTaskEntry, "panel_push", 4096, this, 5, &pushTask_, 1) != pdPASS) {
			ESP_LOGE(kTag, "failed to start panel push task");
			return false;
		}
		ESP_LOGI(kTag,
		         "ST7789 SPI panel ready %dx%d host=%d pclk=%dMHz mode=0 cs=%d dc=%d rst=%d bl=%d gap=(%d,%d)",
		         kNativeWidth, kNativeHeight,
		         static_cast<int>(gea::platform::board::display.spiHost),
		         kSpiPclkHz / 1000000,
		         static_cast<int>(gea::platform::board::display.cs),
		         static_cast<int>(gea::platform::board::display.dc),
		         static_cast<int>(gea::platform::board::display.reset),
		         static_cast<int>(gea::platform::board::display.backlight),
		         kGapPortraitPrimaryX, kGapPortraitPrimaryY);
		return true;
	}

	// Panel-push worker. esp_lcd serializes each draw_bitmap's CASET/RASET
	// command writes against the previous chunk's queued color DMA (polling
	// transactions drain the queue), so pushing from the frame loop eats the
	// full wire time no matter how deep the transaction queue is. Instead the
	// flush paths only memcpy bands into the staging ring and hand them to
	// this task (same pattern as the P4 backend's async panel push); the wire
	// waits happen here, on the other core, under the next frame's
	// rasterization. Backpressure = the slotFree_ counting semaphore: when the
	// renderer outruns the panel it blocks on a free band, capping the frame
	// rate at the wire rate instead of adding to it.
	static void panelPushTaskEntry(void *arg)
	{
		auto *self = static_cast<DisplayBackend *>(arg);
		for (;;) {
			xSemaphoreTake(self->slotReady_, portMAX_DELAY);
			const int slot = self->pushConsumeIdx_;
			self->pushConsumeIdx_ = (slot + 1) % self->stagingDepth_;
			const PushSlot &job = self->pushSlots_[slot];
			const std::int64_t txStartUs = esp_timer_get_time();
			const esp_err_t err = esp_lcd_panel_draw_bitmap(
				self->panel_, job.x0, job.y0, job.x1, job.y1, self->stagingPixels_[slot]);
			if (err == ESP_OK) {
				if (self->flushDoneSem_) xSemaphoreTake(self->flushDoneSem_, portMAX_DELAY);
			} else {
				ESP_LOGE(kTag, "push draw_bitmap failed: %s", esp_err_to_name(err));
			}
			self->flushStats_.txUs += esp_timer_get_time() - txStartUs;
			xSemaphoreGive(self->slotFree_);
		}
	}

	// Grab a free staging band (blocking on the panel when the renderer is
	// ahead), fill it via `fill`, record the panel-coords rect, and queue it
	// for the push task. draw_bitmap end coordinates are exclusive.
	template <typename Fill>
	void submitBand(int panelX0, int panelY0, int panelX1, int panelY1, int pixels, Fill fill)
	{
		xSemaphoreTake(slotFree_, portMAX_DELAY);
		const std::int64_t copyStartUs = esp_timer_get_time();
		fill(stagingPixels_[stagingSlot_]);
		flushStats_.byteSwapUs += esp_timer_get_time() - copyStartUs;
		pushSlots_[stagingSlot_] = PushSlot{panelX0, panelY0, panelX1, panelY1};
		stagingSlot_ = (stagingSlot_ + 1) % stagingDepth_;
		flushStats_.chunkCount++;
		flushStats_.pixelCount += pixels;
		xSemaphoreGive(slotReady_);
	}

	// Block until every queued band is on the glass. Needed before panel-level
	// commands (MADCTL/gap changes): esp_lcd panel handles are not
	// thread-safe against a concurrent draw_bitmap from the push task.
	void waitPanelPushIdle()
	{
		if (!slotFree_) return;
		while (uxSemaphoreGetCount(slotFree_) < static_cast<UBaseType_t>(stagingDepth_)) {
			vTaskDelay(1);
		}
	}

	bool flushNativeLogicalRect(present::Rect logicalRect)
	{
		logicalRect = clampLogicalRect(logicalRect, logicalWidth(), logicalHeight());
		if (!present::valid(logicalRect)) return true;
		if (flushRotation_ != kFlushRotationNone) return flushRotatedLogicalRect(logicalRect);

		take();
		bindCanvasLocked();
		if (!panel_ || !frameBuffer_ || !stagingPixels_[0]) {
			give();
			return false;
		}

		// The logical frame maps 1:1 to the panel, so flush full-width bands:
		// that keeps each band's framebuffer rows contiguous and lets one packed
		// staging band feed the SPI DMA. The framebuffer is stored panel-endian
		// (GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=1), so staging is a straight memcpy;
		// the push task pays the wire time.
		const int width = logicalWidth();
		const int y0 = std::max(0, logicalRect.y0);
		const int y1 = std::min(logicalHeight() - 1, logicalRect.y1);
		const int maxRows = std::max(1, std::min(flushRows_, kFlushRowsDefault));

		for (int row = y0; row <= y1; row += maxRows) {
			int rows = maxRows;
			if (row + rows > y1 + 1) rows = y1 - row + 1;
			const std::size_t bandPixels = static_cast<std::size_t>(width) * rows;
			const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_;
			submitBand(0, row, width, row + rows, static_cast<int>(bandPixels), [&](std::uint16_t *dst) {
				std::memcpy(dst, src, bandPixels * sizeof(std::uint16_t));
			});
		}
		give();
		return true;
	}

	// Landscape flush onto the portrait-addressed panel: rotate each band into
	// the panel's scan order so the GRAM write front advances WITH the gate
	// scan. A logical rect {x0..x1, y0..y1} maps to the panel rect
	// {px: y0..y1, py: panelH-1-x1 .. panelH-1-x0} (LandscapePrimary) or
	// {px: panelW-1-y1 .. panelW-1-y0, py: x0..x1} (LandscapeSecondary).
	bool flushRotatedLogicalRect(const present::Rect &logicalRect)
	{
		take();
		bindCanvasLocked();
		if (!panel_ || !frameBuffer_ || !stagingPixels_[0]) {
			give();
			return false;
		}

		const bool primary = flushRotation_ == kFlushRotationLandscapePrimary;
		const int panelH = kNativeHeight;  // 240 physical scan lines
		const int panelW = kNativeWidth;   // 135 pixels per scan line
		const int x0 = logicalRect.x0, x1 = logicalRect.x1;
		const int y0 = logicalRect.y0, y1 = logicalRect.y1;

		const int px0 = primary ? y0 : panelW - 1 - y1;
		const int px1 = primary ? y1 : panelW - 1 - y0;
		const int py0 = primary ? panelH - 1 - x1 : x0;
		const int py1 = primary ? panelH - 1 - x0 : x1;
		const int bandW = px1 - px0 + 1;
		const int maxRows = std::max(1, std::min(flushRows_, kFlushRowsDefault));

		for (int row = py0; row <= py1; row += maxRows) {
			int rows = maxRows;
			if (row + rows > py1 + 1) rows = py1 - row + 1;
			submitBand(px0, row, px1 + 1, row + rows, bandW * rows, [&](std::uint16_t *staging) {
				// Panel-endian framebuffer: the rotate is a plain strided copy.
				for (int ly = y0; ly <= y1; ++ly) {
					const std::uint16_t *srcRow = frameBuffer_ + static_cast<std::size_t>(ly) * logicalStride_;
					const int cx = primary ? ly - y0 : y1 - ly;  // column inside the staging band
					std::uint16_t *dst = staging + cx;
					for (int r = row; r < row + rows; ++r) {
						const int lx = primary ? panelH - 1 - r : r;
						dst[static_cast<std::size_t>(r - row) * bandW] = srcRow[lx];
					}
				}
			});
		}
		give();
		return true;
	}

	void rasterPresentRegion(const present::Frame &frame, const present::Rect &region)
	{
		if (!present::valid(region)) return;
		const int width = region.x1 - region.x0 + 1;
		const int height = logicalHeight();
		const int maxRows = std::max(1, kFlushRowsDefault);
		for (int row = region.y0; row <= region.y1; row += maxRows) {
			int rows = maxRows;
			if (row + rows > region.y1 + 1) rows = region.y1 - row + 1;
			std::uint16_t *rasterTarget = frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_ + region.x0;
			const std::int64_t rasterStartUs = esp_timer_get_time();
			present::rasterFrameRowsStrided(rasterTarget,
			                                logicalStride_,
			                                region.x0,
			                                width,
			                                row,
			                                rows,
			                                height,
			                                frame);
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
	SemaphoreHandle_t flushDoneSem_ = nullptr;
	std::uint16_t *frameBuffer_ = nullptr;
	std::uint16_t *ownedFrameBuffer_ = nullptr;
	std::uint16_t *stagingPixels_[kStagingDepth] = {};
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	gea::framework::graphics::Canvas workerCanvas_;
	int logicalStride_ = kMaxLogicalWidth;
	int renderCoreId_ = -1;
	int flushRows_ = kFlushRowsDefault;
	// Staging ring producer/consumer state (see panelPushTaskEntry/submitBand).
	struct PushSlot {
		int x0, y0, x1, y1;  // panel coords; x1/y1 exclusive (draw_bitmap API)
	};
	PushSlot pushSlots_[kStagingDepth] = {};
	SemaphoreHandle_t slotFree_ = nullptr;
	SemaphoreHandle_t slotReady_ = nullptr;
	TaskHandle_t pushTask_ = nullptr;
	int stagingSlot_ = 0;
	int pushConsumeIdx_ = 0;
	// Number of staging bands actually allocated (<= kStagingDepth); set in
	// initPanel, which degrades the ring under internal-RAM pressure.
	int stagingDepth_ = kStagingDepth;
	// Landscape-on-portrait-panel flush rotation (see applyPanelOrientation).
	int flushRotation_ = kFlushRotationNone;
	int flushDepth_ = kStagingDepth;
	int brightness_ = 100;
	bool brightnessInitialized_ = false;
	bool initialized_ = false;
	bool backgroundCacheAttempted_ = false;
	bool backdropCacheAttempted_ = false;
	platform_display::DisplayFlushPerfStats flushStats_{};
	present::Frame previousPresentFrame_{};
	bool previousPresentValid_ = false;
	bool needsFullPhysicalFlush_ = true;
};

}  // namespace gea::platform::esp32_s3_sticks3::display

extern "C" std::uint16_t *gea_bg_cache(int *cap_px)
{
	return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().backgroundCache(cap_px);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().backdropCache(cap_px);
}

namespace {

struct RenderWorker {
	TaskHandle_t task = nullptr;
	void (*volatile fn)(void *, int, int) = nullptr;
	void *volatile ctx = nullptr;
	volatile int y0 = 0;
	volatile int y1 = 0;
	std::atomic<std::uint32_t> jobSeq{0};
	std::atomic<bool> done{true};
	bool attempted = false;
};

RenderWorker gRenderWorker;
volatile std::int64_t gRenderWorkerSubmitUs = 0;
std::int64_t gRenderWorkerStartLatencyUs = 0;
std::int64_t gRenderWorkerBandUs = 0;
std::int64_t gRenderWorkerWaitUs = 0;
int gRenderWorkerJobs = 0;
constexpr std::uint32_t kRenderWorkerStackBytes = 12288;

void renderWorkerTask(void *)
{
	// MUST start at 0 (jobSeq's initial value), NOT jobSeq.load(): submit()
	// creates this task and increments jobSeq immediately after, so loading
	// here usually reads 1 and swallows job 1. Same race fixed on the elecrow
	// in d22bcf5f9.
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
		const std::int64_t started = esp_timer_get_time();
		gRenderWorkerStartLatencyUs += started - gRenderWorkerSubmitUs;
		void (*fn)(void *, int, int) = gRenderWorker.fn;
		if (fn) fn(gRenderWorker.ctx, gRenderWorker.y0, gRenderWorker.y1);
		gRenderWorkerBandUs += esp_timer_get_time() - started;
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
		std::printf("[render-worker] task=%p create=%d renderCore=%d workerCore=%d prio=%u stack=%u\n",
		            static_cast<void *>(gRenderWorker.task),
		            static_cast<int>(created),
		            static_cast<int>(renderCore),
		            static_cast<int>(workerCore),
		            static_cast<unsigned>(priority),
		            static_cast<unsigned>(kRenderWorkerStackBytes));
		if (created != pdPASS || !gRenderWorker.task) {
			multi_heap_info_t info{};
			heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
			std::printf("[render-worker] create failed: create=%d stack=%u internal_free=%u internal_largest=%u\n",
			            static_cast<int>(created),
			            static_cast<unsigned>(kRenderWorkerStackBytes),
			            static_cast<unsigned>(info.total_free_bytes),
			            static_cast<unsigned>(info.largest_free_block));
		}
	}
	if (!gRenderWorker.task) return false;
	gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().setRenderCore(static_cast<int>(xPortGetCoreID()));
	gRenderWorker.fn = fn;
	gRenderWorker.ctx = ctx;
	gRenderWorker.y0 = y0;
	gRenderWorker.y1 = y1;
	gRenderWorker.done.store(false, std::memory_order_release);
	gRenderWorkerSubmitUs = esp_timer_get_time();
	gRenderWorker.jobSeq.fetch_add(1, std::memory_order_release);
	xTaskNotifyGive(gRenderWorker.task);
	return true;
}

extern "C" void gea_render_parallel_wait()
{
	if (!gRenderWorker.task) return;
	const std::int64_t waitStarted = esp_timer_get_time();
	const std::int64_t deadline = waitStarted + 100000;
	while (!gRenderWorker.done.load(std::memory_order_acquire)) {
		if (esp_timer_get_time() > deadline) {
			std::printf("[render-worker] WAIT TIMEOUT\n");
			break;
		}
	}
	gRenderWorkerWaitUs += esp_timer_get_time() - waitStarted;
	if (++gRenderWorkerJobs >= 300) {
		std::printf("[worker] startLat=%dus band=%dus wait=%dus (avg/300)\n",
		            static_cast<int>(gRenderWorkerStartLatencyUs / 300),
		            static_cast<int>(gRenderWorkerBandUs / 300),
		            static_cast<int>(gRenderWorkerWaitUs / 300));
		gRenderWorkerStartLatencyUs = 0;
		gRenderWorkerBandUs = 0;
		gRenderWorkerWaitUs = 0;
		gRenderWorkerJobs = 0;
	}
}

extern "C" void gea_render_parallel_merge_dirty()
{
	gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().absorbWorkerDirty();
}

extern "C" int gea_current_render_core()
{
	return static_cast<int>(xPortGetCoreID());
}

namespace gea::platform::display {

namespace {

gea::framework::graphics::Canvas *drawingCanvas()
{
	return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().drawingCanvas();
}

}  // namespace

bool Display::init() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().init(); }
bool Display::start() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().start(); }
gea::framework::graphics::Canvas *Display::canvas() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().canvas(); }
bool Display::framebufferIsPanelDirect() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().framebufferIsPanelDirect(); }
void Display::clear() { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().clear(); }
void Display::clearNoFlush() { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flush(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushRects(rects, count); }
// Fused rasterized flush is not implemented on this backend: raster into the
// framebuffer region band-by-band, then push those bands (functionally
// equivalent, keeps the framebuffer coherent).
void Display::flushRectsRasterized(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, bool)
{
	if (!rects || count <= 0 || !raster) return;
	for (int i = 0; i < count; ++i) {
		const auto &r = rects[i];
		gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().streamRect(
			r.x0, r.y0, r.x1 - r.x0 + 1, r.y1 - r.y0 + 1, raster, user);
	}
}
void Display::rebindCanvasToFramebuffer() { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().rebindCanvasToFramebuffer(); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int command_count) { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().presentCommands(commands, command_count); }
void Display::setFlushConfig(int chunk_rows, int queue_depth) { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().setFlushConfig(chunk_rows, queue_depth); }
void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushChunkRows(); }
int Display::flushQueueDepth() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushQueueDepth(); }
int Display::flushBufferBytes() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushBufferBytes(); }
void Display::pushClip(int x, int y, int w, int h) { if (auto *c = drawingCanvas()) c->pushClip(x, y, w, h); }
void Display::popClip() { if (auto *c = drawingCanvas()) c->popClip(); }
void Display::resetClip() { if (auto *c = drawingCanvas()) c->resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { if (auto *c = drawingCanvas()) c->setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return drawingCanvas() ? drawingCanvas()->globalAlpha() : 255; }
int Display::brightness() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().brightness(); }
void Display::setBrightness(int brightness_percent) { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().setBrightness(brightness_percent); }
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
	const auto stats = gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStats();
	if (total_us) *total_us = stats.totalUs;
	if (call_count) *call_count = stats.callCount;
	if (pixel_count) *pixel_count = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStats(); }
void Display::flushStatsReset() { gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStatsReset(); }
const char *Display::flushStageName() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStageName(); }
int Display::flushStageChunk() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStageChunk(); }
DisplayFlushStageDetail Display::flushStageDetail() { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().flushStageDetail(); }
bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixel_capacity, int *width, int *height, bool) { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().copySnapshotRgb565(dst, pixel_capacity, width, height); }
int Display::countNonBlackPixels(bool) { return gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().countNonBlackPixels(); }

void applyOrientation(gea::framework::display::DisplayOrientation)
{
	gea::platform::esp32_s3_sticks3::display::DisplayBackend::instance().applyOrientation();
}

}  // namespace gea::platform::display
