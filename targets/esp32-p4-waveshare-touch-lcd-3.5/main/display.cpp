#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "panel_native.h"
#include "host/display_orientation.h"
#include "pixel.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <utility>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_periph.h"

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;
namespace present = gea::framework::display_present;

namespace gea::platform::esp32_p4_waveshare::display {

namespace {

constexpr const char *kTag = "waveshare_p4_display";
constexpr int kNativeWidth = platform_display::kNativeWidth;
constexpr int kNativeHeight = platform_display::kNativeHeight;
constexpr int kMaxLogicalWidth = kNativeHeight > kNativeWidth ? kNativeHeight : kNativeWidth;
constexpr int kMaxLogicalHeight = kNativeHeight;
constexpr int kMaxPixels = kNativeWidth * kNativeHeight;
constexpr int kFlushRowsDefault = 64;
// ST7796 SPI panel (Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5): 320x480, SPI mode 3,
// 8-bit command/parameter, 80 MHz pixel clock, BGR element order, color-inverted,
// X-mirrored. Flushed via esp_lcd_panel_draw_bitmap. The panel latches RGB565
// MSB-first, so each band is byte-swapped into an internal-RAM staging buffer
// before the SPI DMA (the PSRAM framebuffer stays native little-endian RGB565,
// which keeps the camera's PPA direct-present path correct).
constexpr int kSpiPclkHz = 80 * 1000 * 1000;
constexpr int kPanelBitsPerPixel = 16;
constexpr int kSpiTransQueueDepth = 8;
// Flush rotation modes: landscape logical frames are rotated into the panel's
// portrait scan order at flush time instead of via MADCTL (which would make
// GRAM writes perpendicular to the gate scan and shear/tear on motion).
constexpr int kFlushRotationNone = 0;
constexpr int kFlushRotationLandscapePrimary = 1;
constexpr int kFlushRotationLandscapeSecondary = 2;
// Internal-RAM bounce band for CPU-composed flushes — sized for the widest
// band across both orientations (landscape is 480 wide).
constexpr int kStagingBytes = kMaxLogicalWidth * kFlushRowsDefault * static_cast<int>(sizeof(std::uint16_t));
// The SPI bus itself takes up to a full panel frame in one transaction: the
// camera NativeOverlay pushes its PPA-rendered, pre-byteswapped PSRAM buffer
// as a single DMA (like the stock Waveshare demo's max_transfer_sz).
constexpr int kSpiMaxTransferBytes = kMaxPixels * static_cast<int>(sizeof(std::uint16_t));
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_1;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_1;
constexpr int kBacklightLedcMaxDuty = 1023;
constexpr int kMaxPresentRects = 16;
constexpr int kOwnedFrameBufferPixels = kMaxPixels;
// ST7796 register reads (GSCAN scanline) need a slow clock — the panel's read
// cycle is ~150 ns vs the 12.5 ns write cycle, so a dedicated low-speed IO on
// the same bus handles them. 2 MHz is comfortably inside read timing.
constexpr int kSpiReadClkHz = 2 * 1000 * 1000;
constexpr std::uint8_t kSt7796CmdGetScanline = 0x45;

// Waveshare's vendor init for this glass (ported from the board's stock
// demo BSP, esp32_p4_wifi6_touch_lcd_35.c): command-2 unlock, inversion
// mode, power/VCOM, display-output timing adjust (0xE8), gamma, lock.
// Without it the panel runs on ST7796 power-on defaults, which differ from
// the stock firmware's drive timing. SLPOUT/COLMOD/MADCTL/INVON/DISPON are
// issued by the esp_lcd_st7796 driver itself and are not repeated here.
constexpr std::uint8_t kInitCmd2Unlock1[] = {0xC3};
constexpr std::uint8_t kInitCmd2Unlock2[] = {0x96};
// Panel scan timing — LOAD-BEARING for camera cadence AND tear sync. All
// numbers GSCAN-measured on this glass; the datasheet frame-rate formula
// does NOT match (measured: RTNA=16 → 17.095 ms scan, RTNA=15 → 17.064 ms
// (~31 us/step near here), RTNA=7 → 14.286 ms (non-linear cliff), FRS=9 →
// 18.671 ms, all at VFP/VBP=2/2 → 484 lines, ~35.3 us/line).
//
// Goal: the vsync-gated camera present must hit an INTEGER number of scans
// per sensor frame or motion judders ("two frames forward, one back").
// The AE paces the sensor at 50.0 ms/frame in normal light (the 1280x960
// mode is nominally 45 fps; the IPA stretches it, so sensor-side VTS
// alignment would fight the AE). Panel-side, porch lines give fine-grained
// control — and porch lines are SHORTER than active lines on this glass
// (two-point GSCAN calibration: VFP+VBP=4 → 17.095 ms; VFP+VBP=228 →
// 22.741 ms → ~25.2 us per porch line vs ~35.3 us per active line). Stretch
// to VFP=VBP=159 (318 porch lines) → scan ≈ 25.0 ms (40 Hz), TWO scans ≈
// 50.02 ms ≈ one sensor frame; the ~20-30 us residual = a single-scan
// cadence slip only every ~1 min. The ~31 ms push still fits two scan
// periods with wide margin (tear-free needs T_s ≥ ~15.8 ms). RTNA stays at
// the default 16; its ~31 us micro-steps remain available for future trim.
// RTNA=8 → 16.700 ms scan (GSCAN-measured): THREE scans = 50.10 ms ≈ the
// AE's current 50.0 ms sensor period → the vsync-gated camera present locks
// 1:1 with at most one single-scan cadence slip every ~8 s, and the ~31 ms
// full-frame push still fits two scans (33.4 ms) for tear-free geometry.
// This is the best static point on this glass: RTNA steps ~31-41 µs down to
// here (16→17.095, 15→17.064, 14→17.013) and CLIFF to 14.286 ms at RTNA=7
// (which tears); large porches misbehave outright (VFP/VBP=159 → 13.8 ms).
// Lighting changes move the AE's frame period — the static alignment holds for
// ~50 ms-class indoor light; the adaptive runtime controller is the open item
// for full robustness. Re-measure with measureScanTiming() on the glass: the
// ST7796S datasheet frame-rate formula does not match this panel.
constexpr std::uint8_t kInitFrameRate[] = {0xA0, 0x08};
constexpr std::uint8_t kInitPorch[] = {0x02, 0x02, 0x00, 0x04};
constexpr std::uint8_t kInitInversion[] = {0x01};
constexpr std::uint8_t kInitEntryMode[] = {0xC6};
constexpr std::uint8_t kInitPower1[] = {0x80, 0x45};
constexpr std::uint8_t kInitPower2[] = {0x13};
constexpr std::uint8_t kInitPower3[] = {0xA7};
constexpr std::uint8_t kInitVcom[] = {0x0A};
constexpr std::uint8_t kInitDoca[] = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33};
constexpr std::uint8_t kInitGammaP[] = {0xD0, 0x08, 0x0F, 0x06, 0x06, 0x33, 0x30, 0x33,
                                        0x47, 0x17, 0x13, 0x13, 0x2B, 0x31};
constexpr std::uint8_t kInitGammaN[] = {0xD0, 0x0A, 0x11, 0x0B, 0x09, 0x07, 0x2F, 0x33,
                                        0x47, 0x38, 0x15, 0x16, 0x2C, 0x32};
constexpr std::uint8_t kInitCmd2Lock1[] = {0x3C};
constexpr std::uint8_t kInitCmd2Lock2[] = {0x69};
constexpr st7796_lcd_init_cmd_t kVendorInitCmds[] = {
	{0xF0, kInitCmd2Unlock1, sizeof(kInitCmd2Unlock1), 0},
	{0xF0, kInitCmd2Unlock2, sizeof(kInitCmd2Unlock2), 0},
	{0xB1, kInitFrameRate, sizeof(kInitFrameRate), 0},
	{0xB5, kInitPorch, sizeof(kInitPorch), 0},
	{0xB4, kInitInversion, sizeof(kInitInversion), 0},
	{0xB7, kInitEntryMode, sizeof(kInitEntryMode), 0},
	{0xC0, kInitPower1, sizeof(kInitPower1), 0},
	{0xC1, kInitPower2, sizeof(kInitPower2), 0},
	{0xC2, kInitPower3, sizeof(kInitPower3), 0},
	{0xC5, kInitVcom, sizeof(kInitVcom), 0},
	{0xE8, kInitDoca, sizeof(kInitDoca), 0},
	{0xE0, kInitGammaP, sizeof(kInitGammaP), 0},
	{0xE1, kInitGammaN, sizeof(kInitGammaN), 0},
	{0xF0, kInitCmd2Lock1, sizeof(kInitCmd2Lock1), 0},
	{0xF0, kInitCmd2Lock2, sizeof(kInitCmd2Lock2), 120},
};

present::Rect clampLogicalRect(present::Rect rect, int width, int height)
{
	if (width <= 0 || height <= 0) return {};
	rect = present::clampAndAlign(rect, width, height);
	if (!present::valid(rect)) return {};
	return rect;
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
		if (!frameBuffer_) init();
		return &replayCanvas();
	}

	// Public accessor: is canvas() the panel scanout buffer (no logical->panel
	// rotation)? Used by the camera direct-present to skip the rotate-flush.
	bool framebufferIsPanelDirect() const { return usingPanelFrameBufferDirectly(); }

	gea::framework::graphics::Canvas *drawingCanvas()
	{
		if (!frameBuffer_) init();
		return &replayCanvas();
	}

	void applyOrientation()
	{
		take();
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
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		previousPresentValid_ = false;
		needsFullPhysicalFlush_ = true;
		frameBufferCoherent_ = true;
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
		frameBufferCoherent_ = true;
		flush();
		return true;
	}

	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int commandCount)
	{
		if (!commands || commandCount <= 0 || !init()) return false;
		present::Frame current;
		if (!present::extractFrame(commands, commandCount, current)) return false;
		if (!present::frameHasOpaqueBase(current, logicalWidth(), logicalHeight())) return false;
		if (!isFastClearFillCirclesFrame(current) && !frameBufferCoherent_) {
			previousPresentValid_ = false;
			needsFullPhysicalFlush_ = true;
		}
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
			if (!ok) {
				ok = false;
				break;
			}
		}
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

	// A frame whose base layer is [Clear, FillCircles]. Commands AFTER the circle
	// batch (FPS text, a HUD rect, ...) are overlays drawn on top — they ride the
	// incremental path via a per-frame overlay re-raster, so they no longer
	// disqualify it. (Previously this required EXACTLY two commands, which dropped
	// any circle frame carrying an FPS counter onto the slow full-screen path.)
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

	// The incremental circle path erases + redraws every ball (2x scattered writes
	// over the whole framebuffer — cache-thrashing) to save the cache writeback on
	// the untouched area. That trade only pays when the touched area is small; for a
	// dense/full-screen ball field the general chunked rasterizer is faster (measured
	// 21.6 vs 18.2fps for 1000 balls). So gate the incremental path on the would-be
	// dirty area, mirroring the amoled's geometric band-collapse decision.
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

		// Overlays = every command past the circle batch (e.g. the FPS text). They
		// sit on top of the balls and don't move with them, so their box — current
		// AND previous (to erase the stale overlay) — is repainted independently and
		// re-composited from scratch. Empty when the frame is pure [Clear, FillCircles].
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
		// Composite overlays: re-raster their box from the full frame (clear + the
		// balls behind them + the overlay itself), so the old overlay is erased and
		// the new one lands on top of whatever balls sit behind it.
		if (present::valid(overlayDirty)) rasterPresentRegion(current, overlayDirty);
		flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;

		const bool ok = flushNativeLogicalRect(dirty);
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
				const int spanIndex = dy + command.spans.origin;
				const int halfWidth =
					spanIndex >= 0 && spanIndex < static_cast<int>(command.spans.halfWidths.size())
						? command.spans.halfWidths[static_cast<std::size_t>(spanIndex)]
						: present::integerSqrt(r * r - dy * dy);
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
		std::uint16_t *targetFrameBuffer = preferredFrameBuffer();
		if (!targetFrameBuffer) return;
		if (frameBuffer_ != targetFrameBuffer) frameBuffer_ = targetFrameBuffer;
		if (canvas_.pixels() == frameBuffer_ && canvas_.width() == width && canvas_.height() == height && logicalStride_ == stride) return;
		logicalStride_ = stride;
		canvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
		workerCanvas_.bindPixels(frameBuffer_, width, height, logicalStride_);
	}

	std::uint16_t *preferredFrameBuffer() const
	{
		// ST7796 SPI is a push panel: there is no scanout framebuffer the canvas can
		// draw into directly, so we always render into our owned PSRAM buffer and
		// flush dirty bands to the panel.
		return ownedFrameBuffer_;
	}

	// A push panel never exposes its own scanout buffer, so writes always require an
	// explicit flush. The camera direct-present path checks this to decide whether it
	// can scale straight into the canvas.
	bool usingPanelFrameBufferDirectly() const { return false; }


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

	// Rotate the panel itself (ST7796 MADCTL swap_xy + mirror) to match the app's
	// requested orientation, so the logical frame maps 1:1 onto the panel with no
	// software rotation. Portrait-primary matches the vendor BSP (known good); the
	// landscape/secondary mirror flags are verified on-device.
	// The ST7796's gate driver always scans the physical (portrait) lines —
	// MADCTL only remaps GRAM *addressing*, not the refresh scan. Writing
	// landscape full-width bands through a swapped MADCTL therefore advances
	// PERPENDICULAR to the panel's scan, which shears/tears on any motion no
	// matter how fast the push is. (The stock Waveshare camera demo is tear-free
	// precisely because it pushes portrait rows in scan order with the very same
	// SPI config.) So landscape orientations keep the panel in PORTRAIT
	// addressing and the flush path rotates each band into scan order instead
	// (see flushRotatedLogicalRect); only portrait orientations use MADCTL.
	void applyPanelOrientation()
	{
		if (!panel_) return;
		using gea::framework::display::DisplayOrientation;
		bool swapXy = false;
		bool mirrorX = true;
		bool mirrorY = false;
		flushRotation_ = kFlushRotationNone;
		switch (orientation_detail::DisplayOrientationState::orientation()) {
		case DisplayOrientation::LandscapePrimary:
			// Portrait-primary MADCTL + rotate-on-flush. Mapping derived from the
			// (verified) landscape MADCTL swap+mirror(true,true) this replaces:
			// panel x = logical y, panel y = (panelH-1) - logical x.
			flushRotation_ = kFlushRotationLandscapePrimary;
			break;
		case DisplayOrientation::LandscapeSecondary:
			// panel x = (panelW-1) - logical y, panel y = logical x.
			flushRotation_ = kFlushRotationLandscapeSecondary;
			break;
		case DisplayOrientation::PortraitSecondary:
			swapXy = false; mirrorX = false; mirrorY = true;
			break;
		case DisplayOrientation::PortraitPrimary:
		default:
			swapXy = false; mirrorX = true;  mirrorY = false;
			break;
		}
		esp_lcd_panel_swap_xy(panel_, swapXy);
		esp_lcd_panel_mirror(panel_, mirrorX, mirrorY);
	}

	bool initPanel()
	{
		if (panel_) return true;
		if (!initBacklight()) return false;

		flushDoneSem_ = xSemaphoreCreateBinary();
		if (!flushDoneSem_) {
			ESP_LOGE(kTag, "failed to allocate flush-done semaphore");
			return false;
		}

		// One internal-RAM staging band. Each flush copies a full-width band of the
		// PSRAM framebuffer here, byte-swapped to the panel's MSB-first RGB565, then
		// DMAs it over SPI. Internal RAM keeps the SPI GDMA on the fast path.
		stagingPixels_ = static_cast<std::uint16_t *>(
			heap_caps_malloc(static_cast<std::size_t>(kStagingBytes), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		if (!stagingPixels_) {
			ESP_LOGE(kTag, "display staging band alloc (%d bytes) failed", kStagingBytes);
			return false;
		}

		spi_bus_config_t busConfig = {};
		busConfig.mosi_io_num = gea::platform::board::display.mosi;
		// LCD_MISO (panel SDO) — scanline reads for tear-sync; see readScanline().
		busConfig.miso_io_num = gea::platform::board::display.miso;
		busConfig.sclk_io_num = gea::platform::board::display.sclk;
		busConfig.quadwp_io_num = GPIO_NUM_NC;
		busConfig.quadhd_io_num = GPIO_NUM_NC;
		busConfig.max_transfer_sz = kSpiMaxTransferBytes;
		esp_err_t err = spi_bus_initialize(gea::platform::board::display.spiHost, &busConfig, SPI_DMA_CH_AUTO);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_lcd_panel_io_spi_config_t ioConfig = {};
		ioConfig.cs_gpio_num = gea::platform::board::display.cs;
		ioConfig.dc_gpio_num = gea::platform::board::display.dc;
		ioConfig.spi_mode = 3;
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

		// Second, slow IO on the same bus for ST7796 register reads (GSCAN tear
		// sync). cs_gpio_num = -1: the CS pin belongs to the main IO's hardware CS
		// slot; readScanline() temporarily drives it as a plain GPIO around each
		// read transaction (the display mutex serializes all panel access).
		esp_lcd_panel_io_spi_config_t readIoConfig = {};
		readIoConfig.cs_gpio_num = GPIO_NUM_NC;
		readIoConfig.dc_gpio_num = gea::platform::board::display.dc;
		readIoConfig.spi_mode = 3;
		readIoConfig.pclk_hz = kSpiReadClkHz;
		readIoConfig.trans_queue_depth = 1;
		readIoConfig.lcd_cmd_bits = 8;
		readIoConfig.lcd_param_bits = 8;
		err = esp_lcd_new_panel_io_spi(
			static_cast<esp_lcd_spi_bus_handle_t>(gea::platform::board::display.spiHost), &readIoConfig, &ioRead_);
		if (err != ESP_OK) {
			ESP_LOGW(kTag, "scanline read IO init failed (%s); tear sync disabled", esp_err_to_name(err));
			ioRead_ = nullptr;
		}

		esp_lcd_panel_dev_config_t panelConfig = {};
		panelConfig.reset_gpio_num = gea::platform::board::display.reset;
		panelConfig.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
		panelConfig.bits_per_pixel = kPanelBitsPerPixel;
		st7796_vendor_config_t vendorConfig = {};
		vendorConfig.init_cmds = kVendorInitCmds;
		vendorConfig.init_cmds_size = sizeof(kVendorInitCmds) / sizeof(kVendorInitCmds[0]);
		panelConfig.vendor_config = &vendorConfig;
		err = esp_lcd_new_panel_st7796(io_, &panelConfig, &panel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "ST7796 panel create failed: %s", esp_err_to_name(err));
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
		// Waveshare panel ships color-inverted; match the vendor BSP. Orientation
		// (swap_xy + mirror) is applied separately so the panel rotates with the app's
		// requested orientation — already set if the app declared landscape before init.
		esp_lcd_panel_invert_color(panel_, true);
		applyPanelOrientation();
		err = esp_lcd_panel_disp_on_off(panel_, true);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "panel on failed: %s", esp_err_to_name(err));
			return false;
		}
		// Learn the panel's scan period for tear-synced camera pushes.
		measureScanTiming();
		ESP_LOGI(kTag,
		         "ST7796 SPI panel ready %dx%d host=%d pclk=%dMHz mode=3 cs=%d dc=%d rst=%d bl=%d",
		         kNativeWidth, kNativeHeight,
		         static_cast<int>(gea::platform::board::display.spiHost),
		         kSpiPclkHz / 1000000,
		         static_cast<int>(gea::platform::board::display.cs),
		         static_cast<int>(gea::platform::board::display.dc),
		         static_cast<int>(gea::platform::board::display.reset),
		         static_cast<int>(gea::platform::board::display.backlight));
		return true;
	}

	bool flushNativeLogicalRect(present::Rect logicalRect)
	{
		logicalRect = clampLogicalRect(logicalRect, logicalWidth(), logicalHeight());
		if (!present::valid(logicalRect)) return true;
		if (flushRotation_ != kFlushRotationNone) return flushRotatedLogicalRect(logicalRect);

		take();
		bindCanvasLocked();
		if (!panel_ || !frameBuffer_ || !stagingPixels_) {
			give();
			return false;
		}

		// The logical frame maps 1:1 to the panel (the panel MADCTL is rotated to the
		// current orientation), so flush full-width bands: that keeps each band's
		// framebuffer rows contiguous (logicalStride_ == logicalWidth()) and lets one
		// packed staging buffer feed the SPI DMA. Each band is byte-swapped to the
		// panel's MSB-first RGB565.
		const int width = logicalWidth();
		const int y0 = std::max(0, logicalRect.y0);
		const int y1 = std::min(logicalHeight() - 1, logicalRect.y1);
		const int maxRows = std::max(1, std::min(flushRows_, kFlushRowsDefault));

		bool ok = true;
		for (int row = y0; row <= y1 && ok; row += maxRows) {
			int rows = maxRows;
			if (row + rows > y1 + 1) rows = y1 - row + 1;
			const std::size_t bandPixels = static_cast<std::size_t>(width) * rows;

			const std::int64_t swapStartUs = esp_timer_get_time();
			const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_;
			for (std::size_t i = 0; i < bandPixels; ++i) stagingPixels_[i] = __builtin_bswap16(src[i]);
			// Write the band back to PSRAM so physical memory matches what is being
			// pushed. CPU-drawn UI pixels otherwise live only in the write-back cache —
			// and the camera's NativeOverlay presentDirect() INVALIDATES (M2C) its
			// full-width framebuffer rows every frame, discarding those dirty lines.
			// The camera's own full-width band flush then re-pushes the panel columns
			// from stale PSRAM, visibly reverting any small UI update (status text,
			// button labels) ~one camera frame after it appeared. Camera-column lines
			// are clean here (the PPA writes PSRAM directly), so this writeback only
			// persists UI-drawn pixels — it cannot clobber a live camera frame.
			// (Range is cache-line aligned: stride*2 = 960 B per row off a 128-aligned
			// base, so no UNALIGNED flag is needed.)
			esp_cache_msync(const_cast<std::uint16_t *>(src), bandPixels * sizeof(std::uint16_t),
			                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
			flushStats_.byteSwapUs += esp_timer_get_time() - swapStartUs;

			const std::int64_t txStartUs = esp_timer_get_time();
			const esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, 0, row, width, row + rows, stagingPixels_);
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "draw_bitmap failed: %s", esp_err_to_name(err));
				ok = false;
				break;
			}
			// Block until the SPI DMA completes so the single staging band can be reused.
			if (flushDoneSem_) xSemaphoreTake(flushDoneSem_, portMAX_DELAY);
			flushStats_.txUs += esp_timer_get_time() - txStartUs;
			flushStats_.chunkCount++;
			flushStats_.pixelCount += static_cast<int>(bandPixels);
		}
		give();
		return ok;
	}

public:
	// Camera NativeOverlay fast path (panel_native.h): map a logical rect into
	// panel-native portrait coordinates so the camera can PPA-render straight
	// into a panel-ordered buffer (no CPU rotate, no byteswap pass).
	bool panelMapLogicalRect(int lx, int ly, int lw, int lh,
	                         int *px, int *py, int *pw, int *ph, int *extraRotCcw)
	{
		if (flushRotation_ == kFlushRotationNone || lw <= 0 || lh <= 0) return false;
		const int x1 = lx + lw - 1;
		const int y1 = ly + lh - 1;
		if (flushRotation_ == kFlushRotationLandscapePrimary) {
			*px = ly;
			*py = kNativeHeight - 1 - x1;
			*extraRotCcw = 1;
		} else {
			*px = kNativeWidth - 1 - y1;
			*py = lx;
			*extraRotCcw = 3;
		}
		*pw = lh;
		*ph = lw;
		return true;
	}

	// Push a packed little-endian RGB565 block (e.g. PPA output) to the panel at
	// portrait coordinates as ONE continuous SPI DMA. The byteswap to the
	// ST7796's MSB-first stream happens as a single full-block pass into a
	// dedicated PSRAM buffer first (the PPA can't pre-swap — its byte_swap flag
	// scrambles colors through the downscale filter). A single uninterrupted
	// transfer matters: pushing in CPU-interleaved bands makes the GRAM write
	// front advance in stop-start jerks that the panel's scan keeps crossing —
	// visible shear — while one wire-speed burst crosses it once, invisibly
	// (verified on-device: the banded push tore, the single burst didn't).
	bool presentPanelRect(const std::uint16_t *pixels, int px, int py, int w, int h)
	{
		if (!pixels || w <= 0 || h <= 0) return false;
		if (!init()) return false;
		take();
		if (!panel_ || !ensurePanelSwapBuf()) {
			give();
			return false;
		}

		const std::size_t pixelCount = static_cast<std::size_t>(w) * h;
		const std::int64_t swapStartUs = esp_timer_get_time();
		// 4 pixels per 64-bit op: swap the bytes within each 16-bit lane.
		const std::uint64_t *src64 = reinterpret_cast<const std::uint64_t *>(pixels);
		std::uint64_t *dst64 = reinterpret_cast<std::uint64_t *>(panelSwapBuf_);
		const std::size_t quads = pixelCount / 4;
		for (std::size_t i = 0; i < quads; ++i) {
			const std::uint64_t v = src64[i];
			dst64[i] = ((v & 0xFF00FF00FF00FF00ull) >> 8) | ((v & 0x00FF00FF00FF00FFull) << 8);
		}
		for (std::size_t i = quads * 4; i < pixelCount; ++i)
			panelSwapBuf_[i] = __builtin_bswap16(pixels[i]);
		// The SPI GDMA reads physical PSRAM — write the swapped block back.
		esp_cache_msync(panelSwapBuf_,
		                (pixelCount * sizeof(std::uint16_t) + 127) & ~static_cast<std::size_t>(127),
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
		flushStats_.byteSwapUs += esp_timer_get_time() - swapStartUs;

		// Tear sync: start the GRAM write at the panel's scan top. A write that
		// starts at vsync keeps the write front between the two scan fronts for
		// the whole frame, so the scan never crosses it mid-rect — the crawling
		// shear band this panel otherwise shows on camera pushes. This path is
		// camera-exclusive (UI flushes go through the staging-band path), so
		// EVERY push is gated — an inset (non-full-screen) viewfinder pushes
		// fewer pixels but tears just the same without the gate. The wait
		// happens with the mutex RELEASED (readScanline retakes it per poll) so
		// UI flushes aren't blocked behind it; the swap buffer is camera-only,
		// so nothing can clobber it in the gap.
		give();
		waitForScanTop();
		take();
		if (!panel_) {
			give();
			return false;
		}

		const std::int64_t txStartUs = esp_timer_get_time();
		const esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, px, py, px + w, py + h, panelSwapBuf_);
		bool ok = err == ESP_OK;
		if (!ok) {
			ESP_LOGE(kTag, "presentPanelRect draw_bitmap failed: %s", esp_err_to_name(err));
		} else if (flushDoneSem_) {
			xSemaphoreTake(flushDoneSem_, portMAX_DELAY);
		}
		flushStats_.txUs += esp_timer_get_time() - txStartUs;
		flushStats_.chunkCount++;
		flushStats_.pixelCount += static_cast<int>(pixelCount);
		give();
		return ok;
	}

	// Asynchronous variant: hand the block to the panel-push worker task and
	// return; the caller may immediately start composing the NEXT frame into a
	// different buffer (the camera double-buffers), overlapping the ~30+ ms SPI
	// push with the PPA scale of the following frame. Back-pressured to one job
	// in flight: submitting waits for the previous push, so a buffer handed
	// back-to-back-alternately is never read and rewritten at the same time.
	bool presentPanelRectAsync(const std::uint16_t *pixels, int px, int py, int w, int h)
	{
		if (!ensurePanelPushTask()) return presentPanelRect(pixels, px, py, w, h);
		if (panelPushPending_) {
			xSemaphoreTake(panelPushDone_, portMAX_DELAY);
			panelPushPending_ = false;
		}
		panelPushInflight_.fetch_add(1, std::memory_order_acq_rel);
		panelPushJob_ = PanelPushJob{pixels, px, py, w, h};
		panelPushPending_ = true;
		xSemaphoreGive(panelPushStart_);
		return true;
	}

	// Block until no async panel push is queued or streaming. Used by the
	// camera when the viewfinder GEOMETRY changes: a stale-rect push that
	// lands after the renderer repaints the freed region would wipe the new
	// UI with one dead frame of old-geometry camera content. Poll-wait is
	// fine — geometry changes are rare (layout switches), pushes are ≤~50 ms.
	void waitPanelPushIdle()
	{
		while (panelPushInflight_.load(std::memory_order_acquire) != 0) vTaskDelay(pdMS_TO_TICKS(1));
	}

	std::int64_t panelScanPeriodUs() const { return scanPeriodUs_; }

	// See panel_native.h clearLogicalRect: blacks out a logical-fb rect so UI
	// flushes overlapping the camera-owned region paint black, not the stale
	// frame the panel-native present left behind there.
	void clearLogicalRect(int x, int y, int w, int h)
	{
		take();
		if (!frameBuffer_) {
			give();
			return;
		}
		const int lw = logicalWidth();
		const int lh = logicalHeight();
		const int x0 = std::max(0, x);
		const int y0 = std::max(0, y);
		const int x1 = std::min(lw, x + w);
		const int y1 = std::min(lh, y + h);
		if (x0 < x1 && y0 < y1) {
			for (int row = y0; row < y1; ++row) {
				std::memset(frameBuffer_ + static_cast<std::size_t>(row) * logicalStride_ + x0, 0,
				            static_cast<std::size_t>(x1 - x0) * sizeof(std::uint16_t));
			}
			esp_cache_msync(frameBuffer_,
			                (static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t) + 127) &
			                    ~static_cast<std::size_t>(127),
			                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
		}
		give();
	}

private:
	struct PanelPushJob {
		const std::uint16_t *pixels;
		int px, py, w, h;
	};

	static void panelPushTaskEntry(void *arg)
	{
		auto *self = static_cast<DisplayBackend *>(arg);
		for (;;) {
			xSemaphoreTake(self->panelPushStart_, portMAX_DELAY);
			const PanelPushJob job = self->panelPushJob_;
			self->presentPanelRect(job.pixels, job.px, job.py, job.w, job.h);
			self->panelPushInflight_.fetch_sub(1, std::memory_order_acq_rel);
			xSemaphoreGive(self->panelPushDone_);
		}
	}

	bool ensurePanelSwapBuf()
	{
		if (panelSwapBuf_) return true;
		const std::size_t bytes = static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t);
		panelSwapBuf_ = static_cast<std::uint16_t *>(
			heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
		if (!panelSwapBuf_) ESP_LOGE(kTag, "panel swap buffer alloc (%zu bytes) failed", bytes);
		return panelSwapBuf_ != nullptr;
	}

	bool ensurePanelPushTask()
	{
		if (panelPushTask_) return true;
		panelPushStart_ = xSemaphoreCreateBinary();
		panelPushDone_ = xSemaphoreCreateBinary();
		if (!panelPushStart_ || !panelPushDone_) return false;
		// Pin opposite the render core so the byteswap bursts overlap the frame
		// task's PPA/compose work instead of preempting it.
		const BaseType_t core = renderCoreId_ == 0 ? 1 : renderCoreId_ == 1 ? 0 : tskNO_AFFINITY;
		if (xTaskCreatePinnedToCore(panelPushTaskEntry, "panel_push", 4096, this, 5, &panelPushTask_, core) !=
		    pdPASS) {
			panelPushTask_ = nullptr;
			return false;
		}
		return true;
	}

	// Read the ST7796 GSCAN (0x45) register: the current gate line as a
	// monotonic ramp that wraps at the scan top. The read IO has no CS slot of
	// its own (one CS wire, owned by the main IO's hardware CS), so the CS pin
	// is driven as a plain GPIO around the transaction and then handed back to
	// the SPI peripheral. A dummy clock precedes the data, so the absolute
	// value is bit-shifted — callers only rely on ramp+wrap, never the line
	// number itself. Takes the display mutex internally.
	bool readScanline(int *outLine)
	{
		if (!ioRead_) return false;
		take();
		const gpio_num_t cs = gea::platform::board::display.cs;
		gpio_set_level(cs, 1);
		esp_rom_gpio_connect_out_signal(cs, SIG_GPIO_OUT_IDX, false, false);
		gpio_set_direction(cs, GPIO_MODE_OUTPUT);
		gpio_set_level(cs, 0);
		std::uint8_t raw[2] = {0, 0};
		const esp_err_t err = esp_lcd_panel_io_rx_param(ioRead_, kSt7796CmdGetScanline, raw, sizeof(raw));
		gpio_set_level(cs, 1);
		esp_rom_gpio_connect_out_signal(
			cs, spi_periph_signal[gea::platform::board::display.spiHost].spics_out[0], false, false);
		give();
		if (err != ESP_OK) return false;
		*outLine = (static_cast<int>(raw[0]) << 8) | raw[1];
		return true;
	}

	// Sample GSCAN for ~160 ms after panel init: learn the ramp's max value and
	// the scan (vsync) period. Leaves scanPeriodUs_ at 0 — tear sync disabled —
	// when the register can't be read (e.g. SDO not actually wired through).
	// Robustness: a single glitched read used to register a FALSE wrap, which
	// split one inter-wrap delta in two and skewed the mean period by a whole
	// divisor (measured 14.46 ms on a build whose true period was 16.7 ms). So
	// wraps must look like real wraps (high ramp → low ramp), and the period
	// is the MEDIAN of the inter-wrap deltas, which one bad wrap can't move.
	void measureScanTiming()
	{
		if (!ioRead_) return;
		constexpr int kMaxDeltas = 24;
		std::int64_t deltas[kMaxDeltas];
		int deltaCount = 0;
		int maxLine = 0;
		int prev = -1;
		std::int64_t lastWrapUs = 0;
		const std::int64_t until = esp_timer_get_time() + 160 * 1000;
		while (esp_timer_get_time() < until && deltaCount < kMaxDeltas) {
			int line = 0;
			if (!readScanline(&line)) {
				scanPeriodUs_ = 0;
				return;
			}
			if (line > maxLine) maxLine = line;
			const bool wrapped =
				prev >= 0 && maxLine > 16 && prev > (maxLine * 3) / 4 && line < maxLine / 4;
			if (wrapped) {
				const std::int64_t now = esp_timer_get_time();
				if (lastWrapUs != 0 && now > lastWrapUs) {
					if (deltaCount < kMaxDeltas) deltas[deltaCount++] = now - lastWrapUs;
				}
				lastWrapUs = now;
			}
			prev = line;
		}
		if (deltaCount >= 3 && maxLine > 16) {
			std::sort(deltas, deltas + deltaCount);
			scanPeriodUs_ = deltas[deltaCount / 2];
			scanlineMax_ = maxLine;
			ESP_LOGI(kTag, "panel scan: period=%lldus (%.1f Hz, median/%d) gscan-max=%d — tear sync ON",
			         static_cast<long long>(scanPeriodUs_), 1e6 / static_cast<double>(scanPeriodUs_),
			         deltaCount, scanlineMax_);
		} else {
			scanPeriodUs_ = 0;
			ESP_LOGW(kTag, "GSCAN unreadable (max=%d deltas=%d) — tear sync disabled", maxLine, deltaCount);
		}
	}

	// Block until the panel's gate scan wraps to the top. Sleeps through the
	// bulk of the ramp (estimated from the measured period), then spin-polls
	// the wrap. No-op when scan timing is unknown. Called WITHOUT the mutex.
	void waitForScanTop()
	{
		if (!ioRead_ || scanPeriodUs_ <= 0 || scanlineMax_ <= 0) return;
		const std::int64_t deadline = esp_timer_get_time() + 2 * scanPeriodUs_;
		int prev = -1;
		for (;;) {
			int line = 0;
			if (!readScanline(&line)) return;
			if (prev >= 0 && line + scanlineMax_ / 4 < prev) return;  // wrapped: scan is at the top
			if (line >= 0 && line < scanlineMax_) {
				const std::int64_t remainUs =
					static_cast<std::int64_t>(scanlineMax_ - line) * scanPeriodUs_ / scanlineMax_;
				if (remainUs > 3000) vTaskDelay(pdMS_TO_TICKS((remainUs - 2000) / 1000));
			}
			prev = line;
			if (esp_timer_get_time() > deadline) return;
		}
	}

	// Landscape flush onto the portrait-addressed panel: rotate each band into
	// the panel's scan order so the GRAM write front advances WITH the gate
	// scan (top physical row to bottom), the way the tear-free stock demo
	// pushes. A logical rect {x0..x1, y0..y1} maps to the panel rect
	// {px: y0..y1, py: panelH-1-x1 .. panelH-1-x0} (LandscapePrimary) or
	// {px: panelW-1-y1 .. panelW-1-y0, py: x0..x1} (LandscapeSecondary).
	// The transpose loop reads the logical framebuffer row-sequentially (each
	// inner run is a contiguous <=band-height stretch of one logical row in
	// PSRAM) and scatters into the internal-RAM staging band, where strided
	// writes are cheap.
	bool flushRotatedLogicalRect(const present::Rect &logicalRect)
	{
		take();
		bindCanvasLocked();
		if (!panel_ || !frameBuffer_ || !stagingPixels_) {
			give();
			return false;
		}

		const bool primary = flushRotation_ == kFlushRotationLandscapePrimary;
		const int panelH = kNativeHeight;  // 480 physical scan lines
		const int panelW = kNativeWidth;   // 320 pixels per scan line
		const int x0 = logicalRect.x0, x1 = logicalRect.x1;
		const int y0 = logicalRect.y0, y1 = logicalRect.y1;

		// Write back the logical source region first so PSRAM stays coherent
		// with what is pushed (same hazard as the 1:1 path: the camera's
		// NativeOverlay invalidate would otherwise drop CPU-drawn pixels that
		// only live in the write-back cache). Full-width rows keep the range
		// cache-line aligned.
		esp_cache_msync(frameBuffer_ + static_cast<std::size_t>(y0) * logicalStride_,
		                static_cast<std::size_t>(y1 - y0 + 1) * logicalStride_ * sizeof(std::uint16_t),
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M);

		const int px0 = primary ? y0 : panelW - 1 - y1;
		const int px1 = primary ? y1 : panelW - 1 - y0;
		const int py0 = primary ? panelH - 1 - x1 : x0;
		const int py1 = primary ? panelH - 1 - x0 : x1;
		const int bandW = px1 - px0 + 1;
		const int maxRows = std::max(1, std::min(flushRows_, kFlushRowsDefault));

		bool ok = true;
		for (int row = py0; row <= py1 && ok; row += maxRows) {
			int rows = maxRows;
			if (row + rows > py1 + 1) rows = py1 - row + 1;

			const std::int64_t swapStartUs = esp_timer_get_time();
			for (int ly = y0; ly <= y1; ++ly) {
				const std::uint16_t *srcRow = frameBuffer_ + static_cast<std::size_t>(ly) * logicalStride_;
				const int cx = primary ? ly - y0 : y1 - ly;  // column inside the staging band
				std::uint16_t *dst = stagingPixels_ + cx;
				for (int r = row; r < row + rows; ++r) {
					const int lx = primary ? panelH - 1 - r : r;
					dst[static_cast<std::size_t>(r - row) * bandW] = __builtin_bswap16(srcRow[lx]);
				}
			}
			flushStats_.byteSwapUs += esp_timer_get_time() - swapStartUs;

			const std::int64_t txStartUs = esp_timer_get_time();
			const esp_err_t err =
				esp_lcd_panel_draw_bitmap(panel_, px0, row, px1 + 1, row + rows, stagingPixels_);
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "rotated draw_bitmap failed: %s", esp_err_to_name(err));
				ok = false;
				break;
			}
			if (flushDoneSem_) xSemaphoreTake(flushDoneSem_, portMAX_DELAY);
			flushStats_.txUs += esp_timer_get_time() - txStartUs;
			flushStats_.chunkCount++;
			flushStats_.pixelCount += bandW * rows;
		}
		give();
		return ok;
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
	// Slow IO for ST7796 GSCAN reads + the measured panel scan timing
	// (see measureScanTiming/waitForScanTop). scanPeriodUs_ == 0 disables sync.
	esp_lcd_panel_io_handle_t ioRead_ = nullptr;
	std::int64_t scanPeriodUs_ = 0;
	int scanlineMax_ = 0;
	SemaphoreHandle_t flushDoneSem_ = nullptr;
	std::uint16_t *frameBuffer_ = nullptr;
	std::uint16_t *ownedFrameBuffer_ = nullptr;
	std::uint16_t *stagingPixels_ = nullptr;
	std::uint16_t *panelSwapBuf_ = nullptr;  // camera present: byteswapped full block (PSRAM)
	// Panel-push worker (presentPanelRectAsync).
	TaskHandle_t panelPushTask_ = nullptr;
	SemaphoreHandle_t panelPushStart_ = nullptr;
	SemaphoreHandle_t panelPushDone_ = nullptr;
	PanelPushJob panelPushJob_{};
	bool panelPushPending_ = false;
	// Queued-or-streaming async pushes (see waitPanelPushIdle). Distinct from
	// panelPushPending_/panelPushDone_, which belong to the submitter's
	// back-pressure handshake and must not be touched by other threads.
	std::atomic<int> panelPushInflight_{0};
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	gea::framework::graphics::Canvas workerCanvas_;
	int logicalStride_ = kMaxLogicalWidth;
	int renderCoreId_ = -1;
	int flushRows_ = kFlushRowsDefault;
	// Landscape-on-portrait-panel flush rotation (see applyPanelOrientation).
	int flushRotation_ = kFlushRotationNone;
	int flushDepth_ = 1;
	int brightness_ = 100;
	bool brightnessInitialized_ = false;
	bool initialized_ = false;
	bool frameBufferCoherent_ = true;
	bool backgroundCacheAttempted_ = false;
	bool backdropCacheAttempted_ = false;
	platform_display::DisplayFlushPerfStats flushStats_{};
	present::Frame previousPresentFrame_{};
	bool previousPresentValid_ = false;
	bool needsFullPhysicalFlush_ = true;
};

}  // namespace gea::platform::esp32_p4_waveshare::display

extern "C" std::uint16_t *gea_bg_cache(int *cap_px)
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().backgroundCache(cap_px);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().backdropCache(cap_px);
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
volatile std::int64_t gRenderWorkerSubmitUs = 0;
std::int64_t gRenderWorkerStartLatencyUs = 0;
std::int64_t gRenderWorkerBandUs = 0;
std::int64_t gRenderWorkerWaitUs = 0;
int gRenderWorkerJobs = 0;
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
			std::printf("[render-worker] create failed: create=%d stack=%u internal_free=%u internal_largest=%u internal_min=%u free_blocks=%u alloc_blocks=%u total_blocks=%u\n",
			            static_cast<int>(created),
			            static_cast<unsigned>(kRenderWorkerStackBytes),
			            static_cast<unsigned>(info.total_free_bytes),
			            static_cast<unsigned>(info.largest_free_block),
			            static_cast<unsigned>(info.minimum_free_bytes),
			            static_cast<unsigned>(info.free_blocks),
			            static_cast<unsigned>(info.allocated_blocks),
			            static_cast<unsigned>(info.total_blocks));
		}
	}
	if (!gRenderWorker.task) return false;
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().setRenderCore(static_cast<int>(xPortGetCoreID()));
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
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().absorbWorkerDirty();
}

extern "C" int gea_current_render_core()
{
	return static_cast<int>(xPortGetCoreID());
}

namespace gea::platform::esp32_p4_waveshare::display_native {

bool panelMapLogicalRect(int lx, int ly, int lw, int lh,
                         int *px, int *py, int *pw, int *ph, int *extraRotCcw)
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().panelMapLogicalRect(
		lx, ly, lw, lh, px, py, pw, ph, extraRotCcw);
}

bool presentPanelRect(const std::uint16_t *pixels, int px, int py, int w, int h)
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().presentPanelRect(pixels, px, py, w, h);
}

bool presentPanelRectAsync(const std::uint16_t *pixels, int px, int py, int w, int h)
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().presentPanelRectAsync(pixels, px, py, w, h);
}

std::int64_t panelScanPeriodUs()
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().panelScanPeriodUs();
}

void waitPanelPushIdle()
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().waitPanelPushIdle();
}

void clearLogicalRect(int x, int y, int w, int h)
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().clearLogicalRect(x, y, w, h);
}

}  // namespace gea::platform::esp32_p4_waveshare::display_native

namespace gea::platform::display {

namespace {

gea::framework::graphics::Canvas *drawingCanvas()
{
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().drawingCanvas();
}

}  // namespace

bool Display::init() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().init(); }
bool Display::start() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().start(); }
gea::framework::graphics::Canvas *Display::canvas() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().canvas(); }
bool Display::framebufferIsPanelDirect() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().framebufferIsPanelDirect(); }
void Display::clear() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().clear(); }
void Display::clearNoFlush() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flush(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushRects(rects, count); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int command_count) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().presentCommands(commands, command_count); }
void Display::setFlushConfig(int chunk_rows, int queue_depth) { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().setFlushConfig(chunk_rows, queue_depth); }
void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushChunkRows(); }
int Display::flushQueueDepth() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushQueueDepth(); }
int Display::flushBufferBytes() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushBufferBytes(); }
void Display::pushClip(int x, int y, int w, int h) { if (auto *c = drawingCanvas()) c->pushClip(x, y, w, h); }
void Display::popClip() { if (auto *c = drawingCanvas()) c->popClip(); }
void Display::resetClip() { if (auto *c = drawingCanvas()) c->resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { if (auto *c = drawingCanvas()) c->setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return drawingCanvas() ? drawingCanvas()->globalAlpha() : 255; }
int Display::brightness() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().brightness(); }
void Display::setBrightness(int brightness_percent) { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().setBrightness(brightness_percent); }
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
	const auto stats = gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStats();
	if (total_us) *total_us = stats.totalUs;
	if (call_count) *call_count = stats.callCount;
	if (pixel_count) *pixel_count = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStats(); }
void Display::flushStatsReset() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStatsReset(); }
const char *Display::flushStageName() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStageName(); }
int Display::flushStageChunk() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStageChunk(); }
DisplayFlushStageDetail Display::flushStageDetail() { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushStageDetail(); }
bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixel_capacity, int *width, int *height, bool) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().copySnapshotRgb565(dst, pixel_capacity, width, height); }
int Display::countNonBlackPixels(bool) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().countNonBlackPixels(); }

void applyOrientation(gea::framework::display::DisplayOrientation)
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().applyOrientation();
}

}  // namespace gea::platform::display
