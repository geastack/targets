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
#include <new>
#include <utility>

#include "driver/ledc.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9881c.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_async_memcpy.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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
constexpr int kDsiLaneCount = 2;
constexpr float kDsiLaneBitrateMbps = 1000.0f;
constexpr float kDpiClockMhz = 80.0f;
constexpr pixel::Format kFramebufferPixelFormat = pixel::Format::Rgb565;
constexpr pixel::Format kDsiOutputPixelFormat = pixel::Format::Rgb565;
constexpr int kPanelBitsPerPixel = pixel::bitsPerPixel(kDsiOutputPixelFormat);
// Two DPI scanout framebuffers: a full-screen producer (the camera) can write the
// non-scanned (back) buffer and flip it in at vblank — tear-free — instead of writing
// the live buffer mid-scan. The UI / windowed modes keep using whichever buffer is
// currently scanned (single-buffer), so only a full-screen direct producer pays for
// the second buffer's PSRAM.
constexpr int kDpiFrameBufferCount = 2;
constexpr int kDsiPhyLdoChannel = 3;
constexpr int kDsiPhyLdoVoltageMv = 2500;
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_1;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_1;
constexpr int kBacklightLedcMaxDuty = 1023;
constexpr int kMaxPresentRects = 16;
constexpr int kOwnedFrameBufferPixels = kMaxPixels;

struct PanelCoord {
	int x = 0;
	int y = 0;
};

struct PanelInitCommand {
	int cmd = 0;
	std::uint8_t data[3] = {};
	std::size_t dataBytes = 0;
	unsigned int delayMs = 0;
};

constexpr lcd_color_format_t lcdColorFormat(pixel::Format format)
{
	switch (format) {
	case pixel::Format::Rgb565:
		return LCD_COLOR_FMT_RGB565;
	case pixel::Format::Rgb888:
		return LCD_COLOR_FMT_RGB888;
	case pixel::Format::Rgba8888:
		break;
	}
	return LCD_COLOR_FMT_RGB565;
}

static constexpr PanelInitCommand kWaveshareIli9881cInitCommands[] = {
	{0xFF, {0x98, 0x81, 0x03}, 3, 0},
	{0x01, {0x00}, 1, 0},
	{0x02, {0x00}, 1, 0},
	{0x03, {0x73}, 1, 0},
	{0x04, {0x00}, 1, 0},
	{0x05, {0x00}, 1, 0},
	{0x06, {0x0A}, 1, 0},
	{0x07, {0x00}, 1, 0},
	{0x08, {0x00}, 1, 0},
	{0x09, {0x61}, 1, 0},
	{0x0A, {0x00}, 1, 0},
	{0x0B, {0x00}, 1, 0},
	{0x0C, {0x01}, 1, 0},
	{0x0D, {0x00}, 1, 0},
	{0x0E, {0x00}, 1, 0},
	{0x0F, {0x61}, 1, 0},
	{0x10, {0x61}, 1, 0},
	{0x11, {0x00}, 1, 0},
	{0x12, {0x00}, 1, 0},
	{0x13, {0x00}, 1, 0},
	{0x14, {0x00}, 1, 0},
	{0x15, {0x00}, 1, 0},
	{0x16, {0x00}, 1, 0},
	{0x17, {0x00}, 1, 0},
	{0x18, {0x00}, 1, 0},
	{0x19, {0x00}, 1, 0},
	{0x1A, {0x00}, 1, 0},
	{0x1B, {0x00}, 1, 0},
	{0x1C, {0x00}, 1, 0},
	{0x1D, {0x00}, 1, 0},
	{0x1E, {0x40}, 1, 0},
	{0x1F, {0x80}, 1, 0},
	{0x20, {0x06}, 1, 0},
	{0x21, {0x01}, 1, 0},
	{0x22, {0x00}, 1, 0},
	{0x23, {0x00}, 1, 0},
	{0x24, {0x00}, 1, 0},
	{0x25, {0x00}, 1, 0},
	{0x26, {0x00}, 1, 0},
	{0x27, {0x00}, 1, 0},
	{0x28, {0x33}, 1, 0},
	{0x29, {0x03}, 1, 0},
	{0x2A, {0x00}, 1, 0},
	{0x2B, {0x00}, 1, 0},
	{0x2C, {0x00}, 1, 0},
	{0x2D, {0x00}, 1, 0},
	{0x2E, {0x00}, 1, 0},
	{0x2F, {0x00}, 1, 0},
	{0x30, {0x00}, 1, 0},
	{0x31, {0x00}, 1, 0},
	{0x32, {0x00}, 1, 0},
	{0x33, {0x00}, 1, 0},
	{0x34, {0x04}, 1, 0},
	{0x35, {0x00}, 1, 0},
	{0x36, {0x00}, 1, 0},
	{0x37, {0x00}, 1, 0},
	{0x38, {0x3C}, 1, 0},
	{0x39, {0x00}, 1, 0},
	{0x3A, {0x00}, 1, 0},
	{0x3B, {0x00}, 1, 0},
	{0x3C, {0x00}, 1, 0},
	{0x3D, {0x00}, 1, 0},
	{0x3E, {0x00}, 1, 0},
	{0x3F, {0x00}, 1, 0},
	{0x40, {0x00}, 1, 0},
	{0x41, {0x00}, 1, 0},
	{0x42, {0x00}, 1, 0},
	{0x43, {0x00}, 1, 0},
	{0x44, {0x00}, 1, 0},
	{0x50, {0x10}, 1, 0},
	{0x51, {0x32}, 1, 0},
	{0x52, {0x54}, 1, 0},
	{0x53, {0x76}, 1, 0},
	{0x54, {0x98}, 1, 0},
	{0x55, {0xBA}, 1, 0},
	{0x56, {0x10}, 1, 0},
	{0x57, {0x32}, 1, 0},
	{0x58, {0x54}, 1, 0},
	{0x59, {0x76}, 1, 0},
	{0x5A, {0x98}, 1, 0},
	{0x5B, {0xBA}, 1, 0},
	{0x5C, {0xDC}, 1, 0},
	{0x5D, {0xFE}, 1, 0},
	{0x5E, {0x00}, 1, 0},
	{0x5F, {0x0E}, 1, 0},
	{0x60, {0x0F}, 1, 0},
	{0x61, {0x0C}, 1, 0},
	{0x62, {0x0D}, 1, 0},
	{0x63, {0x06}, 1, 0},
	{0x64, {0x07}, 1, 0},
	{0x65, {0x02}, 1, 0},
	{0x66, {0x02}, 1, 0},
	{0x67, {0x02}, 1, 0},
	{0x68, {0x02}, 1, 0},
	{0x69, {0x01}, 1, 0},
	{0x6A, {0x00}, 1, 0},
	{0x6B, {0x02}, 1, 0},
	{0x6C, {0x15}, 1, 0},
	{0x6D, {0x14}, 1, 0},
	{0x6E, {0x02}, 1, 0},
	{0x6F, {0x02}, 1, 0},
	{0x70, {0x02}, 1, 0},
	{0x71, {0x02}, 1, 0},
	{0x72, {0x02}, 1, 0},
	{0x73, {0x02}, 1, 0},
	{0x74, {0x02}, 1, 0},
	{0x75, {0x0E}, 1, 0},
	{0x76, {0x0F}, 1, 0},
	{0x77, {0x0C}, 1, 0},
	{0x78, {0x0D}, 1, 0},
	{0x79, {0x06}, 1, 0},
	{0x7A, {0x07}, 1, 0},
	{0x7B, {0x02}, 1, 0},
	{0x7C, {0x02}, 1, 0},
	{0x7D, {0x02}, 1, 0},
	{0x7E, {0x02}, 1, 0},
	{0x7F, {0x01}, 1, 0},
	{0x80, {0x00}, 1, 0},
	{0x81, {0x02}, 1, 0},
	{0x82, {0x14}, 1, 0},
	{0x83, {0x15}, 1, 0},
	{0x84, {0x02}, 1, 0},
	{0x85, {0x02}, 1, 0},
	{0x86, {0x02}, 1, 0},
	{0x87, {0x02}, 1, 0},
	{0x88, {0x02}, 1, 0},
	{0x89, {0x02}, 1, 0},
	{0x8A, {0x02}, 1, 0},
	{0xFF, {0x98, 0x81, 0x04}, 3, 0},
	{0x38, {0x01}, 1, 0},
	{0x39, {0x00}, 1, 0},
	{0x6C, {0x15}, 1, 0},
	{0x6E, {0x2A}, 1, 0},
	{0x6F, {0x33}, 1, 0},
	{0x3A, {0x94}, 1, 0},
	{0x8D, {0x14}, 1, 0},
	{0x87, {0xBA}, 1, 0},
	{0x26, {0x76}, 1, 0},
	{0xB2, {0xD1}, 1, 0},
	{0xB5, {0x06}, 1, 0},
	{0x3B, {0x98}, 1, 0},
	{0xFF, {0x98, 0x81, 0x01}, 3, 0},
	{0x22, {0x0A}, 1, 0},
	{0x31, {0x00}, 1, 0},
	{0x53, {0x71}, 1, 0},
	{0x55, {0x8F}, 1, 0},
	{0x40, {0x33}, 1, 0},
	{0x50, {0x96}, 1, 0},
	{0x51, {0x96}, 1, 0},
	{0x60, {0x23}, 1, 0},
	{0xA0, {0x08}, 1, 0},
	{0xA1, {0x1D}, 1, 0},
	{0xA2, {0x2A}, 1, 0},
	{0xA3, {0x10}, 1, 0},
	{0xA4, {0x15}, 1, 0},
	{0xA5, {0x28}, 1, 0},
	{0xA6, {0x1C}, 1, 0},
	{0xA7, {0x1D}, 1, 0},
	{0xA8, {0x7E}, 1, 0},
	{0xA9, {0x1D}, 1, 0},
	{0xAA, {0x29}, 1, 0},
	{0xAB, {0x6B}, 1, 0},
	{0xAC, {0x1A}, 1, 0},
	{0xAD, {0x18}, 1, 0},
	{0xAE, {0x4B}, 1, 0},
	{0xAF, {0x20}, 1, 0},
	{0xB0, {0x27}, 1, 0},
	{0xB1, {0x50}, 1, 0},
	{0xB2, {0x64}, 1, 0},
	{0xB3, {0x39}, 1, 0},
	{0xC0, {0x08}, 1, 0},
	{0xC1, {0x1D}, 1, 0},
	{0xC2, {0x2A}, 1, 0},
	{0xC3, {0x10}, 1, 0},
	{0xC4, {0x15}, 1, 0},
	{0xC5, {0x28}, 1, 0},
	{0xC6, {0x1C}, 1, 0},
	{0xC7, {0x1D}, 1, 0},
	{0xC8, {0x7E}, 1, 0},
	{0xC9, {0x1D}, 1, 0},
	{0xCA, {0x29}, 1, 0},
	{0xCB, {0x6B}, 1, 0},
	{0xCC, {0x1A}, 1, 0},
	{0xCD, {0x18}, 1, 0},
	{0xCE, {0x4B}, 1, 0},
	{0xCF, {0x20}, 1, 0},
	{0xD0, {0x27}, 1, 0},
	{0xD1, {0x50}, 1, 0},
	{0xD2, {0x64}, 1, 0},
	{0xD3, {0x39}, 1, 0},
	{0xFF, {0x98, 0x81, 0x00}, 3, 0},
	{0x3A, {0x77}, 1, 0},
	{0x36, {0x00}, 1, 0},
	{0x35, {0x00}, 1, 0},
	{0x35, {0x00}, 1, 0},
	{0x11, {0x00}, 0, 150},
	{0x29, {0x00}, 0, 20},
};

constexpr std::size_t kWaveshareIli9881cInitCommandCount =
	sizeof(kWaveshareIli9881cInitCommands) / sizeof(kWaveshareIli9881cInitCommands[0]);

const ili9881c_lcd_init_cmd_t *waveshareIli9881cInitCommands(std::uint16_t *count)
{
	static ili9881c_lcd_init_cmd_t commands[kWaveshareIli9881cInitCommandCount]{};
	static bool initialized = false;
	if (!initialized) {
		for (std::size_t i = 0; i < kWaveshareIli9881cInitCommandCount; ++i) {
			const auto &src = kWaveshareIli9881cInitCommands[i];
			commands[i].cmd = src.cmd;
			commands[i].data = src.dataBytes > 0 ? src.data : nullptr;
			commands[i].data_bytes = src.dataBytes;
			commands[i].delay_ms = src.delayMs;
		}
		initialized = true;
	}
	if (count) *count = static_cast<std::uint16_t>(kWaveshareIli9881cInitCommandCount);
	return commands;
}

esp_lcd_video_timing_t videoTiming()
{
	esp_lcd_video_timing_t timing{};
	timing.h_size = kNativeWidth;
	timing.v_size = kNativeHeight;
	timing.hsync_pulse_width = 50;
	timing.hsync_back_porch = 239;
	timing.hsync_front_porch = 33;
	timing.vsync_pulse_width = 30;
	timing.vsync_back_porch = 20;
	timing.vsync_front_porch = 2;
	return timing;
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
		if (outBuf) *outBuf = canFlip ? panelFrameBuffers_[1 - scannedFbIndex_] : panelFrameBuffer_;
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
	// PPA-wrote panelFrameBuffers_[1 - scannedFbIndex_] (returned by panelDirectTarget with
	// outFlip=1); draw_bitmap with a pointer inside an internal fb does no copy, it cache-
	// syncs the rows and swaps cur_fb_index, which the DSI DMA picks up at the frame restart.
	void flipPanelToBack()
	{
		if (!panel_ || panelFrameBuffers_[1] == nullptr) return;
		const int back = 1 - scannedFbIndex_;
		esp_lcd_panel_draw_bitmap(panel_, 0, 0, kNativeWidth, kNativeHeight, panelFrameBuffers_[back]);
		scannedFbIndex_ = back;
		panelFrameBuffer_ = panelFrameBuffers_[back];  // UI writes follow the scanned buffer
	}

	gea::framework::graphics::Canvas *drawingCanvas()
	{
		if (!frameBuffer_) init();
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
		presentCalls_++;
		if (presentCalls_ <= 3 || (presentCalls_ & 31) == 1) {
			ESP_LOGI(kTag, "present paths: total=%d direct=%d general=%d externalFlush=%d rejected=%d",
			         presentCalls_, pathDirect_, pathGeneral_, pathExternalFlush_, pathRejected_);
		}
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
			if (refreshDoneSem_ && lastFlipUs_ != 0 && esp_timer_get_time() - lastFlipUs_ < 20000) {
				xSemaphoreTake(refreshDoneSem_, pdMS_TO_TICKS(40));
			}
			std::uint16_t *back = panelFrameBuffers_[1 - scannedFbIndex_];
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
			if (refreshDoneSem_) xSemaphoreTake(refreshDoneSem_, 0);
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
					if (refreshDoneSem_ && lastFlipUs_ != 0 && esp_timer_get_time() - lastFlipUs_ < 20000) {
						xSemaphoreTake(refreshDoneSem_, pdMS_TO_TICKS(40));
					}
					landAlignUs_ = static_cast<int>(esp_timer_get_time() - alignStart);
				}
				std::uint16_t *back = panelFrameBuffers_[1 - scannedFbIndex_];
				std::uint16_t *front = panelFrameBuffers_[scannedFbIndex_];
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
				const int backIndex = 1 - scannedFbIndex_;
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
				if (refreshDoneSem_) xSemaphoreTake(refreshDoneSem_, 0);
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
			landPrevValid_[0] = false;
			landPrevValid_[1] = false;
		}
		{
			using gea::framework::display::DisplayOrientation;
			using Type = gea::platform::display::DisplayPresentCommandType;
			const bool rotatedMode = !(orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary &&
			                           usingPanelFrameBufferDirectly());
			int shiftDx = 0;
			int shiftDy = 0;
			const int absLimitX = logicalWidth() / 2;
			const int absLimitY = logicalHeight() / 2;
			const bool panPreconditions = tileShape && rotatedMode && previousPresentValid_ && !needsFullPhysicalFlush_ && frameBufferCoherent_;
			const bool panShiftFound = panPreconditions && detectUniformShift(previousPresentFrame_, current, &shiftDx, &shiftDy);
			if (panPreconditions && !panShiftFound) {
				static int panMisses = 0;
				panMisses++;
				if ((panMisses & 31) == 1) ESP_LOGI(kTag, "torus MISS #%d (no uniform shift; cmds=%d)", panMisses, static_cast<int>(current.commands.size()));
			}
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
		// Always compose into the off-screen owned buffer (never directly into the
		// scanned-out panel framebuffer). The flush then PPA-blits the dirty region
		// owned->panel: the runtime's multi-ms rasterization no longer touches the
		// live scanout buffer, so the only write the DSI can catch mid-scan is the
		// fast hardware PPA copy — shrinking the tear window from the whole frame's
		// raster down to a sub-ms blit. (Portrait is a 0deg PPA copy.)
		return ownedFrameBuffer_;
	}

	bool usingPanelFrameBufferDirectly() const
	{
		using gea::framework::display::DisplayOrientation;
		return frameBuffer_ && panelFrameBuffer_ && frameBuffer_ == panelFrameBuffer_ &&
		       logicalStride_ == kNativeWidth &&
		       orientation_detail::DisplayOrientationState::orientation() == DisplayOrientation::PortraitPrimary;
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

	bool syncPanelFrameBufferRectToMemory(present::Rect panelRect)
	{
		return syncBufferRectToMemory(panelFrameBuffer_, kNativeWidth, kNativeWidth, kNativeHeight, panelRect, "DPI panel framebuffer");
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
		const std::uint32_t duty = static_cast<std::uint32_t>((kBacklightLedcMaxDuty * brightness_) / 100);
		const esp_err_t setErr = ledc_set_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel, duty);
		const esp_err_t updateErr = ledc_update_duty(LEDC_LOW_SPEED_MODE, kBacklightLedcChannel);
		if (setErr != ESP_OK || updateErr != ESP_OK) {
			ESP_LOGE(kTag, "backlight duty update failed: set=%s update=%s",
			         esp_err_to_name(setErr),
			         esp_err_to_name(updateErr));
		}
	}

	bool initPanel()
	{
		if (panel_) return true;
		if (!initBacklight() || !enableDsiPhyPower()) return false;

		esp_lcd_dsi_bus_config_t busConfig{};
		busConfig.bus_id = 0;
		busConfig.num_data_lanes = kDsiLaneCount;
		busConfig.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
		busConfig.lane_bit_rate_mbps = kDsiLaneBitrateMbps;
		esp_err_t err = esp_lcd_new_dsi_bus(&busConfig, &mipiDsiBus_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "DSI bus init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_lcd_dbi_io_config_t dbiConfig{};
		dbiConfig.virtual_channel = 0;
		dbiConfig.lcd_cmd_bits = 8;
		dbiConfig.lcd_param_bits = 8;
		err = esp_lcd_new_panel_io_dbi(mipiDsiBus_, &dbiConfig, &io_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "DSI DBI IO init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_lcd_dpi_panel_config_t dpiConfig{};
		dpiConfig.virtual_channel = 0;
		dpiConfig.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
		dpiConfig.dpi_clock_freq_mhz = kDpiClockMhz;
		dpiConfig.in_color_format = lcdColorFormat(kFramebufferPixelFormat);
		dpiConfig.out_color_format = lcdColorFormat(kDsiOutputPixelFormat);
		dpiConfig.num_fbs = kDpiFrameBufferCount;
		dpiConfig.video_timing = videoTiming();

		std::uint16_t panelInitCommandCount = 0;
		const ili9881c_lcd_init_cmd_t *panelInitCommands = waveshareIli9881cInitCommands(&panelInitCommandCount);

		ili9881c_vendor_config_t vendorConfig{};
		vendorConfig.init_cmds = panelInitCommands;
		vendorConfig.init_cmds_size = panelInitCommandCount;
		vendorConfig.mipi_config.dsi_bus = mipiDsiBus_;
		vendorConfig.mipi_config.dpi_config = &dpiConfig;
		vendorConfig.mipi_config.lane_num = kDsiLaneCount;

		esp_lcd_panel_dev_config_t panelConfig{};
		panelConfig.reset_gpio_num = gea::platform::board::display.reset;
		panelConfig.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
		panelConfig.bits_per_pixel = kPanelBitsPerPixel;
		panelConfig.vendor_config = &vendorConfig;

		err = esp_lcd_new_panel_ili9881c(io_, &panelConfig, &panel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "ILI9881C panel create failed: %s", esp_err_to_name(err));
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
		void *fb0 = nullptr;
		void *fb1 = nullptr;
		err = esp_lcd_dpi_panel_get_frame_buffer(panel_, kDpiFrameBufferCount, &fb0, &fb1);
		if (err == ESP_OK && fb0) {
			panelFrameBuffers_[0] = static_cast<std::uint16_t *>(fb0);
			panelFrameBuffers_[1] = static_cast<std::uint16_t *>(fb1);  // null when num_fbs == 1
			scannedFbIndex_ = 0;  // the driver scans fb0 first
			panelFrameBuffer_ = panelFrameBuffers_[0];
			ESP_LOGI(kTag, "using DPI panel framebuffers fb0=%p fb1=%p", panelFrameBuffers_[0], panelFrameBuffers_[1]);
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
		if (err != ESP_OK) {
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
		         "ILI9881C DSI panel ready lanes=%d bitrate=%.0fMbps clock=%.0fMHz fb=%s out=%s panel_bpp=%d dpi_fbs=%d init_cmds=%u",
		         kDsiLaneCount,
		         kDsiLaneBitrateMbps,
		         kDpiClockMhz,
		         pixel::name(kFramebufferPixelFormat),
		         pixel::name(kDsiOutputPixelFormat),
		         kPanelBitsPerPixel,
		         kDpiFrameBufferCount,
		         static_cast<unsigned>(panelInitCommandCount));
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
		ok = rotateLogicalRectToPanelWithPpa(logicalRect, panelRect);
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
	LandPrevCmd landPrev_[2][kLandPrevMax];
	int landPrevCount_[2] = {0, 0};
	bool landPrevValid_[2] = {false, false};
	// Hash of the rail-region commands (fills below kRailPanelRow + all text):
	// the pan path skips rail rows entirely, so ANY rail change must force a
	// full raster or sidebar state changes (search keyboard, zoom label)
	// never reach the glass.
	std::uint32_t landPrevRailHash_[2] = {0, 0};

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
		const int maxRows = std::max(1, kFlushRowsDefault);
		for (int p = 0; p < count; ++p) {
			const TorusPart &part = parts[p];
			const int width = part.torus.x1 - part.torus.x0 + 1;
			const int partRows = part.torus.y1 - part.torus.y0 + 1;
			for (int r = 0; r < partRows; r += maxRows) {
				int rows = maxRows;
				if (r + rows > partRows) rows = partRows - r;
				std::uint16_t *rasterTarget =
					frameBuffer_ + static_cast<std::size_t>(part.torus.y0 + r) * logicalStride_ + part.torus.x0;
				const std::int64_t rasterStartUs = esp_timer_get_time();
				present::rasterFrameRowsStrided(rasterTarget,
				                                logicalStride_,
				                                part.logicalX,
				                                width,
				                                part.logicalY + r,
				                                rows,
				                                height,
				                                frame);
				flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;
			}
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
	std::uint16_t *panelFrameBuffers_[2] = {nullptr, nullptr};  // both DPI scanout buffers (num_fbs)
	int scannedFbIndex_ = 0;  // which of panelFrameBuffers_ the DSI scans now (driver default 0)
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
	gea::framework::graphics::Canvas canvas_;
	gea::framework::graphics::Canvas workerCanvas_;
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
bool Display::panelScanoutSurface(uint16_t **outBuffer, int *outW, int *outH) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().panelScanoutSurface(outBuffer, outW, outH); }
bool Display::panelDirectTarget(int logicalX, int logicalY, int logicalW, int logicalH,
                                uint16_t **outBuf, int *outBufW, int *outBufH,
                                int *outPanelX, int *outPanelY, int *outPanelW, int *outPanelH,
                                int *outRotSteps, int *outFlip) {
	return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().panelDirectTarget(
	    logicalX, logicalY, logicalW, logicalH, outBuf, outBufW, outBufH, outPanelX, outPanelY, outPanelW, outPanelH, outRotSteps, outFlip);
}
void Display::flipPanelToBack() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flipPanelToBack(); }
void Display::clear() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().clear(); }
void Display::clearNoFlush() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flush(); }
void Display::rebindCanvasToFramebuffer() { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().rebindCanvasToFramebuffer(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().flushRects(rects, count); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int command_count) { return gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().presentCommands(commands, command_count); }
void Display::presentPathDebug(int *calls, int *direct, int *general, int *rejected, int *tileShapeFailKind, int *tileShapeFailType, int *tileShapeFailCount)
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().presentPathDebug(
	    calls, direct, general, rejected, tileShapeFailKind, tileShapeFailType, tileShapeFailCount);
}
void Display::landFrameDebug(int *total, int *align, int *raster, int *text, int *flip)
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().landFrameDebug(total, align, raster, text, flip);
}
void Display::landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior)
{
	gea::platform::esp32_p4_waveshare::display::DisplayBackend::instance().landPanDebug(detect, kick, strips, wait, interior);
}
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
