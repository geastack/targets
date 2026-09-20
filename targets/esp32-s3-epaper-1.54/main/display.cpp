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

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace platform_display = gea::platform::display;
namespace orientation_detail = gea::framework::display::detail;
namespace pixel = gea::framework::graphics::pixel;
namespace present = gea::framework::display_present;

namespace gea::platform::esp32_s3_epaper::display {

namespace {

constexpr const char *kTag = "epaper_display";
constexpr int kNativeWidth = platform_display::kNativeWidth;    // 200
constexpr int kNativeHeight = platform_display::kNativeHeight;  // 200
constexpr int kMaxPixels = kNativeWidth * kNativeHeight;
// 1-bit packed panel frame: 200/8 = 25 bytes per row.
constexpr int kPackedStride = kNativeWidth / 8;
constexpr int kPackedBytes = kPackedStride * kNativeHeight;
// SSD1681 datasheet caps the write clock at 20 MHz; the full packed frame is
// 5000 bytes, so the RAM write itself is ~2 ms — refresh time is dominated by
// the waveform (~0.4 s partial, ~1.5 s full), not the SPI.
constexpr int kSpiClockHz = 20 * 1000 * 1000;
// Ghosting control: run a full (flashing) refresh after this many partial
// updates — but only when updates are sparse (a flash mid-animation would
// freeze it for ~1.5 s). During a fast streak the full refresh is deferred to
// the first idle moment, with a hard cap so trails can never run away.
// Defaults; apps can retune all of these (and supply their own waveform LUTs)
// at runtime via Display.setEpaperRefreshConfig().
constexpr int kFullRefreshEveryPartials = 30;
constexpr int kFullRefreshHardCapPartials = 240;
// Updates arriving within this window of the previous refresh are a "streak"
// (animation / held button): switch to the fast partial waveform.
constexpr std::int64_t kFastStreakWindowUs = 1000 * 1000;
constexpr int kLutBytes = 159;

// Waveshare 1.54" V2 (SSD1681) waveform LUTs, from the vendor BSP
// (docs/scratch/ref-esp32-s3-epaper-154). 153 LUT bytes + gate/source voltage
// and dummy-line trailer consumed by EPD_SetLut.
const uint8_t kLutFull[159] = {
	0x80, 0x48, 0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x40, 0x48, 0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x80, 0x48, 0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x40, 0x48, 0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0xA,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x8,  0x1,  0x0,  0x8,  0x1,  0x0,  0x2,
	0xA,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0,  0x0,  0x0,
	0x22, 0x17, 0x41, 0x0,  0x32, 0x20,
};

const uint8_t kLutPartial[159] = {
	0x0,  0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x80, 0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x40, 0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0xF,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x1,  0x1,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0,  0x0,  0x0,
	0x02, 0x17, 0x41, 0xB0, 0x32, 0x28,
};

// Fast partial waveform: the vendor partial LUT with the drive phase cut to 4
// frames and the frame-rate nibbles raised 0x2 -> 0x7. Measured on-device:
// ~265 ms/refresh vs the vendor partial's ~360 ms (~30% faster). This panel's
// controller stepping has a ~240 ms floor in the mode-2 update sequence that
// LUT timing fields don't reach (TP counts only cost ~5 ms/frame at FR 0x7;
// FR 0xF is no faster than 0x7) — so this is near the floor, not a tuning
// timidity. Lighter blacks/ghost trails are the trade; used only during
// update streaks, and the periodic full refresh wipes the residue.
const uint8_t kLutPartialFast[159] = {
	0x0,  0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x80, 0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x40, 0x40, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x80, 0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x4,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x1,  0x1,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,
	0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x0,  0x0,  0x0,
	0x02, 0x17, 0x41, 0xB0, 0x32, 0x28,
};

constexpr int kLut4GrayBytes = 153;

// 4-level grayscale waveform LUT for the SSD1681 (GDEY0154D67 panel class),
// from ZinggJM/GxEPD2_4G (src/gdey/GxEPD2_154_GDEY0154D67.cpp, lut_4G). 153
// bytes written verbatim via 0x32; unlike the B/W LUT, gray mode does NOT
// program the gate/source-voltage trailer (0x3F/0x03/0x04/0x2C) — it relies on
// the controller OTP defaults, matching the reference library. The 4 RAM-plane
// bit combinations select VS groups L0..L3 (white / light / dark / black); the
// per-pixel encoding lives in packFrameGray below.
const uint8_t kLut4Gray[kLut4GrayBytes] = {
	0x40, 0x48, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // VS L0 white
	0x08, 0x48, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // VS L1 light grey
	0x02, 0x48, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // VS L2 dark grey
	0x20, 0x48, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // VS L3 black
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // VS L4 vcom
	0x0A, 0x19, 0x00, 0x03, 0x08, 0x00, 0x00,  // TP0
	0x14, 0x01, 0x00, 0x14, 0x01, 0x00, 0x03,  // TP1
	0x0A, 0x03, 0x00, 0x08, 0x19, 0x00, 0x00,  // TP2
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,  // TP3
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP4
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP5
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP6
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP7
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP8
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP9
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP10
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // TP11
	0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
};

// 4x4 ordered-dither thresholds (Bayer), pre-scaled to 8-bit luma. The counter
// UI is dark-themed: a plain 50% threshold would collapse the navy background
// and the gray button fills into the same solid black. Dithering keeps the
// mid-gray fills visually distinct on the 1-bit panel.
constexpr uint8_t kBayer4x4[4][4] = {
	{8, 136, 40, 168},
	{200, 72, 232, 104},
	{56, 184, 24, 152},
	{248, 120, 216, 88},
};

// Whether to ordered-dither into the 1-bit panel. Dithering only earns its keep
// for continuous-tone content (gray fills, gradients, grayscale imagery). For a
// pure black/white UI like voice-notes it does the opposite: the framebuffer's
// only non-B/W pixels are the anti-aliased edges of glyphs (and AA'd shapes),
// so dithering scatters every letter's rim into speckle. With dithering off we
// hard-threshold at 50% luma, snapping those AA edges to crisp black/white.
// Flip this to true on a build whose app renders real gray fills/images.
constexpr bool kDitherToPanel = false;
constexpr uint8_t kMonoThreshold = 128;

inline uint8_t lumaFromRgb565(std::uint16_t c)
{
	const int r = ((c >> 11) & 0x1f);
	const int g = ((c >> 5) & 0x3f);
	const int b = (c & 0x1f);
	const int r8 = (r << 3) | (r >> 2);
	const int g8 = (g << 2) | (g >> 4);
	const int b8 = (b << 3) | (b >> 2);
	return static_cast<uint8_t>((r8 * 77 + g8 * 150 + b8 * 29) >> 8);
}

// 4-level grayscale: classify 8-bit luma into one of four levels. Level 3 is
// white (brightest), level 0 is black. The two intermediate levels map to the
// SSD1681's two single-plane waveform groups.
inline uint8_t grayLevelFromLuma(uint8_t luma)
{
	// Compressed/perceptual quantization. 4 levels is the SSD1681 ceiling, so
	// linear quarter thresholds (64/128/192) snap thin anti-aliased glyph edges
	// — whose pixels cluster near-black/near-white — straight to pure B/W, and
	// small text reads 1-bit. Narrowing the pure-black/white bands and widening
	// the two grey bands (≈ perceptual midpoints of evenly-spaced reflectances)
	// pulls those edge pixels into dark/light grey, softening text. Pure ink
	// (luma 0) stays black and pure paper (255) stays white. No dithering — it
	// would speckle thin text.
	if (luma >= 208) return 3;  // white
	if (luma >= 128) return 2;  // light grey
	if (luma >= 48) return 1;   // dark grey
	return 0;                   // black
}

// The bit a level contributes to each RAM plane, as actually transmitted to the
// controller. Replicates GxEPD2_4G's writeImage_4G (after its ~out_byte
// inversion): white -> {0,0}, black -> {1,1}, light grey -> {plane24=1,
// plane26=0}, dark grey -> {plane24=0, plane26=1}. The (plane26,plane24) RAM
// pair selects VS group L0..L3 in kLut4Gray.
inline bool grayPlane24Bit(uint8_t level) { return level == 2 || level == 0; }
inline bool grayPlane26Bit(uint8_t level) { return level == 1 || level == 0; }

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
		packMutex_ = xSemaphoreCreateMutex();
		if (!mutex_ || !packMutex_) {
			ESP_LOGE(kTag, "failed to allocate display mutexes");
			return false;
		}

		frameBuffer_ = static_cast<std::uint16_t *>(
			heap_caps_malloc(static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t),
		                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!frameBuffer_) {
			ESP_LOGE(kTag, "framebuffer alloc failed");
			return false;
		}
		std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));

		// Two packed 1-bit frames: the frame task packs into pending_, the
		// refresh worker copies into tx_ and transmits. Only tx_ is DMA'd (it is
		// the SPI source), so it stays internal DMA-capable RAM. pending_ is
		// CPU-only (packed by packFrame, then memcpy'd into tx_), so it lives in
		// PSRAM — that reclaims ~5 KB of scarce internal DMA RAM, which with WiFi
		// up is what lets the SD card allocate its read DMA buffer (otherwise
		// sdmmc allocate_dma_buf NO_MEMs and the WAV can't be read to upload).
		pendingPack_ = static_cast<uint8_t *>(
			heap_caps_malloc(kPackedBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		txPack_ = static_cast<uint8_t *>(
			heap_caps_malloc(kPackedBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		// Second RAM plane (0x26) is used ONLY in 4-level grayscale mode. It is now
		// allocated LAZILY on the first setGrayscale(true) instead of up front, so
		// B/W apps (the default — e.g. voice-notes) don't pin ~10 KB of DMA-capable
		// internal RAM. On this RAM-tight board that reclaimed 10 KB is what lets
		// esp_wifi_init succeed: WiFi needs a contiguous DMA-internal pool, and the
		// 1-bit e-paper panel has no large display staging for reserveInternal to
		// reclaim the way the AMOLED boards do.
		if (!pendingPack_ || !txPack_) {
			ESP_LOGE(kTag, "packed frame alloc failed");
			return false;
		}
		std::memset(pendingPack_, 0xff, kPackedBytes);
		std::memset(txPack_, 0xff, kPackedBytes);

		if (!initPanel()) {
			ESP_LOGE(kTag, "panel init failed");
			return false;
		}
		bindCanvas();

		// Refresh worker off the frame task: a partial refresh holds BUSY for
		// ~0.4 s (full ~1.5 s) and must never stall input/event processing.
		// Updates landing while a refresh is in flight coalesce into pending_.
		if (xTaskCreatePinnedToCore(refreshTaskEntry, "epd_refresh", 4096, this, 5, &refreshTask_, 1) != pdPASS) {
			ESP_LOGE(kTag, "failed to start refresh task");
			return false;
		}

		initialized_ = true;
		ESP_LOGI(kTag, "e-paper display ready %dx%d (packed %d bytes)", kNativeWidth, kNativeHeight, kPackedBytes);
		return true;
	}

	bool start() { return init(); }

	gea::framework::graphics::Canvas *canvas()
	{
		if (!frameBuffer_) init();
		return &canvas_;
	}

	bool framebufferIsPanelDirect() const { return false; }

	void applyOrientation()
	{
		// Square panel: every orientation is 200x200; just repaint fully.
		take();
		bindCanvasLocked();
		if (frameBuffer_) {
			std::memset(frameBuffer_, 0, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
			canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
			previousPresentValid_ = false;
			needsFullPhysicalFlush_ = true;
		}
		give();
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

	// The panel always receives the full packed 200x200 frame (5000 bytes —
	// cheaper to pack wholesale than to window), so flush granularity only
	// decides WHETHER to refresh, never what subset to send.
	void flush()
	{
		if (!init()) return;
		if (!needsFullPhysicalFlush_ && !canvas_.dirty(nullptr, nullptr, nullptr, nullptr)) return;
		const std::int64_t started = esp_timer_get_time();
		if (needsFullPhysicalFlush_) forceFullRefresh_.store(true, std::memory_order_release);
		requestRefresh();
		canvas_.resetDirty();
		needsFullPhysicalFlush_ = false;
		flushStats_.callCount++;
		flushStats_.totalUs += esp_timer_get_time() - started;
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		if (!rects || count <= 0) return;
		if (!init()) return;
		const std::int64_t started = esp_timer_get_time();
		if (needsFullPhysicalFlush_) forceFullRefresh_.store(true, std::memory_order_release);
		requestRefresh();
		canvas_.resetDirty();
		needsFullPhysicalFlush_ = false;
		flushStats_.callCount++;
		flushStats_.totalUs += esp_timer_get_time() - started;
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
		for (int row = y0; row <= y1; ++row) {
			raster(frameBuffer_ + static_cast<std::size_t>(row) * kNativeWidth + x0, width, 1, x0, row, user);
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
		// The panel gets the whole packed frame anyway, so raster the whole
		// frame — diffing against the previous present buys nothing here.
		const present::Rect full = present::fullScreen(logicalWidth(), logicalHeight());
		const std::int64_t started = esp_timer_get_time();
		rasterPresentRegion(current, full);
		if (needsFullPhysicalFlush_) forceFullRefresh_.store(true, std::memory_order_release);
		requestRefresh();
		flushStats_.callCount++;
		flushStats_.totalUs += esp_timer_get_time() - started;
		previousPresentFrame_ = std::move(current);
		previousPresentValid_ = true;
		needsFullPhysicalFlush_ = false;
		canvas_.resetDirty();
		return true;
	}

	void setFlushConfig(int, int) {}
	int flushChunkRows() const { return kNativeHeight; }
	int flushQueueDepth() const { return 1; }
	int flushBufferBytes() const { return kPackedBytes; }

	bool copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height)
	{
		if (!dst || pixelCapacity < kMaxPixels || !frameBuffer_) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		std::memcpy(dst, frameBuffer_, static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t));
		return true;
	}

	int countNonBlackPixels() const
	{
		if (!frameBuffer_) return 0;
		int count = 0;
		for (int i = 0; i < kMaxPixels; ++i) {
			if (pixel::toRgb565(frameBuffer_[i]) != 0) ++count;
		}
		return count;
	}

	// No backlight on an e-paper panel.
	void setBrightness(int) {}
	int brightness() const { return 100; }

	platform_display::DisplayFlushPerfStats flushStats() const { return flushStats_; }
	void flushStatsReset() { flushStats_ = {}; }
	const char *flushStageName() const { return refreshInFlight_.load(std::memory_order_acquire) ? "epd-refresh" : "idle"; }
	int flushStageChunk() const { return 0; }
	platform_display::DisplayFlushStageDetail flushStageDetail() const { return {}; }

	std::uint16_t *backgroundCache(int *capPx) { return fullScreenScratch(backgroundCache_, backgroundCacheAttempted_, capPx, "background"); }
	std::uint16_t *backdropCache(int *capPx) { return fullScreenScratch(backdropCache_, backdropCacheAttempted_, capPx, "backdrop"); }

private:
	DisplayBackend() = default;

	int logicalWidth() const { return orientation_detail::DisplayOrientationState::width(); }
	int logicalHeight() const { return orientation_detail::DisplayOrientationState::height(); }

	std::uint16_t *fullScreenScratch(std::uint16_t *&buffer, bool &attempted, int *capPx, const char *label)
	{
		if (!attempted) {
			attempted = true;
			buffer = static_cast<std::uint16_t *>(
				heap_caps_malloc(static_cast<std::size_t>(kMaxPixels) * sizeof(std::uint16_t),
			                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
			if (!buffer) ESP_LOGW(kTag, "failed to allocate %s cache", label);
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
		if (!frameBuffer_) return;
		if (canvas_.pixels() == frameBuffer_ && canvas_.width() == kNativeWidth && canvas_.height() == kNativeHeight) return;
		canvas_.bindPixels(frameBuffer_, kNativeWidth, kNativeHeight, kNativeWidth);
	}

	void rasterPresentRegion(const present::Frame &frame, const present::Rect &region)
	{
		if (!present::valid(region)) return;
		const int width = region.x1 - region.x0 + 1;
		std::uint16_t *target = frameBuffer_ + static_cast<std::size_t>(region.y0) * kNativeWidth + region.x0;
		present::rasterFrameRowsStrided(target,
		                                kNativeWidth,
		                                region.x0,
		                                width,
		                                region.y0,
		                                region.y1 - region.y0 + 1,
		                                logicalHeight(),
		                                frame);
	}

	// --- RGB565 -> 1-bit packing -------------------------------------------

	// Pack the live framebuffer into pending_ and kick the worker. Runs on the
	// frame task; ~40k pixels of integer math, well under a millisecond.
	void requestRefresh()
	{
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		if (grayscaleEnabled_) {
			packFrameGray(pendingPack_, pendingPack26_);
		} else {
			packFrame(pendingPack_);
		}
		pendingSeq_.fetch_add(1, std::memory_order_release);
		xSemaphoreGive(packMutex_);
		if (refreshTask_) xTaskNotifyGive(refreshTask_);
	}

	void packFrame(uint8_t *dst)
	{
		for (int y = 0; y < kNativeHeight; ++y) {
			const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(y) * kNativeWidth;
			uint8_t *row = dst + static_cast<std::size_t>(y) * kPackedStride;
			for (int xb = 0; xb < kPackedStride; ++xb) {
				uint8_t bits = 0;
				const int x0 = xb << 3;
				for (int bit = 0; bit < 8; ++bit) {
					const uint8_t luma = lumaFromRgb565(src[x0 + bit]);
					const uint8_t threshold = kDitherToPanel ? kBayer4x4[y & 3][(x0 + bit) & 3] : kMonoThreshold;
					if (luma >= threshold) bits |= static_cast<uint8_t>(0x80 >> bit);
				}
				row[xb] = bits;
			}
		}
	}

	// Pack the framebuffer into the two SSD1681 RAM planes for 4-level
	// grayscale. Each pixel's luma picks one of four levels; the level's bit in
	// each plane is laid down MSB-first, matching the gray waveform LUT's group
	// selection. Records the per-level histogram for the next refresh log.
	void packFrameGray(uint8_t *dst24, uint8_t *dst26)
	{
		std::uint32_t counts[4] = {0, 0, 0, 0};
		for (int y = 0; y < kNativeHeight; ++y) {
			const std::uint16_t *src = frameBuffer_ + static_cast<std::size_t>(y) * kNativeWidth;
			uint8_t *row24 = dst24 + static_cast<std::size_t>(y) * kPackedStride;
			uint8_t *row26 = dst26 + static_cast<std::size_t>(y) * kPackedStride;
			for (int xb = 0; xb < kPackedStride; ++xb) {
				uint8_t b24 = 0;
				uint8_t b26 = 0;
				const int x0 = xb << 3;
				for (int bit = 0; bit < 8; ++bit) {
					const uint8_t level = grayLevelFromLuma(lumaFromRgb565(src[x0 + bit]));
					counts[level]++;
					const uint8_t mask = static_cast<uint8_t>(0x80 >> bit);
					if (grayPlane24Bit(level)) b24 |= mask;
					if (grayPlane26Bit(level)) b26 |= mask;
				}
				row24[xb] = b24;
				row26[xb] = b26;
			}
		}
		for (int i = 0; i < 4; ++i) lastGrayLevelCounts_[i] = counts[i];
	}

	// --- SSD1681 panel transport (vendor BSP command sequences) -------------

	void setCs(int level) { gpio_set_level(gea::platform::board::display.cs, level); }
	void setDc(int level) { gpio_set_level(gea::platform::board::display.dc, level); }
	void setRst(int level) { gpio_set_level(gea::platform::board::display.reset, level); }

	void waitBusy()
	{
		// BUSY high = controller refreshing. Full refresh holds it ~1.5 s.
		const std::int64_t deadline = esp_timer_get_time() + 5000000;
		while (gpio_get_level(gea::platform::board::display.busy) == 1) {
			if (esp_timer_get_time() > deadline) {
				ESP_LOGE(kTag, "BUSY stuck high >5s");
				return;
			}
			vTaskDelay(pdMS_TO_TICKS(5));
		}
	}

	void spiSendByte(uint8_t data)
	{
		spi_transaction_t t = {};
		t.length = 8;
		t.tx_buffer = &data;
		ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi_, &t));
	}

	void sendCommand(uint8_t command)
	{
		setDc(0);
		setCs(0);
		spiSendByte(command);
		setCs(1);
	}

	void sendData(uint8_t data)
	{
		setDc(1);
		setCs(0);
		spiSendByte(data);
		setCs(1);
	}

	void sendDataBytes(const uint8_t *data, int len)
	{
		setDc(1);
		setCs(0);
		spi_transaction_t t = {};
		t.length = static_cast<std::size_t>(len) * 8;
		t.tx_buffer = data;
		ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi_, &t));
		setCs(1);
	}

	void setWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
	{
		sendCommand(0x44);
		sendData((xStart >> 3) & 0xFF);
		sendData((xEnd >> 3) & 0xFF);
		sendCommand(0x45);
		sendData(yStart & 0xFF);
		sendData((yStart >> 8) & 0xFF);
		sendData(yEnd & 0xFF);
		sendData((yEnd >> 8) & 0xFF);
	}

	void setCursor(uint16_t x, uint16_t y)
	{
		sendCommand(0x4E);
		sendData(x & 0xFF);
		sendCommand(0x4F);
		sendData(y & 0xFF);
		sendData((y >> 8) & 0xFF);
	}

	void setLut(const uint8_t *lut)
	{
		sendCommand(0x32);
		sendDataBytes(lut, 153);
		waitBusy();
		sendCommand(0x3f);
		sendData(lut[153]);
		sendCommand(0x03);
		sendData(lut[154]);
		sendCommand(0x04);
		sendData(lut[155]);
		sendData(lut[156]);
		sendData(lut[157]);
		sendCommand(0x2c);
		sendData(lut[158]);
	}

	void hardwareReset()
	{
		setRst(1);
		vTaskDelay(pdMS_TO_TICKS(50));
		setRst(0);
		vTaskDelay(pdMS_TO_TICKS(20));
		setRst(1);
		vTaskDelay(pdMS_TO_TICKS(50));
	}

	// Full-waveform init (vendor EPD_Init): SWRESET, driver output 0xC7=199
	// gates, data entry Y-decrement/X-increment with the cursor at the top, so
	// linearly streamed RAM bytes land top row first.
	void panelInitFull()
	{
		hardwareReset();
		waitBusy();
		sendCommand(0x12);  // SWRESET
		waitBusy();
		sendCommand(0x01);  // driver output control
		sendData(0xC7);
		sendData(0x00);
		sendData(0x01);
		sendCommand(0x11);  // data entry mode: X inc, Y dec
		sendData(0x01);
		setWindow(0, kNativeWidth - 1, kNativeHeight - 1, 0);
		sendCommand(0x3C);  // border waveform
		sendData(0x01);
		sendCommand(0x18);  // internal temperature sensor
		sendData(0x80);
		sendCommand(0x22);  // load temperature + waveform
		sendData(0xB1);
		sendCommand(0x20);
		setCursor(0, kNativeHeight - 1);
		waitBusy();
		setLut(kLutFull);
	}

	// Vendor partial-LUT activation: the LUT write only lands after the
	// display-option block + the 0x22=0xC0/0x20 analog re-enable — a bare
	// 0x32 write mid-flight leaves the previously latched waveform running.
	void activatePartialLut(const uint8_t *lut)
	{
		setLut(lut);
		sendCommand(0x37);
		const uint8_t otp[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00};
		for (int i = 0; i < 10; ++i) sendData(otp[i]);
		sendCommand(0x3C);  // border waveform
		sendData(0x80);
		sendCommand(0x22);
		sendData(0xc0);
		sendCommand(0x20);
		waitBusy();
	}

	void panelInitPartial()
	{
		hardwareReset();
		waitBusy();
		activatePartialLut(kLutPartial);
	}

	void turnOnDisplayFull()
	{
		sendCommand(0x22);
		sendData(0xc7);
		sendCommand(0x20);
		waitBusy();
	}

	void turnOnDisplayPartial()
	{
		sendCommand(0x22);
		sendData(0xcf);
		sendCommand(0x20);
		waitBusy();
	}

	void writeRam(uint8_t reg, const uint8_t *packed)
	{
		setCursor(0, kNativeHeight - 1);
		sendCommand(reg);
		sendDataBytes(packed, kPackedBytes);
	}

	// --- 4-level grayscale transport (GxEPD2_4G GDEY0154D67 sequences) -------

	// Gray mode scans top-to-bottom (X inc, Y inc) with the cursor at the
	// origin, so packFrameGray's natural top-row-first layout lands correctly.
	void setGrayRamArea()
	{
		sendCommand(0x11);  // data entry mode: X inc, Y inc
		sendData(0x03);
		sendCommand(0x44);
		sendData(0x00);
		sendData((kNativeWidth - 1) >> 3);
		sendCommand(0x45);
		sendData(0x00);
		sendData(0x00);
		sendData((kNativeHeight - 1) & 0xFF);
		sendData(((kNativeHeight - 1) >> 8) & 0xFF);
		sendCommand(0x4E);
		sendData(0x00);
		sendCommand(0x4F);
		sendData(0x00);
		sendData(0x00);
	}

	// Gray init: load the 4-level waveform via 0x32 (153 bytes, no voltage
	// trailer — gray mode uses OTP defaults). Border waveform 0x00 per the
	// reference library.
	void panelInitGray()
	{
		hardwareReset();
		waitBusy();
		sendCommand(0x12);  // SWRESET
		waitBusy();
		sendCommand(0x01);  // driver output control
		sendData(0xC7);
		sendData(0x00);
		sendData(0x00);
		sendCommand(0x3C);  // border waveform
		sendData(0x00);
		sendCommand(0x18);  // internal temperature sensor
		sendData(0x80);
		setGrayRamArea();
		sendCommand(0x32);
		sendDataBytes(kLut4Gray, kLut4GrayBytes);
	}

	// Write a full RAM plane in gray mode. Resets the address counter to the
	// origin first so each plane starts at the top-left regardless of the
	// previous plane's write.
	void writeGrayPlane(uint8_t reg, const uint8_t *packed)
	{
		setGrayRamArea();
		sendCommand(reg);
		sendDataBytes(packed, kPackedBytes);
	}

	void turnOnDisplayGray()
	{
		sendCommand(0x22);
		sendData(0xc7);
		sendCommand(0x20);
		waitBusy();
	}

	bool initPanel()
	{
		// Power rail first: EPD power is ACTIVE LOW (vendor board_power_bsp).
		{
			gpio_config_t railConf = {};
			railConf.intr_type = GPIO_INTR_DISABLE;
			railConf.mode = GPIO_MODE_OUTPUT;
			railConf.pin_bit_mask = 1ULL << gea::platform::board::power.epdPower;
			railConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
			railConf.pull_up_en = GPIO_PULLUP_ENABLE;
			ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&railConf));
			gpio_set_level(gea::platform::board::power.epdPower, 0);
			vTaskDelay(pdMS_TO_TICKS(10));
		}

		// Control GPIOs: RST/DC/CS outputs, BUSY input.
		{
			gpio_config_t outConf = {};
			outConf.intr_type = GPIO_INTR_DISABLE;
			outConf.mode = GPIO_MODE_OUTPUT;
			outConf.pin_bit_mask = (1ULL << gea::platform::board::display.reset) |
			                       (1ULL << gea::platform::board::display.dc) |
			                       (1ULL << gea::platform::board::display.cs);
			outConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
			outConf.pull_up_en = GPIO_PULLUP_ENABLE;
			ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&outConf));

			gpio_config_t inConf = outConf;
			inConf.mode = GPIO_MODE_INPUT;
			inConf.pin_bit_mask = 1ULL << gea::platform::board::display.busy;
			ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&inConf));
			setRst(1);
			setCs(1);
		}

		spi_bus_config_t busConfig = {};
		busConfig.mosi_io_num = gea::platform::board::display.mosi;
		busConfig.miso_io_num = -1;
		busConfig.sclk_io_num = gea::platform::board::display.sclk;
		busConfig.quadwp_io_num = -1;
		busConfig.quadhd_io_num = -1;
		busConfig.max_transfer_sz = kPackedBytes;
		esp_err_t err = spi_bus_initialize(gea::platform::board::display.spiHost, &busConfig, SPI_DMA_CH_AUTO);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
			return false;
		}

		spi_device_interface_config_t devConfig = {};
		devConfig.spics_io_num = -1;  // CS driven manually around cmd/data phases
		devConfig.clock_speed_hz = kSpiClockHz;
		devConfig.mode = 0;
		devConfig.queue_size = 7;
		err = spi_bus_add_device(gea::platform::board::display.spiHost, &devConfig, &spi_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI device add failed: %s", esp_err_to_name(err));
			return false;
		}

		// Vendor bring-up: full init, white base image into BOTH RAM planes
		// (0x24 current + 0x26 previous — the partial waveform diffs against
		// the previous plane), one full refresh, then switch to the partial LUT.
		panelInitFull();
		std::memset(txPack_, 0xff, kPackedBytes);
		writeRam(0x24, txPack_);
		writeRam(0x26, txPack_);
		turnOnDisplayFull();
		panelInitPartial();
		partialsSinceFull_ = 0;
		ESP_LOGI(kTag, "SSD1681 panel ready cs=%d dc=%d rst=%d busy=%d clk=%dMHz",
		         static_cast<int>(gea::platform::board::display.cs),
		         static_cast<int>(gea::platform::board::display.dc),
		         static_cast<int>(gea::platform::board::display.reset),
		         static_cast<int>(gea::platform::board::display.busy),
		         kSpiClockHz / 1000000);
		return true;
	}

	// --- refresh worker ------------------------------------------------------

	static void refreshTaskEntry(void *arg) { static_cast<DisplayBackend *>(arg)->refreshLoop(); }

	void refreshLoop()
	{
		for (;;) {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			for (;;) {
				bool have = false;
				xSemaphoreTake(packMutex_, portMAX_DELAY);
				const uint32_t seq = pendingSeq_.load(std::memory_order_acquire);
				if (seq != transmittedSeq_) {
					std::memcpy(txPack_, pendingPack_, kPackedBytes);
					if (grayscaleEnabled_) std::memcpy(txPack26_, pendingPack26_, kPackedBytes);
					txGrayscale_ = grayscaleEnabled_;
					transmittedSeq_ = seq;
					have = true;
				}
				xSemaphoreGive(packMutex_);
				if (!have) break;
				refreshInFlight_.store(true, std::memory_order_release);
				transmitFrame();
				refreshInFlight_.store(false, std::memory_order_release);
			}
		}
	}

	enum class PartialLut : uint8_t { Standard, Fast, Invalid };
	enum class PanelMode : uint8_t { BlackWhite, Gray };

	// Return from gray mode to the B/W full+partial waveform pipeline. Gray
	// mode latched the gray LUT and a different RAM scan layout, so just reset
	// the B/W state and force a full refresh — the dueFull branch re-runs the
	// complete B/W init (panelInitFull re-establishes window/data-entry).
	void reinitBlackWhite()
	{
		panelMode_ = PanelMode::BlackWhite;
		partialsSinceFull_ = 0;
		loadedPartialLut_ = PartialLut::Invalid;
		forceFullRefresh_.store(true, std::memory_order_release);
	}

public:
	// Runtime refresh policy (Display.setEpaperRefreshConfig). Values < 0 keep
	// the current setting. LUT pointers: null = unchanged, len 0 = restore the
	// vendor waveform, len 159 = custom waveform (other lengths rejected).
	void setRefreshConfig(int fullEvery, int hardCap, int fastWindowMs,
	                      const uint8_t *partialLut, int partialLutLen,
	                      const uint8_t *fastLut, int fastLutLen)
	{
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		if (fullEvery >= 0) fullRefreshEveryPartials_ = fullEvery;
		if (hardCap >= 0) fullRefreshHardCapPartials_ = hardCap;
		if (fastWindowMs >= 0) fastStreakWindowUs_ = static_cast<std::int64_t>(fastWindowMs) * 1000;
		bool lutsChanged = false;
		lutsChanged |= applyCustomLut(partialLut, partialLutLen, customPartialLut_, hasCustomPartialLut_, "partialLut");
		lutsChanged |= applyCustomLut(fastLut, fastLutLen, customFastLut_, hasCustomFastLut_, "fastLut");
		// Force the worker to re-activate whatever waveform applies next.
		if (lutsChanged) loadedPartialLut_ = PartialLut::Invalid;
		xSemaphoreGive(packMutex_);
		ESP_LOGI(kTag,
		         "refresh config: fullEvery=%d hardCap=%d fastWindowMs=%lld customPartial=%d customFast=%d",
		         fullRefreshEveryPartials_, fullRefreshHardCapPartials_,
		         static_cast<long long>(fastStreakWindowUs_ / 1000),
		         hasCustomPartialLut_ ? 1 : 0, hasCustomFastLut_ ? 1 : 0);
	}

	// Force a full (anti-ghost) refresh of the current frame.
	void requestFullRefresh()
	{
		if (!init()) return;
		forceFullRefresh_.store(true, std::memory_order_release);
		requestRefresh();
	}

	// Toggle 4-level grayscale rendering. Gray mode renders the framebuffer's
	// luma into 4 reflectance levels via the SSD1681's two RAM planes + gray
	// waveform LUT; every update is a full ~1 s refresh (no partial path). B/W
	// mode (the default) keeps the fast partial waveform. The mode switch and a
	// re-pack of the current frame happen on the next refresh.
	void setGrayscale(bool enabled)
	{
		if (enabled && (!pendingPack26_ || !txPack26_)) {
			// Lazily allocate the 0x26 grayscale plane (DMA-capable internal) on the
			// first enable; B/W apps never pay this ~10 KB (see init()). Bail to B/W
			// if the DMA-internal pool can't spare it.
			if (!pendingPack26_)
				pendingPack26_ = static_cast<uint8_t *>(
					heap_caps_malloc(kPackedBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (!txPack26_)
				txPack26_ = static_cast<uint8_t *>(
					heap_caps_malloc(kPackedBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (!pendingPack26_ || !txPack26_) {
				ESP_LOGE(kTag, "grayscale plane alloc failed; staying in B/W");
				return;
			}
			std::memset(pendingPack26_, 0x00, kPackedBytes);
			std::memset(txPack26_, 0x00, kPackedBytes);
		}
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		const bool changed = grayscaleEnabled_ != enabled;
		grayscaleEnabled_ = enabled;
		xSemaphoreGive(packMutex_);
		ESP_LOGI(kTag, "grayscale %s", enabled ? "on" : "off");
		if (changed && initialized_) {
			forceFullRefresh_.store(true, std::memory_order_release);
			requestRefresh();
		}
	}

private:
	static bool applyCustomLut(const uint8_t *lut, int len, uint8_t *store, bool &hasCustom, const char *name)
	{
		if (!lut && len < 0) return false;  // absent — unchanged
		if (len == 0) {                     // explicit reset to vendor waveform
			if (!hasCustom) return false;
			hasCustom = false;
			return true;
		}
		if (len != kLutBytes || !lut) {
			ESP_LOGE(kTag, "%s must be exactly %d bytes (got %d) — ignored", name, kLutBytes, len);
			return false;
		}
		std::memcpy(store, lut, kLutBytes);
		hasCustom = true;
		return true;
	}

	const uint8_t *partialLutBytes() const { return hasCustomPartialLut_ ? customPartialLut_ : kLutPartial; }
	const uint8_t *fastLutBytes() const { return hasCustomFastLut_ ? customFastLut_ : kLutPartialFast; }

	// 4-level grayscale refresh: full-waveform every time (no partial path in
	// gray mode), ~1 s. Loads the gray LUT on first entry / after leaving B/W,
	// writes both RAM planes, then a single full update drives the 4 levels.
	void transmitGrayFrame(std::int64_t started)
	{
		forceFullRefresh_.exchange(false, std::memory_order_acq_rel);
		if (panelMode_ != PanelMode::Gray) {
			panelInitGray();
			panelMode_ = PanelMode::Gray;
			loadedPartialLut_ = PartialLut::Invalid;
		}
		writeGrayPlane(0x26, txPack26_);
		writeGrayPlane(0x24, txPack_);
		turnOnDisplayGray();
		partialsSinceFull_ = 0;
		lastTransmitEndUs_ = esp_timer_get_time();
		const std::int64_t tookUs = lastTransmitEndUs_ - started;
		flushStats_.txUs += tookUs;
		flushStats_.chunkCount++;
		flushStats_.pixelCount += kMaxPixels;
		ESP_LOGI(kTag, "gray refresh %d ms levels[blk=%u dark=%u light=%u wht=%u]",
		         static_cast<int>(tookUs / 1000),
		         static_cast<unsigned>(lastGrayLevelCounts_[0]),
		         static_cast<unsigned>(lastGrayLevelCounts_[1]),
		         static_cast<unsigned>(lastGrayLevelCounts_[2]),
		         static_cast<unsigned>(lastGrayLevelCounts_[3]));
	}

	void transmitFrame()
	{
		const std::int64_t started = esp_timer_get_time();
		// Snapshot the (frame-task-writable) policy under the pack mutex.
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		const int fullEvery = fullRefreshEveryPartials_;
		const int hardCap = fullRefreshHardCapPartials_;
		const std::int64_t fastWindowUs = fastStreakWindowUs_;
		const bool grayscale = txGrayscale_;
		xSemaphoreGive(packMutex_);

		if (grayscale) {
			transmitGrayFrame(started);
			return;
		}
		// Leaving gray mode: re-establish the B/W full+partial waveform path.
		if (panelMode_ != PanelMode::BlackWhite) {
			reinitBlackWhite();
		}

		const bool fastStreak = fastWindowUs > 0 && started - lastTransmitEndUs_ < fastWindowUs;
		const bool dueFull = forceFullRefresh_.exchange(false, std::memory_order_acq_rel) ||
		                     partialsSinceFull_ >= hardCap ||
		                     (!fastStreak && partialsSinceFull_ >= fullEvery);
		if (dueFull) {
			// Anti-ghosting flash: full-waveform refresh of the current frame,
			// re-seed both RAM planes, then back to the partial LUT.
			panelInitFull();
			writeRam(0x24, txPack_);
			turnOnDisplayFull();
			writeRam(0x24, txPack_);
			writeRam(0x26, txPack_);
			turnOnDisplayFull();
			hardwareReset();
			waitBusy();
			activatePartialLut(partialLutBytes());
			loadedPartialLut_ = PartialLut::Standard;
			partialsSinceFull_ = 0;
		} else {
			const PartialLut want = fastStreak ? PartialLut::Fast : PartialLut::Standard;
			if (want != loadedPartialLut_) {
				activatePartialLut(want == PartialLut::Fast ? fastLutBytes() : partialLutBytes());
				loadedPartialLut_ = want;
			}
			writeRam(0x24, txPack_);
			turnOnDisplayPartial();
			partialsSinceFull_++;
		}
		lastTransmitEndUs_ = esp_timer_get_time();
		const std::int64_t tookUs = lastTransmitEndUs_ - started;
		flushStats_.txUs += tookUs;
		flushStats_.chunkCount++;
		flushStats_.pixelCount += kMaxPixels;
		refreshLogAccumUs_ += tookUs;
		if (++refreshLogCount_ >= 16) {
			ESP_LOGI(kTag, "16 refreshes avg %d ms (lut=%s)",
			         static_cast<int>(refreshLogAccumUs_ / 16000),
			         loadedPartialLut_ == PartialLut::Fast ? "fast" : "standard");
			refreshLogAccumUs_ = 0;
			refreshLogCount_ = 0;
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
	SemaphoreHandle_t packMutex_ = nullptr;
	spi_device_handle_t spi_ = nullptr;
	std::uint16_t *frameBuffer_ = nullptr;
	uint8_t *pendingPack_ = nullptr;
	uint8_t *txPack_ = nullptr;
	uint8_t *pendingPack26_ = nullptr;
	uint8_t *txPack26_ = nullptr;
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	TaskHandle_t refreshTask_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	std::atomic<uint32_t> pendingSeq_{0};
	uint32_t transmittedSeq_ = 0;
	std::atomic<bool> refreshInFlight_{false};
	int partialsSinceFull_ = 0;
	PartialLut loadedPartialLut_ = PartialLut::Standard;
	PanelMode panelMode_ = PanelMode::BlackWhite;
	bool grayscaleEnabled_ = false;  // guarded by packMutex_
	bool txGrayscale_ = false;       // mode the snapshotted tx planes were packed in
	std::uint32_t lastGrayLevelCounts_[4] = {0, 0, 0, 0};
	std::int64_t lastTransmitEndUs_ = 0;
	std::int64_t refreshLogAccumUs_ = 0;
	int refreshLogCount_ = 0;
	// Runtime refresh policy (Display.setEpaperRefreshConfig); guarded by packMutex_.
	int fullRefreshEveryPartials_ = kFullRefreshEveryPartials;
	int fullRefreshHardCapPartials_ = kFullRefreshHardCapPartials;
	std::int64_t fastStreakWindowUs_ = kFastStreakWindowUs;
	uint8_t customPartialLut_[kLutBytes]{};
	uint8_t customFastLut_[kLutBytes]{};
	bool hasCustomPartialLut_ = false;
	bool hasCustomFastLut_ = false;
	std::atomic<bool> forceFullRefresh_{false};
	bool backgroundCacheAttempted_ = false;
	bool backdropCacheAttempted_ = false;
	bool initialized_ = false;
	bool needsFullPhysicalFlush_ = true;
	platform_display::DisplayFlushPerfStats flushStats_{};
	present::Frame previousPresentFrame_{};
	bool previousPresentValid_ = false;
};

}  // namespace gea::platform::esp32_s3_epaper::display

// Strong definitions for the weak hooks in lib/gea-embedded/host/display.cpp:
// the JS-facing Display.setEpaperRefreshConfig / Display.epaperFullRefresh.
extern "C" void gea_epaper_set_refresh_config(
	int full_refresh_every_partials,
	int full_refresh_hard_cap_partials,
	int fast_streak_window_ms,
	const std::uint8_t *partial_lut, int partial_lut_len,
	const std::uint8_t *fast_lut, int fast_lut_len)
{
	gea::platform::esp32_s3_epaper::display::DisplayBackend::instance().setRefreshConfig(
		full_refresh_every_partials, full_refresh_hard_cap_partials, fast_streak_window_ms,
		partial_lut, partial_lut_len, fast_lut, fast_lut_len);
}

extern "C" void gea_epaper_full_refresh(void)
{
	gea::platform::esp32_s3_epaper::display::DisplayBackend::instance().requestFullRefresh();
}

extern "C" void gea_epaper_set_grayscale(int enabled)
{
	gea::platform::esp32_s3_epaper::display::DisplayBackend::instance().setGrayscale(enabled != 0);
}

extern "C" std::uint16_t *gea_bg_cache(int *cap_px)
{
	return gea::platform::esp32_s3_epaper::display::DisplayBackend::instance().backgroundCache(cap_px);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	return gea::platform::esp32_s3_epaper::display::DisplayBackend::instance().backdropCache(cap_px);
}

namespace gea::platform::display {

namespace {

using Backend = gea::platform::esp32_s3_epaper::display::DisplayBackend;

gea::framework::graphics::Canvas *drawingCanvas()
{
	return Backend::instance().canvas();
}

}  // namespace

bool Display::init() { return Backend::instance().init(); }
bool Display::start() { return Backend::instance().start(); }
gea::framework::graphics::Canvas *Display::canvas() { return Backend::instance().canvas(); }
bool Display::framebufferIsPanelDirect() { return Backend::instance().framebufferIsPanelDirect(); }
void Display::clear() { Backend::instance().clear(); }
void Display::clearNoFlush() { Backend::instance().clearNoFlush(); }
void Display::print(const char *) {}
void Display::flush() { Backend::instance().flush(); }
void Display::flushRects(const DisplayFlushRect *rects, int count, bool) { Backend::instance().flushRects(rects, count); }
bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user) { return Backend::instance().streamRect(x, y, w, h, raster, user); }
bool Display::present(const DisplayPresentCommand *commands, int command_count) { return Backend::instance().presentCommands(commands, command_count); }
void Display::setFlushConfig(int chunk_rows, int queue_depth) { Backend::instance().setFlushConfig(chunk_rows, queue_depth); }
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
void Display::setBrightness(int brightness_percent) { Backend::instance().setBrightness(brightness_percent); }
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
	const auto stats = Backend::instance().flushStats();
	if (total_us) *total_us = stats.totalUs;
	if (call_count) *call_count = stats.callCount;
	if (pixel_count) *pixel_count = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead() { return Backend::instance().flushStats(); }
void Display::flushStatsReset() { Backend::instance().flushStatsReset(); }
const char *Display::flushStageName() { return Backend::instance().flushStageName(); }
int Display::flushStageChunk() { return Backend::instance().flushStageChunk(); }
DisplayFlushStageDetail Display::flushStageDetail() { return Backend::instance().flushStageDetail(); }
bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixel_capacity, int *width, int *height, bool) { return Backend::instance().copySnapshotRgb565(dst, pixel_capacity, width, height); }
int Display::countNonBlackPixels(bool) { return Backend::instance().countNonBlackPixels(); }

void applyOrientation(gea::framework::display::DisplayOrientation)
{
	Backend::instance().applyOrientation();
}

}  // namespace gea::platform::display
