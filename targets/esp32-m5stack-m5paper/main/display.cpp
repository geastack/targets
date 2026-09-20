#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "host/display_orientation.h"
#include "pixel.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

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

namespace gea::platform::m5paper::display {

namespace {

constexpr char kTag[] = "m5paper_display";
constexpr int kControllerWidth = 960;
constexpr int kControllerHeight = 540;
constexpr int kMaxPixels = kControllerWidth * kControllerHeight;
constexpr int kMaxPackedBytes = kMaxPixels / 2;  // 4 bits per pixel
// M5GFX caps the M5Paper IT8951 write clock at 12 MHz. The older M5EPD
// library used 10 MHz, but 12 MHz is the controller-tested fast path.
// 12 MHz is this board's ceiling: 16 and 20 MHz were both tried for faster
// partial transfers (a 540x856 partial costs ~500 ms of pixel push) and the
// IT8951 failed init with a BUSY timeout at either speed — the M5Paper's SPI
// link, not the controller spec (24 MHz), is the limit. Don't raise this.
// 12 MHz is the reliable ceiling for this IT8951 wiring: 20 MHz hard-failed
// (BUSY timeout at init, controller never came up). The page-turn win comes
// from the interrupt-driven bulk transfer, not the clock.
constexpr int kSpiClockHz = 12 * 1000 * 1000;
constexpr int kDmaChunkBytes = 4096;
// One IT8951 waveform at a time is the reliable presentation unit. Multiple
// disjoint commands make a nominal UI frame take N complete panel cycles and
// allow the game to advance several states before the glass catches up. Keep
// a single union region containing every change since the in-flight frame.
constexpr int kMaxBatchRegions = 1;
constexpr int kDefaultFullRefreshEvery = 160;
constexpr int kDefaultFullRefreshHardCap = 600;
constexpr std::uint32_t kFallbackImageMemory = 0x001236e0;

constexpr std::uint16_t kCmdSystemRun = 0x0001;
constexpr std::uint16_t kCmdRegisterRead = 0x0010;
constexpr std::uint16_t kCmdRegisterWrite = 0x0011;
constexpr std::uint16_t kCmdLoadImageArea = 0x0021;
constexpr std::uint16_t kCmdLoadImageEnd = 0x0022;
constexpr std::uint16_t kCmdGetDeviceInfo = 0x0302;
constexpr std::uint16_t kCmdDisplayBufferArea = 0x0037;
constexpr std::uint16_t kCmdVcom = 0x0039;
constexpr std::uint16_t kRegisterPackedWrite = 0x0004;
constexpr std::uint16_t kRegisterImageAddress = 0x0208;
constexpr std::uint16_t kRegisterLutActive = 0x1224;

enum UpdateMode : std::uint16_t {
	UpdateInit = 0,
	UpdateDirect = 1,
	UpdateGc16 = 2,
	UpdateGl16 = 3,
	UpdateDu4 = 6,
	UpdateA2 = 7,
};

struct Region {
	int x = 0;
	int y = 0;
	int w = 0;
	int h = 0;
	int rotation = 1;
	bool full = false;
	bool qualityGrayscale = false;
	// Part of a rapid-update streak (scroll/animation): displayed with the fast
	// 1-bit A2 waveform instead of GL16 — 16-level partials arriving faster than
	// the ~450 ms grayscale waveform read as flicker. A quality re-display of
	// the streak's union region runs once the streak goes quiet.
	bool fastStreak = false;
	int packedBytes = 0;
};

struct RefreshBatch {
	std::array<Region, kMaxBatchRegions> regions = {};
	int count = 0;
	int pixelCount = 0;
	bool full = false;
};

// GRAY4: the framebuffer already holds 16-level gray in the panel's own 4bpp
// packing, so there is NO flush-time pack/copy stage at all — the refresh task
// streams region rows straight from the live framebuffer into the SPI DMA
// chunk (A2 partials snap each nibble to the two endpoints on the way in).

int rotationFor(gea::framework::display::DisplayOrientation orientation)
{
	using Orientation = gea::framework::display::DisplayOrientation;
	switch (orientation) {
	case Orientation::PortraitSecondary: return 3;
	case Orientation::LandscapePrimary: return 0;
	case Orientation::LandscapeSecondary: return 2;
	case Orientation::PortraitPrimary:
	default: return 1;
	}
}

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
		if (!mutex_ || !packMutex_) return false;

		frameBuffer_ = static_cast<std::uint8_t *>(
			heap_caps_calloc(kMaxPackedBytes, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		dmaChunk_ = static_cast<std::uint8_t *>(
			heap_caps_malloc(kDmaChunkBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
		dmaChunkB_ = static_cast<std::uint8_t *>(
			heap_caps_malloc(kDmaChunkBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
		if (!frameBuffer_ || !dmaChunk_ || !dmaChunkB_) {
			ESP_LOGE(kTag, "framebuffer/transfer allocation failed (PSRAM required)");
			return false;
		}

		if (!initPanel()) return false;
		bindCanvas();
		if (xTaskCreatePinnedToCore(refreshTaskEntry, "m5paper_epd", 4096, this, 6, &refreshTask_, 1) != pdPASS) {
			ESP_LOGE(kTag, "failed to start refresh task");
			return false;
		}
		initialized_ = true;
		ESP_LOGI(kTag, "M5Paper ready 540x960, 4/16 gray, partial update, packed=%d bytes", kMaxPackedBytes);
		return true;
	}

	bool start() { return init(); }

	gea::framework::graphics::Canvas *canvas()
	{
		if (!initialized_) init();
		return &canvas_;
	}

	bool framebufferIsPanelDirect() const { return false; }

	void applyOrientation(gea::framework::display::DisplayOrientation orientation)
	{
		if (!init()) return;
		take();
		rotation_ = rotationFor(orientation);
		bindCanvasLocked();
		std::memset(frameBuffer_, 0, kMaxPackedBytes);
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		needsFullPhysicalFlush_ = true;
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
		std::memset(frameBuffer_, 0, kMaxPackedBytes);
		canvas_.markDirty(0, 0, logicalWidth() - 1, logicalHeight() - 1);
		needsFullPhysicalFlush_ = true;
		give();
	}

	void flush()
	{
		if (!init()) return;
		int x0 = 0;
		int y0 = 0;
		int x1 = logicalWidth() - 1;
		int y1 = logicalHeight() - 1;
		if (!needsFullPhysicalFlush_ && !canvas_.dirty(&x0, &y0, &x1, &y1)) return;
		requestRefresh(x0, y0, x1, y1, needsFullPhysicalFlush_);
		canvas_.resetDirty();
		needsFullPhysicalFlush_ = false;
	}

	void flushRects(const platform_display::DisplayFlushRect *rects, int count)
	{
		if (!rects || count <= 0 || !init()) return;
		requestRefreshRects(rects, count, needsFullPhysicalFlush_);
		canvas_.resetDirty();
		needsFullPhysicalFlush_ = false;
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
		const int rowBytes = (logicalWidth() + 1) / 2;
		// The raster callback fills an unpacked native-pixel (gray-value) row; pack
		// it into the framebuffer's nibbles per row.
		static pixel::native_t rowbuf[kControllerWidth];
		for (int row = y0; row <= y1; ++row) {
			raster(rowbuf, width, 1, x0, row, user);
			pixel::packed::writeUnpacked(frameBuffer_ + static_cast<std::size_t>(row) * rowBytes, x0, rowbuf, width);
		}
		canvas_.markDirty(x0, y0, x1, y1);
		flush();
		return true;
	}

	bool presentCommands(const platform_display::DisplayPresentCommand *commands, int commandCount)
	{
		// The strided direct-present fast path assumes an unpacked native
		// framebuffer; GRAY4 storage is nibble-packed, so decline and let the
		// framework fall back to normal Canvas replay (which packs correctly).
		(void)commands;
		(void)commandCount;
		return false;
	}

	void setFlushConfig(int, int) {}
	int flushChunkRows() const { return logicalHeight(); }
	int flushQueueDepth() const { return 1; }
	int flushBufferBytes() const { return kMaxPackedBytes; }

	bool copySnapshotRgb565(std::uint16_t *dst, int capacity, int *width, int *height)
	{
		if (!dst || capacity < kMaxPixels || !frameBuffer_) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		// Framebuffer is 4bpp packed; unpack each nibble to an RGB565 gray.
		for (int i = 0; i < kMaxPixels; ++i)
			dst[i] = pixel::gray4ToRgb565(pixel::packed::get(frameBuffer_, i));
		return true;
	}

	// Packed (native GRAY4) snapshot -- frameBuffer_ IS already a flat packed
	// pixel stream over the whole logical image (kMaxPackedBytes = kMaxPixels/2),
	// so this is a straight copy: no per-pixel unpack/expand to RGB565, and a
	// quarter the allocation copySnapshotRgb565 needs.
	bool copySnapshotPacked(std::uint8_t *dst, int byteCapacity, int *width, int *height)
	{
		if (!dst || byteCapacity < kMaxPackedBytes || !frameBuffer_) return false;
		if (width) *width = logicalWidth();
		if (height) *height = logicalHeight();
		std::memcpy(dst, frameBuffer_, static_cast<std::size_t>(kMaxPackedBytes));
		return true;
	}

	int countNonBlackPixels() const
	{
		int count = 0;
		for (int i = 0; frameBuffer_ && i < kMaxPixels; ++i)
			if (pixel::packed::get(frameBuffer_, i) != 0) ++count;
		return count;
	}

	void setBrightness(int) {}
	int brightness() const { return 100; }
	platform_display::DisplayFlushPerfStats flushStats() const { return stats_; }
	void flushStatsReset() { stats_ = {}; }
	const char *flushStageName() const { return refreshInFlight_.load() ? "it8951-refresh" : "idle"; }
	int flushStageChunk() const { return 0; }
	platform_display::DisplayFlushStageDetail flushStageDetail() const { return {}; }

	std::uint16_t *backgroundCache(int *capacity) { return scratch(backgroundCache_, backgroundAttempted_, capacity); }
	std::uint16_t *backdropCache(int *capacity) { return scratch(backdropCache_, backdropAttempted_, capacity); }

	void setRefreshConfig(int fullEvery, int hardCap, int fastStreakWindowMs,
	                      const std::uint8_t *, int, const std::uint8_t *, int)
	{
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		if (fullEvery >= 0) fullRefreshEvery_ = fullEvery;
		if (hardCap >= 0) fullRefreshHardCap_ = hardCap;
		if (fastStreakWindowMs >= 0) fastStreakWindowUs_ = fastStreakWindowMs * 1000;
		xSemaphoreGive(packMutex_);
		ESP_LOGI(kTag, "refresh config fullEvery=%d hardCap=%d fastStreakMs=%d (IT8951 waveforms are controller-resident)",
		         fullRefreshEvery_, fullRefreshHardCap_, fastStreakWindowUs_ / 1000);
	}

	void requestFullRefresh()
	{
		if (!init()) return;
		forceFullRefresh_.store(true);
		requestRefresh(0, 0, logicalWidth() - 1, logicalHeight() - 1, true);
	}

	void setQualityGrayscale(bool enabled)
	{
		qualityGrayscale_.store(enabled);
		forceFullRefresh_.store(true);
		if (initialized_) requestRefresh(0, 0, logicalWidth() - 1, logicalHeight() - 1, true);
		ESP_LOGI(kTag, "grayscale mode=%s", enabled ? "16-level quality" : "4-level fast");
	}

	void setFullOnCover(bool enabled) { fullOnCover_.store(enabled); }

	void rebindCanvasToFramebuffer() { bindCanvas(); }

private:
	DisplayBackend() = default;

	int logicalWidth() const { return orientation_detail::DisplayOrientationState::width(); }
	int logicalHeight() const { return orientation_detail::DisplayOrientationState::height(); }

	void take() { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
	void give() { if (mutex_) xSemaphoreGive(mutex_); }

	void bindCanvas()
	{
		take();
		bindCanvasLocked();
		give();
	}

	void bindCanvasLocked()
	{
		canvas_.bindPixels(frameBuffer_, logicalWidth(), logicalHeight(), logicalWidth());
	}

	std::uint16_t *scratch(std::uint16_t *&buffer, bool &attempted, int *capacity)
	{
		if (!attempted) {
			attempted = true;
			buffer = static_cast<std::uint16_t *>(
				heap_caps_malloc(kMaxPixels * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		}
		if (capacity) *capacity = buffer ? kMaxPixels : 0;
		return buffer;
	}

	Region alignedRegion(int x0, int y0, int x1, int y1, bool full)
	{
		const int width = logicalWidth();
		const int height = logicalHeight();
		x0 = std::clamp(x0, 0, width - 1);
		y0 = std::clamp(y0, 0, height - 1);
		x1 = std::clamp(x1, 0, width - 1);
		y1 = std::clamp(y1, 0, height - 1);
		if (x1 < x0 || y1 < y0) return {};
		// IT8951 4bpp loads require the controller X axis to be 4-pixel
		// aligned. In portrait rotations logical Y maps to controller X, so
		// align and expand both logical axes before packing. Aligning only X
		// allowed small UI updates such as the FPS badge at y=927 to submit an
		// unaligned physical-X area and eventually wedge panel updates.
		x0 &= ~3;
		x1 = std::min(width - 1, ((x1 + 4) & ~3) - 1);
		y0 &= ~3;
		y1 = std::min(height - 1, ((y1 + 4) & ~3) - 1);
		Region region;
		region.x = x0;
		region.y = y0;
		region.w = x1 - x0 + 1;
		region.h = y1 - y0 + 1;
		region.rotation = rotation_;
		region.full = full;
		region.qualityGrayscale = qualityGrayscale_.load();
		region.packedBytes = region.w * region.h / 2;
		return region;
	}

	bool waitForWaveform(const char *reason, std::int64_t &waitUs)
	{
		const std::int64_t started = esp_timer_get_time();
		const bool ok = waitDisplay();
		waitUs += esp_timer_get_time() - started;
		if (!ok) {
			ESP_LOGE(kTag, "IT8951 waveform wait failed reason=%s",
			         reason ? reason : "unknown");
			return false;
		}
		return true;
	}

	bool addRegion(RefreshBatch &batch, const Region &region)
	{
		if (region.packedBytes <= 0) return true;
		if (batch.count >= kMaxBatchRegions) return false;
		batch.regions[batch.count++] = region;
		return true;
	}

	void requestRefresh(int x0, int y0, int x1, int y1, bool requestedFull)
	{
		const platform_display::DisplayFlushRect rect = {x0, y0, x1, y1};
		requestRefreshRects(&rect, 1, requestedFull);
	}

	void requestRefreshRects(const platform_display::DisplayFlushRect *rects, int count, bool requestedFull)
	{
		if (!rects || count <= 0) return;
		const std::int64_t started = esp_timer_get_time();
		xSemaphoreTake(packMutex_, portMAX_DELAY);
		const int partials = partialsSinceFull_.load();
		const bool hasPending = pendingSeq_.load(std::memory_order_acquire) != transmittedSeq_;
		bool coversScreen = false;
		for (int i = 0; i < count && !coversScreen; ++i) {
			coversScreen = rects[i].x0 <= 0 && rects[i].y0 <= 0 &&
			               rects[i].x1 >= logicalWidth() - 1 && rects[i].y1 >= logicalHeight() - 1;
		}
		// Rapid-succession requests (a scroll) are a streak: they ride the fast
		// 1-bit waveform AND suppress the full-refresh promotions — a GC16 flash
		// mid-scroll is exactly the visual we're avoiding; the post-streak GL16
		// settle (and the next isolated refresh's cadence) recover quality.
		const std::int64_t now = esp_timer_get_time();
		const bool streak = fastStreakWindowUs_ > 0 && lastRequestUs_ > 0 &&
		                    now - lastRequestUs_ <= fastStreakWindowUs_;
		lastRequestUs_ = now;
		bool full = requestedFull || (hasPending && pendingBatch_.full) || forceFullRefresh_.exchange(false) ||
		            (!streak && ((coversScreen && fullOnCover_.load()) ||
		                         (fullRefreshEvery_ > 0 && partials >= fullRefreshEvery_) ||
		                         (fullRefreshHardCap_ > 0 && partials >= fullRefreshHardCap_)));
		RefreshBatch batch = {};
		if (full) {
			Region region = alignedRegion(0, 0, logicalWidth() - 1, logicalHeight() - 1, true);
			addRegion(batch, region);
			batch.full = true;
		} else {
			int unionX0 = logicalWidth();
			int unionY0 = logicalHeight();
			int unionX1 = -1;
			int unionY1 = -1;
			if (hasPending) {
				for (int i = 0; i < pendingBatch_.count; ++i) {
					const Region &region = pendingBatch_.regions[i];
					unionX0 = std::min(unionX0, region.x);
					unionY0 = std::min(unionY0, region.y);
					unionX1 = std::max(unionX1, region.x + region.w - 1);
					unionY1 = std::max(unionY1, region.y + region.h - 1);
				}
			}
			for (int i = 0; i < count; ++i) {
				Region region = alignedRegion(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1, false);
				if (region.packedBytes <= 0) continue;
				unionX0 = std::min(unionX0, region.x);
				unionY0 = std::min(unionY0, region.y);
				unionX1 = std::max(unionX1, region.x + region.w - 1);
				unionY1 = std::max(unionY1, region.y + region.h - 1);
			}
			if (unionX1 >= unionX0 && unionY1 >= unionY0) {
				addRegion(batch, alignedRegion(unionX0, unionY0, unionX1, unionY1, false));
			}
		}
		if (batch.count > 0) {
			if (!batch.full && streak) {
				for (int i = 0; i < batch.count; ++i) batch.regions[i].fastStreak = true;
			}
			batch.pixelCount = 0;
			for (int i = 0; i < batch.count; ++i)
				batch.pixelCount += batch.regions[i].w * batch.regions[i].h;
			pendingBatch_ = batch;
			pendingSeq_.fetch_add(1, std::memory_order_release);
		}
		xSemaphoreGive(packMutex_);
		if (refreshTask_) xTaskNotifyGive(refreshTask_);

		stats_.callCount++;
		stats_.pixelCount += batch.pixelCount;
		stats_.totalUs += esp_timer_get_time() - started;
	}

	bool waitReady(std::uint32_t timeoutMs = 3000)
	{
		const std::int64_t deadline = esp_timer_get_time() + static_cast<std::int64_t>(timeoutMs) * 1000;
		while (gpio_get_level(gea::platform::board::display.busy) == 0) {
			if (esp_timer_get_time() >= deadline) {
				ESP_LOGE(kTag, "IT8951 BUSY timeout");
				return false;
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		return true;
	}

	bool transfer(const void *tx, void *rx, int bytes)
	{
		spi_transaction_t transaction = {};
		transaction.length = static_cast<std::size_t>(bytes) * 8;
		transaction.tx_buffer = tx;
		transaction.rx_buffer = rx;
		return spi_device_polling_transmit(spi_, &transaction) == ESP_OK;
	}

	bool writeCommand(std::uint16_t command)
	{
		if (!waitReady()) return false;
		const std::uint8_t prefix[2] = {0x60, 0x00};
		const std::uint8_t word[2] = {static_cast<std::uint8_t>(command >> 8), static_cast<std::uint8_t>(command)};
		gpio_set_level(gea::platform::board::display.cs, 0);
		bool ok = transfer(prefix, nullptr, sizeof(prefix));
		if (ok) ok = waitReady();
		if (ok) ok = transfer(word, nullptr, sizeof(word));
		gpio_set_level(gea::platform::board::display.cs, 1);
		return ok;
	}

	bool writeWord(std::uint16_t value)
	{
		if (!waitReady()) return false;
		const std::uint8_t prefix[2] = {0x00, 0x00};
		const std::uint8_t word[2] = {static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value)};
		gpio_set_level(gea::platform::board::display.cs, 0);
		bool ok = transfer(prefix, nullptr, sizeof(prefix));
		if (ok) ok = waitReady();
		if (ok) ok = transfer(word, nullptr, sizeof(word));
		gpio_set_level(gea::platform::board::display.cs, 1);
		return ok;
	}

	bool writeArgs(std::uint16_t command, const std::uint16_t *args, int count)
	{
		if (!args || count < 0 || count > 20 || !writeCommand(command)) return false;
		for (int i = 0; i < count; ++i)
			if (!writeWord(args[i])) return false;
		return true;
	}

	bool readWords(std::uint16_t *values, int count)
	{
		if (!values || count <= 0 || count > 20 || !waitReady()) return false;
		const std::uint8_t prefix[2] = {0x10, 0x00};
		const std::uint8_t dummy[2] = {};
		std::uint8_t zeros[40] = {};
		std::uint8_t received[40] = {};
		gpio_set_level(gea::platform::board::display.cs, 0);
		bool ok = transfer(prefix, nullptr, sizeof(prefix));
		if (ok) ok = waitReady();
		if (ok) ok = transfer(dummy, nullptr, sizeof(dummy));
		if (ok) ok = waitReady();
		if (ok) ok = transfer(zeros, received, count * 2);
		gpio_set_level(gea::platform::board::display.cs, 1);
		if (!ok) return false;
		for (int i = 0; i < count; ++i)
			values[i] = static_cast<std::uint16_t>((received[i * 2] << 8) | received[1 + i * 2]);
		return true;
	}

	bool writeRegister(std::uint16_t address, std::uint16_t value)
	{
		return writeCommand(kCmdRegisterWrite) && writeWord(address) && writeWord(value);
	}

	bool setImageAddress()
	{
		return writeRegister(kRegisterImageAddress + 2, static_cast<std::uint16_t>(imageMemory_ >> 16)) &&
		       writeRegister(kRegisterImageAddress, static_cast<std::uint16_t>(imageMemory_));
	}

	bool waitDisplay()
	{
		const std::int64_t deadline = esp_timer_get_time() + 5000000;
		for (;;) {
			std::uint16_t active = 1;
			if (!writeCommand(kCmdRegisterRead) || !writeWord(kRegisterLutActive) || !readWords(&active, 1)) return false;
			if (active == 0) return true;
			if (esp_timer_get_time() >= deadline) return false;
			vTaskDelay(pdMS_TO_TICKS(2));
		}
	}

	// Stream a region's pixel rows from the LIVE framebuffer into the controller.
	// The framebuffer already holds the panel's 4bpp layout (region.x/w are 4-px
	// aligned, so the row slice is byte-aligned) — rows are copied straight into
	// the internal-RAM DMA chunk, no intermediate pack buffer. Full/GL16 loads
	// keep all 16 levels; A2 partials need strict two-endpoint data, so those
	// snap each nibble to 0/15 during the chunk fill.
	bool writeRegionPixels(const Region &region)
	{
		if (!waitReady()) return false;
		const std::uint8_t prefix[2] = {0x00, 0x00};
		gpio_set_level(gea::platform::board::display.cs, 0);
		bool ok = transfer(prefix, nullptr, sizeof(prefix));
		const int rowBytes = (logicalWidth() + 1) / 2;
		const int regionBytes = region.w / 2;
		const bool quality = region.full || (region.qualityGrayscale && !region.fastStreak);
		// Double-buffered, INTERRUPT-driven bulk transfer: while one chunk DMAs,
		// this task fills the other and then BLOCKS in get_trans_result. A
		// polling transmit here busy-spun this priority-6 task for the whole
		// ~200 ms load and starved the frame task on the same core — the visible
		// page-turn stall, not the wire time itself.
		std::uint8_t *bufs[2] = {dmaChunk_, dmaChunkB_};
		spi_transaction_t txn = {};
		int cur = 0;
		int fill = 0;
		bool inFlight = false;
		const auto awaitInFlight = [&]() {
			if (!inFlight) return;
			spi_transaction_t *done = nullptr;
			if (spi_device_get_trans_result(spi_, &done, portMAX_DELAY) != ESP_OK) ok = false;
			inFlight = false;
		};
		const auto queueChunk = [&](int bytes) {
			awaitInFlight();
			if (!ok || bytes <= 0) return;
			txn = {};
			txn.length = static_cast<std::size_t>(bytes) * 8;
			txn.tx_buffer = bufs[cur];
			if (spi_device_queue_trans(spi_, &txn, portMAX_DELAY) != ESP_OK) {
				ok = false;
				return;
			}
			inFlight = true;
			cur ^= 1;
		};
		for (int row = 0; ok && row < region.h; ++row) {
			const std::uint8_t *src = frameBuffer_ + static_cast<std::size_t>(region.y + row) * rowBytes + region.x / 2;
			int copied = 0;
			while (ok && copied < regionBytes) {
				const int take = std::min(kDmaChunkBytes - fill, regionBytes - copied);
				if (quality) {
					std::memcpy(bufs[cur] + fill, src + copied, static_cast<std::size_t>(take));
				} else {
					for (int b = 0; b < take; ++b) {
						const std::uint8_t byte = src[copied + b];
						const std::uint8_t hi = (byte >> 4) < 8 ? 0 : 15;
						const std::uint8_t lo = (byte & 0x0F) < 8 ? 0 : 15;
						bufs[cur][fill + b] = static_cast<std::uint8_t>((hi << 4) | lo);
					}
				}
				fill += take;
				copied += take;
				if (fill == kDmaChunkBytes) {
					queueChunk(fill);
					fill = 0;
				}
			}
		}
		if (ok && fill > 0) queueChunk(fill);
		awaitInFlight();
		gpio_set_level(gea::platform::board::display.cs, 1);
		return ok;
	}

	bool loadRegion(const Region &region)
	{
		const std::uint16_t args[5] = {
			static_cast<std::uint16_t>((1 << 8) | (2 << 4) | region.rotation),
			static_cast<std::uint16_t>(region.x),
			static_cast<std::uint16_t>(region.y),
			static_cast<std::uint16_t>(region.w),
			static_cast<std::uint16_t>(region.h),
		};
		return setImageAddress() && writeArgs(kCmdLoadImageArea, args, 5) &&
		       writeRegionPixels(region) && writeCommand(kCmdLoadImageEnd);
	}

	bool updateRegion(const Region &region, UpdateMode mode)
	{
		int x = region.x;
		int y = region.y;
		int w = region.w;
		int h = region.h;
		std::uint16_t args[7] = {};
		switch (region.rotation) {
		case 0: args[0] = x; args[1] = y; args[2] = w; args[3] = h; break;
		case 1: args[0] = y; args[1] = kControllerHeight - w - x; args[2] = h; args[3] = w; break;
		case 2: args[0] = kControllerWidth - w - x; args[1] = kControllerHeight - h - y; args[2] = w; args[3] = h; break;
		case 3: args[0] = kControllerWidth - h - y; args[1] = x; args[2] = h; args[3] = w; break;
		default: return false;
		}
		// IT8951 can freeze when a physical update's left and right edges are
		// inside the same 4-pixel controller group. Logical X alignment is not
		// sufficient in portrait mode because logical Y becomes physical X.
		// Expand only the displayed area; adjacent image-memory pixels retain
		// their previous contents.
		int left = args[0];
		int right = left + args[2] - 1;
		if ((left & ~3) == (right & ~3)) {
			if (right + 1 < kControllerWidth) {
				right = std::min(kControllerWidth - 1, (right + 4) & ~3);
			} else if (left > 0) {
				--left;
			}
			args[0] = static_cast<std::uint16_t>(left);
			args[2] = static_cast<std::uint16_t>(right - left + 1);
		}
		args[4] = mode;
		args[5] = static_cast<std::uint16_t>(imageMemory_);
		args[6] = static_cast<std::uint16_t>(imageMemory_ >> 16);
		return writeArgs(kCmdDisplayBufferArea, args, 7);
	}

	bool initPanel()
	{
		// USB/DTR resets restart the ESP32 without removing power from the IT8951.
		// Power-cycle the EPD rail so a controller left busy by the previous app
		// always returns to its command-ready reset state.
		gpio_set_level(gea::platform::board::power.epdPower, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(gea::platform::board::power.epdPower, 1);
		vTaskDelay(pdMS_TO_TICKS(500));

		gpio_config_t csConfig = {};
		csConfig.pin_bit_mask = 1ULL << gea::platform::board::display.cs;
		csConfig.mode = GPIO_MODE_OUTPUT;
		csConfig.pull_up_en = GPIO_PULLUP_ENABLE;
		csConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
		csConfig.intr_type = GPIO_INTR_DISABLE;
		ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&csConfig));
		gpio_set_level(gea::platform::board::display.cs, 1);

		gpio_config_t busyConfig = {};
		busyConfig.pin_bit_mask = 1ULL << gea::platform::board::display.busy;
		busyConfig.mode = GPIO_MODE_INPUT;
		busyConfig.pull_up_en = GPIO_PULLUP_ENABLE;
		busyConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
		busyConfig.intr_type = GPIO_INTR_DISABLE;
		ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&busyConfig));

		spi_bus_config_t bus = {};
		bus.mosi_io_num = gea::platform::board::display.mosi;
		bus.miso_io_num = gea::platform::board::display.miso;
		bus.sclk_io_num = gea::platform::board::display.sclk;
		bus.quadwp_io_num = -1;
		bus.quadhd_io_num = -1;
		bus.max_transfer_sz = kDmaChunkBytes;
		esp_err_t err = spi_bus_initialize(gea::platform::board::display.spiHost, &bus, SPI_DMA_CH_AUTO);
		if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
			ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
			return false;
		}

		spi_device_interface_config_t device = {};
		device.clock_speed_hz = kSpiClockHz;
		device.mode = 0;
		device.spics_io_num = -1;
		device.queue_size = 1;
		err = spi_bus_add_device(gea::platform::board::display.spiHost, &device, &spi_);
		if (err != ESP_OK) return false;

		ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_acquire_bus(spi_, portMAX_DELAY));
		std::uint16_t info[20] = {};
		if (writeCommand(kCmdGetDeviceInfo)) {
			vTaskDelay(pdMS_TO_TICKS(1));
			if (readWords(info, 20)) {
				const std::uint32_t reported = (static_cast<std::uint32_t>(info[3]) << 16) | info[2];
				if (reported != 0 && reported != 0xffffffffu) imageMemory_ = reported;
			}
		}
		const bool ok = writeCommand(kCmdSystemRun) &&
		                writeRegister(kRegisterPackedWrite, 1) && setImageAddress() &&
		                writeCommand(kCmdVcom) && writeWord(1) && writeWord(2300);
		spi_device_release_bus(spi_);
		if (!ok) return false;
		ESP_LOGI(kTag, "IT8951 info=%ux%u image=0x%08lx CS=%d BUSY=%d SPI=%dMHz",
		         static_cast<unsigned>(info[0]), static_cast<unsigned>(info[1]),
		         static_cast<unsigned long>(imageMemory_), static_cast<int>(gea::platform::board::display.cs),
		         static_cast<int>(gea::platform::board::display.busy), kSpiClockHz / 1000000);
		return true;
	}

	static void refreshTaskEntry(void *arg) { static_cast<DisplayBackend *>(arg)->refreshLoop(); }

	void refreshLoop()
	{
		for (;;) {
			const TickType_t waitTicks = (settlePending_ && fastStreakWindowUs_ > 0)
			                                 ? pdMS_TO_TICKS(fastStreakWindowUs_ / 1000 + 50)
			                                 : portMAX_DELAY;
			if (ulTaskNotifyTake(pdTRUE, waitTicks) == 0) {
				// Streak went quiet: one GL16 re-display of the union restores the
				// 16-level rendering the 1-bit scroll waveform flattened.
				if (settlePending_ && qualityGrayscale_.load() &&
				    pendingSeq_.load(std::memory_order_acquire) == transmittedSeq_) {
					Region region = alignedRegion(settleX0_, settleY0_, settleX1_, settleY1_, false);
					settlePending_ = false;
					settleX1_ = -1;
					if (region.packedBytes > 0) {
						refreshInFlight_.store(true);
						spi_device_acquire_bus(spi_, portMAX_DELAY);
						bool ok = true;
						std::int64_t waitUs = 0;
						if (waveformPending_) {
							ok = waitForWaveform("streak-settle", waitUs);
							waveformPending_ = false;
						}
						if (ok) {
							ok = loadRegion(region) && updateRegion(region, UpdateGl16);
							if (ok) waveformPending_ = true;
						}
						spi_device_release_bus(spi_);
						if (ok)
							ESP_LOGI(kTag, "streak settle %dx%d@%d,%d", region.w, region.h, region.x, region.y);
						refreshInFlight_.store(false);
					}
				}
				continue;
			}
			for (;;) {
				RefreshBatch batch;
				xSemaphoreTake(packMutex_, portMAX_DELAY);
				const std::uint32_t seq = pendingSeq_.load(std::memory_order_acquire);
				if (seq == transmittedSeq_) {
					xSemaphoreGive(packMutex_);
					break;
				}
				batch = pendingBatch_;
				transmittedSeq_ = seq;
				xSemaphoreGive(packMutex_);

				refreshInFlight_.store(true);
				const std::int64_t started = esp_timer_get_time();
				spi_device_acquire_bus(spi_, portMAX_DELAY);
				bool ok = batch.count == 1;
				std::int64_t waveformWaitUs = 0;
				const Region &region = batch.regions[0];
				// A2 is the controller's fast, non-flashing waveform specifically for
				// repeated black/white animation. DU is intended for isolated direct
				// changes and accumulated a visible gray bias during continuous game
				// updates on this panel. Full recovery stays on GC16, and GL16 remains
				// available for explicitly requested grayscale content.
				const UpdateMode mode = region.full ? UpdateGc16 :
				                            ((region.qualityGrayscale && !region.fastStreak) ? UpdateGl16 : UpdateA2);
				// Wait for the PREVIOUS update's waveform lazily, right before this
				// load needs the controller idle — not after our own update. The
				// waveform (~300 ms for a large A2 partial) then runs while the main
				// task rasters the next frame instead of serializing behind the SPI
				// transfer, which cuts a back-to-back scroll cycle by that much.
				if (ok && waveformPending_) {
					ok = waitForWaveform("previous", waveformWaitUs);
					waveformPending_ = false;
				}
				if (ok) {
					// Single-shot load+display: the whole region transitions as ONE
					// waveform. A split top/bottom pipelined load was tried (top half
					// inks ~100 ms sooner) and reverted — the two out-of-phase half
					// waveforms read as tearing on a page turn.
					ok = loadRegion(region) && updateRegion(region, mode);
					if (ok) waveformPending_ = true;
				}
				spi_device_release_bus(spi_);
				const std::int64_t elapsed = esp_timer_get_time() - started;
				stats_.txUs += elapsed;
				stats_.chunkCount++;
				if (ok) {
					if (batch.full) partialsSinceFull_.store(0);
					else partialsSinceFull_.fetch_add(1);
					if (region.fastStreak) {
						if (!settlePending_ || settleX1_ < settleX0_) {
							settleX0_ = region.x;
							settleY0_ = region.y;
							settleX1_ = region.x + region.w - 1;
							settleY1_ = region.y + region.h - 1;
						} else {
							settleX0_ = std::min(settleX0_, region.x);
							settleY0_ = std::min(settleY0_, region.y);
							settleX1_ = std::max(settleX1_, region.x + region.w - 1);
							settleY1_ = std::max(settleY1_, region.y + region.h - 1);
						}
						settlePending_ = true;
					} else if (settlePending_ &&
					           (region.full ||
					            (region.x <= settleX0_ && region.y <= settleY0_ &&
					             region.x + region.w - 1 >= settleX1_ &&
					             region.y + region.h - 1 >= settleY1_))) {
						// A quality refresh already covered the streak's union.
						settlePending_ = false;
						settleX1_ = -1;
					}
				}
				if (!ok) ESP_LOGE(kTag, "IT8951 refresh failed");
				else {
					ESP_LOGI(kTag, "%s refresh %dx%d@%d,%d mode=%u tx=%lldms wait=%lldms",
					          region.full ? "full" : "partial", region.w, region.h, region.x, region.y,
					          static_cast<unsigned>(mode), static_cast<long long>(elapsed / 1000),
					          static_cast<long long>(waveformWaitUs / 1000));
				}
				refreshInFlight_.store(false);
			}
		}
	}

	SemaphoreHandle_t mutex_ = nullptr;
	SemaphoreHandle_t packMutex_ = nullptr;
	spi_device_handle_t spi_ = nullptr;
	TaskHandle_t refreshTask_ = nullptr;
	gea::framework::graphics::Canvas canvas_;
	// GRAY4: the framebuffer IS the panel's native 4bpp packed format (2 px/byte),
	// so the render output needs no per-frame repack — the flush copies (or DMAs)
	// these bytes straight to the IT8951. Canvas writes packed nibbles here.
	std::uint8_t *frameBuffer_ = nullptr;
	std::uint8_t *dmaChunk_ = nullptr;
	std::uint8_t *dmaChunkB_ = nullptr;
	std::uint16_t *backgroundCache_ = nullptr;
	std::uint16_t *backdropCache_ = nullptr;
	std::uint32_t imageMemory_ = kFallbackImageMemory;
	std::atomic<std::uint32_t> pendingSeq_{0};
	std::uint32_t transmittedSeq_ = 0;
	RefreshBatch pendingBatch_ = {};
	std::atomic<bool> refreshInFlight_{false};
	// Refresh-task-local: an update was issued whose waveform completion has not
	// been waited on yet (waited lazily before the next load; see refreshLoop).
	bool waveformPending_ = false;
	// Fast-streak (1-bit scroll) state: window from setRefreshConfig, the last
	// request timestamp (streak detection), and the union of streak-touched
	// logical pixels awaiting the post-streak GL16 quality re-display.
	int fastStreakWindowUs_ = 0;
	std::int64_t lastRequestUs_ = 0;
	bool settlePending_ = false;
	int settleX0_ = 0;
	int settleY0_ = 0;
	int settleX1_ = -1;
	int settleY1_ = -1;
	std::atomic<bool> forceFullRefresh_{false};
	std::atomic<bool> qualityGrayscale_{false};
	// Promote full-screen-covering partials to a flashing GC16 full (default).
	// A reading app whose every page turn covers the screen opts out via
	// Display.setEpaperRefreshConfig({ fullOnCover: 0 }) and clears ghosting
	// through the fullRefreshEvery cadence instead.
	std::atomic<bool> fullOnCover_{true};
	std::atomic<int> partialsSinceFull_{0};
	int fullRefreshEvery_ = kDefaultFullRefreshEvery;
	int fullRefreshHardCap_ = kDefaultFullRefreshHardCap;
	int rotation_ = 1;
	bool backgroundAttempted_ = false;
	bool backdropAttempted_ = false;
	bool initialized_ = false;
	bool needsFullPhysicalFlush_ = true;
	platform_display::DisplayFlushPerfStats stats_ = {};
};

}  // namespace

}  // namespace gea::platform::m5paper::display

extern "C" void gea_epaper_set_refresh_config(
	int fullEvery, int hardCap, int fastWindowMs,
	const std::uint8_t *partialLut, int partialLutLength,
	const std::uint8_t *fastLut, int fastLutLength)
{
	gea::platform::m5paper::display::DisplayBackend::instance().setRefreshConfig(
		fullEvery, hardCap, fastWindowMs, partialLut, partialLutLength, fastLut, fastLutLength);
}

extern "C" void gea_epaper_full_refresh(void)
{
	gea::platform::m5paper::display::DisplayBackend::instance().requestFullRefresh();
}

extern "C" void gea_epaper_set_grayscale(int enabled)
{
	gea::platform::m5paper::display::DisplayBackend::instance().setQualityGrayscale(enabled != 0);
}

extern "C" void gea_epaper_set_full_on_cover(int enabled)
{
	gea::platform::m5paper::display::DisplayBackend::instance().setFullOnCover(enabled != 0);
}

extern "C" std::uint16_t *gea_bg_cache(int *capacity)
{
	return gea::platform::m5paper::display::DisplayBackend::instance().backgroundCache(capacity);
}

extern "C" std::uint16_t *gea_backdrop_cache(int *capacity)
{
	return gea::platform::m5paper::display::DisplayBackend::instance().backdropCache(capacity);
}

namespace gea::platform::display {

namespace {

using Backend = gea::platform::m5paper::display::DisplayBackend;
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
// copy IS the live frame; the retained-snapshot re-replay reads the packed
// storage as unpacked and lies.
extern "C" bool geaDisplaySnapshotPrefersPresented() { return true; }

void applyOrientation(gea::framework::display::DisplayOrientation orientation) { Backend::instance().applyOrientation(orientation); }
bool autoRotateAllowed() { return false; }

}  // namespace gea::platform::display
