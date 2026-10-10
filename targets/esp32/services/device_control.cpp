#if __has_include("esp_vfs_fat.h")
#include "esp_vfs_fat.h"
#define GEA_DEVICE_CONTROL_FAT_DIAGNOSTICS 1
#else
#define GEA_DEVICE_CONTROL_FAT_DIAGNOSTICS 0
#endif
#include "services/device_control.h"
#if defined(GEA_NATIVE_DEBUGGER) && GEA_NATIVE_DEBUGGER
#include "services/debugger.h"
#endif

#include "apps.h"
#include "canvas.h"
#include "display.h"
#include "events.h"
#include "i2c.h"
#include "input.h"
#include "pixel.h"
#include "touch.h"  // Touchscreen::injectEvent for GEADEV DRAG/SWIPE
#include "wifi.h"   // network::wifi() for GEADEV PING ip=/mac=
#include "host/video.h"

extern "C" uint32_t gea_display_completed_chunks() __attribute__((weak));
extern "C" bool gea_display_wait_for_uploads() __attribute__((weak));
extern "C" bool gea_frame_capture_boundary_ready() __attribute__((weak));
#include "services/app_state.h"
#include "memory_config.h"
#include "services/comparison_benchmark.h"
#include "services/comparison_input_trace.h"
#include "services/comparison_gesture.h"
#include "services/comparison_upload_capture.h"
#include "services/storage_service.h"
#include "ui/internal.h"
#include "ui/canvas_element.h"
#include "ui/tree_internal.h"
#include "ui/virtual_keyboard.h"

#if defined(GEA_EMBEDDED_SHARED_STYLES) && GEA_EMBEDDED_SHARED_STYLES
#define GEA_DIAGNOSTIC_NODE_STYLE(node) ((node).computedStyle())
#else
#define GEA_DIAGNOSTIC_NODE_STYLE(node) ((node).style)
#endif

#include "driver/i2c_master.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif
#include "esp_heap_caps.h"
#if CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#include "esp_memory_utils.h"
#endif
#include "platform/file_cache.h"  // gea::platform::storage::ensureMounted for GEADEV PUSH
#include "esp_log.h"
#include "esp_timer.h"
#include <atomic>
#if CONFIG_ESP_WIFI_ENABLED
#include "esp_wifi.h"
#endif
#include "esp_err.h"
#include "esp_mmu_map.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <dirent.h>
#include <cstdarg>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <new>
#include <sys/stat.h>  // ::mkdir for GEADEV PUSH parent dirs
#include <sys/time.h>
#include "esp_system.h"  // esp_restart() for GEADEV REBOOT
#include "esp_rom_sys.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "soc/lp_system_reg.h"  // FORCE_DOWNLOAD_BOOT for GEADEV DOWNLOAD
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
#include "soc/rtc_cntl_reg.h"  // FORCE_DOWNLOAD_BOOT for GEADEV DOWNLOAD
#endif
#if GEA_DEVICE_CONTROL_USB_OTA
#include "esp_ota_ops.h"  // GEADEV OTA
#include <algorithm>
#include <vector>
#endif
#include "audio.h"  // gea::platform::audio::AudioSystem for GEADEV PLAYFILE
#if GEA_AUDIO_EXPERIMENT
#include "../chip_bindings/audio/echo_cancellation.h"
#include <cmath>
#endif
#include "power.h"  // Power::batteryPercent() for STATE battery
#include "host/notify.h"  // gea::host::postNotification() for GEADEV NOTIFY
#include "host/backends.h"  // WifiBackend for GEADEV WIFI
#include "host/storage.h"  // gea::host::Storage for GEADEV STORAGE SET
#include "services/vp8_benchmark.h"
#include "services/opus_benchmark.h"
#include "services/srtp_benchmark.h"
#include "services/video_simd_benchmark.h"
#include "services/pcm_simd_check.h"
#if defined(GEA_VIDEO_SIMD_ESP32S3) && GEA_VIDEO_SIMD_ESP32S3
#include "../components/gea_vpx/vp8_profile_s3.h"
#endif

// Older engines expose all four direct position fields.
#ifndef GEA_CSS_POSITION_PX
#define GEA_CSS_POSITION_PX(style, side) ((style).pos_offsets[side])
#endif

namespace gea::platform::esp32::services {

namespace {

// The GEADEV command console is a development facility, and its task stack is
// 8 KiB of internal SRAM -- which on an S3 is the same memory an app's DMA
// buffers and real-time arenas come from. An app whose own budget lives there
// can set GEA_EMBEDDED_DEVICE_CONTROL=0 in gea.defines to leave the task
// unstarted. captureScreenshotRgb565() below is unaffected either way: the
// Wi-Fi OTA server's /screenshot endpoint calls it without a console.
#ifndef GEA_EMBEDDED_DEVICE_CONTROL
#define GEA_EMBEDDED_DEVICE_CONTROL 1
#endif

constexpr const char *kTag = "gea_device_ctl";
// 16 KB: a screenshot of a retained or fused-flush frame re-rasterizes the
// display list on this task (renderRetainedSnapshotRgb565), and the triangle
// rasterizer overflowed 8 KB there (stack protection fault in
// Canvas::fillTriangleOpaque on the mosaico).
constexpr int kTaskStackBytes = 16384;
constexpr int kTaskPriority = 4;
constexpr int kUsbJtagTaskStackBytes = 16384;

enum class CommandSource {
	Stdio,
	UsbSerialJtag,
};

SemaphoreHandle_t gCommandMutex = nullptr;

std::uint32_t crc32Stream(std::uint32_t crc, const std::uint8_t *data, std::size_t n);
extern "C" bool geaDisplaySnapshotPrefersPresented() __attribute__((weak));
// Defined by engines that rasterize straight into the panel's DMA buffers (gea-threejs):
// captures the next presented frame, the only place the real pixels exist.
extern "C" bool geaDisplayCapturePresentedFrame(std::uint16_t *dst, int pixelCapacity, int *width, int *height) __attribute__((weak));

// Mirrors the engine's default (core/packages/engine/ui/tree_render.cpp): a target
// that does not fuse the replay into the flush keeps a persistent framebuffer.
#ifndef GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH
#define GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH 0
#endif

char *skipSpace(char *p)
{
	while (p && *p && std::isspace(static_cast<unsigned char>(*p))) p++;
	return p;
}

char *nextToken(char *&p)
{
	p = skipSpace(p);
	if (!p || !*p) return nullptr;
	char *token = p;
	while (*p && !std::isspace(static_cast<unsigned char>(*p))) p++;
	if (*p) *p++ = '\0';
	return token;
}

bool parseInt(char *token, int *out)
{
	if (!token || !out) return false;
	char *end = nullptr;
	const long value = std::strtol(token, &end, 10);
	if (!end || *end != '\0') return false;
	if (value < -32768 || value > 32767) return false;
	*out = static_cast<int>(value);
	return true;
}

bool tokenEquals(const char *a, const char *b)
{
	if (!a || !b) return false;
	while (*a && *b) {
		const int ca = std::toupper(static_cast<unsigned char>(*a++));
		const int cb = std::toupper(static_cast<unsigned char>(*b++));
		if (ca != cb) return false;
	}
	return *a == '\0' && *b == '\0';
}

bool storageKeyIsSensitive(const char *key)
{
	if (!key) return true;
	std::string lower;
	for (const char *p = key; *p; ++p)
		lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(*p))));
	return lower.find("key") != std::string::npos ||
	       lower.find("token") != std::string::npos ||
	       lower.find("secret") != std::string::npos ||
	       lower.find("password") != std::string::npos;
}

class Base64Writer {
public:
	void writeByte(std::uint8_t byte)
	{
		pending_[pendingLength_++] = byte;
		if (pendingLength_ == 3) {
			writeQuantum(pending_, 3);
			pendingLength_ = 0;
		}
	}

	void finish()
	{
		if (pendingLength_ > 0) {
			writeQuantum(pending_, pendingLength_);
			pendingLength_ = 0;
		}
		flushLine();
	}

private:
	void writeQuantum(const std::uint8_t *bytes, int length)
	{
		const std::uint32_t a = length > 0 ? bytes[0] : 0;
		const std::uint32_t b = length > 1 ? bytes[1] : 0;
		const std::uint32_t c = length > 2 ? bytes[2] : 0;
		const std::uint32_t value = (a << 16) | (b << 8) | c;

		append(kAlphabet[(value >> 18) & 0x3f]);
		append(kAlphabet[(value >> 12) & 0x3f]);
		append(length > 1 ? kAlphabet[(value >> 6) & 0x3f] : '=');
		append(length > 2 ? kAlphabet[value & 0x3f] : '=');
	}

	void append(char ch)
	{
		line_[lineLength_++] = ch;
		if (lineLength_ >= kLineLength) flushLine();
	}

	void flushLine()
	{
		if (lineLength_ <= 0) return;
		std::printf("GEADEV:DATA ");
		std::fwrite(line_, 1, static_cast<std::size_t>(lineLength_), stdout);
		std::printf("\n");
		lineLength_ = 0;
	}

	static constexpr int kLineLength = 76;
	static constexpr char kAlphabet[] =
	    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	std::uint8_t pending_[3]{};
	int pendingLength_ = 0;
	char line_[kLineLength]{};
	int lineLength_ = 0;
};

void writeU16(Base64Writer &writer, std::uint16_t value)
{
	writer.writeByte(static_cast<std::uint8_t>(value & 0xff));
	writer.writeByte(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

void writeRun(Base64Writer &writer, std::uint16_t count, std::uint16_t rgb565)
{
	writeU16(writer, count);
	writeU16(writer, rgb565);
}

bool captureSnapshotRgb565(std::uint16_t *snapshot, int pixelCapacity, int *width, int *height,
                            char *appIdBuffer, std::size_t appIdBufferSize)
{
	using gea::platform::display::Display;

	// Before the lock: the engine waits for the frame task to present a frame.
	if (geaDisplayCapturePresentedFrame && geaDisplayCapturePresentedFrame(snapshot, pixelCapacity, width, height)) {
		gea::framework::services::AppState::lock();
		const char *presentedAppId = gea::framework::apps::AppManager::currentId();
		if (appIdBuffer && appIdBufferSize > 0)
			std::snprintf(appIdBuffer, appIdBufferSize, "%s", presentedAppId ? presentedAppId : "");
		gea::framework::services::AppState::unlock();
		return *width > 0 && *height > 0;
	}

	gea::framework::services::AppState::lock();
	bool copied = false;
	bool copiedRetained = false;
	// A screenshot must show what the panel is actually showing. The retained
	// re-replay below rebuilds the frame from the display list, so it reports what
	// the engine MEANT to draw -- it cannot witness a defect in the rasterizer, the
	// flush, or the panel byte order, and it silently skips whatever the live frame
	// took from a cache (the static backdrop blit, the bg gradient cache). That made
	// it a false witness while chasing corrupted pixels on css-3d-cube: the capture
	// came back clean every time while the panel was visibly wrong.
	//
	// When the framebuffer is persistent (the replay is NOT fused into the flush) it
	// holds the real, just-presented frame, so copy that instead. Only the fused path
	// leaves the framebuffer frozen at frame 0 -- that is the case the re-replay was
	// written for, and it keeps it.
#if GEA_EMBEDDED_DISPLAY_BANDED_UI
	// Banded UI mode streams retained frames band by band from internal RAM; the
	// framebuffer only holds the frame when the last one fell back to it.
	const bool framebufferIsLive = GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH == 0 &&
	                               !gea::embedded::ui::retainedFramebufferStale();
#else
	const bool framebufferIsLive = GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH == 0;
#endif
	if (framebufferIsLive ||
	    (geaDisplaySnapshotPrefersPresented && geaDisplaySnapshotPrefersPresented())) {
		copied = Display::copySnapshotRgb565(snapshot, pixelCapacity, width, height, true);
	}

	// Retained apps render through the fused flush, which writes the DMA staging buffer
	// and leaves the PSRAM framebuffer frozen at frame 0 — so copySnapshotRgb565 would
	// return a stale boot frame. Re-replay the live display list into the snapshot first;
	// fall back to the framebuffer/present copy for canvas apps (renderRetained returns
	// false when there's no retained command list).
	if (!copied) {
		copied = gea::embedded::ui::renderRetainedSnapshotRgb565(
		    snapshot, gea::platform::display::kWidth, gea::platform::display::kHeight);
		copiedRetained = copied;
	}
	if (copiedRetained)
	{
		*width = gea::platform::display::kWidth;
		*height = gea::platform::display::kHeight;
	}
	else if (!copied)
	{
		copied = Display::copySnapshotRgb565(snapshot, pixelCapacity, width, height, true);
	}
	const char *appId = gea::framework::apps::AppManager::currentId();
	if (appIdBuffer && appIdBufferSize > 0)
		std::snprintf(appIdBuffer, appIdBufferSize, "%s", appId ? appId : "");
	gea::framework::services::AppState::unlock();

	return copied && *width > 0 && *height > 0;
}

#if GEA_PIXEL_STORAGE_PACKED
// Same shape as captureSnapshotRgb565, but for a board whose native storage is
// sub-byte packed (GRAY4/GRAY2): copies the packed pixel stream under the
// AppState lock (cheap -- it's the board's real, small framebuffer, not an
// RGB565 expansion of it) and releases the lock before any encoding happens.
// There is no fused-replay/retained-snapshot variant here: that optimization
// (GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH) targets fast panels with a DMA
// staging buffer, never the e-paper GRAY4/GRAY2 boards this path serves.
bool captureSnapshotPacked(std::uint8_t *snapshot, int byteCapacity, int *width, int *height,
                           char *appIdBuffer, std::size_t appIdBufferSize)
{
	using gea::platform::display::Display;

	gea::framework::services::AppState::lock();
	const bool copied = Display::copySnapshotPacked(snapshot, byteCapacity, width, height, true);
	const char *appId = gea::framework::apps::AppManager::currentId();
	if (appIdBuffer && appIdBufferSize > 0)
		std::snprintf(appIdBuffer, appIdBufferSize, "%s", appId ? appId : "");
	gea::framework::services::AppState::unlock();

	return copied && *width > 0 && *height > 0;
}
#endif  // GEA_PIXEL_STORAGE_PACKED

void writeScreenshot()
{
	int width = 0;
	int height = 0;
	char appIdBuffer[96];

#if GEA_PIXEL_STORAGE_PACKED
	// A packed (GRAY4/GRAY2) board's real framebuffer is a quarter/eighth the
	// size an RGB565 expansion of the same pixels would need -- allocate that
	// (what the device can actually spare) and expand to RGB565 per pixel while
	// RLE-encoding below, instead of asking heap_caps_malloc for a buffer sized
	// for a pixel format this board doesn't have.
	const int pixelCountInt = gea::platform::display::kWidth * gea::platform::display::kHeight;
	const std::size_t pixelCount = static_cast<std::size_t>(pixelCountInt);
	const std::size_t byteCount =
	    static_cast<std::size_t>(gea::framework::graphics::pixel::packed::rowBytes(pixelCountInt));
	auto *snapshot = static_cast<std::uint8_t *>(
	    heap_caps_malloc(byteCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	bool heapCapsAllocated = snapshot != nullptr;
	if (!snapshot) snapshot = static_cast<std::uint8_t *>(std::malloc(byteCount));
	if (!snapshot) {
		std::printf("GEADEV:ERR SCREENSHOT no-memory bytes=%u\n", static_cast<unsigned>(byteCount));
		std::fflush(stdout);
		return;
	}

	const bool copied = captureSnapshotPacked(snapshot, static_cast<int>(byteCount), &width, &height,
	                                          appIdBuffer, sizeof(appIdBuffer));
	if (!copied) {
		if (heapCapsAllocated) heap_caps_free(snapshot);
		else std::free(snapshot);
		std::printf("GEADEV:ERR SCREENSHOT display-unavailable\n");
		std::fflush(stdout);
		return;
	}

	flockfile(stdout);
	std::printf("GEADEV:SCREENSHOT BEGIN width=%d height=%d encoding=rgb565-rle-v1 app=%s\n",
	            width,
	            height,
	            appIdBuffer);

	Base64Writer writer;
	if (pixelCount > 0) {
		using gea::framework::graphics::pixel::packed;
		using gea::framework::graphics::pixel::fromNative;
		std::uint16_t runValue = fromNative(packed::get(snapshot, 0));
		std::uint16_t runCount = 1;
		for (std::size_t i = 1; i < pixelCount; i++) {
			const std::uint16_t value = fromNative(packed::get(snapshot, static_cast<int>(i)));
			if (value == runValue && runCount < 65535) {
				runCount++;
				continue;
			}
			writeRun(writer, runCount, runValue);
			runValue = value;
			runCount = 1;
		}
		writeRun(writer, runCount, runValue);
	}
	writer.finish();
	std::printf("GEADEV:SCREENSHOT END\n");
	std::fflush(stdout);
	funlockfile(stdout);

	if (heapCapsAllocated) heap_caps_free(snapshot);
	else std::free(snapshot);
#else
	const int pixelCountInt = gea::platform::display::kWidth * gea::platform::display::kHeight;
	const std::size_t pixelCount = static_cast<std::size_t>(pixelCountInt);
	auto *snapshot = static_cast<std::uint16_t *>(
	    heap_caps_malloc(pixelCount * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	bool heapCapsAllocated = snapshot != nullptr;
	if (!snapshot) snapshot = static_cast<std::uint16_t *>(std::malloc(pixelCount * sizeof(std::uint16_t)));
	if (!snapshot) {
		std::printf("GEADEV:ERR SCREENSHOT no-memory bytes=%u\n",
		            static_cast<unsigned>(pixelCount * sizeof(std::uint16_t)));
		std::fflush(stdout);
		return;
	}

	const bool copied = captureSnapshotRgb565(snapshot, pixelCountInt, &width, &height,
	                                          appIdBuffer, sizeof(appIdBuffer));
	if (!copied) {
		if (heapCapsAllocated) heap_caps_free(snapshot);
		else std::free(snapshot);
		std::printf("GEADEV:ERR SCREENSHOT display-unavailable\n");
		std::fflush(stdout);
		return;
	}

	flockfile(stdout);
	std::printf("GEADEV:SCREENSHOT BEGIN width=%d height=%d encoding=rgb565-rle-v1 app=%s\n",
	            width,
	            height,
	            appIdBuffer);

	Base64Writer writer;
	if (pixelCount > 0) {
		std::uint16_t runValue = snapshot[0];
		std::uint16_t runCount = 1;
		for (std::size_t i = 1; i < pixelCount; i++) {
			const std::uint16_t value = snapshot[i];
			if (value == runValue && runCount < 65535) {
				runCount++;
				continue;
			}
			writeRun(writer, runCount, runValue);
			runValue = value;
			runCount = 1;
		}
		writeRun(writer, runCount, runValue);
	}
	writer.finish();
	std::printf("GEADEV:SCREENSHOT END\n");
	std::fflush(stdout);
	funlockfile(stdout);

	if (heapCapsAllocated) heap_caps_free(snapshot);
	else std::free(snapshot);
#endif  // GEA_PIXEL_STORAGE_PACKED
}

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
class ScopedLogSilence {
public:
	ScopedLogSilence()
	    : previous_(esp_log_level_get(nullptr))
	{
		esp_log_level_set("*", ESP_LOG_NONE);
	}

	~ScopedLogSilence()
	{
		esp_log_level_set("*", previous_);
	}

private:
	esp_log_level_t previous_;
};

bool usbWriteAll(const void *data, std::size_t size)
{
	const auto *bytes = static_cast<const std::uint8_t *>(data);
	std::size_t offset = 0;
	int emptyWrites = 0;
	while (offset < size) {
		const std::size_t remaining = size - offset;
		const std::size_t chunk = remaining > 512 ? 512 : remaining;
		const int wrote = usb_serial_jtag_write_bytes(bytes + offset, chunk, pdMS_TO_TICKS(1000));
		if (wrote <= 0) {
			if (++emptyWrites > 5) return false;
			vTaskDelay(pdMS_TO_TICKS(2));
			continue;
		}
		emptyWrites = 0;
		offset += static_cast<std::size_t>(wrote);
	}
	return true;
}

bool usbPrintf(const char *format, ...)
{
	char line[192];
	va_list args;
	va_start(args, format);
	const int n = std::vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	if (n <= 0) return true;
	const std::size_t len = static_cast<std::size_t>(n);
	if (len < sizeof(line)) return usbWriteAll(line, len);

	char *heapLine = static_cast<char *>(std::malloc(len + 1));
	if (!heapLine) return false;
	va_start(args, format);
	std::vsnprintf(heapLine, len + 1, format, args);
	va_end(args);
	const bool ok = usbWriteAll(heapLine, len);
	std::free(heapLine);
	return ok;
}

#if GEA_PIXEL_STORAGE_PACKED
void writeScreenshotUsbRaw()
{
	int width = 0;
	int height = 0;
	const int pixelCountInt = gea::platform::display::kWidth * gea::platform::display::kHeight;
	const std::size_t byteCount =
	    static_cast<std::size_t>(gea::framework::graphics::pixel::packed::rowBytes(pixelCountInt));
	auto *snapshot = static_cast<std::uint8_t *>(
	    heap_caps_malloc(byteCount, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	bool heapCapsAllocated = snapshot != nullptr;
	if (!snapshot) snapshot = static_cast<std::uint8_t *>(std::malloc(byteCount));
	if (!snapshot) {
		usbPrintf("GEADEV:ERR SCREENSHOTBIN no-memory bytes=%u\n", static_cast<unsigned>(byteCount));
		return;
	}

	char appIdBuffer[96];
	const bool copied = captureSnapshotPacked(snapshot, static_cast<int>(byteCount), &width, &height,
	                                          appIdBuffer, sizeof(appIdBuffer));
	if (!copied) {
		if (heapCapsAllocated) heap_caps_free(snapshot);
		else std::free(snapshot);
		usbPrintf("GEADEV:ERR SCREENSHOTBIN display-unavailable\n");
		return;
	}

	// Wire format stays rgb565-raw-v1 (host decoder is unchanged): expand packed
	// pixels to RGB565 ONE ROW AT A TIME while streaming to USB, instead of
	// holding a second full-frame RGB565 buffer the device does not have room
	// for. The packed snapshot above was copied under the AppState lock; the
	// expand-and-send loop below runs entirely outside it, same as the RGB565
	// path.
	const std::size_t bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * sizeof(std::uint16_t);
	const std::size_t rowBytes = static_cast<std::size_t>(width) * sizeof(std::uint16_t);
	auto *rowRgb = static_cast<std::uint16_t *>(heap_caps_malloc(rowBytes, MALLOC_CAP_8BIT));
	bool rowHeapCapsAllocated = rowRgb != nullptr;
	if (!rowRgb) rowRgb = static_cast<std::uint16_t *>(std::malloc(rowBytes));
	if (!rowRgb) {
		if (heapCapsAllocated) heap_caps_free(snapshot);
		else std::free(snapshot);
		usbPrintf("GEADEV:ERR SCREENSHOTBIN no-memory bytes=%u\n", static_cast<unsigned>(rowBytes));
		return;
	}

	ScopedLogSilence silence;
	flockfile(stdout);
	std::fflush(stdout);
	usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(500));
	const bool headerOk = usbPrintf(
	    "GEADEV:SCREENSHOTBIN BEGIN width=%d height=%d encoding=rgb565-raw-v1 bytes=%u app=%s\n",
	    width, height, static_cast<unsigned>(bytes), appIdBuffer);

	using gea::framework::graphics::pixel::packed;
	using gea::framework::graphics::pixel::fromNative;
	std::uint32_t crc = 0xFFFFFFFFu;
	bool bodyOk = headerOk;
	for (int row = 0; row < height && bodyOk; ++row) {
		for (int x = 0; x < width; ++x)
			rowRgb[x] = fromNative(packed::get(snapshot, row * width + x));
		crc = crc32Stream(crc, reinterpret_cast<const std::uint8_t *>(rowRgb), rowBytes);
		bodyOk = usbWriteAll(rowRgb, rowBytes);
	}
	crc ^= 0xFFFFFFFFu;
	const bool footerOk = bodyOk && usbPrintf("GEADEV:SCREENSHOTBIN END bytes=%u crc=0x%08x\n",
	                                          static_cast<unsigned>(bytes), static_cast<unsigned>(crc));
	(void)footerOk;
	usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(5000));
	funlockfile(stdout);

	if (rowHeapCapsAllocated) heap_caps_free(rowRgb);
	else std::free(rowRgb);
	if (heapCapsAllocated) heap_caps_free(snapshot);
	else std::free(snapshot);
}
#else
void writeScreenshotUsbRaw()
{
	int width = 0;
	int height = 0;
	const int pixelCountInt = gea::platform::display::kWidth * gea::platform::display::kHeight;
	const std::size_t pixelCount = static_cast<std::size_t>(pixelCountInt);
	auto *snapshot = static_cast<std::uint16_t *>(
	    heap_caps_malloc(pixelCount * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	bool heapCapsAllocated = snapshot != nullptr;
	if (!snapshot) snapshot = static_cast<std::uint16_t *>(std::malloc(pixelCount * sizeof(std::uint16_t)));
	if (!snapshot) {
		usbPrintf("GEADEV:ERR SCREENSHOTBIN no-memory bytes=%u\n",
		          static_cast<unsigned>(pixelCount * sizeof(std::uint16_t)));
		return;
	}

	char appIdBuffer[96];
	const bool copied = captureSnapshotRgb565(snapshot, pixelCountInt, &width, &height,
	                                          appIdBuffer, sizeof(appIdBuffer));
	if (!copied) {
		if (heapCapsAllocated) heap_caps_free(snapshot);
		else std::free(snapshot);
		usbPrintf("GEADEV:ERR SCREENSHOTBIN display-unavailable\n");
		return;
	}

	const std::size_t bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * sizeof(std::uint16_t);
	const std::uint32_t crc = crc32Stream(0xFFFFFFFFu, reinterpret_cast<const std::uint8_t *>(snapshot), bytes) ^ 0xFFFFFFFFu;
	ScopedLogSilence silence;
	flockfile(stdout);
	std::fflush(stdout);
	usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(500));
	const bool headerOk = usbPrintf(
	    "GEADEV:SCREENSHOTBIN BEGIN width=%d height=%d encoding=rgb565-raw-v1 bytes=%u app=%s\n",
	    width, height, static_cast<unsigned>(bytes), appIdBuffer);
	const bool bodyOk = headerOk && usbWriteAll(snapshot, bytes);
	const bool footerOk = bodyOk && usbPrintf("GEADEV:SCREENSHOTBIN END bytes=%u crc=0x%08x\n",
	                                          static_cast<unsigned>(bytes), static_cast<unsigned>(crc));
	(void)footerOk;
	usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(5000));
	funlockfile(stdout);

	if (heapCapsAllocated) heap_caps_free(snapshot);
	else std::free(snapshot);
}
#endif  // GEA_PIXEL_STORAGE_PACKED
#endif

// Set the system wall clock from the host (no WiFi/SNTP needed). The watch face
// reads it via Date(); the host sends its own clock so the watch shows real time.
void handleSetTime(char *&cursor)
{
	char *tok = nextToken(cursor);
	if (!tok || tok[0] == '\0') {
		std::printf("GEADEV:ERR SETTIME usage=GEADEV_SETTIME_epoch_seconds\n");
		return;
	}
	const long long epoch = std::strtoll(tok, nullptr, 10);
	struct timeval tv;
	tv.tv_sec = static_cast<time_t>(epoch);
	tv.tv_usec = 0;
	if (settimeofday(&tv, nullptr) == 0) {
		struct timeval back;
		gettimeofday(&back, nullptr);
		std::printf("GEADEV:OK SETTIME epoch=%lld readback=%lld\n", epoch, static_cast<long long>(back.tv_sec));
	} else {
		std::printf("GEADEV:ERR SETTIME settimeofday-failed\n");
	}
}

// Get or set the display backlight brightness (0-100%) from the host. No arg
// reports the current value; with an arg, clamps to 0-100, sets it via the
// Display service, and reads it back. Gives the Mac companion a brightness
// control over USB (the same Display::setBrightness the settings panel uses).
void handleBrightness(char *&cursor)
{
	using gea::platform::display::Display;
	char *tok = nextToken(cursor);
	if (!tok || tok[0] == '\0') {
		std::printf("GEADEV:OK BRIGHTNESS value=%d\n", Display::brightness());
		return;
	}
	int pct = 0;
	if (!parseInt(tok, &pct)) {
		std::printf("GEADEV:ERR BRIGHTNESS usage=GEADEV_BRIGHTNESS_[0-100]\n");
		return;
	}
	if (pct < 0) pct = 0;
	if (pct > 100) pct = 100;
	Display::setBrightness(pct);
	std::printf("GEADEV:OK BRIGHTNESS value=%d readback=%d\n", pct, Display::brightness());
}

// GEADEV HBM / VSYNC [on|off] -- the panel toggles that sit beside
// brightness. Every display knob has to answer on BOTH transports: an app
// with no network binding builds firmware with WiFi disabled, so the HTTP
// endpoints in ota.cpp are unreachable and USB is the only way in.
// Accepts on/off, 1/0, true/false, yes/no. `off` and `on` share no first
// letter with each other's negation, so the first character decides except
// for the o-pair, which needs the second.
bool parseOnOff(const char *token, bool *out)
{
	if (!token || token[0] == '\0') return false;
	const char first = static_cast<char>(std::tolower(static_cast<unsigned char>(token[0])));
	const char second = static_cast<char>(std::tolower(static_cast<unsigned char>(token[1])));
	if (first == 'o') {
		if (second == 'n') {
			*out = true;
			return true;
		}
		if (second == 'f') {
			*out = false;
			return true;
		}
		return false;
	}
	if (first == '1' || first == 't' || first == 'y') {
		*out = true;
		return true;
	}
	if (first == '0' || first == 'f' || first == 'n') {
		*out = false;
		return true;
	}
	return false;
}

void handleHbm(char *&cursor)
{
	using gea::platform::display::Display;
	char *tok = nextToken(cursor);
	if (!tok || tok[0] == '\0') {
		std::printf("GEADEV:OK HBM value=%d\n", Display::highBrightnessMode() ? 1 : 0);
		return;
	}
	bool on = false;
	if (!parseOnOff(tok, &on)) {
		std::printf("GEADEV:ERR HBM usage=GEADEV_HBM_[on|off]\n");
		return;
	}
	// A panel without a high-brightness ceiling reports supported=0 rather
	// than erroring: the caller asked a question the board can answer.
	const bool ok = Display::setHighBrightnessMode(on);
	std::printf("GEADEV:OK HBM value=%d supported=%d\n", Display::highBrightnessMode() ? 1 : 0, ok ? 1 : 0);
}

void handleVsync(char *&cursor)
{
	using gea::platform::display::Display;
	char *tok = nextToken(cursor);
	if (!tok || tok[0] == '\0') {
		std::printf("GEADEV:OK VSYNC value=%d\n", Display::vsyncEnabled() ? 1 : 0);
		return;
	}
	bool on = false;
	if (!parseOnOff(tok, &on)) {
		std::printf("GEADEV:ERR VSYNC usage=GEADEV_VSYNC_[on|off]\n");
		return;
	}
	Display::setVSync(on);
	std::printf("GEADEV:OK VSYNC value=%d\n", Display::vsyncEnabled() ? 1 : 0);
}

void handleI2cScan()
{
	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) {
		std::printf("GEADEV:ERR I2CSCAN bus-unavailable\n");
		return;
	}

	auto nativeBus = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
	int count = 0;
	std::printf("GEADEV:I2CSCAN BEGIN\n");
	for (std::uint16_t address = 0x08; address <= 0x77; address++) {
		const esp_err_t err = i2c_master_probe(nativeBus, address, 50);
		if (err == ESP_OK) {
			std::printf("GEADEV:I2CSCAN ADDR 0x%02x\n", static_cast<unsigned>(address));
			count++;
		}
	}
	std::printf("GEADEV:I2CSCAN END count=%d\n", count);
}

void handleNode(char *&cursor)
{
	const char *className = nextToken(cursor);
	if (!className || className[0] == '\0') {
		std::printf("GEADEV:ERR NODE usage=GEADEV_NODE_class\n");
		return;
	}

	gea::framework::services::AppState::lock();
	auto &tree = gea::embedded::ui::Tree::instance();
	const int nodeCount = tree.nodeCount();
	int printed = 0;
	for (int nodeId = 0; nodeId < nodeCount; nodeId++) {
		if (!tree.hasClass(nodeId, className)) continue;
		const auto &node = tree.node(nodeId);
		int16_t xs[4] = {};
		int16_t ys[4] = {};
		int commandX0 = 0;
		int commandY0 = 0;
		int commandX1 = 0;
		int commandY1 = 0;
		const auto &displayList = gea::embedded::ui::DisplayList::instance();
		displayList.nodeCommandBounds(nodeId, &commandX0, &commandY0, &commandX1, &commandY1);
		const gea::embedded::ui::DisplayCommand *textCommand = nullptr;
		for (int commandIndex = 0; commandIndex < displayList.nodeCommandCount(nodeId); commandIndex++) {
			const auto *command = displayList.nodeCommandAt(nodeId, commandIndex);
			if (command && command->type == gea::embedded::ui::DisplayCommandType::DrawText) {
				textCommand = command;
				break;
			}
		}
		gea::embedded::ui::ViewRenderer::transformedRectCorners(node,
		                                                        false,
		                                                        node.layout.x,
		                                                        node.layout.y,
		                                                        node.layout.width,
		                                                        node.layout.height,
		                                                        xs,
		                                                        ys);
		std::printf(
		    "GEADEV:NODE class=%s id=%d parent=%d type=%d display=%d style=%dx%d min=%dx%d max=%dx%d pos=%d,%d,%d,%d transform=%d,%d,%d,%d,%d layout=%d,%d,%d,%d corners=%d,%d;%d,%d;%d,%d;%d,%d ovf=%d,%d,%d scroll=%d,%d,%d,%d font=%d,%d,%d opacity=%d commands=%d bounds=%d,%d,%d,%d drawtext=%d,%d,%d,%d,%d,%d,%d,%d,%g\n",
		    className,
		    nodeId,
		    node.parent,
		    static_cast<int>(node.type),
		    GEA_DIAGNOSTIC_NODE_STYLE(node).display,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).width,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).height,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).min_width,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).min_height,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).max_width,
		    GEA_DIAGNOSTIC_NODE_STYLE(node).max_height,
		    GEA_CSS_POSITION_PX(GEA_DIAGNOSTIC_NODE_STYLE(node), 3),
		    GEA_CSS_POSITION_PX(GEA_DIAGNOSTIC_NODE_STYLE(node), 0),
		    GEA_CSS_POSITION_PX(GEA_DIAGNOSTIC_NODE_STYLE(node), 1),
		    GEA_CSS_POSITION_PX(GEA_DIAGNOSTIC_NODE_STYLE(node), 2),
		    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_x,
		    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_y,
		    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_x_percent,
		    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_y_percent,
		    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_rotate,
		    node.layout.x,
		    node.layout.y,
		    node.layout.width,
		    node.layout.height,
		    xs[0],
		    ys[0],
		    xs[1],
		    ys[1],
		    xs[2],
		    ys[2],
		    xs[3],
		    ys[3],
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).overflow),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).overflow_x),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).overflow_y),
		    static_cast<int>(node.layout.scroll_x),
		    static_cast<int>(node.layout.scroll_y),
		    static_cast<int>(node.layout.scroll_content_width),
		    static_cast<int>(node.layout.scroll_content_height),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).font_id),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).font_size),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).line_height),
		    static_cast<int>(GEA_DIAGNOSTIC_NODE_STYLE(node).opacity),
		    displayList.nodeCommandCount(nodeId),
		    commandX0,
		    commandY0,
		    commandX1,
		    commandY1,
		    textCommand && textCommand->text.text ? static_cast<unsigned char>(textCommand->text.text[0]) : -1,
		    textCommand ? textCommand->text.x : 0,
		    textCommand ? textCommand->text.y : 0,
		    textCommand ? textCommand->bx : 0,
		    textCommand ? textCommand->by : 0,
		    textCommand ? textCommand->bw : 0,
		    textCommand ? textCommand->bh : 0,
		    textCommand ? textCommand->text.fontId : -1,
		    textCommand ? static_cast<double>(textCommand->text.scale) : 0.0);
		printed++;
	}
	gea::framework::services::AppState::unlock();
	if (printed == 0) std::printf("GEADEV:NODE class=%s none=1 nodes=%d\n", className, nodeCount);
}

void handleHitTest(char *&cursor)
{
	int x = 0;
	int y = 0;
	if (!parseInt(nextToken(cursor), &x) || !parseInt(nextToken(cursor), &y)) {
		std::printf("GEADEV:ERR HITTEST usage=GEADEV_HITTEST_x_y\n");
		return;
	}

	gea::framework::services::AppState::lock();
	auto &tree = gea::embedded::ui::Tree::instance();
	const int nodeId = tree.hitTestNode(x, y);
	const int pressTarget = tree.hitTest(x, y);
	if (nodeId < 0 || nodeId >= tree.nodeCount()) {
		gea::framework::services::AppState::unlock();
		std::printf("GEADEV:HITTEST x=%d y=%d node=-1 press=-1\n", x, y);
		return;
	}
	const auto &node = tree.node(nodeId);
	const std::string classes = tree.className(nodeId);
	const int pressId = tree.pressId(nodeId);
	std::printf(
	    "GEADEV:HITTEST x=%d y=%d node=%d parent=%d type=%d display=%d class=\"%s\" press=%d press_id=%d transform=%d,%d,%d,%d,%d layout=%d,%d,%d,%d\n",
	    x,
	    y,
	    nodeId,
	    node.parent,
	    static_cast<int>(node.type),
	    GEA_DIAGNOSTIC_NODE_STYLE(node).display,
	    classes.c_str(),
	    pressTarget,
	    pressId,
	    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_x,
	    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_y,
	    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_x_percent,
	    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_translate_y_percent,
	    gea::embedded::ui::rstyle(GEA_DIAGNOSTIC_NODE_STYLE(node)).transform_rotate,
	    node.layout.x,
	    node.layout.y,
	    node.layout.width,
	    node.layout.height);
	gea::framework::services::AppState::unlock();
}

void handleTap(char *&cursor)
{
	int x = 0;
	int y = 0;
	int holdMs = 80;
	if (!parseInt(nextToken(cursor), &x) || !parseInt(nextToken(cursor), &y)) {
		std::printf("GEADEV:ERR TAP usage=GEADEV_TAP_x_y_[hold_ms]\n");
		return;
	}
	char *holdToken = nextToken(cursor);
	if (holdToken && !parseInt(holdToken, &holdMs)) {
		std::printf("GEADEV:ERR TAP invalid-hold-ms\n");
		return;
	}
	if (holdMs < 0) holdMs = 0;
	if (holdMs > 2000) holdMs = 2000;

	gea::framework::events::TouchRuntime::queueTouchEvent(gea::framework::events::TouchPhase::Down, true, x, y);
	vTaskDelay(pdMS_TO_TICKS(holdMs));
	gea::framework::events::TouchRuntime::queueTouchEvent(gea::framework::events::TouchPhase::Up, false, x, y);
	std::printf("GEADEV:OK TAP x=%d y=%d hold_ms=%d\n", x, y, holdMs);
}

void handleDrag(char *&cursor)
{
	int x1 = 0;
	int y1 = 0;
	int x2 = 0;
	int y2 = 0;
	int steps = 6;
	int delayMs = 24;
	if (!parseInt(nextToken(cursor), &x1) || !parseInt(nextToken(cursor), &y1) ||
	    !parseInt(nextToken(cursor), &x2) || !parseInt(nextToken(cursor), &y2)) {
		std::printf("GEADEV:ERR DRAG usage=GEADEV_DRAG_x1_y1_x2_y2_[steps]_[delay_ms]\n");
		return;
	}
	char *stepsToken = nextToken(cursor);
	if (stepsToken && !parseInt(stepsToken, &steps)) {
		std::printf("GEADEV:ERR DRAG invalid-steps\n");
		return;
	}
	char *delayToken = nextToken(cursor);
	if (delayToken && !parseInt(delayToken, &delayMs)) {
		std::printf("GEADEV:ERR DRAG invalid-delay-ms\n");
		return;
	}
	if (steps < 1) steps = 1;
	if (steps > 64) steps = 64;
	if (delayMs < 0) delayMs = 0;
	if (delayMs > 250) delayMs = 250;

	// Drive the drag through Touchscreen::injectEvent (not queueTouchEvent): the
	// Move phase must update the controller's latestMove_ cache, or the
	// TouchRuntime's consumeLatestMove overwrites these coords with the last real
	// hardware sample and the synthetic drag never scrolls a momentum container.
	using gea::platform::touch::Touchscreen;
	using gea::platform::touch::Phase;
	Touchscreen::injectEvent(Phase::Down, true, x1, y1);
	for (int i = 1; i <= steps; ++i) {
		if (delayMs > 0) vTaskDelay(pdMS_TO_TICKS(delayMs));
		const int x = x1 + ((x2 - x1) * i) / steps;
		const int y = y1 + ((y2 - y1) * i) / steps;
		Touchscreen::injectEvent(Phase::Move, true, x, y);
	}
	if (delayMs > 0) vTaskDelay(pdMS_TO_TICKS(delayMs));
	Touchscreen::injectEvent(Phase::Up, false, x2, y2);
	std::printf("GEADEV:OK DRAG x1=%d y1=%d x2=%d y2=%d steps=%d delay_ms=%d\n",
	            x1, y1, x2, y2, steps, delayMs);
}

void handleRotary(char *&cursor)
{
	int delta = 0;
	int repeat = 1;
	int delayMs = 0;
	if (!parseInt(nextToken(cursor), &delta)) {
		std::printf("GEADEV:ERR ROTARY usage=GEADEV_ROTARY_delta_[repeat]_[delay_ms]\n");
		return;
	}
	char *repeatToken = nextToken(cursor);
	if (repeatToken && !parseInt(repeatToken, &repeat)) {
		std::printf("GEADEV:ERR ROTARY invalid-repeat\n");
		return;
	}
	char *delayToken = nextToken(cursor);
	if (delayToken && !parseInt(delayToken, &delayMs)) {
		std::printf("GEADEV:ERR ROTARY invalid-delay-ms\n");
		return;
	}
	if (repeat < 1) repeat = 1;
	if (repeat > 200) repeat = 200;
	if (delayMs < 0) delayMs = 0;
	if (delayMs > 2000) delayMs = 2000;

	for (int i = 0; i < repeat; ++i) {
		gea::framework::input::queueRotaryDelta(delta);
		if (delayMs > 0 && i + 1 < repeat) vTaskDelay(pdMS_TO_TICKS(delayMs));
	}
	std::printf("GEADEV:OK ROTARY delta=%d repeat=%d delay_ms=%d\n", delta, repeat, delayMs);
}

// Inject a key press (web keyCode) through the same queue hardware buttons
// use, so button-driven boards (e-paper BOOT/PWR = ArrowUp/ArrowDown) are
// exercisable over serial.
void handleKey(char *&cursor)
{
	int keyCode = 0;
	if (!parseInt(nextToken(cursor), &keyCode) || keyCode == 0) {
		std::printf("GEADEV:ERR KEY usage=GEADEV_KEY_keycode\n");
		return;
	}
	gea::framework::input::queueKeyDown(keyCode);
	gea::framework::input::queueKeyUp(keyCode);
	std::printf("GEADEV:OK KEY keycode=%d\n", keyCode);
}

// Inspect or seed app-facing localStorage from the host. GET is base64 encoded
// so tab/newline-rich values survive the line protocol, and secret-like key
// names are denied to avoid echoing credentials into terminal logs.
void handleStorage(char *&cursor)
{
	char *action = nextToken(cursor);
	if (!action) {
		std::printf("GEADEV:ERR STORAGE usage=GEADEV_STORAGE_SET_key_value|GEADEV_STORAGE_GET_key\n");
		return;
	}

	char *key = nextToken(cursor);
	if (!key || key[0] == '\0') {
		std::printf("GEADEV:ERR STORAGE usage=GEADEV_STORAGE_SET_key_value|GEADEV_STORAGE_GET_key\n");
		return;
	}

	if (tokenEquals(action, "GET")) {
		if (storageKeyIsSensitive(key)) {
			std::printf("GEADEV:STORAGE GET ERR key=%s denied=sensitive\n", key);
			return;
		}
		const std::string value = gea::host::Storage.getItem(std::string(key));
		flockfile(stdout);
		std::printf("GEADEV:STORAGE GET BEGIN key=%s bytes=%d encoding=base64\n",
		            key, static_cast<int>(value.size()));
		Base64Writer writer;
		for (unsigned char ch : value)
			writer.writeByte(ch);
		writer.finish();
		std::printf("GEADEV:STORAGE GET END key=%s bytes=%d\n", key, static_cast<int>(value.size()));
		std::fflush(stdout);
		funlockfile(stdout);
		return;
	}

	if (!tokenEquals(action, "SET")) {
		std::printf("GEADEV:ERR STORAGE usage=GEADEV_STORAGE_SET_key_value|GEADEV_STORAGE_GET_key\n");
		return;
	}

	char *value = skipSpace(cursor);
	if (!value || value[0] == '\0') {
		std::printf("GEADEV:ERR STORAGE usage=GEADEV_STORAGE_SET_key_value\n");
		return;
	}

	gea::host::Storage.setItem(std::string(key), std::string(value));
	std::printf("GEADEV:OK STORAGE key=%s bytes=%d\n", key, static_cast<int>(std::strlen(value)));
}

// Simulate a swipe: touch down at (x,y1), move, up at (x,y2). Lets the host
// exercise gestures like the top-edge swipe-down that opens settings.
void handleSwipe(char *&cursor)
{
	int x = 0;
	int y1 = 0;
	int y2 = 0;
	if (!parseInt(nextToken(cursor), &x) || !parseInt(nextToken(cursor), &y1) || !parseInt(nextToken(cursor), &y2)) {
		std::printf("GEADEV:ERR SWIPE usage=GEADEV_SWIPE_x_y1_y2\n");
		return;
	}
	// Same injectEvent path as handleDrag so the Move updates latestMove_ and the
	// swipe actually drives scroll/gesture handlers (see handleDrag note).
	using gea::platform::touch::Touchscreen;
	using gea::platform::touch::Phase;
	Touchscreen::injectEvent(Phase::Down, true, x, y1);
	vTaskDelay(pdMS_TO_TICKS(40));
	Touchscreen::injectEvent(Phase::Move, true, x, (y1 + y2) / 2);
	vTaskDelay(pdMS_TO_TICKS(40));
	Touchscreen::injectEvent(Phase::Up, false, x, y2);
	std::printf("GEADEV:OK SWIPE x=%d y1=%d y2=%d\n", x, y1, y2);
}

// geaos Phase 2 spike probe: map the spare ota_1 flash region onto the
// instruction bus at runtime and report the vaddr the MMU assigns + the free
// executable vaddr space. Settles whether a dynamically-loaded app can be
// fixed-base linked (link vaddr must equal the vaddr esp_mmu_map hands back).
// Uses ota_1 so it never touches the protected partitions.csv.
void handleMmuProbe()
{
	const esp_partition_t *slot =
	    esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
	if (!slot) {
		std::printf("GEADEV:ERR MMUPROBE no-ota_1\n");
		return;
	}
	const mmu_mem_caps_t caps = static_cast<mmu_mem_caps_t>(MMU_MEM_CAP_EXEC | MMU_MEM_CAP_READ);
	size_t maxBlock = 0;
	esp_mmu_map_get_max_consecutive_free_block_size(caps, MMU_TARGET_FLASH0, &maxBlock);
	void *vaddr = nullptr;
	const esp_err_t err =
	    esp_mmu_map(static_cast<esp_paddr_t>(slot->address), 0x10000, MMU_TARGET_FLASH0, caps, 0, &vaddr);
	std::printf("GEADEV:MMUPROBE ota1_paddr=0x%08x map=%s vaddr=%p max_free_exec=%u\n",
	            static_cast<unsigned>(slot->address), esp_err_to_name(err), vaddr,
	            static_cast<unsigned>(maxBlock));
	if (err == ESP_OK && vaddr) {
		const uint32_t first = *static_cast<volatile uint32_t *>(vaddr);
		std::printf("GEADEV:MMUPROBE first_word=0x%08x (instruction-bus read ok)\n",
		            static_cast<unsigned>(first));
		esp_mmu_unmap(vaddr);
	}
	// Data-bus (READ-only) mapping: where app .rodata must live so byte reads
	// work (the EXEC/instruction-bus mapping above faults on byte data access).
	void *dvaddr = nullptr;
	const esp_err_t derr = esp_mmu_map(static_cast<esp_paddr_t>(slot->address), 0x10000,
	                                   MMU_TARGET_FLASH0, MMU_MEM_CAP_READ, 0, &dvaddr);
	std::printf("GEADEV:MMUPROBE read_only_map=%s dbus_vaddr=%p\n", esp_err_to_name(derr), dvaddr);
	if (derr == ESP_OK && dvaddr) {
		const uint8_t b0 = *static_cast<volatile uint8_t *>(dvaddr);
		std::printf("GEADEV:MMUPROBE dbus_byte0=0x%02x (data-bus byte read ok)\n", static_cast<unsigned>(b0));
		esp_mmu_unmap(dvaddr);
	}
}

// Firmware callback the loaded app invokes (passed as a pointer — no symbol
// resolution, isolates the execute/callback mechanism).
void loadtest_log(const char *msg) { std::printf("GEADEV:LOADTEST app-says: %s\n", msg ? msg : ""); }

// geaos Phase 2 execute-proof: map the app blob in ota_1 onto the instruction
// bus, read its 16-byte header (magic/abi/entry-offset), and CALL the entry
// (XIP from flash) passing loadtest_log. Proves map -> jump -> execute ->
// callback -> return across the loader boundary. Blob built at
// lib/gea-embedded/geaos/spike/ and flashed to ota_1 (0x1010000).
void handleLoadTest()
{
	const esp_partition_t *slot =
	    esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
	if (!slot) {
		std::printf("GEADEV:ERR LOADTEST no-ota_1\n");
		return;
	}
	// Two views of the one blob: code on the instruction bus (EXEC), data on the
	// data bus (READ). PADDR_SHARED lets the same flash paddr map to both vaddrs.
	void *codeVaddr = nullptr;
	void *dataVaddr = nullptr;
	// Code page (blob offset 0) -> instruction bus; rodata page (offset 0x10000)
	// -> data bus. Different paddrs, so each maps once (no PADDR_SHARED).
	const esp_err_t ce = esp_mmu_map(static_cast<esp_paddr_t>(slot->address), 0x10000, MMU_TARGET_FLASH0,
	                                 static_cast<mmu_mem_caps_t>(MMU_MEM_CAP_EXEC | MMU_MEM_CAP_READ), 0,
	                                 &codeVaddr);
	const esp_err_t de = esp_mmu_map(static_cast<esp_paddr_t>(slot->address) + 0x10000, 0x10000,
	                                 MMU_TARGET_FLASH0, MMU_MEM_CAP_READ, 0, &dataVaddr);
	std::printf("GEADEV:LOADTEST code=%p(%s) data=%p(%s)\n", codeVaddr, esp_err_to_name(ce), dataVaddr,
	            esp_err_to_name(de));
	if (ce != ESP_OK || de != ESP_OK || !codeVaddr || !dataVaddr) {
		if (codeVaddr) esp_mmu_unmap(codeVaddr);
		if (dataVaddr) esp_mmu_unmap(dataVaddr);
		return;
	}
	// The app is fixed-base linked to these exact vaddrs (code 0x43060000,
	// data 0x3d060000); only jump if the runtime mapping matched the link.
	if (reinterpret_cast<uintptr_t>(codeVaddr) != 0x43060000u ||
	    reinterpret_cast<uintptr_t>(dataVaddr) != 0x3d070000u) {
		std::printf("GEADEV:LOADTEST vaddr-mismatch (want code=0x43060000 data=0x3d070000); not jumping\n");
		esp_mmu_unmap(codeVaddr);
		esp_mmu_unmap(dataVaddr);
		return;
	}
	const uint32_t *hdr = static_cast<const uint32_t *>(codeVaddr);  // header in code page (32-bit reads ok on IBUS)
	std::printf("GEADEV:LOADTEST magic=0x%08x abi=%u entry_off=0x%x\n", static_cast<unsigned>(hdr[0]),
	            static_cast<unsigned>(hdr[1]), static_cast<unsigned>(hdr[2]));
	if (hdr[0] != 0x47454131u) {
		std::printf("GEADEV:LOADTEST bad-magic; not jumping\n");
		esp_mmu_unmap(codeVaddr);
		esp_mmu_unmap(dataVaddr);
		return;
	}
	using AppEntry = int (*)(void (*)(const char *));
	const AppEntry entry = reinterpret_cast<AppEntry>(reinterpret_cast<uintptr_t>(codeVaddr) + hdr[2]);
	std::printf("GEADEV:LOADTEST calling entry @%p ...\n", reinterpret_cast<void *>(entry));
	const int ret = entry(&loadtest_log);
	std::printf("GEADEV:LOADTEST entry returned %d (expected 42)\n", ret);
	esp_mmu_unmap(codeVaddr);
	esp_mmu_unmap(dataVaddr);
}

// ── GEADEV PUSH: stream a file from the host to the device's microSD ─────────
// Raw-binary file upload over the same serial command channel (on the P4 this
// IS the USB-Serial-JTAG CDC endpoint, so this is the "CDC bulk" path — no
// hex/base64, one byte per byte). The host sends:
//   GEADEV PUSH <dest-path> <size> [crc32]\n
// then exactly <size> raw bytes. The device opens the file, replies
// "GEADEV:PUSH READY", drains <size> bytes straight off stdin into the file,
// verifies CRC32 (zlib/binascii-compatible), and replies "GEADEV:PUSH OK" or
// "GEADEV:PUSH ERR ...". Loads a region's .pmtiles onto /sdcard without
// removing the card.

// Streaming CRC32 (poly 0xEDB88320). `crc` runs un-inverted: init 0xFFFFFFFF,
// invert the final result to match zlib/binascii.crc32.
std::uint32_t crc32Stream(std::uint32_t crc, const std::uint8_t *data, std::size_t n)
{
	for (std::size_t i = 0; i < n; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) crc = (crc & 1) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
	}
	return crc;
}

int base64Value(char ch)
{
	if (ch >= 'A' && ch <= 'Z') return ch - 'A';
	if (ch >= 'a' && ch <= 'z') return ch - 'a' + 26;
	if (ch >= '0' && ch <= '9') return ch - '0' + 52;
	if (ch == '+') return 62;
	if (ch == '/') return 63;
	return -1;
}

bool decodeBase64Char(char ch, std::uint8_t *out, long size, long &got,
                      int *quartet, int &quartetLen, int &padding)
{
	if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') return true;
	if (ch == '=') {
		quartet[quartetLen++] = 0;
		padding++;
	} else {
		const int value = base64Value(ch);
		if (value < 0 || padding > 0) return false;
		quartet[quartetLen++] = value;
	}
	if (quartetLen < 4) return true;

	const std::uint32_t triple =
	    (static_cast<std::uint32_t>(quartet[0]) << 18) |
	    (static_cast<std::uint32_t>(quartet[1]) << 12) |
	    (static_cast<std::uint32_t>(quartet[2]) << 6) |
	    static_cast<std::uint32_t>(quartet[3]);
	const int bytes = padding == 0 ? 3 : (padding == 1 ? 2 : 1);
	for (int i = 0; i < bytes; ++i) {
		if (got >= size) return false;
		out[got++] = static_cast<std::uint8_t>((triple >> (16 - i * 8)) & 0xff);
	}
	quartetLen = 0;
	padding = 0;
	return true;
}

// mkdir -p of the parent directories in `path` (FAT/VFS); existing dirs are fine.
void makeParentDirs(const char *path)
{
	char buf[160];
	std::snprintf(buf, sizeof(buf), "%s", path);
	for (char *p = buf + 1; *p; p++) {
		if (*p == '/') {
			*p = '\0';
			::mkdir(buf, 0777);
			*p = '/';
		}
	}
}

// A board whose file storage can be rebuilt (the Mosaico's NAND) overrides
// this: erase the medium and lay down a fresh filesystem at `mount`. 0 = done.
extern "C" __attribute__((weak)) int gea_board_storage_format(const char *mount)
{
	(void)mount;
	return -1;
}

// GEADEV FORMAT <mount>: the recovery for a volume that reads but no longer
// writes (an interrupted write can leave FAT unusable): everything on it is
// lost, so the host re-pushes what it needs.
void handleFormat(char *args)
{
	char *mount = nextToken(args);
	if (!mount) {
		std::printf("GEADEV:FORMAT ERR usage=GEADEV_FORMAT_mount\n");
		return;
	}
	std::printf("GEADEV:FORMAT BUSY mount=%s\n", mount);
	std::fflush(stdout);
	const int rc = gea_board_storage_format(mount);
	if (rc != 0) std::printf("GEADEV:FORMAT ERR mount=%s rc=%d\n", mount, rc);
	else std::printf("GEADEV:FORMAT OK mount=%s\n", mount);
}

void handlePush(char *args)
{
	char *pathTok = nextToken(args);
	char *sizeTok = nextToken(args);
	char *crcTok = nextToken(args);
	if (!pathTok || !sizeTok) {
		std::printf("GEADEV:PUSH ERR usage=GEADEV_PUSH_path_size_[crc32]\n");
		return;
	}
	char *end = nullptr;
	const long size = std::strtol(sizeTok, &end, 10);
	if (!end || *end != '\0' || size < 0) {
		std::printf("GEADEV:PUSH ERR bad-size\n");
		return;
	}
	std::uint32_t expectCrc = 0;
	const bool haveCrc = crcTok != nullptr;
	if (haveCrc) expectCrc = static_cast<std::uint32_t>(std::strtoul(crcTok, nullptr, 0));

	char path[160];
	std::snprintf(path, sizeof(path), "%s", pathTok);
	// Only the SD mount needs lazy initialization. SPIFFS is already mounted
	// at /storage; an absent SD card must not block writes to internal flash.
	if (std::strncmp(path, "/sdcard/", 8) == 0 && !gea::platform::storage::ensureMounted()) {
		std::printf("GEADEV:PUSH ERR no-storage path=%s\n", path);
		return;
	}
	makeParentDirs(path);
	std::FILE *f = std::fopen(path, "wb");
	if (!f) {
		std::printf("GEADEV:PUSH ERR open-failed path=%s errno=%d\n", path, errno);
		return;
	}
	// Read the whole file CONTINUOUSLY into a PSRAM buffer (no per-chunk ACK
	// round-trips — those stalled — and no SD writes mid-read, so the read never
	// pauses). The flow-control-less console RX buffer can't overflow because the
	// HOST throttles its send rate below our drain rate. SD is written once at
	// the end. (Buffer fits PSRAM for region-sized archives.)
	std::printf("GEADEV:PUSH READY path=%s bytes=%ld\n", path, size);
	std::fflush(stdout);

	std::uint8_t *mem = size > 0 ? static_cast<std::uint8_t *>(heap_caps_malloc(static_cast<std::size_t>(size), MALLOC_CAP_SPIRAM)) : nullptr;
	if (size > 0 && !mem) {
		std::fclose(f);
		std::printf("GEADEV:PUSH ERR no-memory bytes=%ld\n", size);
		return;
	}
	long got = 0;
	int stalls = 0;
	bool ioError = false;
	while (got < size) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
		// Binary payloads must bypass the text-oriented stdin VFS, just like
		// command bytes. It can block or translate JPEG bytes in a raw upload.
		const int received = usb_serial_jtag_read_bytes(mem + got, static_cast<std::size_t>(size - got), pdMS_TO_TICKS(50));
		const std::size_t n = received > 0 ? static_cast<std::size_t>(received) : 0;
#else
		const std::size_t n = std::fread(mem + got, 1, static_cast<std::size_t>(size - got), stdin);
#endif
		if (n == 0) {
			std::clearerr(stdin);  // an idle read sets the sticky EOF flag
			if (++stalls >
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
				800
#else
				20000
#endif
			) {  // ~40s of silence → abort
				ioError = true;
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(2));
			continue;
		}
		stalls = 0;
		got += static_cast<long>(n);
	}
	std::uint32_t finalCrc = 0;
	if (!ioError) {
		const std::size_t wrote = std::fwrite(mem, 1, static_cast<std::size_t>(size), f);
		if (wrote != static_cast<std::size_t>(size)) ioError = true;
		else finalCrc = crc32Stream(0xFFFFFFFFu, mem, static_cast<std::size_t>(size)) ^ 0xFFFFFFFFu;
	}
	// fwrite only fills the stdio buffer: the medium is written by the flush
	// (and the close). A full or failing volume shows up here, not above, and
	// left unchecked it leaves an empty file behind a "PUSH OK".
	const int flushed = std::fflush(f);
	const int flushErrno = errno;
	const int closed = std::fclose(f);
	const int closeErrno = errno;
	if (mem) heap_caps_free(mem);
	if (ioError) {
		std::printf("GEADEV:PUSH ERR transfer-failed path=%s got=%ld\n", path, got);
		return;
	}
	if (flushed != 0 || closed != 0) {
		std::uint64_t total = 0, free = 0;
#if GEA_DEVICE_CONTROL_FAT_DIAGNOSTICS
		if (std::strncmp(path, "/nand/", 6) == 0) esp_vfs_fat_info("/nand", &total, &free);
#endif
		std::printf("GEADEV:PUSH ERR write-failed path=%s flush=%d errno=%d close=%d errno=%d nandKB=%llu freeKB=%llu\n", path, flushed, flushErrno, closed, closeErrno,
		            static_cast<unsigned long long>(total / 1024), static_cast<unsigned long long>(free / 1024));
		return;
	}
	if (haveCrc && finalCrc != expectCrc) {
		std::printf("GEADEV:PUSH ERR crc-mismatch path=%s got=0x%08x want=0x%08x\n",
		            path, static_cast<unsigned>(finalCrc), static_cast<unsigned>(expectCrc));
		return;
	}
	std::printf("GEADEV:PUSH OK path=%s bytes=%ld crc=0x%08x\n", path, size, static_cast<unsigned>(finalCrc));
}

// ── GEADEV OTA: install a new app image over the USB console ─────────────────
// The PUSH transfer, written to the next OTA slot instead of a file:
//   GEADEV OTA <size> <crc32>\n
// then exactly <size> raw bytes. The image is drained into PSRAM first (the
// console has no flow control, so flash erases must not stall the read),
// checked against the CRC, written with esp_ota_*, selected for boot, and the
// board restarts into it. This is how a board whose USB port is not a ROM
// console -- the ESP32-S31's OTG port -- is flashed without the BOOT button.
// Compiled only for boards that ask for it (GEA_DEVICE_CONTROL_USB_OTA).
#if GEA_DEVICE_CONTROL_USB_OTA
// The fallback when the image cannot be staged: a firmware that runs from
// PSRAM (XIP) of about its own size leaves no room for the next image, so
// Skytail could only be updated from a smaller app. Each 64 KB read goes
// straight to flash, erased sector by sector as it is written: a whole-slot
// erase up front would hold READY past the host's 15 s wait. stdin reads the TinyUSB CDC FIFO, which leaves
// the OUT endpoint un-armed while it is full, so the host waits out a write
// instead of losing bytes. The boot slot only changes after the CRC matches; a
// bad transfer aborts and the running slot stays the boot slot.
static void handleOtaStreamed(const esp_partition_t *slot, long size, std::uint32_t expectCrc)
{
	constexpr std::size_t kChunk = 64 * 1024;
	auto *buffer = static_cast<std::uint8_t *>(heap_caps_malloc(kChunk, MALLOC_CAP_8BIT));
	if (!buffer) {
		std::printf("GEADEV:OTA ERR no-memory bytes=%ld\n", size);
		return;
	}
	esp_ota_handle_t handle = 0;
	esp_err_t err = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle);
	if (err != ESP_OK) {
		heap_caps_free(buffer);
		std::printf("GEADEV:OTA ERR write-failed slot=%s err=%s\n", slot->label, esp_err_to_name(err));
		return;
	}
	std::printf("GEADEV:OTA READY slot=%s bytes=%ld\n", slot->label, size);
	std::fflush(stdout);

	std::uint32_t crc = 0xFFFFFFFFu;
	long got = 0;
	int stalls = 0;
	while (got < size && err == ESP_OK) {
		const std::size_t want = std::min(kChunk, static_cast<std::size_t>(size - got));
		std::size_t filled = 0;
		while (filled < want) {
			const std::size_t n = std::fread(buffer + filled, 1, want - filled, stdin);
			if (n == 0) {
				std::clearerr(stdin);
				if (++stalls > 5000) break;
				vTaskDelay(pdMS_TO_TICKS(2));
				continue;
			}
			stalls = 0;
			filled += n;
		}
		if (filled < want) break;
		crc = crc32Stream(crc, buffer, filled);
		err = esp_ota_write(handle, buffer, filled);
		got += static_cast<long>(filled);
	}
	heap_caps_free(buffer);
	crc ^= 0xFFFFFFFFu;
	if (got != size || err != ESP_OK) {
		esp_ota_abort(handle);
		if (err != ESP_OK) std::printf("GEADEV:OTA ERR write-failed slot=%s err=%s\n", slot->label, esp_err_to_name(err));
		else std::printf("GEADEV:OTA ERR transfer-failed got=%ld\n", got);
		return;
	}
	if (crc != expectCrc) {
		esp_ota_abort(handle);
		std::printf("GEADEV:OTA ERR crc-mismatch got=0x%08x want=0x%08x\n",
		            static_cast<unsigned>(crc), static_cast<unsigned>(expectCrc));
		return;
	}
	err = esp_ota_end(handle);
	if (err == ESP_OK) err = esp_ota_set_boot_partition(slot);
	if (err != ESP_OK) {
		std::printf("GEADEV:OTA ERR write-failed slot=%s err=%s\n", slot->label, esp_err_to_name(err));
		return;
	}
	std::printf("GEADEV:OTA OK slot=%s bytes=%ld crc=0x%08x\n", slot->label, size, static_cast<unsigned>(crc));
	std::fflush(stdout);
	vTaskDelay(pdMS_TO_TICKS(200));
	esp_restart();
}

void handleOta(char *args)
{
	char *sizeTok = nextToken(args);
	char *crcTok = nextToken(args);
	if (!sizeTok || !crcTok) {
		std::printf("GEADEV:OTA ERR usage=GEADEV_OTA_size_crc32\n");
		return;
	}
	char *end = nullptr;
	const long size = std::strtol(sizeTok, &end, 10);
	if (!end || *end != '\0' || size <= 0) {
		std::printf("GEADEV:OTA ERR bad-size\n");
		return;
	}
	const std::uint32_t expectCrc = static_cast<std::uint32_t>(std::strtoul(crcTok, nullptr, 0));
	const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);
	if (!slot) {
		std::printf("GEADEV:OTA ERR no-ota-slot\n");
		return;
	}
	if (static_cast<std::size_t>(size) > slot->size) {
		std::printf("GEADEV:OTA ERR too-large bytes=%ld slot=%u\n", size, static_cast<unsigned>(slot->size));
		return;
	}
	// Staged in 1 MB pieces, not one image-sized block: a board that runs its
	// firmware from PSRAM (XIP) never has a free block as large as its own
	// image -- an 8 MB Skytail image met a 6.8 MB largest block on a fresh boot.
	constexpr std::size_t kStage = 1024 * 1024;
	const std::size_t pieces = (static_cast<std::size_t>(size) + kStage - 1) / kStage;
	std::vector<std::uint8_t *> stage(pieces, nullptr);
	const auto freeStage = [&stage] {
		for (std::uint8_t *piece : stage) heap_caps_free(piece);
	};
	for (std::size_t i = 0; i < pieces; i++) {
		const std::size_t length = std::min(kStage, static_cast<std::size_t>(size) - i * kStage);
		stage[i] = static_cast<std::uint8_t *>(heap_caps_malloc(length, MALLOC_CAP_SPIRAM));
		if (!stage[i]) {
			freeStage();
			handleOtaStreamed(slot, size, expectCrc);
			return;
		}
	}
	std::printf("GEADEV:OTA READY slot=%s bytes=%ld\n", slot->label, size);
	std::fflush(stdout);

	long got = 0;
	int stalls = 0;
	while (got < size) {
		const std::size_t piece = static_cast<std::size_t>(got) / kStage;
		const std::size_t at = static_cast<std::size_t>(got) % kStage;
		const std::size_t want = std::min(kStage - at, static_cast<std::size_t>(size - got));
		const std::size_t n = std::fread(stage[piece] + at, 1, want, stdin);
		if (n == 0) {
			std::clearerr(stdin);  // an idle read sets the sticky EOF flag
			if (++stalls > 5000) break;  // ~10s of silence
			vTaskDelay(pdMS_TO_TICKS(2));
			continue;
		}
		stalls = 0;
		got += static_cast<long>(n);
	}
	if (got != size) {
		freeStage();
		std::printf("GEADEV:OTA ERR transfer-failed got=%ld\n", got);
		return;
	}
	std::uint32_t crc = 0xFFFFFFFFu;
	for (std::size_t i = 0; i < pieces; i++)
		crc = crc32Stream(crc, stage[i], std::min(kStage, static_cast<std::size_t>(size) - i * kStage));
	crc ^= 0xFFFFFFFFu;
	if (crc != expectCrc) {
		freeStage();
		std::printf("GEADEV:OTA ERR crc-mismatch got=0x%08x want=0x%08x\n",
		            static_cast<unsigned>(crc), static_cast<unsigned>(expectCrc));
		return;
	}

	esp_ota_handle_t handle = 0;
	esp_err_t err = esp_ota_begin(slot, static_cast<std::size_t>(size), &handle);
	if (err == ESP_OK) {
		constexpr std::size_t kChunk = 64 * 1024;  // divides kStage: no write spans two pieces
		for (long offset = 0; offset < size && err == ESP_OK; offset += static_cast<long>(kChunk)) {
			const std::size_t n = std::min<std::size_t>(kChunk, static_cast<std::size_t>(size - offset));
			const std::size_t at = static_cast<std::size_t>(offset);
			err = esp_ota_write(handle, stage[at / kStage] + at % kStage, n);
		}
		const esp_err_t ended = esp_ota_end(handle);
		if (err == ESP_OK) err = ended;
	}
	freeStage();
	if (err == ESP_OK) err = esp_ota_set_boot_partition(slot);
	if (err != ESP_OK) {
		std::printf("GEADEV:OTA ERR write-failed slot=%s err=%s\n", slot->label, esp_err_to_name(err));
		return;
	}
	std::printf("GEADEV:OTA OK slot=%s bytes=%ld crc=0x%08x\n", slot->label, size, static_cast<unsigned>(crc));
	std::fflush(stdout);
	vTaskDelay(pdMS_TO_TICKS(200));  // let the reply reach the host before the reset
	esp_restart();
}
#endif  // GEA_DEVICE_CONTROL_USB_OTA

void handlePushBase64(char *args)
{
	char *pathTok = nextToken(args);
	char *sizeTok = nextToken(args);
	char *crcTok = nextToken(args);
	if (!pathTok || !sizeTok) {
		std::printf("GEADEV:PUSH64 ERR usage=GEADEV_PUSH64_path_size_[crc32]\n");
		return;
	}
	char *end = nullptr;
	const long size = std::strtol(sizeTok, &end, 10);
	if (!end || *end != '\0' || size < 0) {
		std::printf("GEADEV:PUSH64 ERR bad-size\n");
		return;
	}
	std::uint32_t expectCrc = 0;
	const bool haveCrc = crcTok != nullptr;
	if (haveCrc) expectCrc = static_cast<std::uint32_t>(std::strtoul(crcTok, nullptr, 0));

	char path[160];
	std::snprintf(path, sizeof(path), "%s", pathTok);
	if (std::strncmp(path, "/sdcard/", 8) == 0 && !gea::platform::storage::ensureMounted()) {
		std::printf("GEADEV:PUSH64 ERR no-storage path=%s\n", path);
		return;
	}
	makeParentDirs(path);
	std::FILE *f = std::fopen(path, "wb");
	if (!f) {
		std::printf("GEADEV:PUSH64 ERR open-failed path=%s errno=%d\n", path, errno);
		return;
	}

	std::uint8_t *mem = size > 0 ? static_cast<std::uint8_t *>(heap_caps_malloc(static_cast<std::size_t>(size), MALLOC_CAP_SPIRAM)) : nullptr;
	if (size > 0 && !mem) {
		std::fclose(f);
		std::printf("GEADEV:PUSH64 ERR no-memory bytes=%ld\n", size);
		return;
	}

	std::printf("GEADEV:PUSH64 READY path=%s bytes=%ld\n", path, size);
	std::fflush(stdout);

	long got = 0;
	bool ioError = false;
	int quartet[4] = {0, 0, 0, 0};
	int quartetLen = 0;
	int padding = 0;
	char line[600];
	while (got < size) {
		while (!std::fgets(line, sizeof(line), stdin)) {
			if (std::ferror(stdin)) std::clearerr(stdin);
			vTaskDelay(pdMS_TO_TICKS(2));
		}
		for (char *p = line; *p; ++p) {
			if (!decodeBase64Char(*p, mem, size, got, quartet, quartetLen, padding)) {
				ioError = true;
				break;
			}
		}
		if (ioError) break;
	}

	std::uint32_t finalCrc = 0;
	if (!ioError && got == size) {
		std::uint32_t crc = 0xFFFFFFFFu;
		long writtenTotal = 0;
		while (writtenTotal < size) {
			const std::size_t remaining = static_cast<std::size_t>(size - writtenTotal);
			const std::size_t chunk = remaining > 4096 ? 4096 : remaining;
			const std::size_t wrote = std::fwrite(mem + writtenTotal, 1, chunk, f);
			if (wrote != chunk) {
				ioError = true;
				break;
			}
			crc = crc32Stream(crc, mem + writtenTotal, chunk);
			writtenTotal += static_cast<long>(chunk);
		}
		finalCrc = crc ^ 0xFFFFFFFFu;
	} else {
		ioError = true;
	}
	std::fclose(f);
	if (mem) heap_caps_free(mem);
	if (ioError) {
		std::printf("GEADEV:PUSH64 ERR transfer-failed path=%s got=%ld errno=%d\n", path, got, errno);
		return;
	}
	if (haveCrc && finalCrc != expectCrc) {
		std::printf("GEADEV:PUSH64 ERR crc-mismatch path=%s got=0x%08x want=0x%08x\n",
		            path, static_cast<unsigned>(finalCrc), static_cast<unsigned>(expectCrc));
		return;
	}
	std::printf("GEADEV:PUSH64 OK path=%s bytes=%ld crc=0x%08x\n", path, size, static_cast<unsigned>(finalCrc));
}

void handleListFiles(char *args)
{
	const char *pathTok = nextToken(args);
	const char *path = pathTok && pathTok[0] ? pathTok : "/sdcard";
	if (!gea::platform::storage::ensureMounted()) {
		std::printf("GEADEV:LS ERR no-storage path=%s\n", path);
		return;
	}
	DIR *dir = opendir(path);
	if (!dir) {
		std::printf("GEADEV:LS ERR open-failed path=%s\n", path);
		return;
	}
	std::printf("GEADEV:LS BEGIN path=%s\n", path);
	while (true) {
		struct dirent *entry = readdir(dir);
		if (!entry) break;
		if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) continue;
		std::string fullPath = std::string(path) + "/" + entry->d_name;
		struct stat st {};
		const long size = stat(fullPath.c_str(), &st) == 0 ? static_cast<long>(st.st_size) : -1;
		const char *type = size >= 0 && S_ISDIR(st.st_mode) ? "dir" : "file";
		std::printf("GEADEV:LS ENTRY name=%s type=%s size=%ld\n", entry->d_name, type, size);
	}
	closedir(dir);
	std::printf("GEADEV:LS END path=%s\n", path);
}

void handleRemoveFile(char *args)
{
	const char *path = nextToken(args);
	if (!path || path[0] == '\0') {
		std::printf("GEADEV:RM ERR usage=GEADEV_RM_path\n");
		return;
	}
	if (!gea::platform::storage::ensureMounted()) {
		std::printf("GEADEV:RM ERR no-storage path=%s\n", path);
		return;
	}
	const int rc = std::remove(path);
	if (rc != 0) {
		std::printf("GEADEV:RM ERR remove-failed path=%s errno=%d\n", path, errno);
		return;
	}
	std::printf("GEADEV:RM OK path=%s\n", path);
}

void handlePull(char *args)
{
	const char *path = nextToken(args);
	if (!path || path[0] == '\0') {
		std::printf("GEADEV:PULL ERR usage=GEADEV_PULL_path\n");
		return;
	}
	if (!gea::platform::storage::ensureMounted()) {
		std::printf("GEADEV:PULL ERR no-storage path=%s\n", path);
		return;
	}
	std::FILE *f = std::fopen(path, "rb");
	if (!f) {
		std::printf("GEADEV:PULL ERR open-failed path=%s\n", path);
		return;
	}
	std::fseek(f, 0, SEEK_END);
	const long size = std::ftell(f);
	std::fseek(f, 0, SEEK_SET);
	flockfile(stdout);
	std::printf("GEADEV:PULL BEGIN path=%s bytes=%ld encoding=base64\n", path, size);
	Base64Writer writer;
	std::uint32_t crc = 0xFFFFFFFFu;
	std::uint8_t buffer[384];
	long total = 0;
	while (true) {
		const std::size_t got = std::fread(buffer, 1, sizeof(buffer), f);
		if (got == 0) break;
		crc = crc32Stream(crc, buffer, got);
		for (std::size_t i = 0; i < got; i++) writer.writeByte(buffer[i]);
		total += static_cast<long>(got);
	}
	writer.finish();
	std::fclose(f);
	const std::uint32_t finalCrc = crc ^ 0xFFFFFFFFu;
	std::printf("GEADEV:PULL END path=%s bytes=%ld crc=0x%08x\n", path, total, static_cast<unsigned>(finalCrc));
	std::fflush(stdout);
	funlockfile(stdout);
}

void handlePlayFile(char *args)
{
	const char *path = nextToken(args);
	if (!path || path[0] == '\0') {
		std::printf("GEADEV:PLAYFILE ERR usage=GEADEV_PLAYFILE_path\n");
		return;
	}
	const bool ok = gea::platform::audio::AudioSystem::playFile(std::string(path));
	std::printf("GEADEV:PLAYFILE %s path=%s\n", ok ? "OK" : "ERR", path);
}

void printTaskDiagnostics(bool networkOnly)
{
#if configUSE_TRACE_FACILITY && INCLUDE_xTaskGetHandle
	struct Sample {
		const char *name;
		UBaseType_t taskNumber = 0;
		configRUN_TIME_COUNTER_TYPE cpu = 0;
		std::int64_t at = 0;
	};
	static Sample samples[] = {{"wifi"}, {"tcpip"}, {"websocket_task"}, {"gea-ws-send"},
	    {"gea_runtime"}, {"app_frame"}, {"gea_mic"}, {"gea_audio"},
	    {"gea_mjpeg_dec"}, {"gea_mjpeg_rx"}, {"gea_rwrk"}, {"IDLE0"}, {"IDLE1"}};
	const char *prefix = networkOnly ? "NETDIAG" : "TASKSTATS";
	unsigned index = 0;
	for (auto &sample : samples) {
		if (networkOnly && index++ >= 4) break;
		const TaskHandle_t task = xTaskGetHandle(sample.name);
		if (!task) {
			std::printf("GEADEV:%s task=%s absent=1\n", prefix, sample.name);
			sample.at = 0;
			continue;
		}
		TaskStatus_t info{};
		// Inspect known media/runtime tasks only, without scanning their stacks.
		vTaskGetInfo(task, &info, pdFALSE, eInvalid);
		const auto now = esp_timer_get_time();
		const bool previous = sample.at && sample.taskNumber == info.xTaskNumber;
		const char *state = "invalid";
		switch (info.eCurrentState) {
			case eRunning: state = "running"; break;
			case eReady: state = "ready"; break;
			case eBlocked: state = "blocked"; break;
			case eSuspended: state = "suspended"; break;
			case eDeleted: state = "deleted"; break;
			default: break;
		}
		std::printf("GEADEV:%s task=%s id=%u state=%s priority=%u base_priority=%u core=%d cpu_ticks=%llu delta_ticks=%llu span_us=%lld\n",
		    prefix, sample.name, static_cast<unsigned>(info.xTaskNumber), state,
		    static_cast<unsigned>(info.uxCurrentPriority),
		    static_cast<unsigned>(info.uxBasePriority),
		    static_cast<int>(xTaskGetCoreID(task)),
		    static_cast<unsigned long long>(info.ulRunTimeCounter),
		    static_cast<unsigned long long>(previous ? info.ulRunTimeCounter - sample.cpu : 0),
		    static_cast<long long>(previous ? now - sample.at : 0));
		sample.taskNumber = info.xTaskNumber;
		sample.cpu = info.ulRunTimeCounter;
		sample.at = now;
	}
#else
	std::printf("GEADEV:NETDIAG task-stats-unavailable\n");
#endif
}

void printNetworkDiagnostics()
{
	std::printf("GEADEV:NETDIAG BEGIN\n");
	printTaskDiagnostics(true);
#if CONFIG_ESP_WIFI_ENABLED
	// The Wi-Fi driver owns these counters; use its public diagnostic entry.
	// Emit the marker first so USB also locates a blocked driver dump.
	std::printf("GEADEV:NETDIAG wifi-dump-begin\n");
	std::fflush(stdout);
	const auto result = esp_wifi_statis_dump(
	    WIFI_STATIS_BUFFER | WIFI_STATIS_RXTX | WIFI_STATIS_HW | WIFI_STATIS_DIAG);
	std::printf("GEADEV:NETDIAG wifi-dump-result=%s\n", esp_err_to_name(result));
#else
	std::printf("GEADEV:NETDIAG native-wifi-unavailable\n");
#endif
	std::printf("GEADEV:NETDIAG END\n");
}

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
namespace diagnostic_gesture = gea::platform::comparison::gesture;
namespace diagnostic_input = gea::platform::comparison::input;
static diagnostic_gesture::Plan *gDiagnosticGesture = nullptr;
static bool gDiagnosticTouching = false;

void diagnosticJsonString(const char *value)
{
	std::putchar('"');
	for (const unsigned char *p = reinterpret_cast<const unsigned char *>(value); *p; ++p) {
		if (*p == '"' || *p == '\\') {
			std::printf("\\%c", *p);
		} else if (*p < 32) {
			std::printf("\\u%04x", unsigned(*p));
		} else {
			std::putchar(*p);
		}
	}
	std::putchar('"');
}

void collectDiagnosticScroll(diagnostic_gesture::Observation &sample, int filter = -1, bool gestureOnly = false)
{
	gea::framework::services::AppState::lock();
	sample.actualUs = esp_timer_get_time();
	sample.count = sample.dropped = 0;
	auto &tree = gea::embedded::ui::Tree::instance();
	const int root = tree.mountedRoot();
	const int count = tree.nodeCount();
	for (int id = 0; id < count; ++id) {
		const auto &node = tree.node(id);
		if (GEA_DIAGNOSTIC_NODE_STYLE(node).display == gea::embedded::ui::kDisplayNone) {
			continue;
		}
		bool mounted = false;
		for (int parent = id, guard = 0; parent >= 0 && parent < count && guard++ < count;
			 parent = tree.node(parent).parent) {
			if (GEA_DIAGNOSTIC_NODE_STYLE(tree.node(parent)).display == gea::embedded::ui::kDisplayNone) {
				break;
			}
			if (parent == root) {
				mounted = true;
				break;
			}
		}
		if (!mounted) {
			continue;
		}
		int kind = -1;
		for (unsigned index = 0;
			 index < (gestureOnly ? diagnostic_gesture::kGestureClassCount : std::size(diagnostic_gesture::kClasses));
			 ++index) {
			if ((filter < 0 || filter == int(index)) && tree.hasClass(id, diagnostic_gesture::kClasses[index])) {
				kind = int(index);
				break;
			}
		}
		if (kind < 0) {
			continue;
		}
		if (sample.count == diagnostic_gesture::kNodeCapacity) {
			++sample.dropped;
			continue;
		}
		auto &entry = sample.nodes[sample.count++];
		entry.id = id;
		entry.kind = kind;
		entry.parent = node.parent;
		entry.x = node.layout.x;
		entry.y = node.layout.y;
		entry.width = node.layout.width;
		entry.height = node.layout.height;
		entry.scrollX = node.layout.scroll_x;
		entry.scrollY = node.layout.scroll_y;
		entry.contentWidth = node.layout.scroll_content_width;
		entry.contentHeight = node.layout.scroll_content_height;
		entry.selected = tree.hasClass(id, "selected") || tree.hasClass(id, "roller-selected-track") ||
								 (tree.hasClass(id, "switch") && tree.hasClass(id, "on"))
							 ? 1
							 : 0;
		entry.children = 0;
		entry.textTruncated = false;
		entry.text[0] = '\0';
		for (int child = node.first_child, guard = 0; child >= 0 && child < count && guard++ < count;
			 child = tree.node(child).next_sibling) {
			++entry.children;
		}
		std::size_t length = 0;
		// Traverse actual descendants without allocations. Keep bounded text and
		// explicitly disclose truncation; never infer application state privately.
		for (int descendant = id, guard = 0; descendant >= 0 && descendant < count && guard++ < count;) {
			const auto &current = tree.node(descendant);
			const auto &text = current.text;
			if (!text.empty()) {
				const auto available = sizeof(entry.text) - 1 - length;
				const auto copied = std::min(available, text.size());
				std::memcpy(entry.text + length, text.data(), copied);
				length += copied;
				entry.text[length] = '\0';
				entry.textTruncated |= copied != text.size();
			}
			if (current.first_child >= 0 && current.first_child < count) {
				descendant = current.first_child;
				continue;
			}
			int climbed = 0;
			while (descendant != id && descendant >= 0 && descendant < count &&
				   tree.node(descendant).next_sibling < 0 && climbed++ < count) {
				descendant = tree.node(descendant).parent;
			}
			if (climbed >= count) {
				break;
			}
			if (descendant == id || descendant < 0 || descendant >= count) {
				break;
			}
			descendant = tree.node(descendant).next_sibling;
		}
		gea::embedded::ui::ViewRenderer::transformedRectCorners(node, false, node.layout.x, node.layout.y,
																node.layout.width, node.layout.height, entry.cornerX,
																entry.cornerY);
	}
	gea::framework::services::AppState::unlock();
}

void printDiagnosticNodes(const diagnostic_gesture::Observation &sample)
{
	std::putchar('[');
	for (unsigned index = 0; index < sample.count; ++index) {
		const auto &node = sample.nodes[index];
		std::printf("%s{\"id\":%d,\"parent\":%d,\"class\":", index ? "," : "", node.id, node.parent);
		diagnosticJsonString(diagnostic_gesture::kClasses[node.kind]);
		std::printf(
			",\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d,\"selected_class\":%d,\"scroll_x\":%d,\"scroll_y\":%d,"
			"\"content_width\":%d,\"content_height\":%d,\"children_count\":%d,\"text_truncated\":%s,\"corners\":[[%d,%"
			"d],[%d,%d],[%d,%d],[%d,%d]],\"text\":",
			node.x, node.y, node.width, node.height, node.selected, node.scrollX, node.scrollY, node.contentWidth,
			node.contentHeight, node.children, node.textTruncated ? "true" : "false", node.cornerX[0], node.cornerY[0],
			node.cornerX[1], node.cornerY[1], node.cornerX[2], node.cornerY[2], node.cornerX[3], node.cornerY[3]);
		diagnosticJsonString(node.text);
		std::putchar('}');
	}
	std::putchar(']');
}

void printDiagnosticInput(const diagnostic_input::Snapshot &trace)
{
	std::putchar('[');
	for (unsigned index = 0; index < trace.count; ++index) {
		const auto &entry = trace.entries[index];
		std::printf("%s[%lld,%d,%d,%d,%d,%d,%d,%d]", index ? "," : "", static_cast<long long>(entry.timestampUs),
					entry.phase, entry.touching, entry.x, entry.y, entry.pointerId, entry.handlerX, entry.handlerY);
	}
	std::putchar(']');
}

void releaseDiagnosticGesture()
{
	if (!gDiagnosticGesture) {
		return;
	}
	heap_caps_free(gDiagnosticGesture->pointerReads.entries);
	gDiagnosticGesture->~Plan();
	heap_caps_free(gDiagnosticGesture);
	gDiagnosticGesture = nullptr;
}

bool beginDiagnosticInput()
{
	constexpr unsigned capacity = 256;
	auto *storage = static_cast<diagnostic_input::Entry *>(
		heap_caps_malloc(sizeof(diagnostic_input::Entry) * capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!diagnostic_input::begin(storage, capacity)) {
		heap_caps_free(storage);
		return false;
	}
	return true;
}

void handleDiagnosticGesture(char *&cursor)
{
	const char *action = nextToken(cursor);
	if (gea::platform::comparison::enabled.load(std::memory_order_acquire) ||
		diagnostic_input::recording.load(std::memory_order_acquire) ||
		gea::platform::comparison::upload::armed.load(std::memory_order_acquire)) {
		std::puts("GEADEV:ERR GESTURE benchmark-input-trace-or-upload-active");
		return;
	}
	if (tokenEquals(action, "RESET")) {
		releaseDiagnosticGesture();
		void *storage = heap_caps_malloc(sizeof(diagnostic_gesture::Plan), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		if (!storage) {
			std::puts("GEADEV:ERR GESTURE allocation-failed");
			return;
		}
		gDiagnosticGesture = new (storage) diagnostic_gesture::Plan{};
		std::puts("GEADEV:OK GESTURE RESET");
	} else if (tokenEquals(action, "CLEAR")) {
		releaseDiagnosticGesture();
		std::puts("GEADEV:OK GESTURE CLEAR");
	} else if (!gDiagnosticGesture) {
		std::puts("GEADEV:ERR GESTURE reset-required");
	} else if (tokenEquals(action, "TOUCH")) {
		int ms, state, x, y;
		if (std::sscanf(cursor, "%d %d %d %d", &ms, &state, &x, &y) != 4 ||
			!gDiagnosticGesture->addTouch(ms, state, x, y)) {
			std::puts("GEADEV:ERR GESTURE invalid-touch");
			return;
		}
		std::puts("GEADEV:OK GESTURE TOUCH");
	} else if (tokenEquals(action, "OBSERVE")) {
		int ms;
		if (std::sscanf(cursor, "%d", &ms) != 1 || !gDiagnosticGesture->addObservation(ms)) {
			std::puts("GEADEV:ERR GESTURE invalid-observation");
			return;
		}
		std::puts("GEADEV:OK GESTURE OBSERVE");
	} else if (tokenEquals(action, "BEGIN")) {
		if (!gDiagnosticGesture->valid() || !beginDiagnosticInput()) {
			std::puts("GEADEV:ERR GESTURE invalid-plan-or-trace");
			return;
		}
		std::puts("GEADEV:OK GESTURE BEGIN");
		std::fflush(stdout);
		diagnostic_gesture::play(
			*gDiagnosticGesture, [] { return esp_timer_get_time(); },
			[](std::int64_t remaining) {
				if (remaining >= 1000) {
					vTaskDelay(pdMS_TO_TICKS(std::max<std::int64_t>(1, remaining / 1000)));
				} else {
					taskYIELD();
				}
			},
			[](int phase, bool touching, int x, int y) {
				gea::platform::touch::Touchscreen::injectEvent(static_cast<gea::platform::touch::Phase>(phase),
															   touching, x, y);
			},
			[](diagnostic_gesture::Observation &sample) { collectDiagnosticScroll(sample, -1, true); });
		gDiagnosticGesture->pointerReads = diagnostic_input::end();
	} else if (tokenEquals(action, "RESULT")) {
		const auto &plan = *gDiagnosticGesture;
		if (!plan.complete) {
			std::puts("GEADEV:ERR GESTURE incomplete");
			return;
		}
		std::printf(
			"SWGESTURE {\"framework\":\"gea\",\"clock_origin_us\":%lld,\"normal_input_authority\":\"TouchRuntime "
			"consumed dispatch\",\"coordinate_space\":\"logical\",\"events\":[",
			static_cast<long long>(plan.originUs));
		for (unsigned index = 0; index < plan.eventCount; ++index) {
			const auto &event = plan.events[index];
			std::printf("%s{\"requested_at_ms\":%d,\"actual_at_us\":%lld,\"state\":%d,\"phase\":%d,\"x\":%d,\"y\":%d}",
						index ? "," : "", event.requestedMs, static_cast<long long>(event.actualUs), event.state,
						event.phase, event.x, event.y);
		}
		std::printf("],\"pointer_reads_dropped\":%u,\"pointer_reads\":", plan.pointerReads.dropped);
		printDiagnosticInput(plan.pointerReads);
		std::printf(",\"observations\":[");
		for (unsigned index = 0; index < plan.observationCount; ++index) {
			const auto &sample = plan.observations[index];
			std::printf("%s{\"requested_at_ms\":%d,\"actual_at_us\":%lld,\"nodes_dropped\":%u,\"nodes\":",
						index ? "," : "", sample.requestedMs, static_cast<long long>(sample.actualUs), sample.dropped);
			printDiagnosticNodes(sample);
			std::putchar('}');
		}
		std::puts("]}");
	} else {
		std::puts("GEADEV:ERR GESTURE unknown-action");
	}
}

const char *diagnosticOptimization()
{
#if CONFIG_COMPILER_OPTIMIZATION_PERF
	return "O2";
#elif CONFIG_COMPILER_OPTIMIZATION_DEBUG
	return "Og";
#elif CONFIG_COMPILER_OPTIMIZATION_SIZE
	return "Os";
#elif CONFIG_COMPILER_OPTIMIZATION_NONE
	return "O0";
#else
	return "unknown";
#endif
}

bool lockDiagnosticUploadBoundary()
{
	gea::framework::services::AppState::lock();
	const auto start = esp_timer_get_time();
	while (gea_frame_capture_boundary_ready && !gea_frame_capture_boundary_ready()) {
		if (esp_timer_get_time() - start > 1000000) {
			gea::framework::services::AppState::unlock();
			return false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	if (!gea_frame_capture_boundary_ready || !gea_display_wait_for_uploads || !gea_display_wait_for_uploads()) {
		gea::framework::services::AppState::unlock();
		return false;
	}
	return true;
}

void handleDiagnosticUpload(char *&cursor)
{
	namespace upload = gea::platform::comparison::upload;
	if (gea::platform::comparison::enabled.load(std::memory_order_acquire) ||
		diagnostic_input::recording.load(std::memory_order_acquire)) {
		std::puts("GEADEV:ERR UPLOADSHOT benchmark-or-input-trace-active");
		return;
	}
	const char *action = nextToken(cursor);
	if (tokenEquals(action, "ARM")) {
		if (upload::armed.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR UPLOADSHOT already-armed");
			return;
		}
		const int width = gea::platform::display::kNativeWidth, height = gea::platform::display::kNativeHeight;
		const auto count = std::size_t(width) * height;
		auto *pixels = static_cast<std::uint16_t *>(
			heap_caps_malloc(count * 2 + (count + 7) / 8, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!pixels || !lockDiagnosticUploadBoundary()) {
			heap_caps_free(pixels);
			std::puts("GEADEV:ERR UPLOADSHOT no-memory-or-display-busy");
			return;
		}
		const bool started = upload::begin(pixels, width, height);
		if (started) {
			gea::embedded::ui::Tree::instance().markDisplayListDirty();
			gea::platform::display::Display::invalidate();
		}
		gea::framework::services::AppState::unlock();
		if (!started) {
			heap_caps_free(pixels);
			std::puts("GEADEV:ERR UPLOADSHOT unavailable");
			return;
		}
		std::puts("GEADEV:OK UPLOADSHOT ARM source=co5300-submitted-rgb565 normal-full-invalidate=1");
	} else if (tokenEquals(action, "DISARM")) {
		const auto image = upload::end();
		heap_caps_free(image.pixels);
		std::puts("GEADEV:OK UPLOADSHOT DISARM");
	} else if (!action || tokenEquals(action, "READ")) {
		if (!upload::armed.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR UPLOADSHOT arm-required");
			return;
		}
		if (!lockDiagnosticUploadBoundary()) {
			std::puts("GEADEV:ERR UPLOADSHOT display-busy");
			return;
		}
		const auto image = upload::end();
		const auto capturedUs = esp_timer_get_time();
		gea::framework::services::AppState::unlock();
		const auto count = std::size_t(image.width) * image.height;
		if (!image.pixels || image.covered != count || image.errors) {
			heap_caps_free(image.pixels);
			std::printf("GEADEV:ERR UPLOADSHOT incomplete covered=%u required=%u errors=%u\n", image.covered,
						unsigned(count), image.errors);
			return;
		}
		flockfile(stdout);
		std::printf("GEADEV:UPLOADSHOT BEGIN width=%d height=%d encoding=rgb565-rle-v1 source=co5300-submitted-rgb565 "
					"completed_dma=1 timestamp_us=%lld submitted_pixels=%llu\n",
					image.width, image.height, static_cast<long long>(capturedUs),
					static_cast<unsigned long long>(image.submittedPixels));
		Base64Writer writer;
		std::uint16_t value = image.pixels[0], run = 1;
		for (std::size_t index = 1; index < count; ++index) {
			if (image.pixels[index] == value && run < 65535) {
				++run;
				continue;
			}
			writeRun(writer, run, value);
			value = image.pixels[index];
			run = 1;
		}
		writeRun(writer, run, value);
		writer.finish();
		std::puts("GEADEV:UPLOADSHOT END");
		std::fflush(stdout);
		funlockfile(stdout);
		heap_caps_free(image.pixels);
	} else {
		std::puts("GEADEV:ERR UPLOADSHOT usage=ARM_READ_DISARM");
	}
}
#endif

// Optional per-target GEADEV command extension. The weak default (defined at the
// end of this file, outside the namespace) returns false; a target that wants
// extra commands — e.g. the ESP32-P4 camera capture (CAMSTILL / CAMCLIP) in
// targets/esp32-p4-waveshare-touch-lcd-7/main/camera.cpp — provides a strong
// override. Receives the command token plus the remaining argument cursor.
extern "C" bool geaHandleExtraDevCommand(const char *command, char *args);

void handleCommand(char *line, CommandSource source)
{
	char *cursor = line;
	char *prefix = nextToken(cursor);
	if (!prefix || !tokenEquals(prefix, "GEADEV")) return;

	char *command = nextToken(cursor);
	if (!command) {
		std::printf("GEADEV:ERR missing-command\n");
		return;
	}

	#if defined(GEA_NATIVE_DEBUGGER) && GEA_NATIVE_DEBUGGER
	if (tokenEquals(command, "DEBUG")) {
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
      ScopedLogSilence silence;
      gea::debugger::handle(cursor, &usbWriteAll);
#else
      gea::debugger::handle(cursor);
#endif
      return;
    }
#endif
	if (tokenEquals(command, "PING")) {
		// `gea boards discover` identifies a plugged-in unit from this one
		// line: the app it runs, the address it holds (0.0.0.0 without WiFi)
		// and the station MAC, which is also the USB serial an ESP32 board
		// enumerates with -- so the reply confirms which alias the port is.
		const char *appId = gea::framework::apps::AppManager::currentId();
		const std::string ip = gea::framework::network::wifi().ip();
		const std::string mac = gea::framework::network::wifi().mac();
		std::printf("GEADEV:PONG app=%s ip=%s mac=%s\n", appId ? appId : "", ip.c_str(), mac.c_str());
	} else if (tokenEquals(command, "NETDIAG")) {
		printNetworkDiagnostics();
	} else if (tokenEquals(command, "IPCSTACK")) {
#if INCLUDE_xTaskGetHandle && CONFIG_ESP_IPC_ENABLE
		// Inspect only the two small stock IPC stacks, while the device is idle.
		// Never scan the application's large PSRAM stack or every task in a room.
		for (const char *name : {"ipc0", "ipc1"}) {
			const TaskHandle_t task = xTaskGetHandle(name);
			if (!task) std::printf("GEADEV:IPCSTACK task=%s absent=1\n", name);
			else std::printf("GEADEV:IPCSTACK task=%s capacity_bytes=%u free_bytes=%u\n", name,
			    unsigned(CONFIG_ESP_IPC_TASK_STACK_SIZE), unsigned(uxTaskGetStackHighWaterMark(task)));
		}
		std::puts("GEADEV:IPCSTACK END");
#else
		std::puts("GEADEV:IPCSTACK unavailable");
#endif
	} else if (tokenEquals(command, "VP8PROFILE")) {
#if defined(GEA_VIDEO_SIMD_ESP32S3) && GEA_VIDEO_SIMD_ESP32S3
		bool enabled = false;
		if (!parseOnOff(nextToken(cursor), &enabled)) {
			std::puts("GEADEV:ERR VP8PROFILE expected-on-or-off");
		} else {
			gea_vp8_profile_enable_s3(enabled);
			std::printf("GEADEV:VP8PROFILE enabled=%u\n", unsigned(enabled));
		}
#else
		std::puts("GEADEV:ERR VP8PROFILE unavailable");
#endif
	} else if (tokenEquals(command, "APP")) {
		const char *appId = gea::framework::apps::AppManager::currentId();
		std::printf("GEADEV:APP id=%s\n", appId ? appId : "");
	} else if (tokenEquals(command, "TASKSTATS")) {
		std::puts("GEADEV:TASKSTATS BEGIN");
		printTaskDiagnostics(false);
		std::puts("GEADEV:TASKSTATS END");
	} else if (tokenEquals(command, "VIDEOSTATS")) {
		gea::framework::services::AppState::lock();
		const auto stats = gea::host::HTMLVideoElement::presentationStats();
		gea::framework::services::AppState::unlock();
		std::printf("GEADEV:VIDEOSTATS decoded=%u selected=%u max_gap_ms=%u dma_chunks=%u\n",
		    unsigned(stats.decoded), unsigned(stats.selected), unsigned(stats.maxGapMs),
		    unsigned(gea_display_completed_chunks ? gea_display_completed_chunks() : 0));
	} else if (tokenEquals(command, "VIDEO")) {
		// Replay a local diagnostic clip through the real video/render path,
		// without creating a hosted session or opening microphone hardware.
		static gea::host::HTMLVideoElement retainedVideo;
		const char *className = nextToken(cursor);
		const char *url = nextToken(cursor);
		if (!className || !url) {
			std::puts("GEADEV:ERR VIDEO usage=GEADEV_VIDEO_class_url|STOP");
		} else {
			gea::framework::services::AppState::lock();
			auto &tree = gea::embedded::ui::Tree::instance();
			bool found = false;
			for (int id = 0; id < tree.nodeCount(); ++id) {
				if (!tree.hasClass(id, className)) continue;
				const gea::embedded::ui::NodeHandle node(id);
				gea::host::HTMLVideoElement video{node};
				const bool stopping = tokenEquals(url, "STOP");
				node.classList().toggle("ready", !stopping);
				for (int cover = 0; cover < tree.nodeCount(); ++cover) {
					if (tree.hasClass(cover, "portrait"))
						gea::embedded::ui::NodeHandle(cover).classList().toggle("hidden", !stopping);
				}
				if (stopping) {
					video.pause();
					video.setSrcObject(nullptr);
					retainedVideo = {};
				} else {
					retainedVideo = video;
					video.setSrc(url);
					video.play();
				}
				found = true;
				break;
			}
			gea::framework::services::AppState::unlock();
			std::printf("GEADEV:VIDEO %s\n", found ? "OK" : "missing-node");
		}
	} else if (tokenEquals(command, "TOUCH")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		if (gea::platform::comparison::enabled.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR TOUCH benchmark-active");
			return;
		}
		const char *action = nextToken(cursor);
		int x, y, phase = 0;
		if (tokenEquals(action, "down")) {
			phase = 1;
		} else if (tokenEquals(action, "move") || tokenEquals(action, "2")) {
			phase = 2;
		} else if (tokenEquals(action, "up") || tokenEquals(action, "0")) {
			phase = 3;
		} else if (tokenEquals(action, "1")) {
			phase = gDiagnosticTouching ? 2 : 1;
		}
		if (!phase || std::sscanf(cursor, "%d %d", &x, &y) != 2) {
			std::puts("GEADEV:ERR TOUCH usage=down-move-up_x_y");
			return;
		}
		gDiagnosticTouching = phase != 3;
		gea::platform::touch::Touchscreen::injectEvent(static_cast<gea::platform::touch::Phase>(phase), phase != 3, x,
													   y);
		std::puts("GEADEV:OK TOUCH");
#else
		std::puts("GEADEV:ERR TOUCH comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "INPUTTRACE")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		const char *action = nextToken(cursor);
		if (gea::platform::comparison::enabled.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR INPUTTRACE benchmark-active");
			return;
		}
		if (tokenEquals(action, "BEGIN")) {
			if (gDiagnosticGesture || !beginDiagnosticInput()) {
				std::puts("GEADEV:ERR INPUTTRACE unavailable-or-active");
				return;
			}
			std::puts("GEADEV:OK INPUTTRACE BEGIN");
		} else if (tokenEquals(action, "END")) {
			if (!diagnostic_input::recording.load(std::memory_order_acquire)) {
				std::puts("GEADEV:ERR INPUTTRACE inactive");
				return;
			}
			const auto trace = diagnostic_input::end();
			std::printf("SWINPUT {\"framework\":\"gea\",\"coordinate_space\":\"logical\",\"dropped\":%u,\"samples\":",
						trace.dropped);
			printDiagnosticInput(trace);
			std::puts("}");
			heap_caps_free(trace.entries);
		} else {
			std::puts("GEADEV:ERR INPUTTRACE usage=BEGIN_or_END");
		}
#else
		std::puts("GEADEV:ERR INPUTTRACE comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "SCROLLSTATE")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		if (gea::platform::comparison::enabled.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR SCROLLSTATE benchmark-active");
			return;
		}
		int filter = -1;
		if (const char *name = nextToken(cursor)) {
			for (unsigned index = 0; index < std::size(diagnostic_gesture::kClasses); ++index) {
				if (tokenEquals(name, diagnostic_gesture::kClasses[index])) {
					filter = int(index);
				}
			}
			if (filter < 0) {
				std::puts("GEADEV:ERR SCROLLSTATE unknown-class");
				return;
			}
		}
		auto *sample = static_cast<diagnostic_gesture::Observation *>(
			heap_caps_malloc(sizeof(diagnostic_gesture::Observation), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!sample) {
			std::puts("GEADEV:ERR SCROLLSTATE allocation-failed");
			return;
		}
		collectDiagnosticScroll(*sample, filter);
		std::printf("SWSCROLL {\"timestamp_us\":%lld,\"nodes_dropped\":%u,\"nodes\":",
					static_cast<long long>(sample->actualUs), sample->dropped);
		printDiagnosticNodes(*sample);
		std::puts("}");
		heap_caps_free(sample);
#else
		std::puts("GEADEV:ERR SCROLLSTATE comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "GESTURE")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		handleDiagnosticGesture(cursor);
#else
		std::puts("GEADEV:ERR GESTURE comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "CLOCKFREEZE")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		long long epoch;
		if (gea::platform::comparison::enabled.load(std::memory_order_acquire)) {
			std::puts("GEADEV:ERR CLOCKFREEZE benchmark-active");
			return;
		}
		if (std::sscanf(cursor, "%lld", &epoch) != 1 || epoch < 0 || epoch > 4102444800LL) {
			std::puts("GEADEV:ERR CLOCKFREEZE invalid-epoch");
			return;
		}
		gea::platform::comparison::frozenEpochSeconds.store(epoch, std::memory_order_relaxed);
		std::printf("GEADEV:OK CLOCKFREEZE epoch=%lld\n", epoch);
#else
		std::puts("GEADEV:ERR CLOCKFREEZE comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "UPLOADSHOT")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		handleDiagnosticUpload(cursor);
#else
		std::puts("GEADEV:ERR UPLOADSHOT comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "COMPLETION")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		bool on;
		if (gea::platform::comparison::enabled.load(std::memory_order_acquire) || !parseOnOff(nextToken(cursor), &on)) {
			std::puts("GEADEV:ERR COMPLETION inactive-benchmark-and-0-or-1-required");
			return;
		}
		gea::platform::comparison::completionFence.store(on, std::memory_order_relaxed);
		std::printf("GEADEV:OK COMPLETION fence=%d\n", on ? 1 : 0);
#else
		std::puts("GEADEV:ERR COMPLETION comparison-diagnostic-not-enabled");
#endif
	} else if (tokenEquals(command, "BENCH")) {
#if GEA_EMBEDDED_COMPARISON_BENCHMARK
		const char *action = nextToken(cursor);
		if (tokenEquals(action, "BEGIN")) {
			if (gDiagnosticGesture || diagnostic_input::recording.load(std::memory_order_acquire) ||
				gea::platform::comparison::frozenEpochSeconds.load(std::memory_order_relaxed) ||
				gea::platform::comparison::upload::armed.load(std::memory_order_acquire)) {
				std::puts("GEADEV:ERR BENCH clear-gesture-input-trace-and-clock-freeze-first");
				return;
			}
			const char *runId = nextToken(cursor);
			const char *scenario = nextToken(cursor);
			const auto validName = [](const char *name) {
				if (!name || !*name || std::strlen(name) >= 64) {
					return false;
				}
				for (const char *p = name; *p; ++p) {
					if (!std::isalnum(static_cast<unsigned char>(*p)) && *p != '-' && *p != '_' && *p != '.') {
						return false;
					}
				}
				return true;
			};
			if (!validName(runId) || !validName(scenario)) {
				std::puts(
					"GEADEV:ERR BENCH usage=BEGIN_run-id_scenario_names-up-to-63-alphanumeric-dash-dot-underscore");
				return;
			}
			gea::platform::comparison::begin(runId, scenario, esp_timer_get_time());
			std::printf("GEADEV:OK BENCH BEGIN run_id=%s scenario=%s\n", runId, scenario);
		} else if (tokenEquals(action, "END")) {
			if (!gea::platform::comparison::enabled.load(std::memory_order_acquire)) {
				std::puts("GEADEV:ERR BENCH no-active-window");
				return;
			}
			const auto sample = gea::platform::comparison::end(esp_timer_get_time());
			constexpr auto internal = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
			constexpr auto psram = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
			char presented[32];
			const bool completed = sample.completionFence && !sample.completionFailures &&
								   gea_display_completed_chunks && gea_display_wait_for_uploads;
			if (completed) {
				std::snprintf(presented, sizeof(presented), "%llu",
							  static_cast<unsigned long long>(sample.presentedFrames));
			} else {
				std::snprintf(presented, sizeof(presented), "null");
			}
			std::printf(
				"SWBENCH "
				"{\"schema_version\":1,\"framework\":\"gea\",\"optimization\":\"%s\",\"variant\":\"%s\",\"run_id\":\"%"
				"s\",\"scenario\":\"%s\",\"phase\":\"end\","
				"\"timestamp_us\":%lld,\"window_us\":%lld,\"scheduler_frames\":%llu,\"rendered_frames\":%llu,"
				"\"presented_frames\":%s,"
				"\"present_chunks\":%llu,\"present_pixels\":%llu,\"work_us\":%llu,\"work_max_us\":%llu,\"work_p99_"
				"upper_us\":%lld,"
				"\"cadence_count\":%llu,\"cadence_sum_us\":%llu,\"cadence_max_us\":%llu,\"cadence_p99_upper_us\":%lld,"
				"\"presented_cadence_count\":%llu,\"presented_cadence_sum_us\":%llu,\"presented_cadence_max_us\":%llu,"
				"\"presented_cadence_p99_upper_us\":%lld,"
				"\"completion_fence\":%s,\"completion_wait_us_sum\":%llu,\"completion_wait_us_max\":%llu,\"completion_"
				"failures\":%u,"
				"\"rendered_duration_sum_us\":null,\"heap_min_scope\":\"boot\",\"presented_scope\":\"%s\","
				"\"internal\":{\"total_bytes\":%u,\"free_bytes\":%u,\"min_free_bytes\":%u,\"largest_free_bytes\":%u},"
				"\"psram\":{\"total_bytes\":%u,\"free_bytes\":%u,\"min_free_bytes\":%u,\"largest_free_bytes\":%u}}\n",
				diagnosticOptimization(), sample.completionFence ? "benchmark-fenced" : "benchmark-unfenced",
				sample.runId, sample.scenario, static_cast<long long>(sample.endUs),
				static_cast<long long>(sample.endUs - sample.startUs),
				static_cast<unsigned long long>(sample.schedulerFrames),
				static_cast<unsigned long long>(sample.renderedFrames), presented,
				static_cast<unsigned long long>(sample.presentChunks),
				static_cast<unsigned long long>(sample.presentPixels), static_cast<unsigned long long>(sample.work.sum),
				static_cast<unsigned long long>(sample.work.maximum),
				static_cast<long long>(sample.work.percentileUpper(99)),
				static_cast<unsigned long long>(sample.cadence.count),
				static_cast<unsigned long long>(sample.cadence.sum),
				static_cast<unsigned long long>(sample.cadence.maximum),
				static_cast<long long>(sample.cadence.percentileUpper(99)),
				static_cast<unsigned long long>(sample.presentedCadence.count),
				static_cast<unsigned long long>(sample.presentedCadence.sum),
				static_cast<unsigned long long>(sample.presentedCadence.maximum),
				static_cast<long long>(sample.presentedCadence.percentileUpper(99)),
				sample.completionFence ? "true" : "false", static_cast<unsigned long long>(sample.completionWaitSum),
				static_cast<unsigned long long>(sample.completionWaitMax), unsigned(sample.completionFailures),
				completed ? "fenced-upload-frame" : "unavailable-unfenced-or-failed",
				unsigned(heap_caps_get_total_size(internal)), unsigned(heap_caps_get_free_size(internal)),
				unsigned(heap_caps_get_minimum_free_size(internal)),
				unsigned(heap_caps_get_largest_free_block(internal)), unsigned(heap_caps_get_total_size(psram)),
				unsigned(heap_caps_get_free_size(psram)), unsigned(heap_caps_get_minimum_free_size(psram)),
				unsigned(heap_caps_get_largest_free_block(psram)));
		} else {
			std::puts("GEADEV:ERR BENCH expected-BEGIN-or-END");
		}
#else
		std::puts("GEADEV:ERR BENCH GEA_EMBEDDED_COMPARISON_BENCHMARK-not-enabled");
#endif
	} else if (tokenEquals(command, "MEM")) {
		const unsigned intFree = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intTotal = static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intLargest = static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intMinFree = static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned psramFree = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const unsigned psramTotal = static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const unsigned psramLargest = static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const unsigned psramMinFree = static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const char *appId = gea::framework::apps::AppManager::currentId();
		std::printf(
		    "GEADEV:MEM app=%s internal_used=%u internal_free=%u internal_total=%u internal_largest=%u internal_min_free=%u psram_used=%u psram_free=%u psram_total=%u psram_largest=%u flush_rows=%d flush_depth=%d flush_bytes=%d psram_min_free=%u\n",
		    appId ? appId : "",
		    intTotal - intFree, intFree, intTotal, intLargest, intMinFree,
		    psramTotal - psramFree, psramFree, psramTotal, psramLargest,
		    gea::platform::display::Display::flushChunkRows(),
		    gea::platform::display::Display::flushQueueDepth(),
		    gea::platform::display::Display::flushBufferBytes(), psramMinFree);
	} else if (tokenEquals(command, "HEAPTRACE")) {
#if CONFIG_HEAP_TRACING_STANDALONE
		// Allocate records only on demand, outside internal DMA RAM. Emit
		// addresses/sizes/stacks, never allocation contents or credentials.
		static heap_trace_record_t *records = nullptr;
		constexpr size_t recordCount = 4096;
		char *action = nextToken(cursor);
		if (action && tokenEquals(action, "START")) {
			if (!records) {
				records = static_cast<heap_trace_record_t *>(heap_caps_calloc(recordCount, sizeof(heap_trace_record_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
				if (!records) { std::printf("GEADEV:ERR HEAPTRACE no-memory\n"); return; }
				const esp_err_t init = heap_trace_init_standalone(records, recordCount);
				if (init != ESP_OK) {
					heap_caps_free(records); records = nullptr;
					std::printf("GEADEV:ERR HEAPTRACE init=%s\n", esp_err_to_name(init)); return;
				}
			}
			const esp_err_t start = heap_trace_start(HEAP_TRACE_LEAKS);
			std::printf("GEADEV:HEAPTRACE START result=%s\n", esp_err_to_name(start));
		} else if (action && tokenEquals(action, "STOP") && records) {
			const esp_err_t stop = heap_trace_stop();
			if (stop != ESP_OK) { std::printf("GEADEV:ERR HEAPTRACE stop=%s\n", esp_err_to_name(stop)); return; }
			heap_trace_summary_t summary{};
			heap_trace_summary(&summary);
			std::printf("GEADEV:HEAPTRACE SUMMARY count=%u capacity=%u high_water=%u overflow=%u\n",
			            unsigned(summary.count), unsigned(summary.capacity), unsigned(summary.high_water_mark), unsigned(summary.has_overflowed));
			for (size_t i = 0; i < heap_trace_get_count(); ++i) {
				heap_trace_record_t record{};
				if (heap_trace_get(i, &record) != ESP_OK || record.freed) continue;
				std::printf("GEADEV:HEAPTRACE RECORD bytes=%u region=%s address=%p stack=",
				            unsigned(record.size), esp_ptr_internal(record.address) ? "internal" : "external", record.address);
				for (int j = 0; j < CONFIG_HEAP_TRACING_STACK_DEPTH; ++j) std::printf("%s%p", j ? "," : "", record.alloced_by[j]);
				std::printf("\n");
			}
			if (heap_trace_init_standalone(nullptr, 0) == ESP_OK) { heap_caps_free(records); records = nullptr; }
			std::printf("GEADEV:HEAPTRACE END\n");
		} else std::printf("GEADEV:ERR HEAPTRACE usage=START_or_STOP\n");
#else
		std::printf("GEADEV:ERR HEAPTRACE standalone-tracing-not-enabled\n");
#endif
	} else if (tokenEquals(command, "CANVASSTATS")) {
		int tBegin = 0, tEnd = 0, tFillRect = 0, tDrawImage = 0, tNullPx = 0, tPresentOk = 0;
		gea::embedded::ui::canvasTotalsRead(&tBegin, &tEnd, &tFillRect, &tDrawImage, &tNullPx, &tPresentOk);
		std::printf("GEADEV:CANVASSTATS begin=%d end=%d fillRect=%d drawImage=%d nullPixels=%d presentOk=%d\n",
		            tBegin, tEnd, tFillRect, tDrawImage, tNullPx, tPresentOk);
	} else if (tokenEquals(command, "LANDFRAME")) {
		using gea::platform::display::Display;
		int total = 0, align = 0, raster = 0, text = 0, flip = 0;
		Display::landFrameDebug(&total, &align, &raster, &text, &flip);
		std::printf("GEADEV:LANDFRAME totalUs=%d alignUs=%d rasterUs=%d textUs=%d flipUs=%d\n", total, align, raster, text, flip);
	} else if (tokenEquals(command, "KBINFO")) {
		auto &tree = gea::embedded::ui::Tree::instance();
		const int kb = gea::embedded::ui::VirtualKeyboard::instance().debugRootNode();
		const int active = tree.activeInputId();
		if (kb >= 0 && kb < tree.nodeCount()) {
			const auto &node = tree.node(kb);
			std::printf("GEADEV:KBINFO kb=%d active=%d parent=%d display=%d layout=%d,%d %dx%d styleTop=%d styleW=%d styleH=%d\n",
			            kb, active, node.parent, GEA_DIAGNOSTIC_NODE_STYLE(node).display, node.layout.x, node.layout.y,
			            node.layout.width, node.layout.height, GEA_CSS_POSITION_PX(GEA_DIAGNOSTIC_NODE_STYLE(node), 0), GEA_DIAGNOSTIC_NODE_STYLE(node).width, GEA_DIAGNOSTIC_NODE_STYLE(node).height);
		} else {
			std::printf("GEADEV:KBINFO kb=%d active=%d\n", kb, active);
		}
	} else if (tokenEquals(command, "LANDPAN")) {
		using gea::platform::display::Display;
		int detect = 0, kick = 0, strips = 0, wait = 0, interior = 0;
		Display::landPanDebug(&detect, &kick, &strips, &wait, &interior);
		std::printf("GEADEV:LANDPAN detectUs=%d kickUs=%d stripUs=%d waitUs=%d interiorUs=%d\n", detect, kick, strips, wait, interior);
	} else if (tokenEquals(command, "FLUSHSTATS")) {
		using gea::platform::display::Display;
		const auto stats = Display::flushPerfStatsRead();
		int dbReason = 0, dbExtra = 0, dbCount = 0;
		gea::embedded::ui::Tree::instance().displayBackedFailDebug(&dbReason, &dbExtra, &dbCount);
		int ppCalls = 0, ppDirect = 0, ppGeneral = 0, ppRejected = 0, tsKind = 0, tsType = 0, tsCount = 0;
		Display::presentPathDebug(&ppCalls, &ppDirect, &ppGeneral, &ppRejected, &tsKind, &tsType, &tsCount);
		int slotMode = -1, slotRebinds = 0;
		gea::embedded::ui::Tree::instance().canvasSlotDebug(0, &slotMode, &slotRebinds);
		std::printf("GEADEV:FLUSHSTATS calls=%d chunks=%d pixels=%d totalUs=%lld rasterUs=%lld copyUs=%lld txUs=%lld slotWaitUs=%lld completeWaitUs=%lld dbReason=%d dbExtra=%d dbCount=%d ppCalls=%d ppDirect=%d ppGeneral=%d ppRejected=%d tsKind=%d tsType=%d tsCount=%d slotMode=%d rebinds=%d\n",
		            stats.callCount, stats.chunkCount, stats.pixelCount,
		            static_cast<long long>(stats.totalUs), static_cast<long long>(stats.rasterUs),
		            static_cast<long long>(stats.copyUs), static_cast<long long>(stats.txUs),
		            static_cast<long long>(stats.slotWaitUs), static_cast<long long>(stats.completeWaitUs),
		            dbReason, dbExtra, dbCount, ppCalls, ppDirect, ppGeneral, ppRejected, tsKind, tsType, tsCount, slotMode, slotRebinds);
	} else if (tokenEquals(command, "STATE")) {
		using gea::platform::display::Display;

		gea::framework::services::AppState::lock();
		const char *appId = gea::framework::apps::AppManager::currentId();
		char appIdBuffer[96];
		std::snprintf(appIdBuffer, sizeof(appIdBuffer), "%s", appId ? appId : "");
		auto &tree = gea::embedded::ui::Tree::instance();
		const int nodes = tree.nodeCount();
		const int root = tree.mountedRoot();
		const int width = tree.mountedWidth();
		const int height = tree.mountedHeight();
		const int refresh = tree.refreshRequired() ? 1 : 0;
		// Reachability census: `nodes` is the slot high-water mark, so a climbing
		// count alone cannot say whether the surplus is still hanging in the tree
		// (a rebuild that appends without removing) or detached-but-unfreed (a
		// remove that never ran). Walking from the mounted root separates them.
		int reachable = 0;
		int rootless = 0;
		{
			const auto *nodeArray = tree.nodes();
			int stack[256];
			int top = 0;
			if (root >= 0 && root < nodes) stack[top++] = root;
			int guard = 0;
			while (top > 0 && guard++ < nodes * 4) {
				const int id = stack[--top];
				reachable++;
				for (int c = nodeArray[id].first_child; c >= 0 && c < nodes; c = nodeArray[c].next_sibling) {
					if (top < 256) stack[top++] = c;
				}
			}
			for (int i = 0; i < nodes; i++)
				if (i != root && nodeArray[i].parent < 0) rootless++;
		}
		const int drawNonBlack = Display::countNonBlackPixels(false);
		const int presentedNonBlack = Display::countNonBlackPixels(true);
		gea::framework::services::AppState::unlock();
		const int battery = gea::platform::power::Power::batteryPercent();
		std::printf("GEADEV:STATE app=%s nodes=%d reachable=%d rootless=%d root=%d width=%d height=%d refresh=%d draw_nonblack=%d presented_nonblack=%d battery=%d\n",
		            appIdBuffer,
		            nodes,
		            reachable,
		            rootless,
		            root,
		            width,
		            height,
		            refresh,
		            drawNonBlack,
		            presentedNonBlack,
		            battery);
	} else if (tokenEquals(command, "TAP")) {
		handleTap(cursor);
	} else if (tokenEquals(command, "DRAG")) {
		handleDrag(cursor);
	} else if (tokenEquals(command, "ROTARY")) {
		handleRotary(cursor);
	} else if (tokenEquals(command, "KEY")) {
		handleKey(cursor);
	} else if (tokenEquals(command, "STORAGE")) {
		handleStorage(cursor);
	} else if (tokenEquals(command, "SWIPE")) {
		handleSwipe(cursor);
	} else if (tokenEquals(command, "BRIGHTNESS")) {
		handleBrightness(cursor);
	} else if (tokenEquals(command, "HBM")) {
		handleHbm(cursor);
	} else if (tokenEquals(command, "VSYNC")) {
		handleVsync(cursor);
	} else if (tokenEquals(command, "I2CSCAN")) {
		handleI2cScan();
	} else if (tokenEquals(command, "NODE")) {
		handleNode(cursor);
	} else if (tokenEquals(command, "HITTEST")) {
		handleHitTest(cursor);
	} else if (tokenEquals(command, "DOWNLOAD")) {
		// Restart into ROM download mode, so a host can flash over a UART bridge
		// whose DTR/RTS lines cannot reset the chip (e.g. the Waveshare P4 LCD-3.5).
		// The flag survives a watchdog reset; the flasher clears it before resetting.
#if CONFIG_IDF_TARGET_ESP32P4
		std::printf("GEADEV:OK DOWNLOAD\n");
		std::fflush(stdout);
		vTaskDelay(pdMS_TO_TICKS(120));
		REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);
		esp_restart();
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
		std::printf("GEADEV:OK DOWNLOAD\n");
		std::fflush(stdout);
		vTaskDelay(pdMS_TO_TICKS(120));
		REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
		esp_restart();
#else
		std::printf("GEADEV:ERR DOWNLOAD unsupported-chip\n");
#endif
	} else if (tokenEquals(command, "REBOOT")) {
		// Soft-reboot the device. Lets the Mac companion apply a face change
		// (SETDEFAULT writes NVS; the new default only takes effect on boot).
		std::printf("GEADEV:OK REBOOT\n");
		std::fflush(stdout);
		vTaskDelay(pdMS_TO_TICKS(120));  // let the serial response flush before reset
		esp_restart();
	} else if (tokenEquals(command, "NOTIFY")) {
		// The rest of the line is the notification text; push it to the Notify
		// channel that watch faces read to show a transient banner.
		while (*cursor == ' ') cursor++;
		gea::host::postNotification(std::string(cursor));
		std::printf("GEADEV:OK NOTIFY len=%d\n", static_cast<int>(std::strlen(cursor)));
	} else if (tokenEquals(command, "PUSH")) {
		if (source == CommandSource::Stdio) handlePush(cursor);
		else std::printf("GEADEV:PUSH ERR unsupported-on-usb-direct\n");
#if GEA_DEVICE_CONTROL_USB_OTA
	} else if (tokenEquals(command, "OTA")) {
		if (source == CommandSource::Stdio) handleOta(cursor);
		else std::printf("GEADEV:OTA ERR unsupported-on-usb-direct\n");
#endif
	} else if (tokenEquals(command, "PUSH64")) {
		if (source == CommandSource::Stdio) handlePushBase64(cursor);
		else std::printf("GEADEV:PUSH64 ERR unsupported-on-usb-direct\n");
	} else if (tokenEquals(command, "LS")) {
		handleListFiles(cursor);
	} else if (tokenEquals(command, "RM")) {
		handleRemoveFile(cursor);
	} else if (tokenEquals(command, "FORMAT")) {
		if (source == CommandSource::Stdio) handleFormat(cursor);
		else std::printf("GEADEV:FORMAT ERR unsupported-on-usb-direct\n");
	} else if (tokenEquals(command, "PULL")) {
		handlePull(cursor);
#if GEA_AUDIO_EXPERIMENT
  } else if (tokenEquals(command, "AUDIOCAPTURE")) {
    const char *value = nextToken(cursor);
    char *end = nullptr;
    const unsigned long offset = value ? std::strtoul(value, &end, 10) : 0;
    int16_t samples[128 * 3];
    size_t frames = 0, total = 0;
    int64_t started = 0;
    uint32_t drops = 0;
    const bool ok = value && end && *end == '\0' && offset <= 128000 &&
        geaAudioAecCaptureRead(offset, samples, 128, &frames, &total, &started, &drops);
    if (!ok) std::puts("GEADEV:AUDIOCAPTURE ERR");
    else {
      char encoded[128 * 3 * 4 + 1];
      constexpr char digits[] = "0123456789abcdef";
      const auto *bytes = reinterpret_cast<const uint8_t *>(samples);
      for (size_t i = 0; i < frames * 6; ++i) {
        encoded[i * 2] = digits[bytes[i] >> 4];
        encoded[i * 2 + 1] = digits[bytes[i] & 15];
      }
      encoded[frames * 12] = '\0';
      std::printf("GEADEV:AUDIOCAPTURE OK offset=%lu frames=%u total=%u started_us=%lld drops=%lu data=%s\n",
          offset, unsigned(frames), unsigned(total), (long long)started, (unsigned long)drops, encoded);
    }
  } else if (tokenEquals(command, "AUDIO")) {
    char *key = nextToken(cursor);
    char *value = nextToken(cursor);
    bool ok = true;
    if (key) {
      char *end = nullptr;
      const float number = value ? std::strtof(value, &end) : NAN;
      ok = value && end && *end == '\0' && std::isfinite(number);
      if (ok && !std::strcmp(key, "speaker")) {
        ok = number >= 0 && number <= 100 && std::floor(number) == number;
        if (ok) gea::platform::audio::AudioSystem::setVolume(int(number));
      } else if (ok) ok = geaAudioExperimentConfigure(key, number);
    }
    char state[512];
    geaAudioExperimentDescribe(state, sizeof(state));
    std::printf("GEADEV:AUDIO %s %s\n", ok ? "OK" : "ERR", state);
#endif
	} else if (tokenEquals(command, "PLAYFILE")) {
		handlePlayFile(cursor);
	} else if (tokenEquals(command, "VP8BENCH")) {
		benchmarkVp8File(nextToken(cursor));
	} else if (tokenEquals(command, "VP8CAPTURE")) {
		const auto action = nextToken(cursor);
		if (tokenEquals(action, "START")) startVp8Capture();
		else if (tokenEquals(action, "SAVE")) saveVp8Capture();
		else if (tokenEquals(action, "CANCEL")) cancelVp8Capture();
		else std::puts("GEADEV:ERR VP8CAPTURE expected-start-save-or-cancel");
	} else if (tokenEquals(command, "OPUSBENCH")) {
		benchmarkOpus();
	} else if (tokenEquals(command, "SIMDBENCH")) {
		const auto mode = nextToken(cursor);
		benchmarkVideoSimd(tokenEquals(mode, "swap") ? 1 : tokenEquals(mode, "kernels") ? 2 : 0);
	} else if (tokenEquals(command, "PCMSIMD")) {
		checkPcmSimd();
	} else if (tokenEquals(command, "SRTPBENCH")) {
		benchmarkSrtp();
	} else if (tokenEquals(command, "BACK")) {
		const bool ok = gea::framework::apps::AppManager::returnRunningAppToLauncher("device control");
		std::printf("GEADEV:OK BACK returned=%d\n", ok ? 1 : 0);
	} else if (tokenEquals(command, "WIFI")) {
		// Dev/test: WiFi is opt-in (off at boot); bring it up or down on demand.
		const char *arg = nextToken(cursor);
		const bool on = !arg || tokenEquals(arg, "ON") || tokenEquals(arg, "1");
		gea::framework::network::WifiBackend::setEnabled(on);
		std::printf("GEADEV:OK WIFI %s connected=%d\n", on ? "on" : "off",
		            gea::framework::network::WifiBackend::connected() ? 1 : 0);
	} else if (tokenEquals(command, "SETDEFAULT")) {
		// Set the boot default app (NVS 'default_app'), read by Runtime::run at
		// boot. Lets the Mac companion choose which app the watch boots into.
		const char *appId = nextToken(cursor);
		if (!appId || appId[0] == '\0')
			std::printf("GEADEV:ERR SETDEFAULT usage=GEADEV_SETDEFAULT_appid\n");
		else if (gea::framework::services::StorageService::setString("default_app", appId))
			std::printf("GEADEV:OK SETDEFAULT default_app=%s\n", appId);
		else
			std::printf("GEADEV:ERR SETDEFAULT nvs-write-failed\n");
	} else if (tokenEquals(command, "SETTIME")) {
		handleSetTime(cursor);
	} else if (tokenEquals(command, "SCREENSHOT")) {
		writeScreenshot();
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
	} else if (tokenEquals(command, "SCREENSHOTBIN")) {
		// With the primary USB console, stdio commands arrive over the same
		// USB driver as secondary-console commands. Stream binary pixels through
		// its reliable bulk writer: the VFS character writer can silently drop
		// bytes when its TX ring fills during a large screenshot.
		bool usbSource = source == CommandSource::UsbSerialJtag;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
		usbSource = usbSource || source == CommandSource::Stdio;
#endif
		if (usbSource) writeScreenshotUsbRaw();
		else std::printf("GEADEV:ERR SCREENSHOTBIN usb-serial-jtag-only\n");
#endif
	} else if (tokenEquals(command, "MMUPROBE")) {
		handleMmuProbe();
	} else if (tokenEquals(command, "LOADTEST")) {
		handleLoadTest();
	} else if (geaHandleExtraDevCommand(command, cursor)) {
		// Handled by a target-specific extension (e.g. ESP32-P4 camera capture).
	} else {
		std::printf("GEADEV:ERR unknown-command command=%s\n", command);
	}
	std::fflush(stdout);
}

void dispatchLineTooLong()
{
	if (gCommandMutex) xSemaphoreTake(gCommandMutex, portMAX_DELAY);
	std::printf("GEADEV:ERR line-too-long max=%d\n", 511);
	std::fflush(stdout);
	if (gCommandMutex) xSemaphoreGive(gCommandMutex);
}

void dispatchCommand(char *line, CommandSource source)
{
	if (gCommandMutex) xSemaphoreTake(gCommandMutex, portMAX_DELAY);
	handleCommand(line, source);
	if (gCommandMutex) xSemaphoreGive(gCommandMutex);
}

class CommandLineAccumulator {
public:
	explicit CommandLineAccumulator(CommandSource source) : source_(source) {}

	void feed(const char *data, std::size_t length)
	{
		for (std::size_t i = 0; i < length; ++i) feed(data[i]);
	}

private:
	void feed(char ch)
	{
		if (ch == '\n' || ch == '\r') {
			if (overflow_) {
				dispatchLineTooLong();
			} else if (length_ > 0) {
				buffer_[length_] = '\0';
				dispatchCommand(buffer_, source_);
			}
			length_ = 0;
			overflow_ = false;
			return;
		}
		if (length_ + 1 >= sizeof(buffer_)) {
			overflow_ = true;
		} else if (!overflow_) {
			buffer_[length_++] = ch;
		}
	}

	CommandSource source_;
	char buffer_[512]{};
	std::size_t length_ = 0;
	bool overflow_ = false;
};

#if !CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
// The console loop turns every 50 ms while idle. When it stops turning (a
// command that never returns, a console read that never comes back), the
// board is unreachable and only a power cycle used to recover it -- the
// stuck task holds nothing the task watchdog watches. Restart instead.
std::atomic<std::int64_t> &consoleBeat()
{
	static std::atomic<std::int64_t> beat{0};
	return beat;
}

std::atomic<bool> &consoleInCommand()
{
	static std::atomic<bool> busy{false};
	return busy;
}

void startConsoleWatchdog()
{
	static bool started = false;
	if (started) return;
	started = true;
	xTaskCreate(
	    [](void *) {
		    while (true) {
			    vTaskDelay(pdMS_TO_TICKS(1000));
			    const std::int64_t last = consoleBeat().load(std::memory_order_relaxed);
			    if (last == 0) continue;
			    // A streamed OTA or a large push legitimately holds the loop for minutes.
			    const std::int64_t limit = consoleInCommand().load(std::memory_order_relaxed) ? 600000000LL : 20000000LL;
			    if (esp_timer_get_time() - last > limit) {
				    esp_rom_printf("gea_devctl: console stalled for %lld ms, restarting\n",
				                   static_cast<long long>((esp_timer_get_time() - last) / 1000));
				    esp_restart();
			    }
		    }
	    },
	    "gea_devctl_wd", 2048, nullptr, configMAX_PRIORITIES - 1, nullptr);
}
#endif

#if !CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
class DeviceControlTask {
public:
	static void run(void *)
	{
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
		// The primary USB VFS otherwise uses a ROM/FIFO polling writer. It can
		// busy-spin for up to 50 ms while a log line drains, at the caller's
		// media priority. Use IDF's bounded driver rings and blocking waits, as
		// the secondary USB console already does. stdin blocks on RX instead
		// of polling; no extra console/network task is required.
		usb_serial_jtag_driver_config_t config = {
		    .tx_buffer_size = 1024,
		    .rx_buffer_size = 1024,
		};
		if (!usb_serial_jtag_is_driver_installed()) {
			const esp_err_t err = usb_serial_jtag_driver_install(&config);
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "Failed to install primary USB console driver: %s", esp_err_to_name(err));
				vTaskDelete(nullptr);
				return;
			}
		}
		usb_serial_jtag_vfs_use_driver();
#endif
		std::setvbuf(stdin, nullptr, _IONBF, 0);
		std::setvbuf(stdout, nullptr, _IONBF, 0);
		ESP_LOGI(kTag, "Device control ready: send 'GEADEV PING', 'GEADEV TAP x y', 'GEADEV BACK', or 'GEADEV SCREENSHOT'");

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
		char readBuffer[160];
#endif
		CommandLineAccumulator parser(CommandSource::Stdio);
		startConsoleWatchdog();
		while (true) {
			consoleBeat().store(esp_timer_get_time(), std::memory_order_relaxed);
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
			// fgets waits indefinitely on the driver-backed VFS, so normal USB
			// silence would stop the heartbeat. Read one byte with a deadline;
			// never prefetch a PUSH/OTA payload that its handler reads from stdin.
			char ch;
			const int n = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(50));
			if (n <= 0) continue;
			consoleInCommand().store(true, std::memory_order_relaxed);
			parser.feed(&ch, 1);
			consoleInCommand().store(false, std::memory_order_relaxed);
#else
			if (!std::fgets(readBuffer, sizeof(readBuffer), stdin)) {
				// A console whose read returns 0 when idle (the TinyUSB CDC VFS)
				// sets stdin's end-of-file flag, and that flag is sticky: every
				// later fgets fails at once until it is cleared.
				std::clearerr(stdin);
				vTaskDelay(pdMS_TO_TICKS(50));
				continue;
			}
			consoleInCommand().store(true, std::memory_order_relaxed);
			parser.feed(readBuffer, std::strlen(readBuffer));
			consoleInCommand().store(false, std::memory_order_relaxed);
#endif
		}
	}
};
#endif

#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
class UsbSerialJtagControlTask {
public:
	static void run(void *)
	{
		usb_serial_jtag_driver_config_t config = {
		    .tx_buffer_size = 4096,
		    .rx_buffer_size = 1024,
		};
		if (!usb_serial_jtag_is_driver_installed()) {
			const esp_err_t err = usb_serial_jtag_driver_install(&config);
			if (err != ESP_OK) {
				ESP_LOGE(kTag, "Failed to install USB Serial/JTAG driver: %s", esp_err_to_name(err));
				vTaskDelete(nullptr);
				return;
			}
		}
		usb_serial_jtag_vfs_use_driver();
		ESP_LOGI(kTag, "USB Serial/JTAG device control ready: send 'GEADEV PING' or 'GEADEV SCREENSHOTBIN'");

		CommandLineAccumulator parser(CommandSource::UsbSerialJtag);
		char readBuffer[128];
		while (true) {
			const int n = usb_serial_jtag_read_bytes(readBuffer, sizeof(readBuffer), pdMS_TO_TICKS(50));
			if (n > 0) parser.feed(readBuffer, static_cast<std::size_t>(n));
		}
	}
};
#endif

bool gStarted = false;

}  // namespace

// Public capture entry point. The anonymous-namespace helper above stays the
// single implementation; this only lifts it out for the WiFi screenshot path.
bool captureScreenshotRgb565(std::uint16_t *snapshot, int pixelCapacity, int *width, int *height,
                             char *appIdBuffer, std::size_t appIdBufferSize)
{
	return captureSnapshotRgb565(snapshot, pixelCapacity, width, height, appIdBuffer, appIdBufferSize);
}

#if GEA_PIXEL_STORAGE_PACKED
bool captureScreenshotPacked(std::uint8_t *snapshot, int byteCapacity, int *width, int *height,
                             char *appIdBuffer, std::size_t appIdBufferSize)
{
	return captureSnapshotPacked(snapshot, byteCapacity, width, height, appIdBuffer, appIdBufferSize);
}
#endif

void startDeviceControlTask()
{
#if !GEA_EMBEDDED_DEVICE_CONTROL
	ESP_LOGI(kTag, "device control disabled by the app (GEA_EMBEDDED_DEVICE_CONTROL=0)");
#else
	if (gStarted) return;
	gStarted = true;
	if (!gCommandMutex) {
		gCommandMutex = xSemaphoreCreateMutex();
		if (!gCommandMutex) {
			gStarted = false;
			ESP_LOGE(kTag, "Failed to create device control mutex");
			return;
		}
	}
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
	TaskHandle_t usbTask = nullptr;
	const BaseType_t usbCreated = xTaskCreateWithCaps(&UsbSerialJtagControlTask::run,
	                                                  "gea_devctl_usb",
	                                                  kUsbJtagTaskStackBytes,
	                                                  nullptr,
	                                                  kTaskPriority,
	                                                  &usbTask,
	                                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (usbCreated != pdPASS) {
		gStarted = false;
		ESP_LOGE(kTag, "Failed to start USB Serial/JTAG device control task");
	}
#else
	TaskHandle_t task = nullptr;
	const BaseType_t created = xTaskCreateWithCaps(&DeviceControlTask::run,
	                                               "gea_devctl",
	                                               kTaskStackBytes,
	                                               nullptr,
	                                               kTaskPriority,
	                                               &task,
	                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (created != pdPASS) {
		gStarted = false;
		ESP_LOGE(kTag, "Failed to start device control task");
	}
#endif
#endif  // GEA_EMBEDDED_DEVICE_CONTROL
}

}  // namespace gea::platform::esp32::services

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
extern "C" void gea_touch_trace_consumed(int phase, bool touching, int x, int y, int pointerId, int handlerX,
										 int handlerY)
{
	if (!gea::platform::comparison::input::recording.load(std::memory_order_acquire)) {
		return;
	}
	gea::platform::comparison::input::consumed(esp_timer_get_time(), phase, touching, x, y, pointerId, handlerX,
											   handlerY);
}
#endif

// Weak no-op default for the per-target GEADEV command extension. Targets that
// add commands (e.g. ESP32-P4 camera capture) override this with a strong symbol;
// the linker then picks the override. Defined at global scope to match the
// `extern "C"` declaration used at the call site.
extern "C" __attribute__((weak)) bool geaHandleExtraDevCommand(const char *command, char *args)
{
	(void)command;
	(void)args;
	return false;
}
