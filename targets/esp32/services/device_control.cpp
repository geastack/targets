#include "services/device_control.h"

#include "apps.h"
#include "canvas.h"
#include "display.h"
#include "events.h"
#include "i2c.h"
#include "input.h"
#include "pixel.h"
#include "touch.h"  // Touchscreen::injectEvent for GEADEV DRAG/SWIPE
#include "wifi.h"   // network::wifi() for GEADEV PING ip=/mac=
#include "services/app_state.h"
#include "services/storage_service.h"
#include "ui/internal.h"
#include "ui/canvas_element.h"
#include "ui/tree_internal.h"
#include "ui/virtual_keyboard.h"

#include "driver/i2c_master.h"
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#endif
#include "esp_heap_caps.h"
#include "platform/file_cache.h"  // gea::platform::storage::ensureMounted for GEADEV PUSH
#include "esp_log.h"
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
#include <sys/stat.h>  // ::mkdir for GEADEV PUSH parent dirs
#include <sys/time.h>
#include "esp_system.h"  // esp_restart() for GEADEV REBOOT
#include "audio.h"  // gea::platform::audio::AudioSystem for GEADEV PLAYFILE
#include "power.h"  // Power::batteryPercent() for STATE battery
#include "host/notify.h"  // gea::host::postNotification() for GEADEV NOTIFY
#include "host/backends.h"  // WifiBackend for GEADEV WIFI
#include "host/storage.h"  // gea::host::Storage for GEADEV STORAGE SET

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
constexpr int kTaskStackBytes = 8192;
constexpr int kTaskPriority = 4;
constexpr int kUsbJtagTaskStackBytes = 8192;

enum class CommandSource {
	Stdio,
	UsbSerialJtag,
};

SemaphoreHandle_t gCommandMutex = nullptr;

std::uint32_t crc32Stream(std::uint32_t crc, const std::uint8_t *data, std::size_t n);
extern "C" bool geaDisplaySnapshotPrefersPresented() __attribute__((weak));

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
	const bool framebufferIsLive = GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH == 0;
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

#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
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
		gea::embedded::ui::ViewRenderer::transformedRectCorners(node,
		                                                        false,
		                                                        node.layout.x,
		                                                        node.layout.y,
		                                                        node.layout.width,
		                                                        node.layout.height,
		                                                        xs,
		                                                        ys);
		std::printf(
		    "GEADEV:NODE class=%s id=%d parent=%d type=%d display=%d style=%dx%d min=%dx%d max=%dx%d pos=%d,%d,%d,%d transform=%d,%d,%d,%d,%d layout=%d,%d,%d,%d corners=%d,%d;%d,%d;%d,%d;%d,%d ovf=%d,%d,%d scroll=%d,%d,%d,%d\n",
		    className,
		    nodeId,
		    node.parent,
		    static_cast<int>(node.type),
		    node.style.display,
		    node.style.width,
		    node.style.height,
		    node.style.min_width,
		    node.style.min_height,
		    node.style.max_width,
		    node.style.max_height,
		    node.style.pos_offsets[3],
		    node.style.pos_offsets[0],
		    node.style.pos_offsets[1],
		    node.style.pos_offsets[2],
		    gea::embedded::ui::rstyle(node.style).transform_translate_x,
		    gea::embedded::ui::rstyle(node.style).transform_translate_y,
		    gea::embedded::ui::rstyle(node.style).transform_translate_x_percent,
		    gea::embedded::ui::rstyle(node.style).transform_translate_y_percent,
		    gea::embedded::ui::rstyle(node.style).transform_rotate,
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
		    static_cast<int>(node.style.overflow),
		    static_cast<int>(node.style.overflow_x),
		    static_cast<int>(node.style.overflow_y),
		    static_cast<int>(node.layout.scroll_x),
		    static_cast<int>(node.layout.scroll_y),
		    static_cast<int>(node.layout.scroll_content_width),
		    static_cast<int>(node.layout.scroll_content_height));
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
	    node.style.display,
	    classes.c_str(),
	    pressTarget,
	    pressId,
	    gea::embedded::ui::rstyle(node.style).transform_translate_x,
	    gea::embedded::ui::rstyle(node.style).transform_translate_y,
	    gea::embedded::ui::rstyle(node.style).transform_translate_x_percent,
	    gea::embedded::ui::rstyle(node.style).transform_translate_y_percent,
	    gea::embedded::ui::rstyle(node.style).transform_rotate,
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
	// Mount the microSD (the tile cache does this via writeCacheFile; a raw
	// fopen alone won't trigger it).
	if (!gea::platform::storage::ensureMounted()) {
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
		const std::size_t n = std::fread(mem + got, 1, static_cast<std::size_t>(size - got), stdin);
		if (n == 0) {
			if (++stalls > 20000) {  // ~40s of silence → abort
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
	std::fclose(f);
	if (mem) heap_caps_free(mem);
	if (ioError) {
		std::printf("GEADEV:PUSH ERR transfer-failed path=%s got=%ld\n", path, got);
		return;
	}
	if (haveCrc && finalCrc != expectCrc) {
		std::printf("GEADEV:PUSH ERR crc-mismatch path=%s got=0x%08x want=0x%08x\n",
		            path, static_cast<unsigned>(finalCrc), static_cast<unsigned>(expectCrc));
		return;
	}
	std::printf("GEADEV:PUSH OK path=%s bytes=%ld crc=0x%08x\n", path, size, static_cast<unsigned>(finalCrc));
}

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
	if (!gea::platform::storage::ensureMounted()) {
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

	if (tokenEquals(command, "PING")) {
		// `gea boards discover` identifies a plugged-in unit from this one
		// line: the app it runs, the address it holds (0.0.0.0 without WiFi)
		// and the station MAC, which is also the USB serial an ESP32 board
		// enumerates with -- so the reply confirms which alias the port is.
		const char *appId = gea::framework::apps::AppManager::currentId();
		const std::string ip = gea::framework::network::wifi().ip();
		const std::string mac = gea::framework::network::wifi().mac();
		std::printf("GEADEV:PONG app=%s ip=%s mac=%s\n", appId ? appId : "", ip.c_str(), mac.c_str());
	} else if (tokenEquals(command, "APP")) {
		const char *appId = gea::framework::apps::AppManager::currentId();
		std::printf("GEADEV:APP id=%s\n", appId ? appId : "");
	} else if (tokenEquals(command, "MEM")) {
		const unsigned intFree = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intTotal = static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intLargest = static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned intMinFree = static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
		const unsigned psramFree = static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const unsigned psramTotal = static_cast<unsigned>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const unsigned psramLargest = static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		const char *appId = gea::framework::apps::AppManager::currentId();
		std::printf(
		    "GEADEV:MEM app=%s internal_used=%u internal_free=%u internal_total=%u internal_largest=%u internal_min_free=%u psram_used=%u psram_free=%u psram_total=%u psram_largest=%u\n",
		    appId ? appId : "",
		    intTotal - intFree, intFree, intTotal, intLargest, intMinFree,
		    psramTotal - psramFree, psramFree, psramTotal, psramLargest);
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
			            kb, active, node.parent, node.style.display, node.layout.x, node.layout.y,
			            node.layout.width, node.layout.height, node.style.pos_offsets[0], node.style.width, node.style.height);
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
	} else if (tokenEquals(command, "PUSH64")) {
		if (source == CommandSource::Stdio) handlePushBase64(cursor);
		else std::printf("GEADEV:PUSH64 ERR unsupported-on-usb-direct\n");
	} else if (tokenEquals(command, "LS")) {
		handleListFiles(cursor);
	} else if (tokenEquals(command, "RM")) {
		handleRemoveFile(cursor);
	} else if (tokenEquals(command, "PULL")) {
		handlePull(cursor);
	} else if (tokenEquals(command, "PLAYFILE")) {
		handlePlayFile(cursor);
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
#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
	} else if (tokenEquals(command, "SCREENSHOTBIN")) {
		if (source == CommandSource::UsbSerialJtag) writeScreenshotUsbRaw();
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
class DeviceControlTask {
public:
	static void run(void *)
	{
		std::setvbuf(stdin, nullptr, _IONBF, 0);
		std::setvbuf(stdout, nullptr, _IONBF, 0);
		ESP_LOGI(kTag, "Device control ready: send 'GEADEV PING', 'GEADEV TAP x y', 'GEADEV BACK', or 'GEADEV SCREENSHOT'");

		char readBuffer[160];
		CommandLineAccumulator parser(CommandSource::Stdio);
		while (true) {
			if (!std::fgets(readBuffer, sizeof(readBuffer), stdin)) {
				vTaskDelay(pdMS_TO_TICKS(50));
				continue;
			}
			parser.feed(readBuffer, std::strlen(readBuffer));
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
