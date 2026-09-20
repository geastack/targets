#include "camera.h"

#include "canvas.h"
#include "display.h"
#include "i2c.h"
#include "image.h"
#include "pixel.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/i2c_master.h"
#include "driver/ppa.h"
#include "driver/sdmmc_host.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_video_init.h"
#include "esp_video_device.h"  // ESP_VIDEO_H264_DEVICE_NAME (/dev/video11 HW encoder)
#include "esp_video_isp_ioctl.h"
#include "linux/videodev2.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"  // ESP32-P4 SD power is via the on-chip LDO
#include "sdmmc_cmd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"  // capture/preview serialization (opLock_)
#include "freertos/task.h"

#include <cstdio>

// Hardware Motion-JPEG recording uses the ESP32-P4 JPEG encoder. Guarded so the
// build stays green if the esp_driver_jpeg component isn't present; recording
// then degrades to a no-op (startRecording returns false).
#if __has_include("driver/jpeg_encode.h")
#include "driver/jpeg_encode.h"
#define GEA_P4_HAS_JPEG_ENCODER 1
#else
#define GEA_P4_HAS_JPEG_ENCODER 0
#endif

namespace pixel = gea::framework::graphics::pixel;

namespace gea::platform::esp32_p4_waveshare::camera {

namespace {

constexpr char kTag[] = "p4_camera";
constexpr char kDevicePath[] = "/dev/video0";
constexpr int kDefaultWidth = 800;
constexpr int kDefaultHeight = 1280;
constexpr int kBufferCount = 3;
constexpr int kSccbFrequencyHz = 400000;
// The OV5647 module is mounted 90deg rotated, so frames arrive rotated 90deg
// clockwise; rotate the preview 90deg counter-clockwise (one quarter turn) to
// bring the scene upright. With the display in LANDSCAPE orientation (it PPA-
// rotates the framebuffer 90deg), this rot=1 lands the scene upright on-screen.
// (Confirmed correct on-device.)
constexpr int kPreviewRotateCcw = 1;
// OV5647 default auto-exposure target brightness (OV5647_AE_TARGET_DEFAULT, range
// 2..235). setExposure() shifts this to brighten/darken the auto-exposed preview.
constexpr int kAeTargetDefault = 0x50;

struct MappedBuffer {
	std::uint8_t *data = nullptr;
	std::size_t length = 0;
};

struct CapturedFrame {
	const std::uint8_t *data = nullptr;
	std::size_t bytes = 0;
	int width = 0;
	int height = 0;
	std::uint32_t format = 0;
	bool mirror = false;
};

struct PreviewRaster {
	CapturedFrame *frame = nullptr;
	int destX = 0;
	int destY = 0;
	int destW = 0;
	int destH = 0;
};

int clampByte(int value)
{
	return std::min(255, std::max(0, value));
}

std::uint16_t yuvToRgb565(int y, int u, int v)
{
	const int c = y - 16;
	const int d = u - 128;
	const int e = v - 128;
	const int r = clampByte((298 * c + 409 * e + 128) >> 8);
	const int g = clampByte((298 * c - 100 * d - 208 * e + 128) >> 8);
	const int b = clampByte((298 * c + 516 * d + 128) >> 8);
	return pixel::fromRgb888(r, g, b);
}

std::uint16_t framePixel(const CapturedFrame &frame, int x, int y)
{
	if (!frame.data || frame.width <= 0 || frame.height <= 0) return 0;
	x = std::min(frame.width - 1, std::max(0, x));
	y = std::min(frame.height - 1, std::max(0, y));
	if (frame.mirror) x = frame.width - 1 - x;

	switch (frame.format) {
	case V4L2_PIX_FMT_RGB565: {
		const std::size_t offset = static_cast<std::size_t>(y * frame.width + x) * 2;
		if (offset + 1 >= frame.bytes) return 0;
		const std::uint16_t rgb565 = static_cast<std::uint16_t>(frame.data[offset] | (frame.data[offset + 1] << 8));
		return pixel::fromRgb565(rgb565);
	}
	case V4L2_PIX_FMT_RGB24: {
		const std::size_t offset = static_cast<std::size_t>(y * frame.width + x) * 3;
		if (offset + 2 >= frame.bytes) return 0;
		return pixel::fromRgb888(frame.data[offset], frame.data[offset + 1], frame.data[offset + 2]);
	}
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YUV422P: {
		const int pairX = x & ~1;
		const std::size_t offset = static_cast<std::size_t>(y * frame.width + pairX) * 2;
		if (offset + 3 >= frame.bytes) return 0;
		const int y0 = frame.data[offset];
		const int u = frame.data[offset + 1];
		const int y1 = frame.data[offset + 2];
		const int v = frame.data[offset + 3];
		return yuvToRgb565((x & 1) ? y1 : y0, u, v);
	}
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_SBGGR8: {
		const std::size_t offset = static_cast<std::size_t>(y * frame.width + x);
		if (offset >= frame.bytes) return 0;
		const int v = frame.data[offset];
		return pixel::fromRgb888(v, v, v);
	}
	default:
		return 0;
	}
}

void rasterPreview(std::uint16_t *pixels, int width, int height, int originX, int originY, void *user)
{
	auto *preview = static_cast<PreviewRaster *>(user);
	if (!preview || !preview->frame) return;
	CapturedFrame &frame = *preview->frame;
	const int previewW = std::max(1, preview->destW);
	const int previewH = std::max(1, preview->destH);
	for (int y = 0; y < height; ++y) {
		const int srcY = ((originY + y - preview->destY) * frame.height) / previewH;
		for (int x = 0; x < width; ++x) {
			const int srcX = ((originX + x - preview->destX) * frame.width) / previewW;
			pixels[y * width + x] = framePixel(frame, srcX, srcY);
		}
	}
}

}  // namespace

class CameraDriver {
public:
	static CameraDriver &instance()
	{
		static CameraDriver driver;
		return driver;
	}

	bool isAvailable() const { return true; }
	bool hasPermission() const { return true; }
	bool requestPermission() const { return true; }

	bool open(const std::string &facingHint, int preferredWidth, int preferredHeight)
	{
		ensureOpLock();
		close();
		if (!initializeVideo()) return false;
		fd_ = ::open(kDevicePath, O_RDWR);
		if (fd_ < 0) {
			ESP_LOGE(kTag, "open %s failed: errno=%d", kDevicePath, errno);
			return false;
		}

		if (facingHint == "back") facing_ = gea::platform::camera::Facing::Back;
		else if (facingHint == "external") facing_ = gea::platform::camera::Facing::External;
		else facing_ = gea::platform::camera::Facing::Front;
		mirror_ = facing_ == gea::platform::camera::Facing::Front;

		if (!queryAndConfigureFormat(preferredWidth, preferredHeight)) {
			close();
			return false;
		}
		configureFlipControls();
		if (!mapBuffers(kBufferCount)) {
			close();
			return false;
		}
		if (!setStreaming(true)) {
			close();
			return false;
		}
		open_ = true;
		ESP_LOGI(kTag, "camera ready %dx%d format=%c%c%c%c facing=%s",
		         width_,
		         height_,
		         static_cast<char>(format_ & 0xff),
		         static_cast<char>((format_ >> 8) & 0xff),
		         static_cast<char>((format_ >> 16) & 0xff),
		         static_cast<char>((format_ >> 24) & 0xff),
		         currentFacingString().c_str());
		return true;
	}

	void close()
	{
		if (fd_ >= 0) {
			if (streaming_) setStreaming(false);
			for (auto &buffer : buffers_) {
				if (buffer.data && buffer.length > 0) {
					munmap(buffer.data, buffer.length);
				}
				buffer = {};
			}
			::close(fd_);
		}
		fd_ = -1;
		open_ = false;
		streaming_ = false;
		bufferCount_ = 0;
	}

	bool isOpen() const { return open_; }
	int width() const { return width_; }
	int height() const { return height_; }
	int orientation() const { return 0; }
	gea::platform::camera::Facing currentFacing() const { return facing_; }

	std::string currentFacingString() const
	{
		switch (facing_) {
		case gea::platform::camera::Facing::Back:
			return "back";
		case gea::platform::camera::Facing::External:
			return "external";
		case gea::platform::camera::Facing::Front:
		default:
			return "front";
		}
	}

	int deviceCount() const { return 1; }

	gea::platform::camera::DeviceInfo deviceAt(int index) const
	{
		if (index != 0) return {};
		return {"mipi-csi-ov5647-center", gea::platform::camera::Facing::Front, kDefaultWidth, kDefaultHeight};
	}

	void drawPreview(int x, int y, int destW, int destH)
	{
		if (!open_) return;
		if (destW <= 0) destW = width_;
		if (destH <= 0) destH = height_;
		withFrame([&](CapturedFrame &frame) {
			PreviewRaster preview{.frame = &frame, .destX = x, .destY = y, .destW = destW, .destH = destH};
			gea::platform::display::Display::streamRect(x, y, destW, destH, rasterPreview, &preview);
			recordFrameIfDue(frame);
		});
	}

	// NativeOverlay: the camera owns the viewfinder rect and PPA-scales each frame
	// straight into the display framebuffer (presentNativeOverlay), so the renderer
	// never blits/replays/flushes the camera region.
	int previewMode() const { return 1; }
	void positionPreviewLayer(int x, int y, int width, int height)
	{
		vfX_ = x;
		vfY_ = y;
		vfW_ = width;
		vfH_ = height;
	}
	void hidePreviewLayer()
	{
		vfW_ = 0;
		vfH_ = 0;
	}
	void presentNativeOverlay()
	{
		if (open_ && vfW_ > 0 && vfH_ > 0) presentDirect(vfX_, vfY_, vfW_, vfH_);
	}

	// Scale + convert the latest frame into the caller's RGB565 buffer using the
	// requested fit (0=cover, 1=contain, 2=fill). Drives the <camera> leaf.
	bool fillPreview(std::uint16_t *dst, int dstW, int dstH, int fit, bool mirror)
	{
		if (!open_ || !dst || dstW <= 0 || dstH <= 0) return false;
		bool wrote = false;
		const std::int64_t t0 = esp_timer_get_time();
		std::int64_t tAfterDq = 0;
		withFrame([&](CapturedFrame &frame) {
			tAfterDq = esp_timer_get_time();
			const bool oldMirror = frame.mirror;
			frame.mirror = mirror || mirror_;
			// HW scale+rotate via the P4 PPA; fall back to the CPU scaler if the
			// PPA path can't handle this frame/format (or isn't available).
			if (!fillScaledPpa(frame, dst, dstW, dstH, fit))
				fillScaled(frame, dst, dstW, dstH, fit, kPreviewRotateCcw);
			frame.mirror = oldMirror;
			recordFrameIfDue(frame);
			wrote = true;
		});
		if (wrote) {
			const std::int64_t t2 = esp_timer_get_time();
			static std::int64_t accDq = 0, accScale = 0;
			static int n = 0;
			accDq += tAfterDq - t0;
			accScale += t2 - tAfterDq;
			if (++n >= 30) {
				ESP_LOGI(kTag, "preview perf: dqbuf=%lldus scale+qbuf=%lldus dst=%dx%d (avg/30)",
				         (long long)(accDq / 30), (long long)(accScale / 30), dstW, dstH);
				accDq = accScale = 0;
				n = 0;
			}
		}
		return wrote;
	}

	int capture(bool mirror)
	{
		if (!open_) return -1;
		int imageId = -1;
		withFrame([&](CapturedFrame &frame) {
			const int w = width_;
			const int h = height_;
			auto *pixels = static_cast<std::uint16_t *>(
				heap_caps_malloc(static_cast<std::size_t>(w) * h * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
			if (!pixels) return;
			const bool oldMirror = frame.mirror;
			frame.mirror = mirror || mirror_;
			for (int y = 0; y < h; ++y) {
				for (int x = 0; x < w; ++x) {
					pixels[y * w + x] = framePixel(frame, x, y);
				}
			}
			frame.mirror = oldMirror;
			imageId = gea::framework::graphics::ImageStore::instance().registerBuffer(pixels, w, h, -1);
			if (imageId < 0) heap_caps_free(pixels);
		});
		return imageId;
	}

	void setFlash(const std::string &mode) { setTorch(mode, 0.0); }

	void setZoom(double factor)
	{
		// The OV5647 has no optical/sensor zoom, so this is a digital zoom applied
		// in the preview scaler (ppaScaleRotateInto crops + up-scales by this factor).
		zoom_ = factor > 1.0 ? factor : 1.0;
	}

	void setMirror(bool mirror)
	{
		mirror_ = mirror;
		configureFlipControls();
	}

	// The OV5647/ISP pipeline only exposes the auto-exposure *target* brightness
	// (ESP_CAM_SENSOR_EXPOSURE_VAL, 2..235) — the AE loop then drives the sensor to
	// hit it. Map `bias` (EV-ish steps) around the default target so the button
	// visibly brightens/darkens the live image while staying auto-exposed.
	void setExposure(const std::string &mode, double bias, double iso, double durationMs)
	{
		(void)mode;
		(void)iso;
		(void)durationMs;
		int target = kAeTargetDefault + static_cast<int>(bias * 45.0);
		if (target < 16) target = 16;
		if (target > 230) target = 230;
		setExtControl(V4L2_CID_EXPOSURE, target);
	}

	// White balance lives in the ISP (V4L2_CID_USER_ESP_ISP_WB). enable=false hands
	// it back to the AWB algorithm; enable=true applies fixed red/blue gains so a
	// low colour temperature looks warm (boost red) and a high one looks cool
	// (boost blue).
	void setWhiteBalance(const std::string &mode, double temperatureK, double tint)
	{
		(void)tint;
		if (mode == "auto" || mode == "continuous") {
			setIspWb(false, 1.0f, 1.0f);  // resume AWB
			return;
		}
		float redGain = 1.6f, blueGain = 1.6f;  // neutral manual
		if (temperatureK > 0.0 && temperatureK <= 4500.0) {
			redGain = 2.1f;
			blueGain = 1.25f;  // warm
		} else if (temperatureK >= 6500.0) {
			redGain = 1.25f;
			blueGain = 2.1f;  // cool
		}
		setIspWb(true, redGain, blueGain);
	}

	// mode: "auto" | "continuous" | "locked"
	void setFocus(const std::string &mode, double, double)
	{
		setControl(V4L2_CID_FOCUS_AUTO, mode != "locked" ? 1 : 0);
	}

	// The OV5647 module on this board has no controllable LED, so torch is a
	// no-op here; the hook exists so the unified API stays uniform.
	void setTorch(const std::string &, double) {}

	bool startRecording(const std::string &path, double fps);
	double stopRecording();
	bool isRecording() const { return recording_; }

	// ---- Full-resolution capture-to-buffer (driven by the GEADEV dev commands) ----
	// These take over the camera for their full duration (preview frames are
	// skipped via opLock_) and hand back a heap_caps buffer the caller must free
	// with heap_caps_free(). Both run at the sensor's native size (1920x1080 in the
	// configured mode).

	// Grab one frame and HW-JPEG encode it at full resolution (quality 90). The
	// live preview already streams RGB565, which is a valid JPEG-encoder input, so
	// no format switch is needed for the still.
	bool captureStillJpeg(std::uint8_t **outBuf, std::size_t *outLen, int *outW, int *outH)
	{
#if GEA_P4_HAS_JPEG_ENCODER
		if (!open_ || !outBuf || !outLen) return false;
		ensureOpLock();
		xSemaphoreTake(opLock_, portMAX_DELAY);
		bool ok = false;
		// JPEG HW encoder input must be RGB565 here; restore it if a prior clip left
		// the device in YUV420 (recordH264Clip already restores, but be defensive).
		if (format_ != V4L2_PIX_FMT_RGB565) reconfigureFormat(V4L2_PIX_FMT_RGB565, kBufferCount);

		jpeg_encoder_handle_t enc = nullptr;
		jpeg_encode_engine_cfg_t engineCfg = {};
		engineCfg.timeout_ms = 2000;
		if (format_ == V4L2_PIX_FMT_RGB565 && jpeg_new_encoder_engine(&engineCfg, &enc) == ESP_OK) {
			withFrameLocked([&](CapturedFrame &frame) {
				if (frame.format != V4L2_PIX_FMT_RGB565) return;
				const std::size_t cap = static_cast<std::size_t>(frame.width) * frame.height * 2u;
				jpeg_encode_memory_alloc_cfg_t memCfg = {};
				memCfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
				std::size_t allocated = 0;
				auto *jbuf = static_cast<std::uint8_t *>(jpeg_alloc_encoder_mem(cap, &memCfg, &allocated));
				if (!jbuf) return;
				jpeg_encode_cfg_t jc = {};
				jc.height = static_cast<std::uint32_t>(frame.height);
				jc.width = static_cast<std::uint32_t>(frame.width);
				jc.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
				jc.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
				jc.image_quality = 90;
				std::uint32_t jlen = 0;
				const esp_err_t e = jpeg_encoder_process(enc, &jc, frame.data,
				                                         static_cast<std::uint32_t>(frame.bytes), jbuf,
				                                         static_cast<std::uint32_t>(allocated), &jlen);
				if (e == ESP_OK && jlen > 0) {
					*outBuf = jbuf;
					*outLen = jlen;
					if (outW) *outW = frame.width;
					if (outH) *outH = frame.height;
					ok = true;
				} else {
					heap_caps_free(jbuf);
				}
			});
			jpeg_del_encoder_engine(enc);
		}
		xSemaphoreGive(opLock_);
		return ok;
#else
		(void)outBuf;
		(void)outLen;
		(void)outW;
		(void)outH;
		return false;
#endif
	}

	// Record `seconds` of 1920x1080 H.264 via the P4 hardware encoder (/dev/video11
	// V4L2 M2M). Switches /dev/video0 to YUV420 (the encoder's input format), runs
	// the capture->encode loop at the sensor frame rate writing the Annex-B
	// elementary stream straight to `path` on the SD card (no PSRAM size cap), then
	// restores RGB565 so the preview resumes.
	bool recordH264ClipToFile(double seconds, const char *path, std::size_t *outBytes, int *outW, int *outH,
	                          double *outFps, int *outFrames)
	{
		if (!open_ || !path) return false;
		ensureOpLock();
		xSemaphoreTake(opLock_, portMAX_DELAY);

		bool ok = false;
		int m2m = -1;
		std::uint8_t *m2mCap = nullptr;
		std::size_t m2mCapLen = 0;
		std::FILE *outFile = nullptr;
		const int W = width_;
		const int H = height_;
		const std::size_t yuvLen = static_cast<std::size_t>(W) * H * 3u / 2u;

		do {
			// 0) Reserve the clip buffer FIRST, while PSRAM still has a large contiguous
			// free region. Doing it after reconfigureFormat() + the encoder's ~8MB
			// capture buffer fragments the heap and a 7MB contiguous alloc then fails.
			// Allocated once and reused across captures.
			const std::size_t accCap = 7u * 1024u * 1024u;
			if (!clipBuf_) {
				clipBuf_ = static_cast<std::uint8_t *>(heap_caps_malloc(accCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
				clipCap_ = clipBuf_ ? accCap : 0;
			}
			if (!clipBuf_) {
				ESP_LOGE(kTag, "h264: clip buffer alloc (%u) failed", static_cast<unsigned>(accCap));
				break;
			}

			// 1) Camera output -> YUV420 (encoder input). 2 ring buffers keep PSRAM
			// headroom for the encoder's large capture buffer (~W*H*4) + the clip.
			if (!reconfigureFormat(V4L2_PIX_FMT_YUV420, 2)) {
				ESP_LOGE(kTag, "h264: failed to switch sensor to YUV420");
				break;
			}

			// 2) Open + configure the HW H.264 M2M encoder device.
			m2m = ::open(ESP_VIDEO_H264_DEVICE_NAME, O_RDWR);
			if (m2m < 0) {
				ESP_LOGE(kTag, "h264: open %s failed errno=%d", ESP_VIDEO_H264_DEVICE_NAME, errno);
				break;
			}
			setH264Ctrl(m2m, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 30);
			setH264Ctrl(m2m, V4L2_CID_MPEG_VIDEO_BITRATE, 5000000);  // 5 Mbps target (good 1080p)
			// Writing to SD removes the old PSRAM size cap, so we can afford higher
			// quality. min_qp=22 still bounds per-frame size enough that the HW rate
			// controller (slow to adapt) can't balloon a detailed scene unboundedly.
			setH264Ctrl(m2m, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 22);
			setH264Ctrl(m2m, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 51);

			v4l2_format ofmt{};
			ofmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			ofmt.fmt.pix.width = static_cast<std::uint32_t>(W);
			ofmt.fmt.pix.height = static_cast<std::uint32_t>(H);
			ofmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
			if (ioctl(m2m, VIDIOC_S_FMT, &ofmt) != 0) {
				ESP_LOGE(kTag, "h264: S_FMT output failed errno=%d", errno);
				break;
			}
			v4l2_requestbuffers oreq{};
			oreq.count = 1;
			oreq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			oreq.memory = V4L2_MEMORY_USERPTR;
			if (ioctl(m2m, VIDIOC_REQBUFS, &oreq) != 0) {
				ESP_LOGE(kTag, "h264: REQBUFS output failed errno=%d", errno);
				break;
			}

			v4l2_format cfmt{};
			cfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			cfmt.fmt.pix.width = static_cast<std::uint32_t>(W);
			cfmt.fmt.pix.height = static_cast<std::uint32_t>(H);
			cfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
			if (ioctl(m2m, VIDIOC_S_FMT, &cfmt) != 0) {
				ESP_LOGE(kTag, "h264: S_FMT capture failed errno=%d", errno);
				break;
			}
			v4l2_requestbuffers creq{};
			creq.count = 1;
			creq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			creq.memory = V4L2_MEMORY_MMAP;
			if (ioctl(m2m, VIDIOC_REQBUFS, &creq) != 0) {
				ESP_LOGE(kTag, "h264: REQBUFS capture failed errno=%d", errno);
				break;
			}
			v4l2_buffer qcap{};
			qcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			qcap.memory = V4L2_MEMORY_MMAP;
			qcap.index = 0;
			if (ioctl(m2m, VIDIOC_QUERYBUF, &qcap) != 0) {
				ESP_LOGE(kTag, "h264: QUERYBUF failed errno=%d", errno);
				break;
			}
			m2mCapLen = qcap.length;
			m2mCap = static_cast<std::uint8_t *>(
				mmap(nullptr, qcap.length, PROT_READ | PROT_WRITE, MAP_SHARED, m2m, qcap.m.offset));
			if (m2mCap == reinterpret_cast<void *>(static_cast<intptr_t>(-1))) {
				m2mCap = nullptr;
				ESP_LOGE(kTag, "h264: mmap capture failed errno=%d", errno);
				break;
			}
			if (ioctl(m2m, VIDIOC_QBUF, &qcap) != 0) {
				ESP_LOGE(kTag, "h264: initial QBUF capture failed errno=%d", errno);
				break;
			}
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(m2m, VIDIOC_STREAMON, &type) != 0) {
				ESP_LOGE(kTag, "h264: STREAMON capture failed errno=%d", errno);
				break;
			}
			type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			if (ioctl(m2m, VIDIOC_STREAMON, &type) != 0) {
				ESP_LOGE(kTag, "h264: STREAMON output failed errno=%d", errno);
				break;
			}

			// Warm up: right after the YUV420 format switch + stream-on the ISP/sensor
			// emits a handful of black/invalid frames. Drain them (no encode) so the
			// clip opens on a real frame instead of black.
			for (int i = 0; i < 10; ++i) {
				v4l2_buffer wb{};
				wb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				wb.memory = V4L2_MEMORY_MMAP;
				if (ioctl(fd_, VIDIOC_DQBUF, &wb) == 0) ioctl(fd_, VIDIOC_QBUF, &wb);
			}

			// 3) Record into the PSRAM clip buffer (allocated in step 0), then bulk-write
			// to SD after. A per-frame fwrite inside the encode loop stalls it and ~halves
			// the frame rate, so we buffer in RAM (fast memcpy) and write the clip once.
			std::uint8_t *acc = clipBuf_;
			std::size_t accLen = 0;
			int frames = 0;
			const std::int64_t startUs = esp_timer_get_time();
			const std::int64_t durUs = static_cast<std::int64_t>(seconds * 1e6);
			const std::int64_t hardStopUs = startUs + durUs + 5 * 1000000;  // safety

			while (true) {
				const std::int64_t now = esp_timer_get_time();
				if (now - startUs >= durUs) break;
				if (now >= hardStopUs) break;

				v4l2_buffer capBuf{};
				capBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				capBuf.memory = V4L2_MEMORY_MMAP;
				if (ioctl(fd_, VIDIOC_DQBUF, &capBuf) != 0) {
					vTaskDelay(1);
					continue;
				}
				const int idx = static_cast<int>(capBuf.index);
				if (idx < 0 || idx >= bufferCount_ || !buffers_[idx].data) {
					ioctl(fd_, VIDIOC_QBUF, &capBuf);
					continue;
				}

				// Feed the camera frame to the encoder (zero-copy via USERPTR). Use the
				// full YUV420 size, not bytesused (the ISP reports the RAW10 input size).
				v4l2_buffer mout{};
				mout.index = 0;
				mout.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
				mout.memory = V4L2_MEMORY_USERPTR;
				mout.m.userptr = reinterpret_cast<unsigned long>(buffers_[idx].data);
				mout.length = static_cast<std::uint32_t>(yuvLen);
				if (ioctl(m2m, VIDIOC_QBUF, &mout) != 0) {
					ioctl(fd_, VIDIOC_QBUF, &capBuf);
					continue;
				}

				v4l2_buffer mcap{};
				mcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				mcap.memory = V4L2_MEMORY_MMAP;
				if (ioctl(m2m, VIDIOC_DQBUF, &mcap) == 0) {
					const std::size_t n = mcap.bytesused;
					if (n > 0 && accLen + n <= clipCap_) {
						std::memcpy(acc + accLen, m2mCap, n);
						accLen += n;
						++frames;
					}
					ioctl(m2m, VIDIOC_QBUF, &mcap);  // requeue the encoder output buffer
				}
				ioctl(fd_, VIDIOC_QBUF, &capBuf);  // requeue the camera frame

				v4l2_buffer mout2{};
				mout2.index = 0;
				mout2.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
				mout2.memory = V4L2_MEMORY_USERPTR;
				ioctl(m2m, VIDIOC_DQBUF, &mout2);  // reclaim the encoder input buffer

				if (accLen + 256u * 1024u > clipCap_) break;  // out of clip-buffer headroom
			}

			// fps is measured over the CAPTURE window only — not the SD write that
			// follows — so the remuxed clip plays back at real time.
			const double durS = (esp_timer_get_time() - startUs) / 1e6;

			// Bulk-write the captured clip to the SD card (sequential, fast).
			outFile = std::fopen(path, "wb");
			if (!outFile) {
				ESP_LOGE(kTag, "h264: fopen %s failed errno=%d", path, errno);
				break;
			}
			const std::size_t wrote = std::fwrite(acc, 1, accLen, outFile);
			std::fflush(outFile);
			std::fclose(outFile);
			outFile = nullptr;
			if (wrote != accLen) {
				ESP_LOGE(kTag, "h264: short SD write %u/%u", static_cast<unsigned>(wrote), static_cast<unsigned>(accLen));
				accLen = wrote;
			}

			if (outBytes) *outBytes = accLen;
			if (outW) *outW = W;
			if (outH) *outH = H;
			if (outFrames) *outFrames = frames;
			if (outFps) *outFps = durS > 0.0 ? frames / durS : 0.0;
			ok = accLen > 0;
		} while (false);

		if (outFile) std::fclose(outFile);

		// Teardown the encoder.
		if (m2m >= 0) {
			int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			ioctl(m2m, VIDIOC_STREAMOFF, &type);
			type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			ioctl(m2m, VIDIOC_STREAMOFF, &type);
			if (m2mCap) munmap(m2mCap, m2mCapLen);
			::close(m2m);
		}
		// clipBuf_ is persistent and reused across captures — never freed here.

		// Restore RGB565 so the live preview resumes.
		reconfigureFormat(V4L2_PIX_FMT_RGB565, kBufferCount);
		xSemaphoreGive(opLock_);
		return ok;
	}

	// Mount SD (if needed), capture one full-res JPEG, and write it to `path`.
	bool captureStillToSd(const char *path, std::size_t *outBytes, int *outW, int *outH)
	{
		if (!ensureSdMounted() || !path) return false;
		std::uint8_t *buf = nullptr;
		std::size_t len = 0;
		int w = 0, h = 0;
		if (!captureStillJpeg(&buf, &len, &w, &h) || !buf) return false;
		std::FILE *f = std::fopen(path, "wb");
		bool ok = false;
		if (f) {
			ok = std::fwrite(buf, 1, len, f) == len;
			std::fflush(f);
			std::fclose(f);
		} else {
			ESP_LOGE(kTag, "still: fopen %s failed errno=%d", path, errno);
		}
		heap_caps_free(buf);
		if (ok) {
			if (outBytes) *outBytes = len;
			if (outW) *outW = w;
			if (outH) *outH = h;
		}
		return ok;
	}

	// Mount SD (if needed) and record an H.264 clip straight to `path`.
	bool recordClipToSd(double seconds, const char *path, std::size_t *outBytes, int *outW, int *outH,
	                    double *outFps, int *outFrames)
	{
		if (!ensureSdMounted()) return false;
		return recordH264ClipToFile(seconds, path, outBytes, outW, outH, outFps, outFrames);
	}

private:
	// Preview / one-shot frame access from the render task. Serialized against the
	// long-running capture ops (captureStillJpeg / recordH264Clip) via opLock_:
	// while a capture holds the lock — it may reconfigure /dev/video0's pixel
	// format (RGB565 <-> YUV420) — preview frames are skipped rather than racing a
	// half-reconfigured device. A capture in progress => return false (the renderer
	// just keeps the last frame on screen for the capture's duration).
	template <typename Callback>
	bool withFrame(Callback callback)
	{
		if (fd_ < 0 || !streaming_) return false;
		if (opLock_ && xSemaphoreTake(opLock_, 0) != pdTRUE) return false;
		const bool ok = withFrameLocked(callback);
		if (opLock_) xSemaphoreGive(opLock_);
		return ok;
	}

	// Dequeue one frame, invoke callback, requeue. Caller must already hold opLock_
	// (or run before the preview task exists). Used directly by the capture ops.
	template <typename Callback>
	bool withFrameLocked(Callback callback)
	{
		if (fd_ < 0 || !streaming_) return false;
		v4l2_buffer buffer{};
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		const std::int64_t dqStartUs = esp_timer_get_time();
		if (ioctl(fd_, VIDIOC_DQBUF, &buffer) != 0) {
			ESP_LOGW(kTag, "DQBUF failed: errno=%d", errno);
			return false;
		}
		const std::int64_t cbStartUs = esp_timer_get_time();
		const int index = static_cast<int>(buffer.index);
		if (index >= 0 && index < bufferCount_ && buffers_[index].data) {
			// Use the whole mmap'd buffer, not VIDIOC_DQBUF's bytesused: on this
			// board the ISP demosaics the RAW10 binning frame to a full 1280x960
			// RGB565 image (2,457,600 bytes), but bytesused reports the RAW10 *input*
			// size (1,536,000) — trusting it cropped a complete frame to 1280x600.
			// The buffer is fully written each frame, so its length is the truth.
			const std::size_t usedBytes = buffers_[index].length;
			static bool s_msyncTested = false;
			if (!s_msyncTested) {
				s_msyncTested = true;
				const std::int64_t m0 = esp_timer_get_time();
				esp_cache_msync(buffers_[index].data, usedBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
				const std::int64_t m1 = esp_timer_get_time();
				ESP_LOGI(kTag, "TEST: esp_cache_msync %u bytes C2M = %lldus", static_cast<unsigned>(usedBytes), m1 - m0);
			}
			CapturedFrame frame{
				.data = buffers_[index].data,
				.bytes = usedBytes,
				.width = width_,
				.height = height_,
				.format = format_,
				.mirror = mirror_,
			};
			callback(frame);
		}
		const std::int64_t cbEndUs = esp_timer_get_time();
		// TEMP perf: dq-wait vs callback (scale+present) split, avg over 30 frames.
		dbgDqUs_ += cbStartUs - dqStartUs;
		dbgCbUs_ += cbEndUs - cbStartUs;
		if (++dbgFrames_ >= 30) {
			ESP_LOGI(kTag, "withFrame perf: dqbuf=%lldus callback=%lldus (avg/30)",
			         dbgDqUs_ / dbgFrames_, dbgCbUs_ / dbgFrames_);
			dbgDqUs_ = dbgCbUs_ = 0;
			dbgFrames_ = 0;
		}
		if (ioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
			ESP_LOGE(kTag, "QBUF failed: errno=%d", errno);
			return false;
		}
		return true;
	}

	bool initializeVideo()
	{
		if (videoInitialized_) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus is not ready for camera SCCB");
			return false;
		}
		esp_video_init_csi_config_t csiConfig{};
		csiConfig.sccb_config.init_sccb = false;
		csiConfig.sccb_config.i2c_handle = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
		csiConfig.sccb_config.freq = kSccbFrequencyHz;
		csiConfig.reset_pin = -1;
		csiConfig.pwdn_pin = -1;

		esp_video_init_config_t videoConfig{};
		videoConfig.csi = &csiConfig;

		const esp_err_t err = esp_video_init(&videoConfig);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "esp_video_init failed: %s", esp_err_to_name(err));
			return false;
		}
		videoInitialized_ = true;
		return true;
	}

	bool queryAndConfigureFormat(int preferredWidth, int preferredHeight)
	{
		v4l2_capability capability{};
		if (ioctl(fd_, VIDIOC_QUERYCAP, &capability) != 0) {
			ESP_LOGE(kTag, "VIDIOC_QUERYCAP failed: errno=%d", errno);
			return false;
		}
		ESP_LOGI(kTag, "driver=%s card=%s bus=%s", capability.driver, capability.card, capability.bus_info);

		auto fourcc = [](std::uint32_t f, char *out) {
			out[0] = static_cast<char>(f & 0xff);
			out[1] = static_cast<char>((f >> 8) & 0xff);
			out[2] = static_cast<char>((f >> 16) & 0xff);
			out[3] = static_cast<char>((f >> 24) & 0xff);
			out[4] = '\0';
		};

		// The ISP pipeline's default output format/size for the active sensor mode.
		v4l2_format format{};
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(fd_, VIDIOC_G_FMT, &format) != 0) {
			ESP_LOGE(kTag, "VIDIOC_G_FMT failed: errno=%d", errno);
			return false;
		}
		char cc[5];
		fourcc(format.fmt.pix.pixelformat, cc);
		ESP_LOGI(kTag, "default fmt %s %ux%u", cc, format.fmt.pix.width, format.fmt.pix.height);

		// Log every output format the device (ISP) actually supports — RGB565 may
		// not be one of them, so we negotiate rather than hardcode it.
		for (int i = 0;; ++i) {
			v4l2_fmtdesc desc{};
			desc.index = static_cast<std::uint32_t>(i);
			desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(fd_, VIDIOC_ENUM_FMT, &desc) != 0) break;
			fourcc(desc.pixelformat, cc);
			ESP_LOGI(kTag, "supported fmt[%d]=%s", i, cc);
		}

		// Prefer a format framePixel() can convert, at the ISP's native output size
		// (don't override width/height — the ISP locks them to the sensor mode chosen
		// at esp_video init via CONFIG_CAMERA_OV5647_MIPI_*; e.g. the 1280x960 binning
		// mode is selected there, not here).
		const std::uint32_t prefs[] = {
			V4L2_PIX_FMT_RGB565, V4L2_PIX_FMT_RGB24, V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_YUV422P,
		};
		const std::uint32_t nativeW = format.fmt.pix.width;
		const std::uint32_t nativeH = format.fmt.pix.height;
		bool set = false;
		for (std::uint32_t pf : prefs) {
			v4l2_format trial = format;
			trial.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			trial.fmt.pix.width = nativeW;
			trial.fmt.pix.height = nativeH;
			trial.fmt.pix.pixelformat = pf;
			if (ioctl(fd_, VIDIOC_S_FMT, &trial) == 0) {
				format = trial;
				fourcc(pf, cc);
				ESP_LOGI(kTag, "using fmt %s %ux%u", cc, format.fmt.pix.width, format.fmt.pix.height);
				set = true;
				break;
			}
		}
		if (!set) {
			// Nothing preferred took — fall back to the ISP default (framePixel also
			// handles SBGGR8/GREY), so we still stream rather than bail.
			std::memset(&format, 0, sizeof(format));
			format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (ioctl(fd_, VIDIOC_G_FMT, &format) != 0) {
				ESP_LOGE(kTag, "VIDIOC_G_FMT fallback failed: errno=%d", errno);
				return false;
			}
			fourcc(format.fmt.pix.pixelformat, cc);
			ESP_LOGW(kTag, "no preferred fmt accepted; streaming default %s %ux%u",
			         cc, format.fmt.pix.width, format.fmt.pix.height);
		}
		(void)preferredWidth;
		(void)preferredHeight;

		width_ = static_cast<int>(format.fmt.pix.width);
		height_ = static_cast<int>(format.fmt.pix.height);
		format_ = format.fmt.pix.pixelformat;
		return width_ > 0 && height_ > 0;
	}

	bool mapBuffers(int count)
	{
		v4l2_requestbuffers request{};
		request.count = static_cast<std::uint32_t>(std::min(count, kBufferCount));
		request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		request.memory = V4L2_MEMORY_MMAP;
		if (ioctl(fd_, VIDIOC_REQBUFS, &request) != 0) {
			ESP_LOGE(kTag, "VIDIOC_REQBUFS failed: errno=%d", errno);
			return false;
		}
		bufferCount_ = std::min<int>(request.count, kBufferCount);
		if (bufferCount_ < 2) {
			ESP_LOGE(kTag, "camera returned too few buffers: %d", bufferCount_);
			return false;
		}

		for (int i = 0; i < bufferCount_; ++i) {
			v4l2_buffer buffer{};
			buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buffer.memory = V4L2_MEMORY_MMAP;
			buffer.index = i;
			if (ioctl(fd_, VIDIOC_QUERYBUF, &buffer) != 0) {
				ESP_LOGE(kTag, "VIDIOC_QUERYBUF(%d) failed: errno=%d", i, errno);
				return false;
			}
			void *mapped = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buffer.m.offset);
			if (mapped == reinterpret_cast<void *>(static_cast<intptr_t>(-1))) {
				ESP_LOGE(kTag, "mmap camera buffer %d failed: errno=%d", i, errno);
				return false;
			}
			buffers_[i] = {static_cast<std::uint8_t *>(mapped), buffer.length};
			if (ioctl(fd_, VIDIOC_QBUF, &buffer) != 0) {
				ESP_LOGE(kTag, "initial QBUF(%d) failed: errno=%d", i, errno);
				return false;
			}
		}
		return true;
	}

	bool setStreaming(bool enabled)
	{
		const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(fd_, enabled ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type) != 0) {
			ESP_LOGE(kTag, "%s failed: errno=%d", enabled ? "VIDIOC_STREAMON" : "VIDIOC_STREAMOFF", errno);
			return false;
		}
		streaming_ = enabled;
		return true;
	}

	// Switch /dev/video0's output pixel format in place (the sensor mode and
	// resolution are unchanged; only the ISP output format + buffer count change).
	// Caller must hold opLock_. Used to flip between RGB565 (preview / JPEG still)
	// and YUV420 (the H.264 encoder's input format).
	bool reconfigureFormat(std::uint32_t pixfmt, int count)
	{
		if (fd_ < 0) return false;
		if (streaming_) setStreaming(false);
		for (auto &buffer : buffers_) {
			if (buffer.data && buffer.length > 0) munmap(buffer.data, buffer.length);
			buffer = {};
		}
		v4l2_requestbuffers freeReq{};
		freeReq.count = 0;
		freeReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		freeReq.memory = V4L2_MEMORY_MMAP;
		ioctl(fd_, VIDIOC_REQBUFS, &freeReq);  // release the old buffer set

		v4l2_format format{};
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = static_cast<std::uint32_t>(width_);
		format.fmt.pix.height = static_cast<std::uint32_t>(height_);
		format.fmt.pix.pixelformat = pixfmt;
		if (ioctl(fd_, VIDIOC_S_FMT, &format) != 0) {
			ESP_LOGE(kTag, "reconfigureFormat S_FMT 0x%" PRIx32 " failed errno=%d", pixfmt, errno);
			return false;
		}
		width_ = static_cast<int>(format.fmt.pix.width);
		height_ = static_cast<int>(format.fmt.pix.height);
		format_ = format.fmt.pix.pixelformat;
		if (!mapBuffers(count)) return false;
		return setStreaming(true);
	}

	// Set one H.264 M2M encoder control (bitrate / I-period / QP). The encoder uses
	// the extended-control API with the codec control class.
	bool setH264Ctrl(int fd, std::uint32_t id, int value)
	{
		v4l2_ext_control ctrl{};
		ctrl.id = id;
		ctrl.value = value;
		v4l2_ext_controls ctrls{};
		ctrls.ctrl_class = V4L2_CID_CODEC_CLASS;
		ctrls.count = 1;
		ctrls.controls = &ctrl;
		if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
			ESP_LOGW(kTag, "h264 ctrl 0x%" PRIx32 "=%d failed errno=%d", id, value, errno);
			return false;
		}
		return true;
	}

	void ensureOpLock()
	{
		if (!opLock_) opLock_ = xSemaphoreCreateMutex();
	}

	// Mount the microSD card at /sdcard (SDMMC slot 1, 4-bit). Waveshare ESP32-P4
	// wiring: CLK=43, CMD=44, D0..D3=39..42. The P4's SD I/O rail is powered by the
	// on-chip LDO (channel 4), which must be brought up before the host. Mounts once.
	bool ensureSdMounted()
	{
		if (sdMounted_) return true;

		sdmmc_host_t host = SDMMC_HOST_DEFAULT();
		host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40 MHz
		sd_pwr_ctrl_ldo_config_t ldoConfig = {};
		ldoConfig.ldo_chan_id = 4;  // ESP32-P4 internal LDO feeding the SD I/O rail
		if (!sdPwr_ && sd_pwr_ctrl_new_on_chip_ldo(&ldoConfig, &sdPwr_) != ESP_OK) {
			ESP_LOGE(kTag, "sd: on-chip LDO power-ctrl init failed");
			sdPwr_ = nullptr;
		}
		if (sdPwr_) host.pwr_ctrl_handle = sdPwr_;

		sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
		slot.width = 4;
		slot.clk = static_cast<gpio_num_t>(43);
		slot.cmd = static_cast<gpio_num_t>(44);
		slot.d0 = static_cast<gpio_num_t>(39);
		slot.d1 = static_cast<gpio_num_t>(40);
		slot.d2 = static_cast<gpio_num_t>(41);
		slot.d3 = static_cast<gpio_num_t>(42);
		slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

		esp_vfs_fat_sdmmc_mount_config_t mountConfig = {};
		mountConfig.format_if_mount_failed = false;
		mountConfig.max_files = 4;
		mountConfig.allocation_unit_size = 64 * 1024;

		const esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mountConfig, &sdCard_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "sd: mount failed: %s", esp_err_to_name(err));
			return false;
		}
		sdMounted_ = true;
		const std::uint64_t mb =
			(static_cast<std::uint64_t>(sdCard_->csd.capacity) * sdCard_->csd.sector_size) >> 20;
		ESP_LOGI(kTag, "sd: mounted /sdcard (%llu MB)", static_cast<unsigned long long>(mb));
		return true;
	}

	bool setControl(std::uint32_t id, int value)
	{
		if (fd_ < 0) return false;
		v4l2_control control{};
		control.id = id;
		control.value = value;
		if (ioctl(fd_, VIDIOC_S_CTRL, &control) != 0) {
			ESP_LOGW(kTag, "VIDIOC_S_CTRL 0x%" PRIx32 "=%d failed: errno=%d", id, value, errno);
			return false;
		}
		return true;
	}

	// esp_video exposes sensor/ISP controls only through the EXTENDED control API
	// (VIDIOC_S_EXT_CTRLS); the legacy VIDIOC_S_CTRL above returns EINVAL for them.
	// Scalar variant (e.g. V4L2_CID_EXPOSURE -> sensor AE target).
	bool setExtControl(std::uint32_t id, int value)
	{
		if (fd_ < 0) return false;
		v4l2_ext_control ctrl{};
		ctrl.id = id;
		ctrl.value = value;
		v4l2_ext_controls ctrls{};
		ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
		ctrls.count = 1;
		ctrls.controls = &ctrl;
		if (ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
			ESP_LOGW(kTag, "S_EXT_CTRLS 0x%" PRIx32 "=%d failed: errno=%d", id, value, errno);
			return false;
		}
		return true;
	}

	// Compound variant for the ISP white-balance control (a struct passed by pointer).
	bool setIspWb(bool enable, float redGain, float blueGain)
	{
		if (fd_ < 0) return false;
		esp_video_isp_wb_t wb{};
		wb.enable = enable;
		wb.red_gain = redGain;
		wb.blue_gain = blueGain;
		v4l2_ext_control ctrl{};
		ctrl.id = V4L2_CID_USER_ESP_ISP_WB;
		ctrl.size = sizeof(wb);
		ctrl.p_u8 = reinterpret_cast<std::uint8_t *>(&wb);
		v4l2_ext_controls ctrls{};
		ctrls.ctrl_class = V4L2_CTRL_CLASS_USER;
		ctrls.count = 1;
		ctrls.controls = &ctrl;
		if (ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
			ESP_LOGW(kTag, "S_EXT_CTRLS ISP_WB(en=%d r=%.2f b=%.2f) failed: errno=%d", enable, redGain, blueGain, errno);
			return false;
		}
		return true;
	}

	void configureFlipControls()
	{
		setControl(V4L2_CID_VFLIP, 1);
		setControl(V4L2_CID_HFLIP, mirror_ ? 1 : 0);
	}

	// Scale the latest frame into dst (RGB565), honouring the fit mode:
	// 0=cover (crop to dst aspect), 1=contain (letterbox), 2=fill (stretch).
	// rotateCcw rotates the source by 90*rotateCcw degrees counter-clockwise
	// while scaling. The OV5647 module is mounted 90deg off the landscape
	// viewport, so the preview rotates it back: fit + crop math run on the
	// *rotated* source dimensions, and each pixel read remaps back into the real
	// (unrotated) frame so a single pass both un-rotates and fills the viewfinder.
	void fillScaled(const CapturedFrame &frame, std::uint16_t *dst, int dstW, int dstH, int fit, int rotateCcw = 0)
	{
		const int srcW = frame.width;
		const int srcH = frame.height;
		if (srcW <= 0 || srcH <= 0) return;

		const int rot = ((rotateCcw % 4) + 4) % 4;
		const bool swap = (rot & 1) != 0;
		const int rsrcW = swap ? srcH : srcW;  // rotated-source width
		const int rsrcH = swap ? srcW : srcH;  // rotated-source height

		double sx0 = 0.0, sy0 = 0.0, sw = rsrcW, sh = rsrcH;  // sampled (rotated) source span
		int dx0 = 0, dy0 = 0, dw = dstW, dh = dstH;           // dst image rect (contain)
		const double srcAspect = static_cast<double>(rsrcW) / rsrcH;
		const double dstAspect = static_cast<double>(dstW) / dstH;
		if (fit == 0) {  // cover
			if (srcAspect > dstAspect) {
				sw = rsrcH * dstAspect;
				sx0 = (rsrcW - sw) / 2.0;
			} else {
				sh = rsrcW / dstAspect;
				sy0 = (rsrcH - sh) / 2.0;
			}
		} else if (fit == 1) {  // contain
			if (srcAspect > dstAspect) {
				dh = static_cast<int>(dstW / srcAspect);
				dy0 = (dstH - dh) / 2;
			} else {
				dw = static_cast<int>(dstH * srcAspect);
				dx0 = (dstW - dw) / 2;
			}
		}

		// Per-pixel source addressing in 16.16 fixed point. The ESP32-P4 FPU is
		// single-precision only, so the previous per-pixel `double` divides were
		// software-emulated (~1 s/frame for a 468x628 viewfinder). Precompute the
		// source step once and accumulate per pixel — integer add, no divide.
		const int spanW = (fit == 1) ? dw : dstW;
		const int spanH = (fit == 1) ? dh : dstH;
		const std::int64_t sxStep = spanW > 0 ? static_cast<std::int64_t>(sw * 65536.0) / spanW : 0;
		const std::int64_t syStep = spanH > 0 ? static_cast<std::int64_t>(sh * 65536.0) / spanH : 0;
		const std::int64_t sxBase = static_cast<std::int64_t>(sx0 * 65536.0);
		const std::int64_t syBase = static_cast<std::int64_t>(sy0 * 65536.0);

		// Map a rotated-source coord (lx, ly) back to the real frame and read it.
		const auto sampleRotated = [&](int lx, int ly) -> std::uint16_t {
			int ox, oy;
			switch (rot) {
			case 1: ox = srcW - 1 - ly; oy = lx; break;             // 90 CCW
			case 2: ox = srcW - 1 - lx; oy = srcH - 1 - ly; break;  // 180
			case 3: ox = ly; oy = srcH - 1 - lx; break;             // 270 CCW (= 90 CW)
			default: ox = lx; oy = ly; break;                       // 0
			}
			return framePixel(frame, ox, oy);
		};

		for (int y = 0; y < dstH; ++y) {
			std::uint16_t *row = dst + static_cast<std::size_t>(y) * dstW;
			if (fit == 1 && (y < dy0 || y >= dy0 + dh)) {
				std::memset(row, 0, static_cast<std::size_t>(dstW) * sizeof(std::uint16_t));
				continue;
			}
			const std::int64_t syFx = (fit == 1)
			                              ? syBase + static_cast<std::int64_t>(y - dy0) * syStep
			                              : syBase + static_cast<std::int64_t>(y) * syStep;
			const int ly = static_cast<int>(syFx >> 16);
			if (fit == 1) {
				int x = 0;
				for (; x < dx0 && x < dstW; ++x) row[x] = 0;  // left bar
				std::int64_t sxFx = sxBase;
				for (; x < dstW && x < dx0 + dw; ++x) {
					row[x] = sampleRotated(static_cast<int>(sxFx >> 16), ly);
					sxFx += sxStep;
				}
				for (; x < dstW; ++x) row[x] = 0;  // right bar
			} else {
				std::int64_t sxFx = sxBase;
				for (int x = 0; x < dstW; ++x) {
					row[x] = sampleRotated(static_cast<int>(sxFx >> 16), ly);
					sxFx += sxStep;
				}
			}
		}
	}

	bool initPpa()
	{
		if (ppaClient_) return true;
		ppa_client_config_t cfg = {};
		cfg.oper_type = PPA_OPERATION_SRM;
		cfg.max_pending_trans_num = 1;
		cfg.data_burst_length = PPA_DATA_BURST_LENGTH_128;
		if (ppa_register_client(&cfg, &ppaClient_) != ESP_OK) {
			ppaClient_ = nullptr;
			return false;
		}
		return true;
	}

	// Black the camera surface once (and after a resize) and flush it to RAM: the
	// PPA's scaled+rotated output block can land a pixel or two short of the
	// surface (scale quantisation), so the few edge pixels it never writes stay
	// black instead of showing uninitialised noise.
	void primePpaSurface(std::uint16_t *dst, std::size_t bytes)
	{
		if (dst == ppaLastDst_ && bytes == ppaLastBytes_) return;
		ppaLastDst_ = dst;
		ppaLastBytes_ = bytes;
		std::memset(dst, 0, bytes);
		esp_cache_msync(dst, bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
	}

	// Core PPA scale+rotate: the latest frame lands as a dstW x dstH block at
	// (outOffX,outOffY) inside the outPicW x outPicH output picture. The PPA
	// truncates the scale to 1/16 steps and sets output = floor(block*scale), so a
	// plain cover scale lands a few px short (black edge); round the cover scale UP
	// to the next 1/16, then size the centred input crop so the truncated output
	// is EXACTLY dstW x dstH. Returns false if the PPA can't service this frame.
	bool ppaScaleRotateInto(const CapturedFrame &frame, std::uint16_t *outBuf, std::size_t outBytes,
	                        int outPicW, int outPicH, int outOffX, int outOffY, int dstW, int dstH,
	                        int rotCcw = kPreviewRotateCcw)
	{
		if (!frame.data || frame.width <= 0 || frame.height <= 0) return false;
		if (frame.format != V4L2_PIX_FMT_RGB565) return false;  // RGB565 in/out only
		if (!initPpa()) return false;
		// PPA output to PSRAM must be cache-line aligned (queried in initPpa).
		if ((reinterpret_cast<std::uintptr_t>(outBuf) & (ppaOutAlign_ - 1)) != 0) return false;

		const int rot = ((rotCcw % 4) + 4) % 4;
		const bool swap = (rot & 1) != 0;
		ppa_srm_rotation_angle_t angle = PPA_SRM_ROTATION_ANGLE_0;
		if (rot == 1) angle = PPA_SRM_ROTATION_ANGLE_90;
		else if (rot == 2) angle = PPA_SRM_ROTATION_ANGLE_180;
		else if (rot == 3) angle = PPA_SRM_ROTATION_ANGLE_270;

		const int Fw = frame.width, Fh = frame.height;
		const int rotW = swap ? Fh : Fw;
		const int rotH = swap ? Fw : Fh;
		constexpr int kFrag = 16;  // PPA_LL_SRM_SCALING_FRAG_MAX
		// Digital zoom: scaling up past the cover factor shrinks the source block to
		// a centered crop, so the same output shows a zoomed-in region. zoom_==1 is
		// the plain cover fit. (As a bonus, zooming in reads fewer pixels -> faster.)
		const double zoom = zoom_ > 1.0 ? zoom_ : 1.0;
		const double sCover = std::max(static_cast<double>(dstW) / rotW, static_cast<double>(dstH) / rotH);
		int sNum = static_cast<int>(std::ceil(sCover * zoom * kFrag));  // scale = sNum/16, >= cover*zoom
		if (sNum < 1) sNum = 1;
		const float scale = static_cast<float>(sNum) / kFrag;
		const int needForW = (dstW * kFrag + sNum - 1) / sNum;  // ceil(dstW*16/sNum)
		const int needForH = (dstH * kFrag + sNum - 1) / sNum;
		const int blockW = std::min(Fw, swap ? needForH : needForW);
		const int blockH = std::min(Fh, swap ? needForW : needForH);
		if (blockW <= 0 || blockH <= 0) return false;
		const int offX = (Fw - blockW) / 2;
		const int offY = (Fh - blockH) / 2;

		ppa_srm_oper_config_t cfg = {};
		cfg.in.buffer = frame.data;
		cfg.in.pic_w = static_cast<std::uint32_t>(Fw);
		cfg.in.pic_h = static_cast<std::uint32_t>(Fh);
		cfg.in.block_w = static_cast<std::uint32_t>(blockW);
		cfg.in.block_h = static_cast<std::uint32_t>(blockH);
		cfg.in.block_offset_x = static_cast<std::uint32_t>(offX);
		cfg.in.block_offset_y = static_cast<std::uint32_t>(offY);
		cfg.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		cfg.out.buffer = outBuf;
		cfg.out.buffer_size = static_cast<std::uint32_t>(outBytes);
		cfg.out.pic_w = static_cast<std::uint32_t>(outPicW);
		cfg.out.pic_h = static_cast<std::uint32_t>(outPicH);
		cfg.out.block_offset_x = static_cast<std::uint32_t>(outOffX);
		cfg.out.block_offset_y = static_cast<std::uint32_t>(outOffY);
		cfg.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
		cfg.rotation_angle = angle;
		cfg.scale_x = scale;
		cfg.scale_y = scale;
		cfg.mode = PPA_TRANS_MODE_BLOCKING;
		const std::int64_t ppaT0 = esp_timer_get_time();
		const bool ppaOk = ppa_do_scale_rotate_mirror(ppaClient_, &cfg) == ESP_OK;
		const std::int64_t ppaT1 = esp_timer_get_time();
		static std::int64_t s_ppaAcc = 0;
		static int s_ppaN = 0;
		s_ppaAcc += ppaT1 - ppaT0;
		if (++s_ppaN >= 30) {
			ESP_LOGI(kTag, "TEST: ppa_do_scale_rotate_mirror avg=%lldus in=%dx%d block=%dx%d out=%dx%d dst=%dx%d scale=%.3f",
			         s_ppaAcc / s_ppaN, Fw, Fh, blockW, blockH, outPicW, outPicH, dstW, dstH, scale);
			s_ppaAcc = 0;
			s_ppaN = 0;
		}
		return ppaOk;
	}

	// Framebuffer preview mode: PPA into the camera surface (the renderer blits
	// it). Cover fit only; returns false so the caller uses the CPU scaler.
	bool fillScaledPpa(const CapturedFrame &frame, std::uint16_t *dst, int dstW, int dstH, int fit)
	{
		if (fit != 0) return false;  // cover only
		if ((reinterpret_cast<std::uintptr_t>(dst) & 127u) != 0) return false;
		const std::size_t outBytes = (static_cast<std::size_t>(dstW) * dstH * 2 + 127) & ~static_cast<std::size_t>(127);
		primePpaSurface(dst, outBytes);
		if (!ppaScaleRotateInto(frame, dst, outBytes, dstW, dstH, 0, 0, dstW, dstH)) return false;
		// PPA wrote the surface via DMA — drop stale CPU cache lines for the blit.
		esp_cache_msync(dst, outBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
		return true;
	}

	// NativeOverlay direct-present: PPA-scale the latest frame straight into the
	// display's logical framebuffer at (vfX,vfY) and flush only that rect — no
	// surface, no display-list blit, no full replay. The camera owns this rect, so
	// the renderer never draws or flushes it.
	bool presentDirect(int vfX, int vfY, int vfW, int vfH)
	{
		if (!open_ || vfW <= 0 || vfH <= 0) return false;
		// One-pass fast path: scale+rotate the latest frame STRAIGHT into the live panel
		// scanout buffer at the viewfinder's panel rect, composing the camera's own sensor
		// rotation with the display orientation's rotation. The DPI panel refreshes the
		// buffer continuously, so there is nothing else to do — no off-screen landscape
		// canvas, no second rotate-to-panel flush. That second PPA pass was ~half the
		// per-frame cost (scale 34ms + rotate 29ms); fusing it ~doubles the preview fps.
		// The camera owns this rect; the UI never composes into it.
		std::uint16_t *panelBuf = nullptr;
		int panelBufW = 0, panelBufH = 0, pX = 0, pY = 0, pW = 0, pH = 0, rotSteps = 0, flip = 0;
		if (gea::platform::display::Display::panelDirectTarget(vfX, vfY, vfW, vfH, &panelBuf, &panelBufW, &panelBufH,
		                                                       &pX, &pY, &pW, &pH, &rotSteps, &flip)) {
			const std::size_t panelBytes = (static_cast<std::size_t>(panelBufW) * panelBufH * 2 + 127) & ~static_cast<std::size_t>(127);
			bool ok = false;
			withFrame([&](CapturedFrame &frame) {
				// Full-panel (flip==1): scale into the non-scanned back buffer, then swap it
				// in at vblank — the DSI never sees a half-written frame, so no tearing.
				// Sub-rect (flip==0): scale into the live buffer in place (windowed).
				if (!ppaScaleRotateInto(frame, panelBuf, panelBytes, panelBufW, panelBufH, pX, pY, pW, pH, kPreviewRotateCcw + rotSteps)) return;
				if (flip) gea::platform::display::Display::flipPanelToBack();
				recordFrameIfDue(frame);
				ok = true;
			});
			return ok;
		}

		// Fallback (no panel scanout buffer available): compose into the off-screen logical
		// framebuffer + flush/rotate the rect.
		auto *canvas = gea::platform::display::Display::canvas();
		if (!canvas || !canvas->pixels()) return false;
		std::uint16_t *fb = const_cast<std::uint16_t *>(canvas->pixels());
		const int fbW = canvas->width();
		const int fbH = canvas->height();
		if (fbW <= 0 || fbH <= 0 || canvas->strideBytes() != fbW * 2) return false;  // tightly-packed only
		if (vfX < 0 || vfY < 0 || vfX + vfW > fbW || vfY + vfH > fbH) return false;
		const std::size_t fbBytes = (static_cast<std::size_t>(fbW) * fbH * 2 + 127) & ~static_cast<std::size_t>(127);
		bool ok = false;
		withFrame([&](CapturedFrame &frame) {
			if (!ppaScaleRotateInto(frame, fb, fbBytes, fbW, fbH, vfX, vfY, vfW, vfH)) return;
			std::uint16_t *rows = fb + static_cast<std::size_t>(vfY) * fbW;
			esp_cache_msync(rows, static_cast<std::size_t>(vfH) * fbW * 2, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
			const gea::platform::display::DisplayFlushRect rect{vfX, vfY, vfX + vfW - 1, vfY + vfH - 1};
			gea::platform::display::Display::flushRects(&rect, 1);
			recordFrameIfDue(frame);
			ok = true;
		});
		return ok;
	}

	// Encode the current frame to JPEG (HW) and append to the open recording
	// file, throttled to the requested fps. Driven from the preview path.
	void recordFrameIfDue(const CapturedFrame &frame)
	{
#if GEA_P4_HAS_JPEG_ENCODER
		if (!recording_ || !recFile_ || !recJpegEnc_ || !frame.data) return;
		if (frame.format != V4L2_PIX_FMT_RGB565) return;  // encoder configured for RGB565
		const std::int64_t now = esp_timer_get_time();
		const std::int64_t interval = static_cast<std::int64_t>(1000000.0 / (recFps_ > 0.0 ? recFps_ : 15.0));
		if (recLastFrameUs_ != 0 && now - recLastFrameUs_ < interval) return;
		recLastFrameUs_ = now;

		jpeg_encode_cfg_t cfg = {};
		cfg.height = static_cast<uint32_t>(frame.height);
		cfg.width = static_cast<uint32_t>(frame.width);
		cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
		cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
		cfg.image_quality = 80;
		uint32_t outLen = 0;
		const esp_err_t err = jpeg_encoder_process(recJpegEnc_, &cfg, frame.data,
		                                           static_cast<uint32_t>(frame.bytes), recJpegOut_,
		                                           static_cast<uint32_t>(recJpegOutCap_), &outLen);
		if (err == ESP_OK && outLen > 0) {
			std::fwrite(recJpegOut_, 1, outLen, recFile_);
			++recFrameCount_;
		}
#else
		(void)frame;
#endif
	}

	bool videoInitialized_ = false;
	bool open_ = false;
	bool streaming_ = false;
	bool mirror_ = true;
	// Serializes the render task's preview frames against the long capture ops
	// (still / clip), which reconfigure the device mid-stream. Created lazily.
	SemaphoreHandle_t opLock_ = nullptr;
	// microSD state — captures are written here (no PSRAM size cap, no serial-link
	// corruption) and pulled off afterwards via the reliable GEADEV GET protocol.
	bool sdMounted_ = false;
	sdmmc_card_t *sdCard_ = nullptr;
	sd_pwr_ctrl_handle_t sdPwr_ = nullptr;
	// Persistent H.264 clip buffer: the encode loop accumulates here (fast memcpy)
	// and the whole clip is bulk-written to SD afterwards. Allocated once and reused.
	std::uint8_t *clipBuf_ = nullptr;
	std::size_t clipCap_ = 0;
	double zoom_ = 1.0;
	int fd_ = -1;
	int width_ = kDefaultWidth;
	int height_ = kDefaultHeight;
	int bufferCount_ = 0;
	std::int64_t dbgDqUs_ = 0, dbgCbUs_ = 0;
	int dbgFrames_ = 0;
	std::uint32_t format_ = V4L2_PIX_FMT_RGB565;
	gea::platform::camera::Facing facing_ = gea::platform::camera::Facing::Front;
	std::array<MappedBuffer, kBufferCount> buffers_{};

	// PPA HW scale/rotate state (lazy-initialised on first preview frame). The PPA
	// scales straight into the camera surface; ppaLastDst_ tracks which surface
	// has been blacked so edge pixels the PPA never writes don't show noise.
	ppa_client_handle_t ppaClient_ = nullptr;
	std::size_t ppaOutAlign_ = 64;  // PPA output cache-line alignment (P4 PSRAM = 64 B)
	std::uint16_t *ppaLastDst_ = nullptr;
	std::size_t ppaLastBytes_ = 0;
	// NativeOverlay viewfinder rect (logical framebuffer px), set by the renderer.
	int vfX_ = 0, vfY_ = 0, vfW_ = 0, vfH_ = 0;

	// Motion-JPEG recording state.
	bool recording_ = false;
	std::FILE *recFile_ = nullptr;
	double recFps_ = 15.0;
	std::int64_t recStartUs_ = 0;
	std::int64_t recLastFrameUs_ = 0;
	std::uint8_t *recJpegOut_ = nullptr;
	std::size_t recJpegOutCap_ = 0;
	int recFrameCount_ = 0;
#if GEA_P4_HAS_JPEG_ENCODER
	jpeg_encoder_handle_t recJpegEnc_ = nullptr;
#endif
};

bool CameraDriver::startRecording(const std::string &path, double fps)
{
#if GEA_P4_HAS_JPEG_ENCODER
	if (recording_ || !open_) return false;
	recFile_ = std::fopen(path.c_str(), "wb");
	if (!recFile_) {
		ESP_LOGE(kTag, "recording: fopen %s failed: errno=%d", path.c_str(), errno);
		return false;
	}
	jpeg_encode_engine_cfg_t engineCfg = {};
	engineCfg.timeout_ms = 200;
	if (jpeg_new_encoder_engine(&engineCfg, &recJpegEnc_) != ESP_OK) {
		ESP_LOGE(kTag, "recording: jpeg_new_encoder_engine failed");
		std::fclose(recFile_);
		recFile_ = nullptr;
		return false;
	}
	// JPEG of an RGB565 frame is comfortably smaller than the raw frame; size
	// the output to the raw byte count as a safe upper bound.
	const std::size_t cap = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_) * 2u;
	jpeg_encode_memory_alloc_cfg_t memCfg = {};
	memCfg.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
	std::size_t allocated = 0;
	recJpegOut_ = static_cast<std::uint8_t *>(jpeg_alloc_encoder_mem(cap, &memCfg, &allocated));
	if (!recJpegOut_) {
		ESP_LOGE(kTag, "recording: jpeg output alloc (%zu) failed", cap);
		jpeg_del_encoder_engine(recJpegEnc_);
		recJpegEnc_ = nullptr;
		std::fclose(recFile_);
		recFile_ = nullptr;
		return false;
	}
	recJpegOutCap_ = allocated;
	recFps_ = fps > 0.0 ? fps : 15.0;
	recStartUs_ = esp_timer_get_time();
	recLastFrameUs_ = 0;
	recFrameCount_ = 0;
	recording_ = true;
	ESP_LOGI(kTag, "recording started -> %s @ %.0ffps", path.c_str(), recFps_);
	return true;
#else
	(void)path;
	(void)fps;
	ESP_LOGW(kTag, "recording unavailable: esp_driver_jpeg component not present");
	return false;
#endif
}

double CameraDriver::stopRecording()
{
#if GEA_P4_HAS_JPEG_ENCODER
	if (!recording_) return -1.0;
	recording_ = false;
	const double durationMs = (esp_timer_get_time() - recStartUs_) / 1000.0;
	if (recFile_) {
		std::fclose(recFile_);
		recFile_ = nullptr;
	}
	if (recJpegOut_) {
		heap_caps_free(recJpegOut_);
		recJpegOut_ = nullptr;
	}
	if (recJpegEnc_) {
		jpeg_del_encoder_engine(recJpegEnc_);
		recJpegEnc_ = nullptr;
	}
	ESP_LOGI(kTag, "recording stopped: %d frames, %.0f ms", recFrameCount_, durationMs);
	return durationMs;
#else
	return -1.0;
#endif
}

}  // namespace gea::platform::esp32_p4_waveshare::camera

namespace gea::platform::camera {

bool Camera::isAvailable()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().isAvailable();
}

bool Camera::hasPermission()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().hasPermission();
}

bool Camera::requestPermission()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().requestPermission();
}

bool Camera::open(const std::string &facingHint, int preferredWidth, int preferredHeight)
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().open(facingHint, preferredWidth, preferredHeight);
}

void Camera::close()
{
	esp32_p4_waveshare::camera::CameraDriver::instance().close();
}

bool Camera::isOpen()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().isOpen();
}

int Camera::width()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().width();
}

int Camera::height()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().height();
}

int Camera::orientation()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().orientation();
}

Facing Camera::currentFacing()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().currentFacing();
}

std::string Camera::currentFacingString()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().currentFacingString();
}

int Camera::deviceCount()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().deviceCount();
}

DeviceInfo Camera::deviceAt(int index)
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().deviceAt(index);
}

void Camera::drawPreview(int x, int y, int destW, int destH)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().drawPreview(x, y, destW, destH);
}

int Camera::capture(bool mirror)
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().capture(mirror);
}

void Camera::setFlash(const std::string &mode)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setFlash(mode);
}

void Camera::setZoom(double factor)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setZoom(factor);
}

void Camera::setMirror(bool mirror)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setMirror(mirror);
}

int Camera::previewMode()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().previewMode();
}

bool Camera::fillPreview(std::uint16_t *dst, int dstWidth, int dstHeight, int fit, bool mirror)
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().fillPreview(dst, dstWidth, dstHeight, fit, mirror);
}

void Camera::positionPreviewLayer(int x, int y, int width, int height)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().positionPreviewLayer(x, y, width, height);
}

void Camera::hidePreviewLayer()
{
	esp32_p4_waveshare::camera::CameraDriver::instance().hidePreviewLayer();
}

void Camera::presentNativeOverlay()
{
	esp32_p4_waveshare::camera::CameraDriver::instance().presentNativeOverlay();
}

bool Camera::startRecording(const std::string &path, double fps)
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().startRecording(path, fps);
}

double Camera::stopRecording()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().stopRecording();
}

bool Camera::isRecording()
{
	return esp32_p4_waveshare::camera::CameraDriver::instance().isRecording();
}

void Camera::setExposure(const std::string &mode, double bias, double iso, double durationMs)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setExposure(mode, bias, iso, durationMs);
}

void Camera::setWhiteBalance(const std::string &mode, double temperatureK, double tint)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setWhiteBalance(mode, temperatureK, tint);
}

void Camera::setFocus(const std::string &mode, double pointX, double pointY)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setFocus(mode, pointX, pointY);
}

void Camera::setTorch(const std::string &mode, double level)
{
	esp32_p4_waveshare::camera::CameraDriver::instance().setTorch(mode, level);
}

}  // namespace gea::platform::camera

// ---------------------------------------------------------------------------
// GEADEV dev-command extension (ESP32-P4 only). Streams a captured still / clip
// out over the serial console as base64, framed like `GEADEV SCREENSHOT`. This
// is the strong override of the weak `geaHandleExtraDevCommand` hook declared in
// targets/esp32/services/device_control.cpp; targets without a camera keep the
// weak no-op, so this adds no commands there.
//
//   GEADEV CAMSTILL        -> GEADEV:CAMSTILL BEGIN ... / GEADEV:DATA <b64> / END  (full-res JPEG)
//   GEADEV CAMCLIP [secs]  -> GEADEV:CAMCLIP  BEGIN ... / GEADEV:DATA <b64> / END  (1080p H.264, default 10s)
// ---------------------------------------------------------------------------
namespace {

// zlib-compatible CRC-32 (poly 0xEDB88820, init/final ~0). Lets the host verify each
// pulled chunk with Python's zlib.crc32 and re-request only the bad ones.
std::uint32_t crc32z(const std::uint8_t *p, std::size_t n)
{
	std::uint32_t c = 0xFFFFFFFFu;
	for (std::size_t i = 0; i < n; ++i) {
		c ^= p[i];
		for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88820u & (0u - (c & 1u)));
	}
	return c ^ 0xFFFFFFFFu;
}

// Emit `len` bytes as base64 in 76-char `GEADEV:DATA` lines. Caller owns the framing
// (BEGIN/END), the stdout lock and the log suppression.
void emitBase64(const std::uint8_t *data, std::size_t len)
{
	static const char kAlphabet[] =
	    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	char line[80];
	int col = 0;
	auto emit = [&](char c) {
		line[col++] = c;
		if (col >= 76) {
			std::printf("GEADEV:DATA ");
			std::fwrite(line, 1, static_cast<std::size_t>(col), stdout);
			std::printf("\n");
			col = 0;
		}
	};
	std::size_t i = 0;
	for (; i + 3 <= len; i += 3) {
		const std::uint32_t v = (static_cast<std::uint32_t>(data[i]) << 16) |
		                        (static_cast<std::uint32_t>(data[i + 1]) << 8) |
		                        static_cast<std::uint32_t>(data[i + 2]);
		emit(kAlphabet[(v >> 18) & 0x3f]);
		emit(kAlphabet[(v >> 12) & 0x3f]);
		emit(kAlphabet[(v >> 6) & 0x3f]);
		emit(kAlphabet[v & 0x3f]);
	}
	const std::size_t rem = len - i;
	if (rem == 1) {
		const std::uint32_t v = static_cast<std::uint32_t>(data[i]) << 16;
		emit(kAlphabet[(v >> 18) & 0x3f]);
		emit(kAlphabet[(v >> 12) & 0x3f]);
		emit('=');
		emit('=');
	} else if (rem == 2) {
		const std::uint32_t v = (static_cast<std::uint32_t>(data[i]) << 16) |
		                        (static_cast<std::uint32_t>(data[i + 1]) << 8);
		emit(kAlphabet[(v >> 18) & 0x3f]);
		emit(kAlphabet[(v >> 12) & 0x3f]);
		emit(kAlphabet[(v >> 6) & 0x3f]);
		emit('=');
	}
	if (col > 0) {
		std::printf("GEADEV:DATA ");
		std::fwrite(line, 1, static_cast<std::size_t>(col), stdout);
		std::printf("\n");
	}
}

// Stream a byte range [offset, offset+reqLen) of an SD file as a CRC-tagged base64
// block: `GEADEV:GET BEGIN ... crc=XX` / `GEADEV:DATA ...` / `GEADEV:GET END`. The
// host verifies the CRC and re-requests the chunk on mismatch -> bit-perfect pull.
// ESP_LOG is silenced for the block so the render task's log lines can't corrupt it.
void streamFileRange(const char *path, long offset, std::size_t reqLen)
{
	std::FILE *f = std::fopen(path, "rb");
	if (!f) {
		std::printf("GEADEV:ERR GET no-file path=%s\n", path);
		std::fflush(stdout);
		return;
	}
	if (reqLen == 0 || reqLen > 1u * 1024u * 1024u) reqLen = 256u * 1024u;
	auto *buf = static_cast<std::uint8_t *>(heap_caps_malloc(reqLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!buf) {
		std::fclose(f);
		std::printf("GEADEV:ERR GET no-mem bytes=%u\n", static_cast<unsigned>(reqLen));
		std::fflush(stdout);
		return;
	}
	std::fseek(f, offset, SEEK_SET);
	const std::size_t got = std::fread(buf, 1, reqLen, f);
	std::fclose(f);
	const std::uint32_t crc = crc32z(buf, got);

	esp_log_level_set("*", ESP_LOG_NONE);
	flockfile(stdout);
	std::printf("GEADEV:GET BEGIN path=%s offset=%ld len=%u crc=%08x\n",
	            path, offset, static_cast<unsigned>(got), static_cast<unsigned>(crc));
	emitBase64(buf, got);
	std::printf("GEADEV:GET END\n");
	std::fflush(stdout);
	funlockfile(stdout);
	esp_log_level_set("*", ESP_LOG_INFO);  // restore the dev-default log level
	heap_caps_free(buf);
}

}  // namespace

extern "C" bool geaHandleExtraDevCommand(const char *command, char *args)
{
	using gea::platform::esp32_p4_waveshare::camera::CameraDriver;
	if (!command) return false;

	// Capture commands write to the SD card and return only metadata; the host then
	// pulls the file with GET (chunked + CRC-verified). This keeps the clip clean and
	// uncapped (no PSRAM buffer) and the transfer bit-perfect.
	if (std::strcmp(command, "CAMSTILL") == 0) {
		std::size_t bytes = 0;
		int w = 0, h = 0;
		if (!CameraDriver::instance().captureStillToSd("/sdcard/frame.jpg", &bytes, &w, &h)) {
			std::printf("GEADEV:ERR CAMSTILL capture-failed\n");
		} else {
			std::printf("GEADEV:CAMSTILL OK path=/sdcard/frame.jpg bytes=%u width=%d height=%d\n",
			            static_cast<unsigned>(bytes), w, h);
		}
		std::fflush(stdout);
		return true;
	}

	if (std::strcmp(command, "CAMCLIP") == 0) {
		double seconds = 10.0;
		if (args && *args) {
			const double v = std::atof(args);
			if (v >= 0.5 && v <= 30.0) seconds = v;
		}
		std::size_t bytes = 0;
		int w = 0, h = 0, frames = 0;
		double fps = 0.0;
		if (!CameraDriver::instance().recordClipToSd(seconds, "/sdcard/clip.h264", &bytes, &w, &h, &fps, &frames)) {
			std::printf("GEADEV:ERR CAMCLIP record-failed\n");
		} else {
			std::printf("GEADEV:CAMCLIP OK path=/sdcard/clip.h264 bytes=%u frames=%d fps=%.2f width=%d height=%d\n",
			            static_cast<unsigned>(bytes), frames, fps, w, h);
		}
		std::fflush(stdout);
		return true;
	}

	if (std::strcmp(command, "STAT") == 0) {
		char p[160] = {0};
		if (!args || std::sscanf(args, "%159s", p) != 1) {
			std::printf("GEADEV:ERR STAT usage=GEADEV_STAT_path\n");
		} else {
			std::FILE *f = std::fopen(p, "rb");
			long size = -1;
			if (f) {
				std::fseek(f, 0, SEEK_END);
				size = std::ftell(f);
				std::fclose(f);
			}
			std::printf("GEADEV:STAT path=%s exists=%d size=%ld\n", p, size >= 0 ? 1 : 0, size);
		}
		std::fflush(stdout);
		return true;
	}

	if (std::strcmp(command, "GET") == 0) {
		char p[160] = {0};
		long offset = 0;
		long len = 0;
		if (!args || std::sscanf(args, "%159s %ld %ld", p, &offset, &len) < 1 || !p[0]) {
			std::printf("GEADEV:ERR GET usage=GEADEV_GET_path_offset_len\n");
			std::fflush(stdout);
			return true;
		}
		streamFileRange(p, offset, static_cast<std::size_t>(len > 0 ? len : 0));
		return true;
	}

	return false;
}
