// SPDX-License-Identifier: Apache-2.0
#include "camera.h"
#include "display.h"
#include "image.h"
#include "host/backends.h"
#include "host/camera_frame.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

#if GEA_MOSAICO_CAMERA_ENABLED
#include "driver/jpeg_encode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mosaico_module_camera.h"

namespace gea::platform::camera {
namespace {
namespace pixel = gea::framework::graphics::pixel;
namespace frame = gea::host::camera_frame;
constexpr char kTag[] = "mosaico_camera";
constexpr std::int64_t kFrameIntervalUs = 2'000'000;

class CameraDriver final : public CameraFrameProvider {
public:
  mosaico_camera_handle_t camera = nullptr;
  jpeg_encoder_handle_t encoder = nullptr;
  std::uint8_t *rgb = nullptr;
  std::uint8_t *jpeg = nullptr;
  std::size_t rgbCapacity = 0, jpegCapacity = 0;
  frame::Size size;
  std::atomic<bool> running{false};
  std::atomic<bool> mirror{false};
  SemaphoreHandle_t exited = nullptr;
  std::mutex operations;
  std::mutex lock;
  std::string latest;
  std::vector<pixel::native_t> pixels;

  std::string takeFrameDataUrl() override {
    std::unique_lock guard(lock, std::try_to_lock);
    if (!guard.owns_lock()) return {};
    std::string result;
    result.swap(latest);
    return result;
  }

  static void captureTask(void *opaque) {
    auto &self = *static_cast<CameraDriver *>(opaque);
    std::int64_t nextEncode = 0;
    while (self.running.load()) {
      mosaico_camera_frame_t source{};
      const auto got = mosaico_camera_get_frame(self.camera, &source);
      if (got != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      const auto now = esp_timer_get_time();
      bool converted = false;
      if (now >= nextEncode && source.pixel_format == V4L2_PIX_FMT_UYVY) {
        nextEncode = now + kFrameIntervalUs;
        const std::size_t stride = source.bytes_per_line ? source.bytes_per_line : source.width * 2;
        converted = frame::uyvyToRgb888Ccw90(
            static_cast<const std::uint8_t *>(source.data), source.size,
            source.width, source.height, stride, self.rgb, self.rgbCapacity,
            self.size.width, self.size.height, self.mirror.load());
      }
      // Return every sensor buffer before codec work; draining continues even
      // while the app is disconnected or its UI never asks for another frame.
      const auto returned = mosaico_camera_return_frame(self.camera, &source);
      if (returned != ESP_OK) {
        ESP_LOGE(kTag, "Return camera buffer failed: %s", esp_err_to_name(returned));
        self.running.store(false);
        break;
      }
      if (!converted || !self.running.load()) continue;
      std::vector<pixel::native_t> preview(std::size_t(self.size.width) * self.size.height);
      for (std::size_t i = 0; i < preview.size(); ++i) {
        auto *color = self.rgb + i * 3;
        preview[i] = pixel::nativeColor(color[0], color[1], color[2]);
        // ESP-IDF's RGB888 encoder input is explicitly BGR24 (jpeg_types.h).
        std::swap(color[0], color[2]);
      }
      jpeg_encode_cfg_t config{};
      config.width = self.size.width;
      config.height = self.size.height;
      config.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
      config.sub_sample = JPEG_DOWN_SAMPLING_YUV420;
      config.image_quality = 70;
      std::uint32_t jpegSize = 0;
      const auto encoded = jpeg_encoder_process(self.encoder, &config, self.rgb,
          self.size.width * self.size.height * 3, self.jpeg, self.jpegCapacity, &jpegSize);
      if (encoded != ESP_OK) {
        ESP_LOGW(kTag, "JPEG encode failed: %s", esp_err_to_name(encoded));
        continue;
      }
      auto url = frame::jpegDataUrl(self.jpeg, jpegSize);
      if (url.empty()) continue;
      std::lock_guard guard(self.lock);
      self.latest.swap(url);
      self.pixels.swap(preview);
    }
    xSemaphoreGive(self.exited);
    vTaskDelete(nullptr);
  }

  void closeResources() {
    Camera::setFrameProvider(nullptr);
    running.store(false);
    if (exited) {
      // Sensor reads have a 100ms deadline and JPEG has a 500ms deadline.
      // If a driver nevertheless stalls, preserve its live buffers/handles
      // and let a later close/open retry cleanup instead of hanging the app.
      if (xSemaphoreTake(exited, pdMS_TO_TICKS(1500)) != pdTRUE) {
        ESP_LOGE(kTag, "Camera worker did not stop within 1500ms; resources retained");
        return;
      }
      vSemaphoreDelete(exited);
      exited = nullptr;
    }
    if (camera) {
      const auto result = mosaico_camera_del(camera);
      if (result == ESP_OK) {
        camera = nullptr;
      } else {
        // The SDK retains its context when teardown fails; never erase the
        // sole handle to that still-owned device or its outstanding buffers.
        ESP_LOGE(kTag, "Camera cleanup failed: %s", esp_err_to_name(result));
      }
    }
    if (!camera) mosaico_module_mgr_deinit();
    if (encoder) { jpeg_del_encoder_engine(encoder); encoder = nullptr; }
    std::free(rgb); rgb = nullptr; rgbCapacity = 0;
    std::free(jpeg); jpeg = nullptr; jpegCapacity = 0;
    std::lock_guard guard(lock);
    std::string().swap(latest);
    std::vector<pixel::native_t>().swap(pixels);
    size = {};
  }

  void close() {
    std::lock_guard operation(operations);
    closeResources();
  }

  bool open() {
    std::lock_guard operation(operations);
    if (running.load()) return true;
    closeResources();
    if (camera) return false;
    mosaico_camera_config_t config = MOSAICO_CAMERA_DEFAULT_CONFIG();
    // The two sensors have different supported sizes. Let the SDK detect
    // either sensor's UYVY default, then downsample for upload on this worker.
    config.frame_timeout_ms = 100;
    auto result = mosaico_camera_new(&config, &camera);
    if (result == ESP_OK) result = mosaico_camera_open(camera);
    mosaico_camera_info_t info{};
    if (result == ESP_OK) result = mosaico_camera_get_info(camera, &info);
    if (result == ESP_OK) {
      std::lock_guard guard(lock);
      size = frame::uploadSize(info.width, info.height);
      if (info.pixel_format != V4L2_PIX_FMT_UYVY || size.width == 0) result = ESP_ERR_NOT_SUPPORTED;
    }
    if (result == ESP_OK) {
      jpeg_encode_memory_alloc_cfg_t memory{};
      memory.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER;
      rgb = static_cast<std::uint8_t *>(jpeg_alloc_encoder_mem(size.width * size.height * 3, &memory, &rgbCapacity));
      memory.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
      jpeg = static_cast<std::uint8_t *>(jpeg_alloc_encoder_mem(96 * 1024, &memory, &jpegCapacity));
      if (!rgb || !jpeg) result = ESP_ERR_NO_MEM;
    }
    if (result == ESP_OK) {
      jpeg_encode_engine_cfg_t engine{};
      engine.timeout_ms = 500;
      result = jpeg_new_encoder_engine(&engine, &encoder);
    }
    if (result == ESP_OK) result = mosaico_camera_start_stream(camera);
    if (result != ESP_OK) {
      ESP_LOGE(kTag, "Left-slot camera open failed: %s", esp_err_to_name(result));
      closeResources();
      return false;
    }
    exited = xSemaphoreCreateBinary();
    if (!exited) { closeResources(); return false; }
    running.store(true);
    TaskHandle_t worker = nullptr;
    if (xTaskCreatePinnedToCore(captureTask, "gea_camera", 6144, this, 2, &worker, 1) != pdPASS) {
      // No worker exists to signal the completion semaphore.
      vSemaphoreDelete(exited); exited = nullptr;
      closeResources();
      return false;
    }
    Camera::setFrameProvider(this);
    gea::framework::camera::registerCameraSurface();
    ESP_LOGI(kTag, "DVP camera streaming: %ux%u -> upright %dx%d JPEG, one frame per 2s",
             unsigned(info.width), unsigned(info.height), size.width, size.height);
    return true;
  }
};

CameraDriver &driver() { static CameraDriver state; return state; }
} // namespace

bool Camera::isAvailable() { return true; }
bool Camera::hasPermission() { return true; }
bool Camera::requestPermission() { return true; }
bool Camera::open(const std::string &hint, int, int) {
  if (hint != "back" && hint != "front" && hint != "external" && hint != "mosaico-left-dvp") return false;
  return driver().open();
}
void Camera::close() { driver().close(); }
bool Camera::isOpen() { return driver().running.load(); }
int Camera::width() { auto &state = driver(); std::lock_guard guard(state.lock); return state.size.width; }
int Camera::height() { auto &state = driver(); std::lock_guard guard(state.lock); return state.size.height; }
int Camera::orientation() { return 0; }
Facing Camera::currentFacing() { return Facing::External; }
std::string Camera::currentFacingString() { return "external"; }
int Camera::deviceCount() { return 1; }
DeviceInfo Camera::deviceAt(int index) {
  return index == 0 ? DeviceInfo{"mosaico-left-dvp", Facing::External, width(), height()} : DeviceInfo{};
}

bool Camera::fillPreview(pixel::native_t *dst, int w, int h, int fit, bool mirror) {
  auto &state = driver();
  if (!dst || w <= 0 || h <= 0) return false;
  std::unique_lock guard(state.lock, std::try_to_lock);
  if (!guard.owns_lock() || state.pixels.empty()) return false;
  const int sw = state.size.width, sh = state.size.height;
  const double scale = fit == 1 ? std::min(double(w) / sw, double(h) / sh) : std::max(double(w) / sw, double(h) / sh);
  const double rw = fit == 2 ? w : sw * scale, rh = fit == 2 ? h : sh * scale;
  for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) {
    const double sx = (x - (w - rw) / 2) * sw / rw, sy = (y - (h - rh) / 2) * sh / rh;
    dst[y * w + x] = sx < 0 || sy < 0 || sx >= sw || sy >= sh ? pixel::native_t{} :
        state.pixels[int(sy) * sw + (mirror ? sw - 1 - int(sx) : int(sx))];
  }
  return true;
}
void Camera::drawPreview(int x, int y, int w, int h) {
  if (w <= 0) w = width();
  if (h <= 0) h = height();
  if (w <= 0 || h <= 0) return;
  std::vector<pixel::native_t> buffer(std::size_t(w) * h);
  if (!fillPreview(buffer.data(), w, h, 2, false)) return;
  struct Raster { const pixel::native_t *pixels; int width; int x; int y; } raster{buffer.data(), w, x, y};
  gea::platform::display::Display::streamRect(x, y, w, h,
      [](pixel::native_t *dst, int columns, int rows, int column, int row, void *context) {
        auto &image = *static_cast<Raster *>(context);
        for (int i = 0; i < rows; ++i) std::memcpy(dst + i * columns,
            image.pixels + (row - image.y + i) * image.width + column - image.x,
            columns * sizeof(pixel::native_t));
      }, &raster);
}
int Camera::previewMode() { return 0; }
void Camera::positionPreviewLayer(int, int, int, int) {}
void Camera::hidePreviewLayer() {}
void Camera::presentNativeOverlay() {}
int Camera::capture(bool mirror) {
  const int w = width(), h = height();
  if (w <= 0 || h <= 0) return -1;
  auto *pixels = static_cast<pixel::native_t *>(std::malloc(std::size_t(w) * h * sizeof(pixel::native_t)));
  if (!pixels) return -1;
  if (!fillPreview(pixels, w, h, 2, mirror)) { std::free(pixels); return -1; }
  const int image = gea::framework::graphics::ImageStore::instance().registerBuffer(pixels, w, h, -1, true);
  if (image < 0) std::free(pixels);
  return image;
}
bool Camera::startRecording(const std::string &, double) { return false; }
double Camera::stopRecording() { return -1; }
bool Camera::isRecording() { return false; }
void Camera::setFlash(const std::string &mode) {
  auto &state = driver();
  std::lock_guard operation(state.operations);
  if (!state.camera) return;
  if (mode == "on" || mode == "torch") mosaico_camera_flash_trigger(state.camera);
  else mosaico_camera_flash_stop(state.camera);
}
void Camera::setMirror(bool mirror) { driver().mirror.store(mirror); }
void Camera::setZoom(double) {}
void Camera::setExposure(const std::string &, double, double, double) {}
void Camera::setWhiteBalance(const std::string &, double, double) {}
void Camera::setFocus(const std::string &, double, double) {}
void Camera::setTorch(const std::string &mode, double) { setFlash(mode); }
} // namespace gea::platform::camera

#else
// Apps without the camera binding do not download or link DVP/BSP components.
namespace gea::platform::camera {
bool Camera::isAvailable() { return false; }
bool Camera::hasPermission() { return false; }
bool Camera::requestPermission() { return false; }
bool Camera::open(const std::string &, int, int) { return false; }
void Camera::close() {}
bool Camera::isOpen() { return false; }
int Camera::width() { return 0; }
int Camera::height() { return 0; }
int Camera::orientation() { return 0; }
Facing Camera::currentFacing() { return Facing::External; }
std::string Camera::currentFacingString() { return "external"; }
int Camera::deviceCount() { return 0; }
DeviceInfo Camera::deviceAt(int) { return {}; }
void Camera::drawPreview(int, int, int, int) {}
int Camera::previewMode() { return 0; }
bool Camera::fillPreview(gea::framework::graphics::pixel::native_t *, int, int, int, bool) { return false; }
void Camera::positionPreviewLayer(int, int, int, int) {}
void Camera::hidePreviewLayer() {}
void Camera::presentNativeOverlay() {}
int Camera::capture(bool) { return -1; }
bool Camera::startRecording(const std::string &, double) { return false; }
double Camera::stopRecording() { return -1; }
bool Camera::isRecording() { return false; }
void Camera::setFlash(const std::string &) {}
void Camera::setZoom(double) {}
void Camera::setMirror(bool) {}
void Camera::setExposure(const std::string &, double, double, double) {}
void Camera::setWhiteBalance(const std::string &, double, double) {}
void Camera::setFocus(const std::string &, double, double) {}
void Camera::setTorch(const std::string &, double) {}
} // namespace gea::platform::camera
#endif
