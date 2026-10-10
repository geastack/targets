#include "touch.h"

#include "board.h"
#include "display.h"
#include "i2c.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_log.h"

namespace {

using gea::platform::touch::Phase;
using gea::platform::touch::Touchscreen;
constexpr char kTag[] = "cst820";
i2c_master_dev_handle_t device = nullptr;
Touchscreen::Observer observer = nullptr;
Touchscreen::PointerObserver pointerObserver = nullptr;
bool pressed = false;
int cachedX = 0;
int cachedY = 0;
int latestX = 0;
int latestY = 0;
bool moveQueued = false;
std::uint32_t lastPoll = 0;
std::uint32_t lastReport = 0;
std::atomic<bool> reportPending{false};
std::atomic<bool> injectedGestureActive{false};
std::mutex streamMutex;
std::mutex cacheMutex;

void IRAM_ATTR touchInterrupt(void*) {
  reportPending.store(true, std::memory_order_relaxed);
}

void notify(Phase phase, bool touching, int x, int y) {
  Touchscreen::Observer currentObserver;
  Touchscreen::PointerObserver currentPointerObserver;
  {
    const std::lock_guard<std::mutex> guard(cacheMutex);
    currentObserver = observer;
    currentPointerObserver = pointerObserver;
  }
  if (currentPointerObserver) {
    currentPointerObserver(phase, touching, x, y, 0);
  } else if (currentObserver) {
    currentObserver(phase, touching, x, y);
  }
}

void update(Phase phase, bool touching, int x, int y) {
  {
    const std::lock_guard<std::mutex> guard(cacheMutex);
    pressed = touching;
    cachedX = x;
    cachedY = y;
    latestX = x;
    latestY = y;
    if (phase == Phase::Move) {
      if (moveQueued) {
        return;
      }
      moveQueued = true;
    }
  }
  // The observer may consume a dropped/coalesced Move immediately.
  notify(phase, touching, x, y);
}

} // namespace

namespace gea::platform::touch {

void Touchscreen::setObserver(Observer next) {
  const std::lock_guard<std::mutex> guard(cacheMutex);
  observer = next;
}

void Touchscreen::setPointerObserver(PointerObserver next) {
  const std::lock_guard<std::mutex> guard(cacheMutex);
  pointerObserver = next;
}

bool Touchscreen::init() {
  board::startButtons();
  if (device) {
    return true;
  }
  board::resetTouchPanel();
  const auto bus = i2c::Bus::primary();
  if (!bus.available()) {
    return false;
  }
  i2c_device_config_t config = {};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = 0x15;
  config.scl_speed_hz = 100000;
  const auto handle = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
  esp_err_t err = i2c_master_probe(handle, config.device_address, 100);
  if (err == ESP_OK) {
    err = i2c_master_bus_add_device(handle, &config, &device);
  }
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "CST820 not found: %s", esp_err_to_name(err));
    return false;
  }
  gpio_config_t interrupt = {};
  interrupt.pin_bit_mask = 1ULL << board::touch.interrupt;
  interrupt.mode = GPIO_MODE_INPUT;
  interrupt.pull_up_en = GPIO_PULLUP_ENABLE;
  interrupt.intr_type = GPIO_INTR_NEGEDGE;
  ESP_ERROR_CHECK(gpio_config(&interrupt));
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(err);
  }
  ESP_ERROR_CHECK(gpio_isr_handler_add(board::touch.interrupt, touchInterrupt, nullptr));
  reportPending.store(true, std::memory_order_relaxed);
  ESP_LOGI(kTag, "touch ready (address=0x15, INT=GPIO%d)", board::touch.interrupt);
  return true;
}

void Touchscreen::poll(int nowMs) {
  if (injectedGestureActive.load(std::memory_order_acquire) || !device) {
    return;
  }
  // Serialize a poll already in progress with the control task's injected
  // Down. During its gesture, leave the hardware IRQ latch untouched.
  const std::lock_guard<std::mutex> guard(streamMutex);
  if (injectedGestureActive.load(std::memory_order_acquire)) {
    return;
  }
  const auto now = static_cast<std::uint32_t>(nowMs);
  if (now - lastPoll < 10) {
    return;
  }
  // Avoid waking the controller or blocking animation when it has no report.
  // While held, poll too so a lost release interrupt cannot stick the pointer.
  const bool pending = reportPending.exchange(false, std::memory_order_relaxed);
  if (!pressed && !pending && gpio_get_level(board::touch.interrupt) != 0) {
    return;
  }
  lastPoll = now;
  const std::uint8_t reg = 0x00;
  std::uint8_t report[7] = {};
  if (i2c_master_transmit_receive(device, &reg, 1, report, sizeof(report), 10) != ESP_OK) {
    // The controller sleeps after idle. A missed final report followed by
    // NACKs must not leave a finger permanently held in the runtime.
    if (pressed && now - lastReport >= 60) {
      update(Phase::Up, false, cachedX, cachedY);
    }
    return;
  }
  lastReport = now;
  // The CST820 report is not the CST92xx protocol. Decode the factory
  // firmware's seven-byte report, including its explicit lift event.
  const unsigned event = report[3] >> 6;
  const bool touching = (report[2] & 0x0F) != 0 && (event == 0 || event == 2);
  if (!touching) {
    if (pressed) {
      update(Phase::Up, false, cachedX, cachedY);
    }
    return;
  }
  const int x = std::clamp(((report[3] & 0x0F) << 8) | report[4], 0, display::kWidth - 1);
  const int y = std::clamp(((report[5] & 0x0F) << 8) | report[6], 0, display::kHeight - 1);
  if (!pressed) {
    update(Phase::Down, true, x, y);
  } else if (x != cachedX || y != cachedY) {
    update(Phase::Move, true, x, y);
  }
}

int Touchscreen::read(int* x, int* y) {
  return readCached(x, y);
}

int Touchscreen::readCached(int* x, int* y) {
  const std::lock_guard<std::mutex> guard(cacheMutex);
  if (x) {
    *x = cachedX;
  }
  if (y) {
    *y = cachedY;
  }
  return pressed ? 1 : 0;
}

void Touchscreen::consumeLatestMove(int* x, int* y) {
  const std::lock_guard<std::mutex> guard(cacheMutex);
  moveQueued = false;
  if (x) {
    *x = latestX;
  }
  if (y) {
    *y = latestY;
  }
}

void Touchscreen::injectEvent(Phase phase, bool touching, int x, int y) {
  const std::lock_guard<std::mutex> guard(streamMutex);
  if (phase == Phase::Down) {
    injectedGestureActive.store(true, std::memory_order_release);
  }
  update(phase, touching, x, y);
  if (phase == Phase::Up) {
    // Publish the Up before letting hardware reports resume the stream.
    injectedGestureActive.store(false, std::memory_order_release);
  }
}

} // namespace gea::platform::touch
