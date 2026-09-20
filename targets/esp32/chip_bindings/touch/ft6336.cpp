#include "ft6336_controller.h"
#include "board.h"
#include "i2c.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "ft6336";
constexpr gpio_num_t kResetPin = gea::platform::board::touch.reset;
constexpr gpio_num_t kInterruptPin = gea::platform::board::touch.interrupt;
constexpr int kPollIntervalMs = 10;
// BELOW the render. The renderer rasters in parallel across both cores — the
// main task on CPU0 (priority 5) plus a worker pinned to CPU1 at the SAME
// priority 5 that the main task spin-waits on (display.cpp gea_render_parallel
// _submit/_wait). This poll task shares CPU1 with that worker. At its old
// priority 10 every I2C read (~140/sec, ~550us each, while a finger is down)
// PREEMPTED the worker mid-band, stalling it and forcing the main task to spin
// on CPU0 — ~6ms/frame, dropping a finger-down drag from ~60 to ~42fps (injected
// drags don't run this task, so they rastered uncontended at ~86fps). At
// priority 4 it can't preempt the raster; it samples in the render's idle
// windows instead, still at the panel's full report rate.
constexpr int kTaskPriority = 4;
constexpr TickType_t kI2cTimeoutTicks = pdMS_TO_TICKS(100);

}  // namespace

namespace gea::platform::esp32::chip_bindings::ft6336 {

bool EspI2cRegisterBus::attach(i2c_master_bus_handle_t bus) {
  if (device_) return true;

  i2c_device_config_t config = {};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = gea::chips::ft6336::kI2cAddress;
  config.scl_speed_hz = gea::chips::ft6336::kI2cFrequencyHz;
  return i2c_master_bus_add_device(bus, &config, &device_) == ESP_OK;
}

bool EspI2cRegisterBus::writeRegister(std::uint8_t reg, std::uint8_t value) {
  std::uint8_t data[2] = {reg, value};
  return device_ && i2c_master_transmit(device_, data, sizeof(data), kI2cTimeoutTicks) == ESP_OK;
}

bool EspI2cRegisterBus::readRegister(std::uint8_t reg, std::uint8_t &value) {
  return readRegisters(reg, &value, 1);
}

bool EspI2cRegisterBus::readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) {
  return device_ && i2c_master_transmit_receive(device_, &reg, 1, data, length, kI2cTimeoutTicks) == ESP_OK;
}

TouchController &TouchController::instance() {
  static TouchController controller;
  return controller;
}

void TouchController::setObserver(Observer observer) {
  observer_ = observer;
}

void TouchController::setPointerObserver(PointerObserver observer) {
  pointerObserver_ = observer;
}

esp_err_t TouchController::begin() {
  if (initialized_) return ESP_OK;

  ESP_LOGI(kTag, "Initializing FT6336 touch (RST=%d, INT=%d)", static_cast<int>(kResetPin), static_cast<int>(kInterruptPin));

  auto bus = gea::platform::i2c::Bus::primary();
  if (!bus.available()) return ESP_ERR_INVALID_STATE;
  bus_ = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());

  esp_err_t err = resetChip();
  if (err != ESP_OK) return err;

  err = attachI2cDevice();
  if (err != ESP_OK) return err;

  err = configureChip();
  if (err != ESP_OK) return err;

  err = startTask();
  if (err != ESP_OK) return err;

  err = attachInterrupt();
  if (err != ESP_OK) return err;

  initialized_ = true;
  ESP_LOGI(kTag, "Touch controller ready (%s)", kInterruptPin >= 0 ? "interrupt-driven" : "polled");
  return ESP_OK;
}

bool TouchController::read(TouchSample &sample) {
  const bool touching = core_.read(device_, sample);
  current_ = sample;
  return touching;
}

TouchSample TouchController::cached() const {
  return current_;
}

void TouchController::consumeLatestMove(int *x, int *y) {
  latestMoveQueued_.store(false, std::memory_order_release);
  if (x) *x = latestMove_.x;
  if (y) *y = latestMove_.y;
}

esp_err_t TouchController::resetChip() {
  if constexpr (static_cast<int>(kResetPin) < 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
  } else {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << static_cast<unsigned>(kResetPin);
    config.mode = GPIO_MODE_OUTPUT;
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;

    gpio_set_level(kResetPin, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(kResetPin, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    return ESP_OK;
  }
}

esp_err_t TouchController::attachI2cDevice() {
  return device_.attach(bus_) ? ESP_OK : ESP_FAIL;
}

esp_err_t TouchController::configureChip() {
    // Read the chip id first: it is the only proof the controller is actually
  // answering. A silent I2C failure otherwise looks exactly like a screen
  // nobody is touching.
  std::uint8_t chipId = 0;
  if (!core_.readChipId(device_, chipId)) {
    ESP_LOGE(kTag, "touch controller did not answer on I2C");
    return ESP_ERR_NOT_FOUND;
  }
  if (chipId != gea::chips::ft6336::kExpectedChipId) {
    ESP_LOGW(kTag, "unexpected touch chip id 0x%02X (expected 0x%02X)", chipId,
             gea::chips::ft6336::kExpectedChipId);
  }
  return core_.configure(device_, false) ? ESP_OK : ESP_FAIL;
}

esp_err_t TouchController::startTask() {
  // Pinned to CPU1, where the parallel render worker also runs. kTaskPriority (4)
  // is BELOW that worker (5) so reads sample in its idle windows instead of
  // preempting its raster band — see the kTaskPriority comment for the full why.
  task_ = xTaskCreateStaticPinnedToCore(taskEntry, "ft6336-touch",
                                        static_cast<std::uint32_t>(sizeof(taskStack_) / sizeof(taskStack_[0])), this,
                                        kTaskPriority, taskStack_, &taskControlBlock_, 1);
  if (task_) return ESP_OK;
  ESP_LOGE(kTag, "Failed to start touch task");
  return ESP_ERR_NO_MEM;
}

esp_err_t TouchController::attachInterrupt() {
  // A board can route the controller's INT to an I2C expander rather than to a
  // GPIO, leaving board::touch.interrupt as GPIO_NUM_NC. Shifting by -1 is
  // undefined -- the compiler rejects it outright -- so skip the attach and let
  // the task poll instead: it reads the controller on its own cadence anyway,
  // and the interrupt only ever woke it sooner.
  // `if constexpr`, not `if`: kInterruptPin is a compile-time constant, so the
  // shift below is still evaluated -- and rejected -- in a plain branch.
  if constexpr (kInterruptPin < 0) {
    ESP_LOGI(kTag, "no interrupt GPIO for touch; polling instead");
    return ESP_OK;
  } else {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << static_cast<unsigned>(kInterruptPin);
    config.mode = GPIO_MODE_INPUT;
    config.intr_type = GPIO_INTR_NEGEDGE;
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) return err;

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    return gpio_isr_handler_add(kInterruptPin, interruptEntry, this);
  }
}

void IRAM_ATTR TouchController::interruptEntry(void *arg) {
  auto *self = static_cast<TouchController *>(arg);
  if (self) self->notifyTaskFromInterrupt();
}

void IRAM_ATTR TouchController::notifyTaskFromInterrupt() {
  BaseType_t woken = pdFALSE;
  if (task_) vTaskNotifyGiveFromISR(task_, &woken);
  portYIELD_FROM_ISR(woken);
}

void TouchController::taskEntry(void *arg) {
  static_cast<TouchController *>(arg)->pollLoop();
}

void TouchController::pollLoop() {
  // Per-finger slot tracking (mirrors the CST9217 multi-touch controller): each
  // slot emits its own Down/Move/Up tagged with its pointerId, so a second
  // finger (pointerId 1) reaches the maps pinch-zoom gesture. Slot 0 remains the
  // primary pointer (drives scroll/focus + the consumeLatestMove fast path).
  bool slotActive[kMaxFingers] = {false, false};
  int slotX[kMaxFingers] = {0, 0};
  int slotY[kMaxFingers] = {0, 0};
  bool anyActive = false;

  while (true) {
    // Interrupt-driven while idle (wait for the touch INT); poll at the report
    // rate while any finger is down.
    // Waiting forever while idle is only safe when an interrupt can wake this
    // task. A board that routes INT to an I2C expander has none, so it polls on
    // the report interval instead -- otherwise the first touch is never read.
    constexpr bool kInterruptDriven = kInterruptPin >= 0;
    const TickType_t wait =
        (anyActive || !kInterruptDriven) ? pdMS_TO_TICKS(kPollIntervalMs) : portMAX_DELAY;
    ulTaskNotifyTake(pdTRUE, wait);

    gea::chips::ft6336::MultiTouchSample multi;
    core_.readMulti(device_, multi);  // multi.count == 0 when no contact

    // DIAG: does the chip ever report a 2nd contact?
    static int s_maxCount = 0;
    if (multi.count > s_maxCount) {
      s_maxCount = multi.count;
      ESP_LOGW(kTag, "DIAG maxCount=%d (p0=%d,%d p1=%d,%d)", multi.count, multi.x[0], multi.y[0], multi.x[1],
               multi.y[1]);
    }

    current_ = multi.count > 0 ? TouchSample{true, multi.x[0], multi.y[0]}
                               : TouchSample{false, current_.x, current_.y};

    for (int slot = 0; slot < kMaxFingers; slot++) {
      const bool nowActive = slot < multi.count;
      const int x = nowActive ? multi.x[slot] : slotX[slot];
      const int y = nowActive ? multi.y[slot] : slotY[slot];

      if (nowActive && !slotActive[slot]) {
        ESP_LOGW(kTag, "DIAG slot %d DOWN %d,%d", slot, x, y);
        if (slot == 0) latestMove_ = TouchSample{true, x, y};
        notifyPointer(Phase::Down, true, x, y, slot);
      } else if (nowActive && slotActive[slot] && (x != slotX[slot] || y != slotY[slot])) {
        if (slot == 0) {
          // Coalesce primary moves through latestMove_ so the runtime's
          // consumeLatestMove drains the freshest coord (queued-move fast path).
          latestMove_ = TouchSample{true, x, y};
          if (!latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) {
            notifyPointer(Phase::Move, true, x, y, slot);
          }
        } else {
          notifyPointer(Phase::Move, true, x, y, slot);
        }
      } else if (!nowActive && slotActive[slot]) {
        notifyPointer(Phase::Up, false, x, y, slot);
      }

      if (nowActive) {
        slotX[slot] = x;
        slotY[slot] = y;
      }
      slotActive[slot] = nowActive;
    }
    anyActive = multi.count > 0;
  }
}

void TouchController::notifyObserver(Phase phase, const TouchSample &sample) {
  notifyPointer(phase, sample.touching, sample.x, sample.y, 0);
}

void TouchController::notifyPointer(Phase phase, bool touching, int x, int y, int pointerId) {
  // A multi-finger observer (set by the runtime when an app uses multi-touch)
  // takes every contact; otherwise the legacy single Observer gets the primary.
  if (pointerObserver_) {
    pointerObserver_(phase, touching, x, y, pointerId);
  } else if (observer_ && pointerId == 0) {
    observer_(phase, touching, x, y);
  }
}

void TouchController::notifyMove(const TouchSample &sample) {
  latestMove_ = sample;
  if (latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) return;
  notifyPointer(Phase::Move, true, sample.x, sample.y, 0);
}

void TouchController::injectEvent(Phase phase, bool touching, int x, int y) {
  TouchSample sample;
  sample.x = x;
  sample.y = y;
  sample.touching = touching;
  current_ = sample;
  // Route Move through notifyMove so latestMove_ is set — otherwise the
  // TouchRuntime's consumeLatestMove would overwrite the injected coords with
  // the last real hardware sample and the synthetic drag wouldn't scroll.
  if (phase == Phase::Move) notifyMove(sample);
  else notifyObserver(phase, sample);
}

}  // namespace gea::platform::esp32::chip_bindings::ft6336
