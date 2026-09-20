#pragma once

#include "touch/ft6336/ft6336.h"
#include "touch.h"

#include <cstddef>
#include <cstdint>
#include <atomic>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::chip_bindings::ft6336 {

using TouchSample = gea::chips::ft6336::TouchSample;

class EspI2cRegisterBus final : public gea::chips::ft6336::RegisterBus {
public:
	bool attach(i2c_master_bus_handle_t bus);
	bool writeRegister(std::uint8_t reg, std::uint8_t value) override;
	bool readRegister(std::uint8_t reg, std::uint8_t &value) override;
	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override;

private:
	i2c_master_dev_handle_t device_ = nullptr;
};

class TouchController {
public:
  using Observer = gea::platform::touch::Touchscreen::Observer;
  using PointerObserver = gea::platform::touch::Touchscreen::PointerObserver;
  using Phase = gea::platform::touch::Phase;

  TouchController(const TouchController &) = delete;
  TouchController &operator=(const TouchController &) = delete;

  static TouchController &instance();

  void setObserver(Observer observer);
  // Strong override of the runtime's weak default: a multi-finger observer that
  // receives every contact tagged with a pointerId (0 = primary). The maps
  // pinch-zoom gesture needs the second finger (pointerId 1).
  void setPointerObserver(PointerObserver observer);
  esp_err_t begin();
  bool read(TouchSample &sample);
  TouchSample cached() const;
  void consumeLatestMove(int *x, int *y);
  // Inject a synthetic touch event through the same notify path as the hardware
  // poll loop. A Move updates latestMove_ (so the TouchRuntime's
  // consumeLatestMove returns these coords instead of stale hardware samples),
  // letting GEADEV DRAG/SWIPE drive scroll gestures exactly like a real finger.
  void injectEvent(Phase phase, bool touching, int x, int y);

private:
  TouchController() = default;

	esp_err_t resetChip();
	esp_err_t attachI2cDevice();
	esp_err_t configureChip();
	esp_err_t startTask();
	esp_err_t attachInterrupt();
  static void interruptEntry(void *arg);
  void notifyTaskFromInterrupt();
  static void taskEntry(void *arg);
  void pollLoop();
  void notifyObserver(Phase phase, const TouchSample &sample);
  void notifyPointer(Phase phase, bool touching, int x, int y, int pointerId);
  void notifyMove(const TouchSample &sample);

  static constexpr std::size_t kTaskStackWords = 4096;
  static constexpr int kMaxFingers = gea::chips::ft6336::kMaxTouchPoints;

  bool initialized_ = false;
  i2c_master_bus_handle_t bus_ = nullptr;
  EspI2cRegisterBus device_;
  gea::chips::ft6336::ControllerCore core_;
  TaskHandle_t task_ = nullptr;
  StaticTask_t taskControlBlock_ = {};
  StackType_t taskStack_[kTaskStackWords] = {};
  gea::chips::ft6336::TouchSample current_ = {};
  gea::chips::ft6336::TouchSample latestMove_ = {};
  std::atomic<bool> latestMoveQueued_{false};
  Observer observer_ = nullptr;
  PointerObserver pointerObserver_ = nullptr;
};

}  // namespace gea::platform::esp32::chip_bindings::ft6336
