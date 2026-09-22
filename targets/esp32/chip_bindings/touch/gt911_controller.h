#pragma once

#include "touch/gt911/gt911.h"
#include "touch.h"

#include <cstddef>
#include <cstdint>
#include <atomic>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::chip_bindings::gt911 {

using TouchSample = gea::chips::gt911::TouchSample;

// GT911 registers are 16 bits wide; the address goes out big-endian ahead of
// the data in one transaction.
class EspI2cRegisterBus final : public gea::chips::gt911::RegisterBus {
public:
	bool attach(i2c_master_bus_handle_t bus, std::uint8_t address);
	void detach();
	bool attached() const { return device_ != nullptr; }
	bool writeRegisters(std::uint16_t reg, const std::uint8_t *data, std::size_t length) override;
	bool readRegisters(std::uint16_t reg, std::uint8_t *data, std::size_t length) override;

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
	void setPointerObserver(PointerObserver observer);
	esp_err_t begin();
	bool read(TouchSample &sample);
	TouchSample cached() const;
	void consumeLatestMove(int *x, int *y);
	void injectEvent(Phase phase, bool touching, int x, int y);

private:
	TouchController() = default;

	esp_err_t resetChip();
	esp_err_t attachI2cDevice();
	esp_err_t startTask();
	esp_err_t attachInterrupt();
	static void interruptEntry(void *arg);
	void notifyTaskFromInterrupt();
	static void taskEntry(void *arg);
	void pollLoop();
	void notifyPointer(Phase phase, bool touching, int x, int y, int pointerId);
	void notifyMove(const TouchSample &sample);

	static constexpr std::size_t kTaskStackWords = 4096;
	static constexpr int kMaxFingers = gea::chips::gt911::kMaxTouchPoints;

	bool initialized_ = false;
	i2c_master_bus_handle_t bus_ = nullptr;
	EspI2cRegisterBus device_;
	gea::chips::gt911::ControllerCore core_;
	TaskHandle_t task_ = nullptr;
	StaticTask_t taskControlBlock_ = {};
	StackType_t taskStack_[kTaskStackWords] = {};
	TouchSample current_ = {};
	TouchSample latestMove_ = {};
	std::atomic<bool> latestMoveQueued_{false};
	Observer observer_ = nullptr;
	PointerObserver pointerObserver_ = nullptr;
};

}  // namespace gea::platform::esp32::chip_bindings::gt911
