#pragma once

#include "touch.h"

#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_touch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::chip_bindings::cst9217 {

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
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

	esp_err_t attachTouchDevice();
	esp_err_t startTask();
	static void taskEntry(void *arg);
	void pollLoop();
	void notifyObserver(Phase phase, const TouchSample &sample);
	void notifyPointer(Phase phase, bool touching, int x, int y, int pointerId);
	void notifyMove(const TouchSample &sample);

	static constexpr std::size_t kTaskStackWords = 4096;
	static constexpr int kMaxFingers = 2;

	struct MultiSample {
		int count = 0;
		int x[kMaxFingers] = {0, 0};
		int y[kMaxFingers] = {0, 0};
	};

	MultiSample readMulti();

	bool initialized_ = false;
	i2c_master_bus_handle_t bus_ = nullptr;
	esp_lcd_panel_io_handle_t io_ = nullptr;
	esp_lcd_touch_handle_t touch_ = nullptr;
	TaskHandle_t task_ = nullptr;
	StaticTask_t taskControlBlock_ = {};
	StackType_t taskStack_[kTaskStackWords] = {};
	TouchSample current_ = {};
	TouchSample latestMove_ = {};
	std::atomic<bool> latestMoveQueued_{false};
	Observer observer_ = nullptr;
	PointerObserver pointerObserver_ = nullptr;
};

}  // namespace gea::platform::esp32::chip_bindings::cst9217
