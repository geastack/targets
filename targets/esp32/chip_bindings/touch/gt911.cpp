// ESP-IDF binding for the GT911 touch controller: I2C transport, reset (over a
// GPIO or through the board's I/O expander), the INT line, and the task that
// turns reports into Touchscreen events. Unlike the FT3168 binding this file
// also answers the Touchscreen surface itself, so a composed board links it
// with no board-local glue.

#include "gt911_controller.h"

#include "board.h"
#include "i2c.h"
#if GEA_BOARD_HAS_EXPANDER
#include "chip_bindings/expanders/io_expander.h"
#endif

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "gt911";
constexpr gpio_num_t kResetPin = gea::platform::board::touch.reset;
constexpr gpio_num_t kInterruptPin = gea::platform::board::touch.interrupt;
#if GEA_BOARD_HAS_EXPANDER
constexpr int kExpanderResetPin = gea::platform::board::expander.touchReset;
#else
constexpr int kExpanderResetPin = -1;
#endif
constexpr int kPollIntervalMs = 10;
// Same reasoning as the FT3168 binding: sample in the render's idle windows on
// CPU1 rather than preempting the raster worker; a board with real-time work
// on that core raises it.
#ifndef GEA_EMBEDDED_TOUCH_TASK_PRIORITY
#define GEA_EMBEDDED_TOUCH_TASK_PRIORITY 4
#endif
constexpr int kTaskPriority = GEA_EMBEDDED_TOUCH_TASK_PRIORITY;
constexpr TickType_t kI2cTimeoutTicks = pdMS_TO_TICKS(100);

volatile std::int64_t g_touchIntUs = 0;

}  // namespace

namespace gea::platform::esp32::chip_bindings::gt911 {

bool EspI2cRegisterBus::attach(i2c_master_bus_handle_t bus, std::uint8_t address) {
	if (device_) return true;
	i2c_device_config_t config = {};
	config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	config.device_address = address;
	config.scl_speed_hz = gea::chips::gt911::kI2cFrequencyHz;
	return i2c_master_bus_add_device(bus, &config, &device_) == ESP_OK;
}

void EspI2cRegisterBus::detach() {
	if (!device_) return;
	i2c_master_bus_rm_device(device_);
	device_ = nullptr;
}

bool EspI2cRegisterBus::writeRegisters(std::uint16_t reg, const std::uint8_t *data, std::size_t length) {
	if (!device_ || length > 30) return false;
	std::uint8_t frame[32];
	frame[0] = static_cast<std::uint8_t>(reg >> 8);
	frame[1] = static_cast<std::uint8_t>(reg & 0xff);
	for (std::size_t i = 0; i < length; i++) frame[2 + i] = data[i];
	return i2c_master_transmit(device_, frame, 2 + length, kI2cTimeoutTicks) == ESP_OK;
}

bool EspI2cRegisterBus::readRegisters(std::uint16_t reg, std::uint8_t *data, std::size_t length) {
	if (!device_) return false;
	const std::uint8_t address[2] = {static_cast<std::uint8_t>(reg >> 8), static_cast<std::uint8_t>(reg & 0xff)};
	return i2c_master_transmit_receive(device_, address, sizeof(address), data, length, kI2cTimeoutTicks) == ESP_OK;
}

TouchController &TouchController::instance() {
	static TouchController controller;
	return controller;
}

void TouchController::setObserver(Observer observer) { observer_ = observer; }
void TouchController::setPointerObserver(PointerObserver observer) { pointerObserver_ = observer; }

esp_err_t TouchController::begin() {
	if (initialized_) return ESP_OK;

	ESP_LOGI(kTag, "Initializing GT911 touch (RST=%d, expander RST=%d, INT=%d)", static_cast<int>(kResetPin),
	         kExpanderResetPin, static_cast<int>(kInterruptPin));

	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return ESP_ERR_INVALID_STATE;
	bus_ = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());

	esp_err_t err = resetChip();
	if (err != ESP_OK) return err;

	err = attachI2cDevice();
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

TouchSample TouchController::cached() const { return current_; }

void TouchController::consumeLatestMove(int *x, int *y) {
	latestMoveQueued_.store(false, std::memory_order_release);
	if (x) *x = latestMove_.x;
	if (y) *y = latestMove_.y;
}

// The GT911 latches its I2C address from the INT line as reset is released:
// low picks 0x5d, high picks 0x14. Driving INT low across the reset makes the
// address deterministic whichever way the board's pull goes; a board without
// INT wired just gets both addresses probed in attachI2cDevice.
esp_err_t TouchController::resetChip() {
	if constexpr (kInterruptPin >= 0) {
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << static_cast<unsigned>(kInterruptPin);
		config.mode = GPIO_MODE_OUTPUT;
		esp_err_t err = gpio_config(&config);
		if (err != ESP_OK) return err;
		gpio_set_level(kInterruptPin, 0);
	}

	bool pulsed = false;
	if constexpr (kResetPin >= 0) {
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << static_cast<unsigned>(kResetPin);
		config.mode = GPIO_MODE_OUTPUT;
		esp_err_t err = gpio_config(&config);
		if (err != ESP_OK) return err;
		gpio_set_level(kResetPin, 0);
		vTaskDelay(pdMS_TO_TICKS(10));
		gpio_set_level(kResetPin, 1);
		pulsed = true;
	}
#if GEA_BOARD_HAS_EXPANDER
	if (!pulsed && kExpanderResetPin >= 0) {
		auto &expander = gea::platform::esp32::chip_bindings::expanders::ioExpander();
		if (!expander.writePin(kExpanderResetPin, false)) return ESP_FAIL;
		vTaskDelay(pdMS_TO_TICKS(10));
		if (!expander.writePin(kExpanderResetPin, true)) return ESP_FAIL;
		pulsed = true;
	}
#endif
	if (!pulsed) ESP_LOGI(kTag, "no reset line for touch; relying on power-on reset");
	// The controller needs ~50ms after reset before it answers; give it more
	// on a cold boot where the panel power is still settling.
	vTaskDelay(pdMS_TO_TICKS(pulsed ? 100 : 50));

	if constexpr (kInterruptPin >= 0) {
		// Hand INT back to the controller: input, no pull (the panel drives it).
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << static_cast<unsigned>(kInterruptPin);
		config.mode = GPIO_MODE_INPUT;
		config.intr_type = GPIO_INTR_DISABLE;
		esp_err_t err = gpio_config(&config);
		if (err != ESP_OK) return err;
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return ESP_OK;
}

esp_err_t TouchController::attachI2cDevice() {
	const std::uint8_t addresses[] = {gea::chips::gt911::kI2cAddressPrimary, gea::chips::gt911::kI2cAddressSecondary};
	for (std::uint8_t address : addresses) {
		if (!device_.attach(bus_, address)) continue;
		if (core_.configure(device_)) {
			ESP_LOGI(kTag, "GT911 found at 0x%02x", address);
			return ESP_OK;
		}
		device_.detach();
	}
	ESP_LOGE(kTag, "GT911 not found at 0x%02x or 0x%02x", gea::chips::gt911::kI2cAddressPrimary,
	         gea::chips::gt911::kI2cAddressSecondary);
	return ESP_ERR_NOT_FOUND;
}

esp_err_t TouchController::startTask() {
	task_ = xTaskCreateStaticPinnedToCore(taskEntry, "gt911-touch",
	                                      static_cast<std::uint32_t>(sizeof(taskStack_) / sizeof(taskStack_[0])), this,
	                                      kTaskPriority, taskStack_, &taskControlBlock_, 1);
	if (task_) return ESP_OK;
	ESP_LOGE(kTag, "Failed to start touch task");
	return ESP_ERR_NO_MEM;
}

esp_err_t TouchController::attachInterrupt() {
	if constexpr (kInterruptPin < 0) {
		ESP_LOGI(kTag, "no interrupt GPIO for touch; polling instead");
		return ESP_OK;
	} else {
		gpio_config_t config = {};
		config.pin_bit_mask = 1ULL << static_cast<unsigned>(kInterruptPin);
		config.mode = GPIO_MODE_INPUT;
		// The GT911 pulses INT high for every report by default (the polarity
		// is a config-register bit); waking on either edge covers both.
		config.intr_type = GPIO_INTR_ANYEDGE;
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
	g_touchIntUs = esp_timer_get_time();
	BaseType_t woken = pdFALSE;
	if (task_) vTaskNotifyGiveFromISR(task_, &woken);
	portYIELD_FROM_ISR(woken);
}

void TouchController::taskEntry(void *arg) { static_cast<TouchController *>(arg)->pollLoop(); }

void TouchController::pollLoop() {
	// Per-finger slot tracking (the same shape as the FT3168 and CST9217
	// bindings): each slot emits its own Down/Move/Up tagged with its pointerId,
	// so a second finger reaches pinch-zoom. Slot 0 is the primary pointer.
	bool slotActive[kMaxFingers] = {false, false};
	int slotX[kMaxFingers] = {0, 0};
	int slotY[kMaxFingers] = {0, 0};
	bool anyActive = false;

	while (true) {
		constexpr bool kInterruptDriven = kInterruptPin >= 0;
		const TickType_t wait = (anyActive || !kInterruptDriven) ? pdMS_TO_TICKS(kPollIntervalMs) : portMAX_DELAY;
		ulTaskNotifyTake(pdTRUE, wait);

		gea::chips::gt911::MultiTouchSample multi;
		core_.readMulti(device_, multi);

		current_ = multi.count > 0 ? TouchSample{true, multi.x[0], multi.y[0]}
		                           : TouchSample{false, current_.x, current_.y};

		for (int slot = 0; slot < kMaxFingers; slot++) {
			const bool nowActive = slot < multi.count;
			const int x = nowActive ? multi.x[slot] : slotX[slot];
			const int y = nowActive ? multi.y[slot] : slotY[slot];

			if (nowActive && !slotActive[slot]) {
				if (slot == 0) latestMove_ = TouchSample{true, x, y};
				notifyPointer(Phase::Down, true, x, y, slot);
			} else if (nowActive && slotActive[slot] && (x != slotX[slot] || y != slotY[slot])) {
				if (slot == 0) {
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

void TouchController::notifyPointer(Phase phase, bool touching, int x, int y, int pointerId) {
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
	if (phase == Phase::Move) notifyMove(sample);
	else notifyPointer(phase, touching, x, y, 0);
}

}  // namespace gea::platform::esp32::chip_bindings::gt911

// The Touchscreen surface, answered here so a composed board links the GT911
// without a board-local touch.cpp.
namespace {

gea::platform::esp32::chip_bindings::gt911::TouchController &touchController() {
	return gea::platform::esp32::chip_bindings::gt911::TouchController::instance();
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer) { touchController().setObserver(observer); }

// Strong override of the runtime's weak no-op: the GT911 reports several
// fingers, so every contact reaches the app.
void gea::platform::touch::Touchscreen::setPointerObserver(PointerObserver observer) {
	touchController().setPointerObserver(observer);
}

bool gea::platform::touch::Touchscreen::init() { return touchController().begin() == ESP_OK; }

int gea::platform::touch::Touchscreen::read(int *x, int *y) {
	gea::chips::gt911::TouchSample sample;
	const bool touching = touchController().read(sample);
	if (x) *x = sample.x;
	if (y) *y = sample.y;
	return touching ? 1 : 0;
}

int gea::platform::touch::Touchscreen::readCached(int *x, int *y) {
	const auto sample = touchController().cached();
	if (x) *x = sample.x;
	if (y) *y = sample.y;
	return sample.touching ? 1 : 0;
}

void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y) { touchController().consumeLatestMove(x, y); }

void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y) {
	touchController().injectEvent(phase, touching, x, y);
}
