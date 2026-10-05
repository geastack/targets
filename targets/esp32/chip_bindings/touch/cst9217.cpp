#include "cst9217_controller.h"

#include "board.h"
#include "display.h"
#include "i2c.h"

#include <algorithm>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_lcd_panel_io.h"
// The CST92xx family shares this binding. A board whose controller is the
// CST9220 (the ESP-Mosaico) sets GEA_BOARD_TOUCH_CST9220: that part speaks
// either the legacy report protocol or HYN212, which needs a mode switch
// before reports arrive, and only Espressif's CST9220 driver handles both.
#if GEA_BOARD_TOUCH_CST816
#include "esp_lcd_touch_cst816s.h"
#define GEA_CST92XX_NEW esp_lcd_touch_new_i2c_cst816s
namespace {
esp_lcd_panel_io_i2c_config_t cst816IoConfig()
{
  // Vendor macro uses C designator order, which IDF 6 rejects in C++.
  esp_lcd_panel_io_i2c_config_t config = {};
  config.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS;
  config.control_phase_bytes = 1;
  config.lcd_cmd_bits = 8;
  config.lcd_param_bits = 0;
  config.flags.disable_control_phase = 1;
  config.scl_speed_hz = 400000;
  return config;
}
}
#define GEA_CST92XX_IO_CONFIG cst816IoConfig
#elif GEA_BOARD_TOUCH_CST9220
#include "esp_lcd_touch_cst9220.h"
#define GEA_CST92XX_NEW esp_lcd_touch_new_i2c_cst9220
namespace {
// ESP_LCD_TOUCH_IO_I2C_CST9220_CONFIG() lists its designators out of
// declaration order, which C++ rejects; the same values, field by field.
esp_lcd_panel_io_i2c_config_t cst9220IoConfig()
{
	esp_lcd_panel_io_i2c_config_t config = {};
	config.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST9220_ADDRESS;
	config.control_phase_bytes = 1;
	config.dc_bit_offset = 0;
	config.lcd_cmd_bits = 8;
	config.lcd_param_bits = 8;
	config.flags.disable_control_phase = 1;
	config.scl_speed_hz = 400000;
	return config;
}
}  // namespace
#define GEA_CST92XX_IO_CONFIG cst9220IoConfig
#else
#include "esp_lcd_touch_cst9217.h"
#define GEA_CST92XX_IO_CONFIG ESP_LCD_TOUCH_IO_I2C_CST9217_CONFIG
#define GEA_CST92XX_NEW esp_lcd_touch_new_i2c_cst9217
#endif
#include "esp_log.h"

namespace {

constexpr char kTag[] = "cst9217";
constexpr gpio_num_t kResetPin = gea::platform::board::touch.reset;
constexpr gpio_num_t kInterruptPin = gea::platform::board::touch.interrupt;
constexpr int kPollIntervalMs = 10;
constexpr int kTaskPriority = 10;

#if GEA_BOARD_TOUCH_CST9220
// The CST9220 raises INT (low) once per report. Reading between reports finds
// the frame the driver already acknowledged -- it logs "Malformed report
// header" and drops the sample -- so this binding reads on INT instead of on
// a timer. While a finger is down it still re-reads after kLiftTimeoutMs of
// silence, so a missed final report cannot leave a finger stuck down.
constexpr int kLiftTimeoutMs = 60;

void IRAM_ATTR touchInterrupt(void *arg)
{
	auto *task = static_cast<TaskHandle_t *>(arg);
	BaseType_t woken = pdFALSE;
	if (*task) vTaskNotifyGiveFromISR(*task, &woken);
	portYIELD_FROM_ISR(woken);
}
#endif

}  // namespace

namespace gea::platform::esp32::chip_bindings::cst9217 {

TouchController &TouchController::instance()
{
	static TouchController controller;
	return controller;
}

void TouchController::setObserver(Observer observer)
{
	observer_ = observer;
}

void TouchController::setPointerObserver(PointerObserver observer)
{
	pointerObserver_ = observer;
}

esp_err_t TouchController::begin()
{
	if (initialized_) return ESP_OK;

	ESP_LOGI(kTag, "Initializing CST9217 touch (RST=%d, INT=%d)", static_cast<int>(kResetPin), static_cast<int>(kInterruptPin));

	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return ESP_ERR_INVALID_STATE;
	bus_ = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());

	esp_err_t err = attachTouchDevice();
	if (err != ESP_OK) return err;

	err = startTask();
	if (err != ESP_OK) return err;

	initialized_ = true;
	ESP_LOGI(kTag, "Touch controller ready (polling)");
	return ESP_OK;
}

bool TouchController::read(TouchSample &sample)
{
	const MultiSample multi = readMulti();
	if (multi.count > 0) {
		sample.touching = true;
		sample.x = multi.x[0];
		sample.y = multi.y[0];
	} else {
		sample.touching = false;
	}
	current_ = sample;
	return sample.touching;
}

TouchSample TouchController::cached() const
{
	return current_;
}

void TouchController::consumeLatestMove(int *x, int *y)
{
	latestMoveQueued_.store(false, std::memory_order_release);
	if (x) *x = latestMove_.x;
	if (y) *y = latestMove_.y;
}

esp_err_t TouchController::attachTouchDevice()
{
	esp_lcd_panel_io_i2c_config_t io_config = GEA_CST92XX_IO_CONFIG();
	io_config.scl_speed_hz = 400000;
	esp_err_t err = esp_lcd_new_panel_io_i2c(bus_, &io_config, &io_);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "Failed to create touch I2C panel IO: %s", esp_err_to_name(err));
		return err;
	}

	esp_lcd_touch_config_t touch_config = {
		.x_max = gea::platform::display::kWidth,
		.y_max = gea::platform::display::kHeight,
		.rst_gpio_num = kResetPin,
		.int_gpio_num = kInterruptPin,
		.levels = {
			.reset = 0,
			.interrupt = 0,
		},
		.flags = {
			.swap_xy = gea::platform::board::touch.swapXy,
			.mirror_x = gea::platform::board::touch.mirrorX,
			.mirror_y = gea::platform::board::touch.mirrorY,
		},
	};
	err = GEA_CST92XX_NEW(io_, &touch_config, &touch_);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "Failed to create CST92xx touch device: %s", esp_err_to_name(err));
		return err;
	}
#if GEA_BOARD_TOUCH_CST9220
	// The driver configured INT as a falling-edge input; hook it to the task.
	err = gpio_install_isr_service(0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
	err = gpio_isr_handler_add(kInterruptPin, touchInterrupt, &task_);
	if (err == ESP_OK) err = gpio_intr_enable(kInterruptPin);
	if (err != ESP_OK) ESP_LOGE(kTag, "Touch INT GPIO%d hook failed: %s", static_cast<int>(kInterruptPin), esp_err_to_name(err));
#endif
	return err;
}

esp_err_t TouchController::startTask()
{
	task_ = xTaskCreateStatic(taskEntry, "cst9217-touch", static_cast<std::uint32_t>(sizeof(taskStack_) / sizeof(taskStack_[0])),
		this, kTaskPriority, taskStack_, &taskControlBlock_);
	if (task_) return ESP_OK;
	ESP_LOGE(kTag, "Failed to start touch task");
	return ESP_ERR_NO_MEM;
}

void TouchController::taskEntry(void *arg)
{
	static_cast<TouchController *>(arg)->pollLoop();
}

TouchController::MultiSample TouchController::readMulti()
{
	MultiSample sample;
	if (!touch_) return sample;

	esp_lcd_touch_read_data(touch_);
	esp_lcd_touch_point_data_t points[kMaxFingers] = {};
	uint8_t count = 0;
	const esp_err_t err = esp_lcd_touch_get_data(touch_, points, &count, kMaxFingers);
	if (err != ESP_OK || count == 0) return sample;

	sample.count = std::min(static_cast<int>(count), kMaxFingers);
	for (int i = 0; i < sample.count; i++) {
		sample.x[i] = std::clamp(static_cast<int>(points[i].x), 0, gea::platform::display::kWidth - 1);
		sample.y[i] = std::clamp(static_cast<int>(points[i].y), 0, gea::platform::display::kHeight - 1);
	}
	return sample;
}

void TouchController::pollLoop()
{
	bool slotActive[kMaxFingers] = {false, false};
	int slotX[kMaxFingers] = {0, 0};
	int slotY[kMaxFingers] = {0, 0};

	while (true) {
#if GEA_BOARD_TOUCH_CST9220
		const bool anyActive = slotActive[0] || slotActive[1];
		if (ulTaskNotifyTake(pdTRUE, anyActive ? pdMS_TO_TICKS(kLiftTimeoutMs) : portMAX_DELAY) == 0 && !anyActive)
			continue;
#else
		vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
#endif

		const MultiSample multi = readMulti();
		if (multi.count > 0) {
			current_ = TouchSample{true, multi.x[0], multi.y[0]};
		} else {
			current_.touching = false;
		}

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
	}
}

void TouchController::notifyObserver(Phase phase, const TouchSample &sample)
{
	notifyPointer(phase, sample.touching, sample.x, sample.y, 0);
}

void TouchController::notifyPointer(Phase phase, bool touching, int x, int y, int pointerId)
{
	if (pointerObserver_) {
		pointerObserver_(phase, touching, x, y, pointerId);
	} else if (observer_ && pointerId == 0) {
		observer_(phase, touching, x, y);
	}
}

void TouchController::notifyMove(const TouchSample &sample)
{
	latestMove_ = sample;
	if (latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) return;
	notifyPointer(Phase::Move, true, sample.x, sample.y, 0);
}

void TouchController::injectEvent(Phase phase, bool touching, int x, int y)
{
	TouchSample sample;
	sample.x = x;
	sample.y = y;
	sample.touching = touching;
	current_ = sample;
	if (phase == Phase::Move) notifyMove(sample);
	else notifyObserver(phase, sample);
}

}  // namespace gea::platform::esp32::chip_bindings::cst9217
