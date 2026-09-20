#include "touch.h"

#include "board.h"
#include "display.h"
#include "events.h"
#include "i2c.h"

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "ft5x06";
constexpr int kPollIntervalMs = 10;
constexpr int kTaskPriority = 10;
constexpr int kPanelWidth = gea::platform::display::kNativeWidth;
constexpr int kPanelHeight = gea::platform::display::kNativeHeight;

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

class Ft5x06TouchController {
public:
	static Ft5x06TouchController &instance()
	{
		static Ft5x06TouchController controller;
		return controller;
	}

	void setObserver(gea::platform::touch::Touchscreen::Observer observer)
	{
		observer_ = observer;
	}

	bool begin()
	{
		if (initialized_) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus not ready");
			return false;
		}
		i2cBus_ = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());

		esp_err_t err = attachTouch();
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "FT5x06 not found");
			return false;
		}

		task_ = xTaskCreateStatic(taskEntry,
		                          "gt911-touch",
		                          static_cast<std::uint32_t>(sizeof(taskStack_) / sizeof(taskStack_[0])),
		                          this,
		                          kTaskPriority,
		                          taskStack_,
		                          &taskControlBlock_);
		if (!task_) {
			ESP_LOGE(kTag, "failed to start touch task");
			return false;
		}
		initialized_ = true;
		ESP_LOGI(kTag, "touch ready");
		return true;
	}

	int read(int *x, int *y)
	{
		TouchSample sample = readHardware();
		current_ = sample;
		if (x) *x = sample.x;
		if (y) *y = sample.y;
		return sample.touching ? 1 : 0;
	}

	int readCached(int *x, int *y) const
	{
		const TouchSample sample = current_;
		if (x) *x = sample.x;
		if (y) *y = sample.y;
		return sample.touching ? 1 : 0;
	}

	void consumeLatestMove(int *x, int *y)
	{
		latestMoveQueued_.store(false, std::memory_order_release);
		if (x) *x = latestMove_.x;
		if (y) *y = latestMove_.y;
	}

	void injectEvent(gea::platform::touch::Phase phase, bool touching, int x, int y)
	{
		TouchSample sample{touching, x, y};
		current_ = sample;
		latestMove_ = sample;
		latestMoveQueued_.store(phase == gea::platform::touch::Phase::Move, std::memory_order_release);
		notify(phase, sample);
	}

private:
	esp_err_t attachTouch()
	{
		// Build the FT5x06 panel-IO config by hand: the vendor
		// ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG() macro uses out-of-declaration-order
		// designated initializers, which is a hard error in C++.
		esp_lcd_panel_io_i2c_config_t ioConfig = {};
		ioConfig.dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS;
		ioConfig.scl_speed_hz = 400000;
		ioConfig.control_phase_bytes = 1;
		ioConfig.dc_bit_offset = 0;
		ioConfig.lcd_cmd_bits = 8;
		ioConfig.flags.disable_control_phase = 1;

		esp_lcd_panel_io_handle_t io = nullptr;
		esp_err_t err = esp_lcd_new_panel_io_i2c(i2cBus_, &ioConfig, &io);
		if (err != ESP_OK) return err;

		esp_lcd_touch_config_t touchConfig = {};
		touchConfig.x_max = kPanelWidth;
		touchConfig.y_max = kPanelHeight;
		touchConfig.rst_gpio_num = gea::platform::board::touch.reset;
		touchConfig.int_gpio_num = gea::platform::board::touch.interrupt;
		touchConfig.levels.reset = 0;
		touchConfig.levels.interrupt = 0;
		touchConfig.flags.swap_xy = false;
		touchConfig.flags.mirror_x = false;
		touchConfig.flags.mirror_y = false;

		err = esp_lcd_touch_new_i2c_ft5x06(io, &touchConfig, &touch_);
		if (err != ESP_OK) {
			esp_lcd_panel_io_del(io);
			return err;
		}
		touchIo_ = io;
		ESP_LOGI(kTag, "FT5x06 touch ready");
		return ESP_OK;
	}

	TouchSample readHardware()
	{
		TouchSample sample = current_;
		if (!touch_) return sample;
		esp_lcd_touch_read_data(touch_);
		std::uint16_t x = 0;
		std::uint16_t y = 0;
		std::uint16_t strength = 0;
		std::uint8_t count = 0;
		const bool pressed = esp_lcd_touch_get_coordinates(touch_, &x, &y, &strength, &count, 1);
		if (!pressed || count == 0) {
			sample.touching = false;
			return sample;
		}
		// Report PANEL-NATIVE coordinates. TouchRuntime owns the panel->logical
		// rotation (queueTouchEvent + the dispatcher's consumeLatestMove path);
		// rotating here as well double-rotates in landscape, throwing most taps
		// off the 480x320 viewport (camera buttons stopped registering).
		sample.touching = true;
		sample.x = static_cast<int>(x);
		sample.y = static_cast<int>(y);
		return sample;
	}

	void pollLoop()
	{
		TouchSample last;
		bool downPending = false;
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
			TouchSample next = readHardware();
			if (!next.touching) {
				next.x = last.x;
				next.y = last.y;
			}

			// FT5x06 quirk: the first report of a NEW touch can still carry the
			// PREVIOUS touch's coordinates (TD_STATUS flips one scan before the
			// P1 registers update). Dispatching Down on that report routed taps
			// to whatever widget the user touched LAST (user-visible: tapping
			// the camera preview "pressed" the AE or capture button). Hold the
			// Down for one poll (10 ms) and dispatch with the refreshed
			// coordinates; a release within that window dispatches Down+Up from
			// the freshest read we have.
			if (next.touching && !last.touching && !downPending) {
				downPending = true;
				pendingDown_ = next;
				current_ = next;
				continue;
			}
			if (downPending) {
				downPending = false;
				if (next.touching) {
					notify(gea::platform::touch::Phase::Down, next);
					current_ = next;
					last = next;
					continue;
				}
				// Ultra-short tap: only the first (possibly stale) read exists —
				// dispatch the full tap from it; better than dropping the input.
				notify(gea::platform::touch::Phase::Down, pendingDown_);
				TouchSample up = pendingDown_;
				up.touching = false;
				notify(gea::platform::touch::Phase::Up, up);
				current_ = up;
				last.touching = false;
				continue;
			}

			if (next.touching && !last.touching) {
				notify(gea::platform::touch::Phase::Down, next);
			} else if (!next.touching && last.touching) {
				notify(gea::platform::touch::Phase::Up, next);
			} else if (next.touching && (next.x != last.x || next.y != last.y)) {
				latestMove_ = next;
				if (!latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) {
					notify(gea::platform::touch::Phase::Move, next);
				}
			}

			current_ = next;
			if (next.touching) last = next;
			else last.touching = false;
		}
	}

	void notify(gea::platform::touch::Phase phase, const TouchSample &sample)
	{
		// Down audit trail for transform debugging: panel-native coordinates as
		// reported, alongside the logical coordinates TouchRuntime will hit-test.
		if (phase == gea::platform::touch::Phase::Down) {
			int lx = sample.x;
			int ly = sample.y;
			gea::framework::events::TouchRuntime::transformTouchToLogical(&lx, &ly);
			ESP_LOGI(kTag, "touch down panel=(%d,%d) logical=(%d,%d)", sample.x, sample.y, lx, ly);
		}
		if (observer_) observer_(phase, sample.touching, sample.x, sample.y);
	}

	static void taskEntry(void *arg)
	{
		static_cast<Ft5x06TouchController *>(arg)->pollLoop();
	}

	gea::platform::touch::Touchscreen::Observer observer_ = nullptr;
	i2c_master_bus_handle_t i2cBus_ = nullptr;
	esp_lcd_touch_handle_t touch_ = nullptr;
	esp_lcd_panel_io_handle_t touchIo_ = nullptr;
	TaskHandle_t task_ = nullptr;
	StaticTask_t taskControlBlock_{};
	StackType_t taskStack_[4096]{};
	TouchSample current_{};
	TouchSample latestMove_{};
	// First read of a new touch, held one poll for coordinate confirmation
	// (see pollLoop's FT5x06 stale-first-report note).
	TouchSample pendingDown_{};
	std::atomic<bool> latestMoveQueued_{false};
	bool initialized_ = false;
};

Ft5x06TouchController &touchController()
{
	return Ft5x06TouchController::instance();
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer)
{
	touchController().setObserver(observer);
}

bool gea::platform::touch::Touchscreen::init()
{
	return touchController().begin();
}

int gea::platform::touch::Touchscreen::read(int *x, int *y)
{
	return touchController().read(x, y);
}

int gea::platform::touch::Touchscreen::readCached(int *x, int *y)
{
	return touchController().readCached(x, y);
}

void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y)
{
	touchController().consumeLatestMove(x, y);
}

void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
	touchController().injectEvent(phase, touching, x, y);
}
