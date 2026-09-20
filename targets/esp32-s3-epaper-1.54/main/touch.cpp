#include "touch.h"

#include "board.h"
#include "host/display_orientation.h"
#include "i2c.h"

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "ft6336";
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

		// The FT6336 needs the vendor's slow reset (100 ms high/low/high) and a
		// settle period before it ACKs on I2C — the esp_lcd_touch built-in reset
		// pulse is ~10 ms and the immediate probe after it fails. Reset by hand
		// here and pass rst=NC to the driver so it doesn't re-reset.
		resetController();

		esp_err_t err = ESP_ERR_TIMEOUT;
		for (int attempt = 0; attempt < 3 && err != ESP_OK; ++attempt) {
			if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(100));
			err = attachTouch();
		}
		if (err != ESP_OK) {
			// Waveshare sells this board in touch (FT6336) and non-touch
			// variants. A missing controller must not kill the runtime
			// (runtime.cpp aborts when Touchscreen::init() fails): without
			// hardware touch the app still takes input through GEADEV serial
			// tap injection (injectEvent below). Report ready, skip polling.
			ESP_LOGW(kTag, "FT6336 not found — non-touch board variant; serial tap injection only");
			initialized_ = true;
			return true;
		}

		task_ = xTaskCreateStatic(taskEntry,
		                          "ft6336-touch",
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
	// Vendor FT6336 reset: 100 ms high, 100 ms low, then 200 ms settle after
	// release before the controller answers on I2C.
	void resetController()
	{
		const gpio_num_t rst = gea::platform::board::touch.reset;
		if (rst == GPIO_NUM_NC) return;
		gpio_config_t conf = {};
		conf.intr_type = GPIO_INTR_DISABLE;
		conf.mode = GPIO_MODE_OUTPUT;
		conf.pin_bit_mask = 1ULL << rst;
		conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		conf.pull_up_en = GPIO_PULLUP_DISABLE;
		ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&conf));
		gpio_set_level(rst, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(rst, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(rst, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
	}

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
		// Reset is done by resetController() with the vendor timing; NC here so
		// the driver's short built-in pulse doesn't re-reset the controller.
		touchConfig.rst_gpio_num = GPIO_NUM_NC;
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
		ESP_LOGI(kTag, "FT6336 touch ready");
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
		int mappedX = static_cast<int>(x);
		int mappedY = static_cast<int>(y);
		gea::framework::display::detail::DisplayOrientationState::mapPanelToViewport(&mappedX, &mappedY);
		sample.touching = true;
		sample.x = mappedX;
		sample.y = mappedY;
		return sample;
	}

	void pollLoop()
	{
		TouchSample last;
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
			TouchSample next = readHardware();
			if (!next.touching) {
				next.x = last.x;
				next.y = last.y;
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
