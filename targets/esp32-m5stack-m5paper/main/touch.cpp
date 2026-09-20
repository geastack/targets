#include "touch.h"

#include "board.h"
#include "buttons.h"
#include "host/display_orientation.h"
#include "i2c.h"

#include <atomic>
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "m5paper_touch";
constexpr int kPollIntervalMs = 10;
// Dedicated touch-sampling task cadence. The GT911 reports only the CURRENT
// finger state, not a buffered history, so a tap that goes down-and-up between
// two samples is lost entirely. On this board a page turn (SD read + paginate +
// ~200 ms e-paper flush) blocks the runtime frame loop far longer than a frame,
// and touch polling used to ride that same loop — so every tap during a draw
// vanished (the "rapid taps only skip 2 pages" bug). Sampling on its own task
// at 6 ms keeps catching down/up edges through the whole draw; the events queue
// and the runtime coalesces them into one multi-page skip when it comes back.
constexpr int kTouchTaskIntervalMs = 6;
constexpr int kMaxFingers = 2;

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

struct MultiSample {
	int count = 0;
	int x[kMaxFingers] = {0, 0};
	int y[kMaxFingers] = {0, 0};
};

class Gt911TouchController {
public:
	static Gt911TouchController &instance()
	{
		static Gt911TouchController controller;
		return controller;
	}

	void setObserver(gea::platform::touch::Touchscreen::Observer observer) { observer_ = observer; }
	void setPointerObserver(gea::platform::touch::Touchscreen::PointerObserver observer) { pointerObserver_ = observer; }

	bool begin()
	{
		if (initialized_) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;
		i2cBus_ = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());

		esp_err_t err = attach(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
		if (err != ESP_OK) err = attach(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "GT911 not found at 0x14 or 0x5d");
			return false;
		}

		initialized_ = true;
		hwMutex_ = xSemaphoreCreateMutex();
		// Sample the GT911 on a dedicated high-priority task so touch keeps being
		// read even while the runtime frame task is blocked mid-page-draw. Pinned
		// to PRO_CPU (core 0) at a priority above the runtime task (5) and the
		// e-paper refresh task (6); if a CPU-bound page derive is running on the
		// same core it is preempted every 6 ms, and if it's on the other core the
		// sampling runs fully parallel. The task only READS the panel and QUEUES
		// events (thread-safe via the FrameScheduler queue); event dispatch and
		// rendering stay on the runtime task.
		xTaskCreatePinnedToCore(touchTaskEntry, "m5paper_touch", 3072, this, 7, &touchTask_, 0);
		return true;
	}

	void poll(int nowMs)
	{
		gea::platform::m5paper::buttons::poll(nowMs);
		if (!initialized_) return;
		// The dedicated task owns GT911 sampling; the frame-loop poll only drives
		// the hardware buttons. Fall back to sampling here if the task never
		// started (allocation failure), so touch still works degraded.
		if (touchTask_ != nullptr) return;
		const auto now = static_cast<std::uint32_t>(nowMs);
		if (nextPollMs_ != 0 && static_cast<std::int32_t>(now - nextPollMs_) < 0) return;
		nextPollMs_ = now + kPollIntervalMs;
		pollOnce();
	}

	static void touchTaskEntry(void *arg)
	{
		auto *self = static_cast<Gt911TouchController *>(arg);
		for (;;) {
			self->pollOnce();
			vTaskDelay(pdMS_TO_TICKS(kTouchTaskIntervalMs));
		}
	}

	int read(int *x, int *y)
	{
		current_ = readHardware();
		if (x) *x = current_.x;
		if (y) *y = current_.y;
		return current_.touching ? 1 : 0;
	}

	int readCached(int *x, int *y) const
	{
		if (x) *x = current_.x;
		if (y) *y = current_.y;
		return current_.touching ? 1 : 0;
	}

	void consumeLatestMove(int *x, int *y)
	{
		latestMoveQueued_.store(false, std::memory_order_release);
		if (x) *x = latestMove_.x;
		if (y) *y = latestMove_.y;
	}

	void injectEvent(gea::platform::touch::Phase phase, bool touching, int x, int y)
	{
		current_ = TouchSample{touching, x, y};
		latestMove_ = current_;
		latestMoveQueued_.store(phase == gea::platform::touch::Phase::Move, std::memory_order_release);
		notify(phase, touching, x, y, 0);
	}

private:
	esp_err_t attach(std::uint8_t address)
	{
		esp_lcd_panel_io_i2c_config_t ioConfig = {};
		ioConfig.dev_addr = address;
		ioConfig.scl_speed_hz = 100000;  // shared SHT30 bus is specified at 100 kHz
		ioConfig.control_phase_bytes = 1;
		ioConfig.dc_bit_offset = 0;
		ioConfig.lcd_cmd_bits = 16;
		ioConfig.flags.disable_control_phase = 1;

		esp_lcd_panel_io_handle_t io = nullptr;
		esp_err_t err = esp_lcd_new_panel_io_i2c(i2cBus_, &ioConfig, &io);
		if (err != ESP_OK) return err;

		esp_lcd_touch_config_t config = {};
		config.x_max = gea::platform::display::kNativeWidth;
		config.y_max = gea::platform::display::kNativeHeight;
		config.rst_gpio_num = gea::platform::board::touch.reset;
		config.int_gpio_num = gea::platform::board::touch.interrupt;
		config.levels.reset = 0;
		config.levels.interrupt = 0;
		config.flags.swap_xy = false;
		config.flags.mirror_x = false;
		config.flags.mirror_y = false;

		err = esp_lcd_touch_new_i2c_gt911(io, &config, &touch_);
		if (err != ESP_OK) {
			esp_lcd_panel_io_del(io);
			return err;
		}
		touchIo_ = io;
		ESP_LOGI(kTag, "GT911 ready address=0x%02x INT=%d", address, static_cast<int>(config.int_gpio_num));
		return ESP_OK;
	}

	MultiSample readMulti()
	{
		MultiSample result;
		if (!touch_) return result;
		// read_data fills the touch handle's internal buffer and get_coordinates
		// drains it — a two-step, non-atomic sequence on shared state. Now that a
		// dedicated task samples continuously, an app-driven read() on the runtime
		// task could interleave and mix the two callers' data (or glitch the
		// driver's point state). Serialize the pair (mutex created in begin();
		// a null mutex just skips locking — degraded, not broken).
		if (hwMutex_) xSemaphoreTake(hwMutex_, portMAX_DELAY);
		esp_lcd_touch_read_data(touch_);
		std::uint16_t xs[kMaxFingers] = {};
		std::uint16_t ys[kMaxFingers] = {};
		std::uint16_t strengths[kMaxFingers] = {};
		std::uint8_t count = 0;
		const bool ok = esp_lcd_touch_get_coordinates(touch_, xs, ys, strengths, &count, kMaxFingers);
		if (hwMutex_) xSemaphoreGive(hwMutex_);
		if (!ok || count == 0) return result;
		result.count = count > kMaxFingers ? kMaxFingers : static_cast<int>(count);
		for (int i = 0; i < result.count; ++i) {
			int x = xs[i];
			int y = ys[i];
			gea::framework::display::detail::DisplayOrientationState::mapPanelToViewport(&x, &y);
			result.x[i] = x;
			result.y[i] = y;
		}
		return result;
	}

	TouchSample readHardware()
	{
		const MultiSample sample = readMulti();
		if (sample.count == 0) return TouchSample{false, current_.x, current_.y};
		return TouchSample{true, sample.x[0], sample.y[0]};
	}

	void notify(gea::platform::touch::Phase phase, bool touching, int x, int y, int pointerId)
	{
		if (pointerObserver_) pointerObserver_(phase, touching, x, y, pointerId);
		else if (observer_ && pointerId == 0) observer_(phase, touching, x, y);
	}

	void pollOnce()
	{
		const MultiSample sample = readMulti();
		if (sample.count > 0) current_ = TouchSample{true, sample.x[0], sample.y[0]};
		else current_.touching = false;

		for (int slot = 0; slot < kMaxFingers; ++slot) {
			const bool nowActive = slot < sample.count;
			const int x = nowActive ? sample.x[slot] : oldX_[slot];
			const int y = nowActive ? sample.y[slot] : oldY_[slot];
			if (nowActive && !active_[slot]) {
				notify(gea::platform::touch::Phase::Down, true, x, y, slot);
			} else if (nowActive && active_[slot] && (x != oldX_[slot] || y != oldY_[slot])) {
				if (slot == 0) {
					latestMove_ = TouchSample{true, x, y};
					if (!latestMoveQueued_.exchange(true, std::memory_order_acq_rel))
						notify(gea::platform::touch::Phase::Move, true, x, y, slot);
				} else {
					notify(gea::platform::touch::Phase::Move, true, x, y, slot);
				}
			} else if (!nowActive && active_[slot]) {
				notify(gea::platform::touch::Phase::Up, false, x, y, slot);
			}
			if (nowActive) {
				oldX_[slot] = x;
				oldY_[slot] = y;
			}
			active_[slot] = nowActive;
		}
	}

	gea::platform::touch::Touchscreen::Observer observer_ = nullptr;
	gea::platform::touch::Touchscreen::PointerObserver pointerObserver_ = nullptr;
	i2c_master_bus_handle_t i2cBus_ = nullptr;
	esp_lcd_touch_handle_t touch_ = nullptr;
	esp_lcd_panel_io_handle_t touchIo_ = nullptr;
	TouchSample current_ = {};
	TouchSample latestMove_ = {};
	std::atomic<bool> latestMoveQueued_{false};
	bool active_[kMaxFingers] = {};
	int oldX_[kMaxFingers] = {};
	int oldY_[kMaxFingers] = {};
	std::uint32_t nextPollMs_ = 0;
	bool initialized_ = false;
	TaskHandle_t touchTask_ = nullptr;
	SemaphoreHandle_t hwMutex_ = nullptr;
};

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer) { Gt911TouchController::instance().setObserver(observer); }
void gea::platform::touch::Touchscreen::setPointerObserver(PointerObserver observer) { Gt911TouchController::instance().setPointerObserver(observer); }
bool gea::platform::touch::Touchscreen::init() { return Gt911TouchController::instance().begin(); }
void gea::platform::touch::Touchscreen::poll(int nowMs) { Gt911TouchController::instance().poll(nowMs); }
int gea::platform::touch::Touchscreen::read(int *x, int *y) { return Gt911TouchController::instance().read(x, y); }
int gea::platform::touch::Touchscreen::readCached(int *x, int *y) { return Gt911TouchController::instance().readCached(x, y); }
void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y) { Gt911TouchController::instance().consumeLatestMove(x, y); }
void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
	Gt911TouchController::instance().injectEvent(phase, touching, x, y);
}
