#include "touch.h"

#include "board.h"
#include "host/display_orientation.h"
#include "i2c.h"

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "gt911";
constexpr int kPollIntervalMs = 10;
constexpr int kTaskPriority = 10;
constexpr std::uint8_t kGt911Address = 0x5d;
constexpr std::uint8_t kGt911BackupAddress = 0x14;
constexpr int kPanelWidth = gea::platform::display::kNativeWidth;
constexpr int kPanelHeight = gea::platform::display::kNativeHeight;

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

// Up to two simultaneous fingers (enough for pinch). Slot 0 is the primary
// pointer; slot 1 is the second finger.
constexpr int kMaxFingers = 2;
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

	void setObserver(gea::platform::touch::Touchscreen::Observer observer)
	{
		observer_ = observer;
	}

	void setPointerObserver(gea::platform::touch::Touchscreen::PointerObserver observer)
	{
		pointerObserver_ = observer;
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

		esp_err_t err = attachTouch(kGt911Address);
		if (err != ESP_OK) err = attachTouch(kGt911BackupAddress);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "GT911 not found");
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
	esp_err_t attachTouch(std::uint8_t address)
	{
		esp_lcd_panel_io_i2c_config_t ioConfig{};
		ioConfig.dev_addr = address;
		ioConfig.scl_speed_hz = 400000;
		ioConfig.control_phase_bytes = 1;
		ioConfig.dc_bit_offset = 0;
		ioConfig.lcd_cmd_bits = 16;
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

		err = esp_lcd_touch_new_i2c_gt911(io, &touchConfig, &touch_);
		if (err != ESP_OK) {
			esp_lcd_panel_io_del(io);
			return err;
		}
		touchIo_ = io;
		ESP_LOGI(kTag, "GT911 found at 0x%02x", address);
		return ESP_OK;
	}

	// Read all active fingers (up to kMaxFingers) and map each to viewport space.
	// The GT911 reports points in a stable slot order, so point index doubles as
	// the finger/pointer index.
	MultiSample readMulti()
	{
		MultiSample sample;
		if (!touch_) return sample;
		esp_lcd_touch_read_data(touch_);
		std::uint16_t xs[kMaxFingers] = {0, 0};
		std::uint16_t ys[kMaxFingers] = {0, 0};
		std::uint16_t strengths[kMaxFingers] = {0, 0};
		std::uint8_t count = 0;
		const bool pressed = esp_lcd_touch_get_coordinates(touch_, xs, ys, strengths, &count, kMaxFingers);
		if (!pressed || count == 0) return sample;
		sample.count = count > kMaxFingers ? kMaxFingers : static_cast<int>(count);
		for (int i = 0; i < sample.count; i++) {
			int mappedX = static_cast<int>(xs[i]);
			int mappedY = static_cast<int>(ys[i]);
			gea::framework::display::detail::DisplayOrientationState::mapPanelToViewport(&mappedX, &mappedY);
			sample.x[i] = mappedX;
			sample.y[i] = mappedY;
		}
		return sample;
	}

	// Single-point view (slot 0) for the imperative read()/readCached() API.
	TouchSample readHardware()
	{
		TouchSample sample = current_;
		const MultiSample multi = readMulti();
		if (multi.count == 0) {
			sample.touching = false;
			return sample;
		}
		sample.touching = true;
		sample.x = multi.x[0];
		sample.y = multi.y[0];
		return sample;
	}

	void pollLoop()
	{
		bool slotActive[kMaxFingers] = {false, false};
		int slotX[kMaxFingers] = {0, 0};
		int slotY[kMaxFingers] = {0, 0};
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
			const MultiSample multi = readMulti();

			// Keep the single-point cache (slot 0) live for read()/readCached().
			if (multi.count > 0) current_ = TouchSample{true, multi.x[0], multi.y[0]};
			else current_.touching = false;

			for (int slot = 0; slot < kMaxFingers; slot++) {
				const bool nowActive = slot < multi.count;
				const int x = nowActive ? multi.x[slot] : slotX[slot];
				const int y = nowActive ? multi.y[slot] : slotY[slot];
				const auto phaseFor = [&](gea::platform::touch::Phase phase, bool touching) {
					notifyPointer(phase, touching, x, y, slot);
				};

				if (nowActive && !slotActive[slot]) {
					if (slot == 0) latestMove_ = TouchSample{true, x, y};
					phaseFor(gea::platform::touch::Phase::Down, true);
				} else if (nowActive && slotActive[slot] && (x != slotX[slot] || y != slotY[slot])) {
					if (slot == 0) {
						// Primary move is coalesced via the latest-move cache (the
						// dispatcher reads it through consumeLatestMove); only queue
						// one in flight at a time.
						latestMove_ = TouchSample{true, x, y};
						if (!latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) {
							phaseFor(gea::platform::touch::Phase::Move, true);
						}
					} else {
						phaseFor(gea::platform::touch::Phase::Move, true);
					}
				} else if (!nowActive && slotActive[slot]) {
					phaseFor(gea::platform::touch::Phase::Up, false);
				}

				if (nowActive) {
					slotX[slot] = x;
					slotY[slot] = y;
				}
				slotActive[slot] = nowActive;
			}
		}
	}

	void notify(gea::platform::touch::Phase phase, const TouchSample &sample)
	{
		notifyPointer(phase, sample.touching, sample.x, sample.y, 0);
	}

	// Prefer the multi-finger observer when one is registered (it carries every
	// finger, including the primary as pointerId 0). Fall back to the single-point
	// observer for the primary only, so a consumer that registered just the old
	// Observer still works.
	void notifyPointer(gea::platform::touch::Phase phase, bool touching, int x, int y, int pointerId)
	{
		if (pointerObserver_) {
			pointerObserver_(phase, touching, x, y, pointerId);
		} else if (observer_ && pointerId == 0) {
			observer_(phase, touching, x, y);
		}
	}

	static void taskEntry(void *arg)
	{
		static_cast<Gt911TouchController *>(arg)->pollLoop();
	}

	gea::platform::touch::Touchscreen::Observer observer_ = nullptr;
	gea::platform::touch::Touchscreen::PointerObserver pointerObserver_ = nullptr;
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

Gt911TouchController &touchController()
{
	return Gt911TouchController::instance();
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer)
{
	touchController().setObserver(observer);
}

// Strong override of the weak no-op in touch_runtime.cpp: the GT911 supports
// simultaneous touches, so it delivers every finger here (pinch-to-zoom, etc.).
void gea::platform::touch::Touchscreen::setPointerObserver(PointerObserver observer)
{
	touchController().setPointerObserver(observer);
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
