#include "touch.h"

#include "tab5_drivers.h"

#include <atomic>
#include <cstdint>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "tab5_touch";

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

class Tab5TouchController {
public:
	static Tab5TouchController &instance()
	{
		static Tab5TouchController controller;
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
		if (!gea::platform::tab5::initTouch()) {
			ESP_LOGE(kTag, "Tab5 touch init failed");
			return false;
		}
		if (xTaskCreatePinnedToCore(&Tab5TouchController::taskMain,
		                            "tab5_touch",
		                            4096,
		                            this,
		                            5,
		                            &task_,
		                            tskNO_AFFINITY) != pdPASS) {
			ESP_LOGE(kTag, "touch task start failed");
			return false;
		}
		initialized_ = true;
		ESP_LOGI(kTag, "touch ready");
		return true;
	}

	int read(int *x, int *y)
	{
		const TouchSample sample = current_;
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
	static void taskMain(void *arg)
	{
		static_cast<Tab5TouchController *>(arg)->pollLoop();
	}

	void pollLoop()
	{
		gea::platform::tab5::TouchSample sample{};
		for (;;) {
			if (current_.touching) {
				vTaskDelay(pdMS_TO_TICKS(16));
			} else {
				(void)gea::platform::tab5::waitForTouchReport(100);
			}
			if (gea::platform::tab5::readTouch(&sample)) handleTouch(sample);
		}
	}

	void handleTouch(const gea::platform::tab5::TouchSample &data)
	{
		TouchSample next = current_;
		next.touching = data.touching && data.points > 0;
		if (next.touching) {
			// Report panel-native coordinates. TouchRuntime owns the single
			// panel->logical orientation transform for queued events and move-cache
			// consumption; mapping here as well double-rotates taps on rotated UIs.
			next.x = static_cast<int>(data.x);
			next.y = static_cast<int>(data.y);
		}

		gea::platform::touch::Phase phase = gea::platform::touch::Phase::Move;
		if (next.touching && !current_.touching) {
			phase = gea::platform::touch::Phase::Down;
		} else if (!next.touching && current_.touching) {
			phase = gea::platform::touch::Phase::Up;
			next.x = current_.x;
			next.y = current_.y;
		} else if (!next.touching) {
			current_ = next;
			return;
		} else if (next.x == current_.x && next.y == current_.y) {
			current_ = next;
			return;
		}

		current_ = next;
		if (phase == gea::platform::touch::Phase::Move) {
			latestMove_ = next;
			if (latestMoveQueued_.exchange(true, std::memory_order_acq_rel)) return;
		}
		notify(phase, next);
	}

	void notify(gea::platform::touch::Phase phase, const TouchSample &sample)
	{
		if (pointerObserver_) {
			pointerObserver_(phase, sample.touching, sample.x, sample.y, 0);
		} else if (observer_) {
			observer_(phase, sample.touching, sample.x, sample.y);
		}
	}

	gea::platform::touch::Touchscreen::Observer observer_ = nullptr;
	gea::platform::touch::Touchscreen::PointerObserver pointerObserver_ = nullptr;
	TouchSample current_{};
	TouchSample latestMove_{};
	std::atomic<bool> latestMoveQueued_{false};
	TaskHandle_t task_ = nullptr;
	bool initialized_ = false;
};

Tab5TouchController &touchController()
{
	return Tab5TouchController::instance();
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer)
{
	touchController().setObserver(observer);
}

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
