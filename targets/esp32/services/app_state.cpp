#include "services/app_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <atomic>

namespace gea::framework::services {

namespace {

class AppStateRuntime {
public:
	static AppStateRuntime &instance()
	{
		static AppStateRuntime runtime;
		return runtime;
	}

	bool init()
	{
		if (mutex_) return true;
		mutex_ = xSemaphoreCreateMutex();
		return mutex_ != nullptr;
	}

	// The frame task holds the lock for a whole frame and takes it again
	// straight after releasing it. A FreeRTOS mutex does not hand itself to a
	// waiter on give, so while the app renders continuously (a blinking caret is
	// enough) a lower-priority waiter -- the GEADEV console reading the tree or
	// the screen -- went ~30 s without it. The task that just released steps
	// aside for one tick when someone is waiting; nobody waits on a normal frame.
	void lock()
	{
		if (!mutex_) return;
		if (waiting_.load(std::memory_order_acquire) > 0 && lastReleaser_ == xTaskGetCurrentTaskHandle())
			vTaskDelay(1);
		waiting_.fetch_add(1, std::memory_order_acq_rel);
		xSemaphoreTake(mutex_, portMAX_DELAY);
		waiting_.fetch_sub(1, std::memory_order_acq_rel);
	}

	void unlock()
	{
		if (!mutex_) return;
		lastReleaser_ = xTaskGetCurrentTaskHandle();
		xSemaphoreGive(mutex_);
	}

private:
	SemaphoreHandle_t mutex_ = nullptr;
	std::atomic<int> waiting_{0};
	TaskHandle_t lastReleaser_ = nullptr;
};

}  // namespace

bool AppState::init()
{
	return AppStateRuntime::instance().init();
}

void AppState::lock()
{
	AppStateRuntime::instance().lock();
}

void AppState::unlock()
{
	AppStateRuntime::instance().unlock();
}

}  // namespace gea::framework::services
