#include "services/app_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

	void lock()
	{
		if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
	}

	void unlock()
	{
		if (mutex_) xSemaphoreGive(mutex_);
	}

private:
	SemaphoreHandle_t mutex_ = nullptr;
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
