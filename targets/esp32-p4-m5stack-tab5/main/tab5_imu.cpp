#include "imu.h"
#include "tab5_drivers.h"

#include <atomic>
#include <cmath>
#include <mutex>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char *kTag = "tab5_imu";
constexpr double kStandardGravity = 9.80665;
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr int kSampleIntervalMs = 16;
constexpr std::uint32_t kSamplerStackBytes = 4096;
constexpr UBaseType_t kSamplerPriority = 3;

class Bmi270Binding {
public:
	static Bmi270Binding &instance()
	{
		static Bmi270Binding binding;
		return binding;
	}

	void init()
	{
		std::lock_guard<std::mutex> lock(initMutex_);
		ensureReadyLocked();
	}

	void close()
	{
		sampling_.store(false, std::memory_order_release);
		std::lock_guard<std::mutex> lock(initMutex_);
		initialized_ = false;
	}

	int tiltX()
	{
		return cachedTiltX();
	}

	int tiltY()
	{
		return cachedTiltY();
	}

	double accelerationX() { return cachedAcceleration().x * kStandardGravity; }
	double accelerationY() { return cachedAcceleration().y * kStandardGravity; }
	double accelerationZ() { return cachedAcceleration().z * kStandardGravity; }
	double gyroscopeX() { return cachedGyroscope().x; }
	double gyroscopeY() { return cachedGyroscope().y; }
	double gyroscopeZ() { return cachedGyroscope().z; }

private:
	bool ensureReadyLocked()
	{
		if (!initialized_) {
			initialized_ = gea::platform::tab5::initImu();
			if (!initialized_) return false;
			sampleOnce();
		}
		startSamplerLocked();
		return true;
	}

	void startSamplerLocked()
	{
		sampling_.store(true, std::memory_order_release);
		if (task_) return;
		if (xTaskCreatePinnedToCore(&Bmi270Binding::taskMain,
		                            "tab5_imu",
		                            kSamplerStackBytes,
		                            this,
		                            kSamplerPriority,
		                            &task_,
		                            tskNO_AFFINITY) != pdPASS) {
			task_ = nullptr;
			sampling_.store(false, std::memory_order_release);
			ESP_LOGW(kTag, "IMU sampler task start failed; falling back to synchronous reads");
		}
	}

	static void taskMain(void *arg)
	{
		static_cast<Bmi270Binding *>(arg)->sampleLoop();
	}

	static int tiltXFromAcceleration(const gea::platform::tab5::Vector3 &a)
	{
		return static_cast<int>(std::lround(std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z)) * kRadiansToDegrees));
	}

	static int tiltYFromAcceleration(const gea::platform::tab5::Vector3 &a)
	{
		return static_cast<int>(std::lround(std::atan2(a.y, a.z) * kRadiansToDegrees));
	}

	void sampleLoop()
	{
		while (sampling_.load(std::memory_order_acquire)) {
			sampleOnce();
			vTaskDelay(pdMS_TO_TICKS(kSampleIntervalMs));
		}

		{
			std::lock_guard<std::mutex> lock(initMutex_);
			task_ = nullptr;
		}
		vTaskDelete(nullptr);
	}

	void sampleOnce()
	{
		gea::platform::tab5::Vector3 acceleration{};
		gea::platform::tab5::Vector3 gyroscope{};
		if (!gea::platform::tab5::readImuMotion(&acceleration, &gyroscope)) return;
		const int tiltX = tiltXFromAcceleration(acceleration);
		const int tiltY = tiltYFromAcceleration(acceleration);
		std::lock_guard<std::mutex> lock(sampleMutex_);
		accelerationG_ = acceleration;
		gyroscopeDps_ = gyroscope;
		tiltX_ = tiltX;
		tiltY_ = tiltY;
		haveSample_ = true;
	}

	gea::platform::tab5::Vector3 cachedAcceleration()
	{
		{
			std::lock_guard<std::mutex> lock(initMutex_);
			if (!ensureReadyLocked()) return {};
		}
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return haveSample_ ? accelerationG_ : gea::platform::tab5::Vector3{};
	}

	gea::platform::tab5::Vector3 cachedGyroscope()
	{
		{
			std::lock_guard<std::mutex> lock(initMutex_);
			if (!ensureReadyLocked()) return {};
		}
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return haveSample_ ? gyroscopeDps_ : gea::platform::tab5::Vector3{};
	}

	int cachedTiltX()
	{
		{
			std::lock_guard<std::mutex> lock(initMutex_);
			if (!ensureReadyLocked()) return 0;
		}
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return haveSample_ ? tiltX_ : 0;
	}

	int cachedTiltY()
	{
		{
			std::lock_guard<std::mutex> lock(initMutex_);
			if (!ensureReadyLocked()) return 0;
		}
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return haveSample_ ? tiltY_ : 0;
	}

	std::mutex initMutex_;
	std::mutex sampleMutex_;
	TaskHandle_t task_ = nullptr;
	std::atomic<bool> sampling_{false};
	bool initialized_ = false;
	bool haveSample_ = false;
	gea::platform::tab5::Vector3 accelerationG_{};
	gea::platform::tab5::Vector3 gyroscopeDps_{};
	int tiltX_ = 0;
	int tiltY_ = 0;
};

}  // namespace

namespace gea::platform::sensors {

void Accelerometer::init()
{
	Bmi270Binding::instance().init();
}

void Accelerometer::close()
{
	Bmi270Binding::instance().close();
}

void Accelerometer::calibrateBias()
{
}

int Accelerometer::tiltX()
{
	return Bmi270Binding::instance().tiltX();
}

int Accelerometer::tiltY()
{
	return Bmi270Binding::instance().tiltY();
}

double Accelerometer::accelerationX()
{
	return Bmi270Binding::instance().accelerationX();
}

double Accelerometer::accelerationY()
{
	return Bmi270Binding::instance().accelerationY();
}

double Accelerometer::accelerationZ()
{
	return Bmi270Binding::instance().accelerationZ();
}

double Accelerometer::gyroscopeX()
{
	return Bmi270Binding::instance().gyroscopeX();
}

double Accelerometer::gyroscopeY()
{
	return Bmi270Binding::instance().gyroscopeY();
}

double Accelerometer::gyroscopeZ()
{
	return Bmi270Binding::instance().gyroscopeZ();
}

void Accelerometer::setWebTilt(int, int)
{
}

}  // namespace gea::platform::sensors
