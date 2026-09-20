#include "imu.h"

#include "i2c.h"
#include "imu/qmi8658/qmi8658.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <mutex>

namespace sensors = gea::platform::sensors;

namespace {

constexpr char kTag[] = "qmi8658";
constexpr int kI2cTimeoutMs = 100;
constexpr int64_t kRetryIntervalUs = 30000000;
constexpr double kStandardGravity = 9.80665;
constexpr std::uint8_t kProbeAddresses[] = {
	gea::chips::qmi8658::kI2cAddress,
	0x6A,
};

class EspI2cRegisterBus final : public gea::chips::qmi8658::RegisterBus {
public:
	void setAddress(std::uint8_t address) {
		if (address_ == address) return;
		detach();
		address_ = address;
	}

	std::uint8_t address() const { return address_; }

	bool attach() {
		if (device_) return true;

		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus not available");
			return false;
		}

		i2c_device_config_t config{};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = address_;
		config.scl_speed_hz = gea::chips::qmi8658::kI2cFrequencyHz;
		const esp_err_t err = i2c_master_bus_add_device(
		    static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()),
		    &config,
		    &device_);
		if (err != ESP_OK) ESP_LOGE(kTag, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
		return err == ESP_OK;
	}

	void detach() {
		if (!device_) return;
		i2c_master_bus_rm_device(device_);
		device_ = nullptr;
	}

	bool writeRegister(std::uint8_t reg, std::uint8_t value) override {
		if (!attach()) return false;
		std::uint8_t buffer[2] = {reg, value};
		return i2c_master_transmit(device_, buffer, sizeof(buffer), kI2cTimeoutMs) == ESP_OK;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override {
		if (!attach()) return false;
		return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, kI2cTimeoutMs) == ESP_OK;
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override {
		if (!attach()) return false;
		return i2c_master_transmit_receive(device_, &reg, 1, data, length, kI2cTimeoutMs) == ESP_OK;
	}

private:
	i2c_master_dev_handle_t device_ = nullptr;
	std::uint8_t address_ = gea::chips::qmi8658::kI2cAddress;
};

class FreeRtosDelay final : public gea::chips::qmi8658::Delay {
public:
	void milliseconds(int ms) override {
		vTaskDelay(pdMS_TO_TICKS(ms));
	}
};

class Qmi8658Binding {
public:
	static Qmi8658Binding &instance() {
		static Qmi8658Binding binding;
		return binding;
	}

	// All public accessors lock mutex_: the QMI8658 is now read from TWO tasks — the gea
	// app (air-mouse) and the BLE IMU-notify task (ble_hid.cpp) — and the underlying driver
	// reads I2C into a shared buffer per call, so concurrent unlocked access would tear it.
	void init() {
		std::lock_guard<std::mutex> lock(mutex_);
		ensureInitialized();
	}

	void close() {
		std::lock_guard<std::mutex> lock(mutex_);
		driver_.close();
		bus_.detach();
		initialized_ = false;
		nextRetryUs_ = 0;
	}

	void calibrateBias() { std::lock_guard<std::mutex> lock(mutex_); if (ensureInitialized()) driver_.calibrateBias(); }
	int tiltX() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.tiltX() : 0; }
	int tiltY() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.tiltY() : 0; }
	double accelerationX() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.accelerationX() : 0.0; }
	double accelerationY() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.accelerationY() : 0.0; }
	double accelerationZ() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.accelerationZ() : kStandardGravity; }
	double gyroscopeX() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.gyroscopeX() : 0.0; }
	double gyroscopeY() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.gyroscopeY() : 0.0; }
	double gyroscopeZ() { std::lock_guard<std::mutex> lock(mutex_); return ensureInitialized() ? driver_.gyroscopeZ() : 0.0; }

private:
	Qmi8658Binding() : driver_(bus_, delay_) {}

	bool ensureInitialized() {
		if (initialized_) return true;

		const int64_t nowUs = esp_timer_get_time();
		if (nextRetryUs_ > 0 && nowUs < nextRetryUs_) return false;
		nextRetryUs_ = nowUs + kRetryIntervalUs;

		std::uint8_t lastWhoAmI = 0;
		for (std::uint8_t address : kProbeAddresses) {
			bus_.setAddress(address);
			driver_.close();
			if (driver_.begin()) {
				initialized_ = true;
				ESP_LOGI(kTag, "QMI8658 detected at 0x%02x (WHO_AM_I=0x%02x)", bus_.address(), driver_.lastWhoAmI());
				ESP_LOGI(kTag, "QMI8658 initialized");
				return true;
			}
			lastWhoAmI = driver_.lastWhoAmI();
		}

		ESP_LOGW(kTag, "QMI8658 not found (last WHO_AM_I=0x%02x; tried 0x%02x/0x%02x)",
		         lastWhoAmI,
		         kProbeAddresses[0],
		         kProbeAddresses[1]);
		return false;
	}

	EspI2cRegisterBus bus_;
	FreeRtosDelay delay_;
	gea::chips::qmi8658::Driver driver_;
	bool initialized_ = false;
	int64_t nextRetryUs_ = 0;
	std::mutex mutex_;
};

}  // namespace

void sensors::Accelerometer::init() {
	Qmi8658Binding::instance().init();
}

void sensors::Accelerometer::close() {
	Qmi8658Binding::instance().close();
}

void sensors::Accelerometer::calibrateBias() {
	Qmi8658Binding::instance().calibrateBias();
}

int sensors::Accelerometer::tiltX() {
	return Qmi8658Binding::instance().tiltX();
}

int sensors::Accelerometer::tiltY() {
	return Qmi8658Binding::instance().tiltY();
}

double sensors::Accelerometer::accelerationX() {
	return Qmi8658Binding::instance().accelerationX();
}

double sensors::Accelerometer::accelerationY() {
	return Qmi8658Binding::instance().accelerationY();
}

double sensors::Accelerometer::accelerationZ() {
	return Qmi8658Binding::instance().accelerationZ();
}

double sensors::Accelerometer::gyroscopeX() {
	return Qmi8658Binding::instance().gyroscopeX();
}

double sensors::Accelerometer::gyroscopeY() {
	return Qmi8658Binding::instance().gyroscopeY();
}

double sensors::Accelerometer::gyroscopeZ() {
	return Qmi8658Binding::instance().gyroscopeZ();
}

void sensors::Accelerometer::setWebTilt(int x, int y) {
	(void)x;
	(void)y;
}
