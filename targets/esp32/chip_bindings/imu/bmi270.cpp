// BMI270 6-axis IMU binding (accelerometer + gyroscope) for the M5StickC S3.
// Provides gea::platform::sensors::Accelerometer on the shared primary I2C bus.
//
// The BMI270 (I2C 0x68, CHIP_ID reg 0x00 == 0x24) has an on-chip controller that
// produces NO data until Bosch's ~8KB config firmware is uploaded at startup, so
// init = soft-reset -> upload config -> wait for INTERNAL_STATUS ready -> enable
// accel/gyro. Data registers are little-endian. The config blob is vendored in
// bmi270_config.inl (see its header for provenance). Register map/sequence follow
// M5Stack's M5Unified BMI270 driver.

#include "imu.h"

#include "i2c.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>
#include <cstring>
#include <mutex>

namespace sensors = gea::platform::sensors;

namespace {

#include "bmi270_config.inl"  // static constexpr const uint8_t bmi270_config_file[8192]

constexpr char kTag[] = "bmi270";
constexpr std::uint8_t kI2cAddress = 0x68;
constexpr std::uint32_t kI2cFrequencyHz = 400000;
constexpr int kI2cTimeoutMs = 100;
constexpr int64_t kRetryIntervalUs = 5000000;
constexpr double kStandardGravity = 9.80665;

// BMI270 registers.
constexpr std::uint8_t kRegChipId = 0x00;
constexpr std::uint8_t kRegAccData = 0x0C;       // ACC_X_LSB, 6 bytes little-endian
constexpr std::uint8_t kRegGyrData = 0x12;       // GYR_X_LSB, 6 bytes little-endian
constexpr std::uint8_t kRegInternalStatus = 0x21;
constexpr std::uint8_t kRegAccConf = 0x40;
constexpr std::uint8_t kRegAccRange = 0x41;
constexpr std::uint8_t kRegGyrConf = 0x42;
constexpr std::uint8_t kRegGyrRange = 0x43;
constexpr std::uint8_t kRegInitCtrl = 0x59;
constexpr std::uint8_t kRegInitAddr0 = 0x5B;     // + INIT_ADDR_1 at 0x5C
constexpr std::uint8_t kRegInitData = 0x5E;
constexpr std::uint8_t kRegPwrConf = 0x7C;
constexpr std::uint8_t kRegPwrCtrl = 0x7D;
constexpr std::uint8_t kRegCmd = 0x7E;
constexpr std::uint8_t kChipIdBmi270 = 0x24;
constexpr std::uint8_t kCmdSoftReset = 0xB6;

// ±2 g / ±2000 dps full-scale (gravity fits, finest tilt resolution).
constexpr double kAccelGPerLsb = 2.0 / 32768.0;
constexpr double kGyroDpsPerLsb = 2000.0 / 32768.0;

class Bmi270Binding {
public:
	static Bmi270Binding &instance() {
		static Bmi270Binding binding;
		return binding;
	}

	void init() {
		std::lock_guard<std::mutex> lock(mutex_);
		ensureInitialized();
	}

	void close() {
		std::lock_guard<std::mutex> lock(mutex_);
		detach();
		initialized_ = false;
		nextRetryUs_ = 0;
	}

	// Store the current pose as "flat" so the level tool centres in whatever
	// orientation the stick is held.
	void calibrateBias() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		if (readAccelG(ax, ay, az)) {
			biasX_ = ax;
			biasY_ = ay;
		}
	}

	int tiltX() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		return readAccelG(ax, ay, az) ? tiltFromAxis(ay - biasY_) : 0;
	}
	int tiltY() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		return readAccelG(ax, ay, az) ? tiltFromAxis(-(ax - biasX_)) : 0;
	}
	double accelerationX() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		return readAccelG(ax, ay, az) ? ax * kStandardGravity : 0.0;
	}
	double accelerationY() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		return readAccelG(ax, ay, az) ? ay * kStandardGravity : 0.0;
	}
	double accelerationZ() {
		std::lock_guard<std::mutex> lock(mutex_);
		double ax = 0, ay = 0, az = 0;
		return readAccelG(ax, ay, az) ? az * kStandardGravity : kStandardGravity;
	}
	double gyroscopeX() {
		std::lock_guard<std::mutex> lock(mutex_);
		double gx = 0, gy = 0, gz = 0;
		return readGyroDps(gx, gy, gz) ? gx : 0.0;
	}
	double gyroscopeY() {
		std::lock_guard<std::mutex> lock(mutex_);
		double gx = 0, gy = 0, gz = 0;
		return readGyroDps(gx, gy, gz) ? gy : 0.0;
	}
	double gyroscopeZ() {
		std::lock_guard<std::mutex> lock(mutex_);
		double gx = 0, gy = 0, gz = 0;
		return readGyroDps(gx, gy, gz) ? gz : 0.0;
	}

private:
	static int tiltFromAxis(double axis) {
		int value = static_cast<int>(axis * 70.0);
		if (value < -100) return -100;
		if (value > 100) return 100;
		return value;
	}

	bool attach() {
		if (device_) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus not available");
			return false;
		}
		i2c_device_config_t config{};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = kI2cAddress;
		config.scl_speed_hz = kI2cFrequencyHz;
		const esp_err_t err = i2c_master_bus_add_device(
		    static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device_);
		if (err != ESP_OK) ESP_LOGE(kTag, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
		return err == ESP_OK;
	}

	void detach() {
		if (!device_) return;
		i2c_master_bus_rm_device(device_);
		device_ = nullptr;
	}

	bool writeReg(std::uint8_t reg, std::uint8_t value) {
		std::uint8_t buffer[2] = {reg, value};
		return i2c_master_transmit(device_, buffer, sizeof(buffer), kI2cTimeoutMs) == ESP_OK;
	}

	// Register write of up to 256 payload bytes ([reg][data...] in one transaction).
	bool writeBuf(std::uint8_t reg, const std::uint8_t *data, std::size_t length) {
		if (length > 256) return false;
		std::uint8_t buffer[257];
		buffer[0] = reg;
		std::memcpy(&buffer[1], data, length);
		return i2c_master_transmit(device_, buffer, length + 1, kI2cTimeoutMs) == ESP_OK;
	}

	bool readRegs(std::uint8_t reg, std::uint8_t *data, std::size_t length) {
		return i2c_master_transmit_receive(device_, &reg, 1, data, length, kI2cTimeoutMs) == ESP_OK;
	}

	// Upload the config firmware in 256-byte chunks; a single 8KB burst would
	// exceed the I2C transaction timeout. INIT_ADDR_0/1 carry the word offset.
	bool uploadConfig() {
		const std::size_t total = sizeof(bmi270_config_file);
		constexpr std::size_t kChunk = 256;
		for (std::size_t offset = 0; offset < total; offset += kChunk) {
			const std::size_t length = (offset + kChunk <= total) ? kChunk : (total - offset);
			const std::size_t word = offset >> 1;
			std::uint8_t addr[2] = {static_cast<std::uint8_t>(word & 0x0F), static_cast<std::uint8_t>(word >> 4)};
			if (!writeBuf(kRegInitAddr0, addr, 2)) return false;
			if (!writeBuf(kRegInitData, &bmi270_config_file[offset], length)) return false;
		}
		return true;
	}

	bool ensureInitialized() {
		if (initialized_) return true;
		const int64_t nowUs = esp_timer_get_time();
		if (nextRetryUs_ > 0 && nowUs < nextRetryUs_) return false;
		nextRetryUs_ = nowUs + kRetryIntervalUs;

		if (!attach()) return false;

		std::uint8_t chipId = 0;
		if (!readRegs(kRegChipId, &chipId, 1)) {
			ESP_LOGW(kTag, "CHIP_ID read failed at 0x%02x", kI2cAddress);
			return false;
		}
		if (chipId != kChipIdBmi270) {
			ESP_LOGW(kTag, "unexpected CHIP_ID=0x%02x at 0x%02x (expected BMI270 0x24)", chipId, kI2cAddress);
			return false;
		}

		writeReg(kRegCmd, kCmdSoftReset);
		vTaskDelay(pdMS_TO_TICKS(5));
		writeReg(kRegPwrConf, 0x00);  // disable advanced power save before config load
		vTaskDelay(pdMS_TO_TICKS(2));
		writeReg(kRegInitCtrl, 0x00);  // prepare config load
		if (!uploadConfig()) {
			ESP_LOGW(kTag, "config upload failed");
			return false;
		}
		writeReg(kRegInitCtrl, 0x01);  // config load complete
		vTaskDelay(pdMS_TO_TICKS(25));

		std::uint8_t status = 0;
		int tries = 20;
		do {
			readRegs(kRegInternalStatus, &status, 1);
			if ((status & 0x0F) == 0x01) break;
			vTaskDelay(pdMS_TO_TICKS(2));
		} while (--tries);
		if ((status & 0x0F) != 0x01) {
			ESP_LOGW(kTag, "init not ready (INTERNAL_STATUS=0x%02x)", status);
			return false;
		}

		writeReg(kRegPwrCtrl, 0x0E);   // temp_en | acc_en | gyr_en
		writeReg(kRegAccConf, 0xA8);   // ODR 100 Hz, normal BWP, perf filter
		writeReg(kRegAccRange, 0x00);  // ±2 g
		writeReg(kRegGyrConf, 0xA9);   // ODR 200 Hz, normal BWP, perf filter
		writeReg(kRegGyrRange, 0x00);  // ±2000 dps
		writeReg(kRegPwrConf, 0x02);   // FIFO self-wakeup, adv power save off
		vTaskDelay(pdMS_TO_TICKS(10));

		initialized_ = true;
		ESP_LOGI(kTag, "BMI270 initialized at 0x%02x (CHIP_ID=0x%02x, INTERNAL_STATUS=0x%02x)", kI2cAddress, chipId, status);
		return true;
	}

	static double leSigned(std::uint8_t low, std::uint8_t high) {
		return static_cast<double>(static_cast<std::int16_t>((high << 8) | low));
	}

	bool readAccelG(double &ax, double &ay, double &az) {
		if (!ensureInitialized()) return false;
		std::uint8_t raw[6] = {0};
		if (!readRegs(kRegAccData, raw, sizeof(raw))) return false;
		ax = leSigned(raw[0], raw[1]) * kAccelGPerLsb;
		ay = leSigned(raw[2], raw[3]) * kAccelGPerLsb;
		az = leSigned(raw[4], raw[5]) * kAccelGPerLsb;
		return true;
	}

	bool readGyroDps(double &gx, double &gy, double &gz) {
		if (!ensureInitialized()) return false;
		std::uint8_t raw[6] = {0};
		if (!readRegs(kRegGyrData, raw, sizeof(raw))) return false;
		gx = leSigned(raw[0], raw[1]) * kGyroDpsPerLsb;
		gy = leSigned(raw[2], raw[3]) * kGyroDpsPerLsb;
		gz = leSigned(raw[4], raw[5]) * kGyroDpsPerLsb;
		return true;
	}

	i2c_master_dev_handle_t device_ = nullptr;
	bool initialized_ = false;
	int64_t nextRetryUs_ = 0;
	double biasX_ = 0;
	double biasY_ = 0;
	std::mutex mutex_;
};

}  // namespace

void sensors::Accelerometer::init() { Bmi270Binding::instance().init(); }
void sensors::Accelerometer::close() { Bmi270Binding::instance().close(); }
void sensors::Accelerometer::calibrateBias() { Bmi270Binding::instance().calibrateBias(); }
int sensors::Accelerometer::tiltX() { return Bmi270Binding::instance().tiltX(); }
int sensors::Accelerometer::tiltY() { return Bmi270Binding::instance().tiltY(); }
double sensors::Accelerometer::accelerationX() { return Bmi270Binding::instance().accelerationX(); }
double sensors::Accelerometer::accelerationY() { return Bmi270Binding::instance().accelerationY(); }
double sensors::Accelerometer::accelerationZ() { return Bmi270Binding::instance().accelerationZ(); }
double sensors::Accelerometer::gyroscopeX() { return Bmi270Binding::instance().gyroscopeX(); }
double sensors::Accelerometer::gyroscopeY() { return Bmi270Binding::instance().gyroscopeY(); }
double sensors::Accelerometer::gyroscopeZ() { return Bmi270Binding::instance().gyroscopeZ(); }
void sensors::Accelerometer::setWebTilt(int x, int y) {
	(void)x;
	(void)y;
}
