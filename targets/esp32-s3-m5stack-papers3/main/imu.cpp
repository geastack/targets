// M5PaperS3 IMU — Bosch BMI270 accelerometer + gyroscope on the shared I2C bus.
//
// The register sequence, scaling and configuration payload are the ones proven
// on the M5Stack Tab5 target (targets/esp32-p4-m5stack-tab5/main/tab5_drivers.cpp),
// which carries the same part. The BMI270 will not produce valid data until the
// configuration file below has been uploaded once after reset.

#include "imu.h"

#include "board.h"
#include "i2c.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char kTag[] = "papers3_imu";

constexpr double kStandardGravity = 9.80665;
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr int kSampleIntervalMs = 16;
// Bring-up telemetry: one reading per second on the serial log. Axis mapping and
// the tiltX sign are confirmed on hardware, so this is off; flip it on to
// re-verify orientation (e.g. when fixing tiltY for a pitch-using app).
constexpr bool kLogSamples = false;

// The part answers at 0x68 with SDO low and 0x69 with SDO high. M5 does not
// document which strap this board uses, so probe both and log the winner
// rather than hardcoding a guess.
constexpr std::uint8_t kAddressPrimary = 0x68;
constexpr std::uint8_t kAddressSecondary = 0x69;

constexpr std::uint8_t kRegChipId = 0x00;
constexpr std::uint8_t kRegInternalStatus = 0x21;
constexpr std::uint8_t kRegAccX = 0x0c;
constexpr std::uint8_t kRegAccConf = 0x40;
constexpr std::uint8_t kRegAccRange = 0x41;
constexpr std::uint8_t kRegGyrConf = 0x42;
constexpr std::uint8_t kRegGyrRange = 0x43;
constexpr std::uint8_t kRegInitCtrl = 0x59;
constexpr std::uint8_t kRegInitAddr0 = 0x5b;
constexpr std::uint8_t kRegInitData = 0x5e;
constexpr std::uint8_t kRegPwrConf = 0x7c;
constexpr std::uint8_t kRegPwrCtrl = 0x7d;
constexpr std::uint8_t kRegCmd = 0x7e;

constexpr std::uint8_t kChipId = 0x24;
constexpr std::uint8_t kSoftReset = 0xb6;
constexpr std::uint8_t kInternalStatusInitOk = 0x01;
constexpr std::uint8_t kAccConf100HzNormal = 0xa8;
constexpr std::uint8_t kAccRange4g = 0x01;
constexpr std::uint8_t kGyrConf100HzNormal = 0xe8;
constexpr std::uint8_t kGyrRange2000Dps = 0x00;
constexpr std::uint8_t kPowerAccelGyroTemp = 0x0e;
constexpr std::size_t kConfigChunkSize = 32;
constexpr int kTimeoutMs = 250;

// Accelerometer at +/-4 g -> 8192 LSB/g; gyroscope at +/-2000 dps -> 16.4 LSB/dps.
constexpr double kAccelLsbPerG = 8192.0;
constexpr double kGyroLsbPerDps = 16.4;

// Bosch Sensortec BMI270 SensorAPI v2.86.1 maximum-FIFO configuration file.
constexpr std::uint8_t kConfigFile[] = {
	0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
	0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
	0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5, 0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00,
	0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22, 0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f,
	0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50,
	0x00, 0x30, 0x12, 0x24, 0xeb, 0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f,
	0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5, 0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80,
	0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41, 0x33, 0x98, 0x2e, 0xc2, 0xc4,
	0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24, 0x00, 0xfc,
	0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe,
	0x94, 0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2,
	0xfe, 0x82, 0x05, 0x2f, 0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e,
	0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20, 0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a,
	0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08, 0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42,
	0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0, 0x6f, 0x00, 0x2e,
	0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
	0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30,
	0x23, 0x2e, 0x61, 0xf5, 0xeb, 0x2c, 0xe1, 0x6f,
};
static_assert(sizeof(kConfigFile) == 328);

struct Vector3 {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

std::int16_t le16s(const std::uint8_t *p)
{
	return static_cast<std::int16_t>(static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8));
}

class Bmi270 {
public:
	static Bmi270 &instance()
	{
		static Bmi270 imu;
		return imu;
	}

	void init()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!ensureReadyLocked()) return;
		startSamplerLocked();
	}

	void close()
	{
		sampling_.store(false, std::memory_order_release);
		std::lock_guard<std::mutex> lock(mutex_);
		ready_ = false;
	}

	void calibrateBias()
	{
		// Treat the current orientation as level.
		const Vector3 a = acceleration();
		biasTiltX_.store(tiltXFrom(a), std::memory_order_relaxed);
		biasTiltY_.store(tiltYFrom(a), std::memory_order_relaxed);
	}

	int tiltX() { return tiltXFrom(acceleration()) - biasTiltX_.load(std::memory_order_relaxed); }
	int tiltY() { return tiltYFrom(acceleration()) - biasTiltY_.load(std::memory_order_relaxed); }

	Vector3 acceleration()
	{
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return acceleration_;
	}

	Vector3 gyroscope()
	{
		std::lock_guard<std::mutex> lock(sampleMutex_);
		return gyroscope_;
	}

private:
	// The panel is physically landscape (960x540) but apps run portrait (rotated
	// 90°), so the BMI270's chip X/Y are swapped relative to the SCREEN X/Y.
	// Screen X (the paddle axis) is therefore the chip's Y axis, and screen Y is
	// the chip's X axis. Both signs are verified on hardware: with `-a.y` the
	// paddle tracked the wrong way, so screen X uses `+a.y`.
	static int tiltXFrom(const Vector3 &a)
	{
		return static_cast<int>(std::lround(std::atan2(a.y, std::sqrt(a.x * a.x + a.z * a.z)) * kRadiansToDegrees));
	}

	// NOTE: this game only uses tiltX (the paddle). tiltY's sign/offset is NOT
	// yet hardware-verified — flat currently reads ~±180° because the chip is
	// mounted z-down. Fix it against a real pitch-using app before relying on it.
	static int tiltYFrom(const Vector3 &a)
	{
		return static_cast<int>(std::lround(std::atan2(a.x, a.z) * kRadiansToDegrees));
	}

	bool attach(std::uint8_t address)
	{
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;
		i2c_device_config_t config = {};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = address;
		config.scl_speed_hz = 400000;
		if (i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device_) != ESP_OK) {
			device_ = nullptr;
			return false;
		}
		std::uint8_t id = 0;
		if (!readReg(kRegChipId, &id, 1) || id != kChipId) {
			i2c_master_bus_rm_device(device_);
			device_ = nullptr;
			return false;
		}
		address_ = address;
		return true;
	}

	bool ensureReadyLocked()
	{
		if (ready_) return true;
		if (!device_ && !attach(kAddressPrimary) && !attach(kAddressSecondary)) {
			ESP_LOGW(kTag, "BMI270 not found at 0x%02x or 0x%02x; tilt reports zero",
			         kAddressPrimary, kAddressSecondary);
			return false;
		}

		if (!writeReg(kRegCmd, kSoftReset)) return false;
		vTaskDelay(pdMS_TO_TICKS(3));
		if (!writeReg(kRegPwrConf, 0x00)) return false;
		vTaskDelay(pdMS_TO_TICKS(1));
		if (!uploadConfig()) {
			std::uint8_t status = 0;
			(void)readReg(kRegInternalStatus, &status, 1);
			ESP_LOGW(kTag, "BMI270 config upload failed: internal_status=0x%02x", status);
			return false;
		}
		if (!writeReg(kRegPwrCtrl, kPowerAccelGyroTemp)) return false;
		vTaskDelay(pdMS_TO_TICKS(50));
		if (!writeReg(kRegAccConf, kAccConf100HzNormal)) return false;
		if (!writeReg(kRegAccRange, kAccRange4g)) return false;
		if (!writeReg(kRegGyrConf, kGyrConf100HzNormal)) return false;
		if (!writeReg(kRegGyrRange, kGyrRange2000Dps)) return false;

		ready_ = true;
		ESP_LOGI(kTag, "BMI270 ready at 0x%02x (SDA=%d SCL=%d)", address_,
		         static_cast<int>(gea::platform::board::i2c.sda), static_cast<int>(gea::platform::board::i2c.scl));
		sampleOnce();
		return true;
	}

	bool uploadConfig()
	{
		if (!writeReg(kRegInitCtrl, 0x00)) return false;
		for (std::size_t offset = 0; offset < sizeof(kConfigFile); offset += kConfigChunkSize) {
			const std::size_t chunk = std::min<std::size_t>(sizeof(kConfigFile) - offset, kConfigChunkSize);
			const std::uint16_t wordOffset = static_cast<std::uint16_t>(offset / 2);
			const std::uint8_t addressBytes[] = {
				kRegInitAddr0,
				static_cast<std::uint8_t>(wordOffset & 0x0f),
				static_cast<std::uint8_t>(wordOffset >> 4),
			};
			if (!write(addressBytes, sizeof(addressBytes))) return false;
			std::uint8_t payload[kConfigChunkSize + 1] = {kRegInitData};
			std::memcpy(payload + 1, kConfigFile + offset, chunk);
			if (!write(payload, chunk + 1)) return false;
		}
		if (!writeReg(kRegInitCtrl, 0x01)) return false;
		for (int i = 0; i < 20; ++i) {
			vTaskDelay(pdMS_TO_TICKS(10));
			std::uint8_t status = 0;
			if (!readReg(kRegInternalStatus, &status, 1)) return false;
			if ((status & 0x0f) == kInternalStatusInitOk) return true;
		}
		return false;
	}

	void startSamplerLocked()
	{
		sampling_.store(true, std::memory_order_release);
		if (task_) return;
		if (xTaskCreate(&Bmi270::taskMain, "papers3_imu", 4096, this, 3, &task_) != pdPASS) {
			task_ = nullptr;
			sampling_.store(false, std::memory_order_release);
			ESP_LOGW(kTag, "IMU sampler task start failed");
		}
	}

	static void taskMain(void *arg) { static_cast<Bmi270 *>(arg)->sampleLoop(); }

	void sampleLoop()
	{
		while (sampling_.load(std::memory_order_acquire)) {
			sampleOnce();
			vTaskDelay(pdMS_TO_TICKS(kSampleIntervalMs));
		}
		task_ = nullptr;
		vTaskDelete(nullptr);
	}

	void sampleOnce()
	{
		// Accel and gyro registers are contiguous, so one burst gets both.
		std::uint8_t data[12] = {};
		if (!readReg(kRegAccX, data, sizeof(data))) {
			if (++readFailures_ == 1 || (readFailures_ % 256) == 0)
				ESP_LOGW(kTag, "BMI270 read failed (%u)", static_cast<unsigned>(readFailures_));
			return;
		}
		if (kLogSamples && (++sampleCount_ % 64) == 0) {
			// One line per ~1s. Tilt the board while watching this: if the
			// numbers do not move, the sampler or the bus is the problem; if
			// they move but the paddle does not, the axis mapping is.
			ESP_LOGI(kTag, "accel g x=%.2f y=%.2f z=%.2f  tilt x=%d y=%d",
			         acceleration_.x, acceleration_.y, acceleration_.z,
			         tiltXFrom(acceleration_), tiltYFrom(acceleration_));
		}
		std::lock_guard<std::mutex> lock(sampleMutex_);
		acceleration_.x = static_cast<double>(le16s(&data[0])) / kAccelLsbPerG;
		acceleration_.y = static_cast<double>(le16s(&data[2])) / kAccelLsbPerG;
		acceleration_.z = static_cast<double>(le16s(&data[4])) / kAccelLsbPerG;
		gyroscope_.x = static_cast<double>(le16s(&data[6])) / kGyroLsbPerDps;
		gyroscope_.y = static_cast<double>(le16s(&data[8])) / kGyroLsbPerDps;
		gyroscope_.z = static_cast<double>(le16s(&data[10])) / kGyroLsbPerDps;
	}

	bool write(const std::uint8_t *data, std::size_t size)
	{
		if (!device_) return false;
		return i2c_master_transmit(device_, data, size, kTimeoutMs) == ESP_OK;
	}

	bool writeReg(std::uint8_t reg, std::uint8_t value)
	{
		const std::uint8_t payload[] = {reg, value};
		return write(payload, sizeof(payload));
	}

	bool readReg(std::uint8_t reg, std::uint8_t *out, std::size_t size)
	{
		if (!device_) return false;
		return i2c_master_transmit_receive(device_, &reg, 1, out, size, kTimeoutMs) == ESP_OK;
	}

	i2c_master_dev_handle_t device_ = nullptr;
	std::uint8_t address_ = 0;
	bool ready_ = false;
	TaskHandle_t task_ = nullptr;
	std::atomic<bool> sampling_{false};
	std::atomic<int> biasTiltX_{0};
	std::atomic<int> biasTiltY_{0};
	std::uint32_t sampleCount_ = 0;
	std::uint32_t readFailures_ = 0;
	std::mutex mutex_;
	std::mutex sampleMutex_;
	Vector3 acceleration_{};
	Vector3 gyroscope_{};
};

}  // namespace

namespace gea::platform::sensors {

void Accelerometer::init() { Bmi270::instance().init(); }
void Accelerometer::close() { Bmi270::instance().close(); }
void Accelerometer::calibrateBias() { Bmi270::instance().calibrateBias(); }
int Accelerometer::tiltX() { return Bmi270::instance().tiltX(); }
int Accelerometer::tiltY() { return Bmi270::instance().tiltY(); }
double Accelerometer::accelerationX() { return Bmi270::instance().acceleration().x * kStandardGravity; }
double Accelerometer::accelerationY() { return Bmi270::instance().acceleration().y * kStandardGravity; }
double Accelerometer::accelerationZ() { return Bmi270::instance().acceleration().z * kStandardGravity; }
double Accelerometer::gyroscopeX() { return Bmi270::instance().gyroscope().x; }
double Accelerometer::gyroscopeY() { return Bmi270::instance().gyroscope().y; }
double Accelerometer::gyroscopeZ() { return Bmi270::instance().gyroscope().z; }
void Accelerometer::setWebTilt(int, int) {}

}  // namespace gea::platform::sensors
