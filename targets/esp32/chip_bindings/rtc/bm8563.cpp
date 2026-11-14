// BM8563 real-time clock on the board's primary I2C bus.
//
// The RTC keeps wall-clock time across power cycles (it has its own coin cell),
// so it is the source of the system clock at boot: restoreSystemTime() reads it
// and calls settimeofday() unless the chip reports it lost power. Every later
// wall-clock change made through gea::platform::clock::setEpochMs (the
// companion's SETTIME, an app setting the time) is written back to the chip,
// which overrides the host's settimeofday-only default.

#include "rtc/bm8563/bm8563.h"
#include "host/clock.h"
#include "i2c.h"

#include <cmath>
#include <ctime>
#include <sys/time.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

namespace gea::platform::rtc {
void restoreSystemTime();
}

namespace {

constexpr char kTag[] = "bm8563";
constexpr int kI2cTimeoutMs = 50;

class EspI2cRegisterBus final : public gea::chips::bm8563::RegisterBus {
public:
	bool attach() {
		if (device_) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;
		i2c_device_config_t config{};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = gea::chips::bm8563::kI2cAddress;
		config.scl_speed_hz = gea::chips::bm8563::kI2cFrequencyHz;
		return i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device_) ==
		       ESP_OK;
	}

	bool writeRegisters(std::uint8_t reg, const std::uint8_t *data, std::size_t length) override {
		std::uint8_t buffer[16];
		if (length + 1 > sizeof(buffer)) return false;
		buffer[0] = reg;
		for (std::size_t i = 0; i < length; ++i) buffer[i + 1] = data[i];
		return i2c_master_transmit(device_, buffer, length + 1, kI2cTimeoutMs) == ESP_OK;
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override {
		return i2c_master_transmit_receive(device_, &reg, 1, data, length, kI2cTimeoutMs) == ESP_OK;
	}

private:
	i2c_master_dev_handle_t device_ = nullptr;
};

EspI2cRegisterBus g_bus;
gea::chips::bm8563::RealTimeClock g_clock(g_bus);
bool g_ready = false;

bool ready() {
	if (g_ready) return true;
	if (!g_bus.attach()) {
		ESP_LOGW(kTag, "I2C bus unavailable; no RTC");
		return false;
	}
	if (!g_clock.begin()) {
		ESP_LOGW(kTag, "RTC did not answer at 0x%02x", gea::chips::bm8563::kI2cAddress);
		return false;
	}
	g_ready = true;
	return true;
}

// The RTC holds UTC, which is what the system clock (time_t) counts.
bool writeUtc(time_t seconds) {
	if (!ready()) return false;
	std::tm utc{};
	gmtime_r(&seconds, &utc);
	gea::chips::bm8563::DateTime time;
	time.year = utc.tm_year + 1900;
	time.month = utc.tm_mon + 1;
	time.day = utc.tm_mday;
	time.weekday = utc.tm_wday;
	time.hour = utc.tm_hour;
	time.minute = utc.tm_min;
	time.second = utc.tm_sec;
	return g_clock.setDateTime(time);
}

}  // namespace

void gea::platform::rtc::restoreSystemTime() {
	if (!ready()) return;
	gea::chips::bm8563::DateTime time;
	bool valid = false;
	if (!g_clock.readDateTime(time, valid)) {
		ESP_LOGW(kTag, "RTC read failed");
		return;
	}
	if (!valid) {
		ESP_LOGI(kTag, "RTC lost power; system clock left unset until the time is set");
		return;
	}
	std::tm utc{};
	utc.tm_year = time.year - 1900;
	utc.tm_mon = time.month - 1;
	utc.tm_mday = time.day;
	utc.tm_hour = time.hour;
	utc.tm_min = time.minute;
	utc.tm_sec = time.second;
	const time_t seconds = timegm(&utc);
	timeval now{seconds, 0};
	settimeofday(&now, nullptr);
	ESP_LOGI(kTag, "system clock restored from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC", time.year, time.month, time.day,
	         time.hour, time.minute, time.second);
}

bool gea::platform::clock::setEpochMs(double timestamp) {
	if (!std::isfinite(timestamp) || timestamp < 0) return false;
	const auto seconds = static_cast<time_t>(timestamp / 1000);
	timeval now{seconds, static_cast<suseconds_t>((timestamp - seconds * 1000.0) * 1000)};
	if (settimeofday(&now, nullptr) != 0) return false;
	if (!writeUtc(seconds)) ESP_LOGW(kTag, "system clock set, but the RTC write failed");
	return true;
}
