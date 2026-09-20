#include "chip_bindings/gps/lc76g.h"

#include "board.h"
#include "geolocation.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

using gea::framework::geolocation::GeolocationAdapter;
using gea::framework::geolocation::GeolocationCoordinates;
using gea::framework::geolocation::GeolocationDriver;
using gea::framework::geolocation::GeolocationPosition;

constexpr char kTag[] = "lc76g";
constexpr auto kUartPort = gea::platform::board::gps.uartPort;
constexpr gpio_num_t kTxPin = gea::platform::board::gps.tx;
constexpr gpio_num_t kRxPin = gea::platform::board::gps.rx;
constexpr gpio_num_t kResetPin = gea::platform::board::gps.reset;
constexpr int kBaud = gea::platform::board::gps.baud;
constexpr int kRxBufferBytes = 4096;
constexpr int kTaskStackWords = 4096;
constexpr int kTaskPriority = 6;
constexpr double kKnotsToMetersPerSecond = 0.514444;
constexpr double kDefaultAccuracyMeters = 25.0;

double epochMs()
{
	return static_cast<double>(esp_timer_get_time()) / 1000.0;
}

double parseDouble(const char *text, double fallback = 0.0)
{
	if (!text || !*text) return fallback;
	char *end = nullptr;
	const double value = std::strtod(text, &end);
	return end && end != text ? value : fallback;
}

bool parseIntGreaterThanZero(const char *text)
{
	if (!text || !*text) return false;
	char *end = nullptr;
	const long value = std::strtol(text, &end, 10);
	return end && end != text && value > 0;
}

double parseCoordinate(const char *raw, const char *hemisphere)
{
	if (!raw || !*raw || !hemisphere || !*hemisphere) return 0.0;
	const double encoded = parseDouble(raw, 0.0);
	if (encoded <= 0.0) return 0.0;
	const int degrees = static_cast<int>(encoded / 100.0);
	const double minutes = encoded - static_cast<double>(degrees * 100);
	double value = static_cast<double>(degrees) + minutes / 60.0;
	if (hemisphere[0] == 'S' || hemisphere[0] == 'W') value = -value;
	return value;
}

int splitFields(char *line, char **fields, int cap)
{
	int count = 0;
	char *cursor = line;
	while (cursor && count < cap) {
		fields[count++] = cursor;
		char *comma = std::strchr(cursor, ',');
		if (!comma) break;
		*comma = '\0';
		cursor = comma + 1;
	}
	return count;
}

void stripChecksum(char *line)
{
	if (char *asterisk = std::strchr(line, '*')) *asterisk = '\0';
}

bool sentenceIs(const char *id, const char *suffix)
{
	if (!id || !suffix) return false;
	const std::size_t idLen = std::strlen(id);
	const std::size_t suffixLen = std::strlen(suffix);
	return idLen >= suffixLen && std::strcmp(id + idLen - suffixLen, suffix) == 0;
}

class Lc76gDriver final : public GeolocationDriver {
public:
	static Lc76gDriver &instance()
	{
		static Lc76gDriver driver;
		return driver;
	}

	bool init() override
	{
		if (initialized_) return true;
		pulseReset();

		uart_config_t config = {};
		config.baud_rate = kBaud;
		config.data_bits = UART_DATA_8_BITS;
		config.parity = UART_PARITY_DISABLE;
		config.stop_bits = UART_STOP_BITS_1;
		config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		config.source_clk = UART_SCLK_DEFAULT;

		esp_err_t err = uart_param_config(kUartPort, &config);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "UART param config failed: %s", esp_err_to_name(err));
			return false;
		}
		err = uart_set_pin(kUartPort, kTxPin, kRxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "UART pin config failed: %s", esp_err_to_name(err));
			return false;
		}
		err = uart_driver_install(kUartPort, kRxBufferBytes, 0, 0, nullptr, 0);
		if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
			ESP_LOGE(kTag, "UART driver install failed: %s", esp_err_to_name(err));
			return false;
		}

		task_ = xTaskCreateStatic(taskEntry, "lc76g-gps", kTaskStackWords, this, kTaskPriority, taskStack_, &taskControlBlock_);
		if (!task_) {
			ESP_LOGE(kTag, "Failed to start GPS task");
			return false;
		}

		initialized_ = true;
		ESP_LOGI(kTag, "GPS ready (UART%d TX=%d RX=%d RST=%d baud=%d)", static_cast<int>(kUartPort), static_cast<int>(kTxPin),
			static_cast<int>(kRxPin), static_cast<int>(kResetPin), kBaud);
		return true;
	}

	bool hasFix() const override
	{
		portENTER_CRITICAL(&lock_);
		const bool result = position_.hasFix;
		portEXIT_CRITICAL(&lock_);
		return result;
	}

	GeolocationPosition currentPosition() const override
	{
		portENTER_CRITICAL(&lock_);
		const GeolocationPosition position = position_;
		portEXIT_CRITICAL(&lock_);
		return position;
	}

private:
	void pulseReset()
	{
		if constexpr (static_cast<int>(kResetPin) >= 0) {
			gpio_config_t config = {};
			config.pin_bit_mask = 1ULL << static_cast<unsigned>(kResetPin);
			config.mode = GPIO_MODE_OUTPUT;
			if (gpio_config(&config) != ESP_OK) return;
			gpio_set_level(kResetPin, 0);
			vTaskDelay(pdMS_TO_TICKS(50));
			gpio_set_level(kResetPin, 1);
			vTaskDelay(pdMS_TO_TICKS(250));
		}
	}

	static void taskEntry(void *arg)
	{
		static_cast<Lc76gDriver *>(arg)->readLoop();
	}

	void readLoop()
	{
		uint8_t bytes[128];
		char line[160];
		int lineLen = 0;
		while (true) {
			const int count = uart_read_bytes(kUartPort, bytes, sizeof(bytes), pdMS_TO_TICKS(200));
			if (count <= 0) continue;
			for (int i = 0; i < count; i++) {
				const char ch = static_cast<char>(bytes[i]);
				if (ch == '\r') continue;
				if (ch == '\n') {
					if (lineLen > 0) {
						line[lineLen] = '\0';
						parseSentence(line);
						lineLen = 0;
					}
					continue;
				}
				if (lineLen < static_cast<int>(sizeof(line)) - 1) line[lineLen++] = ch;
				else lineLen = 0;
			}
		}
	}

	void parseSentence(char *line)
	{
		if (line[0] != '$') return;
		stripChecksum(line);
		char *fields[24] = {};
		const int count = splitFields(line + 1, fields, static_cast<int>(sizeof(fields) / sizeof(fields[0])));
		if (count <= 0) return;
		if (sentenceIs(fields[0], "RMC")) parseRmc(fields, count);
		else if (sentenceIs(fields[0], "GGA")) parseGga(fields, count);
	}

	void parseRmc(char **fields, int count)
	{
		if (count < 10) return;
		const bool valid = fields[2] && fields[2][0] == 'A';
		if (!valid) {
			setFix(false);
			return;
		}

		GeolocationPosition position = currentPosition();
		position.hasFix = true;
		position.timestamp = epochMs();
		position.coords.latitude = parseCoordinate(fields[3], fields[4]);
		position.coords.longitude = parseCoordinate(fields[5], fields[6]);
		position.coords.speed = parseDouble(fields[7], -1.0) * kKnotsToMetersPerSecond;
		position.coords.heading = parseDouble(fields[8], -1.0);
		if (position.coords.accuracy < 0.0) position.coords.accuracy = kDefaultAccuracyMeters;
		store(position);
	}

	void parseGga(char **fields, int count)
	{
		if (count < 10) return;
		if (!parseIntGreaterThanZero(fields[6])) {
			setFix(false);
			return;
		}

		GeolocationPosition position = currentPosition();
		position.hasFix = true;
		position.timestamp = epochMs();
		position.coords.latitude = parseCoordinate(fields[2], fields[3]);
		position.coords.longitude = parseCoordinate(fields[4], fields[5]);
		const double hdop = parseDouble(fields[8], 0.0);
		position.coords.accuracy = hdop > 0.0 ? hdop * 5.0 : kDefaultAccuracyMeters;
		position.coords.altitude = parseDouble(fields[9], position.coords.altitude);
		position.coords.altitudeAccuracy = position.coords.accuracy > 0.0 ? position.coords.accuracy * 1.5 : -1.0;
		store(position);
	}

	void setFix(bool hasFix)
	{
		portENTER_CRITICAL(&lock_);
		position_.hasFix = hasFix;
		position_.timestamp = epochMs();
		portEXIT_CRITICAL(&lock_);
	}

	void store(const GeolocationPosition &position)
	{
		const bool wasFixed = hasFix();
		portENTER_CRITICAL(&lock_);
		position_ = position;
		portEXIT_CRITICAL(&lock_);
		if (!wasFixed && position.hasFix) {
			ESP_LOGI(kTag, "GPS fix lat=%.6f lon=%.6f acc=%.1fm", position.coords.latitude, position.coords.longitude,
				position.coords.accuracy);
		}
	}

	bool initialized_ = false;
	TaskHandle_t task_ = nullptr;
	StaticTask_t taskControlBlock_ = {};
	StackType_t taskStack_[kTaskStackWords] = {};
	mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
	GeolocationPosition position_ = {};
};

}  // namespace

namespace gea::targets::esp32::gps {

void registerDriver()
{
	auto &driver = Lc76gDriver::instance();
	GeolocationAdapter::setDriver(&driver);
	(void)driver.init();
}

}  // namespace gea::targets::esp32::gps
