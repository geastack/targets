#include "touch.h"

#include "board.h"
#include "i2c.h"
#include "pcf8574.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <cstdint>

namespace {

constexpr const char *kTag = "cst8xx";
constexpr TickType_t kI2cTimeout = pdMS_TO_TICKS(100);
constexpr int kPollIntervalMs = 10;
constexpr int kTaskPriority = 10;
constexpr std::size_t kTaskStackWords = 4096;
constexpr std::uint8_t kTouchCountReg = 0x02;
constexpr std::uint8_t kTouchDataReg = 0x03;
constexpr std::uint8_t kChipIdReg = 0xaa;

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

i2c_master_dev_handle_t g_device = nullptr;
TaskHandle_t g_task = nullptr;
StaticTask_t g_taskControlBlock = {};
StackType_t g_taskStack[kTaskStackWords] = {};
TouchSample g_current = {};
TouchSample g_latestMove = {};
std::atomic<bool> g_latestMoveQueued{false};
gea::platform::touch::Touchscreen::Observer g_observer = nullptr;

esp_err_t readRegister(std::uint8_t reg, std::uint8_t *data, std::size_t length)
{
	if (!g_device || !data || length == 0) return ESP_ERR_INVALID_ARG;
	return i2c_master_transmit_receive(g_device, &reg, 1, data, length, kI2cTimeout);
}

bool readHardware(TouchSample *sample)
{
	if (!sample) return false;

	std::uint8_t count = 0;
	if (readRegister(kTouchCountReg, &count, 1) != ESP_OK) {
		sample->touching = false;
		return false;
	}

	const int touches = count & 0x0f;
	if (touches <= 0) {
		sample->touching = false;
		return false;
	}

	std::uint8_t data[6] = {};
	if (readRegister(kTouchDataReg, data, sizeof(data)) != ESP_OK) {
		sample->touching = false;
		return false;
	}

	const int rawX = ((data[0] & 0x0f) << 8) | data[1];
	const int rawY = ((data[2] & 0x0f) << 8) | data[3];
	int x = rawX;
	int y = rawY;
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	if (x >= gea::platform::board::touch.maxX) x = gea::platform::board::touch.maxX - 1;
	if (y >= gea::platform::board::touch.maxY) y = gea::platform::board::touch.maxY - 1;

	sample->touching = true;
	sample->x = x;
	sample->y = y;
	return true;
}

void notifyObserver(gea::platform::touch::Phase phase, const TouchSample &sample)
{
	if (g_observer) g_observer(phase, sample.touching, sample.x, sample.y);
}

void notifyMove(const TouchSample &sample)
{
	g_latestMove = sample;
	if (g_latestMoveQueued.exchange(true, std::memory_order_acq_rel)) return;
	notifyObserver(gea::platform::touch::Phase::Move, sample);
}

void pollTask(void *)
{
	TouchSample last;
	while (true) {
		TouchSample next = last;
		const bool touching = readHardware(&next);
		if (!touching) {
			next.touching = false;
			next.x = last.x;
			next.y = last.y;
		}

		if (next.touching && !last.touching) {
			g_current = next;
			notifyObserver(gea::platform::touch::Phase::Down, next);
		} else if (!next.touching && last.touching) {
			g_current = next;
			notifyObserver(gea::platform::touch::Phase::Up, next);
		} else if (next.touching && (next.x != last.x || next.y != last.y)) {
			g_current = next;
			notifyMove(next);
		} else {
			g_current = next;
		}

		last = next;
		vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
	}
}

esp_err_t startTask()
{
	if (g_task) return ESP_OK;
	g_task = xTaskCreateStatic(pollTask,
	                           "cst8xx-touch",
	                           static_cast<std::uint32_t>(kTaskStackWords),
	                           nullptr,
	                           kTaskPriority,
	                           g_taskStack,
	                           &g_taskControlBlock);
	if (!g_task) {
		ESP_LOGE(kTag, "failed to start touch task");
		return ESP_ERR_NO_MEM;
	}
	return ESP_OK;
}

esp_err_t attachDevice()
{
	if (g_device) return ESP_OK;
	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return ESP_ERR_INVALID_STATE;

	i2c_device_config_t config = {};
	config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	config.device_address = gea::platform::board::touch.address;
	config.scl_speed_hz = 400000;

	const esp_err_t err = i2c_master_bus_add_device(
		static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()),
		&config,
		&g_device);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to add CST8xx at 0x%02x: %s",
		         gea::platform::board::touch.address,
		         esp_err_to_name(err));
		g_device = nullptr;
		return err;
	}
	return ESP_OK;
}

esp_err_t resetTouch()
{
	const int bit = gea::platform::board::expander.touchResetBit;
	esp_err_t err = gea::platform::elecrow::Pcf8574::writePin(bit, false);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(5));
	err = gea::platform::elecrow::Pcf8574::writePin(bit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));
	return ESP_OK;
}

}  // namespace

namespace gea::platform::touch {

void Touchscreen::setObserver(Observer observer)
{
	g_observer = observer;
}

bool Touchscreen::init()
{
	esp_err_t err = gea::platform::elecrow::Pcf8574::init();
	if (err != ESP_OK) return false;

	err = resetTouch();
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "touch reset failed: %s", esp_err_to_name(err));
		return false;
	}

	err = attachDevice();
	if (err != ESP_OK) return false;

	std::uint8_t chipId = 0;
	if (readRegister(kChipIdReg, &chipId, 1) == ESP_OK) {
		ESP_LOGI(kTag, "CST8xx ready at 0x%02x chip_id=0x%02x",
		         gea::platform::board::touch.address,
		         chipId);
	} else {
		ESP_LOGW(kTag, "CST8xx responded poorly to chip-id read; continuing");
	}

	return startTask() == ESP_OK;
}

int Touchscreen::read(int *x, int *y)
{
	TouchSample sample = g_current;
	readHardware(&sample);
	g_current = sample;
	if (x) *x = sample.x;
	if (y) *y = sample.y;
	return sample.touching ? 1 : 0;
}

int Touchscreen::readCached(int *x, int *y)
{
	const TouchSample sample = g_current;
	if (x) *x = sample.x;
	if (y) *y = sample.y;
	return sample.touching ? 1 : 0;
}

void Touchscreen::consumeLatestMove(int *x, int *y)
{
	g_latestMoveQueued.store(false, std::memory_order_release);
	if (x) *x = g_latestMove.x;
	if (y) *y = g_latestMove.y;
}

void Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
	TouchSample sample{touching, x, y};
	g_current = sample;
	g_latestMove = sample;
	g_latestMoveQueued.store(phase == Phase::Move, std::memory_order_release);
	notifyObserver(phase, sample);
}

}  // namespace gea::platform::touch
