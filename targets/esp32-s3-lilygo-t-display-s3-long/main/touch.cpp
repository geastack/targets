#include "board.h"
#include "i2c.h"
#include "touch.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

using Phase = gea::platform::touch::Phase;
using Touchscreen = gea::platform::touch::Touchscreen;

constexpr char kTag[] = "t-display-touch";
constexpr std::uint8_t kCst3530Address = 0x58;
constexpr std::uint8_t kAxs15231Address = 0x3b;
constexpr int kNativeWidth = 180;
constexpr int kNativeHeight = 640;
constexpr int kPollIntervalMs = 10;
constexpr int kTaskPriority = 4;
constexpr std::uint32_t kTaskStackWords = 3072;

enum class Controller : std::uint8_t {
	None,
	Cst3530,
	Axs15231,
};

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

struct HardwareSample {
	bool valid = false;
	TouchSample touch{};
};

Touchscreen::Observer gObserver = nullptr;
Touchscreen::PointerObserver gPointerObserver = nullptr;
i2c_master_dev_handle_t gDevice = nullptr;
Controller gController = Controller::None;
TouchSample gCurrent{};
TouchSample gLatestMove{};
std::atomic<bool> gLatestMoveQueued{false};
TaskHandle_t gTask = nullptr;
StaticTask_t gTaskControlBlock{};
StackType_t gTaskStack[kTaskStackWords]{};

void notify(Phase phase, const TouchSample &sample)
{
	if (gPointerObserver) {
		gPointerObserver(phase, sample.touching, sample.x, sample.y, 0);
	} else if (gObserver) {
		gObserver(phase, sample.touching, sample.x, sample.y);
	}
}

void notifyMove(const TouchSample &sample)
{
	gLatestMove = sample;
	if (gLatestMoveQueued.exchange(true, std::memory_order_acq_rel)) return;
	notify(Phase::Move, sample);
}

esp_err_t transmit(const std::uint8_t *bytes, std::size_t size)
{
	return i2c_master_transmit(gDevice, bytes, size, 100);
}

HardwareSample readHardware()
{
	std::uint8_t report[11]{};
	const std::uint8_t *command = nullptr;
	std::size_t commandSize = 0;
	std::size_t reportSize = 0;
	static constexpr std::uint8_t cstCommand[] = {0xd0, 0x07, 0x00, 0x00};
	static constexpr std::uint8_t axsCommand[] = {
		0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
	};

	if (gController == Controller::Cst3530) {
		command = cstCommand;
		commandSize = sizeof(cstCommand);
		reportSize = 9;
	} else if (gController == Controller::Axs15231) {
		command = axsCommand;
		commandSize = sizeof(axsCommand);
		reportSize = 8;
	} else {
		return {};
	}

	esp_err_t status = ESP_FAIL;
	for (int attempt = 0; attempt < 2; ++attempt) {
		status = i2c_master_transmit(gDevice, command, commandSize, 20);
		if (status == ESP_OK) status = i2c_master_receive(gDevice, report, reportSize, 20);
		if (status == ESP_OK) break;
	}
	if (status != ESP_OK) return {};

	HardwareSample sample{.valid = true};
	if (gController == Controller::Cst3530) {
		const std::uint16_t checksum = static_cast<std::uint16_t>(
			0x55U + report[4] + report[5] + report[6] + report[7] + report[8]);
		const std::uint16_t expected = static_cast<std::uint16_t>(report[0] | (report[1] << 8U));
		const std::uint8_t ack[] = {0xd0, 0x00, 0x02, 0xab};
		(void)transmit(ack, sizeof(ack));
		const std::uint8_t fingerCount = report[3] & 0x0fU;
		if (report[2] != 0xffU || fingerCount > 1U) return {};
		if (fingerCount == 0U || (report[8] >> 4U) == 0U) return sample;
		if (checksum != expected) return {};
		sample.touch.touching = true;
		sample.touch.x = report[4] | ((report[7] & 0x0f) << 8);
		sample.touch.y = report[5] | ((report[7] & 0xf0) << 4);
	} else {
		const std::uint8_t fingerCount = report[1];
		const std::uint8_t touchEvent = report[2] >> 4U;
		if (fingerCount != 1U || touchEvent != 0x08U) return sample;
		sample.touch.touching = true;
		sample.touch.x = ((report[4] & 0x0f) << 8) | report[5];
		sample.touch.y = kNativeHeight - (((report[2] & 0x0f) << 8) | report[3]);
	}

	if (sample.touch.x < 0 || sample.touch.x >= kNativeWidth ||
	    sample.touch.y < 0 || sample.touch.y >= kNativeHeight) {
		return {};
	}
	return sample;
}

void pollLoop()
{
	bool active = false;
	while (true) {
		const TickType_t wait = active ? pdMS_TO_TICKS(kPollIntervalMs) : portMAX_DELAY;
		(void)ulTaskNotifyTake(pdTRUE, wait);
		const HardwareSample result = readHardware();
		if (!result.valid) continue;

		const TouchSample next = result.touch.touching
			? result.touch
			: TouchSample{false, gCurrent.x, gCurrent.y};
		if (next.touching && !active) {
			gLatestMove = next;
			notify(Phase::Down, next);
		} else if (next.touching && active &&
		           (next.x != gCurrent.x || next.y != gCurrent.y)) {
			notifyMove(next);
		} else if (!next.touching && active) {
			notify(Phase::Up, next);
		}
		gCurrent = next;
		active = next.touching;
	}
}

void taskEntry(void *)
{
	pollLoop();
}

void IRAM_ATTR interruptEntry(void *)
{
	BaseType_t wake = pdFALSE;
	if (gTask) vTaskNotifyGiveFromISR(gTask, &wake);
	portYIELD_FROM_ISR(wake);
}

esp_err_t resetController()
{
	constexpr gpio_num_t reset = gea::platform::board::touch.reset;
	gpio_config_t config{};
	config.pin_bit_mask = 1ULL << static_cast<unsigned>(reset);
	config.mode = GPIO_MODE_OUTPUT;
	ESP_RETURN_ON_ERROR(gpio_config(&config), kTag, "touch reset GPIO");
	gpio_set_level(reset, 0);
	vTaskDelay(pdMS_TO_TICKS(8));
	gpio_set_level(reset, 1);
	vTaskDelay(pdMS_TO_TICKS(50));
	return ESP_OK;
}

esp_err_t attachDevice(i2c_master_bus_handle_t bus)
{
	std::uint8_t address = 0;
	if (i2c_master_probe(bus, kCst3530Address, 100) == ESP_OK) {
		gController = Controller::Cst3530;
		address = kCst3530Address;
	} else if (i2c_master_probe(bus, kAxs15231Address, 100) == ESP_OK) {
		gController = Controller::Axs15231;
		address = kAxs15231Address;
	} else {
		return ESP_ERR_NOT_FOUND;
	}

	i2c_device_config_t config{};
	config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	config.device_address = address;
	config.scl_speed_hz = 200000;
	ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &gDevice), kTag, "touch device");

	if (gController == Controller::Cst3530) {
		static constexpr std::uint8_t commands[][4] = {
			{0xd0, 0x00, 0x04, 0x00}, {0xd0, 0x00, 0x04, 0x00},
			{0xd0, 0x00, 0x00, 0x00}, {0xd0, 0x00, 0x0c, 0x00},
			{0xd0, 0x00, 0x01, 0x00},
		};
		for (const auto &command : commands) {
			(void)transmit(command, sizeof(command));
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}
	ESP_LOGI(kTag, "Detected %s touch controller at 0x%02x",
	         gController == Controller::Cst3530 ? "CST3530" : "AXS15231", address);
	return ESP_OK;
}

esp_err_t startTaskAndInterrupt()
{
	gTask = xTaskCreateStaticPinnedToCore(taskEntry, "t-display-touch", kTaskStackWords,
	                                      nullptr, kTaskPriority, gTaskStack,
	                                      &gTaskControlBlock, 1);
	if (!gTask) return ESP_ERR_NO_MEM;

	constexpr gpio_num_t interrupt = gea::platform::board::touch.interrupt;
	gpio_config_t config{};
	config.pin_bit_mask = 1ULL << static_cast<unsigned>(interrupt);
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.intr_type = GPIO_INTR_NEGEDGE;
	ESP_RETURN_ON_ERROR(gpio_config(&config), kTag, "touch interrupt GPIO");
	esp_err_t status = gpio_install_isr_service(0);
	if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) return status;
	ESP_RETURN_ON_ERROR(gpio_isr_handler_add(interrupt, interruptEntry, nullptr),
	                    kTag, "touch interrupt handler");
	if (gpio_get_level(interrupt) == 0) xTaskNotifyGive(gTask);
	return ESP_OK;
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer)
{
	gObserver = observer;
}

void gea::platform::touch::Touchscreen::setPointerObserver(PointerObserver observer)
{
	gPointerObserver = observer;
}

bool gea::platform::touch::Touchscreen::init()
{
	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return false;
	if (resetController() != ESP_OK) return false;
	if (attachDevice(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle())) != ESP_OK) return false;
	if (startTaskAndInterrupt() != ESP_OK) return false;
	ESP_LOGI(kTag, "Physical touch ready (interrupt-driven)");
	return true;
}

int gea::platform::touch::Touchscreen::read(int *x, int *y)
{
	if (x) *x = gCurrent.x;
	if (y) *y = gCurrent.y;
	return gCurrent.touching ? 1 : 0;
}

int gea::platform::touch::Touchscreen::readCached(int *x, int *y)
{
	return read(x, y);
}

void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y)
{
	gLatestMoveQueued.store(false, std::memory_order_release);
	if (x) *x = gLatestMove.x;
	if (y) *y = gLatestMove.y;
}

void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
	const TouchSample sample{touching, x, y};
	gCurrent = sample;
	if (phase == Phase::Move) notifyMove(sample);
	else notify(phase, sample);
}
