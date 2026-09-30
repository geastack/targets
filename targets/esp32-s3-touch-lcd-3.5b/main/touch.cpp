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

constexpr char kTag[] = "axs15231b-touch";
constexpr std::uint8_t kAxs15231Address = 0x3b;
constexpr int kNativeWidth = 320;
constexpr int kNativeHeight = 480;
constexpr int kPollIntervalMs = 10;
#ifndef GEA_EMBEDDED_TOUCH_TASK_PRIORITY
#define GEA_EMBEDDED_TOUCH_TASK_PRIORITY 4
#endif
constexpr int kTaskPriority = GEA_EMBEDDED_TOUCH_TASK_PRIORITY;
constexpr std::uint32_t kTaskStackWords = 3072;

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
TouchSample gCurrent{};
TouchSample gLatestMove{};
std::atomic<bool> gLatestMoveQueued{false};
TaskHandle_t gTask = nullptr;
StaticTask_t gTaskControlBlock{};
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
EXT_RAM_BSS_ATTR
#endif
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

// AXS15231B has no wired interrupt/reset on this board. Poll both down and up.
HardwareSample readHardware()
{
    static constexpr std::uint8_t command[] = {
        0xb5, 0xab, 0xa5, 0x5a, 0, 0, 0, 0x0e, 0, 0, 0};
    std::uint8_t report[14]{};
    // The controller's command and report are separate transfers, with a STOP
    // between them (as in Espressif's AXS15231B driver).
    if (i2c_master_transmit(gDevice, command, sizeof(command), 20) != ESP_OK ||
        i2c_master_receive(gDevice, report, sizeof(report), 20) != ESP_OK) return {};
    if (report[0] == 0xff || report[1] > 2) return {};
    HardwareSample sample{.valid = true};
    // A lift packet can retain the last point/count. Bits 7:6 are the event:
    // 0 = down, 1 = up, 2 = contact, 3 = no event. Count alone leaves a lifted
    // finger permanently pressed, triggering long holds and trapping captures.
    const unsigned event = report[2] >> 6;
    if (report[1] == 0 || event == 1 || event == 3) return sample;
    sample.touch.x = ((report[2] & 0x0f) << 8) | report[3];
    sample.touch.y = ((report[4] & 0x0f) << 8) | report[5];
    if (sample.touch.x >= kNativeWidth || sample.touch.y >= kNativeHeight) return {};
    sample.touch.touching = true;
    return sample;
}

void pollLoop()
{
	bool active = false;
	while (true) {
		vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
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

esp_err_t attachDevice(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_ERROR(i2c_master_probe(bus, kAxs15231Address, 100), kTag, "touch probe");
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = kAxs15231Address;
    config.scl_speed_hz = 400000;
    return i2c_master_bus_add_device(bus, &config, &gDevice);
}

esp_err_t startTask()
{
    gTask = xTaskCreateStaticPinnedToCore(taskEntry, "axs15231b-touch", kTaskStackWords,
        nullptr, kTaskPriority, gTaskStack, &gTaskControlBlock, 1);
    return gTask ? ESP_OK : ESP_ERR_NO_MEM;
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
	if (gTask) return true;
	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return false;

	if (attachDevice(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle())) != ESP_OK) return false;
	if (startTask() != ESP_OK) return false;
	ESP_LOGI(kTag, "Physical touch ready (polled)");
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
