#include "board.h"
#include "chip_bindings/expanders/io_expander.h"
#include "i2c.h"
#include "touch.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

using Phase = gea::platform::touch::Phase;
using Touchscreen = gea::platform::touch::Touchscreen;

constexpr char kTag[] = "axs15231b-touch";
constexpr std::uint8_t kAxs15231Address = 0x3b;
constexpr int kNativeWidth = 172;
constexpr int kNativeHeight = 640;
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

struct Contact {
	int id = 0;
	int x = 0;
	int y = 0;
};

// One controller report: up to two contacts, each with the controller's own id.
struct HardwareSample {
	bool valid = false;
	int count = 0;
	Contact contacts[2]{};

	const Contact *find(int id) const
	{
		for (int i = 0; i < count; ++i)
			if (contacts[i].id == id) return &contacts[i];
		return nullptr;
	}
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

HardwareSample readHardware()
{
    static constexpr std::uint8_t command[] = {
        0xb5, 0xab, 0xa5, 0x5a, 0, 0, 0, 0x0e, 0, 0, 0};
    std::uint8_t report[14]{};
    // Command and report in one transaction with a repeated start, at the
    // vendor's 300 kHz -- exactly as Waveshare's V2 examples read this panel.
    if (i2c_master_transmit_receive(gDevice, command, sizeof(command), report, sizeof(report), 20) != ESP_OK)
        return {};
    if (report[0] == 0xff || report[1] > 2) return {};
    HardwareSample sample{.valid = true};
    // report[1] is the contact count; each contact is six bytes from report[2]:
    //   [0] event (bits 7:6: 0 down, 1 up, 2 contact, 3 none) | along 11:8
    //   [1] along 7:0
    //   [2] contact id (bits 7:4) | across 11:8
    //   [3] across 7:0
    //   [4..5] report sequence
    // The controller keeps contacts in fixed slots, so the first slot is not
    // necessarily the finger that went down first; contacts are matched by id.
    for (int i = 0; i < report[1]; ++i) {
        const std::uint8_t *point = report + 2 + 6 * i;
        const unsigned event = point[0] >> 6;
        if (event == 1 || event == 3) continue;
        // The controller reports along the glass's long axis first: its X
        // runs down the 640-pixel side from the bottom, its Y across the
        // 172-pixel side. The vendor example maps it the same way into native
        // portrait.
        const int along = ((point[0] & 0x0f) << 8) | point[1];
        const int across = ((point[2] & 0x0f) << 8) | point[3];
        // Edge reports can land one past the glass; clamp them as the vendor does.
        if (along > kNativeHeight + 8 || across > kNativeWidth + 8) continue;
        Contact &contact = sample.contacts[sample.count++];
        contact.id = point[2] >> 4;
        contact.x = std::min(across, kNativeWidth - 1);
        contact.y = std::clamp(kNativeHeight - along, 0, kNativeHeight - 1);
    }
    return sample;
}

#ifndef GEA_EMBEDDED_TOUCH_POLL
#define GEA_EMBEDDED_TOUCH_POLL 0
#endif
// The controller's INT reaches the ESP32 through the expander, whose own INT
// (GPIO8) falls whenever one of its inputs changes. Reads follow that line:
// the controller is read when it signals, and while a finger is down it is
// also read every kPollIntervalMs so a lift is seen even if its edge is
// shared with a change the expander already reported. With no finger down
// the task sleeps until the next edge. GEA_EMBEDDED_TOUCH_POLL=1 opts back
// into reading on a fixed interval.
constexpr gpio_num_t kInterruptPin = gea::platform::board::touch.interrupt;
constexpr bool kInterruptDriven = kInterruptPin != GPIO_NUM_NC && !GEA_EMBEDDED_TOUCH_POLL;

void IRAM_ATTR expanderInterrupt(void *)
{
	BaseType_t woken = pdFALSE;
	if (gTask) vTaskNotifyGiveFromISR(gTask, &woken);
	if (woken) portYIELD_FROM_ISR();
}

// Reading the expander's input port releases its INT, so the next change
// produces a fresh falling edge.
void acknowledgeExpander()
{
	bool level = true;
	gea::platform::esp32::chip_bindings::expanders::ioExpander().readPin(
		gea::platform::board::touch.expanderInterruptPin, level);
}

// The primary pointer follows one contact id from its down to its lift; a
// second contact never takes it over mid-gesture. A contact that is still
// present when the primary lifts belongs to that gesture and cannot start a new
// one until it lifts too -- otherwise a second contact left on the glass turns
// every lift into a fresh tap at its position.
void pollLoop()
{
	int primaryId = -1;
	unsigned staleIds = 0;
	bool anyContact = false;
	if constexpr (kInterruptDriven) acknowledgeExpander();
	while (true) {
		if constexpr (kInterruptDriven) {
			const TickType_t wait = anyContact ? pdMS_TO_TICKS(kPollIntervalMs) : portMAX_DELAY;
			if (ulTaskNotifyTake(pdTRUE, wait) > 0) acknowledgeExpander();
		} else {
			vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
		}
		const HardwareSample result = readHardware();
		if (!result.valid) continue;
		anyContact = result.count > 0;

		unsigned present = 0;
		for (int i = 0; i < result.count; ++i) present |= 1u << result.contacts[i].id;
		staleIds &= present;

		if (primaryId >= 0) {
			const Contact *contact = result.find(primaryId);
			if (contact) {
				if (contact->x != gCurrent.x || contact->y != gCurrent.y) {
					const TouchSample next{true, contact->x, contact->y};
					notifyMove(next);
					gCurrent = next;
				}
				continue;
			}
			const TouchSample lifted{false, gCurrent.x, gCurrent.y};
			notify(Phase::Up, lifted);
			gCurrent = lifted;
			primaryId = -1;
			staleIds = present;
		}

		for (int i = 0; i < result.count; ++i) {
			const Contact &contact = result.contacts[i];
			if (staleIds & (1u << contact.id)) continue;
			primaryId = contact.id;
			const TouchSample next{true, contact.x, contact.y};
			gLatestMove = next;
			notify(Phase::Down, next);
			gCurrent = next;
			break;
		}
	}
}

esp_err_t attachInterrupt()
{
	if constexpr (!kInterruptDriven) return ESP_OK;
	if (!gea::platform::esp32::chip_bindings::expanders::ioExpander().init()) {
		ESP_LOGE(kTag, "expander unavailable for the touch interrupt");
		return ESP_FAIL;
	}
	gpio_config_t config{};
	config.pin_bit_mask = 1ULL << kInterruptPin;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.intr_type = GPIO_INTR_NEGEDGE;
	ESP_RETURN_ON_ERROR(gpio_config(&config), kTag, "touch interrupt pin");
	const esp_err_t service = gpio_install_isr_service(0);
	if (service != ESP_OK && service != ESP_ERR_INVALID_STATE) return service;
	return gpio_isr_handler_add(kInterruptPin, expanderInterrupt, nullptr);
}

void taskEntry(void *)
{
	pollLoop();
}

// Touch sits on its own I2C pair, apart from the sensor/codec bus that
// Bus::primary() owns.
i2c_master_bus_handle_t touchBus()
{
	static i2c_master_bus_handle_t bus = nullptr;
	if (bus) return bus;
	i2c_master_bus_config_t config{};
	config.i2c_port = -1;
	config.sda_io_num = gea::platform::board::touch.bus.sda;
	config.scl_io_num = gea::platform::board::touch.bus.scl;
	config.clk_source = I2C_CLK_SRC_DEFAULT;
	config.glitch_ignore_cnt = 7;
	config.flags.enable_internal_pullup = true;
	if (i2c_new_master_bus(&config, &bus) != ESP_OK) {
		ESP_LOGE(kTag, "touch I2C bus init failed");
		bus = nullptr;
	}
	return bus;
}

esp_err_t attachDevice(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_ERROR(i2c_master_probe(bus, kAxs15231Address, 100), kTag, "touch probe");
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = kAxs15231Address;
    config.scl_speed_hz = 300000;
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
	i2c_master_bus_handle_t bus = touchBus();
	if (!bus) return false;

	if (attachDevice(bus) != ESP_OK) return false;
	if (startTask() != ESP_OK) return false;
	// The task is created first so an edge arriving during setup has a target.
	if (attachInterrupt() != ESP_OK) return false;
	ESP_LOGI(kTag, "Physical touch ready (%s)", kInterruptDriven ? "interrupt" : "polling");
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
