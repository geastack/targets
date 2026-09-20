#include "rotary_encoder.h"

#include "board.h"
#include "input.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace gea::platform::elecrow {

namespace {

constexpr const char *kTag = "elecrow_rotary";
constexpr int kPollIntervalMs = 1;
constexpr int kTaskStackWords = 2048;
constexpr UBaseType_t kTaskPriority = 10;
constexpr int kTransitionsPerDetent = 2;

TaskHandle_t g_task = nullptr;
StaticTask_t g_taskControlBlock = {};
StackType_t g_taskStack[kTaskStackWords] = {};

int readState()
{
	const int a = gpio_get_level(gea::platform::board::rotary.phaseA) ? 1 : 0;
	const int b = gpio_get_level(gea::platform::board::rotary.phaseB) ? 1 : 0;
	return (a << 1) | b;
}

int transitionDelta(int previous, int current)
{
	static constexpr int8_t table[16] = {
		0, -1, 1, 0,
		1, 0, 0, -1,
		-1, 0, 0, 1,
		0, 1, -1, 0,
	};
	if (previous < 0 || previous > 3 || current < 0 || current > 3) return 0;
	return table[(previous << 2) | current];
}

void queueDetent(int detentDirection)
{
	gea::framework::input::queueRotaryDelta(-detentDirection);
}

void runTask(void *)
{
	int previous = readState();
	int partial = 0;
	while (true) {
		const int current = readState();
		if (current != previous) {
			const int step = transitionDelta(previous, current);
			if (step == 0) {
				partial = 0;
			} else {
				// On a direction reversal, drop any partial-detent credit left
				// over from the previous direction. The detent rest position is
				// offset by one transition from the fire threshold, so `partial`
				// settles at ±1 (not 0) between detents. Without this, the first
				// transition of a reversal only cancels that stale credit instead
				// of counting toward the new direction, so the first click after a
				// direction change never reaches the threshold and is lost ("needs
				// two clicks to reverse"). Resetting makes the reversal accumulate
				// cleanly from zero so the first detent registers immediately.
				if ((step > 0 && partial < 0) || (step < 0 && partial > 0)) partial = 0;
				partial += step;
				if (partial >= kTransitionsPerDetent) {
					partial = 0;
					queueDetent(1);
				} else if (partial <= -kTransitionsPerDetent) {
					partial = 0;
					queueDetent(-1);
				}
			}
			previous = current;
		}
		vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
	}
}

bool configurePin(gpio_num_t pin)
{
	gpio_reset_pin(pin);
	gpio_config_t config = {};
	config.pin_bit_mask = 1ULL << pin;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	const esp_err_t err = gpio_config(&config);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to configure GPIO%d: %s", static_cast<int>(pin), esp_err_to_name(err));
		return false;
	}
	return true;
}

}  // namespace

void startRotaryEncoderTask()
{
	if (g_task) return;
	if (!configurePin(gea::platform::board::rotary.phaseA) ||
	    !configurePin(gea::platform::board::rotary.phaseB)) {
		return;
	}

	g_task = xTaskCreateStatic(
		runTask,
		"elecrow-rotary",
		kTaskStackWords,
		nullptr,
		kTaskPriority,
		g_taskStack,
		&g_taskControlBlock);
	if (!g_task) {
		ESP_LOGE(kTag, "failed to start rotary task");
		return;
	}

	ESP_LOGI(kTag,
		"rotary encoder ready A=GPIO%d B=GPIO%d state=%d",
		static_cast<int>(gea::platform::board::rotary.phaseA),
		static_cast<int>(gea::platform::board::rotary.phaseB),
		readState());
}

}  // namespace gea::platform::elecrow
