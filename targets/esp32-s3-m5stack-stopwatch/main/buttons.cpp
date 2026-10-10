// SPDX-License-Identifier: Apache-2.0
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input.h"

namespace {

TaskHandle_t buttonTask = nullptr;

void pollButtons(void*) {
  int previous = 0;
  int rawPrevious = 0;
  int stable = 0;
  int64_t chordAt = 0;
  bool chordSent = false;
  for (;;) {
    const int raw = (!gpio_get_level(GPIO_NUM_2) ? 1 : 0) | (!gpio_get_level(GPIO_NUM_1) ? 2 : 0);
    stable = raw == rawPrevious ? stable + 1 : 0;
    rawPrevious = raw;
    if (stable >= 2) {
      const auto now = esp_timer_get_time();
      if (raw == 3) {
        if (previous != 3) {
          chordAt = now;
        }
        if (!chordSent && now - chordAt >= 500000) {
          gea::framework::input::queueKeyDown(27);  // Escape / factory home chord.
          chordSent = true;
        }
      }
      for (int mask = 1; mask <= 2; mask <<= 1) {
        const int code = mask == 1 ? 37 : 39;
        if ((raw & mask) && !(previous & mask) && !chordSent) {
          gea::framework::input::queueKeyDown(code);
        }
        if (!(raw & mask) && (previous & mask)) {
          gea::framework::input::queueKeyUp(code);
        }
      }
      if (raw == 0) {
        chordSent = false;
      }
      previous = raw;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

}  // namespace

namespace gea::platform::board {

void startButtons() {
  if (buttonTask) {
    return;
  }
  gpio_config_t config{};
  config.pin_bit_mask = (1ULL << GPIO_NUM_1) | (1ULL << GPIO_NUM_2);
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&config));
  if (xTaskCreate(pollButtons, "stopwatch_keys", 3072, nullptr, 10, &buttonTask) != pdPASS) {
    ESP_LOGE("stopwatch_keys", "Failed to create button task");
    return;
  }
  ESP_LOGI("stopwatch_keys", "A(GPIO2)=ArrowLeft B(GPIO1)=ArrowRight A+B(500ms)=Escape");
}

}  // namespace gea::platform::board

namespace gea::platform::esp32::apps {

void startLauncherButtonTask() { gea::platform::board::startButtons(); }

}  // namespace gea::platform::esp32::apps
