#include "lights.h"

#include <algorithm>
#include "board.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <cstdint>

namespace gea::platform::elecrow::lights {
namespace {
constexpr auto kTag = "crowpanel146_lights";
constexpr auto kMode = LEDC_LOW_SPEED_MODE;
constexpr auto kTimer = LEDC_TIMER_3;
constexpr auto kChannel = LEDC_CHANNEL_7;
}

void enableBoardPower()
{
  gpio_set_level(board::powerIndicator, 0);
  gpio_set_direction(board::powerIndicator, GPIO_MODE_OUTPUT);
  for (auto pin : {board::displayPower, board::panelPower, board::rgbPower}) {
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  }
}

void setBulbPercent(double percent)
{
  static const bool initialized = [] {
    ledc_timer_config_t timer{};
    timer.speed_mode = kMode;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.timer_num = kTimer;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t result = ledc_timer_config(&timer);
    if (result == ESP_OK) {
      ledc_channel_config_t channel{};
      channel.gpio_num = board::bulb;
      channel.speed_mode = kMode;
      channel.channel = kChannel;
      channel.timer_sel = kTimer;
      result = ledc_channel_config(&channel);
    }
    if (result != ESP_OK) ESP_LOGE(kTag, "Bulb PWM init failed: %s", esp_err_to_name(result));
    return result == ESP_OK;
  }();
  if (!initialized) return;
  const auto duty = static_cast<std::uint32_t>(std::clamp(percent, 0.0, 100.0) * 255 / 100);
  const auto result = ledc_set_duty_and_update(kMode, kChannel, duty, 0);
  if (result != ESP_OK) ESP_LOGE(kTag, "Bulb PWM update failed: %s", esp_err_to_name(result));
}
}
