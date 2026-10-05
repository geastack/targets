#include "power.h"
#include "board.h"
#include <algorithm>
#include <mutex>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

namespace gea::platform::power {
namespace {
std::mutex mutex;
adc_oneshot_unit_handle_t adc = nullptr;
adc_cali_handle_t calibration = nullptr;
bool ready = false;
}
bool Power::init() {
  std::lock_guard<std::mutex> lock(mutex);
  if (ready) return true;
  // Hold the battery soft latch before releasing the physical PWR button.
  gpio_config_t output{};
  output.mode = GPIO_MODE_OUTPUT;
  output.pin_bit_mask = 1ULL << board::power.enable;
  if (gpio_config(&output) != ESP_OK || gpio_set_level(board::power.enable, 1) != ESP_OK) return false;
  gpio_config_t input{};
  input.mode = GPIO_MODE_INPUT;
  input.pin_bit_mask = 1ULL << board::power.charging;
  input.pull_up_en = GPIO_PULLUP_ENABLE;
  if (gpio_config(&input) != ESP_OK) return false;
  adc_oneshot_unit_init_cfg_t unit{};
  unit.unit_id = ADC_UNIT_1;
  if (!adc && adc_oneshot_new_unit(&unit, &adc) != ESP_OK) return false;
  adc_oneshot_chan_cfg_t channel{};
  channel.atten = ADC_ATTEN_DB_12;
  channel.bitwidth = ADC_BITWIDTH_DEFAULT;
  if (adc_oneshot_config_channel(adc, ADC_CHANNEL_0, &channel) != ESP_OK) return false;
  adc_cali_curve_fitting_config_t cal{};
  cal.unit_id = ADC_UNIT_1;
  cal.chan = ADC_CHANNEL_0;
  cal.atten = ADC_ATTEN_DB_12;
  cal.bitwidth = ADC_BITWIDTH_DEFAULT;
  if (adc_cali_create_scheme_curve_fitting(&cal, &calibration) != ESP_OK) {
    calibration = nullptr;
    ESP_LOGW("lcd154_power", "ADC calibration unavailable; battery percentage unknown");
  }
  ready = true;
  return true;
}
int Power::batteryPercent() {
  if (!init()) return -1;
  std::lock_guard<std::mutex> lock(mutex);
  if (!calibration) return -1;
  int total = 0;
  for (int i = 0; i < 8; ++i) {
    int raw = 0;
    if (adc_oneshot_read(adc, ADC_CHANNEL_0, &raw) != ESP_OK) return -1;
    total += raw;
  }
  int mv = 0;
  if (adc_cali_raw_to_voltage(calibration, total / 8, &mv) != ESP_OK) return -1;
  // Vendor divider: Vbat = ADC voltage * 3. Percentage is a voltage estimate.
  mv *= 3;
  return std::clamp((mv - 3300) * 100 / 900, 0, 100);
}
bool Power::charging() {
  return init() && gpio_get_level(board::power.charging) == 0;
}
}
