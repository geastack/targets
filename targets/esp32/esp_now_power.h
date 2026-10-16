#pragma once
#include "esp_wifi.h"
#include <cstdint>

namespace gea::targets::esp32::espnow {
// Radio-wide setting: a borrowed ESP-NOW session must not change its owner's
// Wi-Fi power. Leave the SDK default untouched until explicitly requested.
inline bool setMaxTxPower(bool open, bool owned, int quarterDbm) {
  return open && owned && quarterDbm >= 8 && quarterDbm <= 84 &&
         esp_wifi_set_max_tx_power(static_cast<std::int8_t>(quarterDbm)) ==
             ESP_OK;
}
inline int getMaxTxPower(bool open) {
  std::int8_t power = 0;
  return open && esp_wifi_get_max_tx_power(&power) == ESP_OK ? power : -1;
}
} // namespace gea::targets::esp32::espnow
