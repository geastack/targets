#pragma once
#include "esp_now.h"
#include "esp_wifi.h"
#include <cstdint>

namespace gea::targets::esp32::espnow {
// 115 is the integer API selector for HE20 MCS9 short GI (114.7 Mbps),
// supported here only by ESP32-S31. Existing selectors retain their PHY mode.
inline bool rateConfig(int selector, esp_now_rate_config_t &out) {
  esp_now_rate_config_t rate{};
  rate.phymode = WIFI_PHY_MODE_11G;
  switch (selector) {
  case 24:
    rate.rate = WIFI_PHY_RATE_24M;
    break;
  case 36:
    rate.rate = WIFI_PHY_RATE_36M;
    break;
  case 54:
    rate.rate = WIFI_PHY_RATE_54M;
    break;
  case 65:
    rate.phymode = WIFI_PHY_MODE_HT20;
    rate.rate = WIFI_PHY_RATE_MCS7_LGI;
    break;
#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_SOC_WIFI_HE_SUPPORT
  case 115:
    rate.phymode = WIFI_PHY_MODE_HE20;
    rate.rate = WIFI_PHY_RATE_MCS9_SGI;
    break;
#endif
  default:
    return false;
  }
  out = rate;
  return true;
}

// Never change a borrowed radio's interface protocols. The owned S31 default
// includes 11AX; fail closed if the current owner has disabled that capability.
inline bool rateProtocolReady(int selector) {
  if (selector != 115)
    return true;
#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_SOC_WIFI_HE_SUPPORT
  std::uint8_t protocols = 0;
  return esp_wifi_get_protocol(WIFI_IF_STA, &protocols) == ESP_OK &&
         (protocols & WIFI_PROTOCOL_11AX) != 0;
#else
  return false;
#endif
}
inline bool configureRate(const std::uint8_t *mac, int selector) {
  esp_now_rate_config_t rate{};
  return rateConfig(selector, rate) && rateProtocolReady(selector) &&
         esp_now_set_peer_rate_config(mac, &rate) == ESP_OK;
}
} // namespace gea::targets::esp32::espnow
