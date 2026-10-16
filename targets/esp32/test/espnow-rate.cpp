#include "sdkconfig.h"
#if GEA_TEST_LEGACY == 1
#undef CONFIG_IDF_TARGET_ESP32S31
#elif GEA_TEST_LEGACY == 2
#undef CONFIG_SOC_WIFI_HE_SUPPORT
#endif
// Keep the real SDK Wi-Fi/ESP-NOW declarations and rate types. Event-loop and
// netif factory declarations are unrelated to this host-only API contract.
#define ESP_EVENT_H_
#define _ESP_WIFI_DEFAULT_H
#include "../esp_now_rate.h"
#include "host/esp_now.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <limits>
namespace {
using namespace gea::targets::esp32::espnow;
std::uint8_t protocols = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                         WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX;
bool protocolFailure = false, setterFailure = false;
unsigned protocolCalls = 0, setterCalls = 0;
esp_now_rate_config_t applied{};
std::array<std::uint8_t, 6> destination{2, 3, 4, 5, 6, 7};
} // namespace
extern "C" esp_err_t esp_wifi_get_protocol(wifi_interface_t interface,
                                           std::uint8_t *value) {
  assert(interface == WIFI_IF_STA);
  ++protocolCalls;
  if (protocolFailure)
    return ESP_FAIL;
  *value = protocols;
  return ESP_OK;
}
extern "C" esp_err_t esp_now_set_peer_rate_config(const std::uint8_t *mac,
                                                  esp_now_rate_config_t *rate) {
  assert(std::equal(destination.begin(), destination.end(), mac));
  ++setterCalls;
  applied = *rate;
  return setterFailure ? ESP_ERR_ESPNOW_ARG : ESP_OK;
}
extern "C" bool gea_espnow_open(int channel, int rate) {
  esp_now_rate_config_t selected{};
  return channel >= 0 && channel <= 13 && rateConfig(rate, selected) &&
         rateProtocolReady(rate);
}
int main() {
  const auto &api = gea::host::EspNow;
  struct Existing {
    int selector;
    wifi_phy_mode_t mode;
    wifi_phy_rate_t rate;
  };
  for (const auto expected :
       {Existing{24, WIFI_PHY_MODE_11G, WIFI_PHY_RATE_24M},
        Existing{36, WIFI_PHY_MODE_11G, WIFI_PHY_RATE_36M},
        Existing{54, WIFI_PHY_MODE_11G, WIFI_PHY_RATE_54M},
        Existing{65, WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS7_LGI}}) {
    assert(api.open(6, expected.selector));
    const auto before = protocolCalls;
    assert(configureRate(destination.data(), expected.selector));
    assert(protocolCalls == before); // Existing modes have no added SDK query.
    assert(applied.phymode == expected.mode && applied.rate == expected.rate);
    assert(!applied.ersu && !applied.dcm);
  }
  for (double invalid : {-1., 0., 114., 114.7, 115.5, 116.,
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    assert(!api.open(6, invalid));
  for (double invalid :
       {-1., 14., 6.5, std::numeric_limits<double>::infinity()})
    assert(!api.open(invalid, 54));
  esp_now_rate_config_t unchanged{};
  unchanged.phymode = WIFI_PHY_MODE_HT20;
  unchanged.rate = WIFI_PHY_RATE_MCS7_LGI;
  unchanged.ersu = true;
  const auto settersBeforeInvalid = setterCalls;
  for (int invalid : {-1, 0, 114, 116}) {
    assert(!rateConfig(invalid, unchanged));
    assert(unchanged.phymode == WIFI_PHY_MODE_HT20 && unchanged.ersu);
    assert(!configureRate(destination.data(), invalid));
  }
  assert(setterCalls == settersBeforeInvalid);
#if CONFIG_IDF_TARGET_ESP32S31 && CONFIG_SOC_WIFI_HE_SUPPORT
  assert(api.open(6, 115));
  assert(configureRate(destination.data(), 115));
  assert(applied.phymode == WIFI_PHY_MODE_HE20 &&
         applied.rate == WIFI_PHY_RATE_MCS9_SGI);
  assert(!applied.ersu && !applied.dcm);
  protocols &= ~WIFI_PROTOCOL_11AX;
  const auto before = setterCalls;
  assert(!api.open(0, 115)); // Borrowed non-AX radio must remain unchanged.
  assert(!api.open(6, 115));
  assert(!configureRate(destination.data(), 115) && setterCalls == before);
  protocols |= WIFI_PROTOCOL_11AX;
  protocolFailure = true;
  assert(!api.open(6, 115));
  assert(!configureRate(destination.data(), 115) && setterCalls == before);
  protocolFailure = false;
  setterFailure = true;
  assert(!configureRate(destination.data(), 115) && setterCalls == before + 1);
  std::puts("HE rate: actual S31 SDK types, native facade, integer validation, "
            "legacy mappings, protocol prerequisite and SDK failure passed");
#else
  const auto before = protocolCalls;
  assert(!api.open(6, 115));
  assert(!rateConfig(115, unchanged));
  assert(!configureRate(destination.data(), 115));
  assert(protocolCalls == before && setterCalls == settersBeforeInvalid);
  std::puts("HE rate: non-HE/non-S31 compile guard rejects115; all existing "
            "modes pass");
#endif
}
