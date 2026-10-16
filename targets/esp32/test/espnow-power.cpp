#include "../esp_now_power.h"
#include "host/esp_now.h"
#include <cassert>
#include <cstdio>
#include <limits>
namespace {
bool open = false, owned = false, fail = false;
int setCalls = 0, getCalls = 0;
std::int8_t actual = 80;
gea::platform::espnow::Datagram incoming{};
} // namespace
int esp_wifi_set_max_tx_power(std::int8_t power) {
  ++setCalls;
  if (fail)
    return -1;
  // The public SDK returns a discrete cap, not necessarily the request.
  actual = power < 20   ? 8
           : power < 28 ? 20
           : power < 34 ? 28
           : power < 44 ? 34
           : power < 52 ? 44
           : power < 56 ? 52
           : power < 60 ? 56
           : power < 66 ? 60
           : power < 72 ? 66
           : power < 80 ? 72
                        : 80;
  return ESP_OK;
}
int esp_wifi_get_max_tx_power(std::int8_t *power) {
  ++getCalls;
  if (fail)
    return -1;
  *power = actual;
  return ESP_OK;
}
extern "C" bool gea_espnow_set_max_tx_power(int power) {
  return gea::targets::esp32::espnow::setMaxTxPower(open, owned, power);
}
extern "C" int gea_espnow_get_max_tx_power() {
  return gea::targets::esp32::espnow::getMaxTxPower(open);
}
extern "C" bool gea_espnow_receive(gea::platform::espnow::Datagram *packet) {
  *packet = incoming;
  return true;
}
int main() {
  // Native receive metadata survives the API copy independently of the wire
  // payload. Zero is a valid radio time; an absent field has its own flag.
  gea::platform::espnow::Datagram packet{};
  assert(!packet.radioTimestampValid);
  for (const auto clock : {0U, 0xffffffffU}) {
    incoming.radioTimestampUs = clock;
    incoming.radioTimestampValid = true;
    incoming.receivedUs = 9000000000LL;
    assert(gea::platform::espnow::EspNow::receive(packet));
    assert(packet.radioTimestampUs == clock && packet.radioTimestampValid);
    assert(packet.receivedUs == incoming.receivedUs);
  }
  incoming = {};
  assert(gea::platform::espnow::EspNow::receive(packet));
  assert(!packet.radioTimestampValid && packet.radioTimestampUs == 0);
  const auto &api = gea::host::EspNow;
  assert(!api.setMaxTxPower(8) && api.getMaxTxPower() == -1);
  assert(setCalls == 0 && getCalls == 0);
  open = true;
  assert(!api.setMaxTxPower(8)); // Borrowed maintenance radio is unchanged.
  assert(setCalls == 0 && api.getMaxTxPower() == 80);
  owned = true;
  for (double invalid :
       {-1., 0., 7., 85., 256., 8.5, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    assert(!api.setMaxTxPower(invalid));
  assert(setCalls == 0);
  assert(api.setMaxTxPower(8) && api.getMaxTxPower() == 8);
  assert(api.setMaxTxPower(19) && api.getMaxTxPower() == 8);
  assert(api.setMaxTxPower(20) && api.getMaxTxPower() == 20);
  assert(api.setMaxTxPower(84) && api.getMaxTxPower() == 80);
  fail = true;
  assert(!api.setMaxTxPower(8) && api.getMaxTxPower() == -1);
  assert(actual == 80);
  open = false;
  assert(api.getMaxTxPower() == -1);
  std::puts("ESP-NOW power: ownership, input validation, SDK "
            "quantization/error, facade and native receive metadata passed");
}
