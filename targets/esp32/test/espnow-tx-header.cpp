#include "../esp_now_tx_header.h"
#include "esp_now.h"
#include "platform/esp_now.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <type_traits>
namespace {
using gea::platform::espnow::EspNow;
using gea::platform::espnow::Statistics;
using gea::targets::esp32::espnow::actionTxMacHeader;
using gea::targets::esp32::espnow::callbackTxMacHeader;
bool opened = true, owned = true, capturing = false;
unsigned captureCalls = 0;
Statistics completed{};
} // namespace
extern "C" bool gea_espnow_capture_tx_header(bool enabled) {
  ++captureCalls;
  if (enabled && (!opened || !owned))
    return false;
  capturing = enabled;
  return true;
}
extern "C" void gea_espnow_statistics(Statistics *out) { *out = completed; }
int main() {
  // Use the actual public ESP-IDF callback type, including its unusual
  // 8-bit BODY length. No copied SDK struct stands in for this contract.
  static_assert(sizeof(esp_now_send_info_t{}.data_len) == 1);
  std::array<std::uint8_t, 6> source{2, 3, 4, 5, 6, 7};
  std::array<std::uint8_t, 6> destination{2, 8, 9, 10, 11, 12};
  std::array<std::uint8_t, 25> storage{};
  auto *header = storage.data() + 1; // Deliberately unaligned.
  const auto fill = [&](std::uint16_t fc, std::uint16_t duration,
                        std::uint16_t sequence) {
    std::memset(header, 0, 24);
    header[0] = fc;
    header[1] = fc >> 8;
    header[2] = duration;
    header[3] = duration >> 8;
    std::memcpy(header + 4, destination.data(), 6);
    std::memcpy(header + 10, source.data(), 6);
    std::memset(header + 16, 255, 6);
    header[22] = sequence;
    header[23] = sequence >> 8;
  };
  fill(0x48d0, 0x1234, 0xfff0);
  for (unsigned size = 0; size < 24; ++size) {
    auto result =
        actionTxMacHeader(header, size, source.data(), destination.data());
    assert(result.control == 0 && result.sequence == 0);
  }
  assert(actionTxMacHeader(nullptr, 24, source.data(), destination.data())
             .sequence == 0);
  assert(actionTxMacHeader(header, 24, nullptr, destination.data()).sequence ==
         0);
  assert(actionTxMacHeader(header, 24, source.data(), nullptr).sequence == 0);
  esp_now_send_info_t info{};
  info.data = header;
  info.src_addr = source.data();
  info.des_addr = destination.data();
  // data_len excludes the 24-byte header and cannot express large v2 bodies.
  // Zero, one or255 must never become an alleged total/header buffer length.
  for (unsigned body : {0U, 1U, 255U}) {
    info.data_len = body;
    const auto result = callbackTxMacHeader(&info);
    assert(result.control == 0x123448d0U && result.sequence == 0x1fff0U);
  }
  for (unsigned invalid : {0x00d1U, 0x00d2U, 0x00d3U, 0x00c0U, 0x0008U, 0x01d0U,
                           0x02d0U, 0x03d0U}) {
    fill(invalid, 0x1234, 0xfff0);
    assert(callbackTxMacHeader(&info).sequence == 0);
  }
  fill(0x48d0, 0x1234, 0xfff0);
  for (unsigned offset = 4; offset < 16; ++offset) {
    header[offset] ^= 0x01;
    assert(callbackTxMacHeader(&info).sequence == 0);
    header[offset] ^= 0x01;
  }
  assert(callbackTxMacHeader(static_cast<const esp_now_send_info_t *>(nullptr))
             .sequence == 0);
  info.data = nullptr;
  assert(callbackTxMacHeader(&info).sequence == 0);
  info.data = header;
  struct LegacyCallback {
    const std::uint8_t *destination;
  } legacy{};
  assert(callbackTxMacHeader(&legacy).sequence == 0);
  for (std::uint16_t sequence : {0U, 1U, 15U, 16U, 0xfff0U, 0xffffU}) {
    fill(0x00d0, 0, sequence);
    auto saved = callbackTxMacHeader(&info);
    // Valid zero fields are distinct from absent/invalid metadata.
    assert(saved.control == 0xd0 && saved.sequence == (0x10000U | sequence));
    std::memset(header, 0,
                24); // Callback-owned bytes may be reused immediately.
    assert(saved.control == 0xd0 && saved.sequence == (0x10000U | sequence));
    const auto next = callbackTxMacHeader(&info);
    assert(next.control == 0 &&
           next.sequence == 0); // Never carry old validity.
  }
  destination.fill(255);
  fill(0x08d0, 0xffff, 0x9876);
  const auto broadcast = callbackTxMacHeader(&info);
  assert(broadcast.control == 0xffff08d0U && broadcast.sequence == 0x19876);
  assert(EspNow::captureTxHeader(true) && capturing && captureCalls == 1);
  assert(EspNow::captureTxHeader(false) && !capturing && captureCalls == 2);
  opened = false;
  assert(!EspNow::captureTxHeader(true) && !capturing);
  opened = true;
  owned = false;
  assert(!EspNow::captureTxHeader(true) && !capturing);
  assert(EspNow::captureTxHeader(false));
  completed.lastSendFrameControl = 0x48d0;
  completed.lastSendDuration = 0x1234;
  completed.lastSendSequenceControl = 0xfff0;
  completed.lastSendMacValid = true;
  const auto read = EspNow::statistics();
  assert(read.lastSendMacValid && read.lastSendFrameControl == 0x48d0 &&
         read.lastSendDuration == 0x1234 &&
         read.lastSendSequenceControl == 0xfff0);
  completed = {};
  assert(!EspNow::statistics().lastSendMacValid);
  std::puts("TX MAC header: real SDK type, bounds, endianness, format, "
            "address, lifetime, validity and native API passed");
}
