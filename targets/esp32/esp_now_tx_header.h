#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gea::targets::esp32::espnow {
struct TxMacHeader {
  std::uint32_t control = 0;  // Frame Control low16, Duration high16.
  std::uint32_t sequence = 0; // Sequence Control low16, valid bit16.
};

inline TxMacHeader actionTxMacHeader(const std::uint8_t *bytes,
                                     std::size_t headerBytes,
                                     const std::uint8_t *source,
                                     const std::uint8_t *destination) noexcept {
  if (!bytes || headerBytes < 24 || !source || !destination)
    return {};
  const auto read16 = [&](unsigned at) {
    return std::uint16_t(bytes[at]) | (std::uint16_t(bytes[at + 1]) << 8);
  };
  const auto control = read16(0);
  // Version zero, management Action, neither ToDS nor FromDS. Remaining
  // flags (including Retry and Protected) are retained without interpretation.
  if ((control & 0x03ffU) != 0x00d0U ||
      std::memcmp(bytes + 4, destination, 6) ||
      std::memcmp(bytes + 10, source, 6))
    return {};
  return {std::uint32_t(control) | (std::uint32_t(read16(2)) << 16),
          std::uint32_t(read16(22)) | 0x10000U};
}

template <class Info>
inline auto callbackTxMacHeader(const Info *info) noexcept
    -> decltype(info->data, info->src_addr, info->des_addr, TxMacHeader{}) {
  if (!info || !info->data)
    return {};
  // IDF promises data starts at the MAC header. Its data_len is an 8-bit
  // BODY length excluding that header, not an available-buffer length. It
  // cannot bound the complete ESP-NOW v2 frame (up to1470 payload bytes).
  // Read only the fixed24-byte header in this callback; never retain pointers,
  // read the body, mutate SDK memory, or infer retries from a zero Retry bit.
  return actionTxMacHeader(info->data, 24, info->src_addr, info->des_addr);
}
// Older public SDK callback shapes do not expose the MAC header.
inline TxMacHeader callbackTxMacHeader(...) noexcept { return {}; }
} // namespace gea::targets::esp32::espnow
