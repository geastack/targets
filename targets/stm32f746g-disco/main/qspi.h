#pragma once

#include <cstdint>

// The board's 16 MB QSPI flash (N25Q128A on boards before C03, W25Q128J
// after), mapped at kBase for execute-in-place.
namespace gea::stm32::qspi {

inline constexpr std::uint32_t kBase = 0x90000000;
inline constexpr std::uint32_t kSizeBytes = 16 * 1024 * 1024;

// Runs from SystemInit (no globals yet): resets the chip, reads its JEDEC id,
// enables quad reads and maps the flash at kBase.
bool init(std::uint32_t &jedecId);

}  // namespace gea::stm32::qspi
