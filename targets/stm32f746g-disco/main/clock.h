#pragma once

#include <cstdint>

// Free-running microsecond clock on TIM2 (32-bit), widened to 64 bits by
// whoever reads it at least once per 71 minutes.
namespace gea::stm32::clock {

void init();
std::uint64_t micros();
void delayMicros(std::uint32_t us);

}  // namespace gea::stm32::clock
