#pragma once

#include <cstdint>

// Wall-clock time from the RTC, which the 32.768 kHz crystal keeps running
// across resets. The RTC is read once at boot and on every set; between those
// the microsecond timer carries the time forward.
namespace gea::stm32::rtc_clock {

// Starts the crystal and the RTC on first power-up; keeps a running RTC as is.
bool init();
// False until the time has been set once (e.g. `gea devctl set-time`).
bool isSet();
std::int64_t epochMicros();
bool setEpochSeconds(std::int64_t seconds);

}  // namespace gea::stm32::rtc_clock
