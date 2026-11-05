// libstdc++'s clocks for a newlib without clock_gettime: its own chrono.o
// derives steady_clock from gettimeofday, which here is the settable RTC, so
// every clock set would move timers and frame pacing. Defining both clocks
// keeps chrono.o out of the link: steady_clock counts TIM2 microseconds since
// boot, system_clock stays the wall clock.

#include <chrono>

#include "clock.h"
#include "rtc_clock.h"

namespace std::chrono::_V2 {

steady_clock::time_point steady_clock::now() noexcept {
	return time_point(microseconds(gea::stm32::clock::micros()));
}

system_clock::time_point system_clock::now() noexcept {
	return time_point(microseconds(gea::stm32::rtc_clock::epochMicros()));
}

}  // namespace std::chrono::_V2
