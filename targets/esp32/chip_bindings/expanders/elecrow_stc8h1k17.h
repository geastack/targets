#pragma once

#include <cstdint>

// The battery gauge the CrowPanel Advanced P4's STC8H1K17 board-control MCU
// keeps (see elecrow_stc8h1k17.cpp for the register map).
namespace gea::platform::esp32::chip_bindings::expanders {

struct Stc8h1k17Battery {
	std::uint32_t adcMillivolts = 0;
	std::uint32_t batteryMillivolts = 0;
	std::uint8_t levelPercent = 0;
	// 0 idle, 1 charging, 2 full, 3 not charging, 4 error.
	std::uint8_t chargeState = 0;
	std::uint8_t ledState = 0;
};

bool stc8h1k17Battery(Stc8h1k17Battery &out);

}  // namespace gea::platform::esp32::chip_bindings::expanders
