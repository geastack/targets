#include "power.h"

#include "chip_bindings/expanders/elecrow_stc8h1k17.h"

// The battery (PH2.0 J3, TP4059 charger) is gauged by the STC8H1K17
// board-control MCU: it divides VBAT into its ADC and reads the charger's
// CHRG/DONE pins, and the P4 reads the result over I2C.
namespace gea::platform::power {

namespace {
namespace board_control = gea::platform::esp32::chip_bindings::expanders;
}

bool Power::init()
{
	board_control::Stc8h1k17Battery battery;
	return board_control::stc8h1k17Battery(battery);
}

int Power::batteryPercent()
{
	board_control::Stc8h1k17Battery battery;
	if (!board_control::stc8h1k17Battery(battery)) return -1;
	// No cell fitted reads as (almost) no voltage: report "no battery".
	if (battery.batteryMillivolts < 2500) return -1;
	return battery.levelPercent > 100 ? 100 : battery.levelPercent;
}

bool Power::charging()
{
	board_control::Stc8h1k17Battery battery;
	return board_control::stc8h1k17Battery(battery) && battery.chargeState == 1;
}

}  // namespace gea::platform::power
