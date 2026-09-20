// Power for a board with no PMIC.
//
// Same shape as touch_absent.cpp: the framework references Power
// unconditionally (Runtime::boot, BatteryService, the device-control task), so
// a board without the chip still has to provide the symbols.
//
// Compiled in place of a PMIC binding when a composed board selects no
// `chips.power` — the usual case for a module powered straight from USB.

#include "power.h"

namespace gea::platform::power {

// Nothing to bring up, and nothing that can fail.
bool Power::init() { return true; }

// -1 means "this board cannot measure a battery", which is what a USB-powered
// module is. Reporting 100 would read as a full battery and send the UI and any
// low-power policy down a path built on a measurement that was never taken.
int Power::batteryPercent() { return -1; }

}  // namespace gea::platform::power
