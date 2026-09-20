#include "power.h"

#include "tab5_drivers.h"

#include <algorithm>
#include <cmath>

namespace gea::platform::power {

bool Power::init()
{
	return gea::platform::tab5::initPowerMonitor();
}

int Power::batteryPercent()
{
	const auto status = gea::platform::tab5::readBatteryStatus();
	if (!status.isPresent) return -1;
	return static_cast<int>(std::lround(std::clamp(status.chargePercent, 0.0f, 100.0f)));
}

}  // namespace gea::platform::power
