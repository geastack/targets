#include "power.h"

namespace gea::platform::power {

bool Power::init()
{
	return true;
}

int Power::batteryPercent()
{
	return -1;
}

}  // namespace gea::platform::power
