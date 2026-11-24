#pragma once

#include "driver/i2c_master.h"

namespace gea::platform::elecrow {
// External I²C connector on GPIO38/39. The touch bus stays on GPIO6/7.
i2c_master_bus_handle_t auxiliaryI2c();
}
