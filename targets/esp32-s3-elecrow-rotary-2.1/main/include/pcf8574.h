#pragma once

#include "esp_err.h"

#include <cstdint>

namespace gea::platform::elecrow {

class Pcf8574 {
public:
	static esp_err_t init();
	static esp_err_t writePin(int bit, bool high);
	static esp_err_t read(std::uint8_t *value);
};

}  // namespace gea::platform::elecrow
