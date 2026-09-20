#include "power.h"

#include "board.h"

#include "esp_log.h"

// The LilyGo T5 4.7" routes its battery divider to GPIO14, which is ADC2 on the
// ESP32-S3 — ADC2 conflicts with Wi-Fi and is unreliable while the radio is up.
// Battery gauging is not needed for display bring-up, so report "unknown"
// rather than fight the ADC2/Wi-Fi arbitration. Fill this in later with a
// radio-quiescent ADC2 read if a battery indicator is wanted.

namespace gea::platform::power {

namespace {
constexpr char kTag[] = "lilygo_t5_power";
bool initialized = false;
}  // namespace

bool Power::init()
{
	if (!initialized) {
		initialized = true;
		ESP_LOGI(kTag, "battery gauge stubbed (GPIO%d is ADC2)",
		         static_cast<int>(gea::platform::board::power.batteryAdc));
	}
	return true;
}

int Power::batteryPercent()
{
	init();
	return -1;  // unknown
}

}  // namespace gea::platform::power
