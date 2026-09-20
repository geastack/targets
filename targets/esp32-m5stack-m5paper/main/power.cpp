#include "power.h"

#include "board.h"

#include <algorithm>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

namespace gea::platform::power {

namespace {

constexpr char kTag[] = "m5paper_power";
constexpr adc_channel_t kBatteryChannel = ADC_CHANNEL_7;  // GPIO35 / ADC1_CH7
adc_oneshot_unit_handle_t adcHandle = nullptr;
adc_cali_handle_t calibration = nullptr;
bool initialized = false;

}  // namespace

bool Power::init()
{
	if (initialized) return adcHandle != nullptr;
	initialized = true;

	adc_oneshot_unit_init_cfg_t unitConfig = {};
	unitConfig.unit_id = ADC_UNIT_1;
	unitConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
	if (adc_oneshot_new_unit(&unitConfig, &adcHandle) != ESP_OK) return false;

	adc_oneshot_chan_cfg_t channelConfig = {};
	channelConfig.atten = ADC_ATTEN_DB_12;
	channelConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
	if (adc_oneshot_config_channel(adcHandle, kBatteryChannel, &channelConfig) != ESP_OK) return false;

	adc_cali_line_fitting_config_t calConfig = {};
	calConfig.unit_id = ADC_UNIT_1;
	calConfig.atten = ADC_ATTEN_DB_12;
	calConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
	calConfig.default_vref = 1100;
	const esp_err_t calErr = adc_cali_create_scheme_line_fitting(&calConfig, &calibration);
	if (calErr != ESP_OK) {
		calibration = nullptr;
		ESP_LOGW(kTag, "ADC calibration unavailable: %s", esp_err_to_name(calErr));
	}
	return true;
}

int Power::batteryPercent()
{
	if (!init()) return -1;
	int rawTotal = 0;
	constexpr int kSamples = 8;
	for (int i = 0; i < kSamples; ++i) {
		int raw = 0;
		if (adc_oneshot_read(adcHandle, kBatteryChannel, &raw) != ESP_OK) return -1;
		rawTotal += raw;
	}
	const int raw = rawTotal / kSamples;
	int pinMillivolts = 0;
	if (!calibration || adc_cali_raw_to_voltage(calibration, raw, &pinMillivolts) != ESP_OK) {
		// 12-bit ADC, nominal 3.3 V full scale. Calibration normally handles the
		// ESP32's wider 12 dB attenuation curve; this is only a fallback.
		pinMillivolts = (raw * 3300) / 4095;
	}
	const int batteryMillivolts = pinMillivolts * 2;  // board divider is 1:2
	return std::clamp(((batteryMillivolts - 3300) * 100) / 900, 0, 100);
}

}  // namespace gea::platform::power
