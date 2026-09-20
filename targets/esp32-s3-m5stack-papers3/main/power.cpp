#include "power.h"

#include "board.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// M5PaperS3 battery gauge. The 1800 mAh cell is divided by a matched resistor
// pair into GPIO3 = ADC1_CH2:
//
//   Battery+ ── R1 ── GPIO3 ── R2 ── GND      (R1 == R2, so the pin sees Vbat/2)
//
// ADC_ATTEN_DB_12 gives ~2.5 V full scale, so the ½x divider covers 0–5 V —
// comfortably past a full 4.2 V LiPo. Divider ratio and channel are taken from
// upstream PaperBoy's battery.c running on this board.
//
// Unlike the LilyGo T5 (whose divider lands on ADC2 and therefore fights the
// Wi-Fi radio for the ADC), this is ADC1 and stays readable with the radio up.

namespace gea::platform::power {

namespace {

constexpr char kTag[] = "papers3_power";

constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_2;  // GPIO3
constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr int kDividerRatio = 2;

// LiPo discharge endpoints. The curve is not linear, but for a battery glyph a
// linear map between "charged" and "cut-off" is the usual, honest-enough
// approximation.
constexpr int kFullMv = 4200;
constexpr int kEmptyMv = 3300;

bool initialized = false;
bool usable = false;
adc_oneshot_unit_handle_t adc = nullptr;
adc_cali_handle_t cali = nullptr;

}  // namespace

bool Power::init()
{
	if (initialized) return usable;
	initialized = true;

	adc_oneshot_unit_init_cfg_t unit = {};
	unit.unit_id = kAdcUnit;
	if (adc_oneshot_new_unit(&unit, &adc) != ESP_OK) {
		ESP_LOGW(kTag, "ADC1 unavailable; battery reported unknown");
		return false;
	}

	adc_oneshot_chan_cfg_t chan = {};
	chan.atten = kAdcAtten;
	chan.bitwidth = ADC_BITWIDTH_DEFAULT;
	if (adc_oneshot_config_channel(adc, kAdcChannel, &chan) != ESP_OK) {
		ESP_LOGW(kTag, "ADC1 channel config failed; battery reported unknown");
		return false;
	}

	// Curve-fitting calibration turns a raw count into millivolts using the
	// chip's factory eFuse data. Without it the reading is only a raw ratio, so
	// treat the gauge as unusable rather than reporting a made-up percentage.
	adc_cali_curve_fitting_config_t caliCfg = {};
	caliCfg.unit_id = kAdcUnit;
	caliCfg.chan = kAdcChannel;
	caliCfg.atten = kAdcAtten;
	caliCfg.bitwidth = ADC_BITWIDTH_DEFAULT;
	if (adc_cali_create_scheme_curve_fitting(&caliCfg, &cali) != ESP_OK) {
		ESP_LOGW(kTag, "ADC calibration unavailable; battery reported unknown");
		return false;
	}

	usable = true;
	ESP_LOGI(kTag, "battery gauge ready on GPIO%d (ADC1_CH2, 1/%dx divider)",
	         static_cast<int>(gea::platform::board::power.batteryAdc), kDividerRatio);
	return true;
}

int Power::batteryPercent()
{
	if (!init()) return -1;

	int raw = 0;
	if (adc_oneshot_read(adc, kAdcChannel, &raw) != ESP_OK) return -1;

	int mv = 0;
	if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) return -1;

	const int batteryMv = mv * kDividerRatio;
	int percent = ((batteryMv - kEmptyMv) * 100) / (kFullMv - kEmptyMv);
	if (percent < 0) percent = 0;
	if (percent > 100) percent = 100;
	return percent;
}

}  // namespace gea::platform::power
