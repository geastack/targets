// gea::host::Board on the STM32's die temperature sensor and VDDA, read by
// ADC1 (internal channels 18 and 17) with software-started conversions, and
// the same readings plus the hardware RNG for app native code.

#include "platform/sensors.h"

#include "board_sensors.h"

#include "stm32f7xx.h"

#include <cstdint>

namespace {

// Factory calibration, taken at 3.3 V: VREFINT, and the sensor at 30 and 110 °C.
const auto *kVrefCal = reinterpret_cast<const std::uint16_t *>(0x1FF0F44A);
const auto *kTempCal30 = reinterpret_cast<const std::uint16_t *>(0x1FF0F44C);
const auto *kTempCal110 = reinterpret_cast<const std::uint16_t *>(0x1FF0F44E);
constexpr float kCalVolts = 3.3f;
constexpr std::uint32_t kTemperatureChannel = 18;
constexpr std::uint32_t kReferenceChannel = 17;

bool gAdcReady = false;

// ADCCLK = APB2 / 4 = 25 MHz; 480-cycle sampling (19 µs) suits both internal
// channels, which need at least 10 µs.
void initAdc() {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	(void)RCC->APB2ENR;

	ADC->CCR = ADC_CCR_ADCPRE_0 | ADC_CCR_TSVREFE;
	ADC1->SMPR1 = (7U << ADC_SMPR1_SMP17_Pos) | (7U << ADC_SMPR1_SMP18_Pos);
	ADC1->CR2 = ADC_CR2_ADON;
	gAdcReady = true;
}

std::uint32_t convert(std::uint32_t channel) {
	if (!gAdcReady) initAdc();

	ADC1->SQR3 = channel;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while (!(ADC1->SR & ADC_SR_EOC)) {}

	return ADC1->DR;
}

}  // namespace

namespace gea::platform::board {

float temperatureC() {
	const float raw = static_cast<float>(convert(kTemperatureChannel));
	const float atCalibrationSupply = raw * static_cast<float>(*kVrefCal) / static_cast<float>(convert(kReferenceChannel));
	const float span = static_cast<float>(*kTempCal110 - *kTempCal30);
	return 30.0f + (atCalibrationSupply - static_cast<float>(*kTempCal30)) * 80.0f / span;
}

float supplyVolts() {
	return kCalVolts * static_cast<float>(*kVrefCal) / static_cast<float>(convert(kReferenceChannel));
}

}  // namespace gea::platform::board

namespace gea::stm32::sensors {

float temperatureC() { return gea::platform::board::temperatureC(); }

float supplyVolts() { return gea::platform::board::supplyVolts(); }

// The RNG runs from the 48 MHz domain (PLLQ, set up by SystemInit).
std::uint32_t random32() {
	if (!(RNG->CR & RNG_CR_RNGEN)) {
		RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
		(void)RCC->AHB2ENR;
		RNG->CR = RNG_CR_RNGEN;
	}

	while (!(RNG->SR & RNG_SR_DRDY)) {}

	return RNG->DR;
}

}  // namespace gea::stm32::sensors
