#pragma once

#include <cstdint>

// The board's on-chip sensors, for app native code: the STM32's die
// temperature and supply voltage (ADC1) and its hardware random generator.
namespace gea::stm32::sensors {

// Die temperature in °C from the factory-calibrated sensor (~20 µs, blocking).
float temperatureC();
// VDDA in volts, measured against the internal reference (~20 µs, blocking).
float supplyVolts();
// 32 bits from the true random number generator.
std::uint32_t random32();

}  // namespace gea::stm32::sensors
