#include "clock.h"

#include "board.h"
#include "stm32f7xx.h"

namespace gea::stm32::clock {

namespace {

std::uint32_t gLastCount = 0;
std::uint64_t gHighBits = 0;

}  // namespace

void init() {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	(void)RCC->APB1ENR;

	TIM2->CR1 = 0;
	TIM2->PSC = board::kApb1TimerHz / 1'000'000 - 1;
	TIM2->ARR = 0xFFFFFFFF;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->CR1 = TIM_CR1_CEN;
}

std::uint64_t micros() {
	const std::uint32_t count = TIM2->CNT;
	if (count < gLastCount) gHighBits += 1ULL << 32;

	gLastCount = count;
	return gHighBits | count;
}

void delayMicros(std::uint32_t us) {
	const std::uint32_t start = TIM2->CNT;
	while (TIM2->CNT - start < us) {}
}

}  // namespace gea::stm32::clock
