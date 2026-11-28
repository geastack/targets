#include "console.h"

#include "board.h"
#include "gpio_pins.h"
#include "stm32f7xx.h"

namespace gea::stm32::console {

namespace {

constexpr std::size_t kRxRingBytes = 2048;
constexpr std::uint32_t kUsart1RxChannel = 4;

__attribute__((section(".dtcm"), aligned(32))) std::uint8_t gRxRing[kRxRingBytes];
std::size_t gRxTail = 0;

std::size_t rxHead() {
	return kRxRingBytes - DMA2_Stream2->NDTR;
}

void startRxDma() {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	(void)RCC->AHB1ENR;

	DMA2_Stream2->CR = 0;
	while (DMA2_Stream2->CR & DMA_SxCR_EN) {}

	DMA2_Stream2->PAR = reinterpret_cast<std::uint32_t>(&USART1->RDR);
	DMA2_Stream2->M0AR = reinterpret_cast<std::uint32_t>(gRxRing);
	DMA2_Stream2->NDTR = kRxRingBytes;
	DMA2_Stream2->CR = (kUsart1RxChannel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;
}

}  // namespace

void init(std::uint32_t baud) {
	const gpio::PinConfig uart{gpio::Mode::Alternate, gpio::Pull::Up, gpio::Speed::High, gpio::Drive::PushPull, 7};
	gpio::configure(GPIOA, gpio::pins({9}), uart);
	gpio::configure(GPIOB, gpio::pins({7}), uart);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	(void)RCC->APB2ENR;

	USART1->CR1 = 0;
	USART1->BRR = (board::kApb2Hz + baud / 2) / baud;
	USART1->CR3 = USART_CR3_OVRDIS | USART_CR3_DMAR;
	startRxDma();
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void write(const char *data, std::size_t length) {
	for (std::size_t i = 0; i < length; ++i) {
		while (!(USART1->ISR & USART_ISR_TXE)) {}

		USART1->TDR = static_cast<std::uint8_t>(data[i]);
	}
}

std::size_t read(char *data, std::size_t capacity) {
	const std::size_t head = rxHead();
	std::size_t count = 0;
	while (gRxTail != head && count < capacity) {
		data[count++] = static_cast<char>(gRxRing[gRxTail]);
		gRxTail = (gRxTail + 1) % kRxRingBytes;
	}
	return count;
}

}  // namespace gea::stm32::console
