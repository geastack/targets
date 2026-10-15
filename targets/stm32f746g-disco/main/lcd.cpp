#include "lcd.h"

#include "board.h"
#include "clock.h"
#include "gpio_pins.h"
#include "stm32f7xx.h"

extern "C" char __framebuffer_start[];

namespace gea::stm32::lcd {

namespace {

constexpr auto kPanel = board::kDisplay;
constexpr std::uint32_t kBackBufferOffset = 0x40000;
constexpr std::uint32_t kRgb565 = 2;

constexpr std::uint32_t kAccumulatedHbp = kPanel.hsync + kPanel.hbp - 1;
constexpr std::uint32_t kAccumulatedVbp = kPanel.vsync + kPanel.vbp - 1;
constexpr std::uint32_t kAccumulatedActiveH = kAccumulatedVbp + kPanel.height;
constexpr std::uint32_t kLineBytes = kPanel.width * sizeof(std::uint16_t);
// One frame is 16.9 ms; waiting longer means the LTDC is not scanning.
constexpr std::uint64_t kBlankingTimeoutUs = 20'000;

static_assert(kWidth == kPanel.width && kHeight == kPanel.height);
static_assert(kLineBytes * kHeight <= kBackBufferOffset);

// 25 MHz / 25 * 192 = 192 MHz; /5 (R) /4 (DIVR) = 9.6 MHz pixel clock, and
// /4 (P) = 48 MHz for USB and SDMMC.
void startPixelClock() {
	RCC->PLLSAICFGR = (192U << RCC_PLLSAICFGR_PLLSAIN_Pos) | (1U << RCC_PLLSAICFGR_PLLSAIP_Pos) | (4U << RCC_PLLSAICFGR_PLLSAIQ_Pos) |
	                  (5U << RCC_PLLSAICFGR_PLLSAIR_Pos);
	RCC->DCKCFGR1 = (RCC->DCKCFGR1 & ~RCC_DCKCFGR1_PLLSAIDIVR) | (1U << RCC_DCKCFGR1_PLLSAIDIVR_Pos);
	RCC->CR |= RCC_CR_PLLSAION;
	while (!(RCC->CR & RCC_CR_PLLSAIRDY)) {}
}

void configurePins() {
	const gpio::PinConfig ltdc{gpio::Mode::Alternate, gpio::Pull::None, gpio::Speed::High, gpio::Drive::PushPull, 14};
	gpio::configure(GPIOE, gpio::pins({4}), ltdc);
	gpio::configure(GPIOG, gpio::pins({12}), {gpio::Mode::Alternate, gpio::Pull::None, gpio::Speed::High, gpio::Drive::PushPull, 9});
	gpio::configure(GPIOI, gpio::pins({9, 10, 14, 15}), ltdc);
	gpio::configure(GPIOJ, gpio::pins({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15}), ltdc);
	gpio::configure(GPIOK, gpio::pins({0, 1, 2, 4, 5, 6, 7}), ltdc);

	const gpio::PinConfig output{gpio::Mode::Output, gpio::Pull::None, gpio::Speed::Low};
	gpio::configure(GPIOI, gpio::pins({12}), output);
	gpio::configure(GPIOK, gpio::pins({3}), output);
}

void configureTimings() {
	LTDC->SSCR = ((kPanel.hsync - 1) << LTDC_SSCR_HSW_Pos) | (kPanel.vsync - 1);
	LTDC->BPCR = (kAccumulatedHbp << LTDC_BPCR_AHBP_Pos) | kAccumulatedVbp;
	LTDC->AWCR = ((kAccumulatedHbp + kPanel.width) << LTDC_AWCR_AAW_Pos) | kAccumulatedActiveH;
	LTDC->TWCR = ((kAccumulatedHbp + kPanel.width + kPanel.hfp) << LTDC_TWCR_TOTALW_Pos) | (kAccumulatedActiveH + kPanel.vfp);
	LTDC->BCCR = 0;
}

void configureLayer() {
	LTDC_Layer1->WHPCR = ((kAccumulatedHbp + kPanel.width) << LTDC_LxWHPCR_WHSPPOS_Pos) | (kAccumulatedHbp + 1);
	LTDC_Layer1->WVPCR = (kAccumulatedActiveH << LTDC_LxWVPCR_WVSPPOS_Pos) | (kAccumulatedVbp + 1);
	LTDC_Layer1->PFCR = kRgb565;
	LTDC_Layer1->CACR = 255;
	LTDC_Layer1->DCCR = 0;
	LTDC_Layer1->BFCR = (4U << LTDC_LxBFCR_BF1_Pos) | 5U;
	LTDC_Layer1->CFBAR = reinterpret_cast<std::uint32_t>(frontBuffer());
	LTDC_Layer1->CFBLR = (kLineBytes << LTDC_LxCFBLR_CFBP_Pos) | (kLineBytes + 3);
	LTDC_Layer1->CFBLNR = kPanel.height;
	LTDC_Layer1->CR = LTDC_LxCR_LEN;
	LTDC->SRCR = LTDC_SRCR_IMR;
}

void waitForDma2d() {
	while (DMA2D->CR & DMA2D_CR_START) {}
}

void runDma2d() {
	DMA2D->CR |= DMA2D_CR_START;
	waitForDma2d();
}

std::uint32_t offsetOf(int x, int y) {
	return static_cast<std::uint32_t>(y * kWidth + x);
}

// DMA2D wrote behind the D-cache: drop every cache line the rectangle's rows
// touch, rounded out to whole 32-byte lines (the framebuffers are write-through,
// so there is never dirty data to lose).
void invalidateRows(const std::uint16_t *buffer, int x, int y, int w, int h) {
	constexpr std::uintptr_t kLine = 32;
	const auto first = reinterpret_cast<std::uintptr_t>(buffer + offsetOf(x, y)) & ~(kLine - 1);
	const auto last = reinterpret_cast<std::uintptr_t>(buffer + offsetOf(x + w - 1, y + h - 1));
	SCB_InvalidateDCache_by_Addr(reinterpret_cast<std::uint32_t *>(first), static_cast<std::int32_t>(last - first + kLine));
}

}  // namespace

std::uint16_t *frontBuffer() {
	return reinterpret_cast<std::uint16_t *>(__framebuffer_start);
}

std::uint16_t *backBuffer() {
	return reinterpret_cast<std::uint16_t *>(__framebuffer_start + kBackBufferOffset);
}

void init() {
	startPixelClock();
	configurePins();

	RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
	(void)RCC->AHB1ENR;

	fill(frontBuffer(), 0, 0, kWidth, kHeight, 0);
	fill(backBuffer(), 0, 0, kWidth, kHeight, 0);

	configureTimings();
	configureLayer();
	// The line flag only latches with its interrupt enabled; the LTDC IRQ
	// stays off in the NVIC, so nothing is ever taken.
	LTDC->IER = LTDC_IER_LIE;
	LTDC->GCR |= LTDC_GCR_LTDCEN;

	gpio::write(GPIOI, gpio::pins({12}), true);
	setBacklight(true);
}

void waitForBlanking() {
	LTDC->LIPCR = kAccumulatedActiveH + 1;
	LTDC->ICR = LTDC_ICR_CLIF;
	const std::uint64_t deadline = clock::micros() + kBlankingTimeoutUs;
	while (!(LTDC->ISR & LTDC_ISR_LIF) && clock::micros() < deadline) {}
}

// Starts the copy and returns: the CPU renders the next frame meanwhile. A
// pixel it draws before the copy passes is dirty again, so the next flush
// presents it whole.
void present(int x, int y, int w, int h) {
	if (w <= 0 || h <= 0) return;

	waitForDma2d();
	__DSB();
	const std::uint32_t offset = offsetOf(x, y);
	const std::uint32_t lineOffset = static_cast<std::uint32_t>(kWidth - w);
	DMA2D->CR = 0;
	DMA2D->FGPFCCR = kRgb565;
	DMA2D->FGMAR = reinterpret_cast<std::uint32_t>(backBuffer() + offset);
	DMA2D->FGOR = lineOffset;
	DMA2D->OPFCCR = kRgb565;
	DMA2D->OMAR = reinterpret_cast<std::uint32_t>(frontBuffer() + offset);
	DMA2D->OOR = lineOffset;
	DMA2D->NLR = (static_cast<std::uint32_t>(w) << DMA2D_NLR_PL_Pos) | static_cast<std::uint32_t>(h);
	DMA2D->CR |= DMA2D_CR_START;
}

void fill(std::uint16_t *buffer, int x, int y, int w, int h, std::uint16_t color) {
	if (w <= 0 || h <= 0) return;

	waitForDma2d();
	DMA2D->CR = 3U << DMA2D_CR_MODE_Pos;
	DMA2D->OPFCCR = kRgb565;
	DMA2D->OCOLR = color;
	DMA2D->OMAR = reinterpret_cast<std::uint32_t>(buffer + offsetOf(x, y));
	DMA2D->OOR = static_cast<std::uint32_t>(kWidth - w);
	DMA2D->NLR = (static_cast<std::uint32_t>(w) << DMA2D_NLR_PL_Pos) | static_cast<std::uint32_t>(h);
	runDma2d();
	invalidateRows(buffer, x, y, w, h);
}

void setBacklight(bool on) {
	gpio::write(GPIOK, gpio::pins({3}), on);
}

void setLayerAlpha(std::uint8_t alpha) {
	LTDC_Layer1->CACR = alpha;
	LTDC->SRCR = LTDC_SRCR_VBR;
}

}  // namespace gea::stm32::lcd
