// Runs from Reset_Handler before .data/.bss exist and before any static
// constructor: clock tree, MPU and caches, SDRAM, then QSPI in memory-mapped
// mode. Nothing here may touch a global except the uninitialised DTCM record.
// Register values follow ST's QSPI_PreInitConfig and FMC_SDRAM examples.

#include "system.h"

#include <cstdint>

#include "board.h"
#include "gpio_pins.h"
#include "mpu_armv7.h"
#include "qspi.h"
#include "stm32f7xx.h"

using namespace gea::stm32;

extern "C" {
uint32_t SystemCoreClock = board::kSysClockHz;
}

__attribute__((section(".dtcm"))) static BootInfo gBootInfo;

const BootInfo &gea::stm32::bootInfo() {
	return gBootInfo;
}

namespace {

constexpr std::uint32_t kSpinLimit = 10'000'000;

template <typename Condition>
bool spinUntil(Condition condition) {
	for (std::uint32_t i = 0; i < kSpinLimit; ++i) {
		if (condition()) return true;
	}
	return false;
}

void startCycleCounter() {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delayMicros(std::uint32_t us) {
	const std::uint32_t start = DWT->CYCCNT;
	const std::uint32_t cycles = us * (board::kSysClockHz / 1'000'000);
	while (DWT->CYCCNT - start < cycles) {}
}

// HSE 25 MHz -> PLL /25 *400 /2 = 200 MHz, over-drive on, 6 flash wait
// states, APB1 /4 (50 MHz), APB2 /2 (100 MHz); PLLQ /9 keeps the 48 MHz
// domain (RNG, SDMMC) in range at 44 MHz.
bool configureClock() {
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
	PWR->CR1 |= PWR_CR1_VOS;

	RCC->CR |= RCC_CR_HSEON;
	if (!spinUntil([] { return RCC->CR & RCC_CR_HSERDY; })) return false;

	RCC->PLLCFGR = (25U << RCC_PLLCFGR_PLLM_Pos) | (400U << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLSRC_HSE | (9U << RCC_PLLCFGR_PLLQ_Pos);
	RCC->CR |= RCC_CR_PLLON;
	if (!spinUntil([] { return RCC->CR & RCC_CR_PLLRDY; })) return false;

	PWR->CR1 |= PWR_CR1_ODEN;
	if (!spinUntil([] { return PWR->CSR1 & PWR_CSR1_ODRDY; })) return false;

	PWR->CR1 |= PWR_CR1_ODSWEN;
	if (!spinUntil([] { return PWR->CSR1 & PWR_CSR1_ODSWRDY; })) return false;

	FLASH->ACR = FLASH_ACR_LATENCY_6WS | FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN;
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	return spinUntil([] { return (RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL; });
}

struct MpuRegion {
	std::uint32_t base;
	std::uint32_t size;  // ARM_MPU_REGION_SIZE_*
	std::uint32_t access;  // ARM_MPU_AP_*
	std::uint32_t tex;
	bool shareable;
	bool cacheable;
	bool bufferable;
	bool executable;
	std::uint32_t subregionsOff = 0;
};

// CMSIS 5.1's ARM_MPU_RASR drops the size, sub-region and enable fields (a
// region built with it stays disabled), so the value is encoded here.
std::uint32_t attributes(const MpuRegion &region) {
	return (static_cast<std::uint32_t>(!region.executable) << MPU_RASR_XN_Pos) | (region.access << MPU_RASR_AP_Pos) |
	       (region.tex << MPU_RASR_TEX_Pos) | (static_cast<std::uint32_t>(region.shareable) << MPU_RASR_S_Pos) |
	       (static_cast<std::uint32_t>(region.cacheable) << MPU_RASR_C_Pos) | (static_cast<std::uint32_t>(region.bufferable) << MPU_RASR_B_Pos) |
	       (region.subregionsOff << MPU_RASR_SRD_Pos) | (region.size << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;
}

// ST's region 0 makes 0x60000000-0xDFFFFFFF no-access so the core never
// speculates into FMC bank 1 (a 24 us bus stall that starves the LTDC); the
// higher regions re-open exactly the memories this board has. SDRAM must be
// Normal memory: as the default map's Device memory, an unaligned access
// faults.
void configureMpu() {
	const MpuRegion regions[] = {
		{0x00000000, ARM_MPU_REGION_SIZE_4GB, ARM_MPU_AP_NONE, 0, true, false, false, false, 0x87},  // background
		{QSPI_BASE, ARM_MPU_REGION_SIZE_16MB, ARM_MPU_AP_RO, 0, false, true, false, true},        // XIP, write-through
		{FMC_R_BASE, ARM_MPU_REGION_SIZE_8KB, ARM_MPU_AP_FULL, 0, true, false, true, false},      // FMC/QSPI registers, device
		{board::kSdramBase, ARM_MPU_REGION_SIZE_8MB, ARM_MPU_AP_FULL, 1, false, true, true, false},   // heap, write-back
		{board::kSdramBase, ARM_MPU_REGION_SIZE_512KB, ARM_MPU_AP_FULL, 0, false, true, false, false},  // framebuffers, write-through
	};

	ARM_MPU_Disable();
	for (std::uint32_t i = 0; i < sizeof(regions) / sizeof(regions[0]); ++i) {
		ARM_MPU_SetRegion(ARM_MPU_RBAR(i, regions[i].base), attributes(regions[i]));
	}
	ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}

void enableGpioClocks() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
	                RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN | RCC_AHB1ENR_GPIOJEN | RCC_AHB1ENR_GPIOKEN;
	(void)RCC->AHB1ENR;
}

// ---- SDRAM: MT48LC4M32B2 on a 16-bit bus, 8 MB at 0xC0000000 ----------------

bool sdramCommand(std::uint32_t mode, std::uint32_t refreshCount = 1, std::uint32_t modeRegister = 0) {
	if (!spinUntil([] { return !(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY); })) return false;

	FMC_Bank5_6->SDCMR = mode | FMC_SDCMR_CTB1 | ((refreshCount - 1) << FMC_SDCMR_NRFS_Pos) | (modeRegister << FMC_SDCMR_MRD_Pos);
	return spinUntil([] { return !(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY); });
}

bool initSdram() {
	const gpio::PinConfig fmc{gpio::Mode::Alternate, gpio::Pull::Up, gpio::Speed::High, gpio::Drive::PushPull, 12};
	gpio::configure(GPIOC, gpio::pins({3}), fmc);
	gpio::configure(GPIOD, gpio::pins({0, 1, 8, 9, 10, 14, 15}), fmc);
	gpio::configure(GPIOE, gpio::pins({0, 1, 7, 8, 9, 10, 11, 12, 13, 14, 15}), fmc);
	gpio::configure(GPIOF, gpio::pins({0, 1, 2, 3, 4, 5, 11, 12, 13, 14, 15}), fmc);
	gpio::configure(GPIOG, gpio::pins({0, 1, 4, 5, 8, 15}), fmc);
	gpio::configure(GPIOH, gpio::pins({3, 5}), fmc);

	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
	(void)RCC->AHB3ENR;
	FMC_Bank1->BTCR[0] = 0x000030D2;

	// 8-bit columns, 12-bit rows, 16-bit bus, 4 banks, CAS 2, SDCLK = HCLK/2,
	// read burst on.
	FMC_Bank5_6->SDCR[0] = FMC_SDCR1_NR_0 | FMC_SDCR1_MWID_0 | FMC_SDCR1_NB | FMC_SDCR1_CAS_1 | FMC_SDCR1_SDCLK_1 | FMC_SDCR1_RBURST;
	// TMRD 2, TXSR 7, TRAS 4, TRC 7, TWR 2, TRP 2, TRCD 2 (cycles, minus one).
	FMC_Bank5_6->SDTR[0] = (1U << FMC_SDTR1_TMRD_Pos) | (6U << FMC_SDTR1_TXSR_Pos) | (3U << FMC_SDTR1_TRAS_Pos) | (6U << FMC_SDTR1_TRC_Pos) |
	                       (1U << FMC_SDTR1_TWR_Pos) | (1U << FMC_SDTR1_TRP_Pos) | (1U << FMC_SDTR1_TRCD_Pos);

	constexpr std::uint32_t kClockEnable = 1, kPrechargeAll = 2, kAutoRefresh = 3, kLoadMode = 4;
	constexpr std::uint32_t kModeCas2SingleWrite = 0x0220;
	if (!sdramCommand(kClockEnable)) return false;

	delayMicros(100);

	if (!sdramCommand(kPrechargeAll)) return false;
	if (!sdramCommand(kAutoRefresh, 8)) return false;
	if (!sdramCommand(kLoadMode, 1, kModeCas2SingleWrite)) return false;

	FMC_Bank5_6->SDRTR = 0x603U << FMC_SDRTR_COUNT_Pos;
	return true;
}

// ---- QSPI: N25Q128A (boards before C03) or W25Q128J, 16 MB at 0x90000000 ----

bool initQspi() {
	const gpio::PinConfig qspi{gpio::Mode::Alternate, gpio::Pull::None, gpio::Speed::VeryHigh, gpio::Drive::PushPull, 9};
	gpio::configure(GPIOB, gpio::pins({2}), qspi);
	gpio::configure(GPIOB, gpio::pins({6}), {gpio::Mode::Alternate, gpio::Pull::Up, gpio::Speed::VeryHigh, gpio::Drive::PushPull, 10});
	gpio::configure(GPIOD, gpio::pins({11, 12, 13}), qspi);
	gpio::configure(GPIOE, gpio::pins({2}), qspi);

	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
	(void)RCC->AHB3ENR;
	return qspi::init(gBootInfo.qspiJedecId);
}

}  // namespace

extern "C" void SystemInit() {
	SCB->CPACR |= (0xFU << 20);
	__DSB();
	__ISB();

	extern const std::uint32_t g_pfnVectors[];
	SCB->VTOR = reinterpret_cast<std::uint32_t>(g_pfnVectors);

	gBootInfo = {};
	startCycleCounter();
	gBootInfo.clockReady = configureClock();

	configureMpu();
	SCB_EnableICache();
	SCB_EnableDCache();

	enableGpioClocks();
	gBootInfo.sdramReady = initSdram();
	gBootInfo.qspiReady = initQspi();
}
