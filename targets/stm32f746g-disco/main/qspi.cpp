// Register-level QSPI driver, safe to run from SystemInit: it keeps no
// globals. Values follow ST's QSPI_PreInitConfig example.

#include "qspi.h"

#include "board.h"
#include "stm32f7xx.h"

namespace gea::stm32::qspi {

namespace {

constexpr std::uint32_t kSpinLimit = 10'000'000;

constexpr std::uint8_t kWriteEnable = 0x06;
constexpr std::uint8_t kReadStatus = 0x05;
constexpr std::uint8_t kQuadRead = 0xEB;

template <typename Condition>
bool spinUntil(Condition condition) {
	for (std::uint32_t i = 0; i < kSpinLimit; ++i) {
		if (condition()) return true;
	}
	return false;
}

void delayMicros(std::uint32_t us) {
	const std::uint32_t start = DWT->CYCCNT;
	const std::uint32_t cycles = us * (board::kSysClockHz / 1'000'000);
	while (DWT->CYCCNT - start < cycles) {}
}

// Starts a single-line indirect command; the data phase follows through the FIFO.
bool begin(std::uint8_t instruction, bool read, std::uint32_t length) {
	if (!spinUntil([] { return !(QUADSPI->SR & QUADSPI_SR_BUSY); })) return false;

	const std::uint32_t mode = read ? QUADSPI_CCR_FMODE_0 : 0;
	const std::uint32_t dataPhase = length ? QUADSPI_CCR_DMODE_0 : 0;
	if (length) QUADSPI->DLR = length - 1;
	QUADSPI->CCR = mode | QUADSPI_CCR_IMODE_0 | dataPhase | instruction;
	return true;
}

bool finish() {
	if (!spinUntil([] { return QUADSPI->SR & QUADSPI_SR_TCF; })) return false;

	QUADSPI->FCR = QUADSPI_FCR_CTCF;
	return true;
}

bool waitFifo() {
	return spinUntil([] { return QUADSPI->SR & (QUADSPI_SR_FTF | QUADSPI_SR_TCF); });
}

bool send(std::uint8_t instruction, const std::uint8_t *data = nullptr, std::uint32_t length = 0) {
	if (!begin(instruction, false, length)) return false;

	auto *fifo = reinterpret_cast<volatile std::uint8_t *>(&QUADSPI->DR);
	for (std::uint32_t i = 0; i < length; ++i) {
		if (!waitFifo()) return false;

		*fifo = data[i];
	}
	return finish();
}

bool receive(std::uint8_t instruction, std::uint8_t *data, std::uint32_t length) {
	if (!begin(instruction, true, length)) return false;

	auto *fifo = reinterpret_cast<volatile std::uint8_t *>(&QUADSPI->DR);
	for (std::uint32_t i = 0; i < length; ++i) {
		if (!waitFifo()) return false;

		data[i] = *fifo;
	}
	return finish();
}

bool waitNotBusy() {
	for (std::uint32_t i = 0; i < kSpinLimit; ++i) {
		std::uint8_t status = 0;
		if (!receive(kReadStatus, &status, 1)) return false;
		if (!(status & 0x01)) return true;
	}
	return false;
}

bool abort() {
	QUADSPI->CR |= QUADSPI_CR_ABORT;
	return spinUntil([] { return !(QUADSPI->CR & QUADSPI_CR_ABORT); });
}

// Micron: 10 dummy cycles set through the volatile configuration register.
bool prepareMicron(std::uint32_t &ccr) {
	std::uint8_t vcr = 0;
	if (!receive(0x85, &vcr, 1)) return false;
	if (!send(kWriteEnable)) return false;

	vcr = static_cast<std::uint8_t>((vcr & 0x0F) | (10U << 4));
	if (!send(0x81, &vcr, 1)) return false;

	ccr = 10U << QUADSPI_CCR_DCYC_Pos;
	return true;
}

// Winbond: quad enable in status register 2 (volatile write), then the
// 0xEB mode byte as an alternate byte of 0xFF (no continuous read) and four
// dummy cycles.
bool prepareWinbond(std::uint32_t &ccr) {
	std::uint8_t sr2 = 0;
	if (!receive(0x35, &sr2, 1)) return false;

	if (!(sr2 & 0x02)) {
		sr2 |= 0x02;
		if (!send(0x50)) return false;
		if (!send(0x31, &sr2, 1)) return false;
		if (!waitNotBusy()) return false;
	}

	QUADSPI->ABR = 0xFF;
	ccr = (4U << QUADSPI_CCR_DCYC_Pos) | QUADSPI_CCR_ABMODE;
	return true;
}

}  // namespace

bool init(std::uint32_t &jedecId) {
	// 16 MB (2^24), chip select high for 3 cycles, 100 MHz (HCLK/2).
	QUADSPI->CR = 0;
	QUADSPI->DCR = (23U << QUADSPI_DCR_FSIZE_Pos) | (2U << QUADSPI_DCR_CSHT_Pos);
	QUADSPI->CR = (1U << QUADSPI_CR_PRESCALER_Pos) | QUADSPI_CR_SSHIFT | QUADSPI_CR_EN;

	if (!send(0x66) || !send(0x99)) return false;

	delayMicros(100);

	std::uint8_t id[3] = {};
	if (!receive(0x9F, id, sizeof(id))) return false;

	jedecId = (id[0] << 16) | (id[1] << 8) | id[2];

	std::uint32_t chipCcr = 0;
	const bool prepared = id[0] == 0x20 ? prepareMicron(chipCcr) : id[0] == 0xEF ? prepareWinbond(chipCcr) : false;
	if (!prepared) return false;
	if (!abort()) return false;

	// Memory-mapped 0xEB: 1-line instruction, 4-line 24-bit address and data.
	QUADSPI->CCR = QUADSPI_CCR_FMODE | QUADSPI_CCR_IMODE_0 | QUADSPI_CCR_ADMODE | QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_DMODE | chipCcr | kQuadRead;
	return true;
}

}  // namespace gea::stm32::qspi
