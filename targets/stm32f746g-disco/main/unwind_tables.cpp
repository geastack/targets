// The app's code lives in QSPI (0x90000000), everything else in internal flash
// (0x08000000). An .ARM.exidx entry reaches only +-1 GB, so the linker script
// keeps one table per region and libgcc's unwinder asks here which one covers
// a given program counter.

#include "qspi.h"

#include <cstdint>

extern "C" {

extern const std::uint32_t __exidx_start[];
extern const std::uint32_t __exidx_end[];
extern const std::uint32_t __qspi_exidx_start[];
extern const std::uint32_t __qspi_exidx_end[];

std::uintptr_t __gnu_Unwind_Find_exidx(std::uintptr_t pc, int *count) {
	namespace qspi = gea::stm32::qspi;
	const bool inQspi = pc >= qspi::kBase && pc < qspi::kBase + qspi::kSizeBytes;
	const std::uint32_t *start = inQspi ? __qspi_exidx_start : __exidx_start;
	const std::uint32_t *end = inQspi ? __qspi_exidx_end : __exidx_end;

	constexpr int kWordsPerEntry = 2;
	*count = static_cast<int>(end - start) / kWordsPerEntry;
	return reinterpret_cast<std::uintptr_t>(start);
}

}  // extern "C"
