// Every configurable fault lands here and prints the fault status registers
// and the stacked program counter over the console before resetting. An
// uncaught C++ exception does the same through the terminate handler.

#include "clock.h"
#include "console.h"
#include "stm32f7xx.h"

#include <cstdint>

using namespace gea::stm32;

namespace {

void writeHex(const char *label, std::uint32_t value) {
	char line[24] = {};
	int length = 0;
	while (*label) line[length++] = *label++;

	for (int shift = 28; shift >= 0; shift -= 4) {
		line[length++] = "0123456789abcdef"[(value >> shift) & 0xF];
	}
	line[length++] = '\n';
	console::write(line, static_cast<std::size_t>(length));
}

}  // namespace

// frame: r0, r1, r2, r3, r12, lr, pc, xpsr as the core stacked them.
extern "C" [[noreturn]] void gea_fault_report(const std::uint32_t *frame) {
	console::write("\ngea.fault\n", 11);
	writeHex(" pc=", frame[6]);
	writeHex(" lr=", frame[5]);
	writeHex(" cfsr=", SCB->CFSR);
	writeHex(" hfsr=", SCB->HFSR);
	writeHex(" mmfar=", SCB->MMFAR);
	writeHex(" bfar=", SCB->BFAR);

	clock::delayMicros(2'000'000);
	NVIC_SystemReset();
}

extern "C" __attribute__((naked)) void HardFault_Handler() {
	__asm volatile(
		"tst lr, #4\n"
		"ite eq\n"
		"mrseq r0, msp\n"
		"mrsne r0, psp\n"
		"b gea_fault_report\n");
}

extern "C" void MemManage_Handler() __attribute__((alias("HardFault_Handler")));
extern "C" void BusFault_Handler() __attribute__((alias("HardFault_Handler")));
extern "C" void UsageFault_Handler() __attribute__((alias("HardFault_Handler")));

// Replaces libstdc++'s verbose handler, which would link the C++ demangler
// into internal flash just to name the exception type.
namespace __gnu_cxx {

void __verbose_terminate_handler() {
	console::write("\ngea.terminate uncaught exception\n", 34);
	clock::delayMicros(2'000'000);
	NVIC_SystemReset();
}

}  // namespace __gnu_cxx
