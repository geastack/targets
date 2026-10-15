// 64-bit atomics for a core without LDREXD-based lock-free support in the
// toolchain's runtime: GCC lowers them to __atomic_*_8 library calls, which
// newlib does not provide. Each runs with interrupts masked, which makes it
// atomic on this single-core part.

#include "stm32f7xx.h"

#include <stdbool.h>
#include <stdint.h>

#pragma GCC diagnostic ignored "-Wbuiltin-declaration-mismatch"

static uint32_t enterCritical(void) {
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}

static void exitCritical(uint32_t primask) {
	__set_PRIMASK(primask);
}

uint64_t __atomic_load_8(const volatile void *ptr, int memorder) {
	(void)memorder;
	const uint32_t primask = enterCritical();
	const uint64_t value = *(const volatile uint64_t *)ptr;
	exitCritical(primask);
	return value;
}

void __atomic_store_8(volatile void *ptr, uint64_t value, int memorder) {
	(void)memorder;
	const uint32_t primask = enterCritical();
	*(volatile uint64_t *)ptr = value;
	exitCritical(primask);
}

uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t value, int memorder) {
	(void)memorder;
	const uint32_t primask = enterCritical();
	const uint64_t previous = *(volatile uint64_t *)ptr;
	*(volatile uint64_t *)ptr = value;
	exitCritical(primask);
	return previous;
}

bool __atomic_compare_exchange_8(volatile void *ptr, void *expected, uint64_t desired, bool weak, int success, int failure) {
	(void)weak;
	(void)success;
	(void)failure;
	const uint32_t primask = enterCritical();
	volatile uint64_t *target = (volatile uint64_t *)ptr;
	uint64_t *wanted = (uint64_t *)expected;
	const bool matched = *target == *wanted;
	if (matched) {
		*target = desired;
	} else {
		*wanted = *target;
	}
	exitCritical(primask);
	return matched;
}

#define GEA_ATOMIC_FETCH_OP_8(name, op)                                          \
	uint64_t __atomic_fetch_##name##_8(volatile void *ptr, uint64_t value, int memorder) { \
		(void)memorder;                                                          \
		const uint32_t primask = enterCritical();                                \
		const uint64_t previous = *(volatile uint64_t *)ptr;                     \
		*(volatile uint64_t *)ptr = previous op value;                           \
		exitCritical(primask);                                                   \
		return previous;                                                         \
	}

GEA_ATOMIC_FETCH_OP_8(add, +)
GEA_ATOMIC_FETCH_OP_8(sub, -)
GEA_ATOMIC_FETCH_OP_8(and, &)
GEA_ATOMIC_FETCH_OP_8(or, |)
GEA_ATOMIC_FETCH_OP_8(xor, ^)
