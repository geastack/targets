// C++ thread_local on a single-threaded board. The ARM EABI places a thread's
// TLS block 8 bytes (the thread control block) after its thread pointer, so
// with one thread the pointer is a link-time constant just below the block
// the linker script lays out, and the block needs the usual .data/.bss setup
// before the first constructor can touch a thread_local.

#include <cstdint>
#include <cstring>

extern "C" {

extern char __tdata_start[];
extern char __tdata_end[];
extern const char __tdata_load[];
extern char __tbss_start[];
extern char __tbss_end[];

// Called by compiler-generated code with a contract of clobbering only r0.
__attribute__((naked)) void __aeabi_read_tp() {
	__asm volatile(
		"ldr r0, =__tdata_start - 8\n"
		"bx lr\n");
}

}  // extern "C"

namespace {

void initThreadLocalStorage() {
	std::memcpy(__tdata_start, __tdata_load, static_cast<std::size_t>(__tdata_end - __tdata_start));
	std::memset(__tbss_start, 0, static_cast<std::size_t>(__tbss_end - __tbss_start));
}

__attribute__((section(".preinit_array"), used)) void (*const gInitThreadLocalStorage)() = initThreadLocalStorage;

}  // namespace
