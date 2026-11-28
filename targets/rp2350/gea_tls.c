// newlib (the arm-none-eabi toolchain's libc) ships no __aeabi_read_tp, so the
// runtime's `thread_local`s fail to link. memmap_gea.ld defines the thread
// pointer for the linked TLS segment (.tdata, copied with .data, then .tbss).
// Like the SDK's picolibc path, both cores share that one block; the render
// worker on core1 touches no thread_local state.

// Naked: the EABI requires every register but r0 to survive this call.
__attribute__((naked, used)) void *__aeabi_read_tp(void)
{
	__asm volatile(
		"ldr r0, =__gea_tls_thread_pointer\n"
		"bx lr\n");
}
