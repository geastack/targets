/*
 * C++ unwind entries in QSPI reach only +-1 GB, not the personality routine in
 * internal flash. QSPI archives are relinked to name this thunk instead
 * (gea_stm32_place_archive_in_qspi); the linker adds a veneer for the jump.
 * Assembly, so the thunk itself carries no unwind entry.
 */
    .syntax unified
    .thumb
    .section .qspi.text.personality, "ax", %progbits
    .global __gea_qspi_gxx_personality_v0
    .type __gea_qspi_gxx_personality_v0, %function
__gea_qspi_gxx_personality_v0:
    b.w __gxx_personality_v0
    .size __gea_qspi_gxx_personality_v0, . - __gea_qspi_gxx_personality_v0
