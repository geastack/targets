// SPDX-License-Identifier: Apache-2.0
// ESP32-S3 PIE: eight I420 pixels per iteration, exact limited-range BT.601.
// No intermediate image, floating point, or reduced-precision coefficients.
// Inputs: Y 8-byte aligned, U/V 4-byte aligned, output 8-byte aligned;
// count is a positive multiple of eight. Reads exactly the requested pixels.
// Q registers/QACC/SAR are caller-saved and preserved by the IDF task switch.
    .section .rodata.gea_i420_constants,"a",@progbits
    .balign 16
.Li420_coefficients:
    .short 298, 409, -100, -208, 516, 0, 0, 0
.Li420_offsets:
    .short 16, 128, 31, 63

    .text
    // Separate entry points specialize the byte order at assembly time; no
    // per-pixel branch and no second PSRAM read/write pass for panel output.
    .macro GEA_I420_ROW name, panel, pair=0
    .align 4
    .global \name
    .type \name,@function
\name:
    // a2 Y; a3 U; a4 V; a5 RGB565; a6 count.
    entry a1, 32
    .if \pair
    // Sixth argument points to { second_Y, second_output }; S3 pointers are
    // 32-bit. Keep both advancing row pointers in registers for the block.
    l32i a12, a7, 0
    l32i a13, a7, 4
    .endif
    movi a7, .Li420_coefficients
    ee.vld.128.ip q7, a7, 16
    movi a8, 11
    movi a9, 10
    movi a10, 5
.Li420_loop_\@:
    ee.vld.l.64.ip q0, a2, 8
    ee.zero.q q3
    ee.vzip.8 q0, q3

    l32i a11, a3, 0
    addi a3, a3, 4
    ee.movi.32.q q1, a11, 0
    ee.zero.q q3
    ee.vzip.8 q1, q3
    ee.orq q3, q1, q1
    ee.vzip.16 q1, q3

    l32i a11, a4, 0
    addi a4, a4, 4
    ee.movi.32.q q2, a11, 0
    ee.zero.q q3
    ee.vzip.8 q2, q3
    ee.orq q3, q2, q2
    ee.vzip.16 q2, q3

    ee.vldbc.16.ip q3, a7, 2
    ee.vsubs.s16 q0, q0, q3
    ee.vldbc.16.ip q3, a7, 2
    ee.vsubs.s16 q1, q1, q3
    ee.vsubs.s16 q2, q2, q3
    .if \pair
    movi a14, 1
    .endif

.Li420_color_\@:
    // QACC starts at the rounding bias, 128. Shift directly to 5/6 bits;
    // this is identical to clamping RGB888 and then truncating to RGB565.
    ee.mov.s16.qacc q3
    ee.vsmulas.s16.qacc q0, q7, 0
    ee.vsmulas.s16.qacc q2, q7, 1
    ee.srcmb.s16.qacc q4, a8, 0
    ee.mov.s16.qacc q3
    ee.vsmulas.s16.qacc q0, q7, 0
    ee.vsmulas.s16.qacc q1, q7, 2
    ee.vsmulas.s16.qacc q2, q7, 3
    ee.srcmb.s16.qacc q5, a9, 0
    ee.mov.s16.qacc q3
    ee.vsmulas.s16.qacc q0, q7, 0
    ee.vsmulas.s16.qacc q1, q7, 4
    ee.srcmb.s16.qacc q6, a8, 0

    ee.zero.q q3
    ee.vmax.s16 q4, q4, q3
    ee.vmax.s16 q5, q5, q3
    ee.vmax.s16 q6, q6, q3
    ee.vldbc.16.ip q3, a7, 2
    ee.vmin.s16 q4, q4, q3
    ee.vmin.s16 q6, q6, q3
    ee.vldbc.16 q3, a7
    addi a7, a7, -6
    ee.vmin.s16 q5, q5, q3
    wsr.sar a8
    ee.vsl.32 q4, q4
    wsr.sar a10
    ee.vsl.32 q5, q5
    ee.orq q4, q4, q5
    ee.orq q4, q4, q6
    .if \panel
    // Deinterleave low/high bytes, then interleave in reversed order. This
    // packs the panel byte order in three register operations instead of
    // loading two masks and shifting/masking every eight pixels.
    ee.zero.q q0
    ee.vunzip.8 q4, q0
    ee.vzip.8 q0, q4
    ee.vst.l.64.ip q0, a5, 8
    ee.vst.h.64.ip q0, a5, 8
    .else
    ee.vst.l.64.ip q4, a5, 8
    ee.vst.h.64.ip q4, a5, 8
    .endif
    .if \pair
    // Both luma rows share these chroma lanes. Reuse the widened, centered
    // U/V registers instead of reloading and duplicating them for row two.
    mov a11, a5
    mov a5, a13
    mov a13, a11
    beqz a14, .Li420_pair_done_\@
    movi a14, 0
    ee.vld.l.64.ip q0, a12, 8
    ee.zero.q q3
    ee.vzip.8 q0, q3
    ee.vldbc.16.ip q3, a7, 2
    ee.vsubs.s16 q0, q0, q3
    ee.vldbc.16.ip q3, a7, 2
    j .Li420_color_\@
.Li420_pair_done_\@:
    .endif
    addi a6, a6, -8
    bnez a6, .Li420_loop_\@
    retw
    .size \name, .-\name
    .endm
    GEA_I420_ROW gea_i420_rgb565_row_s3, 0
    GEA_I420_ROW gea_i420_rgb565_row_be_s3, 1
    GEA_I420_ROW gea_i420_rgb565_rows_s3, 0, 1
    GEA_I420_ROW gea_i420_rgb565_rows_be_s3, 1, 1

// Eight RGB565 pixels per iteration. Mask after arithmetic right shift so
// the sign bit cannot contaminate the adjacent pixel's high byte.
    .section .rodata.gea_rgb565_swap,"a",@progbits
    .balign 16
.Lswap_low:
    .long 0x00ff00ff, 0x00ff00ff, 0x00ff00ff, 0x00ff00ff
.Lswap_high:
    .long 0xff00ff00, 0xff00ff00, 0xff00ff00, 0xff00ff00
    .text
    .align 4
    .global gea_rgb565_swap_s3
    .type gea_rgb565_swap_s3,@function
gea_rgb565_swap_s3:
    entry a1, 32
    movi a4, .Lswap_low
    ee.vld.128.ip q2, a4, 16
    ee.vld.128.ip q3, a4, 16
    movi a5, 8
    wsr.sar a5
.Lswap_loop:
    ee.vld.128.ip q0, a2, 0
    ee.orq q1, q0, q0
    ee.vsr.32 q1, q1
    ee.andq q1, q1, q2
    ee.vsl.32 q0, q0
    ee.andq q0, q0, q3
    ee.orq q0, q0, q1
    ee.vst.128.ip q0, a2, 16
    addi a3, a3, -8
    bnez a3, .Lswap_loop
    retw
    .size gea_rgb565_swap_s3, .-gea_rgb565_swap_s3
