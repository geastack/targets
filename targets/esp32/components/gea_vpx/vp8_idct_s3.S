// SPDX-License-Identifier: Apache-2.0
// Bit-exact VP8 4x4 inverse DCT. Four transforms run in parallel in 32-bit
// lanes: no intermediate saturating 16-bit additions, even for extreme inputs.
    .section .rodata.gea_vp8_idct_constants,"a",@progbits
    .balign 4
.Lidct_constants:
    .short 20091, -30068
    .long 4, 255

    .macro sx16 reg
    ee.vsl.32 \reg, \reg
    ee.vsr.32 \reg, \reg
    .endm

    .macro transpose4
    ee.vzip.32 q0, q2
    ee.vzip.32 q1, q3
    ee.vzip.32 q0, q1
    ee.vzip.32 q2, q3
    .endm

    .macro idct_pass
    // SAR=16. sin coefficient 35468 = 65536-30068, exactly, including
    // negative rounding: (x*35468)>>16 == x + ((x*-30068)>>16).
    ee.vadds.s32 q5, q0, q2
    ee.vsubs.s32 q0, q0, q2
    ee.vldbc.16 q4, a11
    ee.vmul.s16 q6, q1, q4
    sx16 q6
    ee.vadds.s32 q6, q6, q1
    ee.vldbc.16 q4, a7
    ee.vmul.s16 q7, q3, q4
    sx16 q7
    ee.vadds.s32 q7, q7, q3
    ee.vsubs.s32 q6, q6, q7
    ee.vmul.s16 q7, q1, q4
    sx16 q7
    ee.vadds.s32 q1, q1, q7
    ee.vldbc.16 q4, a11
    ee.vmul.s16 q7, q3, q4
    sx16 q7
    ee.vadds.s32 q3, q3, q7
    ee.vadds.s32 q1, q1, q3
    ee.vsubs.s32 q2, q0, q6
    ee.vadds.s32 q0, q0, q6
    ee.vsubs.s32 q3, q5, q1
    ee.vadds.s32 q1, q5, q1
    ee.orq q4, q0, q0
    ee.orq q0, q1, q1
    ee.orq q1, q4, q4
    .endm

    .macro load_row reg
    ee.vld.l.64.ip \reg, a2, 8
    ee.zero.q q4
    ee.vzip.16 \reg, q4
    sx16 \reg
    .endm

    .macro output_row reg
    // Scalar loads/stores only transfer a packed 4-pixel word. Prediction,
    // saturation and packing are vector operations.
    l32i a12, a3, 0
    ee.movi.32.q q4, a12, 0
    ee.zero.q q5
    ee.vzip.8 q4, q5
    ee.zero.q q5
    ee.vzip.16 q4, q5
    // ZIP overwrites both operands; its upper-half result is not a zero vector.
    ee.zero.q q5
    ee.vadds.s32 \reg, \reg, q4
    ee.vmax.s32 \reg, \reg, q5
    ee.vmin.s32 \reg, \reg, q6
    ee.vunzip.16 \reg, q5
    ee.zero.q q5
    ee.vunzip.8 \reg, q5
    ee.movi.32.a \reg, a12, 0
    s32i a12, a5, 0
    add a3, a3, a4
    add a5, a5, a6
    .endm

    .text
    .align 4
    .global vp8_short_idct4x4llm_s3
    .type vp8_short_idct4x4llm_s3,@function
vp8_short_idct4x4llm_s3:
    // input 8-byte aligned; prediction/destination 4-byte aligned and strides.
    entry a1, 32
    movi a7, .Lidct_constants
    addi a11, a7, 2
    movi a8, 16
    wsr.sar a8
    load_row q0
    load_row q1
    load_row q2
    load_row q3
.Lidct_loaded:
    idct_pass
    // Upstream stores the first pass into short[16]: wrap, do not saturate.
    sx16 q0
    sx16 q1
    sx16 q2
    sx16 q3
    transpose4
    idct_pass
    addi a7, a7, 4
    ee.vldbc.32.ip q4, a7, 4
    ee.vadds.s32 q0, q0, q4
    ee.vadds.s32 q1, q1, q4
    ee.vadds.s32 q2, q2, q4
    ee.vadds.s32 q3, q3, q4
    movi a9, 3
    wsr.sar a9
    ee.vsr.32 q0, q0
    ee.vsr.32 q1, q1
    ee.vsr.32 q2, q2
    ee.vsr.32 q3, q3
    wsr.sar a8
    sx16 q0
    sx16 q1
    sx16 q2
    sx16 q3
    transpose4
    ee.vldbc.32 q6, a7
    output_row q0
    output_row q1
    output_row q2
    output_row q3
    retw
    .size vp8_short_idct4x4llm_s3, .-vp8_short_idct4x4llm_s3

    .align 4
    .global vp8_dequant_idct_add_s3
    .type vp8_dequant_idct_add_s3,@function
vp8_dequant_idct_add_s3:
    // input, quantizers, pixels, stride. Keep dequantized coefficients in
    // PIE registers: no write/reload of an intermediate PSRAM block.
    entry a1, 32
    mov a10, a2
    mov a13, a4
    mov a14, a5
    movi a8, 0
    wsr.sar a8
    ee.vld.128.ip q0, a2, 16
    ee.vld.128.ip q1, a2, 16
    ee.vld.128.ip q4, a3, 16
    ee.vld.128.ip q5, a3, 16
    ee.vmul.s16 q0, q0, q4
    ee.vmul.s16 q1, q1, q5
    ee.zero.q q2
    ee.zero.q q3
    ee.vzip.16 q0, q2
    ee.vzip.16 q1, q3
    ee.orq q4, q1, q1
    ee.orq q1, q2, q2
    ee.orq q2, q4, q4
    movi a8, 16
    wsr.sar a8
    sx16 q0
    sx16 q1
    sx16 q2
    sx16 q3
    // Match upstream's coefficient clearing, with exactly two native stores.
    ee.zero.q q5
    ee.vst.128.ip q5, a10, 16
    ee.vst.128.ip q5, a10, 16
    mov a3, a13
    mov a4, a14
    mov a5, a13
    mov a6, a14
    movi a7, .Lidct_constants
    addi a11, a7, 2
    j .Lidct_loaded
    .size vp8_dequant_idct_add_s3, .-vp8_dequant_idct_add_s3

    .align 4
    .global vp8_dc_only_idct_add_s3
    .type vp8_dc_only_idct_add_s3,@function
vp8_dc_only_idct_add_s3:
    entry a1, 32
    // DC-only residual is bounded to [-4096,4096] after rounding, so eight
    // signed 16-bit lanes suffice even with the 255-valued prediction. Full
    // transforms still use 32-bit lanes to preserve their extreme-input ABI.
    addi a2, a2, 4
    srai a2, a2, 3
    s16i a2, a1, 0
    ee.vldbc.16 q0, a1
    movi a7, .Lidct_constants+8
    ee.vldbc.16 q6, a7
    ee.zero.q q7
    movi a9, 4
.Ldc_row:
    l32i a8, a3, 0
    ee.movi.32.q q1, a8, 0
    ee.zero.q q2
    ee.vzip.8 q1, q2
    ee.vadds.s16 q1, q1, q0
    ee.vmax.s16 q1, q1, q7
    ee.vmin.s16 q1, q1, q6
    ee.zero.q q2
    ee.vunzip.8 q1, q2
    ee.movi.32.a q1, a8, 0
    s32i a8, a5, 0
    add a3, a3, a4
    add a5, a5, a6
    addi a9, a9, -1
    bnez a9, .Ldc_row
    retw
    .size vp8_dc_only_idct_add_s3, .-vp8_dc_only_idct_add_s3

    .macro walsh_pass
    ee.vadds.s32 q4, q0, q3
    ee.vadds.s32 q5, q1, q2
    ee.vsubs.s32 q6, q1, q2
    ee.vsubs.s32 q7, q0, q3
    ee.vadds.s32 q0, q4, q5
    ee.vadds.s32 q1, q6, q7
    ee.vsubs.s32 q2, q4, q5
    ee.vsubs.s32 q3, q7, q6
    .endm
    .macro scatter_dc reg
    ee.movi.32.a \reg, a4, 0
    s16i a4, a3, 0
    ee.movi.32.a \reg, a4, 1
    s16i a4, a3, 32
    ee.movi.32.a \reg, a4, 2
    s16i a4, a3, 64
    ee.movi.32.a \reg, a4, 3
    s16i a4, a3, 96
    addi a3, a3, 128
    .endm
    .align 4
    .global vp8_short_inv_walsh4x4_s3
    .type vp8_short_inv_walsh4x4_s3,@function
vp8_short_inv_walsh4x4_s3:
    entry a1, 32
    movi a8, 16
    wsr.sar a8
    load_row q0
    load_row q1
    load_row q2
    load_row q3
    walsh_pass
    sx16 q0
    sx16 q1
    sx16 q2
    sx16 q3
    transpose4
    walsh_pass
    movi a4, 3
    s32i a4, a1, 0
    ee.vldbc.32 q4, a1
    ee.vadds.s32 q0, q0, q4
    ee.vadds.s32 q1, q1, q4
    ee.vadds.s32 q2, q2, q4
    ee.vadds.s32 q3, q3, q4
    wsr.sar a4
    ee.vsr.32 q0, q0
    ee.vsr.32 q1, q1
    ee.vsr.32 q2, q2
    ee.vsr.32 q3, q3
    transpose4
    scatter_dc q0
    scatter_dc q1
    scatter_dc q2
    scatter_dc q3
    retw
    .size vp8_short_inv_walsh4x4_s3, .-vp8_short_inv_walsh4x4_s3
