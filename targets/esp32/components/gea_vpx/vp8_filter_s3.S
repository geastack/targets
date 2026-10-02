// SPDX-License-Identifier: Apache-2.0
// Eight parallel VP8 subpixel convolutions (bilinear or six-tap).
// Source has libvpx's padded border; destination is at least 4-byte aligned.
    .section .rodata.gea_vp8_filter_constants,"a",@progbits
    .balign 4
.Lfilter_constants:
    .short 64, 255
    .text
    .align 4
    .global gea_vp8_filter_rows_s3
    .type gea_vp8_filter_rows_s3,@function
gea_vp8_filter_rows_s3:
    // src, source row stride, coefficients, dst, destination row stride,
    // packed: height<<8 | vertical<<4 | eight_lanes<<3 | taps.
    // Constants stay in PIE registers across every row of this block.
    entry a1, 32
    movi a8, .Lfilter_constants
    ee.vldbc.16.ip q6, a8, 2
    ee.vldbc.16 q5, a8
    srli a9, a7, 8
    movi a14, 1
    bbci a7, 4, .Lfilter_row
    mov a14, a3
.Lfilter_row:
    mov a11, a2
    mov a12, a4
    extui a10, a7, 0, 3
    ee.mov.s16.qacc q6
    loopnez a10, .Lfilter_tap_end
.Lfilter_tap:
    mov a8, a11
    ee.ld.128.usar.ip q1, a8, 16
    ee.ld.128.usar.ip q2, a8, 16
    ee.src.q.qup q0, q1, q2
    ee.zero.q q3
    ee.vzip.8 q0, q3
    ee.vldbc.16.ip q7, a12, 2
    ee.vmulas.s16.qacc q0, q7
    add a11, a11, a14
.Lfilter_tap_end:
    movi a8, 7
    ee.srcmb.s16.qacc q0, a8, 0
    ee.zero.q q4
    ee.vmax.s16 q0, q0, q4
    ee.vmin.s16 q0, q0, q5
    ee.vunzip.8 q0, q4
    ee.movi.32.a q0, a8, 0
    s32i a8, a5, 0
    bbci a7, 3, .Lfilter_next_row
    ee.movi.32.a q0, a8, 1
    s32i a8, a5, 4
.Lfilter_next_row:
    add a2, a2, a3
    add a5, a5, a6
    addi a9, a9, -1
    bnez a9, .Lfilter_row
    retw
    .size gea_vp8_filter_rows_s3, .-gea_vp8_filter_rows_s3
