// SPDX-License-Identifier: Apache-2.0
// True-motion intra prediction: clip(left[row] + above[column] - top_left).
// Eight columns run in signed 16-bit PIE lanes, with exact saturation to bytes.
    .section .rodata.gea_vp8_tm,"a",@progbits
    .balign 4
.Ltm_max:
    .short 255
    .text
    .align 4
    .global gea_vp8_tm_s3
    .type gea_vp8_tm_s3,@function
gea_vp8_tm_s3:
    // dst (4-byte aligned), stride, aligned above, left, block size (4/8/16).
    entry a1, 32
    addi a9, a4, -1
    l8ui a8, a9, 0
    ee.zero.q q5
    movi a9, .Ltm_max
    ee.vldbc.16 q6, a9
    beqi a6, 4, .Ltm_four
    ee.vld.l.64.ip q0, a4, 8
    ee.vzip.8 q0, q5
    beqi a6, 8, .Ltm_ready
    ee.zero.q q5
    ee.vld.l.64.ip q1, a4, 8
    ee.vzip.8 q1, q5
    j .Ltm_ready
.Ltm_four:
    l32i a9, a4, 0
    ee.movi.32.q q0, a9, 0
    ee.vzip.8 q0, q5
.Ltm_ready:
    mov a10, a6
.Ltm_row:
    l8ui a9, a5, 0
    addi a5, a5, 1
    sub a9, a9, a8
    s16i a9, a1, 0
    ee.vldbc.16 q2, a1
    ee.zero.q q5
    ee.vadds.s16 q3, q0, q2
    ee.vmax.s16 q3, q3, q5
    ee.vmin.s16 q3, q3, q6
    ee.vunzip.8 q3, q5
    ee.movi.32.a q3, a9, 0
    s32i a9, a2, 0
    beqi a6, 4, .Ltm_next
    ee.movi.32.a q3, a9, 1
    s32i a9, a2, 4
    beqi a6, 8, .Ltm_next
    ee.zero.q q5
    ee.vadds.s16 q3, q1, q2
    ee.vmax.s16 q3, q3, q5
    ee.vmin.s16 q3, q3, q6
    ee.vunzip.8 q3, q5
    ee.movi.32.a q3, a9, 0
    s32i a9, a2, 8
    ee.movi.32.a q3, a9, 1
    s32i a9, a2, 12
.Ltm_next:
    add a2, a2, a3
    addi a10, a10, -1
    bnez a10, .Ltm_row
    retw
    .size gea_vp8_tm_s3, .-gea_vp8_tm_s3
