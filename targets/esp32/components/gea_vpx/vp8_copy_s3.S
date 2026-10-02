// SPDX-License-Identifier: Apache-2.0
// Integer-motion copy for libvpx's padded reference planes. One window entry
// per complete block, arbitrary source alignment, 4-byte aligned destination.
    .text
    .align 4
    .global gea_vp8_copy_rect_s3
    .type gea_vp8_copy_rect_s3,@function
gea_vp8_copy_rect_s3:
    // src, src stride, dst, dst stride, width (4/8/16), height
    entry a1, 32
.Lcopy_row:
    mov a8, a2
    ee.ld.128.usar.ip q0, a8, 16
    ee.ld.128.usar.ip q1, a8, 16
    ee.src.q.qup q2, q0, q1
    bnei a6, 16, .Lcopy_words
    extui a8, a4, 0, 4
    bnez a8, .Lcopy_words
    mov a8, a4
    ee.vst.128.ip q2, a8, 16
    j .Lcopy_next
.Lcopy_words:
    ee.movi.32.a q2, a8, 0
    s32i a8, a4, 0
    beqi a6, 4, .Lcopy_next
    ee.movi.32.a q2, a8, 1
    s32i a8, a4, 4
    beqi a6, 8, .Lcopy_next
    ee.movi.32.a q2, a8, 2
    s32i a8, a4, 8
    ee.movi.32.a q2, a8, 3
    s32i a8, a4, 12
.Lcopy_next:
    add a2, a2, a3
    add a4, a4, a5
    addi a7, a7, -1
    bnez a7, .Lcopy_row
    retw
    .size gea_vp8_copy_rect_s3, .-gea_vp8_copy_rect_s3
