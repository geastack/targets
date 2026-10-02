// SPDX-License-Identifier: Apache-2.0
// Eight parallel VP8 deblocking lanes. data holds p3..q3 as eight aligned
// vectors of unsigned pixel values widened to int16. mode: 0 block, 1 MB,
// 2 simple. Returns zero when every lane has zero correction, allowing the
// wrapper to avoid scattering an unchanged edge back into reference PSRAM.
    .section .rodata.gea_vp8_loop_constants,"a",@progbits
    .balign 16
.Lloop_constants:
    .short -128, 127, 255, -1, 4, 3, 1, 63, 27, 18, 9

    .macro loadtap reg, offset
    addi a8, a2, \offset
    ee.vld.128.ip \reg, a8, 0
    .endm
    .macro storetap reg, offset
    addi a8, a2, \offset
    ee.vst.128.ip \reg, a8, 0
    .endm
    .macro constant reg, offset
    addi a8, a7, \offset
    ee.vldbc.16 \reg, a8
    .endm
    .macro absdiff dest, x, y, temp
    ee.vsubs.s16 \dest, \x, \y
    ee.vsubs.s16 \temp, \y, \x
    ee.vmax.s16 \dest, \dest, \temp
    .endm
    .macro signed_clip reg
    ee.vmax.s16 \reg, \reg, q6
    ee.vmin.s16 \reg, \reg, q7
    .endm
    .macro shift_right reg, bits
    movi a9, \bits
    ee.mov.s16.qacc \reg
    ee.srcmb.s16.qacc \reg, a9, 0
    .endm
    .macro update_pixels delta, p, q
    loadtap q5, \p
    ee.vadds.s16 q5, q5, \delta
    ee.vmax.s16 q5, q5, q6
    ee.vmin.s16 q5, q5, q7
    storetap q5, \p
    loadtap q5, \q
    ee.vsubs.s16 q5, q5, \delta
    ee.vmax.s16 q5, q5, q6
    ee.vmin.s16 q5, q5, q7
    storetap q5, \q
    .endm
    .macro wide_delta coefficient
    constant q0, \coefficient
    movi a9, 0
    wsr.sar a9
    ee.vmul.s16 q0, q4, q0
    ee.vadds.s16 q0, q0, q2
    shift_right q0, 7
    .endm

    .text
    .align 4
    .global gea_vp8_loopfilter8_s3
    .type gea_vp8_loopfilter8_s3,@function
gea_vp8_loopfilter8_s3:
    // a2 data; a3 limit; a4 blimit; a5 hev threshold; a6 mode
    entry a1, 32
    s16i a3, a1, 0
    s16i a4, a1, 2
    s16i a5, a1, 4
    movi a7, .Lloop_constants

    // Edge metric: 2*abs(p0-q0) + floor(abs(p1-q1)/2).
    loadtap q0, 48
    loadtap q1, 64
    absdiff q2, q0, q1, q3
    ee.vadds.s16 q2, q2, q2
    loadtap q0, 32
    loadtap q1, 80
    absdiff q4, q0, q1, q3
    shift_right q4, 1
    ee.vadds.s16 q2, q2, q4
    addi a8, a1, 2
    ee.vldbc.16 q4, a8
    ee.vcmp.gt.s16 q2, q2, q4
    beqi a6, 2, .Lloop_simple_mask

    // Maximum of adjacent differences on either side of the boundary.
    ee.zero.q q3
    mov a10, a2
    movi a11, 2
.Lloop_side:
    ee.vld.128.ip q0, a10, 16
    movi a12, 3
.Lloop_adjacent:
    ee.vld.128.ip q1, a10, 16
    absdiff q4, q0, q1, q5
    ee.vmax.s16 q3, q3, q4
    ee.orq q0, q1, q1
    addi a12, a12, -1
    bnez a12, .Lloop_adjacent
    addi a11, a11, -1
    bnez a11, .Lloop_side
    ee.vldbc.16 q4, a1
    ee.vcmp.gt.s16 q3, q3, q4
    ee.orq q2, q2, q3

    loadtap q0, 32
    loadtap q1, 48
    absdiff q3, q0, q1, q4
    loadtap q0, 64
    loadtap q1, 80
    absdiff q4, q0, q1, q5
    ee.vmax.s16 q3, q3, q4
    addi a8, a1, 4
    ee.vldbc.16 q4, a8
    ee.vcmp.gt.s16 q3, q3, q4
    j .Lloop_mask_ready
.Lloop_simple_mask:
    constant q3, 6
.Lloop_mask_ready:
    constant q4, 6
    ee.xorq q2, q2, q4
    constant q6, 0
    constant q7, 2
    loadtap q0, 32
    loadtap q1, 80
    ee.vsubs.s16 q4, q0, q1
    signed_clip q4
    bnez a6, .Lloop_outer_ready
    ee.andq q4, q4, q3
.Lloop_outer_ready:
    loadtap q0, 64
    loadtap q1, 48
    ee.vsubs.s16 q0, q0, q1
    ee.vadds.s16 q5, q0, q0
    ee.vadds.s16 q0, q0, q5
    ee.vadds.s16 q4, q4, q0
    signed_clip q4
    ee.andq q4, q4, q2
    // Flat areas and rejected edges need no update. Reduce the eight signed
    // 16-bit corrections without storing the vector to the PSRAM stack.
    ee.movi.32.a q4, a10, 0
    ee.movi.32.a q4, a11, 1
    or a10, a10, a11
    ee.movi.32.a q4, a11, 2
    or a10, a10, a11
    ee.movi.32.a q4, a11, 3
    or a10, a10, a11
    beqz a10, .Lloop_unchanged
    ee.orq q5, q4, q4
    bnei a6, 1, .Lloop_narrow_ready
    ee.andq q5, q5, q3
.Lloop_narrow_ready:
    constant q0, 8
    constant q1, 10
    ee.vadds.s16 q0, q5, q0
    ee.vadds.s16 q1, q5, q1
    signed_clip q0
    signed_clip q1
    shift_right q0, 3
    shift_right q1, 3
    ee.zero.q q6
    constant q7, 4
    loadtap q5, 48
    ee.vadds.s16 q5, q5, q1
    ee.vmax.s16 q5, q5, q6
    ee.vmin.s16 q5, q5, q7
    storetap q5, 48
    loadtap q5, 64
    ee.vsubs.s16 q5, q5, q0
    ee.vmax.s16 q5, q5, q6
    ee.vmin.s16 q5, q5, q7
    storetap q5, 64
    beqi a6, 2, .Lloop_done
    constant q1, 6
    ee.xorq q3, q3, q1
    beqi a6, 1, .Lloop_wide

    constant q1, 12
    ee.vadds.s16 q0, q0, q1
    shift_right q0, 1
    ee.andq q0, q0, q3
    update_pixels q0, 32, 80
    j .Lloop_done
.Lloop_wide:
    ee.andq q4, q4, q3
    constant q2, 14
    wide_delta 16
    update_pixels q0, 48, 64
    wide_delta 18
    update_pixels q0, 32, 80
    wide_delta 20
    update_pixels q0, 16, 96
.Lloop_done:
    movi a2, 1
    retw
.Lloop_unchanged:
    movi a2, 0
    retw
    .size gea_vp8_loopfilter8_s3, .-gea_vp8_loopfilter8_s3

    .macro edge_load_row reg
    mov a6, a2
    ee.ld.128.usar.ip q6, a6, 16
    ee.ld.128.usar.ip q7, a6, 16
    ee.src.q.qup \reg, q6, q7
    add a2, a2, a3
    .endm
    .macro edge_load_lanes reg
    ee.vld.128.ip \reg, a4, 16
    ee.zero.q q5
    ee.vunzip.8 \reg, q5
    .endm
    .macro edge_transpose load
    \load q0
    \load q1
    ee.vzip.8 q0, q1
    \load q1
    \load q2
    ee.vzip.8 q1, q2
    \load q2
    \load q3
    ee.vzip.8 q2, q3
    \load q3
    \load q4
    ee.vzip.8 q3, q4
    ee.vzip.16 q0, q1
    ee.vzip.16 q2, q3
    ee.vzip.32 q0, q2
    ee.vzip.32 q1, q3
    .endm
    .macro edge_save_lanes reg
    ee.zero.q q5
    ee.vzip.8 \reg, q5
    ee.vst.128.ip \reg, a4, 16
    ee.vst.128.ip q5, a4, 16
    .endm
    .align 4
    .global gea_vp8_edge_load_s3
    .type gea_vp8_edge_load_s3,@function
gea_vp8_edge_load_s3:
    // a2 first p3 pixel; a3 image stride; a4 lanes; a5 horizontal
    entry a1, 32
    beqz a5, .Ledge_load_vertical
    movi a7, 8
.Ledge_load_horizontal:
    // q7 is a vector register and does not alias the a7 loop counter.
    edge_load_row q0
    ee.zero.q q1
    ee.vzip.8 q0, q1
    ee.vst.128.ip q0, a4, 16
    addi a7, a7, -1
    bnez a7, .Ledge_load_horizontal
    retw
.Ledge_load_vertical:
    edge_transpose edge_load_row
    edge_save_lanes q0
    edge_save_lanes q2
    edge_save_lanes q1
    edge_save_lanes q3
    retw
    .size gea_vp8_edge_load_s3, .-gea_vp8_edge_load_s3

    .macro edge_store_row reg, lo, hi
    ee.movi.32.a \reg, a6, \lo
    ee.movi.32.a \reg, a7, \hi
    extui a8, a2, 0, 2
    bnez a8, 1f
    s32i a6, a2, 0
    s32i a7, a2, 4
    j 2f
1:
    s8i a6, a2, 0
    srli a6, a6, 8
    s8i a6, a2, 1
    srli a6, a6, 8
    s8i a6, a2, 2
    srli a6, a6, 8
    s8i a6, a2, 3
    s8i a7, a2, 4
    srli a7, a7, 8
    s8i a7, a2, 5
    srli a7, a7, 8
    s8i a7, a2, 6
    srli a7, a7, 8
    s8i a7, a2, 7
2:
    add a2, a2, a3
    .endm
    .align 4
    .global gea_vp8_edge_store_s3
    .type gea_vp8_edge_store_s3,@function
gea_vp8_edge_store_s3:
    entry a1, 32
    beqz a5, .Ledge_store_vertical
    // Horizontal edges occupy separate reference-plane rows/cache lines.
    // Write only the rows this filter can modify: p1..q1 normal, p2..q2 MB,
    // p0..q0 simple. Vertical edges retain their aligned packed row stores.
    // a6 mode must be consumed before edge_store_row uses it as scratch.
    beqi a6, 1, .Ledge_store_horizontal_mb
    beqi a6, 2, .Ledge_store_horizontal_simple
    add a2, a2, a3
    add a2, a2, a3
    addi a4, a4, 32
    movi a9, 4
    j .Ledge_store_horizontal
.Ledge_store_horizontal_mb:
    add a2, a2, a3
    addi a4, a4, 16
    movi a9, 6
    j .Ledge_store_horizontal
.Ledge_store_horizontal_simple:
    add a2, a2, a3
    add a2, a2, a3
    add a2, a2, a3
    addi a4, a4, 48
    movi a9, 2
.Ledge_store_horizontal:
    edge_load_lanes q0
    edge_store_row q0, 0, 1
    addi a9, a9, -1
    bnez a9, .Ledge_store_horizontal
    retw
.Ledge_store_vertical:
    edge_transpose edge_load_lanes
    edge_store_row q0, 0, 1
    edge_store_row q0, 2, 3
    edge_store_row q2, 0, 1
    edge_store_row q2, 2, 3
    edge_store_row q1, 0, 1
    edge_store_row q1, 2, 3
    edge_store_row q3, 0, 1
    edge_store_row q3, 2, 3
    retw
    .size gea_vp8_edge_store_s3, .-gea_vp8_edge_store_s3
