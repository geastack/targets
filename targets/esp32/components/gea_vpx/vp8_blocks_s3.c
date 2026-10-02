// SPDX-License-Identifier: BSD-3-Clause
// Whole-macroblock reconstruction follows libvpx 1.16.0 idct_blk.c
// (Copyright (c) 2010 The WebM project authors). Upstream's C block functions
// call scalar leaf symbols directly, bypassing generated SIMD dispatch.
#include "vp8_s3.h"

static inline void add_block(short* q, short* dq, unsigned char* dst,
                             int stride, char eob) {
  if (eob > 1) {
    vp8_dequant_idct_add_s3(q, dq, dst, stride);
  } else {
    const short dc = (short)(q[0] * dq[0]);
    // A zero DC residual leaves all 16 predicted pixels untouched.
    if (dc) vp8_dc_only_idct_add_s3(dc, dst, stride, dst, stride);
    q[0] = q[1] = 0;
  }
}

void vp8_dequant_idct_add_y_block_s3(short* q, short* dq, unsigned char* dst,
                                     int stride, char* eobs) {
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      add_block(q, dq, dst + column*4, stride, *eobs++);
      q += 16;
    }
    dst += 4*stride;
  }
}

void vp8_dequant_idct_add_uv_block_s3(short* q, short* dq, unsigned char* dst_u,
                                      unsigned char* dst_v, int stride, char* eobs) {
  for (int plane = 0; plane < 2; ++plane) {
    unsigned char* dst = plane ? dst_v : dst_u;
    for (int row = 0; row < 2; ++row) {
      for (int column = 0; column < 2; ++column) {
        add_block(q, dq, dst + column*4, stride, *eobs++);
        q += 16;
      }
      dst += 4*stride;
    }
  }
}
