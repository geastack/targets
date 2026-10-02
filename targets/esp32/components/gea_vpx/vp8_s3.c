// SPDX-License-Identifier: Apache-2.0
#include "vp8_s3.h"
#include <stdint.h>
#include <stdalign.h>
#include <string.h>

// These are VP8's normative interpolation coefficients. Both passes round and
// clip separately; combining them would change reference pixels and cause drift.
static const int16_t six_tap[8][6] = {
  {0, 0, 128, 0, 0, 0}, {0, -6, 123, 12, -1, 0},
  {2, -11, 108, 36, -8, 1}, {0, -9, 93, 50, -6, 0},
  {3, -16, 77, 77, -16, 3}, {0, -6, 50, 93, -9, 0},
  {1, -8, 36, 108, -11, 2}, {0, -1, 12, 123, -6, 0}
};
static const int16_t bilinear[8][2] = {
  {128, 0}, {112, 16}, {96, 32}, {80, 48},
  {64, 64}, {48, 80}, {32, 96}, {16, 112}
};

extern void gea_vp8_filter_rows_s3(const uint8_t*, int, const int16_t*, uint8_t*, int, int);
extern void gea_vp8_copy_rect_s3(const uint8_t*, int, uint8_t*, int, int, int);

static void copy_rect(const uint8_t* src, int stride, uint8_t* dst, int pitch,
                      int width, int height) {
  // Codec reference planes have padded borders, so both aligned source loads
  // are valid even for fractional-address integer motion. Destination stores
  // write only the requested block, with scalar fallback for odd alignment.
  if (((uintptr_t)dst | (unsigned)pitch) & 3) {
    for (int y = 0; y < height; ++y) memcpy(dst + y*pitch, src + y*stride, width);
  } else {
    gea_vp8_copy_rect_s3(src, stride, dst, pitch, width, height);
  }
}

#define GEA_VP8_COPY(size, width, height) \
  void vp8_copy_mem##size##_s3(unsigned char* src, int stride, unsigned char* dst, int pitch) { \
    copy_rect(src, stride, dst, pitch, width, height); \
  }
GEA_VP8_COPY(16x16, 16, 16)
GEA_VP8_COPY(8x8, 8, 8)
GEA_VP8_COPY(8x4, 8, 4)
#undef GEA_VP8_COPY

static void filter_rows(const uint8_t* src, int stride, const int16_t* coefficients,
                         uint8_t* dst, int pitch, int taps, int lanes, int height, int vertical) {
  gea_vp8_filter_rows_s3(src, stride, coefficients, dst, pitch,
                         (height << 8) | (vertical << 4) | ((lanes == 8) << 3) | taps);
}

static void predict(uint8_t* src, int stride, int xoffset, int yoffset,
                    uint8_t* dst, int pitch, int width, int height, int taps) {
  // 32-byte rows provide safe vector loads at both edges. The original generic
  // path used up to 2016 bytes of int32 intermediates; this uses 672 bytes.
  alignas(16) uint8_t fallback[21][32];
  uint8_t (*intermediate)[32] = gea_vp8_workspace_s3();
  if (!intermediate) intermediate = fallback;
  // Odd six-tap phases have zero outer coefficients. Four taps give exactly
  // the same rounded result without loading/multiplying two unused borders.
  const int horizontal_taps = taps == 6 && (xoffset & 1) ? 4 : taps;
  const int vertical_taps = taps == 6 && (yoffset & 1) ? 4 : taps;
  const int horizontal_first = taps == 6 ? (horizontal_taps == 4 ? 1 : 2) : 0;
  const int vertical_first = taps == 6 ? (vertical_taps == 4 ? 1 : 2) : 0;
  const int16_t* horizontal = taps == 6 ? six_tap[xoffset] : bilinear[xoffset];
  const int16_t* vertical = taps == 6 ? six_tap[yoffset] : bilinear[yoffset];
  horizontal += horizontal_taps == 4;
  vertical += vertical_taps == 4;
  const int lanes = width == 4 ? 4 : 8;
  // An integer-pixel axis is an identity filter. Avoid an entire convolution
  // pass (and its intermediate writes) rather than multiplying by zero taps.
  if (!xoffset && !yoffset) {
    copy_rect(src, stride, dst, pitch, width, height);
    return;
  }
  if (!xoffset || !yoffset) {
    const int step = xoffset ? 1 : stride;
    const int first = xoffset ? horizontal_first : vertical_first;
    const int active_taps = xoffset ? horizontal_taps : vertical_taps;
    const int16_t* coefficients = xoffset ? horizontal : vertical;
    for (int x = 0; x < width; x += 8)
      filter_rows(src + x - first*step, stride, coefficients, dst + x, pitch,
                    active_taps, lanes, height, !xoffset);
    return;
  }
  for (int x = 0; x < width; x += 8)
    filter_rows(src - vertical_first * stride + x - horizontal_first, stride,
                  horizontal, intermediate[0] + x, 32, horizontal_taps, lanes,
                  height + vertical_taps - 1, 0);
  for (int x = 0; x < width; x += 8)
    filter_rows(intermediate[0] + x, 32, vertical, dst + x, pitch,
                  vertical_taps, lanes, height, 1);
}

#define GEA_VP8_FILTER(kind, size, w, h, taps) \
  void vp8_##kind##_predict##size##_s3(unsigned char* src, int stride, int x, int y, \
                                      unsigned char* dst, int pitch) { \
    predict(src, stride, x, y, dst, pitch, w, h, taps); \
  }
GEA_VP8_FILTER(sixtap, 4x4, 4, 4, 6)
GEA_VP8_FILTER(sixtap, 8x4, 8, 4, 6)
GEA_VP8_FILTER(sixtap, 8x8, 8, 8, 6)
GEA_VP8_FILTER(sixtap, 16x16, 16, 16, 6)
GEA_VP8_FILTER(bilinear, 4x4, 4, 4, 2)
GEA_VP8_FILTER(bilinear, 8x4, 8, 4, 2)
GEA_VP8_FILTER(bilinear, 8x8, 8, 8, 2)
GEA_VP8_FILTER(bilinear, 16x16, 16, 16, 2)
#undef GEA_VP8_FILTER

extern void gea_vp8_tm_s3(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*, int);
#define GEA_VP8_TM(size) \
  void vpx_tm_predictor_##size##x##size##_s3(uint8_t* dst, ptrdiff_t stride, \
                                           const uint8_t* above, const uint8_t* left) { \
    alignas(16) uint8_t top[32]; \
    top[15] = above[-1]; \
    memcpy(top + 16, above, size); \
    gea_vp8_tm_s3(dst, stride, top + 16, left, size); \
  }
GEA_VP8_TM(4)
GEA_VP8_TM(8)
GEA_VP8_TM(16)
#undef GEA_VP8_TM
