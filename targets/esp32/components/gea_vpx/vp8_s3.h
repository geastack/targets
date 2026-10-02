// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Interpolation needs 21 aligned 32-byte rows; deblocking reuses 128 bytes.
enum { GEA_VP8_WORKSPACE_BYTES = 21 * 32 };
void* gea_vp8_workspace_s3(void);
void* gea_vp8_bind_workspace_s3(void*);
#define GEA_VP8_DECLARE_COPY(size) \
  void vp8_copy_mem##size##_s3(unsigned char*, int, unsigned char*, int);
GEA_VP8_DECLARE_COPY(16x16)
GEA_VP8_DECLARE_COPY(8x8)
GEA_VP8_DECLARE_COPY(8x4)
#undef GEA_VP8_DECLARE_COPY
#define GEA_VP8_DECLARE_FILTER(kind, size) \
  void vp8_##kind##_predict##size##_s3(unsigned char*, int, int, int, unsigned char*, int);
GEA_VP8_DECLARE_FILTER(sixtap, 4x4)
GEA_VP8_DECLARE_FILTER(sixtap, 8x4)
GEA_VP8_DECLARE_FILTER(sixtap, 8x8)
GEA_VP8_DECLARE_FILTER(sixtap, 16x16)
GEA_VP8_DECLARE_FILTER(bilinear, 4x4)
GEA_VP8_DECLARE_FILTER(bilinear, 8x4)
GEA_VP8_DECLARE_FILTER(bilinear, 8x8)
GEA_VP8_DECLARE_FILTER(bilinear, 16x16)
#undef GEA_VP8_DECLARE_FILTER
void vp8_short_idct4x4llm_s3(short*, unsigned char*, int, unsigned char*, int);
void vp8_dequant_idct_add_s3(short*, short*, unsigned char*, int);
void vp8_dequant_idct_add_y_block_s3(short*, short*, unsigned char*, int, char*);
void vp8_dequant_idct_add_uv_block_s3(short*, short*, unsigned char*, unsigned char*, int, char*);
void vp8_dc_only_idct_add_s3(short, unsigned char*, int, unsigned char*, int);
void vp8_short_inv_walsh4x4_s3(short*, short*);
#define GEA_VP8_DECLARE_TM(size) \
  void vpx_tm_predictor_##size##x##size##_s3(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*);
GEA_VP8_DECLARE_TM(4)
GEA_VP8_DECLARE_TM(8)
GEA_VP8_DECLARE_TM(16)
#undef GEA_VP8_DECLARE_TM
struct loop_filter_info;
#define GEA_DECLARE_LOOP(name) \
  void vp8_loop_filter_##name##_s3(unsigned char*, unsigned char*, unsigned char*, int, int, struct loop_filter_info*); \
  void vp8_loop_filter_simple_##name##_s3(unsigned char*, int, const unsigned char*);
GEA_DECLARE_LOOP(bh)
GEA_DECLARE_LOOP(bv)
GEA_DECLARE_LOOP(mbh)
GEA_DECLARE_LOOP(mbv)
#undef GEA_DECLARE_LOOP
struct gea_vp8_entropy_result {
  unsigned cases;
  uint64_t scalar_us;
  uint64_t native_us;
};
int gea_vp8_entropy_check_s3(struct gea_vp8_entropy_result*);
#ifdef __cplusplus
}
#endif
