// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { GEA_VP8_PROFILE_TOKENS, GEA_VP8_PROFILE_MACROBLOCK,
       GEA_VP8_PROFILE_LOOPFILTER, GEA_VP8_PROFILE_STAGES };
struct gea_vp8_profile_s3 {
  uint64_t us[GEA_VP8_PROFILE_STAGES];
  uint32_t calls[GEA_VP8_PROFILE_STAGES];
};
void gea_vp8_profile_enable_s3(int enabled);
int gea_vp8_profile_requested_s3(void);
struct gea_vp8_profile_s3* gea_vp8_profile_bind_s3(struct gea_vp8_profile_s3* next);
uint64_t gea_vp8_profile_clock_s3(void);
void gea_vp8_profile_add_s3(unsigned stage, uint64_t started);
#ifdef __cplusplus
}
#endif
