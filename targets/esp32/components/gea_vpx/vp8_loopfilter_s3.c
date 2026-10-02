// SPDX-License-Identifier: Apache-2.0
#include "vp8_s3.h"
#include "vp8/common/loopfilter.h"
#include <stdint.h>
#include <stdalign.h>

extern int gea_vp8_loopfilter8_s3(int16_t*, int, int, int, int);
extern void gea_vp8_edge_load_s3(const uint8_t*, int, int16_t*, int);
extern void gea_vp8_edge_store_s3(uint8_t*, int, const int16_t*, int, int);

static void edge(uint8_t* pixels, int stride, int horizontal, int count,
                 int limit, int blimit, int threshold, int mode) {
  alignas(16) int16_t fallback[8][8];
  int16_t (*lanes)[8] = gea_vp8_workspace_s3();
  if (!lanes) lanes = fallback;
  const int along = horizontal ? 1 : stride;
  const int across = horizontal ? stride : 1;
  for (int block = 0; block < count; ++block) {
    gea_vp8_edge_load_s3(pixels - 4*across, stride, &lanes[0][0], horizontal);
    if (gea_vp8_loopfilter8_s3(&lanes[0][0], limit, blimit, threshold, mode))
      gea_vp8_edge_store_s3(pixels - 4*across, stride, &lanes[0][0], horizontal, mode);
    pixels += 8*along;
  }
}

static void filter(uint8_t* y, uint8_t* u, uint8_t* v, int ys, int uvs,
                   loop_filter_info* info, int horizontal, int mb) {
  const int limit = info->lim[0], threshold = info->hev_thr[0];
  const int blimit = mb ? info->mblim[0] : info->blim[0];
  const int yStep = horizontal ? ys : 1, uvStep = horizontal ? uvs : 1;
  if (mb) edge(y, ys, horizontal, 2, limit, blimit, threshold, 1);
  else for (int i = 4; i <= 12; i += 4)
    edge(y + i*yStep, ys, horizontal, 2, limit, blimit, threshold, 0);
  if (u) edge(u + (mb ? 0 : 4*uvStep), uvs, horizontal, 1, limit, blimit, threshold, mb);
  if (v) edge(v + (mb ? 0 : 4*uvStep), uvs, horizontal, 1, limit, blimit, threshold, mb);
}
#define GEA_LOOP_WRAPPER(name, horizontal, mb) \
  void vp8_loop_filter_##name##_s3(unsigned char* y, unsigned char* u, unsigned char* v, \
                                   int ys, int uvs, struct loop_filter_info* info) { \
    filter(y, u, v, ys, uvs, info, horizontal, mb); \
  }
GEA_LOOP_WRAPPER(bh, 1, 0)
GEA_LOOP_WRAPPER(bv, 0, 0)
GEA_LOOP_WRAPPER(mbh, 1, 1)
GEA_LOOP_WRAPPER(mbv, 0, 1)
#undef GEA_LOOP_WRAPPER

static void simple(uint8_t* y, int stride, const uint8_t* blimit, int horizontal, int mb) {
  const int step = horizontal ? stride : 1;
  if (mb) edge(y, stride, horizontal, 2, 0, blimit[0], 0, 2);
  else for (int i = 4; i <= 12; i += 4)
    edge(y + i*step, stride, horizontal, 2, 0, blimit[0], 0, 2);
}
#define GEA_SIMPLE_WRAPPER(name, horizontal, mb) \
  void vp8_loop_filter_simple_##name##_s3(unsigned char* y, int stride, const unsigned char* blimit) { \
    simple(y, stride, blimit, horizontal, mb); \
  }
GEA_SIMPLE_WRAPPER(bh, 1, 0)
GEA_SIMPLE_WRAPPER(bv, 0, 0)
GEA_SIMPLE_WRAPPER(mbh, 1, 1)
GEA_SIMPLE_WRAPPER(mbv, 0, 1)
#undef GEA_SIMPLE_WRAPPER
