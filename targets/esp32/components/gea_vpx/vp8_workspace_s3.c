// SPDX-License-Identifier: Apache-2.0
#include "vp8_s3.h"

// Bound by the decoder only during vpx_codec_decode. Each task has its own
// binding; nested decoders restore the previous binding rather than sharing
// mutable SIMD intermediates across tasks.
static _Thread_local void* workspace;

void* gea_vp8_workspace_s3(void) { return workspace; }
void* gea_vp8_bind_workspace_s3(void* next) {
  void* previous = workspace;
  workspace = next;
  return previous;
}
