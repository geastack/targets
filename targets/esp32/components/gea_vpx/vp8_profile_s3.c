// SPDX-License-Identifier: Apache-2.0
#include "vp8_profile_s3.h"
#include "esp_timer.h"
#include <stdatomic.h>

// Explicit diagnostic only: disabled on boot, with no timers or extra tasks.
// Each decoder binds its own counters; stages are never shared across tasks.
static atomic_int requested;
static _Thread_local struct gea_vp8_profile_s3* current;
void gea_vp8_profile_enable_s3(int enabled) { atomic_store_explicit(&requested, !!enabled, memory_order_relaxed); }
int gea_vp8_profile_requested_s3(void) { return atomic_load_explicit(&requested, memory_order_relaxed); }
struct gea_vp8_profile_s3* gea_vp8_profile_bind_s3(struct gea_vp8_profile_s3* next) {
  struct gea_vp8_profile_s3* previous = current;
  current = next;
  return previous;
}
uint64_t gea_vp8_profile_clock_s3(void) { return current ? esp_timer_get_time() : 0; }
void gea_vp8_profile_add_s3(unsigned stage, uint64_t started) {
  if (!started || !current || stage >= GEA_VP8_PROFILE_STAGES) return;
  current->us[stage] += esp_timer_get_time() - started;
  ++current->calls[stage];
}
