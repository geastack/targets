#pragma once

#if __has_include("gea_embedded_app_config.h")
#include "gea_embedded_app_config.h"
#endif

// Per-frame perf instrumentation flags + the GEA_EMBEDDED_PERF master switch.
// Included after the app config above so a per-app override of GEA_EMBEDDED_PERF
// (or any individual sub-flag) is respected; this header owns the defaults and
// the master override that forces every sub-flag off when the master is off.
#include "gea_perf_config.h"

#ifndef GEA_EMBEDDED_GEA_MAIN_TASK_STACK_BYTES
#define GEA_EMBEDDED_GEA_MAIN_TASK_STACK_BYTES 24576
#endif

#ifndef GEA_EMBEDDED_GEA_INIT_TASK_STACK_BYTES
// 16 KiB was tight for any geatsc-emitted app init that builds large
// `std::vector<gea_cpp_value>` initializer-list literals on the stack
// (e.g. button-tetris's 28×8 shape table at program.cpp:4102 materialises
// ~30 KiB of stack temps). Until the emitter rewrites those literals as
// static-const data, give the init task more headroom — this stack only
// lives for the duration of `__gea_top_level()` and is freed afterwards.
#define GEA_EMBEDDED_GEA_INIT_TASK_STACK_BYTES 65536
#endif

#ifndef GEA_EMBEDDED_APP_FRAME_TASK_STACK_WORDS
#define GEA_EMBEDDED_APP_FRAME_TASK_STACK_WORDS 3072
#endif

#ifndef GEA_EMBEDDED_APP_FRAME_TASK_PRIORITY
#define GEA_EMBEDDED_APP_FRAME_TASK_PRIORITY 5
#endif

// GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG / _PERF_LITE and
// GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG (and the GEA_EMBEDDED_PERF master that
// governs them) are defined in gea_perf_config.h, included above.

#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX
#define GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX 80
#endif

#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH
#define GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH 2
#endif

#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_BUFFER_MAX_BYTES
#define GEA_EMBEDDED_DISPLAY_FLUSH_BUFFER_MAX_BYTES (136 * 1024)
#endif

// Optional pre-reserved contiguous flush pool. 0 = disabled (legacy behavior:
// per-slot buffers malloc'd lazily on setFlushConfig, which can fall back to a
// slow 1-deep pipeline once the heap is fragmented). When > 0, a single
// contiguous DMA-internal block of this many bytes is reserved early at display
// init (heap still unfragmented) and the flush slots are carved from it — so a
// deep pipeline (e.g. 48 rows x 2) is guaranteed regardless of later
// fragmentation. Must be >= chunkRows*kWidth*2 rounded up to 64, times depth.
#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES
#define GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES 0
#endif

#ifndef GEA_EMBEDDED_DISPLAY_PRESENT_RECT_LIMIT
#define GEA_EMBEDDED_DISPLAY_PRESENT_RECT_LIMIT 24
#endif
