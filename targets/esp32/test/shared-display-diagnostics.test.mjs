#!/usr/bin/env node
// Contracts that belong to the SHARED esp32 sources in this package, not to any
// one board. Three boards each carried their own copy of these assertions; the
// 2.41 copy asserted 2.06's files, which is how the copies drifted unnoticed.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const esp32Dir = resolve(new URL('..', import.meta.url).pathname)
const source = (p) => readFileSync(resolve(esp32Dir, p), 'utf8')

const framePerfMacro = 'GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG'
const heapDiagnosticsMacro = 'GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG'

// Both macros default in @geastack/core's gea_perf_config.h, under the
// GEA_EMBEDDED_PERF master switch; that package asserts the defaults. What is
// ours is that every diagnostic line here stays behind them.
function assertPatternBehindMacro(path, macro, patterns) {
  const lines = source(path).split(/\r?\n/)
  const guardedStack = []
  for (const [index, line] of lines.entries()) {
    const directive = line.match(/^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$/)
    if (directive) {
      const [, kind, rest] = directive
      if (kind === 'if' || kind === 'ifdef' || kind === 'ifndef') {
        guardedStack.push(guardedStack.includes(true) || rest.includes(macro))
      } else if (kind === 'elif') {
        const parentGuarded = guardedStack.slice(0, -1).includes(true)
        guardedStack[guardedStack.length - 1] = parentGuarded || rest.includes(macro)
      } else if (kind === 'endif') {
        guardedStack.pop()
      }
    }
    for (const pattern of patterns) {
      if (!pattern.test(line)) continue
      assert.ok(guardedStack.includes(true), `${path}:${index + 1} should keep ${pattern} behind ${macro}`)
    }
  }
}

assertPatternBehindMacro('services/frame_scheduler.cpp', framePerfMacro, [
  /stack probe \[frame_scheduler:after_app_frame_task\]/,
  /perf: frame=/
])
assertPatternBehindMacro('services/app_runner.cpp', framePerfMacro, [/stack probe \[/])
assertPatternBehindMacro('services/app_runner.cpp', heapDiagnosticsMacro, [
  /#include "esp_heap_trace\.h"/,
  /heap_trace_/,
  /\[heap-trace\]/,
  /\[heap-live\]/,
  /HeapProbe::log\(/
])
assertPatternBehindMacro('connectivity/diagnostics_transport.cpp', framePerfMacro, [/stack probe \[/])
assertPatternBehindMacro('connectivity/diagnostics_transport.cpp', heapDiagnosticsMacro, [
  /\[heap-map\]/,
  /\[heap-free\]/,
  /heap probe \[/,
  /heap task summary \[/,
  /heap task \[/
])

// The contiguous flush pool is OPT-IN and off by default: staging comes from
// the current app budget unless a board asks for a reserved block.
const memoryConfig = source('memory_config.h')
assert.match(
  memoryConfig,
  /#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES\s*\n#define GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES 0\b/,
  'the reserved flush pool must default to disabled'
)
const displaySource = source('display.cpp')
// The reservation itself is what must be opt-in. `flushBuffersFromPool_` is an
// always-compiled flag that stays false when no pool exists.
assertPatternBehindMacro('display.cpp', 'GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES', [
  /g_flushPool/,
  /LCD flush pool reserved/
])

// CO5300 partial-flush padding: one helper, every edge, used by every path.
// translateY animations leave stale rows on the panel without it.
assert.match(displaySource, /present::Rect padPartialFlushWindowForCO5300\(present::Rect rect\)/,
  'padding should be centralized so tree and present paths stay consistent')
assert.match(displaySource, /rect\.x0 = rect\.x0 >= kEdgePadXPx \? rect\.x0 - kEdgePadXPx : 0;/, 'left edge')
assert.match(displaySource, /rect\.y0 = rect\.y0 >= kEdgePadYPx \? rect\.y0 - kEdgePadYPx : 0;/, 'top edge')
assert.match(displaySource, /rect\.x1 = rect\.x1 \+ kEdgePadXPx < width \? rect\.x1 \+ kEdgePadXPx : width - 1;/, 'right edge')
assert.match(displaySource, /rect\.y1 = rect\.y1 \+ kEdgePadYPx < height \? rect\.y1 \+ kEdgePadYPx : height - 1;/, 'bottom edge')
assert.ok(
  [...displaySource.matchAll(/padPartialFlushWindowForCO5300\(/g)].length >= 4,
  'padding should be used by the helper declaration plus flush, flushRects and presentRegion'
)
assert.match(
  displaySource,
  /present::Rect padded = padPartialFlushWindowForCO5300\(\{x0, y0, x1, y1\}\);/,
  'Display::flush() must pad its single dirty window; translateY uses this path, not flushRects'
)

console.log('shared esp32 display/diagnostics contracts passed')
