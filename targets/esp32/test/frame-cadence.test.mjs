// Execute the production scheduler/DMA bodies with deterministic clock/RTOS
// doubles. No wall-clock assertions, ESP SDK, or production test hooks required.
import assert from 'node:assert/strict'
import { readFileSync, mkdirSync, writeFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'
import test from 'node:test'

const root = fileURLToPath(new URL('..', import.meta.url))
const scheduler = readFileSync(`${root}/services/frame_scheduler.cpp`, 'utf8')
const display = readFileSync(`${root}/display.cpp`, 'utf8')
function between(source, begin, end) {
  const start = source.indexOf(begin)
  assert.notEqual(start, -1, `missing production section: ${begin}`)
  const stop = source.indexOf(end, start)
  assert.notEqual(stop, -1, `missing end of production section: ${end}`)
  return source.slice(start, stop)
}
const defaults = between(scheduler, '#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES', '#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER')
const catchup = between(scheduler, '\t\tconst bool timerCatchUpDue =', '\n\t\t}\n\n\tvoid setFrameIntervalMs')
const blockingWait = between(display, '    bool waitForFlushComplete(int reservedSlots', '\n    // Like waitForFlushComplete()')
const wait = between(display, '    bool waitForFlushCompleteSpin(', '\n    esp_err_t tryConfigureFlushPipelineCandidate')
const tailStart = display.indexOf('// Region tail: spin for the last queued chunks')
assert.ok(tailStart >= 0)
const tail = between(display.slice(tailStart), '        const bool ok =', '\n      }')
const fixture = `
#include <atomic>
#include <cassert>
#include <cstdint>
#include <vector>
${defaults}
std::vector<int> delays;
void vTaskDelay(int ticks) { delays.push_back(ticks); }
struct Scheduler {
  std::atomic<bool> catchUpFrameDue_{false}, catchUpRequest_{false}, vsyncDriven_{false};
  int catchUpBurstFrames_ = 0;
  int64_t frameIntervalUs() { return 8333; }
  void finish(int64_t frameStartUs, int64_t frameDoneUs) { ${catchup}
  }
};
using UBaseType_t = unsigned;
int64_t nowUs = 0, dmaDoneAt = 0;
unsigned reservedSlots = 0;
int64_t esp_timer_get_time() { nowUs += 25; return nowUs; }
unsigned uxSemaphoreGetCount(void *) { return nowUs >= dmaDoneAt ? 2 - reservedSlots : 0; }
struct Flush {
  void *flushSlots_ = this;
  int flushQueueDepth_ = 2, blockingCalls = 0, closes = 0, lastReserved = -1;
  bool fallbackResult = true;
  struct { int64_t completeWaitUs = 0; } flushStats_;
  enum class FlushStage { Idle };
  void setFlushStage(FlushStage, int) {}
  void closeCsHeldStream() { ++closes; }
  bool waitForFlushComplete(int reserved = 0) { lastReserved = reserved; ++blockingCalls; if (fallbackResult) nowUs = dmaDoneAt; return fallbackResult; }
  ${wait}
  bool finish() { const int64_t waitStartUs = esp_timer_get_time(); ${tail} }
};
struct Slots { int count; };
constexpr int pdTRUE = 1;
int pdMS_TO_TICKS(int value) { return value; }
int xSemaphoreTake(void *handle, int) {
  auto &slots = *static_cast<Slots*>(handle);
  if (!slots.count) return 0;
  --slots.count; return pdTRUE;
}
void xSemaphoreGive(void *handle) { ++static_cast<Slots*>(handle)->count; }
#define ESP_LOGE(...) ((void)0)
struct BlockingFlush {
  void *flushSlots_;
  int flushQueueDepth_ = 2;
  ${blockingWait}
};
int main() {
  Slots complete{2}; BlockingFlush all{&complete};
  assert(all.waitForFlushComplete() && complete.count == 2);
  Slots oneReserved{1}; BlockingFlush previous{&oneReserved};
  assert(previous.waitForFlushComplete(1) && oneReserved.count == 1);
  Slots partlyComplete{1}; BlockingFlush failed{&partlyComplete};
  assert(!failed.waitForFlushComplete() && partlyComplete.count == 1);
  Slots pending{0}; BlockingFlush failedReserved{&pending};
  assert(!failedReserved.waitForFlushComplete(1) && pending.count == 0);
  Scheduler s;
#if GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES
  s.finish(0, 17000);
  assert(!s.catchUpRequest_ && delays.size() == 1 && delays.back() >= 1);
#else
  constexpr int burst = GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD;
  static_assert(burst > 0, "default must bound catch-up to let the idle watchdog run");
  for (int i = 1; i <= burst * 3; ++i) {
    s.finish(0, 17000);
    assert(s.catchUpRequest_);
    assert(delays.size() == unsigned(i / burst));
    if (i % burst == 0) assert(delays.back() >= 2); // 1 can expire at a tick boundary.
  }
  // A frame that catches up resets the burst, not merely its next deadline.
  for (int i = 0; i < burst - 1; ++i) s.finish(0, 17000);
  s.finish(0, 1000);
  assert(!s.catchUpRequest_ && s.catchUpBurstFrames_ == 0);
  delays.clear();
  s.finish(0, 17000);
  assert(delays.empty());
  s.catchUpFrameDue_ = true;
  s.finish(0, 1000);
  assert(s.catchUpRequest_ && !s.catchUpFrameDue_);
  // TE must remain the sole frame producer: even overdue timers cannot catch up.
  s.vsyncDriven_ = true; s.catchUpFrameDue_ = true;
  s.finish(0, 17000);
  assert(!s.catchUpRequest_ && s.catchUpBurstFrames_ == 0);
#endif
  Flush normal;
  nowUs = 0; dmaDoneAt = 2700; // two normal 64-row QSPI chunks
  assert(normal.finish() && normal.blockingCalls == 0 && normal.closes == 1);
  assert(nowUs >= dmaDoneAt);
  Flush slow;
  nowUs = 0; dmaDoneAt = 20000;
  assert(slow.finish() && slow.blockingCalls == 1 && slow.closes == 1);
  Flush stuck; stuck.fallbackResult = false;
  nowUs = 0; dmaDoneAt = 20000;
  assert(!stuck.finish() && stuck.blockingCalls == 1 && stuck.closes == 1);
  assert(nowUs < 10000); // bounded spin, then propagate failure
  Flush inactive; inactive.flushSlots_ = nullptr;
  assert(inactive.waitForFlushCompleteSpin() && inactive.blockingCalls == 0);
  // A prepared but unsubmitted slot must not be mistaken for outstanding DMA.
  reservedSlots = 1;
  Flush nextBand;
  nowUs = 0; dmaDoneAt = 2700;
  assert(nextBand.waitForFlushCompleteSpin(4000, 1) && nextBand.blockingCalls == 0);
  Flush delayedBand;
  nowUs = 0; dmaDoneAt = 20000;
  assert(delayedBand.waitForFlushCompleteSpin(4000, 1));
  assert(delayedBand.blockingCalls == 1 && delayedBand.lastReserved == 1);
}
`
const build = `${root}/test/.build`
mkdirSync(build, { recursive: true })
const source = `${build}/frame-cadence.cpp`
writeFileSync(source, fixture)
for (const [name, flags] of [
  ['default catch-up and DMA tail', []],
  ['board catch-up override', ['-DGEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD=60']],
  ['drop catch-up override', ['-DGEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES=1']],
]) test(name, () => {
  const binary = `${build}/frame-cadence`
  const compile = spawnSync(process.env.CXX || 'clang++', ['-std=c++20', '-O2', ...flags, source, '-o', binary], { encoding: 'utf8' })
  assert.equal(compile.status, 0, compile.stderr || String(compile.error))
  const run = spawnSync(binary, [], { encoding: 'utf8' })
  assert.equal(run.status, 0, run.stderr || String(run.error))
})

// Prove the harness detects the actual prior regressions, not just valid output.
for (const [name, mutant] of [
  ['one-tick idle yield', fixture.replace('vTaskDelay(2);', 'vTaskDelay(1);')],
  ['short DMA tail spin', fixture.replace('waitForFlushCompleteSpin(4000);', 'waitForFlushCompleteSpin(2000);')],
]) test(`reject prior regression: ${name}`, () => {
  assert.notEqual(mutant, fixture)
  const source = `${build}/frame-cadence-mutant.cpp`, binary = `${build}/frame-cadence-mutant`
  writeFileSync(source, mutant)
  const compile = spawnSync(process.env.CXX || 'clang++', ['-std=c++20', '-O2', source, '-o', binary], { encoding: 'utf8' })
  assert.equal(compile.status, 0, compile.stderr)
  const run = spawnSync(binary, [], { encoding: 'utf8' })
  assert.notEqual(run.status, 0, `${name} should fail`)
  // libc spells the abort differently: macOS "Assertion failed: (expr)", glibc "Assertion `expr' failed."
  assert.match(run.stderr, /Assertion (?:failed|`.*' failed)/)
})
