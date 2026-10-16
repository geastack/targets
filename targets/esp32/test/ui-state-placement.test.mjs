import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { mkdirSync, readFileSync } from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../services/frame_scheduler.cpp', import.meta.url), 'utf8')
const fragment = readFileSync(new URL('../ldfragments/gea_ui_state_external.lf', import.meta.url), 'utf8')
const targets = process.env.GEA_TEST_TARGETS_ROOT || fileURLToPath(new URL('../../..', import.meta.url))
const build = process.env.GEA_TEST_BUILD_DIR || path.join(targets, 'build')
mkdirSync(build, { recursive: true })
function block(needle) {
  const start = source.indexOf(needle)
  assert.notEqual(start, -1, needle)
  const brace = source.indexOf('{', start)
  let depth = 1, end = brace + 1
  while (depth && end < source.length) {
    if (source[end] === '{') ++depth
    else if (source[end] === '}') --depth
    ++end
  }
  assert.equal(depth, 0, needle)
  return source.slice(start, end)
}
const constructor = source.match(/#if GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT\n(?:[^\n]*\n)*?\s*__attribute__\(\(noinline\)\) FrameSchedulerEngine\(\) \{\}\n#else\n\s*FrameSchedulerEngine\(\) = default;\n#endif/)
assert.ok(constructor, 'opt-in must preserve the defaulted constructor when disabled')
const storageStart = source.indexOf('\n\tTaskHandle_t task_ = nullptr;')
const storageEnd = source.indexOf('\n};', storageStart)
assert.ok(storageStart > 0 && storageEnd > storageStart)
const program = `
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#ifndef GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT
#define GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT 0
#endif
using TaskHandle_t = void *;
using QueueHandle_t = void *;
using StackType_t = std::uint8_t;
using esp_timer_handle_t = void *;
struct StaticTask_t { unsigned char bytes[112]; };
struct StaticQueue_t { unsigned char bytes[92]; };
namespace gea::framework::events { struct Event { std::uint32_t words[16]; }; }
struct FrameScheduler {
  static constexpr unsigned kEventQueueDepth = 32;
  static constexpr std::int64_t kDefaultFrameIntervalUs = 66667;
};
constexpr unsigned kTaskStackWords = 97;
${block('enum class FrameStage')} ;
class FrameSchedulerEngine {
public:
${block('static FrameSchedulerEngine &instance()')}
${block('struct PerfWindow')} ;
${constructor[0]}
${source.slice(storageStart, storageEnd)}
};
int main() {
  auto &s = FrameSchedulerEngine::instance();
  assert(!s.task_ && !s.eventQueue_);
  assert(!s.eventPending_ && !s.frameInProgress_ && !s.catchUpFrameDue_);
  assert(!s.catchUpRequest_ && !s.vsyncDriven_ && s.lastVsyncPostUs_ == 0);
  assert(s.frameIntervalUs_ == FrameScheduler::kDefaultFrameIntervalUs);
  assert(s.frameStage_ == static_cast<int>(FrameStage::Idle));
  assert(s.frameSequence_ == 0 && s.timerTickCount_ == 0 && s.timerSendFailCount_ == 0);
  for (auto b : s.eventQueueBuffer_) assert(b == 0);
  for (auto b : s.eventQueueStorage_.bytes) assert(b == 0);
#if GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
  assert(!s.timer_ && !s.timerRunning_ && s.timerIntervalUs_ == 0);
#else
  for (auto b : s.taskStorage_.bytes) assert(b == 0);
#if GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL
  assert(!s.taskStack_);
#else
  for (auto b : s.taskStack_) assert(b == 0);
#endif
#endif
#if GEA_NATIVE_DEBUGGER
  assert(s.debuggerFps_ == GEA_NATIVE_DEBUGGER_FPS && s.nextDebuggerFrameUs_ == 0);
#endif
  auto &p = s.perf_;
  assert(p.frameCount == 0 && p.treeAbsModeRejectNode == -1);
  assert(p.cadenceStartGapMinUs == std::numeric_limits<std::int64_t>::max());
  assert(p.cadenceDoneGapMinUs == std::numeric_limits<std::int64_t>::max());
  p.frameCount = 12; p.treeAbsModeRejectNode = 9; p.cadenceStartGapMinUs = 1;
  p.reset();
  assert(p.frameCount == 0 && p.treeAbsModeRejectNode == -1);
  assert(p.cadenceStartGapMinUs == std::numeric_limits<std::int64_t>::max());
  s.frameSequence_ = 17;
  // The late singleton constructor must run once, without resetting live state.
  std::thread threads[4];
  for (auto &t : threads) t = std::thread([&] {
    assert(&FrameSchedulerEngine::instance() == &s);
    assert(FrameSchedulerEngine::instance().frameSequence_ == 17);
  });
  for (auto &t : threads) t.join();
}
`
for (const enabled of [undefined, 0, 1]) {
  for (const [timer, external, debugMode] of [[0, 0, 0], [0, 1, 1], [1, 1, 0]]) {
    const binary = path.join(build, `frame-state-init-${enabled ?? 'default'}-${timer}${external}${debugMode}`)
    const compiled = spawnSync(process.env.CXX || 'c++', [
      '-std=c++20', '-O2', '-g', '-pthread', '-fsanitize=address,undefined',
      ...(enabled === undefined ? [] : [`-DGEA_EMBEDDED_UI_STATE_DYNAMIC_INIT=${enabled}`]),
      `-DGEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER=${timer}`,
      `-DGEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL=${external}`,
      `-DGEA_NATIVE_DEBUGGER=${debugMode}`, '-DGEA_NATIVE_DEBUGGER_FPS=37',
      '-x', 'c++', '-', '-o', binary,
    ], { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: build } })
    assert.ifError(compiled.error)
    assert.equal(compiled.status, 0, compiled.stderr)
    const ran = spawnSync(binary, [], { encoding: 'utf8', timeout: 15000 })
    assert.ifError(ran.error)
    assert.equal(ran.status, 0, ran.stderr)
  }
}
// Protect the scope of the opt-in. The ISR-owned scheduler remains available to
// an app's internal/RTC mapping, and initialized data never enters this sweep.
assert.match(fragment, /archive: libgea_active_geatsc\.a/)
assert.match(fragment, /legacy_bss -> extern_ram/)
assert.doesNotMatch(fragment, /^\s*(?:data|rodata) -> extern_ram/m)
assert.doesNotMatch(fragment, /^\s*frame_scheduler \(/m)
const cmake = readFileSync(path.join(targets, 'targets/esp32/gea_framework.cmake'), 'utf8')
assert.match(cmake, /if\(_GEA_UI_STATE_EXTERNAL\)[\s\S]*GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT=1[\s\S]*gea_ui_state_external\.lf/)
console.log('UI placement: default/off/on scheduler initialization, timer/stack variants and singleton lifetime passed')
