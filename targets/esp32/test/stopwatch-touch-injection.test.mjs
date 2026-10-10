import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { readFileSync } from 'node:fs'
import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../../esp32-s3-m5stack-stopwatch/main/touch.cpp', import.meta.url), 'utf8')
const output = fileURLToPath(new URL('../../../build/stopwatch-touch-injection-test', import.meta.url))

function method(signature) {
  const start = source.indexOf(signature)
  assert.notEqual(start, -1, `missing production method ${signature}`)
  const brace = source.indexOf('{', start)
  let depth = 1
  let end = brace + 1
  while (depth > 0 && end < source.length) {
    if (source[end] === '{') depth++
    else if (source[end] === '}') depth--
    end++
  }
  assert.equal(depth, 0, `unclosed production method ${signature}`)
  return source.slice(start, end)
}

const stateBegin = source.indexOf('namespace {')
const stateEnd = source.indexOf('} // namespace', stateBegin) + '} // namespace'.length
assert.ok(stateBegin >= 0 && stateEnd > stateBegin)

// Execute the production cache, poll and injection methods with a fake CST820.
// The hardware callback can pause a read to reproduce two-task interleavings.
const program = `
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>
#define IRAM_ATTR
using i2c_master_dev_handle_t = void *;
constexpr int ESP_OK = 0;
namespace board { struct { int interrupt = 7; } touch; }
namespace display { constexpr int kWidth = 466, kHeight = 466; }
namespace gea::platform::touch {
enum class Phase { Down = 1, Move = 2, Up = 3 };
class Touchscreen {
public:
  using Observer = void (*)(Phase, bool, int, int);
  using PointerObserver = void (*)(Phase, bool, int, int, int);
  static void setObserver(Observer);
  static void setPointerObserver(PointerObserver);
  static void poll(int);
  static int read(int *, int *);
  static int readCached(int *, int *);
  static void consumeLatestMove(int *, int *);
  static void injectEvent(Phase, bool, int, int);
};
}
std::array<std::uint8_t, 7> hardwareReport{};
std::atomic<int> hardwareReads{0};
int hardwareLevel = 1;
int hardwareResult = ESP_OK;
std::mutex readMutex;
std::condition_variable readCondition;
bool blockRead = false, readEntered = false, releaseRead = false;
int gpio_get_level(int) { return hardwareLevel; }
int i2c_master_transmit_receive(void *, const std::uint8_t *, int,
                                std::uint8_t *report, std::size_t size, int) {
  hardwareReads.fetch_add(1);
  {
    std::unique_lock<std::mutex> guard(readMutex);
    readEntered = true;
    readCondition.notify_all();
    if (blockRead) readCondition.wait(guard, [] { return releaseRead; });
  }
  std::copy_n(hardwareReport.begin(), size, report);
  return hardwareResult;
}
${source.slice(stateBegin, stateEnd)}
namespace gea::platform::touch {
${method('void Touchscreen::setObserver(Observer next)')}
${method('void Touchscreen::setPointerObserver(PointerObserver next)')}
${method('void Touchscreen::poll(int nowMs)')}
${method('int Touchscreen::read(int* x, int* y)')}
${method('int Touchscreen::readCached(int* x, int* y)')}
${method('void Touchscreen::consumeLatestMove(int* x, int* y)')}
${method('void Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)')}
}
using gea::platform::touch::Phase;
using gea::platform::touch::Touchscreen;
struct Event { Phase phase; bool touching; int x, y; };
std::vector<Event> events;
std::atomic<bool> consumeInObserver{false}, blockUp{false}, upEntered{false};
std::mutex upMutex;
std::condition_variable upCondition;
bool releaseUp = false;
void record(Phase phase, bool touching, int x, int y, int pointer) {
  assert(pointer == 0);
  events.push_back({phase, touching, x, y});
  if (phase == Phase::Move && consumeInObserver.load()) {
    int latestX = -1, latestY = -1;
    Touchscreen::consumeLatestMove(&latestX, &latestY);
    assert(latestX == x && latestY == y);
  }
  if (phase == Phase::Up && blockUp.load()) {
    std::unique_lock<std::mutex> guard(upMutex);
    upEntered.store(true);
    upCondition.notify_all();
    upCondition.wait(guard, [] { return releaseUp; });
  }
}
void sample(bool touching, int x, int y) {
  hardwareReport = {0, 0, static_cast<std::uint8_t>(touching ? 1 : 0),
    static_cast<std::uint8_t>((x >> 8) & 15), static_cast<std::uint8_t>(x),
    static_cast<std::uint8_t>((y >> 8) & 15), static_cast<std::uint8_t>(y)};
}
int main() {
  device = reinterpret_cast<void *>(1);
  Touchscreen::setPointerObserver(record);
  sample(false, 0, 0);
  Touchscreen::injectEvent(Phase::Down, true, 40, 230);
  touchInterrupt(nullptr);
  for (int now : {10, 20, 100}) Touchscreen::poll(now);
  assert(hardwareReads.load() == 0);
  assert(reportPending.load());
  assert(events.size() == 1 && events[0].phase == Phase::Down);
  Touchscreen::injectEvent(Phase::Move, true, 40, 210);
  Touchscreen::injectEvent(Phase::Move, true, 40, 170);
  assert(events.size() == 2 && events[1].phase == Phase::Move);
  int x = -1, y = -1;
  assert(Touchscreen::readCached(&x, &y) == 1 && x == 40 && y == 170);
  Touchscreen::consumeLatestMove(&x, &y);
  assert(x == 40 && y == 170);
  consumeInObserver.store(true);
  Touchscreen::injectEvent(Phase::Move, true, 40, 150);
  consumeInObserver.store(false);
  assert(events.size() == 3);

  // Hardware remains suppressed until the injected Up has actually been queued.
  blockUp.store(true);
  std::thread up([] { Touchscreen::injectEvent(Phase::Up, false, 40, 150); });
  {
    std::unique_lock<std::mutex> guard(upMutex);
    upCondition.wait(guard, [] { return upEntered.load(); });
  }
  Touchscreen::poll(110);
  assert(hardwareReads.load() == 0 && reportPending.load());
  {
    std::lock_guard<std::mutex> guard(upMutex);
    releaseUp = true;
  }
  upCondition.notify_all();
  up.join();
  blockUp.store(false);
  assert(!injectedGestureActive.load());
  assert(Touchscreen::readCached(&x, &y) == 0);

  // An interrupt latched during injection is consumed by the resumed poll.
  sample(true, 100, 200);
  Touchscreen::poll(120);
  assert(hardwareReads.load() == 1 && !reportPending.load());
  assert(events.back().phase == Phase::Down && events.back().x == 100);
  sample(true, 120, 190);
  Touchscreen::poll(130);
  assert(events.back().phase == Phase::Move && events.back().x == 120);
  Touchscreen::consumeLatestMove(&x, &y);
  assert(x == 120 && y == 190);
  sample(false, 0, 0);
  Touchscreen::poll(140);
  assert(events.back().phase == Phase::Up);

  // An already-running read finishes before the injected Down takes ownership.
  {
    std::lock_guard<std::mutex> guard(readMutex);
    blockRead = true;
    readEntered = false;
    releaseRead = false;
  }
  hardwareLevel = 0;
  std::thread poll([] { Touchscreen::poll(150); });
  {
    std::unique_lock<std::mutex> guard(readMutex);
    readCondition.wait(guard, [] { return readEntered; });
  }
  std::atomic<bool> downStarted{false}, downFinished{false};
  std::thread down([&] {
    downStarted.store(true);
    Touchscreen::injectEvent(Phase::Down, true, 200, 300);
    downFinished.store(true);
  });
  while (!downStarted.load()) std::this_thread::yield();
  assert(!downFinished.load());
  {
    std::lock_guard<std::mutex> guard(readMutex);
    releaseRead = true;
  }
  readCondition.notify_all();
  poll.join();
  down.join();
  assert(injectedGestureActive.load());
  assert(Touchscreen::readCached(&x, &y) == 1 && x == 200 && y == 300);
  const int readsBefore = hardwareReads.load();
  touchInterrupt(nullptr);
  Touchscreen::poll(160);
  assert(hardwareReads.load() == readsBefore && reportPending.load());
  Touchscreen::injectEvent(Phase::Up, false, 200, 300);
}
`

const compile = spawnSync(process.env.CXX || 'clang++', [
  '-std=c++20', '-pthread', '-fsanitize=address,undefined',
  '-x', 'c++', '-', '-o', output
], { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: dirname(output) } })
assert.equal(compile.status, 0, compile.stderr)
const run = spawnSync(output, [], { encoding: 'utf8', timeout: 30000 })
assert.equal(run.status, 0, run.stderr || String(run.error || 'test timed out'))
console.log('StopWatch injected gestures own the touch stream, preserve coalescing and defer hardware IRQ reports until Up')
