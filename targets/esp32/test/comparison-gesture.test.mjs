import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { readFileSync } from 'node:fs'
import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const header = fileURLToPath(new URL('../services/comparison_gesture.h', import.meta.url))
const source = readFileSync(
  new URL('../../../../core/packages/engine/touch_runtime.cpp', import.meta.url),
  'utf8',
)
const output = fileURLToPath(new URL('../../../build/comparison-gesture-test', import.meta.url))

function method(signature) {
  const start = source.indexOf(signature)

  assert.notEqual(start, -1)
  let end = source.indexOf('{', start) + 1
  let depth = 1

  while (depth && end < source.length) {
    if (source[end] === '{') {
      depth++
    } else if (source[end] === '}') {
      depth--
    }

    end++
  }

  assert.equal(depth, 0)

  return source.slice(start, end)
}

const consumeStart = source.indexOf('int tx = event.x;')
const consumeEnd = source.indexOf('if (touching && !wasActive_)', consumeStart)

assert.ok(consumeStart >= 0 && consumeEnd > consumeStart)
const consume = source.slice(consumeStart, consumeEnd)

const program = `
#include "${header}"
#include <cassert>
#include <memory>
#include <thread>
#include <vector>
using namespace gea::platform::comparison;
int64_t deviceUs = 1000;
extern "C" void gea_touch_trace_consumed(int phase, bool touching, int x, int y,
                                       int pointerId, int handlerX, int handlerY) {
  input::consumed(deviceUs, phase, touching, x, y, pointerId, handlerX, handlerY);
}
enum class TouchPhase { Down = 1, Move = 2, Up = 3 };
struct Event { TouchPhase touchPhase; bool touching; int x, y, pointerId; };
namespace gea::platform::touch {
struct Touchscreen {
  static void consumeLatestMove(int *x, int *y) { *x = 60; *y = 70; }
};
}
struct TouchRuntime {
  static void transformTouchToLogical(int *x, int *y) { *x += 10; *y += 20; }
};
class Dispatcher {
public:
  bool wasActive_ = true;
  int lastX_ = 41, lastY_ = 42;
  ${method('static void recordConsumed(const Event &event, bool touching, int x, int y,')}
  void consume(const Event &event) { ${consume} }
};
int main() {
  input::Entry storage[3]{};
  input::consumed(500, 1, true, 1, 2, 0, 1, 2);
  assert(input::begin(storage, 3));
  assert(!input::begin(storage, 3));
  Dispatcher dispatcher;
  // Queue marker coordinates are stale; production dispatch consumes and
  // transforms the latest cache before recording its actual input.
  dispatcher.consume({TouchPhase::Move, true, 1, 2, 0});
  dispatcher.consume({TouchPhase::Up, false, 90, 91, 0});
  input::consumed(1200, 1, true, 3, 4, 0, 3, 4);
  input::consumed(1300, 2, true, 5, 6, 0, 5, 6);
  auto trace = input::end();
  assert(trace.entries == storage && trace.count == 3 && trace.dropped == 1);
  assert(trace.entries[0].x == 70 && trace.entries[0].y == 90);
  assert(trace.entries[0].timestampUs == 1000 && trace.entries[0].phase == 2);
  assert(trace.entries[1].x == 90 && trace.entries[1].y == 91);
  assert(trace.entries[1].handlerX == 41 && trace.entries[1].handlerY == 42);
  assert(!input::recording.load() && !input::entries);
  begin("bench", "idle", 2000);
  assert(!input::begin(storage, 3));
  end(3000);
  assert(input::begin(storage, 3));
  begin("bench", "guard", 4000);
  input::consumed(4100, 1, true, 1, 2, 0, 1, 2);
  assert(input::end().count == 0);
  end(5000);
  auto plan = std::make_unique<gesture::Plan>();
  assert(!plan->valid());
  assert(plan->addTouch(0, 1, 233, 220));
  assert(plan->addTouch(33, 1, 280, 220));
  assert(!plan->addTouch(20, 1, 280, 220));
  assert(!plan->addTouch(40, 2, 280, 220));
  assert(plan->addTouch(66, 0, 280, 220));
  assert(plan->addObservation(0) && plan->addObservation(33) && plan->addObservation(166));
  assert(!plan->addObservation(10) && plan->valid());
  std::vector<int> phases;
  std::vector<int> order;
  deviceUs = 10000;
  gesture::play(*plan, [] { return deviceUs; }, [](int64_t remaining) { deviceUs += remaining; },
    [&](int phase, bool touching, int, int) {
      phases.push_back(phase); order.push_back(1);
      assert(touching == (phase != 3));
    }, [&](gesture::Observation &sample) {
      sample.actualUs = deviceUs; order.push_back(2);
    });
  assert(plan->complete && plan->originUs == 10000);
  assert((phases == std::vector<int>{1, 2, 3}));
  assert((order == std::vector<int>{1, 2, 1, 2, 1, 2}));
  assert(plan->events[1].actualUs - plan->originUs == 33000);
  assert(plan->observations[2].actualUs - plan->originUs == 166000);
  assert(!plan->addTouch(200, 0, 1, 2));
  assert(!plan->addObservation(200));
  // Stopping on the control task detaches storage before any concurrent
  // consumer can use it again. A later sample cannot write the old buffer.
  input::Entry race[1]{};
  assert(input::begin(race, 1));
  std::thread stop([] { input::end(); });
  stop.join();
  input::consumed(500000, 1, true, 99, 99, 0, 99, 99);
  assert(race[0].timestampUs == 0);
}
`
const compiled = spawnSync(
  process.env.CXX || 'clang++',
  [
    '-std=c++20',
    '-pthread',
    '-DGEA_EMBEDDED_COMPARISON_BENCHMARK=1',
    '-I',
    fileURLToPath(new URL('../', import.meta.url)),
    '-fsanitize=address,undefined',
    '-fno-omit-frame-pointer',
    '-x',
    'c++',
    '-',
    '-o',
    output,
  ],
  { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: dirname(output) } },
)

assert.equal(compiled.status, 0, compiled.stderr)
const result = spawnSync(output, [], { encoding: 'utf8' })

assert.equal(result.status, 0, result.stderr)
console.log(
  'Comparison gestures: consumed cache coordinates, Up routing, bounded traces, BENCH exclusion and device-clock playback verified',
)
