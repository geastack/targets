import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../display.cpp', import.meta.url), 'utf8')
function method(name, type) {
  const match = source.match(new RegExp(`    ${type} ${name}\\([^]*?\\n    \\{[^]*?\\n    \\}`))
  assert.ok(match, `missing ${name}`)
  return match[0]
}
const paceStart = source.indexOf('        if (streamFlush)\n        {\n          streamChunksSincePace++;')
assert.ok(paceStart >= 0, 'missing framebuffer stream pacing')
const pacing = source.slice(paceStart, source.indexOf('\n#endif', paceStart))
const spin = source.match(/    bool waitForFlushCompleteSpin\([^]*?\n    \{[^]*?\n    \}/)[0]
const program = `
#include "displays/co5300/co5300.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <vector>

constexpr int kFlushChunkMin = 2;
constexpr int kFlushChunkDefault = 16;
constexpr int kFlushChunkLimit = 128;
struct Pipeline {
  int flushQueueDepth_ = 2;
  int flushBufferCapacity_ = 466 * 16;
  int requestedFlushChunkRows_ = 16;
  int logicalDisplayWidth() const { return 466; }
  int logicalDisplayHeight() const { return 466; }
${method('normalizeFlushChunkRows', 'int')}
${method('canUseFramebufferColorStream', 'bool')}
};

struct Panel {
  std::vector<std::uint16_t> pixels = std::vector<std::uint16_t>(480 * 480, 0xf81f);
  unsigned windows = 0, writes = 0;
  int left = 0, top = 0, width = 0, height = 0, cursor = 0;
  void window(int x, int y, int w, int h) {
    const auto address = gea::chips::co5300::CommandSet::addressWindow(x, y, x + w - 1, y + h - 1);
    const auto unpack = [](auto bytes, int offset) { return int(bytes[offset]) * 256 + bytes[offset + 1]; };
    left = unpack(address.columns, 0); top = unpack(address.rows, 0);
    width = unpack(address.columns, 2) - left + 1;
    height = unpack(address.rows, 2) - top + 1;
    assert(left == x + 6 && top == y && width == w && height == h);
    assert(left >= 0 && left + width <= 480 && top >= 0 && top + height <= 480);
    cursor = 0; ++windows;
  }
  void color(int command, const std::vector<std::uint16_t>& data) {
    if (command == 0x2c) cursor = 0;
    else assert(command == 0x3c);
    ++writes;
    for (const auto pixel : data) {
      assert(cursor < width * height);
      pixels[(top + cursor / width) * 480 + left + cursor % width] = pixel;
      ++cursor;
    }
  }
};

constexpr int GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS = 4;
using int64_t = std::int64_t;
int64_t simulatedClock = 0;
int64_t esp_timer_get_time() { simulatedClock += 100; return simulatedClock; }
using UBaseType_t = unsigned;
unsigned semaphorePolls = 0, readyAfter = 0;
UBaseType_t uxSemaphoreGetCount(void*) {
  ++semaphorePolls;
  return readyAfter && semaphorePolls >= readyAfter ? 2 : 0;
}
struct SpinDrain {
  void* flushSlots_ = this;
  int flushQueueDepth_ = 2;
  unsigned blockingCalls = 0;
  bool waitForFlushComplete(int reserved) { assert(reserved == 0); ++blockingCalls; return true; }
${spin}
};
enum class FlushStage { WaitComplete, Idle };
struct Pacing {
  unsigned drains = 0;
  struct { int64_t completeWaitUs = 0; } flushStats_;
  void setFlushStage(FlushStage, int) {}
  bool waitForFlushCompleteSpin(int budget) { assert(budget == 4000); ++drains; return true; }
  bool run(bool streamFlush) {
    int streamChunksSincePace = 0;
    const int y1 = 333;
    for (int row = 0; row <= y1; row += 16) {
      const int chunkRows = std::min(16, y1 - row + 1);
${pacing}
    }
    return true;
  }
};

int main() {
  SpinDrain slow;
  const auto before = simulatedClock;
  assert(slow.waitForFlushCompleteSpin(4000));
  assert(slow.blockingCalls == 1 && simulatedClock - before <= 4200);
  SpinDrain ready;
  readyAfter = 3; semaphorePolls = 0;
  assert(ready.waitForFlushCompleteSpin(4000) && ready.blockingCalls == 0);
  readyAfter = 0;
  Pacing paced;
  assert(paced.run(true) && paced.drains == 5);
  assert(paced.run(false) && paced.drains == 5);
  Pipeline pipeline;
  assert(pipeline.canUseFramebufferColorStream(66, 334, 334, false));
  assert(pipeline.canUseFramebufferColorStream(66, 334, 334, true));
  assert(pipeline.canUseFramebufferColorStream(462, 4, 100, true));
  assert(!pipeline.canUseFramebufferColorStream(66, 334, 16, false));
  assert(!pipeline.canUseFramebufferColorStream(-1, 334, 334, false));
  assert(!pipeline.canUseFramebufferColorStream(133, 334, 334, false));
  assert(!pipeline.canUseFramebufferColorStream(466, 1, 334, false));
  assert(!pipeline.canUseFramebufferColorStream(0, 0, 334, false));
  assert(!pipeline.canUseFramebufferColorStream(0, 334, 467, false));
  pipeline.flushQueueDepth_ = 1;
  assert(!pipeline.canUseFramebufferColorStream(66, 334, 334, false));
  pipeline.flushQueueDepth_ = 2;
  pipeline.flushBufferCapacity_ = 466 * 2;
  assert(!pipeline.canUseFramebufferColorStream(0, 4, 16, false)); // Packed single chunk.
  assert(pipeline.canUseFramebufferColorStream(0, 4, 18, false));

  Panel serial, stream;
  stream.window(66, 66, 334, 334);
  for (int row = 0; row < 334; row += 16) {
    const int rows = std::min(16, 334 - row);
    std::vector<std::uint16_t> payload;
    for (int y = 0; y < rows; ++y)
      for (int x = 0; x < 334; ++x)
        payload.push_back(std::uint16_t((x * 31 + (row + y) * 71) & 65535));
    serial.window(66, 66 + row, 334, rows);
    serial.color(0x2c, payload);
    stream.color(row == 0 ? 0x2c : 0x3c, payload);
  }
  assert(stream.cursor == 334 * 334 && stream.windows == 1 && stream.writes == 21);
  assert(serial.windows == 21 && serial.pixels == stream.pixels);
  assert(stream.pixels[66 * 480 + 71] == 0xf81f); // Left neighbor remains intact.
  assert(stream.pixels[400 * 480 + 72] == 0xf81f); // Bottom neighbor remains intact.
}
`
const output = fileURLToPath(new URL('../../../../core/packages/geatsc-plugin-gea/dist/test_framebuffer_partial_stream', import.meta.url))
const chips = fileURLToPath(new URL('../../../../core/packages/chips/', import.meta.url))
const result = spawnSync(process.env.CXX || 'clang++', [
  '-std=c++20', '-fsanitize=address,undefined', '-DGEA_EMBEDDED_CO5300_PANEL_X_GAP=6',
  '-DGEA_EMBEDDED_CO5300_PANEL_Y_GAP=0', '-I' + chips,
  '-x', 'c++', '-', chips + 'displays/co5300/co5300.cpp', '-o', output,
], { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: fileURLToPath(new URL('../../../../core/packages/geatsc-plugin-gea/dist/', import.meta.url)) } })
assert.equal(result.status, 0, result.stderr)
const run = spawnSync(output, [], { encoding: 'utf8' })
assert.equal(run.status, 0, run.stderr)
console.log('partial-window stream eligibility, packed chunk limits, panel gap and pixel equivalence: PASS')
