import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { readFileSync } from 'node:fs'
import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../display.cpp', import.meta.url), 'utf8')
const output = fileURLToPath(new URL('../../../build/display-snapshot-routing-test', import.meta.url))

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

// Compile the actual routing and cleanup methods against two tiny canvases.
// No ESP-IDF task scheduler is needed: changing currentCore between calls
// deterministically exercises the unpinned control task's CPU migration.
const program = `
#include <cassert>
#include <cstdint>
namespace gea::embedded::ui { bool gSnapshotRasterActive = false; }
int currentCore = 0;
int xPortGetCoreID() { return currentCore; }
namespace platform_display { constexpr int kWidth = 4, kHeight = 2; }
constexpr int kFramebufferStridePx = 4;
namespace gea::framework::graphics {
struct Canvas {
  std::uint16_t *pixels = nullptr;
  std::uint8_t alpha = 255;
  void bindPixels(std::uint16_t *next, int, int, int) { pixels = next; }
  void setGlobalAlpha(std::uint8_t next) { alpha = next; }
  std::uint8_t globalAlpha() const { return alpha; }
};
}
struct DisplayBackend {
  gea::framework::graphics::Canvas canvas_, workerCanvas_;
  std::uint16_t *frameBuffer_ = nullptr;
  int renderCoreId_ = -1;
  ${method('bool onWorkerCore() const')}
  ${method('gea::framework::graphics::Canvas &replayCanvas()')}
  ${method('gea::framework::graphics::Canvas *canvas()')}
  ${method('void setAlpha(std::uint8_t alpha)')}
  ${method('std::uint8_t alpha() const')}
  ${method('void rebindCanvasToFramebuffer()')}
};
int main() {
  for (int renderCore : {0, 1}) {
    std::uint16_t framebuffer[8]{}, snapshot[8]{};
    DisplayBackend backend;
    backend.frameBuffer_ = framebuffer;
    backend.renderCoreId_ = renderCore;
    backend.canvas_.bindPixels(framebuffer, 4, 2, 4);
    backend.workerCanvas_.bindPixels(framebuffer, 4, 2, 4);
    backend.canvas_.setGlobalAlpha(255);
    backend.workerCanvas_.setGlobalAlpha(87);
    currentCore = 1 - renderCore;
    assert(backend.canvas() == &backend.workerCanvas_);
    assert(backend.alpha() == 87);

    gea::embedded::ui::gSnapshotRasterActive = true;
    auto *bound = backend.canvas();
    assert(bound == &backend.canvas_);
    bound->bindPixels(snapshot, 4, 2, 4);
    for (int step = 0; step < 8; ++step) {
      currentCore = step & 1;
      assert(backend.canvas() == bound);
      assert(backend.canvas()->pixels == snapshot);
      backend.setAlpha(static_cast<std::uint8_t>(100 + step));
      assert(backend.alpha() == 100 + step);
      backend.canvas()->pixels[step] = static_cast<std::uint16_t>(step + 1);
      assert(backend.workerCanvas_.pixels == framebuffer);
      assert(backend.workerCanvas_.globalAlpha() == 87);
    }
    for (int i = 0; i < 8; ++i) {
      assert(snapshot[i] == i + 1);
      assert(framebuffer[i] == 0);
    }
    currentCore = 1 - renderCore;
    backend.rebindCanvasToFramebuffer();
    gea::embedded::ui::gSnapshotRasterActive = false;
    assert(backend.canvas() == &backend.workerCanvas_);
    assert(backend.canvas_.pixels == framebuffer);
    assert(backend.workerCanvas_.pixels == framebuffer);
    currentCore = renderCore;
    assert(backend.canvas() == &backend.canvas_);
  }
}
`

const compile = spawnSync(process.env.CXX || 'clang++', [
  '-std=c++20', '-include', 'initializer_list', '-fsanitize=address,undefined',
  '-x', 'c++', '-', '-o', output
], { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: dirname(output) } })
assert.equal(compile.status, 0, compile.stderr)
const run = spawnSync(output, [], { encoding: 'utf8' })
assert.equal(run.status, 0, run.stderr)
console.log('Snapshot replay preserves canvas/alpha routing across CPU migration and restores framebuffer bindings')
