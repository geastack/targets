import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { readFileSync } from 'node:fs'
import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../services/device_control.cpp', import.meta.url), 'utf8')
const header = fileURLToPath(new URL('../services/comparison_gesture.h', import.meta.url))
const output = fileURLToPath(new URL('../../../build/comparison-control-test', import.meta.url))

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

const helpers = source.slice(
  source.indexOf('namespace diagnostic_gesture ='),
  source.indexOf('// Optional per-target GEADEV'),
)
const commandsStart = source.indexOf('} else if (tokenEquals(command, "TOUCH"))') + 2
const commandsEnd = source.indexOf('} else if (tokenEquals(command, "MEM"))', commandsStart) + 1

assert.ok(commandsStart > 1 && commandsEnd > commandsStart)
const program = `
#include "${header}"
#include "services/comparison_upload_capture.h"
#include <cassert>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#define GEA_DIAGNOSTIC_NODE_STYLE(node) ((node).style)
#define pdMS_TO_TICKS(value) (value)
constexpr int MALLOC_CAP_SPIRAM=1, MALLOC_CAP_8BIT=2, MALLOC_CAP_INTERNAL=4;
size_t heap_caps_get_total_size(int) { return 1000; }
size_t heap_caps_get_free_size(int) { return 800; }
size_t heap_caps_get_minimum_free_size(int) { return 700; }
size_t heap_caps_get_largest_free_block(int) { return 600; }
uint32_t gea_display_completed_chunks() { return 10; }
int allocations = 0;
int64_t deviceUs = 1000;
bool locked = false;
bool captureReady = true, uploadComplete = true;
int invalidations = 0;
bool gea_frame_capture_boundary_ready() { return captureReady; }
bool gea_display_wait_for_uploads() { assert(locked); return uploadComplete; }
namespace gea::platform::display {
constexpr int kNativeWidth=2,kNativeHeight=2;
struct Display { static void invalidate() { assert(locked); ++invalidations; } };
}
int64_t esp_timer_get_time() { return deviceUs; }
void vTaskDelay(int ms) { deviceUs += ms * 1000; }
void taskYIELD() { ++deviceUs; }
void *heap_caps_malloc(size_t size, int) { ++allocations; return std::malloc(size); }
void heap_caps_free(void *value) { if (value) { --allocations; std::free(value); } }
namespace gea::framework::services {
struct AppState {
  static void lock() { assert(!locked); locked = true; }
  static void unlock() { assert(locked); locked = false; }
};
}
namespace gea::embedded::ui {
constexpr int kDisplayNone=1;
struct Node {
  int parent=-1, first_child=-1, next_sibling=-1;
  struct { int display=0; } style;
  struct { int x=0,y=0,width=100,height=100,scroll_x=0,scroll_y=0,
    scroll_content_width=100,scroll_content_height=100; } layout;
  std::string text, classes;
};
struct Tree {
  Node nodes[6];
  Tree() { nodes[1].parent=0; nodes[1].classes="menu-icon";
           nodes[1].text="quoted " + std::string(1, char(34)) + " label"; nodes[2].classes="menu-icon";
           nodes[3].parent=0; nodes[3].classes="switch on"; nodes[3].first_child=4;
           nodes[4].parent=3; nodes[4].first_child=5;
           nodes[5].parent=4; nodes[5].text="nested label"; }
  static Tree &instance() { static Tree value; return value; }
  int mountedRoot() { return 0; }
  int nodeCount() { return 6; }
  void markDisplayListDirty() { assert(locked); ++invalidations; }
  Node &node(int id) { return nodes[id]; }
  bool hasClass(int id, const char *value) { return (" " + nodes[id].classes + " ").find(" " + std::string(value) + " ")!=std::string::npos; }
};
struct ViewRenderer {
  static void transformedRectCorners(const Node &,bool,int x,int y,int w,int h,int16_t *xs,int16_t *ys) {
    assert(locked); xs[0]=xs[3]=x; xs[1]=xs[2]=x+w;
    ys[0]=ys[1]=y; ys[2]=ys[3]=y+h;
  }
};
}
namespace gea::platform::touch {
enum class Phase { Down=1,Move=2,Up=3 };
struct Touchscreen {
  static void injectEvent(Phase phase,bool touching,int x,int y) {
    gea::embedded::ui::Tree::instance().node(1).layout.x=x;
    gea::platform::comparison::input::consumed(deviceUs,static_cast<int>(phase),touching,x,y,0,x,y);
  }
};
}
${method('char *skipSpace(char *p)')}
${method('char *nextToken(char *&p)')}
${method('bool tokenEquals(const char *a, const char *b)')}
${method('bool parseOnOff(const char *token, bool *out)')}
${source.slice(source.indexOf('class Base64Writer {'), source.indexOf('bool captureSnapshotRgb565('))}
${helpers.replace(/#endif\s*$/, '')}
void command(const char *text) {
  char buffer[128]; std::snprintf(buffer,sizeof(buffer),"%s",text);
  char *cursor=buffer;
  const char *command=nextToken(cursor);
  if(false) {} ${source.slice(commandsStart, commandsEnd)}
}
int main() {
  assert(!gea::platform::comparison::input::recording.load() && allocations==0);
  {
    namespace upload = gea::platform::comparison::upload;
    uint16_t storage[16]{};
    assert(upload::begin(storage,3,2));
    upload::window(0,0,2,0);
    const uint8_t first[] = {0x12,0x34,0xab,0xcd};
    const uint8_t rest[] = {0x56,0x78};
    upload::submitted(first,4); upload::submitted(rest,2);
    upload::window(0,1,1,1); upload::submitted(first,4);
    upload::restart(); upload::submitted(first,4);
    upload::window(2,1,2,1); upload::submitted(rest,2);
    auto image=upload::end();
    assert(image.covered==6 && image.submittedPixels==8 && image.errors==0);
    assert(storage[0]==0x1234 && storage[1]==0xabcd && storage[2]==0x5678 && storage[5]==0x5678);
    upload::submitted(first,4); assert(storage[0]==0x1234);
    assert(upload::begin(storage,3,2));
    upload::window(-2,-2,-1,-1); upload::submitted(first,4);
    image=upload::end(); assert(image.covered==0 && image.errors==2);
    assert(upload::begin(storage,3,2));
    upload::window(0,0,0,0); upload::submitted(first,4); upload::failed();
    image=upload::end(); assert(image.covered==0 && image.errors==2);
    gea::platform::comparison::begin("blocked","idle",deviceUs);
    assert(!upload::begin(storage,3,2));
    gea::platform::comparison::end(deviceUs);
  }
  command("SCROLLSTATE menu-icon"); assert(allocations==0 && !locked);
  command("SCROLLSTATE switch"); assert(allocations==0 && !locked);
  command("COMPLETION 0"); assert(!gea::platform::comparison::completionFence.load());
  command("COMPLETION 1"); assert(gea::platform::comparison::completionFence.load());
  command("BENCH BEGIN compile-check idle"); command("BENCH END");
  command("COMPLETION 0"); command("BENCH BEGIN unfenced idle"); command("BENCH END");
  command("COMPLETION 1");
  command("UPLOADSHOT ARM"); assert(allocations==1 && invalidations==2);
  command("GESTURE RESET"); assert(allocations==1 && !gDiagnosticGesture);
  gea::platform::comparison::upload::window(0,0,1,1);
  const uint8_t wire[] = {0xf8,0x00,0x07,0xe0,0x00,0x1f,0xff,0xff};
  gea::platform::comparison::upload::submitted(wire, sizeof(wire));
  command("UPLOADSHOT READ"); assert(allocations==0 && !locked);
  captureReady=false; command("UPLOADSHOT ARM"); assert(allocations==0 && !locked);
  captureReady=true; command("UPLOADSHOT ARM"); assert(allocations==1);
  command("UPLOADSHOT READ"); assert(allocations==0 && !locked);
  command("UPLOADSHOT ARM"); assert(allocations==1);
  command("UPLOADSHOT DISARM"); assert(allocations==0);
  command("INPUTTRACE BEGIN"); assert(allocations==1);
  command("TOUCH down 233 220"); command("TOUCH move 280 220"); command("TOUCH up 280 220");
  command("INPUTTRACE END"); assert(allocations==0);
  command("CLOCKFREEZE 1234"); assert(gea::platform::comparison::frozenEpochSeconds.load()==1234);
  gea::platform::comparison::begin("one","idle",deviceUs);
  command("CLOCKFREEZE 0"); assert(gea::platform::comparison::frozenEpochSeconds.load()==1234);
  command("INPUTTRACE BEGIN"); assert(allocations==0);
  gea::platform::comparison::end(deviceUs);
  command("CLOCKFREEZE 0");
  command("GESTURE RESET"); assert(allocations==1);
  command("GESTURE TOUCH 0 1 233 220"); command("GESTURE TOUCH 33 1 280 220");
  command("GESTURE TOUCH 66 0 280 220"); command("GESTURE OBSERVE 0");
  command("GESTURE OBSERVE 33"); command("GESTURE OBSERVE 99");
  command("GESTURE BEGIN"); assert(allocations==2 && !locked);
  command("GESTURE RESULT"); command("GESTURE CLEAR"); assert(allocations==0);
}
`
const compiled = spawnSync(
  process.env.CXX || 'clang++',
  [
    '-std=c++20',
    '-DGEA_EMBEDDED_COMPARISON_BENCHMARK=1',
    '-DCONFIG_COMPILER_OPTIMIZATION_PERF=1',
    '-Wformat',
    '-Werror=format',
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
const records = result.stdout
  .split('\n')
  .filter((line) => /^SW(?:SCROLL|INPUT|GESTURE) /.test(line))
const parsed = records.map((line) => JSON.parse(line.slice(line.indexOf(' ') + 1)))

assert.equal(parsed[0].nodes.length, 1)
assert.equal(parsed[0].nodes[0].class, 'menu-icon')
assert.match(parsed[0].nodes[0].text, /quoted/)
assert.equal(parsed[1].nodes[0].class, 'switch')
assert.equal(parsed[1].nodes[0].selected_class, 1)
assert.equal(parsed[1].nodes[0].text, 'nested label')
assert.equal(parsed[1].nodes[0].children_count, 1)
assert.equal(parsed[1].nodes[0].text_truncated, false)
const pixels = Buffer.from(result.stdout.match(/^GEADEV:DATA (.+)$/m)[1], 'base64')

assert.deepEqual(
  [0, 4, 8, 12].map((offset) => pixels.readUInt16LE(offset + 2)),
  [0xf800, 0x07e0, 0x001f, 0xffff],
)
assert.match(result.stdout, /UPLOADSHOT BEGIN .*source=co5300-submitted-rgb565 completed_dma=1/)
assert.match(result.stdout, /UPLOADSHOT incomplete covered=0 required=4 errors=0/)
const samples = result.stdout
  .split('\n')
  .filter((line) => line.startsWith('SWBENCH '))
  .map((line) => JSON.parse(line.slice(8)))

assert.equal(samples[0].optimization, 'O2')
assert.equal(samples[0].variant, 'benchmark-fenced')
assert.equal(samples[0].completion_fence, true)
assert.equal(samples[0].presented_frames, 0)
assert.equal(samples[1].variant, 'benchmark-unfenced')
assert.equal(samples[1].completion_fence, false)
assert.equal(samples[1].presented_frames, null)
assert.equal(parsed[2].samples.length, 3)
assert.equal(parsed[3].events.length, 3)
assert.deepEqual(
  parsed[3].events.map((event) => event.phase),
  [1, 2, 3],
)
assert.equal(parsed[3].observations.length, 3)
assert.equal(parsed[3].pointer_reads.length, 3)
assert.equal(parsed[3].events[1].actual_at_us - parsed[3].clock_origin_us, 33000)
console.log(
  'Comparison control: production commands compile, snapshots are locked, JSON is valid, traces are bounded and allocation is released',
)
