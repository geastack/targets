import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { mkdirSync, readFileSync } from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const build = process.env.GEA_TEST_BUILD_DIR || fileURLToPath(new URL('../../../build/', import.meta.url))
mkdirSync(build, { recursive: true })
// Windows cannot start an executable without its suffix.
const binary = path.join(build, `mosaico-startup-memory-test${process.platform === 'win32' ? '.exe' : ''}`)
const board = readFileSync(new URL('../../esp32-s31-espressif-mosaico/main/board_power.cpp', import.meta.url), 'utf8')
const touch = readFileSync(new URL('../chip_bindings/touch/cst9217.cpp', import.meta.url), 'utf8')
const touchHeader = readFileSync(new URL('../chip_bindings/touch/cst9217_controller.h', import.meta.url), 'utf8')

function method(source, signature) {
  const start = source.indexOf(signature)
  assert.notEqual(start, -1, `missing production method ${signature}`)
  const brace = source.indexOf('{', start)
  let depth = 1, end = brace + 1
  while (depth && end < source.length) {
    if (source[end] === '{') depth++
    else if (source[end] === '}') depth--
    end++
  }
  assert.equal(depth, 0)
  return source.slice(start, end)
}

function run(program) {
  const compiled = spawnSync(process.env.CXX || (process.platform === 'win32' ? 'clang++' : 'c++'), [
    '-std=c++20', '-O1', '-g', '-fsanitize=address,undefined',
    '-x', 'c++', '-', '-o', binary,
  ], { input: program, encoding: 'utf8', env: { ...process.env, TMPDIR: build } })
  assert.ifError(compiled.error)
  assert.equal(compiled.status, 0, compiled.stderr)
  const ran = spawnSync(binary, [], { encoding: 'utf8', timeout: 15000 })
  assert.ifError(ran.error)
  assert.equal(ran.status, 0, ran.stderr)
}

const platform = `
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_FAIL = 1, ESP_ERR_NO_MEM = 2, ESP_ERR_INVALID_STATE = 3;
constexpr unsigned MALLOC_CAP_INTERNAL = 1, MALLOC_CAP_8BIT = 2, MALLOC_CAP_SPIRAM = 4;
struct Allocation { std::size_t bytes; unsigned caps; };
std::map<void *, Allocation> live;
int allocations = 0, frees = 0;
bool failAllocation = false;
void *heap_caps_malloc(std::size_t bytes, unsigned caps) {
  ++allocations;
  if (failAllocation) { failAllocation = false; return nullptr; }
  void *p = std::malloc(bytes);
  assert(p);
  live.emplace(p, Allocation{bytes, caps});
  return p;
}
void heap_caps_free(void *p) {
  assert(p && live.erase(p) == 1);
  ++frees;
  std::free(p);
}
`

// Execute migration itself with flash calls replaced by fakes. Every destructive
// operation asserts that the comparison allocation has already been released.
run(`${platform}
#define CONFIG_PARTITION_TABLE_OFFSET 0x8000
#define pdMS_TO_TICKS(ms) (ms)
std::array<std::uint8_t, 4097> table{};
const std::uint8_t *partitionTableStart = table.data(), *partitionTableEnd = table.data() + 64;
bool sameTable = false, failRead = false, missingPartition = false;
int failOperation = 0, operations = 0, reads = 0, erases = 0, writes = 0, restarts = 0;
struct esp_partition_t { std::uint32_t address = 0x10000; } running;
bool builtPartition(int type, int, std::uint32_t &offset, std::uint32_t &size) {
  if (missingPartition) return false;
  offset = type == 0 ? 0x10000 : 0xd000;
  size = 4096;
  return true;
}
const esp_partition_t *esp_ota_get_running_partition() { return &running; }
int esp_flash_read(void *, void *destination, unsigned, std::size_t bytes) {
  ++reads;
  assert(live.size() == 1 && live.at(destination).bytes == bytes);
  assert(live.at(destination).caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (failRead) return ESP_FAIL;
  std::memcpy(destination, table.data(), bytes);
  if (!sameTable) static_cast<std::uint8_t *>(destination)[0] ^= 1;
  return ESP_OK;
}
int esp_flash_erase_region(void *, unsigned, unsigned) {
  assert(live.empty());
  ++erases;
  return ++operations == failOperation ? ESP_FAIL : ESP_OK;
}
int esp_flash_write(void *, const void *, unsigned, std::size_t) {
  assert(live.empty());
  ++writes;
  return ++operations == failOperation ? ESP_FAIL : ESP_OK;
}
const char *esp_err_to_name(int) { return "fake"; }
void vTaskDelay(int) {}
void esp_restart() { assert(live.empty()); ++restarts; }
${method(board, '  void migratePartitionTable()')}
void reset() {
  assert(live.empty());
  allocations = frees = operations = reads = erases = writes = restarts = 0;
  failAllocation = sameTable = failRead = missingPartition = false;
  failOperation = 0;
  running.address = 0x10000;
  partitionTableEnd = table.data() + 64;
}
int main() {
  for (std::size_t size : {std::size_t(0), std::size_t(4097)}) {
    reset(); partitionTableEnd = table.data() + size; migratePartitionTable();
    assert(allocations == 0 && reads == 0 && operations == 0);
  }
  reset(); failAllocation = true; migratePartitionTable();
  assert(allocations == 1 && frees == 0 && reads == 0 && operations == 0);
  reset(); failRead = true; migratePartitionTable();
  assert(allocations == 1 && frees == 1 && operations == 0);
  reset(); sameTable = true; migratePartitionTable();
  assert(allocations == 1 && frees == 1 && operations == 0);
  reset(); missingPartition = true; migratePartitionTable();
  assert(frees == 1 && operations == 0);
  reset(); running.address = 0x20000; migratePartitionTable();
  assert(frees == 1 && operations == 0);
  for (int fault = 1; fault <= 3; ++fault) {
    reset(); failOperation = fault; migratePartitionTable();
    assert(frees == 1 && operations == fault && restarts == 0);
  }
  reset(); migratePartitionTable();
  assert(frees == 1 && erases == 2 && writes == 1 && restarts == 1);
  assert(live.empty());
}
`)

// Compile the real controller declaration in both configurations. A fake task
// creator checks the unchanged stack size/priority and internal TCB ownership.
for (const external of [0, 1]) {
  run(`${platform}
#define GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL ${external}
using StackType_t = std::uint8_t;
using TaskHandle_t = void *;
struct StaticTask_t { char bytes[128]; };
using i2c_master_bus_handle_t = void *;
using esp_lcd_panel_io_handle_t = void *;
using esp_lcd_touch_handle_t = void *;
constexpr int kResetPin = 1, kInterruptPin = 2, kTaskPriority = 10;
bool busAvailable = true, failAttach = false, failCreation = false;
int attaches = 0, creates = 0;
namespace gea::platform::touch {
enum class Phase { Down, Move, Up };
struct Touchscreen {
  using Observer = void (*)(Phase, bool, int, int);
  using PointerObserver = void (*)(Phase, bool, int, int, int);
};
}
namespace gea::platform::i2c {
struct Bus {
  static Bus primary() { return {}; }
  bool available() { return busAvailable; }
  void *nativeHandle() { return reinterpret_cast<void *>(1); }
};
}
#define private public
${touchHeader.replace(/^#(?:include|pragma) .*\n/gm, '')}
#undef private
using gea::platform::esp32::chip_bindings::cst9217::TouchController;
TaskHandle_t xTaskCreateStatic(void (*)(void *), const char *name, std::uint32_t bytes,
                              void *context, int priority, StackType_t *stack, StaticTask_t *storage) {
  auto *controller = static_cast<TouchController *>(context);
  assert(std::strcmp(name, "cst9217-touch") == 0 && bytes == 4096 && priority == 10);
  assert(storage == &controller->taskControlBlock_ && live.count(controller) == 0);
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
  assert(live.at(stack).bytes == 4096);
  assert(live.at(stack).caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#else
  assert(stack == controller->taskStack_ && live.empty());
#endif
  ++creates;
  if (failCreation) { failCreation = false; return nullptr; }
  return storage;
}
namespace gea::platform::esp32::chip_bindings::cst9217 {
esp_err_t TouchController::attachTouchDevice() {
  ++attaches;
  // A driver can publish its handle before a later interrupt setup fails.
  touch_ = reinterpret_cast<void *>(1);
  return failAttach ? ESP_FAIL : ESP_OK;
}
void TouchController::taskEntry(void *) {}
${method(touch, 'esp_err_t TouchController::begin()')}
${method(touch, 'esp_err_t TouchController::startTask()')}
}
int main() {
  TouchController controller;
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
  static_assert(sizeof(TouchController) < 512);
#else
  static_assert(sizeof(TouchController) > 4096);
#endif
  busAvailable = false;
  assert(controller.begin() == ESP_ERR_INVALID_STATE && attaches == 0 && allocations == 0);
  busAvailable = true; failAttach = true;
  assert(controller.begin() == ESP_FAIL && attaches == 1 && creates == 0);
  assert(!controller.deviceAttached_ && !controller.initialized_);
  failAttach = false;
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
  failAllocation = true;
  assert(controller.begin() == ESP_ERR_NO_MEM);
  assert(attaches == 2 && allocations == 1 && creates == 0);
#endif
  failCreation = true;
  assert(controller.begin() == ESP_ERR_NO_MEM && creates == 1 && attaches == 2);
  assert(controller.deviceAttached_ && !controller.initialized_);
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
  assert(allocations == 2 && frees == 1 && controller.taskStack_ == nullptr);
#else
  assert(allocations == 0 && frees == 0);
#endif
  assert(controller.begin() == ESP_OK && creates == 2 && attaches == 2);
  assert(controller.initialized_ && controller.task_ == &controller.taskControlBlock_);
  const int allocated = allocations;
  for (int i = 0; i < 20; ++i) {
    assert(controller.begin() == ESP_OK && controller.startTask() == ESP_OK);
  }
  assert(allocations == allocated && creates == 2 && attaches == 2);
#if GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL
  assert(allocations == 3 && frees == 1);
  // Only the fake task is stopped; production retains this stack for its task.
  heap_caps_free(controller.taskStack_);
#endif
  assert(live.empty());
}
`)
}
console.log('Mosaico startup memory: partition cleanup and both touch-stack failure/retry paths passed')
