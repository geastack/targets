import assert from 'node:assert/strict'
import { readFileSync, existsSync } from 'node:fs'
import path from 'node:path'
import { spawnSync } from 'node:child_process'
import test from 'node:test'

// Compile the actual hardware reader with a fake I2C peripheral. Keep the
// executable in the caller's existing build output, never in source/scratch.
const source = readFileSync(new URL('../main/touch.cpp', import.meta.url), 'utf8')
const structs = source.slice(source.indexOf('struct TouchSample'), source.indexOf('Touchscreen::Observer'))
const reader = source.slice(source.indexOf('HardwareSample readHardware()'), source.indexOf('void pollLoop()'))
const build = process.env.GEA_TEST_BUILD_DIR || path.join(process.cwd(), 'build')
const binary = path.join(build, 'axs15231b-touch-report-test')

test('AXS report down, drag, lift and next gesture survive stale counts', () => {
  assert.ok(existsSync(build), 'Set GEA_TEST_BUILD_DIR to an existing build output directory')
  const program = `
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
constexpr int ESP_OK = 0;
constexpr int kNativeWidth = 320, kNativeHeight = 480;
int gDevice = 1;
std::uint8_t packet[14]{};
bool commandSent = false;
bool failWrite = false, failRead = false;
int i2c_master_transmit(int, const std::uint8_t *data, std::size_t n, int) {
  assert(n == 11 && data[0] == 0xb5 && data[7] == 14);
  commandSent = true;
  return failWrite ? -1 : 0;
}
int i2c_master_receive(int, std::uint8_t *data, std::size_t n, int) {
  assert(commandSent && n == 14);
  commandSent = false;
  std::memcpy(data, packet, n);
  return failRead ? -1 : 0;
}
${structs}
${reader}
void report(int event, int count, int x, int y) {
  std::memset(packet, 0, sizeof(packet));
  packet[1] = count;
  packet[2] = (event << 6) | (x >> 8);
  packet[3] = x;
  packet[4] = y >> 8;
  packet[5] = y;
}
int main() {
  report(0, 1, 0, 0);
  auto down = readHardware();
  assert(down.valid && down.touch.touching && down.touch.x == 0 && down.touch.y == 0);
  report(2, 1, 319, 479);
  auto move = readHardware();
  assert(move.valid && move.touch.touching && move.touch.x == 319 && move.touch.y == 479);
  report(1, 1, 319, 479); // release retains the previous point and count
  auto up = readHardware();
  assert(up.valid && !up.touch.touching);
  report(3, 1, 319, 479);
  assert(!readHardware().touch.touching);
  report(0, 1, 150, 240); // a fresh gesture must work after release
  assert(readHardware().touch.touching);
  report(2, 0, 150, 240);
  assert(!readHardware().touch.touching);
  report(2, 1, 320, 100);
  assert(!readHardware().valid);
  report(2, 1, 100, 480);
  assert(!readHardware().valid);
  report(2, 1, 100, 100);
  failRead = true;
  assert(!readHardware().valid);
  failRead = false;
  failWrite = true;
  assert(!readHardware().valid);
}
`
  const compile = spawnSync(process.env.CXX || 'c++', ['-std=c++20', '-x', 'c++', '-', '-o', binary], { input: program, encoding: 'utf8' })
  assert.equal(compile.status, 0, compile.stderr)
  const run = spawnSync(binary, [], { encoding: 'utf8' })
  assert.equal(run.status, 0, run.stderr)
})
