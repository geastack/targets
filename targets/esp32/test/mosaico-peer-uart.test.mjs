import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const include = fileURLToPath(new URL('../../esp32-s31-espressif-mosaico/main/include/', import.meta.url))
function compile(defines, assertions = '') {
  const result = spawnSync(process.env.CXX || 'c++', [
    '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsyntax-only', '-x', 'c++',
    '-I', include, ...Object.entries(defines).map(([key, value]) => `-DGEA_MOSAICO_PEER_${key}_GPIO=${value}`), '-'
  ], {
    input: `#include "mosaico_peer_uart_pins.h"\nnamespace peer = gea::platform::board::peer;\n${assertions}\n`,
    encoding: 'utf8'
  })
  assert.ifError(result.error)
  return result
}

const defaults = compile({}, `
static_assert(peer::left.tx == 15 && peer::left.rx == 4);
static_assert(peer::right.tx == 5 && peer::right.rx == 38);
`)
assert.equal(defaults.status, 0, defaults.stderr)

// A remapped pair joins right GPIO46 -> left GPIO0 and right GPIO47 <- left GPIO1.
// The same image must provide complementary TX/RX for both neighbours.
const harness = { LEFT_TX: 1, LEFT_RX: 0, RIGHT_TX: 46, RIGHT_RX: 47 }
const remapped = compile(harness, `
static_assert(peer::right.tx == 46 && peer::left.rx == 0);
static_assert(peer::right.rx == 47 && peer::left.tx == 1);
`)
assert.equal(remapped.status, 0, remapped.stderr)

for (const pin of [-1, 3, 20, 33, 34, 42, 49, 56, 58, 59, 60, 61, 62]) {
  const rejected = compile({ ...harness, RIGHT_TX: pin })
  assert.notEqual(rejected.status, 0, `reserved/invalid GPIO${pin} was accepted`)
  assert.match(rejected.stderr, /Mosaico peer UART pins must be unclaimed expansion GPIOs/)
}

for (const [a, b] of [
  ['LEFT_TX', 'LEFT_RX'], ['LEFT_TX', 'RIGHT_TX'], ['LEFT_TX', 'RIGHT_RX'],
  ['LEFT_RX', 'RIGHT_TX'], ['LEFT_RX', 'RIGHT_RX'], ['RIGHT_TX', 'RIGHT_RX']
]) {
  const rejected = compile({ ...harness, [b]: harness[a] })
  assert.notEqual(rejected.status, 0, `${a} and ${b} shared a GPIO`)
  assert.match(rejected.stderr, /Mosaico peer UART requires four distinct GPIOs/)
}

console.log('Mosaico peer UART: default/custom harnesses and pin ownership checks passed')
