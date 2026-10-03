// Verify the production pre-clamp gain with both AMOLED board calibrations.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'

const source = readFileSync(new URL('../chip_bindings/audio/es8311.cpp', import.meta.url), 'utf8')
const begin = source.indexOf('#ifndef GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE')
const end = source.indexOf('template <typename AudioConfig>', begin)
assert.ok(begin >= 0 && end > begin)
const gain = source.slice(begin, end)

for (const baseline of [53.3408610, 53.0884444]) {
  for (const scale of [undefined, 0.501187234, 0]) {
    const program = `
      ${scale === undefined ? '' : `#define GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE ${scale.toFixed(9)}f`}
      namespace gea::platform::board { constexpr float audio = ${baseline}f; }
      constexpr float aecOutputGainOf(float value) { return value; }
      ${gain}
      constexpr float expected = ${(baseline * (scale ?? 1)).toFixed(12)}f;
      static_assert(kAecOutputGain >= expected - 0.0001f);
      static_assert(kAecOutputGain <= expected + 0.0001f);
    `
    const result = spawnSync(process.env.CXX || 'c++', ['-std=c++20', '-fsyntax-only', '-x', 'c++', '-'], {
      input: program,
      encoding: 'utf8',
    })
    assert.equal(result.status, 0, result.stderr)
  }
}
for (const scale of [-1.0, 65.0]) {
  const result = spawnSync(process.env.CXX || 'c++', ['-std=c++20', '-fsyntax-only', '-x', 'c++', '-'], {
    input: `
      #define GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE ${scale.toFixed(1)}f
      namespace gea::platform::board { constexpr float audio = 1.0f; }
      constexpr float aecOutputGainOf(float value) { return value; }
      ${gain}
    `,
    encoding: 'utf8',
  })
  assert.notEqual(result.status, 0)
  assert.match(result.stderr, /AEC output gain scale must be between 0 and 64/)
}
console.log('AEC output gain: both board baselines, -6 dB, mute and invalid scales verified')
