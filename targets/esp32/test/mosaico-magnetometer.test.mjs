import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const board = new URL('../../esp32-s31-espressif-mosaico/', import.meta.url)
const cmake = readFileSync(new URL('main/CMakeLists.txt', board), 'utf8')
const manifest = readFileSync(new URL('main/idf_component.yml', board), 'utf8')
const composition = readFileSync(new URL('main/include/board.h', board), 'utf8')
assert.match(manifest, /espressif\/bmm150:\s*'1\.1\.0'/)
assert.match(cmake, /chip_bindings\/magnetometer\/bmm150\.cpp/)
assert.match(cmake, /target_compile_options\(\$\{_mosaico_bmm150_lib\} PUBLIC -UBMM150_USE_FLOATING_POINT\)/)
assert.match(cmake, /-Wl,-u,gea_magnetometer_open/)
assert.match(composition, /magnetometerAddresses\{0x11, 0x12\}/)

// The BMM150 managed component defaults to a different struct layout. A
// mismatched build must fail at the adapter boundary, never reinterpret the
// driver's float values as the integers consumed by magnetic calibration.
const fixture = fileURLToPath(new URL('magnetometer-fixture/', import.meta.url))
const source = fileURLToPath(new URL('../chip_bindings/magnetometer/bmm150.cpp', import.meta.url))
const include = fileURLToPath(new URL('../../../../core/packages/core/include/', import.meta.url))
for (const floating of [false, true]) {
  const result = spawnSync(process.env.CXX || 'c++', [
    '-std=c++20', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
    '-I', fixture, '-I', include,
    ...(floating ? ['-DBMM150_USE_FLOATING_POINT'] : []), source,
  ], { encoding: 'utf8' })
  assert.ifError(result.error)
  if (floating) {
    assert.notEqual(result.status, 0)
    assert.match(result.stderr, /BMM150 and its consumers must use the same integer ABI/)
  } else {
    assert.equal(result.status, 0, result.stderr)
  }
}
console.log('Mosaico magnetometers: dependency, shared sensor order and integer ABI checks passed')
