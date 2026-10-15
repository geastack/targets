import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'

const repoRoot = fileURLToPath(new URL('../../..', import.meta.url)).replace(/[\\/]$/, '')
const display = readFileSync(`${repoRoot}/targets/esp32/display.cpp`, 'utf8').replace(/\r\n/g, '\n')
const launcher = readFileSync(
  `${repoRoot}/targets/esp32-s3-touch-amoled-2.06/main/launcher_button.cpp`,
  'utf8'
).replace(/\r\n/g, '\n')

assert.match(display, /constexpr int kFlushChunkMin = 2;/)
assert.match(display, /constexpr std::size_t kFlushDmaReserveBytes = 768;/)
assert.match(display, /freeDma < totalBytes \+ kFlushDmaReserveBytes/)
assert.match(launcher, /constexpr std::uint32_t kTaskStackBytes = 2048;/)
