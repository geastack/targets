#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

const boardCmake = source('targets/esp32-s3-touch-amoled-2.06/main/CMakeLists.txt')
// A reserved pool is allowed only inside the direct-canvas profile, where the
// radios are off and the internal RAM is genuinely free (the block says why).
// The default profile must not take it: app memory needs vary.
for (const match of boardCmake.matchAll(/GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES=\d+/g)) {
  const enclosingList = boardCmake.slice(0, match.index).lastIndexOf('list(APPEND ')
  const listName = boardCmake.slice(enclosingList, enclosingList + 80)
  assert.match(
    listName,
    /GEA_EMBEDDED_DIRECT_CANVAS_COMPILE_DEFINITIONS/,
    'a fixed DMA-internal flush pool may only be reserved in the radios-off direct-canvas profile'
  )
}

const sdkconfigDefaults = source('targets/esp32-s3-touch-amoled-2.06/sdkconfig.defaults')
assert.match(
  sdkconfigDefaults,
  /^CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=0$/m,
  'ESP32-S3 AMOLED board must disable IDF SPIRAM_MALLOC_RESERVE_INTERNAL; the hidden 32 KiB DMA reserve splits the internal heap map'
)

// That the shared display backend only reserves a pool when a board asks for
// one is asserted in targets/esp32/test, where that source lives.
