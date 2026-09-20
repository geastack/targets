#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

const boardCmake = source('targets/esp32-s3-touch-amoled-1.8/main/CMakeLists.txt')
assert.doesNotMatch(
  boardCmake,
  /GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES=\d+\b/,
  'ESP32-S3 AMOLED board must not reserve a fixed DMA-internal flush pool; app memory needs vary'
)

const sdkconfigDefaults = source('targets/esp32-s3-touch-amoled-1.8/sdkconfig.defaults')
assert.match(
  sdkconfigDefaults,
  /^CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=0$/m,
  'ESP32-S3 AMOLED board must disable IDF SPIRAM_MALLOC_RESERVE_INTERNAL; the hidden 32 KiB DMA reserve splits the internal heap map'
)

// That the shared display backend only reserves a pool when a board asks for
// one is asserted in targets/esp32/test, where that source lives.
