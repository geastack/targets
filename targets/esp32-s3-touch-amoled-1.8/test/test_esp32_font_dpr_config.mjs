#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const cmakePath = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-1.8/main/CMakeLists.txt')
const cmake = readFileSync(cmakePath, 'utf8')

assert.match(
  cmake,
  /set\(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO\s+"1\.5"\s+CACHE\s+STRING/,
  'ESP32 AMOLED target should declare its CSS device pixel ratio once'
)

assert.match(
  cmake,
  /GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=\$\{GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO\}/,
  'runtime CSS DPR compile definition should use the target DPR variable'
)

const fontDprArgs = [...cmake.matchAll(/--font-device-pixel-ratio\s+([^\s)]+)/g)]
  .map((match) => match[1])

assert.deepEqual(
  fontDprArgs,
  ['${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}'],
  'application font generation should use only the target DPR'
)
