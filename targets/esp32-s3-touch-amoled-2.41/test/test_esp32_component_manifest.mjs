#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const targetRoot = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.06')
const manifest = readFileSync(resolve(targetRoot, 'main/idf_component.yml'), 'utf8')
const lockfile = readFileSync(resolve(targetRoot, 'dependencies.lock'), 'utf8')
const cmakeLists = readFileSync(resolve(targetRoot, 'main/CMakeLists.txt'), 'utf8')

const unusedDirectDependencies = [
  '78/esp-opus',
  'espressif/esp-sr'
]

for (const dependency of unusedDirectDependencies) {
  assert.doesNotMatch(
    manifest,
    new RegExp(`^\\s+${dependency.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}:`, 'm'),
    `ESP32 main manifest should not declare unused direct dependency ${dependency}`
  )
  assert.doesNotMatch(
    lockfile,
    new RegExp(`^- ${dependency.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}$`, 'm'),
    `ESP32 dependency lockfile should not list unused direct dependency ${dependency}`
  )
}

assert.match(
  cmakeLists,
  /set\(GEATSC_SOURCE_LIST "\$\{GENERATED_APP_DIR\}\/geatsc-sources\.txt"\)/,
  'Standalone geatsc app output should consume geatsc-sources.txt'
)
assert.match(
  cmakeLists,
  /gea_geatsc_add_generated_archive\([\s\S]*SOURCE_LIST "\$\{GEATSC_SOURCE_LIST\}"[\s\S]*COMPILE_OPTIONS \$\{GEATSC_COMPILE_OPTIONS\}/,
  'Standalone geatsc app output should be compiled through the generated archive helper'
)
assert.doesNotMatch(
  cmakeLists,
  /GEATSC_PROGRAM_CPP/,
  'Standalone geatsc app output should not depend on a generated program.cpp wrapper'
)

for (const requiredDependency of [
  'espressif/esp_codec_dev',
  'espressif/esp_lcd_co5300',
  'esp_lcd_panel_io_additions',
  'espressif/esp_peer',
  'espressif/esp_websocket_client'
]) {
  assert.match(
    manifest,
    new RegExp(`^\\s+${requiredDependency.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}:`, 'm'),
    `ESP32 main manifest should still declare required dependency ${requiredDependency}`
  )
}
