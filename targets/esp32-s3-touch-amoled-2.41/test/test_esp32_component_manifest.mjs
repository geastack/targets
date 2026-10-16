#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

const repoRoot = resolve(fileURLToPath(new URL('../../..', import.meta.url)))
const targetRoot = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.41')
const manifest = readFileSync(resolve(targetRoot, 'main/idf_component.yml'), 'utf8').replace(/\r\n/g, '\n')
const dependencies = JSON.parse(readFileSync(resolve(targetRoot, 'main/gea-dependencies.json'), 'utf8').replace(/\r\n/g, '\n'))
const escaped = (name) => name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
const inManifest = (name) => new RegExp(`^\\s+${escaped(name)}:`, 'm').test(manifest)
// Always-required SDKs live in idf_component.yml; capability-gated ones in
// gea-dependencies.json, which the CLI capability analyzer selects from.
const declared = (name) => inManifest(name) || name in dependencies
const gatedBy = (name, capability) => !inManifest(name) && (dependencies[name]?.when ?? []).includes(capability)
const lockfile = readFileSync(resolve(targetRoot, 'dependencies.lock'), 'utf8').replace(/\r\n/g, '\n')
// This target's main/CMakeLists.txt includes the 1.8 board's, which owns the
// geatsc archive wiring asserted below.
const cmakeLists = readFileSync(resolve(repoRoot, 'targets/esp32-s3-touch-amoled-1.8/main/CMakeLists.txt'), 'utf8').replace(/\r\n/g, '\n')

// This board carries no audio codec; the 1.8 build it includes adds the
// echo-cancellation library only for an app that asks for aec, never by default.
// 78/esp-opus IS used (host/rtc_esp.cpp), behind the NETWORK capability rule.
assert.ok(gatedBy('espressif/esp-sr', 'aec'), 'esp-sr is selected only by the aec capability')
const unusedDirectDependencies = []

for (const dependency of unusedDirectDependencies) {
  assert.ok(!declared(dependency), `ESP32 target should not declare unused direct dependency ${dependency}`)
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

assert.ok(gatedBy('78/esp-opus', 'network'), 'esp-opus is selected only by the network capability')

for (const requiredDependency of [
  'espressif/esp_h264',
  'espressif/esp_lcd_sh8601',
  'esp_lcd_panel_io_additions',
  'espressif/esp_peer',
  'espressif/esp_websocket_client'
]) {
  assert.ok(declared(requiredDependency), `ESP32 target should still declare required dependency ${requiredDependency}`)
}
