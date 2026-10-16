#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { fileURLToPath } from 'node:url'

const repoRoot = resolve(fileURLToPath(new URL('../../..', import.meta.url)))
const targetRoot = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.06')
const manifest = readFileSync(resolve(targetRoot, 'main/idf_component.yml'), 'utf8').replace(/\r\n/g, '\n')
const dependencies = JSON.parse(readFileSync(resolve(targetRoot, 'main/gea-dependencies.json'), 'utf8').replace(/\r\n/g, '\n'))
const escaped = (name) => name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
const inManifest = (name) => new RegExp(`^\\s+${escaped(name)}:`, 'm').test(manifest)
// Always-required SDKs live in idf_component.yml; capability-gated ones in
// gea-dependencies.json, which the CLI capability analyzer selects from.
const declared = (name) => inManifest(name) || name in dependencies
const gatedBy = (name, capability) => !inManifest(name) && (dependencies[name]?.when ?? []).includes(capability)
const cmakeLists = readFileSync(resolve(targetRoot, 'main/CMakeLists.txt'), 'utf8').replace(/\r\n/g, '\n')

// Echo processing is opt-in: neither its library nor its adapter belongs in
// apps using raw audio or the default half-duplex driver.
assert.ok(gatedBy('espressif/esp-sr', 'aec'), 'esp-sr is selected only by the aec capability')
assert.match(cmakeLists, /if\("\$\{GEA_EMBEDDED_CAPABILITY_AEC\}" STREQUAL "1"\)[\s\S]*?echo_cancellation\.cpp[\s\S]*?REQUIRES esp-sr/)

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
  'espressif/esp_new_jpeg',
  'espressif/esp_audio_codec',
  'espressif/esp_codec_dev',
  'espressif/esp_lcd_co5300',
  'esp_lcd_panel_io_additions',
  'espressif/esp_peer',
  'espressif/esp_websocket_client'
]) {
  assert.ok(declared(requiredDependency), `ESP32 target should still declare required dependency ${requiredDependency}`)
}

// A framework-only dependency leaves main's native MJPEG source compiling its
// unavailable-decoder fallback even though the decoder library was downloaded.
assert.match(cmakeLists, /if\(GEA_EMBEDDED_CAPABILITY_NETWORK\)\s+list\(APPEND GEA_EMBEDDED_TARGET_REQUIRES\s+esp_new_jpeg/)
