#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const targetRoot = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.06')
const cmakeLists = readFileSync(resolve(targetRoot, 'main/CMakeLists.txt'), 'utf8')
const appMain = readFileSync(resolve(targetRoot, 'main/app_main.cpp'), 'utf8')
const sdkconfigDefaults = readFileSync(resolve(targetRoot, 'sdkconfig.defaults'), 'utf8')

const compileDefinitions = cmakeLists.match(/target_compile_definitions\(\$\{COMPONENT_LIB\} PRIVATE([\s\S]*?)\n\)/)?.[1] ?? ''

function definitionValue(name) {
  const match = compileDefinitions.match(new RegExp(`\\b${name}=(-?\\d+)\\b`))
  return match?.[1] ?? null
}

assert.equal(
  definitionValue('GEA_EMBEDDED_APP_FRAME_TASK_CORE'),
  '0',
  'app_frame timer task should remain pinned to core 0'
)

assert.equal(
  definitionValue('GEA_EMBEDDED_GEA_MAIN_TASK_CORE'),
  null,
  'runtime should run on IDF main_task; do not resurrect a second gea_main task'
)

assert.match(
  appMain,
  /constexpr int kAppMainTaskStack = CONFIG_ESP_MAIN_TASK_STACK_SIZE;/,
  'app_main should use the IDF main_task stack for the runtime loop'
)

assert.match(
  appMain,
  /vTaskPrioritySet\(nullptr,\s*kRuntimeTaskPriority\);[\s\S]*?startDeviceControlTask\(\);[\s\S]*?runRuntime\(\);/,
  'app_main should start device control, then run the runtime directly on main_task'
)

assert.doesNotMatch(
  appMain,
  /xTaskCreate(?:PinnedToCore)?WithCaps\(\s*&RuntimeTask::run/,
  'do not create a separate gea_main task; returning app_main frees the IDF main_task stack into the middle of internal DRAM'
)

assert.match(
  appMain,
  /Runtime returned unexpectedly; parking main task[\s\S]*while \(true\)/,
  'app_main should never return and free its stack into the display heap'
)

assert.match(
  sdkconfigDefaults,
  /^CONFIG_ESP_MAIN_TASK_STACK_SIZE=32768$/m,
  'IDF main_task needs the runtime stack budget because it now owns the runtime loop'
)

// The gea CLI pins CONFIG_ESP_MAIN_TASK_STACK_SIZE=32768 in every build-local
// sdkconfig (cli/test/esp32.test.mjs), so a stale value cannot shrink it back.

// A loop task on an EXTERNAL stack must never be the task that brings the
// framework up: StorageService::init mounts SPIFFS, the localStorage restore
// and the radios go through NVS, and a flash operation from a PSRAM stack
// aborts in esp_task_stack_is_sane_cache_disabled. Hoisting only the app's own
// native_boot hook was not enough -- the SPIFFS mount inside Runtime::run
// asserted on every boot, roughly nine seconds in, which read as a black panel
// with the audio engine apparently running.
assert.match(
  appMain,
  /Runtime::boot\(runtimeOptions\(\)\)/,
  'the whole bring-up must run on main_task before the loop moves to its own task'
)

assert.doesNotMatch(
  appMain,
  /Runtime::runNativeBoot\(\)/,
  'runNativeBoot alone leaves the framework bring-up, and its flash access, on the loop task'
)
