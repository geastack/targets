#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const targetRoot = resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.06')
const sdkconfigDefaults = readFileSync(resolve(targetRoot, 'sdkconfig.defaults'), 'utf8')

assert.doesNotMatch(
  sdkconfigDefaults,
  /^CONFIG_GEA_EMBEDDED_PRODUCTION_LOCKDOWN=y$/m,
  'ESP32 dev defaults should not enable production lockdown'
)

assert.match(
  sdkconfigDefaults,
  /^# CONFIG_GEA_EMBEDDED_PRODUCTION_LOCKDOWN is not set$/m,
  'ESP32 dev defaults should explicitly leave production lockdown disabled'
)

const expectedEnabledDefaults = [
  'CONFIG_LOG_DEFAULT_LEVEL_INFO=y',
  'CONFIG_LOG_DEFAULT_LEVEL=3',
  'CONFIG_LOG_MAXIMUM_LEVEL=3',
  'CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE=y',
  'CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=2',
  'CONFIG_BOOTLOADER_LOG_LEVEL_INFO=y',
  'CONFIG_BOOTLOADER_LOG_LEVEL=3'
]

for (const line of expectedEnabledDefaults) {
  assert.match(sdkconfigDefaults, new RegExp(`^${line}$`, 'm'), `sdkconfig.defaults should include ${line}`)
}

const disabledLoggingLines = [
  'CONFIG_LOG_DEFAULT_LEVEL_NONE=y',
  'CONFIG_LOG_MAXIMUM_LEVEL=0',
  'CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_DISABLE=y',
  'CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=0',
  'CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y',
  'CONFIG_BOOTLOADER_LOG_LEVEL=0'
]

for (const line of disabledLoggingLines) {
  assert.doesNotMatch(
    sdkconfigDefaults,
    new RegExp(`^${line}$`, 'm'),
    `sdkconfig.defaults should not include lockdown line ${line}`
  )
}

// Stale per-build sdkconfig logging values are repaired by the gea CLI's
// sdkconfig policy (cli/test/esp32.test.mjs).
