#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

const targetMetadata = JSON.parse(source('targets.json'))
const boardHeader = source('targets/rp2350-waveshare-touch-amoled-2.41/main/include/board.h')
const cmake = source('targets/rp2350-waveshare-touch-amoled-2.41/CMakeLists.txt')

assert.deepEqual(
  targetMetadata['rp2350-waveshare-touch-amoled-2.41'],
  {
    adapter: 'rp2350-pico',
    targetPath: 'targets/rp2350-waveshare-touch-amoled-2.41',
    flashSize: '16MB',
    appPlatform: 'rp2350'
  },
  'RP2350 target should advertise its Pico SDK adapter metadata'
)

const expectedPins = [
  '.sda = 6',
  '.scl = 7',
  '.cs = 9',
  '.sclk = 10',
  '.data0 = 11',
  '.data1 = 12',
  '.data2 = 13',
  '.data3 = 14',
  '.reset = 15',
  '.interrupt = 4',
  '.powerKey = 24',
  '.powerHold = 25',
  '.batteryAdc = 40',
  '.batteryAdcChannel = 0'
]
for (const pin of expectedPins) {
  assert.match(boardHeader, new RegExp(pin.replace('.', '\\.')), `board.h should include ${pin}`)
}

assert.match(boardHeader, /rm690b0::kWidth/, 'board should use the RM690B0 display core')
assert.match(boardHeader, /ft6336::kI2cAddress/, 'board should use the FT6336 touch core')
assert.match(boardHeader, /qmi8658::kI2cAddress/, 'board should use the QMI8658 IMU core')
assert.match(boardHeader, /pcf85063::kI2cAddress/, 'board should use the PCF85063 RTC core')
assert.match(boardHeader, /16U \* 1024U \* 1024U/, 'board should declare 16 MB flash')
assert.match(boardHeader, /2U \* 1024U \* 1024U/, 'board should declare 2 MB PSRAM')

assert.match(cmake, /PICO_BOARD waveshare_rp2350_touch_amoled_2_41/, 'target should select the custom Pico board')
assert.match(cmake, /rm690b0\/rm690b0\.cpp/, 'target should link the RM690B0 chip core')
assert.match(cmake, /ft6336\/ft6336\.cpp/, 'target should link the FT6336 chip core')
assert.match(cmake, /pcf85063\/pcf85063\.cpp/, 'target should link the PCF85063 chip core')
assert.match(cmake, /eta6098\/eta6098\.cpp/, 'target should link the ETA6098 chip core')

