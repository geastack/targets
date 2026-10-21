import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import test from 'node:test'

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..')
const target = path.join(root, 'targets/esp32-s3-touch-lcd-3.49')
const read = file => readFileSync(path.join(target, file), 'utf8').replace(/\r\n/g, '\n')

test('catalog resolves the 16 MB S3 target and its build policy', () => {
  const entry = JSON.parse(readFileSync(path.join(root, 'targets.json')))['esp32-s3-touch-lcd-3.49']
  assert.equal(entry.idfTarget, 'esp32s3')
  assert.equal(entry.flashSize, '16MB')
  assert.equal(path.join(root, entry.targetPath), target)
  const policy = JSON.parse(readFileSync(path.join(root, 'build-config.json'))).boards['esp32-s3-touch-lcd-3.49']
  assert.deepEqual(policy.constraints['display.framebufferStream'], ['independent'])
  assert.deepEqual(policy.constraints['renderer.fuseReplayFlush'], [false])
})

test('default partitions fit physical flash without overlap and retain two OTA slots', () => {
  let end = 0
  const slots = []
  for (const line of read('partitions.csv').split('\n').filter(line => line && !line.startsWith('#'))) {
    const [name, type, subtype, offset, size] = line.split(',')
    assert.ok(Number(offset) >= end, `${name} overlaps the previous partition`)
    end = Number(offset) + Number(size)
    assert.ok(end <= 16 * 1024 * 1024, `${name} exceeds physical flash`)
    if (type === 'app') slots.push(subtype)
  }
  assert.deepEqual(slots, ['ota_0', 'ota_1'])
})

test('the 172x640 glass is rotated in software and its backlight is active low', () => {
  const cmake = read('main/CMakeLists.txt')
  assert.match(cmake, /GEA_BOARD_NATIVE_DISPLAY_WIDTH 172\)/)
  assert.match(cmake, /GEA_BOARD_NATIVE_DISPLAY_HEIGHT 640\)/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY=1/)
  assert.match(cmake, /GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO=8/)
  assert.match(cmake, /GEA_EMBEDDED_AXS15231B_BACKLIGHT_ACTIVE_LOW=1/)
  assert.match(cmake, /GEA_BOARD_AXS15231B_INIT_HEADER=/)
})

test('touch polls its own I2C pair, apart from the sensor and codec bus', () => {
  const board = read('main/include/board.h')
  assert.match(board, /\.sda = GPIO_NUM_47,\n\t\t\.scl = GPIO_NUM_48/)
  assert.match(board, /\.bus = \{\.sda = GPIO_NUM_17, \.scl = GPIO_NUM_18\}/)
  const touch = read('main/touch.cpp')
  assert.match(touch, /i2c_new_master_bus/)
  assert.doesNotMatch(touch, /i2c::Bus::primary/)
})
