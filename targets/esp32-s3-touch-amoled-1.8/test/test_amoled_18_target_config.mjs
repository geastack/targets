import assert from 'node:assert/strict'
import fs from 'node:fs'
import path from 'node:path'

const repoRoot = path.resolve(import.meta.dirname, '../../..')
const targetDir = path.join(repoRoot, 'targets/esp32-s3-touch-amoled-1.8')

function read(relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), 'utf8')
}

assert.equal(fs.existsSync(targetDir), true, 'AMOLED 1.8 target directory should exist')
assert.equal(fs.existsSync(path.join(targetDir, 'main/CMakeLists.txt')), true, 'target CMakeLists.txt should exist')
assert.equal(fs.existsSync(path.join(targetDir, 'main/include/board.h')), true, 'board profile should exist')

const board = read('targets/esp32-s3-touch-amoled-1.8/main/include/board.h')
for (const expected of [
  'GPIO_NUM_15',
  'GPIO_NUM_14',
  'GPIO_NUM_12',
  'GPIO_NUM_11',
  'GPIO_NUM_4',
  'GPIO_NUM_21',
  'GPIO_NUM_9',
  'GPIO_NUM_8',
  'GPIO_NUM_10',
  'GPIO_NUM_46',
  'GPIO_NUM_NC'
]) {
  assert.match(board, new RegExp(expected.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')), `board.h should contain ${expected}`)
}

const cmake = read('targets/esp32-s3-touch-amoled-1.8/main/CMakeLists.txt')
assert.match(cmake, /GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-amoled-1\.8"/)
// Geometry is a board default that a caller may override, wired to the macro
// the framework reads. Asserting a literal `...WIDTH=368` missed that split.
assert.match(cmake, /set\(GEA_BOARD_DISPLAY_WIDTH 368\)/)
assert.match(cmake, /set\(GEA_BOARD_DISPLAY_HEIGHT 448\)/)
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_WIDTH=\$\{GEA_BOARD_DISPLAY_WIDTH\}/)
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_HEIGHT=\$\{GEA_BOARD_DISPLAY_HEIGHT\}/)
assert.doesNotMatch(cmake, /GEA_EMBEDDED_DISPLAY_PANEL_SH8601/)
assert.match(cmake, /sh8601\.cpp/)
assert.match(cmake, /qmi8658\.cpp/)
assert.match(cmake, /es8311\.cpp/)
assert.match(cmake, /ft3168\.cpp/)
assert.doesNotMatch(cmake, /co5300\.cpp/)

const idfComponent = read('targets/esp32-s3-touch-amoled-1.8/main/idf_component.yml')
assert.match(idfComponent, /esp_lcd_sh8601/)

const sdkconfigDefaults = read('targets/esp32-s3-touch-amoled-1.8/sdkconfig.defaults')
assert.match(sdkconfigDefaults, /CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y/)
assert.doesNotMatch(sdkconfigDefaults, /CONFIG_ESPTOOLPY_FLASHSIZE_32MB=y/)

const targets = JSON.parse(read('targets.json'))
assert.deepEqual(targets['esp32-s3-touch-amoled-1.8'], {
  adapter: 'esp32-idf',
  targetPath: 'targets/esp32-s3-touch-amoled-1.8',
  flashSize: '16MB',
  appPlatform: 'esp32',
  idfTarget: 'esp32s3',
  esptoolChip: 'esp32s3'
})

const sh8601Binding = read('targets/esp32/chip_bindings/displays/sh8601.cpp')
assert.match(sh8601Binding, /esp_lcd_new_panel_sh8601/)
assert.match(sh8601Binding, /0x51/)
