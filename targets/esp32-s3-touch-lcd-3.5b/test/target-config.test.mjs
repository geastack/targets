import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import test from 'node:test'

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..')
const target = path.join(root, 'targets/esp32-s3-touch-lcd-3.5b')
const read = file => readFileSync(path.join(target, file), 'utf8')

test('catalog resolves the 16 MB S3 target and its build policy', () => {
  const entry = JSON.parse(readFileSync(path.join(root, 'targets.json')))['esp32-s3-touch-lcd-3.5b']
  assert.equal(entry.idfTarget, 'esp32s3')
  assert.equal(entry.flashSize, '16MB')
  assert.equal(path.join(root, entry.targetPath), target)
  const policy = JSON.parse(readFileSync(path.join(root, 'build-config.json'))).boards['esp32-s3-touch-lcd-3.5b']
  assert.deepEqual(policy.constraints['display.framebufferStream'], ['independent'])
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

test('panel initialization supplies valid payload lengths and sleep-out delay', () => {
  const init = read('main/include/axs15231b_panel_init.h')
  const commands = [...init.matchAll(/\{(0x[0-9a-f]+), \(uint8_t\[\]\)\{([^}]+)\}, (\d+), (\d+)\}/gi)]
  assert.ok(commands.length > 25)
  for (const [, command, payload, size] of commands) {
    assert.ok(payload.split(',').length >= Number(size), `${command} reads past its payload`)
  }
  assert.ok(commands.some(([, command, , , delay]) => Number(command) === 0x11 && Number(delay) >= 120))
  assert.match(read('main/CMakeLists.txt'), /GEA_BOARD_AXS15231B_INIT_HEADER=/)
})
