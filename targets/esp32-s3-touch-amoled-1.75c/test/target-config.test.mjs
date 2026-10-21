import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import path from 'node:path'
import test from 'node:test'

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../..')
const target = path.join(root, 'targets/esp32-s3-touch-amoled-1.75c')
const read = file => readFileSync(path.join(target, file), 'utf8').replace(/\r\n/g, '\n')

test('catalog resolves the 32 MB S3 target and its build policy', () => {
  const entry = JSON.parse(readFileSync(path.join(root, 'targets.json')))['esp32-s3-touch-amoled-1.75c']
  assert.equal(entry.idfTarget, 'esp32s3')
  assert.equal(entry.flashSize, '32MB')
  assert.equal(path.join(root, entry.targetPath), target)
  assert.ok(JSON.parse(readFileSync(path.join(root, 'build-config.json'))).boards['esp32-s3-touch-amoled-1.75c'])
})

test('default partitions fit physical flash without overlap and retain two OTA slots', () => {
  let end = 0
  const slots = []
  for (const line of read('partitions.csv').split('\n').filter(line => line && !line.startsWith('#'))) {
    const [name, type, subtype, offset, size] = line.split(',').map(field => field.trim())
    assert.ok(Number(offset) >= end, `${name} overlaps the previous partition`)
    end = Number(offset) + Number(size)
    assert.ok(end <= 32 * 1024 * 1024, `${name} exceeds physical flash`)
    if (type === 'app') slots.push(subtype)
  }
  assert.deepEqual(slots, ['ota_0', 'ota_1'])
})

test('panel and touch resets and the codec clock are the 1.75C pins, not the 1.75 pins', () => {
  const board = read('main/include/board.h')
  assert.match(board, /\.reset = GPIO_NUM_1,/)
  assert.match(board, /\.reset = GPIO_NUM_2,/)
  assert.match(board, /\.mclk = GPIO_NUM_16,/)
  assert.match(board, /\.es7210Address = 0x40,/)
  assert.match(read('main/CMakeLists.txt'), /GEA_EMBEDDED_CO5300_WAVESHARE_175_INIT=1/)
})
