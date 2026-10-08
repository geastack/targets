import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const root = new URL('../../../', import.meta.url)
const read = file => JSON.parse(readFileSync(new URL(file, root), 'utf8'))
const id = 'esp32-s3-touch-amoled-2.06-sh8601'
const catalog = read('targets.json')
const base = catalog['esp32-s3-touch-amoled-2.06']
const variant = catalog[id]
assert.equal(variant.variantOf, 'esp32-s3-touch-amoled-2.06')
assert.equal(variant.targetPath, base.targetPath)
assert.equal(variant.adapter, base.adapter)
assert.equal(variant.flashSize, base.flashSize)
assert.equal(base.definition, undefined, 'the existing board must keep its native profile')
const definition = read(variant.definition)
assert.equal(definition.id, id)
const display = definition.chips.display
assert.equal(display.driver, 'sh8601')
assert.equal(display.panel.minChunkRows, 2, 'Wi-Fi staging reduction must preserve two-row windows')
assert.equal(display.panel.transferMode, 'bitmap')
const decode = command => {
  const data = display.panel.initCommands.find(c => c.command === command).data
  return [(data[0] << 8) | data[1], (data[2] << 8) | data[3]]
}
assert.deepEqual(decode(0x2a), [display.panel.xGap, display.panel.xGap + display.width - 1])
assert.deepEqual(decode(0x2b), [display.panel.yGap, display.panel.yGap + display.height - 1])
assert.equal(display.height % display.panel.minChunkRows, 0)
const settings = read('build-config.json').boards[id]
assert.equal(settings.defaults.display.framebufferStream, 'independent')
assert.equal(settings.defaults.display.presentStream, 'independent')
console.log('AMOLED SH8601 variant catalog, windows and transfer defaults passed')
