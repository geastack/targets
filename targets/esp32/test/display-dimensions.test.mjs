import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { readFileSync, readdirSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { cmakeBinary } from '../../cmake/test/cmake-binary.mjs'
const result = spawnSync(cmakeBinary() ?? 'cmake', ['-P', fileURLToPath(new URL('./display-dimensions.cmake', import.meta.url))], { encoding: 'utf8' })
assert.equal(result.status, 0, result.stderr || String(result.error))
console.log('Board defaults and landscape dimension overrides passed')

const targets = new URL('../../', import.meta.url)
for (const target of readdirSync(targets, { withFileTypes: true })) {
  if (!target.isDirectory()) continue
  const buildFiles = [
    new URL(`${target.name}/CMakeLists.txt`, targets),
    new URL(`${target.name}/main/CMakeLists.txt`, targets),
    new URL(`${target.name}/gea_spi_panel_component.cmake`, targets),
  ]
  for (const file of buildFiles) {
    let source
    try {
      source = readFileSync(file, 'utf8')
    } catch (error) {
      if (error.code !== 'ENOENT') throw error
      continue
    }
    if (!source.includes('--cpp-board') || !source.includes('build-gea-vite-geatsc.mjs')) continue
    assert.match(source, /--font-viewport-width\s+\$\{\w+\}/, `${file.pathname} should forward the render width`)
    assert.match(source, /--font-viewport-height\s+\$\{\w+\}/, `${file.pathname} should forward the render height`)
  }
}
