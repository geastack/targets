import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'
const result = spawnSync('cmake', ['-P', fileURLToPath(new URL('./display-dimensions.cmake', import.meta.url))], { encoding: 'utf8' })
assert.equal(result.status, 0, result.stderr || String(result.error))
console.log('Board defaults and landscape dimension overrides passed')
