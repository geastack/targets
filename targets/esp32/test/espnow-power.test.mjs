import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { mkdirSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
const here = dirname(fileURLToPath(import.meta.url))
const root = resolve(here, '../../../..')
const core = resolve(process.env.GEA_CORE_DIR || resolve(root, 'core/packages/core'))
const host = resolve(process.env.GEA_HOST_DIR || resolve(root, 'core/packages/host'))
const output = resolve(process.env.GEA_TEST_OUTPUT || resolve(process.cwd(), 'build'))
mkdirSync(output, { recursive: true })
const binary = resolve(output, 'espnow-power-test')
const compiled = spawnSync(process.env.CXX || 'c++', ['-std=c++20', '-O1', '-g',
  '-fsanitize=address,undefined', '-Wall', '-Wextra', '-Werror',
  '-I', resolve(here, 'espnow-fixture'), '-I', resolve(core, 'include'),
  '-I', resolve(host, 'include'), resolve(here, 'espnow-power.cpp'), '-o', binary],
  { encoding: 'utf8' })
assert.equal(compiled.status, 0, compiled.stderr)
const run = spawnSync(binary, [], { encoding: 'utf8' })
assert.equal(run.status, 0, run.stderr)
process.stdout.write(run.stdout)
