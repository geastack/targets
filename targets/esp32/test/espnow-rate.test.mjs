import assert from 'node:assert/strict'
import { spawnSync } from 'node:child_process'
import { existsSync, mkdirSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
const here = dirname(fileURLToPath(import.meta.url))
const root = resolve(here, '../../../..')
const core = resolve(process.env.GEA_CORE_DIR || resolve(root, 'core/packages/core'))
const host = resolve(process.env.GEA_HOST_DIR || resolve(root, 'core/packages/host'))
const idf = process.env.IDF_PATH
const config = process.env.GEA_IDF_CONFIG_DIR
if (!idf || !config || !existsSync(resolve(config, 'sdkconfig.h'))) {
  process.stdout.write('ESP-NOW rate SDK-contract test skipped: provide IDF_PATH and GEA_IDF_CONFIG_DIR\n')
  process.exit(0)
}
const output = resolve(process.env.GEA_TEST_OUTPUT || resolve(process.cwd(), 'build'))
mkdirSync(output, { recursive: true })
const includes = [config, resolve(core, 'include'), resolve(host, 'include'),
  ...['esp_wifi/include', 'esp_wifi/include/local', 'esp_common/include', 'esp_event/include']
    .map(path => resolve(idf, 'components', path))]
for (const legacy of [0, 1, 2]) {
  const binary = resolve(output, `espnow-rate-test-${legacy}${process.platform === 'win32' ? '.exe' : ''}`)
  const compiled = spawnSync(process.env.CXX || (process.platform === 'win32' ? 'clang++' : 'c++'), ['-std=c++20', '-O1', '-g',
    '-fsanitize=address,undefined', '-Wall', '-Wextra', '-Werror',
    `-DGEA_TEST_LEGACY=${legacy}`, ...includes.flatMap(path => ['-I', path]),
    resolve(here, 'espnow-rate.cpp'), '-o', binary], { encoding: 'utf8' })
  assert.equal(compiled.status, 0, compiled.stderr)
  const run = spawnSync(binary, [], { encoding: 'utf8' })
  assert.equal(run.status, 0, run.stderr)
  process.stdout.write(run.stdout)
}
