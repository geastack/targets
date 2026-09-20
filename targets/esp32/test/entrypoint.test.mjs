import assert from 'node:assert/strict'
import { existsSync, readFileSync } from 'node:fs'
import { join } from 'node:path'

const repoRoot = new URL('../../..', import.meta.url).pathname

// Board workflows are `gea build|flash|run|monitor|ota --board <alias>` in
// @geastack/cli. This package ships sources, metadata and CMake only.
assert.equal(existsSync(join(repoRoot, 'scripts/board')), false, 'the bash board wrapper was replaced by the gea CLI')
assert.equal(existsSync(join(repoRoot, 'scripts/boards')), false, 'board resolution lives in @geastack/cli')
assert.equal(existsSync(join(repoRoot, 'targets.json')), true, 'target metadata is the package-root targets.json')

const targets = JSON.parse(readFileSync(join(repoRoot, 'targets.json'), 'utf8'))
for (const [name, target] of Object.entries(targets)) {
  assert.ok(target.adapter, `${name} should name its adapter`)
  assert.ok(target.targetPath, `${name} should name its project directory`)
  assert.ok(target.appPlatform, `${name} should name its app platform`)
}

const hostSources = readFileSync(join(repoRoot, 'targets/esp32/host_sources.cmake'), 'utf8')
const rp2350Targets = [
  'targets/rp2350-tufty-2350/CMakeLists.txt',
  'targets/rp2350-waveshare-touch-amoled-2.41/CMakeLists.txt'
].map((file) => readFileSync(join(repoRoot, file), 'utf8')).join('\n')
assert.match(hostSources, /"\$\{GEA_HOST\}\/host\/fetch\.cpp"/, 'ESP32 host sources should come from @geastack/host')
assert.doesNotMatch(hostSources, /lib\/gea-embedded/, 'ESP32 host sources should not assume a source checkout')
assert.match(rp2350Targets, /ENV\{GEA_CHIPS_DIR\}/, 'RP2350 targets should consume npm package paths from the CLI')
assert.doesNotMatch(rp2350Targets, /GEA_STACK_ROOT|core\/packages|core\/chips/, 'RP2350 targets should not assume a source checkout')
assert.match(rp2350Targets, /GEA_EMBEDDED_APP_META/, 'RP2350 targets should take app metadata from the CLI, not spawn node')
assert.doesNotMatch(rp2350Targets, /gea-embedded\.mjs/, 'RP2350 targets should not spawn the retired gea-embedded tool')

// The framework manifest (`@geastack/core/gea_sources.sh`) must be handed the
// REAL package paths, the same spelling `GEA_FW_APP_SENSITIVE_SOURCES` is
// written in. A workspace that symlinks `@geastack/host` into node_modules
// otherwise gets `node_modules/@geastack/host/host/fetch.cpp` from the
// manifest and `core/packages/host/host/fetch.cpp` from `${GEA_HOST}`;
// `list(REMOVE_ITEM)` compares strings, misses, and fetch.cpp/websocket.cpp
// compile a second time in the framework component without the network
// requirements only main carries (`esp_http_client.h: No such file`).
const frameworkCmake = readFileSync(join(repoRoot, 'targets/esp32/gea_framework.cmake'), 'utf8')
// The manifest is gea_sources.mjs, and CMake runs it directly: sourcing it as
// a shell script needs `bash -c "source gea_sources.sh"`, which is not
// available on every build host. gea_sources.sh is a thin wrapper over it.
const manifestExport = frameworkCmake.match(/set\(_GEA_FW_ENV[\s\S]*?\)\n/)?.[0] ?? ''
assert.ok(manifestExport, 'gea_framework.cmake should pass the package roots to the manifest')
assert.match(frameworkCmake, /gea_sources\.mjs/, 'the framework manifest is gea_sources.mjs')
assert.doesNotMatch(manifestExport, /\$ENV\{GEA_(HOST|ENGINE|ELEMENTS|GEAOS_PACKAGE)_DIR\}/, 'the manifest must see real package paths, not the env spelling')
for (const name of ['GEA_HOST', 'GEA_ENGINE', 'GEA_ELEMENTS', 'GEA_GEAOS_PACKAGE']) {
  assert.match(manifestExport, new RegExp(`\\$\\{${name}\\}`), `the manifest should receive \${${name}} (a REALPATH)`)
}
