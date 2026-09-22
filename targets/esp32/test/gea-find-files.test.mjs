import assert from 'node:assert/strict'
import { execFileSync, spawnSync } from 'node:child_process'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { findFiles } from '../gea-find-files.mjs'

const here = path.dirname(fileURLToPath(import.meta.url))
const script = path.join(here, '..', 'gea-find-files.mjs')
const root = fs.realpathSync(fs.mkdtempSync(path.join(os.tmpdir(), 'gea-find-files-')))
const write = (rel, content = '') => {
  const full = path.join(root, rel)
  fs.mkdirSync(path.dirname(full), { recursive: true })
  fs.writeFileSync(full, content)
}
write('app/src/App.tsx')
write('app/src/style.css')
write('app/package.json')
write('app/README.md')
write('app/logo.PNG')
write('app/node_modules/dep/index.js')
write('app/dist/bundle.js')
write('vendor/lib.ts')
write('linked/target.ts')
fs.symlinkSync(path.join(root, 'linked'), path.join(root, 'app', 'link'), 'dir')
fs.symlinkSync(path.join(root, 'app'), path.join(root, 'linked', 'loop'), 'dir')

const rel = (files) => files.map((f) => path.relative(root, f).split(path.sep).join('/')).sort()

try {
  const sources = findFiles({
    dir: path.join(root, 'app'),
    prune: ['node_modules', 'dist'],
    ext: ['.tsx', '.ts', '.css'],
    name: ['package.json'],
  })
  assert.deepEqual(rel(sources), ['app/package.json', 'app/src/App.tsx', 'app/src/style.css'],
    'prunes node_modules and dist, matches by extension and name, ignores symlinks by default')

  const followed = findFiles({
    dir: path.join(root, 'app'),
    followSymlinks: true,
    prune: ['node_modules', 'dist'],
    ext: ['.ts', '.tsx'],
  })
  assert.deepEqual(rel(followed), ['app/link/target.ts', 'app/src/App.tsx'],
    'follows symlinked directories once and survives the loop back into app')

  const assets = findFiles({ dir: root, ext: ['.png'] })
  assert.deepEqual(rel(assets), ['app/logo.PNG'], 'extension match is case-insensitive')

  assert.deepEqual(findFiles({ dir: path.join(root, 'missing') }), [], 'a missing directory yields nothing')

  const cli = execFileSync(process.execPath, [script, path.join(root, 'app'), '--prune', 'node_modules,dist', '--ext', '.tsx', '--name', 'package.json'], { encoding: 'utf8' })
  assert.deepEqual(rel(cli.trim().split('\n')), ['app/package.json', 'app/src/App.tsx'], 'CLI prints one forward-slash path per line')
  assert.equal(execFileSync(process.execPath, [script, path.join(root, 'missing')], { encoding: 'utf8' }), '', 'CLI prints nothing for a missing directory')
  assert.equal(spawnSync(process.execPath, [script, root, '--bogus']).status, 2, 'an unknown option fails loudly')

  const cmake = spawnSync('cmake', ['--version'])
  if (cmake.status === 0) {
    const targetsRoot = path.resolve(here, '..', '..', '..')
    write('probe.cmake', `
include("${targetsRoot.split(path.sep).join('/')}/targets/esp32/gea_find_sources.cmake")
_gea_find_sources(_srcs "${root.split(path.sep).join('/')}/app")
message(STATUS "SOURCES=\${_srcs}")
_gea_collect_app_assets(_assets "${root.split(path.sep).join('/')}/app")
message(STATUS "ASSETS=\${_assets}")
_gea_find_sources(_vendor "${root.split(path.sep).join('/')}" PRUNE_VENDOR)
message(STATUS "VENDOR=\${_vendor}")
`)
    const out = execFileSync('cmake', ['-P', path.join(root, 'probe.cmake')], { encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] })
    const list = (key) => (out.match(new RegExp(`${key}=(.*)`)) || [, ''])[1].split(';').filter(Boolean)
    assert.deepEqual(rel(list('SOURCES')), ['app/link/target.ts', 'app/package.json', 'app/src/App.tsx', 'app/src/style.css'],
      'CMake wrapper follows symlinks and prunes build output')
    assert.deepEqual(rel(list('ASSETS')), ['app/logo.PNG'], 'CMake asset wrapper finds images')
    assert.ok(!list('VENDOR').some((f) => f.includes('/vendor/')), 'PRUNE_VENDOR skips vendor')
  } else {
    console.log('cmake not on PATH; skipping the CMake wrapper probe')
  }
  console.log('gea-find-files: ok')
} finally {
  fs.rmSync(root, { recursive: true, force: true })
}
