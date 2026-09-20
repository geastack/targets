import assert from 'node:assert/strict'
import { after, test } from 'node:test'
import {
  appendFileSync,
  chmodSync,
  existsSync,
  mkdtempSync,
  mkdirSync,
  readFileSync,
  rmSync,
  statSync,
  utimesSync,
  writeFileSync,
} from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

import { buildGeneratedArchive, defaultJobCount, parseDepfile } from '../geatsc-generated-archive-build.mjs'

const temporaryRoots = []
after(() => {
  for (const root of temporaryRoots) rmSync(root, { recursive: true, force: true })
})

function temporaryRoot() {
  const root = mkdtempSync(path.join(os.tmpdir(), 'geatsc-archive-test-'))
  temporaryRoots.push(root)
  return root
}

function executable(filePath, source) {
  writeFileSync(filePath, source)
  chmodSync(filePath, 0o755)
}

function fakeToolchain(root) {
  const compileLog = path.join(root, 'compile.log')
  const compiler = path.join(root, 'fake-compiler.mjs')
  const compilerLauncher = path.join(root, 'fake-compiler-launcher.mjs')
  const ar = path.join(root, 'fake-ar.mjs')
  const ranlib = path.join(root, 'fake-ranlib.mjs')

  executable(
    compiler,
    `#!/usr/bin/env node
import { appendFileSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs'
import path from 'node:path'
const args = process.argv.slice(2)
const valueAfter = (flag) => args[args.indexOf(flag) + 1]
const source = valueAfter('-c')
const object = valueAfter('-o')
const depfile = valueAfter('-MF')
const name = path.basename(source)
appendFileSync(process.env.FAKE_COMPILE_LOG, JSON.stringify({ type: 'start', name, args, at: Date.now() }) + '\\n')
await new Promise((resolve) => setTimeout(resolve, Number(process.env.FAKE_COMPILE_DELAY_MS || 100)))
const includes = [...readFileSync(source, 'utf8').matchAll(/#include\\s+"([^"]+)"/g)]
  .map((match) => path.resolve(path.dirname(source), match[1]))
const escape = (value) => value.replace(/\\\\/g, '\\\\\\\\').replace(/ /g, '\\\\ ')
mkdirSync(path.dirname(object), { recursive: true })
writeFileSync(object, JSON.stringify({ source: name }))
writeFileSync(depfile, escape(object) + ': ' + [source, ...includes].map(escape).join(' ') + '\\n')
appendFileSync(process.env.FAKE_COMPILE_LOG, JSON.stringify({ type: 'end', name, at: Date.now() }) + '\\n')
`,
  )
  executable(
    compilerLauncher,
    `#!/usr/bin/env node
import { appendFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
const [compiler, ...args] = process.argv.slice(2)
appendFileSync(process.env.FAKE_COMPILE_LOG, JSON.stringify({ type: 'launcher', compiler, at: Date.now() }) + '\\n')
const result = spawnSync(compiler, args, { stdio: 'inherit' })
process.exit(result.status ?? 1)
`,
  )
  executable(
    ar,
    `#!/usr/bin/env node
import { readFileSync, writeFileSync } from 'node:fs'
const [, archive, ...objects] = process.argv.slice(2)
writeFileSync(archive, JSON.stringify(objects.map((object) => JSON.parse(readFileSync(object, 'utf8')).source)))
`,
  )
  executable(
    ranlib,
    `#!/usr/bin/env node
import { appendFileSync } from 'node:fs'
appendFileSync(process.env.FAKE_COMPILE_LOG, JSON.stringify({ type: 'ranlib', at: Date.now() }) + '\\n')
`,
  )
  return { compiler, compilerLauncher, ar, ranlib, compileLog }
}

function compileEvents(logPath) {
  return readFileSync(logPath, 'utf8')
    .trim()
    .split('\n')
    .filter(Boolean)
    .map((line) => JSON.parse(line))
    .filter((event) => event.type === 'start' || event.type === 'end')
}

function maximumConcurrency(events) {
  let active = 0
  let maximum = 0
  for (const event of [...events].sort((left, right) => left.at - right.at || (left.type === 'start' ? -1 : 1))) {
    active += event.type === 'start' ? 1 : -1
    maximum = Math.max(maximum, active)
  }
  return maximum
}

test('parseDepfile handles continuations and escaped spaces', () => {
  const dependencies = parseDepfile('out.o: source.cpp path\\ with\\ spaces/header.h \\\n next.h\nignored-phony.h:\n', '/work')
  assert.deepEqual(dependencies, ['/work/source.cpp', '/work/path with spaces/header.h', '/work/next.h'])
})

test('archive jobs default to a memory-safe ceiling with an explicit override', () => {
  assert.ok(defaultJobCount({}) <= 4)
  assert.equal(defaultJobCount({ GEA_GEATSC_JOBS: '3', CMAKE_BUILD_PARALLEL_LEVEL: '7' }), 3)
  assert.ok(defaultJobCount({ CMAKE_BUILD_PARALLEL_LEVEL: '7' }) <= 4)
})

test('generated archive compilation is parallel, incremental, dependency-aware, and ordered', async () => {
  const root = temporaryRoot()
  const sourceDir = path.join(root, 'sources')
  const objectDir = path.join(root, 'objects')
  mkdirSync(sourceDir)
  const header = path.join(sourceDir, 'private header.h')
  writeFileSync(header, '#define PRIVATE_VALUE 1\n')
  const sourceNames = ['gamma.cpp', 'alpha.cpp', 'delta.cpp', 'beta.cpp']
  for (const name of sourceNames) {
    writeFileSync(path.join(sourceDir, name), name === 'beta.cpp' ? '#include "private header.h"\nint beta = PRIVATE_VALUE;\n' : `int ${name[0]} = 1;\n`)
  }

  const sourceList = path.join(root, 'geatsc-sources.txt')
  const flagsFile = path.join(root, 'flags.rsp')
  const archive = path.join(root, 'libgenerated.a')
  const depfile = path.join(root, 'archive.d')
  writeFileSync(sourceList, `${sourceNames.map((name) => path.join(sourceDir, name)).join('\n')}\n`)
  writeFileSync(flagsFile, '-O2\n')
  const tools = fakeToolchain(root)
  process.env.FAKE_COMPILE_LOG = tools.compileLog
  process.env.FAKE_COMPILE_DELAY_MS = '120'
  const options = {
    sourceList,
    archive,
    objectDir,
    depfile,
    flagsFile,
    compiler: tools.compiler,
    compilerLauncher: tools.compilerLauncher,
    ar: tools.ar,
    ranlib: tools.ranlib,
    jobs: 2,
  }

  const first = await buildGeneratedArchive(options)
  assert.equal(first.compiled, 4)
  assert.equal(first.reused, 0)
  assert.equal(first.archived, true)
  const firstStart = readFileSync(tools.compileLog, 'utf8')
    .trim()
    .split('\n')
    .map((line) => JSON.parse(line))
    .find((event) => event.type === 'start')
  assert.ok(firstStart.args.includes('-g0'), 'generated objects should omit duplicated DWARF by default')
  assert.equal(
    readFileSync(tools.compileLog, 'utf8').split('\n').filter((line) => line.includes('"type":"launcher"')).length,
    4,
    'each stale object should run through the configured compiler launcher',
  )
  assert.ok(maximumConcurrency(compileEvents(tools.compileLog)) >= 2, 'at least two compiler processes should overlap')
  assert.deepEqual(JSON.parse(readFileSync(archive, 'utf8')), sourceNames, 'archive members should follow source-list order')
  assert.match(readFileSync(depfile, 'utf8'), /private\\ header\.h/, 'aggregate depfile should retain header dependencies')

  const firstArchiveTime = statSync(archive).mtimeMs
  const eventCountAfterFirst = compileEvents(tools.compileLog).length
  const second = await buildGeneratedArchive(options)
  assert.equal(second.compiled, 0)
  assert.equal(second.reused, 4)
  assert.equal(second.archived, false)
  assert.equal(statSync(archive).mtimeMs, firstArchiveTime, 'no-op build should preserve archive mtime')
  assert.equal(compileEvents(tools.compileLog).length, eventCountAfterFirst, 'no-op build should not spawn the compiler')

  const betaRecord = first.records.find((record) => path.basename(record.sourcePath) === 'beta.cpp')
  assert.ok(betaRecord)
  const oldTime = new Date(statSync(betaRecord.objectPath).mtimeMs - 2000)
  utimesSync(betaRecord.objectPath, oldTime, oldTime)
  const sourceChanged = await buildGeneratedArchive(options)
  assert.equal(sourceChanged.compiled, 1)
  assert.equal(sourceChanged.reused, 3)

  await new Promise((resolve) => setTimeout(resolve, 25))
  appendFileSync(header, '#define PRIVATE_VALUE_2 2\n')
  const headerChanged = await buildGeneratedArchive(options)
  assert.equal(headerChanged.compiled, 1, 'only the object whose depfile names the header should rebuild')
  assert.equal(headerChanged.reused, 3)

  writeFileSync(sourceList, `${sourceNames.slice(0, 3).map((name) => path.join(sourceDir, name)).join('\n')}\n`)
  const sourceRemoved = await buildGeneratedArchive(options)
  assert.equal(sourceRemoved.compiled, 0)
  assert.equal(sourceRemoved.reused, 3)
  assert.equal(sourceRemoved.archived, true, 'removing a source should recreate the archive without recompiling survivors')
  assert.deepEqual(JSON.parse(readFileSync(archive, 'utf8')), sourceNames.slice(0, 3))

  writeFileSync(flagsFile, '-O3\n')
  const flagsChanged = await buildGeneratedArchive(options)
  assert.equal(flagsChanged.compiled, 3, 'compiler flag signature changes should invalidate every remaining object')

  const debugInfoEnabled = await buildGeneratedArchive({ ...options, debugInfo: true })
  assert.equal(debugInfoEnabled.compiled, 3, 'opting into generated debug info should invalidate generated objects')
  const lastStart = readFileSync(tools.compileLog, 'utf8')
    .trim()
    .split('\n')
    .map((line) => JSON.parse(line))
    .filter((event) => event.type === 'start')
    .at(-1)
  assert.ok(!lastStart.args.includes('-g0'), 'debug-info opt-in should preserve the toolchain debug flags')
})

test('CMake archive helper delegates to the incremental runner', () => {
  const repoRoot = fileURLToPath(new URL('../../..', import.meta.url))
  const wrapperPath = path.join(repoRoot, 'targets/cmake/geatsc_generated_archive_build.cmake')
  const wrapper = readFileSync(wrapperPath, 'utf8')
  const integration = readFileSync(path.join(repoRoot, 'targets/cmake/geatsc_generated_archive.cmake'), 'utf8')
  assert.match(wrapper, /geatsc-generated-archive-build\.mjs/)
  assert.match(integration, /geatsc-generated-archive-build\.mjs/)

  const root = temporaryRoot()
  const source = path.join(root, 'cmake-wrapper.cpp')
  const sourceList = path.join(root, 'geatsc-sources.txt')
  const flagsFile = path.join(root, 'flags.rsp')
  const archive = path.join(root, 'libgenerated.a')
  const objectDir = path.join(root, 'objects')
  const depfile = path.join(root, 'archive.d')
  writeFileSync(source, 'int cmake_wrapper = 1;\n')
  writeFileSync(sourceList, `${source}\n`)
  writeFileSync(flagsFile, '-O2\n')
  const fake = fakeToolchain(root)
  const result = spawnSync(
    'cmake',
    [
      `-DGEA_GEATSC_SOURCE_LIST=${sourceList}`,
      `-DGEA_GEATSC_ARCHIVE=${archive}`,
      `-DGEA_GEATSC_OBJECT_DIR=${objectDir}`,
      `-DGEA_GEATSC_DEPFILE=${depfile}`,
      `-DGEA_GEATSC_FLAGS_FILE=${flagsFile}`,
      `-DGEA_GEATSC_CXX_COMPILER=${fake.compiler}`,
      `-DGEA_GEATSC_COMPILER_LAUNCHER=${fake.compilerLauncher}`,
      `-DGEA_GEATSC_AR=${fake.ar}`,
      '-DGEA_GEATSC_RANLIB=',
      '-P',
      wrapperPath,
    ],
    {
      encoding: 'utf8',
      env: { ...process.env, FAKE_COMPILE_LOG: fake.compileLog, FAKE_COMPILE_DELAY_MS: '10', GEA_GEATSC_JOBS: '1' },
    },
  )
  assert.equal(result.status, 0, `${result.stdout}\n${result.stderr}`)
  assert.deepEqual(JSON.parse(readFileSync(archive, 'utf8')), ['cmake-wrapper.cpp'])
})

test('real host toolchain produces an incremental archive', async (context) => {
  const compiler = '/usr/bin/c++'
  const ar = '/usr/bin/ar'
  const ranlib = '/usr/bin/ranlib'
  if (![compiler, ar, ranlib].every(existsSync)) {
    context.skip('host C++ archive toolchain is not available in /usr/bin')
    return
  }

  const root = temporaryRoot()
  const sourceNames = ['first.cpp', 'second.cpp']
  for (const [index, name] of sourceNames.entries()) writeFileSync(path.join(root, name), `int geatsc_smoke_${index} = ${index};\n`)
  const sourceList = path.join(root, 'geatsc-sources.txt')
  const flagsFile = path.join(root, 'flags.rsp')
  const archive = path.join(root, 'libgenerated.a')
  writeFileSync(sourceList, `${sourceNames.map((name) => path.join(root, name)).join('\n')}\n`)
  writeFileSync(flagsFile, '-O2\n')
  const options = {
    sourceList,
    archive,
    objectDir: path.join(root, 'objects'),
    depfile: path.join(root, 'archive.d'),
    flagsFile,
    compiler,
    ar,
    ranlib,
    jobs: 2,
  }

  const first = await buildGeneratedArchive(options)
  const firstArchiveTime = statSync(archive).mtimeMs
  assert.equal(first.compiled, 2)
  const members = spawnSync(ar, ['-t', archive], { encoding: 'utf8' })
  assert.equal(members.status, 0, members.stderr)
  assert.deepEqual(
    members.stdout
      .trim()
      .split(/\r?\n/)
      .filter((member) => !member.startsWith('__.SYMDEF')),
    first.records.map((record) => path.basename(record.objectPath)),
  )

  const second = await buildGeneratedArchive(options)
  assert.equal(second.compiled, 0)
  assert.equal(second.archived, false)
  assert.equal(statSync(archive).mtimeMs, firstArchiveTime)
})

test('runtime PCH is built once, reused across emissions, and rebuilds on content change', async () => {
  const root = temporaryRoot()
  const sourceDir = path.join(root, 'sources')
  const objectDir = path.join(root, 'objects')
  mkdirSync(sourceDir)
  const pchHeader = path.join(sourceDir, 'runtime_pch.h')
  const runtimePart = path.join(sourceDir, 'runtime.cpp')
  writeFileSync(runtimePart, 'inline int gea_runtime = 1;\n')
  writeFileSync(pchHeader, '#include "runtime.cpp"\n')
  const sourceNames = ['alpha.cpp', 'beta.cpp']
  for (const name of sourceNames) {
    writeFileSync(path.join(sourceDir, name), `#include "runtime_pch.h"\nint ${name[0]} = 1;\n`)
  }

  const sourceList = path.join(sourceDir, 'geatsc-sources.txt')
  writeFileSync(sourceList, `${sourceNames.map((name) => path.join(sourceDir, name)).join('\n')}\n`)
  const flagsFile = path.join(root, 'flags.rsp')
  writeFileSync(flagsFile, '-O2\n')
  const tools = fakeToolchain(root)
  process.env.FAKE_COMPILE_LOG = tools.compileLog
  process.env.FAKE_COMPILE_DELAY_MS = '10'
  const options = {
    sourceList,
    archive: path.join(root, 'libgenerated.a'),
    objectDir,
    depfile: path.join(root, 'archive.d'),
    flagsFile,
    compiler: tools.compiler,
    ar: tools.ar,
    ranlib: tools.ranlib,
    jobs: 2,
  }

  process.env.GEA_GEATSC_PCH = '1'
  const first = await buildGeneratedArchive(options)
  assert.ok(first.pch, 'pch should be prepared when runtime_pch.h sits next to the source list')
  assert.equal(first.pch.rebuilt, true)
  assert.ok(existsSync(`${pchHeader}.gch`), 'the .gch must sit next to the header for GCC auto-pickup')
  const pchCompile = readFileSync(tools.compileLog, 'utf8')
    .trim()
    .split('\n')
    .map((line) => JSON.parse(line))
    .find((event) => event.type === 'start' && event.name === 'runtime_pch.h')
  assert.ok(pchCompile, 'the pch header itself should be compiled')
  assert.ok(pchCompile.args.includes('c++-header'), 'pch compile should use -x c++-header')
  const moduleCompile = compileEvents(tools.compileLog).find((event) => event.name === 'alpha.cpp')
  assert.ok(moduleCompile.args.includes('-Winvalid-pch'), 'TU compiles should carry the PCH consumer flags')
  assert.ok(moduleCompile.args.includes('-fpch-deps'), 'TU depfiles must see through the PCH')

  // Simulate a geatsc re-emission: identical bytes, fresh mtimes everywhere.
  const bumped = new Date(Date.now() + 1500)
  utimesSync(pchHeader, bumped, bumped)
  utimesSync(runtimePart, bumped, bumped)
  const gchTimeBefore = statSync(`${pchHeader}.gch`).mtimeMs
  const second = await buildGeneratedArchive(options)
  assert.equal(second.pch.rebuilt, false, 'byte-identical re-emission must reuse the cached .gch')
  assert.equal(statSync(`${pchHeader}.gch`).mtimeMs, gchTimeBefore)

  // A real content change to a folded-in runtime part rebuilds the PCH and
  // (via the .gch identity in the compile signature) every object.
  await new Promise((resolve) => setTimeout(resolve, 25))
  appendFileSync(runtimePart, 'inline int gea_runtime_2 = 2;\n')
  const third = await buildGeneratedArchive(options)
  assert.equal(third.pch.rebuilt, true, 'runtime content change must rebuild the .gch')
  assert.equal(third.compiled, 2, 'a new .gch must invalidate every generated object')

  // Without the explicit opt-in the PCH machinery must stay off entirely.
  delete process.env.GEA_GEATSC_PCH
  const optedOut = await buildGeneratedArchive(options)
  assert.equal(optedOut.pch, null)
})
