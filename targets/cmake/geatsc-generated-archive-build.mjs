#!/usr/bin/env node

import { createHash } from 'node:crypto'
import { existsSync, mkdirSync, readFileSync, realpathSync, renameSync, rmSync, statSync, writeFileSync } from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import { spawn } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const OBJECT_SIGNATURE_VERSION = 'gea-geatsc-object-v2'
const ARCHIVE_SIGNATURE_VERSION = 'gea-geatsc-archive-v1'
const PCH_SIGNATURE_VERSION = 'gea-geatsc-pch-v1'
const NO_GENERATED_DEBUG_INFO_FLAGS = ['-g0']
// The geatsc runtime umbrella (runtime_pch.h -> runtime.cpp -> prelude +
// value_* parts, ~99k preprocessed lines) is included by EVERY generated TU.
// Precompiling it once per flag-set removes that parse from each of the ~60
// module compiles (measured 2.0s -> 0.06s per small TU when the .gch loads).
// ⛔ OPT-IN ONLY (GEA_GEATSC_PCH=1): xtensa gcc 15.2 on macOS hosts
// NONDETERMINISTICALLY segfaults ("internal compiler error: Segmentation
// fault: 11") while loading the ~174MB .gch this umbrella produces — the same
// .gch loads fine in one process and kills the next. A randomly failing build
// is worse than a slow one, so the default stays textual inclusion until the
// toolchain's PCH relocation is trustworthy (or the runtime is decl/def-split
// small enough for a reliable PCH). Both compiles below share the exact
// @flags-file + generated-flags prefix, as GCC requires for .gch validity.
const PCH_HEADER_NAME = 'runtime_pch.h'
// -Winvalid-pch: GCC silently falls back to the textual include when it
//   rejects a .gch; surface that as a warning so a mismatch is visible.
// -fpch-deps: without it, -MMD omits every header folded INTO the PCH, so the
//   per-object staleness check would miss edits to the runtime parts.
// -fpch-preprocess: keeps the PCH marker in -E output; required for ccache's
//   preprocessor mode to cache PCH-using compiles instead of bailing.
const PCH_CONSUMER_FLAGS = ['-Winvalid-pch', '-fpch-deps', '-fpch-preprocess']

// Warning tolerances applied ONLY to geatsc-generated translation units (never
// to hand-written framework code). gcc 15's -Werror=infinite-recursion is a
// false positive on the generated `gea_cpp_key` callable-adapter: the recursive
// call is on std::function<Sig>, whose instantiation takes the non-recursive
// branch, so it always terminates — the analyzer just can't prove it across the
// type change. Downgrade it for generated code so a compiler heuristic can't
// break app builds.
// -Wno-error=unused-label: the typed-IR backend lowers each function to a basic-
// block CFG (labels + goto). Blocks that become unreachable after CFG construction
// (e.g. the fall-through after an early `return` in `clampIndex`) leave a defined-
// but-unused label — inherent to CFG codegen, exactly like the unused-variable/
// function tolerances already applied to generated TUs. Downgrade for generated
// code so a cosmetic dead-label can't break app builds.
// -Wno-error=overloaded-virtual: a generated class hierarchy re-declares a base
// virtual (e.g. the `__sym_GEA_CREATE_TEMPLATE` reactive hook) on a derived class
// with a related-but-distinct signature; gcc warns the base is "hidden". This is
// benign for the generated dispatch surface — downgrade for generated code.
const GENERATED_WARNING_TOLERANCE_FLAGS = ['-Wno-error=infinite-recursion', '-Wno-error=unused-label', '-Wno-error=overloaded-virtual']

function sha256(parts) {
  const hash = createHash('sha256')
  for (const part of parts) {
    hash.update(String(part))
    hash.update('\0')
  }
  return hash.digest('hex')
}

function sourceObjectHash(sourcePath) {
  // Match CMake's previous `string(SHA256 hash "${source}")` object naming so
  // the migration does not create a second set of differently named objects.
  return createHash('sha256').update(sourcePath).digest('hex')
}

function positiveInteger(value) {
  const parsed = Number(value)
  return Number.isInteger(parsed) && parsed > 0 ? parsed : null
}

export function defaultJobCount(env = process.env) {
  const explicit = positiveInteger(env.GEA_GEATSC_JOBS)
  if (explicit) return explicit

  const hostCpuCount = typeof os.availableParallelism === 'function' ? os.availableParallelism() : os.cpus().length
  const cpuCount = positiveInteger(env.CMAKE_BUILD_PARALLEL_LEVEL) ?? hostCpuCount
  // Generated C++ frontends remain memory-heavy even after moving shared
  // runtime code out of each module. Keep the automatic batch small enough
  // that an ESP build can coexist with the IDF and native builds. Developers
  // who prefer throughput can opt into a larger batch with GEA_GEATSC_JOBS.
  const memoryCount = Math.max(1, Math.floor(os.totalmem() / (3 * 1024 ** 3)))
  return Math.max(1, Math.min(4, cpuCount || 1, memoryCount))
}

function firstRule(text) {
  return text.replace(/\\\r?\n/g, ' ').split(/\r?\n/, 1)[0] ?? ''
}

function ruleColon(rule) {
  let escaped = false
  for (let index = 0; index < rule.length; index++) {
    const char = rule[index]
    if (escaped) {
      escaped = false
      continue
    }
    if (char === '\\') {
      escaped = true
      continue
    }
    if (char === ':') return index
  }
  return -1
}

export function parseDepfile(text, cwd = process.cwd()) {
  const rule = firstRule(text)
  const colon = ruleColon(rule)
  if (colon < 0) return []

  const dependencies = []
  let token = ''
  let escaped = false
  const flush = () => {
    if (!token) return
    dependencies.push(path.isAbsolute(token) ? path.normalize(token) : path.resolve(cwd, token))
    token = ''
  }

  for (const char of rule.slice(colon + 1)) {
    if (escaped) {
      token += char
      escaped = false
    } else if (char === '\\') {
      escaped = true
    } else if (/\s/.test(char)) {
      flush()
    } else {
      token += char
    }
  }
  if (escaped) token += '\\'
  flush()
  return dependencies
}

function depfileDependencies(depfilePath) {
  if (!existsSync(depfilePath)) return []
  return parseDepfile(readFileSync(depfilePath, 'utf8'))
}

function toolIdentity(toolPath) {
  try {
    const resolved = realpathSync(toolPath)
    const stat = statSync(resolved)
    return `${resolved}\0${stat.size}\0${stat.mtimeMs}`
  } catch {
    return toolPath
  }
}

// Windows executables carry an extension from PATHEXT, so a bare name such as
// `ccache` matches nothing on PATH there and the lookup used to fall through to
// `<cwd>/ccache` -- which then failed the existence check and stopped the build
// as soon as the caller started passing a launcher.
function executableCandidates(tool) {
  if (process.platform !== 'win32') return [tool]
  const extensions = (process.env.PATHEXT ?? '.COM;.EXE;.BAT;.CMD').split(';').filter(Boolean)
  return [tool, ...extensions.map((extension) => `${tool}${extension}`)]
}

function resolveExecutable(tool) {
  if (path.isAbsolute(tool)) return path.normalize(tool)
  if (tool.includes(path.sep) || tool.includes('/')) return path.resolve(tool)
  const candidates = executableCandidates(tool)
  for (const directory of (process.env.PATH ?? '').split(path.delimiter).filter(Boolean)) {
    for (const name of candidates) {
      const candidate = path.join(directory, name)
      if (existsSync(candidate)) return candidate
    }
  }
  return path.resolve(tool)
}

function writeFileAtomic(filePath, contents) {
  mkdirSync(path.dirname(filePath), { recursive: true })
  const temporaryPath = `${filePath}.${process.pid}.${Math.random().toString(16).slice(2)}.tmp`
  writeFileSync(temporaryPath, contents)
  try {
    replaceFile(temporaryPath, filePath)
  } catch (error) {
    rmSync(temporaryPath, { force: true })
    throw error
  }
}

function writeFileIfChanged(filePath, contents) {
  if (existsSync(filePath) && readFileSync(filePath, 'utf8') === contents) return
  writeFileAtomic(filePath, contents)
}

function replaceFile(sourcePath, destinationPath) {
  try {
    renameSync(sourcePath, destinationPath)
  } catch (error) {
    if (process.platform !== 'win32') throw error
    rmSync(destinationPath, { force: true })
    renameSync(sourcePath, destinationPath)
  }
}

function objectNeedsCompile(record, compileSignature) {
  if (!existsSync(record.objectPath) || !existsSync(record.depfilePath) || !existsSync(record.signaturePath)) return true
  if (readFileSync(record.signaturePath, 'utf8').trim() !== compileSignature) return true

  const objectTime = statSync(record.objectPath).mtimeMs
  if (statSync(record.sourcePath).mtimeMs > objectTime) return true
  const dependencies = depfileDependencies(record.depfilePath)
  if (dependencies.length === 0) return true
  for (const dependency of dependencies) {
    if (!existsSync(dependency) || statSync(dependency).mtimeMs > objectTime) return true
  }
  return false
}

function runCommand(command, args) {
  return new Promise((resolve, reject) => {
    const child = spawn(command, args, { stdio: ['ignore', 'pipe', 'pipe'], windowsHide: true })
    let stdout = ''
    let stderr = ''
    let spawnError = null
    child.stdout.on('data', (chunk) => {
      stdout += chunk
    })
    child.stderr.on('data', (chunk) => {
      stderr += chunk
    })
    child.on('error', (error) => {
      spawnError = error
    })
    child.on('close', (status, signal) => {
      if (spawnError) {
        reject(spawnError)
      } else if (status !== 0) {
        const detail = stderr || stdout || `terminated by ${signal ?? `exit ${status}`}`
        reject(new Error(`${command} failed: ${detail.trim()}`))
      } else {
        resolve({ stdout, stderr })
      }
    })
  })
}

async function compileObject(record, options, compileSignature) {
  const suffix = `${process.pid}.${Math.random().toString(16).slice(2)}.tmp`
  const temporaryObject = `${record.objectPath}.${suffix}`
  const temporaryDepfile = `${record.depfilePath}.${suffix}`
  mkdirSync(path.dirname(record.objectPath), { recursive: true })
  try {
    const compilerArgs = [
      `@${options.flagsFile}`,
      ...options.generatedCompileFlags,
      '-MMD',
      '-MP',
      '-MF',
      temporaryDepfile,
      '-c',
      record.sourcePath,
      '-o',
      temporaryObject,
    ]
    const result = options.compilerLauncher
      ? await runCommand(options.compilerLauncher, [options.compiler, ...compilerArgs])
      : await runCommand(options.compiler, compilerArgs)
    if (result.stdout) process.stdout.write(result.stdout)
    if (result.stderr) process.stderr.write(result.stderr)
    replaceFile(temporaryObject, record.objectPath)
    replaceFile(temporaryDepfile, record.depfilePath)
    writeFileAtomic(record.signaturePath, `${compileSignature}\n`)
  } catch (error) {
    rmSync(temporaryObject, { force: true })
    rmSync(temporaryDepfile, { force: true })
    throw new Error(`failed to compile ${record.sourcePath}: ${error instanceof Error ? error.message : String(error)}`)
  }
}

function contentHash(filePath) {
  try {
    return createHash('sha256').update(readFileSync(filePath)).digest('hex')
  } catch {
    return `missing:${filePath}`
  }
}

// Content-based staleness for the PCH: geatsc rewrites every generated file
// with a fresh mtime on each emission even when the bytes are identical, so
// the mtime-based objectNeedsCompile would rebuild the (expensive) .gch every
// build. Hash the header and every dependency recorded by the previous
// compile's depfile instead; only real content changes rebuild it.
function pchSignatureFor(options, record) {
  const dependencies = depfileDependencies(record.depfilePath)
  const dependencyHashes = [...new Set([record.sourcePath, ...dependencies])]
    .sort()
    .map((dependency) => `${dependency}\0${contentHash(dependency)}`)
  return sha256([
    PCH_SIGNATURE_VERSION,
    toolIdentity(options.compiler),
    readFileSync(options.flagsFile),
    ...options.generatedCompileFlags,
    ...dependencyHashes,
  ])
}

/**
 * The header to precompile for this generated directory.
 *
 * Preferred: the `runtime=` line of `geatsc-header.txt`, which a per-file
 * compile writes -- the compiler's own prelude (standard headers, the hosts'
 * preambles, `gea_runtime.h`, the hosts' headers) and nothing else. It has to
 * be that text: the preambles PRECEDE `gea_runtime.h` and decide what it
 * declares, so a PCH built from a header without them declares engine
 * stand-ins that collide with the engine's own types in every unit.
 *
 * Two reasons to prefer it over `runtime_pch.h`. It is the only one the
 * current compiler emits at all -- `runtime_pch.h` was the previous one's
 * umbrella, so on a directory this compiler generated the fallback finds
 * nothing and the whole PCH path was dead. And it is far smaller: the umbrella
 * pulls the runtime's `value_01..25` implementations in and produces a ~174MB
 * .gch, the size at which xtensa gcc 15.2 nondeterministically ICEs while
 * loading it (which is why `GEA_GEATSC_PCH` is opt-in). The prelude alone is
 * a fraction of that, so it is the configuration in which the opt-in has a
 * chance of becoming the default -- unproven on device, which is why this
 * changes the source and not the gate.
 *
 * One layer only, unlike the macOS chain: GCC takes a single PCH per compile
 * and cannot chain one on another, so the program header (`<stem>.hpp`, which
 * changes whenever a declaration does) is not precompiled here. The runtime
 * layer is the stable one and most of the win.
 */
function pchHeaderFor(options) {
  const dir = path.dirname(options.sourceList)
  const manifest = path.join(dir, 'geatsc-header.txt')
  if (existsSync(manifest)) {
    for (const line of readFileSync(manifest, 'utf8').split('\n')) {
      if (!line.startsWith('runtime=')) continue
      const named = line.slice('runtime='.length).trim()
      if (named && existsSync(named)) return named
    }
  }
  const fallback = path.join(dir, PCH_HEADER_NAME)
  return existsSync(fallback) ? fallback : null
}

async function prepareRuntimePch(options) {
  if (process.env.GEA_GEATSC_PCH !== '1') return null
  const headerPath = pchHeaderFor(options)
  if (!headerPath) return null
  const gchPath = `${headerPath}.gch`
  const headerName = path.basename(headerPath)
  const record = {
    sourcePath: headerPath,
    objectPath: gchPath,
    depfilePath: path.join(options.objectDir, `${headerName}.gch.d`),
    signaturePath: path.join(options.objectDir, `${headerName}.gch.compile.sig`),
  }
  const signature = pchSignatureFor(options, record)
  const stale =
    !existsSync(record.objectPath) ||
    !existsSync(record.depfilePath) ||
    !existsSync(record.signaturePath) ||
    readFileSync(record.signaturePath, 'utf8').trim() !== signature
  if (stale) {
    const suffix = `${process.pid}.${Math.random().toString(16).slice(2)}.tmp`
    const temporaryGch = `${gchPath}.${suffix}`
    const temporaryDepfile = `${record.depfilePath}.${suffix}`
    mkdirSync(options.objectDir, { recursive: true })
    try {
      // The launcher (ccache) is deliberately skipped: this is one compile per
      // flag-set, and ccache treats PCH *creation* as uncacheable anyway.
      const result = await runCommand(options.compiler, [
        `@${options.flagsFile}`,
        ...options.generatedCompileFlags,
        '-x',
        'c++-header',
        '-MMD',
        '-MP',
        '-MF',
        temporaryDepfile,
        '-c',
        headerPath,
        '-o',
        temporaryGch,
      ])
      if (result.stdout) process.stdout.write(result.stdout)
      if (result.stderr) process.stderr.write(result.stderr)
      replaceFile(temporaryGch, gchPath)
      replaceFile(temporaryDepfile, record.depfilePath)
      // Re-derive the signature from the freshly written depfile: the first
      // build has no previous depfile, so the pre-compile signature covered
      // only the header itself.
      writeFileAtomic(record.signaturePath, `${pchSignatureFor(options, record)}\n`)
    } catch (error) {
      rmSync(temporaryGch, { force: true })
      rmSync(temporaryDepfile, { force: true })
      throw new Error(`failed to precompile ${headerPath}: ${error instanceof Error ? error.message : String(error)}`)
    }
  }
  return { gchPath, headerPath, rebuilt: stale }
}

async function compileInParallel(records, options, compileSignature, jobs) {
  let nextIndex = 0
  let failed = false
  const errors = []
  const worker = async () => {
    while (!failed) {
      const index = nextIndex++
      if (index >= records.length) return
      try {
        await compileObject(records[index], options, compileSignature)
      } catch (error) {
        failed = true
        errors.push(error)
      }
    }
  }
  const workerCount = Math.min(jobs, records.length)
  await Promise.all(Array.from({ length: workerCount }, () => worker()))
  if (errors.length > 0) throw errors[0]
}

function makeEscape(value) {
  return value.replace(/\\/g, '\\\\').replace(/\$/g, '$$$$').replace(/#/g, '\\#').replace(/ /g, '\\ ').replace(/:/g, '\\:')
}

function writeAggregateDepfile(options, records) {
  const dependencies = new Set([options.sourceList, options.flagsFile])
  for (const record of records) {
    dependencies.add(record.sourcePath)
    for (const dependency of depfileDependencies(record.depfilePath)) dependencies.add(dependency)
  }
  const continuation = ` \\${String.fromCharCode(10)}  `
  const lines = [...dependencies].sort().map((dependency) => continuation + makeEscape(dependency))
  writeFileIfChanged(options.depfile, `${makeEscape(options.archive)}:${lines.join('')}\n`)
}

async function rebuildArchive(options, records) {
  mkdirSync(path.dirname(options.archive), { recursive: true })
  const temporaryArchive = `${options.archive}.${process.pid}.${Math.random().toString(16).slice(2)}.tmp`
  rmSync(temporaryArchive, { force: true })
  try {
    const archived = await runCommand(options.ar, ['qc', temporaryArchive, ...records.map((record) => record.objectPath)])
    if (archived.stdout) process.stdout.write(archived.stdout)
    if (archived.stderr) process.stderr.write(archived.stderr)
    if (options.ranlib) {
      const indexed = await runCommand(options.ranlib, [temporaryArchive])
      if (indexed.stdout) process.stdout.write(indexed.stdout)
      if (indexed.stderr) process.stderr.write(indexed.stderr)
    }
    replaceFile(temporaryArchive, options.archive)
  } catch (error) {
    rmSync(temporaryArchive, { force: true })
    throw error
  }
}

function readSources(sourceList) {
  const sourceRoot = path.dirname(sourceList)
  const sources = readFileSync(sourceList, 'utf8')
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter(Boolean)
    .map((source) => (path.isAbsolute(source) ? path.normalize(source) : path.resolve(sourceRoot, source)))
  if (sources.length === 0) throw new Error(`geatsc source list is empty: ${sourceList}`)
  for (const source of sources) {
    if (!existsSync(source)) throw new Error(`geatsc generated source not found: ${source}`)
  }
  return sources
}

export async function buildGeneratedArchive(rawOptions) {
  const options = {
    ...rawOptions,
    sourceList: path.resolve(rawOptions.sourceList),
    archive: path.resolve(rawOptions.archive),
    objectDir: path.resolve(rawOptions.objectDir),
    depfile: path.resolve(rawOptions.depfile),
    flagsFile: path.resolve(rawOptions.flagsFile),
    compiler: path.resolve(rawOptions.compiler),
    compilerLauncher: rawOptions.compilerLauncher ? resolveExecutable(rawOptions.compilerLauncher) : '',
    ar: path.resolve(rawOptions.ar),
    ranlib: rawOptions.ranlib ? path.resolve(rawOptions.ranlib) : '',
    // ESP-IDF enables full DWARF globally. The geatsc runtime interface is
    // intentionally shared by every generated translation unit, so emitting
    // its type information into every object made even a four-line module
    // tens of megabytes. Keep source-level generated-code debugging available
    // as an explicit opt-in without charging normal app builds for it.
    generatedCompileFlags: [
      ...GENERATED_WARNING_TOLERANCE_FLAGS,
      ...(rawOptions.debugInfo === true || process.env.GEA_GEATSC_DEBUG_INFO === '1' ? [] : NO_GENERATED_DEBUG_INFO_FLAGS),
    ],
  }
  for (const [name, filePath] of Object.entries({
    sourceList: options.sourceList,
    flagsFile: options.flagsFile,
    compiler: options.compiler,
    ...(options.compilerLauncher ? { compilerLauncher: options.compilerLauncher } : {}),
    ar: options.ar,
  })) {
    if (!existsSync(filePath)) throw new Error(`${name} not found: ${filePath}`)
  }
  if (options.ranlib && !existsSync(options.ranlib)) throw new Error(`ranlib not found: ${options.ranlib}`)

  const sources = readSources(options.sourceList)
  mkdirSync(options.objectDir, { recursive: true })
  const records = sources.map((sourcePath) => {
    const extension = path.extname(sourcePath)
    const objectPath = path.join(options.objectDir, `${sourceObjectHash(sourcePath)}${extension}.o`)
    return {
      sourcePath,
      objectPath,
      depfilePath: `${objectPath}.d`,
      signaturePath: `${objectPath}.compile.sig`,
    }
  })

  const pch = await prepareRuntimePch(options)
  if (pch) {
    // GCC only consumes a .gch when the header is named from the MAIN file;
    // the generated TUs reach runtime_pch.h through generated_support.hpp, one
    // include level down, which silently degrades to textual inclusion (no
    // -Winvalid-pch diagnostic — the PCH is simply never considered). Forcing
    // the header first via -include makes it a main-file include, and the
    // TU's own nested include then no-ops on the already-recorded guard.
    options.generatedCompileFlags = [...options.generatedCompileFlags, ...PCH_CONSUMER_FLAGS, '-include', pch.headerPath]
    // ccache rejects (or worse, mis-keys) GCC-PCH compiles unless it is told
    // to ignore the defines/time macros folded into the .gch. The env var
    // overrides ccache.conf wholesale, so merge instead of clobbering.
    if (options.compilerLauncher && path.basename(options.compilerLauncher).includes('ccache')) {
      const sloppiness = new Set(
        (process.env.CCACHE_SLOPPINESS ?? '')
          .split(',')
          .map((entry) => entry.trim())
          .filter(Boolean),
      )
      sloppiness.add('pch_defines')
      sloppiness.add('time_macros')
      process.env.CCACHE_SLOPPINESS = [...sloppiness].join(',')
    }
  }

  const compileSignature = sha256([
    OBJECT_SIGNATURE_VERSION,
    options.compilerLauncher ? toolIdentity(options.compilerLauncher) : '',
    toolIdentity(options.compiler),
    readFileSync(options.flagsFile),
    '@flags',
    '-MMD',
    '-MP',
    ...options.generatedCompileFlags,
    // A new .gch means every object that baked the old one in must rebuild;
    // stat identity is stable across builds that reuse the cached PCH.
    pch ? toolIdentity(pch.gchPath) : 'no-pch',
  ])
  const staleRecords = records.filter((record) => objectNeedsCompile(record, compileSignature))
  const jobs = positiveInteger(rawOptions.jobs) ?? defaultJobCount()
  await compileInParallel(staleRecords, options, compileSignature, jobs)

  const archiveSignature = sha256([
    ARCHIVE_SIGNATURE_VERSION,
    toolIdentity(options.ar),
    options.ranlib ? toolIdentity(options.ranlib) : '',
    ...records.flatMap((record) => {
      const stat = statSync(record.objectPath)
      return [record.objectPath, stat.size, stat.mtimeMs]
    }),
  ])
  const archiveSignaturePath = `${options.archive}.objects.sig`
  const previousArchiveSignature = existsSync(archiveSignaturePath) ? readFileSync(archiveSignaturePath, 'utf8').trim() : ''
  const archiveChanged = !existsSync(options.archive) || previousArchiveSignature !== archiveSignature
  if (archiveChanged) {
    await rebuildArchive(options, records)
    writeFileAtomic(archiveSignaturePath, `${archiveSignature}\n`)
  }
  writeAggregateDepfile(options, records)

  const compiled = staleRecords.length
  const reused = records.length - compiled
  process.stdout.write(
    `geatsc-generated-archive: compiled=${compiled} reused=${reused} jobs=${Math.min(jobs, Math.max(1, compiled))} archived=${archiveChanged ? 1 : 0} pch=${pch ? (pch.rebuilt ? 'built' : 'reused') : 'off'}\n`,
  )
  return { compiled, reused, jobs, archived: archiveChanged, records, pch }
}

function parseArguments(argv) {
  const options = {}
  for (let index = 0; index < argv.length; index++) {
    const argument = argv[index]
    if (!argument.startsWith('--')) throw new Error(`unexpected argument: ${argument}`)
    const equals = argument.indexOf('=')
    const key = (equals >= 0 ? argument.slice(2, equals) : argument.slice(2)).replace(/-([a-z])/g, (_, char) => char.toUpperCase())
    const value = equals >= 0 ? argument.slice(equals + 1) : argv[++index]
    if (value === undefined) throw new Error(`missing value for ${argument}`)
    options[key] = value
  }
  for (const required of ['sourceList', 'archive', 'objectDir', 'depfile', 'flagsFile', 'compiler', 'ar']) {
    if (!options[required]) throw new Error(`missing --${required.replace(/[A-Z]/g, (char) => `-${char.toLowerCase()}`)}`)
  }
  return options
}

async function main() {
  await buildGeneratedArchive(parseArguments(process.argv.slice(2)))
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  main().catch((error) => {
    process.stderr.write(`geatsc-generated-archive: ${error instanceof Error ? error.message : String(error)}\n`)
    process.exitCode = 1
  })
}
