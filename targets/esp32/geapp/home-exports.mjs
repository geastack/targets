#!/usr/bin/env node
// SPDX-License-Identifier: GPL-3.0-only
// Fills the firmware's .geapp export table (geapp_exports.cpp) from the linked
// ELF's own symbol table, in place, before the ELF becomes the flashable image.
//
//   node home-exports.mjs <firmware.elf> <readelf> [--list <exports.txt>]
//
// Exported: every defined global or weak symbol with a nonzero address, minus
//  - anonymous-namespace entities (`_GLOBAL__N_`): they are private to their
//    translation unit, and a program's own may share a mangled name with the
//    firmware's by accident (geatsc numbers declarations per file);
//  - the program hooks: what a program defines for the runtime to call. A
//    program must bind its own, never the firmware program's;
//  - the runtime's per-program registries (PROGRAM_STATE_REGISTRIES).
// The table is keyed by the 64-bit FNV-1a hash of the name, sorted by hash; a
// collision between two exported names fails the build.

import { execFileSync } from 'node:child_process'
import fs from 'node:fs'
import { pathToFileURL } from 'node:url'

const MAGIC = 'GEAXPORT'
const VERSION = 1
const HEADER_BYTES = 24
const ENTRY_BYTES = 16

// Names a program defines and the runtime calls (gea_app_entry.cpp,
// app_main.cpp, the asset and font registries).
export const PROGRAM_HOOK_NAMES = new Set([
  '_Z15__gea_top_levelv',
  '_Z31gea_plugin_cpp_register_preludev',
  '_Z24gea_cpp_clear_microtasksv',
  'gea_cycle_collection_defer_begin',
  'gea_cycle_collection_defer_end',
  'gea_embedded_asset_lookup',
  'gea_embedded_assets_prepare',
  'gea_home_exports'
])
// Whole namespaces of generated program entry points.
export const PROGRAM_HOOK_PREFIXES = [
  '_ZN3gea9framework3app9generated',
  '_ZN3gea9framework8graphics9generated'
]

// The compiler runtime's registries of per-program state (gea_runtime.h):
// class statics, key tables, boxing tables, caches keyed by a program's own
// addresses or declaration names. Bound to the firmware's copy, a program
// files its entries in a table that outlives it -- objects its static
// destructors have destroyed, keys into an image that has been freed, and a
// declaration name another program reuses for a different class. Unexported,
// each program keeps its own copy, destroyed at finalize and freed with it.
// Matched on the mangled function name, so the function, its local statics
// and their guards all stay private together.
export const PROGRAM_STATE_REGISTRIES = [
  '27constructorStaticViewOwners',
  '28sharedConstructorStaticState',
  '23nativeKeyTablePositions',
  '17hostViewToStrings',
  '15boxedPromiseOps',
  '11boxedMapOps',
  '19promisePayloadTypes',
  '17errorPayloadTypes',
  '14symbolRegistry',
  '17declaredSymbolIds',
  '20hostIntrinsicSidecar',
  '23builtinFunctionIdentity',
  '18literalPropertyGet'
]

export function isExportable(name) {
  if (!name || name.includes('_GLOBAL__N_')) return false
  if (PROGRAM_HOOK_NAMES.has(name)) return false
  if (PROGRAM_STATE_REGISTRIES.some((registry) => name.includes(registry))) return false
  return !PROGRAM_HOOK_PREFIXES.some((prefix) => name.startsWith(prefix))
}

export function fnv1a64(name) {
  let hash = 0xcbf29ce484222325n
  for (const byte of Buffer.from(name, 'utf8')) {
    hash ^= BigInt(byte)
    hash = (hash * 0x100000001b3n) & 0xffffffffffffffffn
  }
  return hash
}

// readelf -sW: Num: Value Size Type Bind Vis Ndx Name
export function readSymbols(elf, readelf) {
  const output = execFileSync(readelf, ['-sW', elf], { encoding: 'utf8', maxBuffer: 1 << 28 })
  const symbols = []
  for (const line of output.split('\n')) {
    const match = line.match(/^\s*\d+:\s+([0-9a-f]+)\s+(\d+|0x[0-9a-f]+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)/)
    if (!match) continue
    const [, value, size, type, bind, , ndx, name] = match
    symbols.push({ value: parseInt(value, 16), size: Number(size), type, bind, ndx, name })
  }
  return symbols
}

function readSections(elf, readelf) {
  const output = execFileSync(readelf, ['-SW', elf], { encoding: 'utf8', maxBuffer: 1 << 26 })
  const sections = []
  for (const line of output.split('\n')) {
    const match = line.match(/^\s*\[\s*\d+\]\s+(\S+)\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)/)
    if (!match) continue
    const [, name, type, address, offset, size] = match
    sections.push({ name, type, address: parseInt(address, 16), offset: parseInt(offset, 16), size: parseInt(size, 16) })
  }
  return sections
}

export function exportedSymbols(symbols) {
  const byName = new Map()
  for (const symbol of symbols) {
    // UNIQUE: a header-inline function's static local, the singleton programs must share.
    if (symbol.bind !== 'GLOBAL' && symbol.bind !== 'WEAK' && symbol.bind !== 'UNIQUE') continue
    if (symbol.ndx === 'UND' || symbol.value === 0) continue
    if (symbol.type === 'FILE' || symbol.type === 'SECTION' || symbol.type === 'TLS') continue
    if (!isExportable(symbol.name)) continue
    if (!byName.has(symbol.name)) byName.set(symbol.name, symbol.value)
  }
  return byName
}

export function fillExports(elf, readelf, { list } = {}) {
  const symbols = readSymbols(elf, readelf)
  const table = symbols.find((symbol) => symbol.name === 'gea_home_exports' && symbol.ndx !== 'UND')
  if (!table) throw new Error(`${elf} has no gea_home_exports table (is geapp_exports.cpp linked?)`)
  const section = readSections(elf, readelf).find(
    (candidate) => candidate.type === 'PROGBITS' && table.value >= candidate.address && table.value + table.size <= candidate.address + candidate.size
  )
  if (!section) throw new Error(`gea_home_exports at 0x${table.value.toString(16)} is in no loadable section`)
  const fileOffset = section.offset + (table.value - section.address)

  const fd = fs.openSync(elf, 'r+')
  try {
    const header = Buffer.alloc(HEADER_BYTES)
    fs.readSync(fd, header, 0, HEADER_BYTES, fileOffset)
    if (header.toString('latin1', 0, 8) !== MAGIC) throw new Error(`gea_home_exports in ${elf} does not start with ${MAGIC}`)
    const capacity = header.readUInt32LE(16)
    if (HEADER_BYTES + capacity * ENTRY_BYTES > table.size) throw new Error(`gea_home_exports capacity ${capacity} exceeds its ${table.size} bytes`)

    const exports = exportedSymbols(symbols)
    if (exports.size > capacity) {
      throw new Error(`the firmware exports ${exports.size} symbols but the table holds ${capacity}; raise GEA_GEAPP_EXPORTS_CAPACITY`)
    }
    const entries = []
    const seen = new Map()
    for (const [name, address] of exports) {
      const hash = fnv1a64(name)
      const other = seen.get(hash)
      if (other !== undefined) throw new Error(`export hash collision: ${other} and ${name}`)
      seen.set(hash, name)
      entries.push({ hash, address, name })
    }
    entries.sort((a, b) => (a.hash < b.hash ? -1 : a.hash > b.hash ? 1 : 0))

    const body = Buffer.alloc(HEADER_BYTES + entries.length * ENTRY_BYTES)
    body.write(MAGIC, 0, 'latin1')
    body.writeUInt32LE(VERSION, 8)
    body.writeUInt32LE(entries.length, 12)
    body.writeUInt32LE(capacity, 16)
    entries.forEach((entry, index) => {
      const at = HEADER_BYTES + index * ENTRY_BYTES
      body.writeBigUInt64LE(entry.hash, at)
      body.writeUInt32LE(entry.address >>> 0, at + 8)
    })
    fs.writeSync(fd, body, 0, body.length, fileOffset)
    if (list) fs.writeFileSync(list, entries.map((entry) => entry.name).sort().join('\n') + '\n')
    return { count: entries.length, capacity }
  } finally {
    fs.closeSync(fd)
  }
}

if (process.argv[1] && import.meta.url === pathToFileURL(fs.realpathSync(process.argv[1])).href) {
  const [elf, readelf, ...rest] = process.argv.slice(2)
  if (!elf || !readelf) {
    console.error('usage: home-exports.mjs <firmware.elf> <readelf> [--list <exports.txt>]')
    process.exit(2)
  }
  const listIndex = rest.indexOf('--list')
  const list = listIndex >= 0 ? rest[listIndex + 1] : undefined
  try {
    const { count, capacity } = fillExports(elf, readelf, { list })
    console.log(`geapp: exported ${count} of ${capacity} firmware symbols`)
  } catch (error) {
    console.error(`geapp: ${error.message}`)
    process.exit(1)
  }
}
