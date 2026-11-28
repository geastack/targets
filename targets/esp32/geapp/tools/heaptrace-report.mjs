#!/usr/bin/env node
// SPDX-License-Identifier: GPL-3.0-only
// Groups the allocations a `gea devctl heaptrace stop` dump still holds by the
// call site that made them, symbolized against the firmware and, for frames
// inside a loaded program, against that program's unstripped image.
//
//   node heaptrace-report.mjs <dump.txt> --firmware <gea_embedded.elf>
//        [--program <program.debug.so> --base <0x...>] [--region internal|external]
//        [--depth 5] [--top 30] [--addr2line <riscv32-esp-elf-addr2line>]
//
// The base is what `gea devctl back` prints for the program it unloads
// (`base=0x...`). Firmware needs CONFIG_HEAP_TRACING_STANDALONE, and on RISC-V
// CONFIG_ESP_SYSTEM_USE_FRAME_POINTER for callers to be recorded at all.

import { execFileSync } from 'node:child_process'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'

const ALLOCATOR_FRAMES = [
  'malloc', 'heap_caps', 'operator new', 'trace_malloc', 'multi_heap', 'tlsf', 'calloc', 'realloc',
  '__new_allocator', 'std::allocator', 'allocate', 'AllocationPool', 'pvPortMalloc', 'xQueueGenericCreate',
  'pthread_mutex_init', '_Sp_counted', '__shared_count', 'make_shared', '__allocate_shared'
]

function option(args, name, fallback = '') {
  const index = args.indexOf(`--${name}`)
  return index >= 0 && index + 1 < args.length ? args[index + 1] : fallback
}

function findAddr2line() {
  const root = path.join(os.homedir(), '.espressif', 'tools', 'riscv32-esp-elf')
  for (const version of fs.existsSync(root) ? fs.readdirSync(root).sort().reverse() : []) {
    const candidate = path.join(root, version, 'riscv32-esp-elf', 'bin', 'riscv32-esp-elf-addr2line')
    if (fs.existsSync(candidate)) return candidate
  }
  return 'riscv32-esp-elf-addr2line'
}

// A record line can be cut by an interleaved log line: keep only whole hex frames.
export function parseDump(text) {
  const records = []
  for (const line of text.split('\n')) {
    const match = line.match(/bytes=(\d+) region=(\w+).*?stack=(\S*)/)
    if (!match) continue
    const frames = match[3].split(',').filter((token) => /^0x[0-9a-f]+$/i.test(token) && !/^0x0+$/i.test(token)).map((token) => parseInt(token, 16))
    records.push({ bytes: Number(match[1]), region: match[2], frames })
  }
  return records
}

function symbolize(addr2line, elf, addresses) {
  const names = new Map()
  for (let i = 0; i < addresses.length; i += 400) {
    const batch = addresses.slice(i, i + 400)
    const output = execFileSync(addr2line, ['-fC', '-e', elf, ...batch.map((a) => `0x${a.toString(16)}`)], { encoding: 'utf8', maxBuffer: 1 << 26 }).split('\n')
    batch.forEach((address, index) => names.set(address, output[index * 2] || '??'))
  }
  return names
}

function main(args) {
  const dump = args[0]
  const firmware = option(args, 'firmware')
  if (!dump || !firmware) {
    console.error('usage: heaptrace-report.mjs <dump.txt> --firmware <elf> [--program <so> --base <0x...>] [--region r] [--depth n] [--top n]')
    process.exit(2)
  }
  const program = option(args, 'program')
  const base = program ? parseInt(option(args, 'base', '0'), 16) : 0
  const programEnd = program ? base + fs.statSync(program).size : 0
  const region = option(args, 'region')
  const depth = Number(option(args, 'depth', '5'))
  const top = Number(option(args, 'top', '30'))
  const addr2line = option(args, 'addr2line') || findAddr2line()

  const records = parseDump(fs.readFileSync(dump, 'utf8')).filter((record) => !region || record.region === region)
  const inProgram = (address) => program && address >= base && address < programEnd
  const addresses = [...new Set(records.flatMap((record) => record.frames))]
  const names = symbolize(addr2line, firmware, addresses.filter((a) => !inProgram(a)))
  if (program) {
    const local = addresses.filter(inProgram)
    const programNames = symbolize(addr2line, program, local.map((a) => a - base))
    for (const address of local) names.set(address, `P:${programNames.get(address - base)}`)
  }

  const groups = new Map()
  let total = 0
  for (const record of records) {
    const site = record.frames.map((a) => names.get(a) || '??').filter((name) => !ALLOCATOR_FRAMES.some((frame) => name.includes(frame))).slice(0, depth).map((name) => name.slice(0, 70)).join(' < ')
    const key = `${record.region} ${site}`
    const group = groups.get(key) || { bytes: 0, count: 0 }
    group.bytes += record.bytes
    group.count += 1
    groups.set(key, group)
    total += record.bytes
  }
  console.log(`${records.length} live allocations, ${total} bytes`)
  for (const [key, group] of [...groups].sort((a, b) => b[1].bytes - a[1].bytes).slice(0, top))
    console.log(`${String(group.bytes).padStart(8)} x${String(group.count).padEnd(4)} ${key}`)
}

main(process.argv.slice(2))
