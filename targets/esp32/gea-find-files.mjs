#!/usr/bin/env node
// Portable replacement for the find(1) walk the target CMakeLists used to enumerate
// geatsc dependencies. Windows has no find(1) (find.exe is a text filter), so the
// configure step there produced an empty DEPENDS list and TSX edits never re-ran
// codegen. Node is already required by the build, so the walk lives here.
//
// usage: node gea-find-files.mjs <dir> [--follow-symlinks] [--prune a,b] [--ext .x,.y] [--name f.json,...]
//
// Prints one absolute path per line, forward slashes, in directory order. Pruned
// directory names are skipped at the directory boundary (like find -prune), which is
// what keeps configure fast next to a node_modules tree. A missing directory prints
// nothing and exits 0, matching the old behaviour where find's error was ignored.
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

function parseArgs(argv) {
  const options = { dir: null, followSymlinks: false, prune: [], ext: [], name: [] }
  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i]
    const list = () => String(argv[++i] || '').split(',').filter(Boolean)
    if (arg === '--follow-symlinks') options.followSymlinks = true
    else if (arg === '--prune') options.prune.push(...list())
    else if (arg === '--ext') options.ext.push(...list())
    else if (arg === '--name') options.name.push(...list())
    else if (arg.startsWith('--')) throw new Error(`unknown option ${arg}`)
    else if (options.dir === null) options.dir = arg
    else throw new Error(`unexpected argument ${arg}`)
  }
  if (options.dir === null) throw new Error('usage: gea-find-files.mjs <dir> [options]')
  return options
}

export function findFiles({ dir, followSymlinks = false, prune = [], ext = [], name = [] }) {
  const pruned = new Set(prune)
  const exts = new Set(ext.map((e) => e.toLowerCase()))
  const names = new Set(name)
  const visited = new Set()
  const out = []

  const kind = (dirent, full) => {
    if (dirent.isDirectory()) return 'dir'
    if (dirent.isFile()) return 'file'
    if (dirent.isSymbolicLink() && followSymlinks) {
      try {
        const stat = fs.statSync(full)
        if (stat.isDirectory()) return 'dir'
        if (stat.isFile()) return 'file'
      } catch {
        return null
      }
    }
    return null
  }

  const walk = (current) => {
    if (followSymlinks) {
      let real
      try {
        real = fs.realpathSync(current)
      } catch {
        return
      }
      if (visited.has(real)) return
      visited.add(real)
    }
    let entries
    try {
      entries = fs.readdirSync(current, { withFileTypes: true })
    } catch {
      return
    }
    for (const entry of entries) {
      const full = path.join(current, entry.name)
      const type = kind(entry, full)
      if (type === 'dir') {
        if (!pruned.has(entry.name)) walk(full)
      } else if (type === 'file') {
        if (names.has(entry.name) || exts.has(path.extname(entry.name).toLowerCase())) {
          out.push(full.split(path.sep).join('/'))
        }
      }
    }
  }

  let root = path.resolve(dir)
  try {
    root = fs.realpathSync(root)
  } catch {
    return out
  }
  walk(root)
  return out
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const files = findFiles(parseArgs(process.argv.slice(2)))
    if (files.length) process.stdout.write(files.join('\n') + '\n')
  } catch (error) {
    process.stderr.write(`gea-find-files: ${error.message}\n`)
    process.exit(2)
  }
}
