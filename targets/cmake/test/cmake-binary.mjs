// The cmake a host test should run, found the way a board build finds one:
// $CMAKE, else cmake on PATH, else the copy ESP-IDF installs under its tools
// directory ($IDF_TOOLS_PATH, default ~/.espressif) -- the cmake every ESP32
// build runs, and the only one a Windows host set up for ESP-IDF may have.
// Returns null when there is none.
import { spawnSync } from 'node:child_process'
import { existsSync, readdirSync } from 'node:fs'
import os from 'node:os'
import path from 'node:path'

const runs = (binary) => spawnSync(binary, ['--version'], { stdio: 'ignore' }).status === 0

export function cmakeBinary(env = process.env) {
  if (env.CMAKE) return env.CMAKE
  if (runs('cmake')) return 'cmake'
  const tools = path.join(env.IDF_TOOLS_PATH || path.join(os.homedir(), '.espressif'), 'tools', 'cmake')
  if (!existsSync(tools)) return null
  const versions = readdirSync(tools)
    .sort((a, b) => b.localeCompare(a, undefined, { numeric: true }))
  for (const version of versions) {
    const binary = path.join(tools, version, 'bin', process.platform === 'win32' ? 'cmake.exe' : 'cmake')
    if (existsSync(binary) && runs(binary)) return binary
  }
  return null
}
