#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
// This board's own half. The defaults for both macros live in @geastack/core
// (gea_perf_config.h) and the shared esp32 sources are gated once in
// targets/esp32/test -- each asserted where it lives.
const framePerfMacro = 'GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG'
const heapDiagnosticsMacro = 'GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG'

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

function assertPatternBehindMacro(path, macro, patterns) {
  const lines = source(path).split(/\r?\n/)
  const guardedStack = []

  for (const [index, line] of lines.entries()) {
    const directive = line.match(/^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$/)
    if (directive) {
      const [, kind, rest] = directive
      if (kind === 'if' || kind === 'ifdef' || kind === 'ifndef') {
        guardedStack.push(guardedStack.includes(true) || rest.includes(macro))
      } else if (kind === 'elif') {
        const parentGuarded = guardedStack.slice(0, -1).includes(true)
        guardedStack[guardedStack.length - 1] = parentGuarded || rest.includes(macro)
      } else if (kind === 'else') {
        // Keep the enclosing gate active for both branches of a macro-only block.
      } else if (kind === 'endif') {
        guardedStack.pop()
      }
    }

    for (const pattern of patterns) {
      if (!pattern.test(line)) continue
      assert.ok(
        guardedStack.includes(true),
        `${path}:${index + 1} should keep ${pattern} behind ${macro}`
      )
    }
  }
}

const boardCmake = source('targets/esp32-s3-touch-amoled-2.06/main/CMakeLists.txt')
assert.match(
  boardCmake,
  /set\(GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE 0\)[\s\S]*DEFINED ENV\{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG\}/,
  `${heapDiagnosticsMacro} should be default-off and env-overridable for diagnostic runs`
)
assert.match(
  boardCmake,
  /GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG=\$\{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE\}/,
  `${heapDiagnosticsMacro} should be passed as a board compile definition`
)

assertPatternBehindMacro('targets/esp32-s3-touch-amoled-2.06/main/app_main.cpp', framePerfMacro, [
  /stack probe \[/,
])

assertPatternBehindMacro('targets/esp32-s3-touch-amoled-2.06/main/app_main.cpp', heapDiagnosticsMacro, [
  /heap probe \[/,
  /logHeapProbe\(/
])

assertPatternBehindMacro('targets/esp32-s3-touch-amoled-2.06/main/launcher_button.cpp', framePerfMacro, [
  /stack probe \[/,
])

assertPatternBehindMacro('targets/esp32-s3-touch-amoled-2.06/main/launcher_button.cpp', heapDiagnosticsMacro, [
  /heap probe \[/,
  /logInternalHeap\(/
])
