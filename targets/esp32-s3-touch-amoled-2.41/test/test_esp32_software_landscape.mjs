#!/usr/bin/env node
// The panel is natively portrait (410x502) and the CO5300 has no usable XY-swap
// register, so an app that wants landscape uses the software path in
// targets/esp32/display.cpp. That path only engages when the orientation STATE
// agrees: TouchRuntime::transformTouchToLogical rotates controller coordinates
// from the same state, so a target that rotates the framebuffer without setting
// it renders sideways against un-rotated touch. This board builds the shared
// esp32-s3-touch-amoled-1.8/main/app_main.cpp (see this target's main/CMakeLists.txt),
// so that is the file asserted here.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const cmake = readFileSync(resolve(repoRoot, 'targets/esp32-s3-touch-amoled-2.41/main/CMakeLists.txt'), 'utf8')
assert.match(
  cmake,
  /\.\.\/\.\.\/esp32-s3-touch-amoled-1\.8\/main\/app_main\.cpp/,
  'this target is expected to build the shared 1.8 app_main.cpp'
)
const appMain = readFileSync(resolve(repoRoot, 'targets/esp32-s3-touch-amoled-1.8/main/app_main.cpp'), 'utf8')

assert.match(
  appMain,
  /#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY\s*\n#include "host\/display_orientation\.h"/,
  'the orientation state header must be included when software landscape is on'
)

// runtimeOptions() = runtime-orientation branch, then the fixed software-landscape
// branch, then the portrait default.
const landscapeBranch = appMain.match(/runtimeOptions\(\)[\s\S]*?#elif GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY([\s\S]*?)#else([\s\S]*?)#endif\s*\n\treturn options;/)
assert.ok(landscapeBranch, 'runtimeOptions should size the runtime from the orientation state under software landscape')

assert.match(
  landscapeBranch[1],
  /setSupportedOrientations\("landscape-primary"\)/,
  'landscape has to be declared to the orientation state, or display and touch disagree'
)
assert.match(landscapeBranch[1], /options\.width = OrientationState::width\(\)/)
assert.match(landscapeBranch[1], /options\.height = OrientationState::height\(\)/)

assert.match(
  landscapeBranch[2],
  /options\.width = gea::platform::display::kWidth/,
  'the portrait default must stay exactly as it was'
)
