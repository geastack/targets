#!/usr/bin/env node
// The contracts the STM32F746G-Discovery target keeps with the gea CLI (firmware
// stems, metadata, resolved build settings), with the generated app (pixel
// order, display geometry) and with its own linker script (QSPI archive
// placement, unwind tables, TLS).
import assert from 'node:assert/strict'
import { existsSync, readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)
const targetDir = 'targets/stm32f746g-disco'

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

const targetMetadata = JSON.parse(source('targets.json'))
const buildConfig = JSON.parse(source('build-config.json'))
const cmake = source(`${targetDir}/CMakeLists.txt`)
const linkerScript = source(`${targetDir}/stm32f746g_disco.ld`)
const boardHeader = source(`${targetDir}/main/include/board.h`)

assert.deepEqual(
  targetMetadata['stm32f746g-disco'],
  { adapter: 'stm32-cube', targetPath: targetDir, flashSize: '1MB', appPlatform: 'stm32' },
  'the Discovery should advertise the STM32Cube adapter metadata'
)
assert.ok(buildConfig.platforms.stm32, 'the stm32 app platform needs its build defaults')
assert.equal(buildConfig.boards['stm32f746g-disco'].cssDevicePixelRatio, 1, 'the 480x272 panel lays out at 1 CSS px per pixel')

// The CLI builds `<stem>_app` / `<stem>_bringup` and flashes `<target>.elf`.
assert.match(cmake, /set\(GEA_STM32_STEM gea_stm32f746g_disco\)/)
assert.match(cmake, /add_executable\(\$\{GEA_STM32_STEM\}_bringup /)
assert.match(cmake, /add_executable\(\$\{GEA_STM32_STEM\}_app\b/)

// The CMSIS headers and startup file are vendored: a build needs no STM32CubeF7.
assert.match(cmake, /set\(GEA_STM32_CMSIS "\$\{CMAKE_CURRENT_LIST_DIR\}\/vendor\/cmsis"\)/)
assert.ok(existsSync(resolve(repoRoot, targetDir, 'vendor/cmsis/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s')))

// Resolved package.json build settings reach codegen and the native build.
assert.match(cmake, /include\("\$\{CMAKE_CURRENT_LIST_DIR\}\/\.\.\/cmake\/gea_build_config\.cmake"\)/)
assert.match(cmake, /--build-config "\$\{GEA_BUILD_CONFIG_JSON\}"/)
assert.match(cmake, /target_compile_definitions\(\$\{GEA_STM32_STEM\}_app PRIVATE\s+\$\{GEA_EMBEDDED_APP_DEFINES\}/)

// The LTDC scans little-endian RGB565: codegen colour literals and the
// runtime's pixel order must agree, as must the geometry both are told.
assert.match(cmake, /--pixel-panel-endian 0/)
assert.match(cmake, /GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=0/)
assert.match(cmake, /set\(GEA_STM32_DISPLAY_WIDTH 480\)/)
assert.match(cmake, /set\(GEA_STM32_DISPLAY_HEIGHT 272\)/)
assert.match(cmake, /--font-viewport-width \$\{GEA_STM32_DISPLAY_WIDTH\}/)
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_WIDTH=\$\{GEA_STM32_DISPLAY_WIDTH\}/)
assert.match(cmake, /^\s+GEA_EMBEDDED_CPP_BOARD$/m, 'board-sized runtime allocation pools')

// The generated app runs from QSPI: its archive name must match the linker
// script's *_qspi.a rule and it must name the QSPI personality thunk.
assert.match(cmake, /set\(GEA_APP_ARCHIVE "\$\{GEA_APP_BUILD_DIR\}\/libgea_app_qspi\.a"\)/)
assert.match(cmake, /gea_stm32_place_archive_in_qspi\(\$\{GEA_STM32_STEM\}_app "\$\{GEA_APP_ARCHIVE\}"\)/)
assert.match(cmake, /--redefine-sym __gxx_personality_v0=__gea_qspi_gxx_personality_v0/)

assert.match(linkerScript, /QSPI\s+\(rx\) : ORIGIN = 0x90000000, LENGTH = 16M/)
assert.match(linkerScript, /SDRAM \(rw\) : ORIGIN = 0xC0000000, LENGTH = 8M/)
assert.match(linkerScript, /\*_qspi\.a:\*\(\.text \.text\.\*/)
assert.match(linkerScript, /__qspi_exidx_start = \.;\s+\*_qspi\.a:\*\(\.ARM\.exidx/)
assert.match(linkerScript, /\.tbss_space \(NOLOAD\)/, 'thread_local .tbss needs reserved room')
assert.match(linkerScript, /__heap_start = \.;/)

// Board features: RTC wall clock and the USER button in every image; GPIO
// (LED), GEADEV serial control and multi-touch in the app image.
for (const file of ['main/rtc_clock.cpp', 'main/user_button.cpp', 'main/touch_panel.cpp']) {
  assert.match(cmake, new RegExp(file.replace('.', '\\.')), `board support should compile ${file}`)
}
for (const file of ['main/dev_control.cpp', 'main/platform_gpio.cpp', 'main/platform_touch.cpp']) {
  assert.match(cmake, new RegExp(file.replace('.', '\\.')), `the app image should compile ${file}`)
}
assert.match(cmake, /"\$\{GEA_CHIPS\}\/touch\/ft5336\/ft5336\.cpp"/, 'touch uses the shared FT5336 chip driver')
assert.match(cmake, /GEA_STM32_APP_ID="\$\{GEA_EMBEDDED_APP\}"/, 'GEADEV replies name the app')
assert.match(cmake, /"\$\{GEA_HOST\}\/host\/touch\.cpp"/, 'touch.read() needs the host touch binding')

assert.match(boardHeader, /kSysClockHz = 200'000'000/, 'SDRAM is validated by ST up to 200 MHz')
assert.match(boardHeader, /int width = 480;/)
assert.match(boardHeader, /int height = 272;/)
assert.match(boardHeader, /i2cTiming = 0x5033050C/, 'touch I2C runs in 400 kHz fast mode')

console.log('stm32f746g-disco target config ok')
