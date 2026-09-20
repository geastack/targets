import assert from 'node:assert/strict'
import { readFile } from 'node:fs/promises'
import path from 'node:path'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

const targetDir = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const targetsRoot = path.resolve(targetDir, '../..')

async function text(relativePath) {
  return readFile(path.join(targetDir, relativePath), 'utf8')
}

test('T-Display-S3 Long target matches the vendor display wiring and geometry', async () => {
  const [board, cmake, manifest] = await Promise.all([
    text('main/include/board.h'),
    text('main/CMakeLists.txt'),
    text('main/idf_component.yml')
  ])

  assert.match(cmake, /GEA_BOARD_DISPLAY_WIDTH 640/)
  assert.match(cmake, /GEA_BOARD_DISPLAY_HEIGHT 180/)
  assert.match(cmake, /GEA_BOARD_NATIVE_DISPLAY_WIDTH 180/)
  assert.match(cmake, /GEA_BOARD_NATIVE_DISPLAY_HEIGHT 640/)
  assert.match(cmake, /GEA_BOARD_QSPI_PCLK_HZ 20000000/)
  assert.match(cmake, /GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY=1/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_ROTATE_LANDSCAPE=1/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION=1/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_DEFAULT_PORTRAIT_PRIMARY=0/)
  assert.match(cmake, /GEA_BOARD_FUSE_REPLAY_FLUSH 0/)
  assert.match(cmake, /GEA_EMBEDDED_FLUSH_UNITE_ALL=0/)
  assert.match(cmake, /GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD=60/)
  assert.match(cmake, /GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH=2/)
  assert.match(cmake, /GEA_BOARD_FRAMEBUFFER_CS_HELD_STREAM 0/)
  // -Os on purpose: the one-time large-partition migration image must fit the
  // original 2 MiB OTA layout (see the CMakeLists comment).
  assert.match(cmake, /GEA_BOARD_GEATSC_OPTIMIZATION -Os/)
  assert.match(cmake, /GEA_BOARD_OPTIMIZE_BOUNCING_BALLS_JSX 1/)
  assert.match(cmake, /GEA_EMBEDDED_AXS15231B_PANEL=1/)
  assert.match(cmake, /GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO=1/)

  for (const [field, gpio] of [
    ['cs', 12],
    ['pclk', 17],
    ['data0', 13],
    ['data1', 18],
    ['data2', 21],
    ['data3', 14],
    ['reset', 16]
  ]) {
    assert.match(board, new RegExp(`\\.${field} = GPIO_NUM_${gpio}`))
  }

  assert.match(manifest, /espressif\/esp_lcd_axs15231b: '\^2\.1\.0'/)
})

test('T-Display-S3 Long target reads both physical touch-controller revisions', async () => {
  const [board, cmake, touch] = await Promise.all([
    text('main/include/board.h'),
    text('main/CMakeLists.txt'),
    text('main/touch.cpp')
  ])

  assert.match(board, /\.reset = GPIO_NUM_2/)
  assert.match(board, /\.interrupt = GPIO_NUM_11/)
  assert.match(cmake, /"\$\{CMAKE_CURRENT_LIST_DIR\}\/touch\.cpp"/)
  assert.doesNotMatch(cmake, /m5stack-sticks3\/main\/touch\.cpp/)
  assert.match(touch, /kCst3530Address = 0x58/)
  assert.match(touch, /kAxs15231Address = 0x3b/)
  assert.match(touch, /xTaskCreateStaticPinnedToCore/)
  assert.match(touch, /gpio_isr_handler_add/)
  assert.match(touch, /Touchscreen::setPointerObserver/)
  assert.match(touch, /sample\.touch\.y = kNativeHeight -/)
})

test('the shared AMOLED CMakeLists carries the LILYGO native render fast path', async () => {
  const sharedCmake = await readFile(
    path.join(targetsRoot, 'targets/esp32-s3-touch-amoled-1.8/main/CMakeLists.txt'),
    'utf8'
  )

  assert.match(sharedCmake, /GEA_EMBEDDED_NUMBER_F32 1/)
  assert.match(sharedCmake, /GEA_NUMBER_FLOAT=1/)
  assert.match(sharedCmake, /GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH=\$\{GEA_EMBEDDED_FUSE_FLUSH_VALUE\}/)
  assert.match(sharedCmake, /GEA_EMBEDDED_SUBTREE_REVEAL_CHECK=\$\{GEA_EMBEDDED_REVEAL_CHECK_VALUE\}/)
  assert.match(sharedCmake, /GEA_EMBEDDED_SKIP_POSITION_INLINE_RECORD=\$\{GEA_EMBEDDED_SKIP_POSITION_RECORD_VALUE\}/)
  assert.match(sharedCmake, /GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM=\$\{GEA_EMBEDDED_FB_CS_HELD_VALUE\}/)
  assert.match(sharedCmake, /GEA_EMBEDDED_FB_CS_HELD_VALUE \$\{GEA_BOARD_FRAMEBUFFER_CS_HELD_STREAM\}/)
  assert.match(sharedCmake, /GEA_EMBEDDED_NUMBER_F32=\$\{GEA_EMBEDDED_NUMBER_F32\}/)
  // The other half of this fast path is the app's own
  // `Display.setFlushConfig({ rows: 64, depth: 2 })`. That line lives in
  // geastack/examples, a different repository: asserting it from here only
  // worked in a checkout that happened to have that sibling next door.
})

test('LILYGO orientation is selected at runtime through Display orientation state', async () => {
  const [appMain, display] = await Promise.all([
    readFile(
      path.join(targetsRoot, 'targets/esp32-s3-touch-amoled-1.8/main/app_main.cpp'),
      'utf8'
    ),
    readFile(path.join(targetsRoot, 'targets/esp32/display.cpp'), 'utf8')
  ])

  assert.match(appMain, /GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION/)
  assert.match(appMain, /"portrait-primary", "landscape-primary"/)
  assert.match(appMain, /GEA_EMBEDDED_DISPLAY_DEFAULT_PORTRAIT_PRIMARY/)
  assert.match(appMain, /setOrientation\("portrait-primary"\)/)
  assert.match(appMain, /setOrientation\("landscape-primary"\)/)
  assert.match(display, /softwareLandscapeActive\(\)/)
  assert.match(display, /bindCanvasesForActiveOrientation\(\)/)
  assert.match(display, /bindPixels\(frameBuffer_, width, height, stride\)/)
  assert.match(display, /bindPixelsRotatedLandscape/)
  assert.match(display, /DisplayOrientation::PortraitPrimary/)
  assert.match(display, /x0 = 0;\s*x1 = logicalDisplayWidth\(\) - 1;/)
  assert.match(display, /\{0, r\.y0, logicalDisplayWidth\(\) - 1, r\.y1\}/)
})

test('AXS15231B dirty rectangles program both address axes', async () => {
  const binding = await readFile(
    path.join(targetsRoot, 'targets/esp32/chip_bindings/displays/sh8601.cpp'),
    'utf8'
  )

  assert.match(binding, /GEA_EMBEDDED_AXS15231B_PANEL/)
  assert.match(binding, /setWindow\(x0, y0, x1Exclusive - 1, y1Exclusive - 1\)/)
  assert.match(binding, /txParam\(LCD_CMD_CASET/)
  assert.match(binding, /txParam\(LCD_CMD_RASET/)
  assert.match(binding, /txParam\(LCD_CMD_RAMWR, nullptr, 0\)/)
  assert.match(binding, /command = LCD_CMD_RAMWRC/)
  assert.match(binding, /\{LCD_CMD_SLPOUT, nullptr, 0, 120\}/)
  assert.match(binding, /kInterfacePixelFormat = 0x05/)
})

test('AXS15231B display power bypasses the component callback with inverted IDF 6 polarity', async () => {
  const binding = await readFile(
    path.join(targetsRoot, 'targets/esp32/chip_bindings/displays/sh8601.cpp'),
    'utf8'
  )

  assert.match(binding, /txParam\(LCD_CMD_DISPON, nullptr, 0\)/)
  assert.match(binding, /txParam\(enabled \? LCD_CMD_DISPON : LCD_CMD_DISPOFF, nullptr, 0\)/)
})

test('built-in target metadata exposes the LILYGO target', async () => {
  const targets = JSON.parse(
    await readFile(path.join(targetsRoot, 'targets.json'), 'utf8')
  )
  assert.deepEqual(targets['esp32-s3-lilygo-t-display-s3-long'], {
    adapter: 'esp32-idf',
    targetPath: 'targets/esp32-s3-lilygo-t-display-s3-long',
    flashSize: '16MB',
    appPlatform: 'esp32',
    idfTarget: 'esp32s3',
    esptoolChip: 'esp32s3'
  })
})
