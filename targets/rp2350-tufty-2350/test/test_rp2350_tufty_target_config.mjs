#!/usr/bin/env node
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'

const repoRoot = resolve(new URL('../../..', import.meta.url).pathname)

function source(path) {
  return readFileSync(resolve(repoRoot, path), 'utf8')
}

const targetMetadata = JSON.parse(source('targets.json'))
const boardHeader = source('targets/rp2350-tufty-2350/main/include/board.h')
const picoBoardHeader = source('targets/rp2350-tufty-2350/boards/pimoroni_tufty2350.h')
const cmake = source('targets/rp2350-tufty-2350/CMakeLists.txt')
const rp2350App = source('targets/rp2350-waveshare-touch-amoled-2.41/main/app.cpp')
const rp2350Platform = source('targets/rp2350-waveshare-touch-amoled-2.41/main/rp2350_gea_platform.cpp')
const panelHeader = source('targets/rp2350-tufty-2350/main/rp2350_panel.h')
const panel = source('targets/rp2350-tufty-2350/main/rp2350_panel.cpp')

assert.deepEqual(
  targetMetadata['rp2350-tufty-2350'],
  {
    adapter: 'rp2350-pico',
    targetPath: 'targets/rp2350-tufty-2350',
    flashSize: '16MB',
    appPlatform: 'rp2350',
    compatibleAppPlatforms: ['esp32']
  },
  'Tufty target should advertise its Pico SDK adapter metadata'
)

const expectedPins = [
  '.sda = 4',
  '.scl = 5',
  '.cs = 27',
  '.dc = 28',
  '.wr = 30',
  '.rd = 31',
  '.data0 = 32',
  '.backlight = 26',
  '.vsync = 21',
  '.powerKey = 14',
  '.powerHold = 41',
  '.batteryAdc = 40',
  '.batteryAdcChannel = 0'
]
for (const pin of expectedPins) {
  assert.match(boardHeader, new RegExp(pin.replace('.', '\\.')), `Tufty board.h should include ${pin}`)
}

assert.match(boardHeader, /320,\n\t    \.height = 240/, 'Tufty board should declare the 320x240 LCD')
assert.match(boardHeader, /8U \* 1024U \* 1024U/, 'Tufty board should declare 8 MB PSRAM')
assert.match(boardHeader, /kKeyCodeButtonA = 65/, 'Tufty Button A should use the keyboard A keyCode')
assert.match(boardHeader, /kKeyCodeButtonB = 66/, 'Tufty Button B should use the keyboard B keyCode')
assert.match(boardHeader, /kKeyCodeButtonC = 67/, 'Tufty Button C should use the keyboard C keyCode')
assert.match(boardHeader, /kKeyCodeArrowUp = 38/, 'Tufty Up should use ArrowUp')
assert.match(boardHeader, /kKeyCodeArrowDown = 40/, 'Tufty Down should use ArrowDown')
assert.match(boardHeader, /\.pin = 6, \.keyCode = kKeyCodeArrowDown/, 'Tufty Down button should be GP6')
assert.match(boardHeader, /\.pin = 7, \.keyCode = kKeyCodeButtonA/, 'Tufty A button should be GP7')
assert.match(boardHeader, /\.pin = 9, \.keyCode = kKeyCodeButtonB/, 'Tufty B button should be GP9')
assert.match(boardHeader, /\.pin = 10, \.keyCode = kKeyCodeButtonC/, 'Tufty C button should be GP10')
assert.match(boardHeader, /\.pin = 11, \.keyCode = kKeyCodeArrowUp/, 'Tufty Up button should be GP11')
assert.match(boardHeader, /\.pin = 22, \.keyCode = kKeyCodeEscape/, 'Tufty Home button should map to Escape on GP22')

assert.match(picoBoardHeader, /RP2350_XIP_CSI_PIN TUFTY_PSRAM_CS/, 'Pico board header should put PSRAM on Tufty CS')
assert.match(picoBoardHeader, /CYW43_DEFAULT_PIN_WL_REG_ON TUFTY_WL_ON/, 'Pico board header should describe CYW43 power pin')

assert.match(cmake, /PICO_BOARD pimoroni_tufty2350/, 'Tufty target should select the Pimoroni Pico board')
assert.match(cmake, /gea_rp2350_tufty_2350_bringup/, 'Tufty target should build a bring-up firmware')
assert.match(cmake, /gea_rp2350_tufty_2350_app/, 'Tufty target should build a Gea app firmware')
assert.match(cmake, /set\(GEA_RP2350_TUFTY_NATIVE_WIDTH 320\)/, 'Tufty target should keep the native panel width')
assert.match(cmake, /set\(GEA_RP2350_TUFTY_NATIVE_HEIGHT 240\)/, 'Tufty target should keep the native panel height')
assert.match(
  cmake,
  /GEA_EMBEDDED_APP STREQUAL "css-3d-cube"[\s\S]*set\(GEA_RP2350_TUFTY_PANEL_SCALE 1\)/,
  'Tufty target should build the CSS 3D cube app at native panel scale'
)
assert.match(
  cmake,
  /GEA_EMBEDDED_APP STREQUAL "css-3d-cube"[\s\S]*set\(GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO "1\.0"\)/,
  'Tufty target should build the CSS 3D cube app with native DPR'
)
assert.match(
  cmake,
  /set\(GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO "\$\{GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO\}"\)/,
  'Tufty target should keep the configured DPR for other apps'
)
assert.match(
  cmake,
  /math\(EXPR GEA_RP2350_TUFTY_DISPLAY_WIDTH "\$\{GEA_RP2350_TUFTY_NATIVE_WIDTH\} \/ \$\{GEA_RP2350_TUFTY_PANEL_SCALE\}"\)/,
  'Tufty target should derive logical width from native width and scale'
)
assert.match(
  cmake,
  /set\(GEA_RP2350_TUFTY_FRAMEBUFFER_IN_SRAM 1\)/,
  'Tufty target should keep the framebuffer in internal SRAM'
)
assert.match(cmake, /GEA_RP2350_HAS_TOUCH=0/, 'Tufty app build should disable touch startup')
assert.match(cmake, /GEA_RP2350_HAS_BUTTONS=1/, 'Tufty app build should enable button polling')
assert.match(cmake, /GEA_RP2350_STARTUP_TRACE=0/, 'Tufty app build should not wait for USB startup tracing')
assert.match(cmake, /GEA_RP2350_PERF_LOG=0/, 'Tufty app build should keep USB perf logging quiet by default')
assert.match(cmake, /GEA_RP2350_FRAME_PHASE_PERF=0/, 'Tufty app build should avoid frame-phase timer overhead by default')
assert.match(cmake, /GEA_RP2350_REFRESH_DETAIL_PERF=0/, 'Tufty app build should avoid refresh-detail timer overhead by default')
assert.match(cmake, /RP2350_XIP_CSI_PIN=8/, 'Tufty app build should initialize PSRAM on GP8')
assert.match(cmake, /--font-viewport-width \$\{GEA_RP2350_TUFTY_DISPLAY_WIDTH\}/, 'Tufty app build should compile fonts for logical width')
assert.match(cmake, /--font-viewport-height \$\{GEA_RP2350_TUFTY_DISPLAY_HEIGHT\}/, 'Tufty app build should compile fonts for logical height')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_WIDTH=\$\{GEA_RP2350_TUFTY_DISPLAY_WIDTH\}/, 'Tufty app display width should be logical')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_HEIGHT=\$\{GEA_RP2350_TUFTY_DISPLAY_HEIGHT\}/, 'Tufty app display height should be logical')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=\$\{GEA_RP2350_TUFTY_NATIVE_WIDTH\}/, 'Tufty app native width should remain 320px')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=\$\{GEA_RP2350_TUFTY_NATIVE_HEIGHT\}/, 'Tufty app native height should remain 240px')
assert.match(cmake, /--font-device-pixel-ratio \$\{GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO\}/, 'Tufty app font codegen should use the derived DPR')
assert.match(cmake, /GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=\$\{GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO\}/, 'Tufty app runtime should use the derived DPR')
assert.match(cmake, /GEA_RP2350_FRAMEBUFFER_IN_SRAM=\$\{GEA_RP2350_TUFTY_FRAMEBUFFER_IN_SRAM\}/, 'Tufty app framebuffer storage should follow the target SRAM policy')
assert.match(cmake, /GEA_RP2350_DEFAULT_BACKGROUND_CACHE_IN_SRAM=0/, 'Tufty app should leave the full-screen SRAM background cache disabled by default')
assert.match(cmake, /GEA_RP2350_DEFAULT_DISPLAY_COMMAND_BUFFER_COMMANDS=192/, 'Tufty app should keep a bounded 192-command display list arena in SRAM by default')
assert.match(cmake, /GEA_RP2350_EARLY_PSRAM_ALLOC=1/, 'Tufty app should route pre-main C++ allocations to PSRAM')
assert.match(cmake, /GEA_RP2350_PIE_BLEND=1/, 'Tufty app should enable the RP2350 translucent RGB565 blend hook')
assert.match(cmake, /GEA_RP2350_RENDER_CORE1=1/, 'Tufty app should enable the RP2350 core1 render worker')
assert.match(cmake, /GEA_RP2350_RENDER_CORE1_STACK_BYTES=12288/, 'Tufty app should reserve a larger SRAM stack for core1 render replay')
assert.match(cmake, /GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US=100000/, 'Tufty app should keep a recovery timeout around core1 render joins')
assert.match(cmake, /GEA_EMBEDDED_PIE_BLEND_SPAN8_GATHER=0/, 'Tufty app should avoid the ESP32-style gather scratch for translucent RGB565 blending')
assert.match(cmake, /GEA_EMBEDDED_TRANSFORMED_GRADIENT_A5_MIRROR=0/, 'Tufty app should omit the flat alpha mirror because cube faces use the constant-alpha hook')
assert.match(cmake, /GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_LOCK=1/, 'Tufty app should protect shared transformed-gradient LUT slots during core1 replay')
assert.match(cmake, /GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_BANKS=2/, 'Tufty app should give each render core its own transformed-gradient LUT bank')
assert.match(cmake, /GEA_EMBEDDED_RENDER_PARALLEL_DIRTY_REPLAY=0/, 'Tufty app should keep coarse dirty-region replay on one core until it is visually proven safe')
assert.match(cmake, /GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS=24/, 'Tufty app should keep the coarse dirty replay row threshold explicit')
assert.match(cmake, /GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS=6000/, 'Tufty app should keep the coarse dirty replay pixel threshold explicit')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_NODE_SCRATCH_IN_SRAM=1/, 'Tufty app should keep hot display-list node mirrors in SRAM')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_REPLAY_CLIP_STACK_IN_SRAM=1/, 'Tufty app should keep replay clip stack flags in SRAM')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_RECORD_CHILDREN_IN_SRAM=1/, 'Tufty app should keep record-time child scratch in SRAM')
assert.match(cmake, /GEA_EMBEDDED_RARE_STYLE_SRAM_POOL_ENTRIES=48/, 'Tufty app should keep the first RareStyle entries in a fixed SRAM pool')
assert.match(cmake, /GEA_EMBEDDED_RENDER_HOT_SRAM=1/, 'Tufty app should keep the translucent cube render/view/text hot path in SRAM')
assert.match(cmake, /GEA_EMBEDDED_PROJECTED_TEXT_CACHE_BANKS=2/, 'Tufty app should give each render core its own projected-text cache bank')
assert.match(cmake, /GEA_EMBEDDED_PROJECTED_TEXT_SRAM_CACHE_BYTES=12288/, 'Tufty app should reserve bounded SRAM for projected cube-label coverage buffers')
assert.match(cmake, /GEA_EMBEDDED_TEXT_SPRITE_CACHE=0/, 'Tufty app should avoid the dynamic large-text sprite realloc path')
assert.match(cmake, /GEA_EMBEDDED_UI_TRANSFORM_CACHE_SLOTS=64/, 'Tufty app should keep enough transform cache slots for animated 3D scenes')
assert.match(cmake, /GEA_EMBEDDED_UI_DEPTH_CACHE_SLOTS=64/, 'Tufty app should keep enough depth cache slots for animated 3D scenes')
assert.match(cmake, /GEA_EMBEDDED_UI_CORNER_CACHE_SLOTS=64/, 'Tufty app should keep enough corner cache slots for animated 3D scenes')
assert.match(cmake, /GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_SLOTS=8/, 'Tufty app should keep enough transformed gradient LUT slots for cube faces')
assert.match(cmake, /GEA_EMBEDDED_TRANSFORMED_GRADIENT_EDGE_CACHE=1/, 'Tufty app should precompute transformed-gradient edge slopes to avoid row-time divides')
assert.match(cmake, /GEA_EMBEDDED_RECOLOR_PIXEL_CACHE_SIZE=512/, 'Tufty app should keep the normal recolor cache size while full-screen SRAM caches are disabled')
assert.match(cmake, /GEA_EMBEDDED_LINE_BREAK_CACHE_SLOTS=4/, 'Tufty app should trim line-break cache slots')
assert.match(cmake, /GEA_EMBEDDED_LINE_BREAK_CACHE_LINES=32/, 'Tufty app should trim line-break cache lines')
assert.match(cmake, /GEA_EMBEDDED_CANVAS_CIRCLE_RADIUS_MAX=31/, 'Tufty app should trim canvas circle radius cache')
assert.match(cmake, /GEA_EMBEDDED_CANVAS_CIRCLE_BOX_SPAN_MAX=96/, 'Tufty app should trim canvas circle-box span cache width')
assert.match(cmake, /GEA_EMBEDDED_CANVAS_CIRCLE_BOX_SPAN_SLOTS=4/, 'Tufty app should trim canvas circle-box span cache slots')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH=0/, 'Tufty app should render into SRAM before panel scanout')
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_FUSE_INTERLEAVED_REPLAY_FLUSH=0/, 'Tufty app should avoid per-column fused replay')
assert.match(cmake, /GEA_RP2350_PANEL_SCALE=\$\{GEA_RP2350_TUFTY_PANEL_SCALE\}/, 'Tufty app should pass the panel scale to firmware')
assert.match(cmake, /PICO_CORE1_STACK_SIZE=4096/, 'Tufty app should reserve a bounded core1 render stack')
assert.match(cmake, /pico_multicore/, 'Tufty app should link the Pico SDK multicore worker support')

assert.match(rp2350Platform, /#ifndef GEA_RP2350_FRAMEBUFFER_IN_SRAM/, 'RP2350 platform should gate SRAM framebuffer use')
assert.match(rp2350Platform, /#ifndef GEA_RP2350_BACKGROUND_CACHE_IN_SRAM/, 'RP2350 platform should gate SRAM background-cache use')
assert.match(rp2350Platform, /__has_include\("gea_embedded_app_config\.h"\)/, 'RP2350 platform should consume generated app display-memory config')
assert.match(rp2350Platform, /GEA_EMBEDDED_DISPLAY_BACKGROUND_CACHE_IN_SRAM/, 'RP2350 platform should let JS Display.setMemoryConfig override background-cache storage')
assert.match(rp2350Platform, /GEA_EMBEDDED_DISPLAY_COMMAND_BUFFER_COMMANDS/, 'RP2350 platform should let JS Display.setMemoryConfig override command-buffer size')
assert.match(rp2350Platform, /gSramFramebuffer\[gea::rp2350::kPanelWidth \* gea::rp2350::kPanelHeight\]/, 'RP2350 platform should provide a panel-sized SRAM framebuffer')
assert.match(rp2350Platform, /gSramBackgroundCache\[gea::rp2350::kPanelWidth \* gea::rp2350::kPanelHeight\]/, 'RP2350 platform should provide a panel-sized SRAM background cache')
assert.match(rp2350Platform, /gSramDisplayCommandBuffer\[GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS\]/, 'RP2350 platform should provide a count-based SRAM command buffer')
assert.match(rp2350Platform, /gFramebuffer = gSramFramebuffer/, 'RP2350 platform should bind the framebuffer to SRAM when enabled')
assert.match(rp2350Platform, /extern "C" gea::framework::graphics::pixel::native_t \*gea_bg_cache\(int \*cap_px\)/, 'RP2350 platform should satisfy the renderer background-cache hook')
assert.match(rp2350Platform, /extern "C" void \*gea_display_command_buffer\(int \*cap_commands, int command_size, int command_align\)/, 'RP2350 platform should satisfy the renderer command-buffer hook')
assert.match(rp2350Platform, /gRenderWorkerCanvas/, 'RP2350 platform should provide a private canvas for core1 render replay')
assert.match(rp2350Platform, /#ifndef GEA_RP2350_RENDER_CORE1_STACK_BYTES/, 'RP2350 platform should gate the optional core1 render stack size')
assert.match(rp2350Platform, /#ifndef GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US/, 'RP2350 platform should gate the optional core1 render wait timeout')
assert.match(rp2350Platform, /gRenderWorkerStack\[GEA_RP2350_RENDER_CORE1_STACK_BYTES \/ sizeof\(std::uint32_t\)\]/, 'RP2350 platform should provide an optional SRAM stack for core1 render replay')
assert.match(rp2350Platform, /multicore_launch_core1_with_stack\(renderWorkerMain, gRenderWorkerStack, sizeof\(gRenderWorkerStack\)\)/, 'RP2350 platform should launch core1 render replay with the optional stack when enabled')
assert.match(rp2350Platform, /time_us_64\(\) - startUs >= GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US/, 'RP2350 platform should support an optional timeout for core1 render joins')
assert.match(rp2350Platform, /extern "C" bool gea_render_parallel_submit/, 'RP2350 platform should satisfy the renderer core1 submit hook')
assert.match(rp2350Platform, /extern "C" void gea_render_parallel_wait/, 'RP2350 platform should satisfy the renderer core1 wait hook')
assert.match(rp2350Platform, /extern "C" void gea_render_parallel_merge_dirty/, 'RP2350 platform should merge core1 dirty bounds back into the primary canvas')
assert.match(rp2350Platform, /gea::framework::graphics::Canvas \&activeDisplayCanvas/, 'RP2350 display calls should route through a per-core active canvas')
assert.match(rp2350Platform, /Display::canvas\(\) \{ return &activeDisplayCanvas\(\); \}/, 'RP2350 Display::canvas should return the calling core canvas')
assert.match(rp2350Platform, /extern "C" bool gea_rgb565_lut_blend_available\(\)/, 'RP2350 platform should satisfy the renderer direct RGB565 LUT blend availability hook')
assert.match(rp2350Platform, /extern "C" void __not_in_flash_func\(gea_rgb565_lut_blend_span\)/, 'RP2350 platform should keep the direct RGB565 LUT blend hook in SRAM')
assert.match(rp2350Platform, /extern "C" bool gea_rgb565_lut_blend_const_alpha_available\(\)/, 'RP2350 platform should satisfy the constant-alpha RGB565 LUT blend availability hook')
assert.match(rp2350Platform, /extern "C" void __not_in_flash_func\(gea_rgb565_lut_blend_const_alpha_span\)/, 'RP2350 platform should keep the constant-alpha RGB565 LUT blend hook in SRAM')
assert.match(rp2350Platform, /auto halfBlendRgb565[\s\S]*?0xF7DEu[\s\S]*?0x0821u/, 'RP2350 constant-alpha blend should keep the exact rounded half-alpha cube formula')
assert.match(rp2350Platform, /auto halfBlendRgb565Pair[\s\S]*?0xF7DEF7DEu[\s\S]*?0x08210821u/, 'RP2350 half-alpha cube blend should batch exact RGB565 averaging across two pixels')
assert.match(rp2350Platform, /loadPairNormal[\s\S]*?std::memcpy[\s\S]*?storePairNormal[\s\S]*?std::memcpy/, 'RP2350 packed half-alpha blend should use alias-safe two-pixel loads and stores')
assert.match(rp2350Platform, /while \(n >= 4\)[\s\S]*?storePairNormal\(dst \+ 2/, 'RP2350 packed half-alpha blend should unroll the dominant cube face path across four pixels')
assert.match(rp2350Platform, /if \(av == 16\)[\s\S]*?splitInsideBuckets/, 'RP2350 constant-alpha blend should specialize half-alpha cube faces with span splitting')
assert.match(rp2350Platform, /allocatePreferSpiram\(bytes, alignof\(Pixel\)\)/, 'RP2350 platform should keep PSRAM allocation as the default path')
assert.match(rp2350Platform, /#ifndef GEA_RP2350_EARLY_PSRAM_ALLOC/, 'RP2350 platform should gate early PSRAM allocation')
assert.match(rp2350Platform, /if \(void \*ptr = rp_mem_malloc\(size\)\) return \{ptr, true\}/, 'RP2350 platform should allow pre-main C++ allocations to land in PSRAM')
assert.match(rp2350Platform, /reset_usb_boot\(0, 0\)/, 'RP2350 platform should return to BOOTSEL on allocation failure instead of disappearing from USB')
assert.match(rp2350Platform, /PoolAllocationStats gSramAllocStats/, 'RP2350 platform should track internal-SRAM allocation stats')
assert.match(rp2350Platform, /PoolAllocationStats gPsramAllocStats/, 'RP2350 platform should track PSRAM allocation stats')
assert.match(rp2350Platform, /MemoryBackend::allocationSramCount\(\)/, 'RP2350 Memory API should expose SRAM allocation counts')
assert.match(rp2350Platform, /MemoryBackend::allocationPsramCount\(\)/, 'RP2350 Memory API should expose PSRAM allocation counts')
assert.match(rp2350Platform, /std::array<bool, board::buttons\.size\(\)> gButtonDown/, 'RP2350 platform should track Tufty button press state')
assert.match(rp2350Platform, /queueKeyDown\(button\.keyCode\)/, 'RP2350 button polling should queue mapped keydown events')
assert.match(rp2350Platform, /dispatchKeyInput\(\)/, 'RP2350 platform should dispatch queued hardware key events')
assert.match(rp2350App, /const std::uint64_t frameStartUs = time_us_64\(\)/, 'RP2350 app loop should measure frame work before sleeping')
assert.match(rp2350App, /gea\.rp2350\.alloc sram_allocs=%u psram_allocs=%u/, 'RP2350 app should log per-pool allocation counts')
assert.match(rp2350App, /frameBudgetUs - elapsedUs/, 'RP2350 app loop should sleep only the remaining frame budget')
assert.doesNotMatch(rp2350App, /sleep_ms\(intervalMs > 0 \? intervalMs : 1\)/, 'RP2350 app loop should not sleep a full frame after rendering')
assert.ok(
  rp2350App.indexOf('gea::rp2350::boardIoInit();') < rp2350App.indexOf('waitForUsbConsole();'),
  'RP2350 startup trace should assert Tufty power hold before waiting for USB'
)
// stdio_init_all() now runs FIRST, deliberately: it is a boot-debug beacon, so
// that a crash in the clock/board/PSRAM stages still reaches a console. What
// must stay ordered is the power hold before the boot blocks on that console --
// asserted above -- not the console init itself.
assert.ok(
  rp2350App.indexOf('stdio_init_all();') < rp2350App.indexOf('gea::rp2350::boardIoInit();'),
  'USB console comes up first so a failing boot stage still reports'
)

assert.match(panelHeader, /kNativePanelWidth = 320/, 'Tufty panel should expose native 320px width')
assert.match(panelHeader, /kNativePanelHeight = 240/, 'Tufty panel should expose native 240px height')
assert.match(panelHeader, /kPanelScale = GEA_RP2350_PANEL_SCALE/, 'Tufty panel should consume the compile-time scale')
assert.match(panelHeader, /kPanelWidth = kNativePanelWidth \/ kPanelScale/, 'Tufty panel logical width should derive from scale')
assert.match(panelHeader, /kPanelHeight = kNativePanelHeight \/ kPanelScale/, 'Tufty panel logical height should derive from scale')
assert.match(panel, /st7789_parallel\.pio\.h/, 'Tufty panel should use the ST7789 parallel PIO program')
assert.match(panel, /Reg::RamCtrl/, 'Tufty panel should apply the ST7789 RAMCTRL init')
assert.match(panel, /Reg::MadCtl/, 'Tufty panel should apply the ST7789 MADCTL orientation init')
assert.match(panel, /0x80 \| 0x20 \| 0x10/, 'Tufty panel should use native 320x240 MADCTL flags rotated 180 degrees in hardware')
assert.match(panel, /board::display\.data0/, 'Tufty panel should read its data bus from board composition')
assert.match(panel, /panelStreamRect/, 'Tufty panel should support rasterized streaming flushes')
assert.match(panel, /gLineA\[kNativePanelWidth\]/, 'Tufty panel DMA rows should be native width')
assert.match(panel, /gRasterLine\[kPanelWidth\]/, 'Tufty panel stream raster should use logical width')
assert.match(panel, /expandRowFromFramebuffer/, 'Tufty panel should expand framebuffer rows for lo-res modes')
assert.match(panel, /for \(int repeat = 0; repeat < kPanelScale; \+\+repeat\)/, 'Tufty panel should repeat pixels for scaled modes')
assert.match(panel, /beginPixels\(0, 0, kNativePanelWidth - 1, kNativePanelHeight - 1\)/, 'Tufty panel clear should cover the native panel')

