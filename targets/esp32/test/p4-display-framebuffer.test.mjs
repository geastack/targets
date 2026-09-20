import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const nativeWidth = 720
const nativeHeight = 1280
const framebufferPixels = nativeWidth * nativeHeight
const displaySource = readFileSync(
  new URL('../../../targets/esp32-p4-waveshare-touch-lcd-7/main/display.cpp', import.meta.url),
  'utf8'
)
const boardCmake = readFileSync(
  new URL('../../../targets/esp32-p4-waveshare-touch-lcd-7/main/CMakeLists.txt', import.meta.url),
  'utf8'
)
const sdkconfigDefaults = readFileSync(
  new URL('../../../targets/esp32-p4-waveshare-touch-lcd-7/sdkconfig.defaults', import.meta.url),
  'utf8'
)
const frameSchedulerSource = readFileSync(
  new URL('../../../targets/esp32/services/frame_scheduler.cpp', import.meta.url),
  'utf8'
)
// The UI layer moved out of targets/lib/gea-embedded into its own package; read
// it from the installed @geastack/engine the way the capability tests read
// @geastack/core, so this test tracks the sources the build actually compiles.
const engineUi = (file) => new URL(`../../../node_modules/@geastack/engine/ui/${file}`, import.meta.url)
const renderSource = readFileSync(engineUi('render.cpp'), 'utf8')
const treeRenderSource = readFileSync(engineUi('tree_render.cpp'), 'utf8')
const dirtyRegionsSource = readFileSync(engineUi('dirty_regions.h'), 'utf8')
const memoryConfigSource = readFileSync(
  new URL('../../../targets/esp32/memory_config.h', import.meta.url),
  'utf8'
)

function usedPixels(width, height, stride)
{
  return (height - 1) * stride + width
}

assert.equal(
  usedPixels(nativeWidth, nativeHeight, nativeHeight) <= framebufferPixels,
  false,
  'the previous portrait stride overran the single native framebuffer'
)

assert.equal(
  usedPixels(nativeWidth, nativeHeight, nativeWidth),
  framebufferPixels,
  'portrait should exactly fit one native framebuffer when stride follows logical width'
)

assert.equal(
  usedPixels(nativeHeight, nativeWidth, nativeHeight),
  framebufferPixels,
  'landscape should exactly fit one native framebuffer when stride follows logical width'
)

assert.match(
  displaySource,
  /const\s+int\s+stride\s*=\s*width\s*;/,
  'P4 canvas binding should use the active logical width as row stride'
)

// The panel is double-buffered now: num_fbs = kDpiFrameBufferCount, and both
// scanout buffers are claimed in one call so present() can flip between them.
assert.match(
  displaySource,
  /esp_lcd_dpi_panel_get_frame_buffer\(\s*panel_,\s*kDpiFrameBufferCount,\s*&fb0,\s*&fb1\s*\)/,
  'P4 display should retrieve every MIPI DPI scanout framebuffer from ESP-IDF'
)


assert.match(
  displaySource,
  /std::uint16_t\s+\*panelFrameBuffer_\s*=\s*nullptr\s*;/,
  'P4 display should keep the DPI panel framebuffer pointer separately from the rotation-safe owned framebuffer'
)

assert.match(
  displaySource,
  /std::uint16_t\s+\*panelFrameBuffers_\[2\]\s*=\s*\{nullptr,\s*nullptr\}\s*;/,
  'both scanout buffers must be retained so the flip can pick the non-scanned one'
)

// Composition NEVER targets the buffer the DSI is scanning out, in any
// orientation -- portrait included. Rasterizing straight into the live scanout
// buffer put the runtime's multi-millisecond raster inside the scan window and
// tore the whole frame; composing off-screen and PPA-blitting the dirty region
// leaves only a sub-millisecond hardware copy exposed. Pinned because "portrait
// can render directly, it's a no-op rotation" is the exact shortcut that
// reintroduces the tear.
const preferredFrameBuffer =
  displaySource.match(/std::uint16_t \*preferredFrameBuffer\(\) const\n\t\{[\s\S]*?\n\t\}/)?.[0] ?? ''
assert.ok(preferredFrameBuffer, 'preferredFrameBuffer() not found')
assert.match(
  preferredFrameBuffer,
  /return ownedFrameBuffer_;/,
  'P4 display should compose into an owned off-screen framebuffer'
)
assert.doesNotMatch(
  preferredFrameBuffer,
  /return panelFrameBuffer_;/,
  'composing into the scanned-out panel framebuffer reopens the whole-frame tear window'
)

assert.match(
  displaySource,
  /kWaveshareIli9881cInitCommands/,
  'P4 7-inch ILI9881C should use the Waveshare panel-specific init sequence'
)

assert.match(
  displaySource,
  /vendorConfig\.init_cmds\s*=\s*panelInitCommands\s*;/,
  'P4 display should pass the Waveshare init sequence to the ILI9881C driver'
)

assert.match(
  displaySource,
  /constexpr\s+pixel::Format\s+kFramebufferPixelFormat\s*=\s*pixel::Format::Rgb565\s*;/,
  'P4 runtime framebuffer should declare its renderer pixel format explicitly'
)

assert.match(
  displaySource,
  /dpiConfig\.in_color_format\s*=\s*lcdColorFormat\(\s*kFramebufferPixelFormat\s*\)\s*;/,
  'P4 DPI input should translate the runtime pixel format at the ESP-IDF boundary'
)

assert.match(
  displaySource,
  /constexpr\s+pixel::Format\s+kDsiOutputPixelFormat\s*=\s*pixel::Format::Rgb565\s*;/,
  'P4 DSI bridge should avoid RGB565-to-RGB888 conversion on the panel stream'
)

assert.match(
  displaySource,
  /dpiConfig\.out_color_format\s*=\s*lcdColorFormat\(\s*kDsiOutputPixelFormat\s*\)\s*;/,
  'P4 DPI output should translate the panel-facing pixel format at the ESP-IDF boundary'
)

assert.match(
  displaySource,
  /constexpr\s+int\s+kPanelBitsPerPixel\s*=\s*pixel::bitsPerPixel\(\s*kDsiOutputPixelFormat\s*\)\s*;/,
  'P4 ILI9881C control path should derive bit depth from the panel pixel format'
)

assert.match(
  displaySource,
  /panelConfig\.bits_per_pixel\s*=\s*kPanelBitsPerPixel\s*;/,
  'P4 ILI9881C driver should use the panel-facing bit depth'
)

// Two scanout buffers, not one. A single buffer cannot tear-free present a
// full-screen change, but two only work if the stale-content problem is solved
// explicitly: after a flip the newly scanned buffer holds the PREVIOUS frame,
// so the code must republish UI writes onto the buffer now being scanned rather
// than assuming a partial draw_bitmap landed everywhere. Both halves are pinned
// together -- raising the count without the follow-the-scanned-buffer step is
// what leaves stale frames visible.
assert.match(
  displaySource,
  /constexpr\s+int\s+kDpiFrameBufferCount\s*=\s*2\s*;/,
  'P4 DPI should double-buffer scanout so a full-screen present can flip instead of tearing'
)

assert.match(
  displaySource,
  /panelFrameBuffer_ = panelFrameBuffers_\[back\];/,
  'after a flip, UI writes must follow the buffer the DSI is now scanning'
)

assert.match(
  displaySource,
  /dpiConfig\.num_fbs\s*=\s*kDpiFrameBufferCount\s*;/,
  'P4 DPI framebuffer count should be an explicit target knob'
)

assert.match(
  displaySource,
  /std::uint16_t\s+\*backgroundCache_\s*=\s*nullptr\s*;/,
  'P4 display should provide a full-screen cache buffer for static background gradients'
)

assert.match(
  displaySource,
  /std::uint16_t\s+\*backdropCache_\s*=\s*nullptr\s*;/,
  'P4 display should provide a full-screen cache buffer for static backdrop baking'
)

assert.match(
  displaySource,
  /extern\s+"C"\s+std::uint16_t\s+\*gea_bg_cache\(int\s+\*cap_px\)[\s\S]*?backgroundCache\(cap_px\)/,
  'P4 display should satisfy the renderer background-cache hook'
)

assert.match(
  displaySource,
  /extern\s+"C"\s+std::uint16_t\s+\*gea_backdrop_cache\(int\s+\*cap_px\)[\s\S]*?backdropCache\(cap_px\)/,
  'P4 display should satisfy the renderer static-backdrop hook'
)

assert.match(
  displaySource,
  /gea::framework::graphics::Canvas\s+\*drawingCanvas\(\)[\s\S]*?DisplayBackend::instance\(\)\.drawingCanvas\(\)/,
  'P4 draw calls should use the currently bound replay canvas, including backdrop-cache bake buffers'
)

assert.match(
  displaySource,
  /gea::framework::graphics::Canvas\s+\*canvas\(\)[\s\S]*?return\s+&replayCanvas\(\)\s*;/,
  'P4 Display::canvas should expose the active replay canvas so transformed drawers and worker bands share the right target'
)

assert.match(
  displaySource,
  /usingPanelFrameBufferDirectly\(\)[\s\S]*?frameBuffer_\s*==\s*panelFrameBuffer_/,
  'P4 direct framebuffer detection should recognize the cached panel framebuffer'
)

assert.match(
  boardCmake,
  /esp_driver_ppa/,
  'P4 display target should link the ESP-IDF PPA driver for hardware rotations'
)

assert.match(
  displaySource,
  /#include\s+"driver\/ppa\.h"/,
  'P4 display should use the ESP-IDF PPA driver API directly'
)

assert.match(
  displaySource,
  /config\.oper_type\s*=\s*PPA_OPERATION_SRM[\s\S]*?ppa_register_client\(&config,\s*&ppaSrmClient_\)/,
  'P4 display should register a PPA SRM client for rotated presentation'
)

assert.match(
  displaySource,
  /ppa_do_scale_rotate_mirror\(/,
  'P4 display should rotate dirty logical rectangles into the DPI framebuffer with PPA SRM'
)

assert.doesNotMatch(
  displaySource,
  /esp_lcd_dpi_panel_enable_dma2d|flushBuffer_|presentClearFillCirclesFrameByChunks|presentRegionByChunks/,
  'P4 display should not copy through an internal DMA staging buffer or a chunked presenter'
)

// draw_bitmap is allowed in exactly one shape: handing back a pointer the DPI
// driver already owns, full-panel. That does no copy -- it cache-syncs the rows
// and swaps cur_fb_index at the next vblank. Any OTHER draw_bitmap (a sub-rect,
// or a pointer to memory the driver does not own) is the staging-copy presenter
// this board deliberately does not have, so pin the call count as well as the
// shape: an added second call would otherwise slip past a shape-only match.
const drawBitmapCalls = displaySource.match(/esp_lcd_panel_draw_bitmap\([^;]*\)/g) ?? []
assert.equal(drawBitmapCalls.length, 1, 'the only draw_bitmap on this board is the scanout flip')
assert.match(
  drawBitmapCalls[0],
  /esp_lcd_panel_draw_bitmap\(panel_, 0, 0, kNativeWidth, kNativeHeight, panelFrameBuffers_\[back\]\)/,
  'the flip must hand the driver one of its own full-size scanout buffers'
)

assert.match(
  displaySource,
  /constexpr\s+int\s+kFlags\s*=\s*ESP_CACHE_MSYNC_FLAG_DIR_C2M\s*\|[\s\S]*?ESP_CACHE_MSYNC_FLAG_TYPE_DATA\s*\|[\s\S]*?ESP_CACHE_MSYNC_FLAG_UNALIGNED[\s\S]*?esp_cache_msync\(\s*row\s*,\s*bytes\s*,\s*kFlags\s*\)/,
  'P4 direct DPI framebuffer updates should write CPU cache lines back before scanout reads PSRAM'
)

assert.match(
  displaySource,
  /presentClearFillCirclesFrame\(\s*current\s*\)/,
  'P4 clear+circle animation should draw into the active framebuffer and present by cache sync or PPA'
)

assert.match(
  displaySource,
  /usingPanelFrameBufferDirectly\(\)[\s\S]*?syncFrameBufferRectToMemory\(\s*logicalRect\s*\)[\s\S]*?return\s+ok\s*;/,
  'P4 retained/JSX dirty flushes should cache-sync the direct DPI scanout framebuffer instead of copying through the internal DMA buffer'
)

assert.doesNotMatch(
  displaySource,
  /cacheWritebackCircleBatchDirtyLines|dirtyCacheLineMasks_|panelCacheLinePixels_|CACHE_LL_L2MEM_NON_CACHE_ADDR|panelFrameBufferWrite_/,
  'P4 clear+circle animation should not split direct scanout into thousands of cache-line writebacks or render through the slow non-cacheable PSRAM alias'
)

// The fast path is now additionally gated on being WORTH taking -- an
// incremental circle redraw loses to a plain raster once the dirty area grows --
// so match the predicate, not the exact conjunction.
assert.match(
  displaySource,
  /if\s*\(isFastClearFillCirclesFrame\(current\)\s*&&\s*incrementalCirclePathBeneficial\(current\)\)/,
  'P4 present fast path should render directly into the active framebuffer when it is worth it'
)

assert.match(
  displaySource,
  /rasterPresentRegion\(current,\s*region\);\s*ok\s*=\s*flushNativeLogicalRect\(region\);/,
  'P4 generic present should raster into the active framebuffer and then flush/sync that framebuffer'
)

assert.doesNotMatch(
  displaySource,
  /preferPanelDma2dPresentBlit|rasterAndDma2dBlitPresentRegion|drawBitmap2dAndWaitForDma2d|esp_lcd_panel_draw_bitmap_2d|panelWriteFrameBuffer_/,
  'P4 portrait-primary present should not use the retired panel write-buffer path or old DMA2D staging helpers'
)

assert.match(
  displaySource,
  /rasterFrameRowsStrided\(/,
  'P4 generic present should raster partial-width regions directly into the logical framebuffer'
)

assert.match(
  sdkconfigDefaults,
  /^CONFIG_SPIRAM_SPEED_200M=y$/m,
  'P4 display scanout hits PSRAM directly, so the board should run PSRAM at 200MHz'
)

assert.match(
  displaySource,
  /if\s*\(!ownedFrameBuffer_\)\s*\{/,
  'P4 display initialization should require only the rotation-safe owned framebuffer besides the ESP-IDF panel framebuffer'
)

assert.doesNotMatch(
  displaySource,
  /presentBuffer_|allocatePresentDmaChunkBuffer|kFlushChunkBufferRows/,
  'P4 direct PSRAM scanout path should not copy through a separate internal present buffer'
)

assert.doesNotMatch(
  displaySource,
  /canRasterDirectly/,
  'P4 generic present should not need a compact-buffer/direct-width fork'
)

// The dirty region is now torus-split (a wrapped scroll can land a logical
// rectangle in up to four framebuffer places), so the raster target is computed
// per part rather than from region.x0/row. What must not change is that the
// target is an offset INTO frameBuffer_ at logicalStride_ -- i.e. the raster
// still writes straight into the active framebuffer with no staging copy.
assert.match(
  displaySource,
  /std::uint16_t \*rasterTarget =\s*\n\s*frameBuffer_ \+ static_cast<std::size_t>\(part\.torus\.y0 \+ r\) \* logicalStride_ \+ part\.torus\.x0;[\s\S]*?rasterFrameRowsStrided\(rasterTarget,/,
  'P4 generic present should raster every dirty region directly into the active framebuffer'
)

assert.match(
  displaySource,
  /void\s+Display::clip\(int\s+\*x0,\s*int\s+\*y0,\s*int\s+\*x1,\s*int\s+\*y1\)[\s\S]*?currentClip\(x0,\s*y0,\s*x1,\s*y1\)/,
  'P4 Display::clip should report the active canvas clip rather than the full logical viewport'
)

assert.match(
  displaySource,
  /extern\s+"C"\s+bool\s+gea_render_parallel_submit\([\s\S]*?setRenderCore\(static_cast<int>\(xPortGetCoreID\(\)\)\)/,
  'P4 display should provide the renderer parallel-submit hook and route worker drawing to a private canvas'
)

assert.match(
  displaySource,
  /extern\s+"C"\s+void\s+gea_render_parallel_merge_dirty\(\)[\s\S]*?absorbWorkerDirty\(\)/,
  'P4 display should merge worker-band dirty regions back into the main canvas'
)

assert.match(
  displaySource,
  /if\s*\(\s*previousPresentValid_\s*\)\s*\{[\s\S]*?present::dirtyRects[\s\S]*?\}\s*else\s*\{\s*regions\[0\]\s*=\s*\{0,\s*0,\s*logicalWidth\(\)\s*-\s*1,\s*logicalHeight\(\)\s*-\s*1\}\s*;/,
  'P4 display should force a full first present so a reset panel does not stay black behind dirty rects'
)

assert.match(
  displaySource,
  /bool\s+needsFullPhysicalFlush_\s*=\s*true\s*;/,
  'P4 display should remember that a reset panel needs one full physical flush'
)

assert.match(
  displaySource,
  /const\s+bool\s+fullPhysicalFlush\s*=\s*needsFullPhysicalFlush_;\s*[\s\S]*?dirtyRects\[0\]\s*=\s*\{0,\s*0,\s*logicalWidth\(\)\s*-\s*1,\s*logicalHeight\(\)\s*-\s*1\}\s*;/,
  'P4 Display::flush should bypass dirty rects for the first physical flush'
)

assert.match(
  displaySource,
  /if\s*\(\s*needsFullPhysicalFlush_\s*\)\s*\{[\s\S]*?flushNativeLogicalRect\(\s*\{0,\s*0,\s*logicalWidth\(\)\s*-\s*1,\s*logicalHeight\(\)\s*-\s*1\}\s*\)/,
  'P4 Display::flushRects should also turn the first partial flush into a full physical flush'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER=1/,
  'P4 should use the high-resolution esp_timer frame scheduler instead of FreeRTOS tick pacing'
)

assert.match(
  boardCmake,
  /\$\{GEA_CHIPS\}\/imu\/qmi8658\/qmi8658\.cpp/,
  'P4 should link the reusable QMI8658 driver so an attached IMU can provide accelerometer readings'
)

assert.match(
  boardCmake,
  /chip_bindings\/imu\/qmi8658\.cpp/,
  'P4 should use the ESP32 QMI8658 I2C binding instead of a hardcoded neutral accelerometer stub'
)

assert.doesNotMatch(
  boardCmake,
  /"accelerometer\.cpp"/,
  'P4 should not compile the board-local neutral accelerometer stub'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE=1/,
  'P4 perf bring-up should use compact scheduler logging so serial output does not distort frame cadence'
)

assert.match(
  renderSource,
  /#ifndef\s+GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS[\s\S]*?#define\s+GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS\s+96[\s\S]*?#ifndef\s+GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS[\s\S]*?#define\s+GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS\s+65536/,
  'renderer parallel replay thresholds should keep conservative framework defaults'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS=48[\s\S]*GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS=24576/,
  'P4 should lower coarse replay split thresholds for its large PSRAM-backed display'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD=60/,
  'P4 should yield after sustained catch-up frames so debug logging cannot starve the idle watchdog'
)

assert.match(
  dirtyRegionsSource,
  /#ifndef\s+GEA_EMBEDDED_DIRTY_REGION_MAX_RECTS[\s\S]*?#define\s+GEA_EMBEDDED_DIRTY_REGION_MAX_RECTS\s+32[\s\S]*static constexpr int kMaxRects\s*=\s*GEA_EMBEDDED_DIRTY_REGION_MAX_RECTS/,
  'dirty-region budget should keep a conservative framework default and be target-tunable'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_DIRTY_REGION_MAX_RECTS=64/,
  'P4 should use a larger dirty-region budget for precise rotated-leaf repainting on its large display'
)

assert.match(
  treeRenderSource,
  /#ifndef\s+GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS[\s\S]*?#define\s+GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS\s+1[\s\S]*flushX0\s*=\s*r->x0/,
  'tree renderer should preserve the CO5300 left-anchor workaround as the default while allowing targets without that artifact to flush true dirty x ranges'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS=0/,
  'P4 DSI panel should keep transformed dirty strip flushes narrow instead of applying the CO5300 left-anchor workaround'
)

assert.match(
  frameSchedulerSource,
  /#ifndef\s+GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER[\s\S]*?#define\s+GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER\s+0/,
  'ESP32 scheduler should keep esp_timer pacing behind an explicit board switch'
)

// Compact perf logging is opt-in per board. It carries no `#define ... 0`
// default any more; the `#if` guard in the scheduler compiles it out for every
// board that does not define it, and the P4 is the board that opts in. Assert
// both halves -- a shared header quietly defining it to 1 would turn serial
// spam back on for every existing board, which is what this test protects.
assert.doesNotMatch(
  memoryConfigSource,
  /#\s*define\s+GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE\s+1/,
  'ESP32 scheduler compact perf logging must not be turned on for every board'
)

assert.match(
  frameSchedulerSource,
  /#if\s+GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE/,
  'compact perf logging must be behind an #if so an undefined macro leaves it off'
)

assert.match(
  boardCmake,
  /GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE=1/,
  'the P4 board is the one that opts in to compact perf logging'
)

assert.match(
  frameSchedulerSource,
  /#if\s+GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE[\s\S]*?perf-lite:[\s\S]*?perf_\.reset\(\);\s*return;/,
  'ESP32 scheduler should have a compact perf branch for serial-sensitive board bring-up'
)

assert.match(
  frameSchedulerSource,
  /perf-lite:[\s\S]*replay=%dr[\s\S]*perf_\.treeReplayRegions\s*\/\s*perf_\.frameCount/,
  'ESP32 scheduler compact perf logging should include replay region count for dirty-region bring-up'
)

assert.match(
  frameSchedulerSource,
  /#ifndef\s+GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD[\s\S]*?#define\s+GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD\s+0[\s\S]*catchUpBurstFrames_/,
  'ESP32 scheduler catch-up watchdog guard should default off for existing boards'
)

assert.match(
  treeRenderSource,
  /hasThreeDimensionalTransformState[\s\S]*canUseStaticBackdropStripDirtyForTransformedLeaf[\s\S]*segmentReplayRegions[\s\S]*segmentReplayRegions\s*\?\s*segmentLimit\s*:\s*1[\s\S]*staticBackdropActive\(\)[\s\S]*canUseStaticBackdropStripDirtyForTransformedLeaf/,
  'tree renderer should keep 3D static-backdrop seam protection while allowing flat 2D rotated leaves to replay one bbox and flush segmented dirty regions'
)

assert.match(
  frameSchedulerSource,
  /if\s*\(\s*eventPending_\.load\([\s\S]*?catchUpFrameDue_\.store\(true,\s*std::memory_order_release\);[\s\S]*?timerPendingSkipCount_\.fetch_add/,
  'ESP32 scheduler should treat a timer tick that arrives behind a queued frame as catch-up debt'
)

assert.match(
  frameSchedulerSource,
  /esp_timer_start_periodic\(\s*timer_,\s*static_cast<uint64_t>\(intervalUs\)\s*\)/,
  'ESP32 scheduler should be able to post frames at microsecond intervals'
)
