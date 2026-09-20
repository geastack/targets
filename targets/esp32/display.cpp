#include "display.h"

#include "board.h"
#include "canvas.h"
#include "chip_bindings/displays/qspi_panel.h"
#include "display_present.h"
#include "graphics/font.h"
#include "memory_config.h"
#include "services/frame_scheduler.h" // setVSync → FrameScheduler::setVsyncDriven (TE single-clock)
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#include "host/display_orientation.h"
#endif

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "esp_async_memcpy.h"
#if __has_include("esp_cache.h")
#include "esp_cache.h"
#define GEA_EMBEDDED_DISPLAY_HAS_ESP_CACHE 1
#else
#define GEA_EMBEDDED_DISPLAY_HAS_ESP_CACHE 0
#endif
#include "esp_err.h"
#include "esp_heap_caps.h"
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
#include "esp_attr.h"
#include "esp_heap_trace.h"
#include "esp_memory_utils.h"
#include "esp_rom_sys.h"
#endif
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "ui/state_init.h"

#if GEA_BOARD_PREPARE_DISPLAY_PANEL
namespace gea::platform::board
{
  // Most boards route the panel reset to a GPIO (or leave it to power-on
  // reset) and the panel driver already owns it. A board whose reset sits
  // behind an I2C expander cannot say so through board::display.reset; it
  // defines GEA_BOARD_PREPARE_DISPLAY_PANEL and supplies this instead, and
  // display init calls it just before the panel takes its init sequence.
  // A weak default here would not work: defined in this same translation unit,
  // the call binds to it directly and the board's definition never gets a say.
  void prepareDisplayPanel();
}  // namespace gea::platform::board
#endif

namespace platform_display = gea::platform::display;
namespace pixel = gea::framework::graphics::pixel;
namespace qspi_panel = gea::platform::esp32::chip_bindings::displays;
namespace present = gea::framework::display_present;

// Cross-core raster: run a chunk's fill on the idle CPU (both the app frame task
// and the present task are pinned to APP_FRAME_TASK_CORE) while the present task
// streams the previous chunk. Defined later in this file. submit() returns false
// if the worker couldn't start, so callers must fall back to rastering inline.
extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1);
extern "C" void gea_render_parallel_wait();
extern "C" void gea_render_parallel_wait_blocking();

// gea3d full-frame continuous-DMA experiment: DISABLED. Measured on-device — a
// 411KB frame can't sit in internal SRAM (dma_capable) because the whole DRAM
// data segment is only 334KB (dram0_0_seg=0x53700); free internal was 95KB, the
// biggest contiguous DMA block 54KB. PSRAM holds it but isn't SPI-DMA-capable
// (dma_capable=0). So a full frame can't be streamed in one shot on this board.
constexpr bool kGea3dTryFullFramePsram = false;

namespace gea::platform::esp32::display
{

#ifndef GEA_EMBEDDED_DISPLAY_QSPI_PCLK_HZ
#define GEA_EMBEDDED_DISPLAY_QSPI_PCLK_HZ (80 * 1000 * 1000)
#endif
#ifndef GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH
#define GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH 32
#endif
#ifndef GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#define GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
#define GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION 0
#endif
// Stream-mode flush: set the panel window once, then send chunks via
// RAMWR/RAMWRC. Keep it under target-display control; app-generated display
// defaults may request it, but the CO5300 QSPI transport has shown panel-io
// stalls when polling parameter transactions mix with queued color transfers.
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
#define GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH
#define GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
#define GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_ALLOW_STREAM_FLUSH
#undef GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
#define GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH 0
#undef GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
#define GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE 0
#endif
// Drain DMA after every LCD chunk in the framebuffer rect path. This is a
// transport invariant, not a shape heuristic: queued rect updates can leave
// the next panel command blocked in esp_lcd_panel_draw_bitmap/tx_color even
// when the app heap and UI state are stable. The Display::present path keeps
// its end-of-region drain so canvas-style apps retain the old throughput.
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_RECT_PER_CHUNK_DRAIN
#define GEA_EMBEDDED_DISPLAY_CO5300_RECT_PER_CHUNK_DRAIN 1
#endif
// CS-held present stream: window+RAMWR framed ONCE per present region, then
// every chunk queues as a data-only continuation (CS stays low, no RAMWRC).
// The per-chunk txColor/drawBitmap paths send a command per chunk, and
// esp_lcd_panel_io_tx_color drains ALL in-flight DMA before any command —
// which serializes raster against the wire (measured ~18MB/s effective vs
// the 40MB/s link). With no commands mid-stream there is nothing to drain,
// so chunk raster overlaps the previous chunk's DMA. The historic CO5300
// wedge (commands issued behind in-flight color DMA) cannot occur: the only
// command is the opening RAMWR on an idle queue.
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_PRESENT_CS_HELD_STREAM
#define GEA_EMBEDDED_DISPLAY_CO5300_PRESENT_CS_HELD_STREAM 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM
#define GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
// TEMP MEASUREMENT (gea3d present-path work): revert to 0 after profiling.
#define GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG 1
#endif
#ifndef GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS
#define GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE 1
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGNMENT_BYTES
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGNMENT_BYTES 64
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_MAX_TRANSFER_BYTES
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_MAX_TRANSFER_BYTES 4032
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_EDGE_GUARD_BYTES
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_EDGE_GUARD_BYTES 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STAGED_PREFIX_ROWS
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STAGED_PREFIX_ROWS 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGN_TRANSFERS
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGN_TRANSFERS 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_REQUIRE_LENGTH_ALIGNMENT
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_REQUIRE_LENGTH_ALIGNMENT 1
#endif
#ifndef GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_PADDED_STRIDE
#define GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_PADDED_STRIDE 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_PADDED_ROW_STREAM
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_PADDED_ROW_STREAM 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE 0
#endif

#ifndef GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE_IDLE_FLUSH
#define GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE_IDLE_FLUSH 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_ASYNC_MEMCPY
#define GEA_EMBEDDED_DISPLAY_FLUSH_ASYNC_MEMCPY 0
#endif
#ifndef GEA_EMBEDDED_DISPLAY_ASYNC_MEMCPY_BURST_BYTES
#define GEA_EMBEDDED_DISPLAY_ASYNC_MEMCPY_BURST_BYTES 32
#endif
// DIAGNOSTIC: collapse all dirty flush windows into a single bounding box, so a
// frame flushes as one window's worth of row-chunks instead of N fragmented
// windows. Used to A/B whether per-chunk transaction overhead (not pixel
// bandwidth) dominates the flush for scattered-mover scenes. Default off.
#ifndef GEA_EMBEDDED_FLUSH_UNITE_ALL
#define GEA_EMBEDDED_FLUSH_UNITE_ALL 0
#endif
// Adaptive coalesce: collapse the dirty windows into their bounding box ONLY when
// they are both numerous and densely cover that box. Many small scattered windows
// flush at a poor per-pixel rate (per-chunk node-walk + DMA-slot overhead amortized
// over few pixels); merging them into one box flushes ~the same pixels far more
// efficiently. The density gate avoids over-flushing sparse scenes, where the box
// would be mostly unchanged gap pixels. Default off; tune via the MIN_WINDOWS /
// COVERAGE_PCT knobs below.
#ifndef GEA_EMBEDDED_FLUSH_ADAPTIVE_COALESCE
#define GEA_EMBEDDED_FLUSH_ADAPTIVE_COALESCE 0
#endif
#ifndef GEA_EMBEDDED_FLUSH_ADAPTIVE_MIN_WINDOWS
#define GEA_EMBEDDED_FLUSH_ADAPTIVE_MIN_WINDOWS 6
#endif
#ifndef GEA_EMBEDDED_FLUSH_ADAPTIVE_COVERAGE_PCT
#define GEA_EMBEDDED_FLUSH_ADAPTIVE_COVERAGE_PCT 25
#endif
#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_LIMIT
#if GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX > 32
#define GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_LIMIT GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX
#else
#define GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_LIMIT 32
#endif
#endif
#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH_LIMIT
#if GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH > 4
#define GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH_LIMIT GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH
#else
#define GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH_LIMIT 4
#endif
#endif

// Double-buffered framebuffer for the typography scroll path. Two PSRAM
// framebuffers; flush() swaps them so the next frame's recordNode + canvas
// writes can run while the just-drawn buffer is still being sent over SPI.
//
// Cost: +411 KB PSRAM (one extra full-screen buffer).
//
// Trade-off: apps that only update a small region per frame without calling
// Display::scrollRect or fully repainting will see stale pixels in untouched
// regions — the swapped-in draw buffer is two frames behind, so only regions
// written by canvas (or fed by cross-buffer scrollRect) get refreshed. Roll
// back via `git revert` if that becomes a problem.
#ifndef GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
#define GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER 1
#endif

// Software-scroll register: full-panel vertical scrollRect becomes a single
// integer update (scrollOffsetY_ += dy) instead of an H × W pixel copy. The
// canvas's rowToPhysical() does the logical → physical FB row translation
// on every pixel write; the flush staging copy uses the same translation
// when reading FB rows, so panel rows still receive contiguous content.
//
// Requires single-buffer mode (the per-frame scroll-offset accumulates on
// one persistent buffer — two buffers can't share a coherent offset across
// swaps without copying). When this flag is on we skip the back framebuffer
// and the async_memcpy install.
//
// Rollback via `git revert` or flip this define to 0.
#ifndef GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
#define GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER 1
#endif

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER && GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
#undef GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
#define GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER 0
#endif

  constexpr std::uint16_t kConsoleForeground = pixel::fromRgb565(0xFFFF);
  constexpr std::uint16_t kConsoleBackground = pixel::fromRgb565(0x0000);
  constexpr int kFlushChunkDefault = GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX;
  constexpr int kFlushChunkLimit = GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_LIMIT;
  // One scanline is the final correctness-preserving fallback. Normal builds
  // still start at the board's requested chunk/depth and only descend this far
  // when radios and recovery services leave a few KiB of DMA heap. A radio-heavy
  // AMOLED build reached the old 4-row floor with only 48 bytes free; allowing a
  // 1-row, 820-byte strip keeps the UI alive instead of parking the runtime.
  constexpr int kFlushChunkMin = 1;
  // Leave enough DMA-capable internal RAM for the WiFi driver to receive a
  // packet after the display pipeline is allocated. On the 2.06-inch S3 board,
  // accepting a 1-row, 2-deep pipeline consumed all but 80 bytes and made the
  // already-running OTA HTTP server unreachable. At this size a second row
  // buffer saves negligible wire time, while reserving even this small floor
  // lets the candidate search fall through to the 1-deep pipeline.
  constexpr std::size_t kFlushDmaReserveBytes = 768;
  constexpr int kFlushQueueDepthDefault = GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH;
  constexpr int kFlushQueueDepthLimit = GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH_LIMIT;
  // SPI transaction-queue depth is DECOUPLED from the flush-buffer count. A
  // full-screen present is ~7 chunks, and each 65.6KB chunk splits into 3 SPI
  // sub-transactions (the driver caps a transaction at 32KB), so ~21 are queued
  // per frame. With the old depth-4 pool the color-stream kick blocked in
  // get_trans_result after ~1.3 chunks, paying wire time SYNCHRONOUSLY (txkick
  // dominated the full-screen flush). A pool deep enough to hold a whole frame's
  // sub-transactions lets the kick return immediately, so the DMA streams in the
  // background and overlaps the next chunk's raster. Costs only descriptor RAM
  // (~sizeof(spi_transaction_t) each), NOT another 65.6KB flush buffer.
  constexpr int kSpiTransactionQueueDepth = GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH;
  constexpr int kQspiPclkHz = GEA_EMBEDDED_DISPLAY_QSPI_PCLK_HZ;
  constexpr int kPresentRectLimit = GEA_EMBEDDED_DISPLAY_PRESENT_RECT_LIMIT;
  constexpr std::size_t kFlushBufferMaxBytes = GEA_EMBEDDED_DISPLAY_FLUSH_BUFFER_MAX_BYTES;
  constexpr std::size_t kDirectPsramAlignmentBytes = GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGNMENT_BYTES;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
  constexpr int kFlushScanlineWidth = platform_display::kNativeWidth;
  static_assert(platform_display::kWidth == platform_display::kNativeHeight,
                "software landscape width must match the native panel height");
  static_assert(platform_display::kHeight == platform_display::kNativeWidth,
                "software landscape height must match the native panel width");
#else
  constexpr int kFlushScanlineWidth = platform_display::kWidth;
#endif
  constexpr int kPanelDefaultMaxTransferBytes = kFlushScanlineWidth * kFlushChunkLimit * sizeof(std::uint16_t);
  constexpr int kDirectPsramMaxTransferBytes = GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_MAX_TRANSFER_BYTES;
  constexpr int kDirectPsramStagedPrefixRows = GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STAGED_PREFIX_ROWS;
  constexpr std::size_t kDirectPsramEdgeGuardBytes = GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_EDGE_GUARD_BYTES;

  // Elastic-RAM arbiter: the staging pipeline is the big elastic consumer of
  // internal RAM. When connectivity (WiFi/BLE) needs internal RAM, it calls
  // Display::reserveInternal(bytes), which lowers g_flushBudgetBytes; the flush
  // pipeline then re-sizes its staging buffers down to fit (and back up when the
  // reservation is released). g_flushBudgetDirty is applied on the frame task at
  // the flush entry, AFTER in-flight DMA has drained, so it never races a live
  // flush. kFlushBufferMinBytes is the floor the bring-up shrink may reach. It is
  // ONE minimum-chunk staging buffer (1-deep, kFlushChunkMin rows) — the smallest
  // pipeline that can still flush. A 2-deep floor (the old 13 KB) preserves the
  // rasterize/DMA overlap during bring-up, but on a RAM-tight board (maps on the
  // amoled-2.06: a full-screen canvas leaves the display only an 8-row 2-deep
  // pipeline to begin with) that floor EQUALS the live staging, so reserveInternal
  // frees nothing and WiFi's task-stack + RX buffers never fit. Dropping to 1-deep
  // for the brief bring-up window releases one ~6.5 KB DMA-capable buffer — enough
  // for the WiFi RX buffers — and is restored (grows back) on reserveInternal(0).
  constexpr std::size_t kFlushBufferMinBytes =
      static_cast<std::size_t>(kFlushScanlineWidth) * kFlushChunkMin * sizeof(std::uint16_t);
  // Partial-flush window edge pad. The CO5300 drops a single edge pixel on
  // partial windows, covered by 2px. The SH8601 (amoled-1.8) drops a wider edge
  // and leaves visible residue/trails behind moving content, so it needs a
  // larger pad — made per-board tunable here. Keep the value EVEN so it preserves
  // the even-x0/odd-x1 alignment that clampAndAlign establishes.
// Partial-flush edge pad. Some QSPI AMOLED controllers drop a few-pixel strip at a
// partial window's edge (source-driver settling), so we over-flush the window by a pad
// onto already-correct neighbours. The pad is PER-AXIS: the SH8601 drops only its
// COLUMN (x) edges — its row (y) edges are clean (proven by the canvas full-width band,
// which has y edges mid-screen and never smears). So pad x by the column-drop width and
// leave y at 0. Padding y is pure waste on these panels. EDGE_PAD_PX sets both axes;
// EDGE_PAD_X_PX / EDGE_PAD_Y_PX override per-axis.
#ifndef GEA_EMBEDDED_DISPLAY_STATIC_BACKDROP
#define GEA_EMBEDDED_DISPLAY_STATIC_BACKDROP 1
#endif
#ifndef GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_PX
#define GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_PX 2
#endif
#ifndef GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_X_PX
#define GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_X_PX GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_PX
#endif
#ifndef GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_Y_PX
#define GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_Y_PX GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_PX
#endif
  constexpr int kEdgePadXPx = GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_X_PX;
  constexpr int kEdgePadYPx = GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_Y_PX;

// Full-width-strip mode: widen every dirty rect to the full panel width. Correct but
// expensive (re-flushes every background column in a dirty row) — kept for reference;
// the X-only pad above achieves the same smear-free result at a fraction of the pixels.
#ifndef GEA_EMBEDDED_DISPLAY_FLUSH_FULL_WIDTH_STRIPS
#define GEA_EMBEDDED_DISPLAY_FLUSH_FULL_WIDTH_STRIPS 0
#endif

// Software landscape: send only the native COLUMNS a damage box actually covers,
// instead of every column of the native rows it touches.
//
// The framebuffer is stored native-major -- frameBuffer_[(nativeHeight - 1 -
// logicalX) * nativeWidth + logicalY] -- so logical X selects the native ROW and
// logical Y the native COLUMN. The rotated flush derives its row band from logical
// X and then ignores logical Y, writing whole native rows because that makes each
// chunk one contiguous memcpy and one contiguous DMA. flushRects widens logical Y
// to the full height to match, which is why one chain dot costs 78,720 pixels:
// 192 native rows x all 410 columns, for ~600 pixels of real change. On a board
// where the UI shares its cores with hard real-time audio and gets ~5% of them,
// that inflation is the difference between a frame that lands in tens of
// milliseconds and one that takes half a second.
//
// Off by default: AXS15231B loses horizontal pixel alignment on partial-width
// writes (see flushFramebufferRect), and this axis is exactly that write. A board
// whose panel tolerates a narrow column window opts in; the full-width case still
// takes the single-memcpy path below, so nothing changes for a full-screen flush.
#ifndef GEA_EMBEDDED_DISPLAY_LANDSCAPE_COLUMN_WINDOWS
#define GEA_EMBEDDED_DISPLAY_LANDSCAPE_COLUMN_WINDOWS 0
#endif
  std::size_t g_flushBudgetBytes = kFlushBufferMaxBytes;
  volatile bool g_flushBudgetDirty = false;
#if GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES > 0
  // Pre-reserved flush pool (see memory_config.h). A static .bss array:
  // internal DRAM is DMA-capable on this chip and a static reservation is
  // immune to heap fragmentation by construction, so the exact configured
  // pipeline (e.g. 80 rows x 2-deep) always fits — no descent to a slow
  // small-chunk pipeline. Costs the bytes permanently; enable per board/app
  // profile via the CMake define only where the RAM is truly free (e.g. the
  // lean direct-canvas profile with radios off).
  alignas(64) std::uint16_t g_flushPool[GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES / sizeof(std::uint16_t)];
#endif
  static_assert(kDirectPsramAlignmentBytes > 0 &&
                    (kDirectPsramAlignmentBytes & (kDirectPsramAlignmentBytes - 1)) == 0,
                "direct PSRAM DMA alignment must be a power of two");
  static_assert(kDirectPsramMaxTransferBytes > 0,
                "direct PSRAM SPI transfer cap must be positive");
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_REQUIRE_LENGTH_ALIGNMENT
  static_assert((kDirectPsramMaxTransferBytes % static_cast<int>(kDirectPsramAlignmentBytes)) == 0,
                "direct PSRAM SPI transfer cap must preserve GDMA burst alignment");
#endif
  static_assert((kDirectPsramEdgeGuardBytes % kDirectPsramAlignmentBytes) == 0,
                "direct PSRAM edge guard must preserve GDMA burst alignment");
  static_assert(kDirectPsramStagedPrefixRows >= 0,
                "direct PSRAM staged prefix rows must be non-negative");
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH
  // Direct PSRAM transfer size is board-tuned. The default is conservative,
  // but boards can raise it to a row-aligned span while the SPI driver keeps
  // the internal GDMA descriptor splits aligned.
  constexpr int kPanelMaxTransferBytes =
      kPanelDefaultMaxTransferBytes > kDirectPsramMaxTransferBytes
          ? kDirectPsramMaxTransferBytes
          : kPanelDefaultMaxTransferBytes;
#else
  constexpr int kPanelMaxTransferBytes = kPanelDefaultMaxTransferBytes;
#endif

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER && !GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_PADDED_STRIDE
  // Software-scroll mode does not move full-width vertical scrolls through
  // GDMA, so keep the framebuffer packed. That lets full-width display flushes
  // copy contiguous spans instead of row-by-row slices from a padded source.
  constexpr int kFramebufferStridePx = platform_display::kWidth;
  constexpr const char *kFramebufferStrideMode = "packed";
#else
  // kWidth is 410 px, so a packed RGB565 row is 820 bytes. Padding the
  // framebuffer stride to 416 px makes each row start 832 bytes after the
  // previous row, i.e. exactly 13 x 64 bytes. The extra 6 pixels per row stay
  // in RAM; panel transfers still send only the visible 410 pixels.
  constexpr int kFramebufferStridePx = 416;
  static_assert((kFramebufferStridePx * sizeof(std::uint16_t)) % 64 == 0,
                "framebuffer row stride must be 64-byte aligned for GDMA burst=64");
  constexpr const char *kFramebufferStrideMode = "64-byte row-start aligned";
#endif
  static_assert(kFramebufferStridePx >= platform_display::kWidth,
                "framebuffer row stride must accommodate the full panel width");

  // Inclusive-coordinate overlap test (also true for exactly-touching windows).
  inline bool flushWindowsOverlap(const present::Rect &a, const present::Rect &b)
  {
    return a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1;
  }

  inline bool softwareLandscapeActive()
  {
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
    using Orientation = gea::framework::display::DisplayOrientation;
    return gea::framework::display::detail::DisplayOrientationState::orientation() ==
           Orientation::LandscapePrimary;
#elif GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
    return true;
#else
    return false;
#endif
  }

  inline int logicalDisplayWidth()
  {
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
    return gea::framework::display::detail::DisplayOrientationState::width();
#else
    return platform_display::kWidth;
#endif
  }

  inline int logicalDisplayHeight()
  {
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
    return gea::framework::display::detail::DisplayOrientationState::height();
#else
    return platform_display::kHeight;
#endif
  }

  inline int physicalFramebufferStridePixels()
  {
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
    return platform_display::kNativeWidth;
#else
    return kFramebufferStridePx;
#endif
  }

  present::Rect padPartialFlushWindowForCO5300(present::Rect rect)
  {
    const int width = logicalDisplayWidth();
    const int height = logicalDisplayHeight();
    rect.x0 = rect.x0 >= kEdgePadXPx ? rect.x0 - kEdgePadXPx : 0;
    rect.y0 = rect.y0 >= kEdgePadYPx ? rect.y0 - kEdgePadYPx : 0;
    rect.x1 = rect.x1 + kEdgePadXPx < width ? rect.x1 + kEdgePadXPx : width - 1;
    rect.y1 = rect.y1 + kEdgePadYPx < height ? rect.y1 + kEdgePadYPx : height - 1;
    return rect;
  }

  // Burst=64 panics in gdma_ll_tx_set_burst_size when async_memcpy is installed
  // after panel init (unexplained ESP-IDF v5.5.1 quirk on ESP32-S3). Keep the
  // value board-tunable: packed 410px rows need burst=4 to make every scroll
  // offset copy DMA-eligible, while padded double-buffer copies can use 32.
  constexpr std::size_t kAsyncMemcpyBurstBytes = GEA_EMBEDDED_DISPLAY_ASYNC_MEMCPY_BURST_BYTES;
  static_assert(kAsyncMemcpyBurstBytes > 0 &&
                    (kAsyncMemcpyBurstBytes & (kAsyncMemcpyBurstBytes - 1)) == 0,
                "async memcpy burst must be a power of two");

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
  constexpr std::size_t kDisplayInitHeapTraceRecords = 1024;
  constexpr std::size_t kDisplayInitHeapTracePrintMinBytes = 512;
  constexpr std::uintptr_t kDisplayInitHeapTraceFocusBegin = 0x3fcc0000;
  constexpr std::uintptr_t kDisplayInitHeapTraceFocusEnd = 0x3fce0000;

  EXT_RAM_BSS_ATTR heap_trace_record_t displayInitHeapTraceRecords[kDisplayInitHeapTraceRecords];
  bool displayInitHeapTraceReady = false;

  void printDisplayInitHeapTracePcList(const char *prefix, const void *const *pcs)
  {
    for (int i = 0; i < CONFIG_HEAP_TRACING_STACK_DEPTH; ++i)
    {
      if (!pcs[i])
        break;
      esp_rom_printf(" %s%d=0x%08x",
                     prefix,
                     i,
                     static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(pcs[i])));
    }
  }

  class DisplayInitHeapTraceScope
  {
  public:
    DisplayInitHeapTraceScope()
    {
      if (!displayInitHeapTraceReady)
      {
        const esp_err_t init = heap_trace_init_standalone(displayInitHeapTraceRecords, kDisplayInitHeapTraceRecords);
        displayInitHeapTraceReady = (init == ESP_OK);
        esp_rom_printf("[heap-trace] display_init init=%d records=%u\n",
                       static_cast<int>(init),
                       static_cast<unsigned>(kDisplayInitHeapTraceRecords));
      }
      if (!displayInitHeapTraceReady)
        return;
      const esp_err_t start = heap_trace_start(HEAP_TRACE_ALL);
      active_ = (start == ESP_OK);
      esp_rom_printf("[heap-trace] display_init start=%d\n", static_cast<int>(start));
    }

    ~DisplayInitHeapTraceScope()
    {
      if (!active_)
        return;
      const esp_err_t stop = heap_trace_stop();
      heap_trace_summary_t summary{};
      const esp_err_t summaryErr = heap_trace_summary(&summary);
      esp_rom_printf(
          "[heap-trace] display_init stop=%d summary=%d mode=%d allocs=%u frees=%u count=%u capacity=%u high=%u overflow=%u\n",
          static_cast<int>(stop),
          static_cast<int>(summaryErr),
          static_cast<int>(summary.mode),
          static_cast<unsigned>(summary.total_allocations),
          static_cast<unsigned>(summary.total_frees),
          static_cast<unsigned>(summary.count),
          static_cast<unsigned>(summary.capacity),
          static_cast<unsigned>(summary.high_water_mark),
          static_cast<unsigned>(summary.has_overflowed));

      std::uint32_t internalEvents = 0;
      std::uint32_t printedEvents = 0;
      const size_t count = heap_trace_get_count();
      for (size_t i = 0; i < count; ++i)
      {
        heap_trace_record_t record{};
        if (heap_trace_get(i, &record) != ESP_OK || !record.address)
          continue;
        if (!esp_ptr_internal(record.address))
          continue;
        internalEvents += 1;

        const auto address = reinterpret_cast<std::uintptr_t>(record.address);
        const bool inFocusRange = address >= kDisplayInitHeapTraceFocusBegin && address < kDisplayInitHeapTraceFocusEnd;
        if (record.size < kDisplayInitHeapTracePrintMinBytes && !inFocusRange)
          continue;

        printedEvents += 1;
        esp_rom_printf("[heap-event] scope=display_init ptr=0x%08x size=%u freed=%u",
                       static_cast<unsigned>(address),
                       static_cast<unsigned>(record.size),
                       record.freed ? 1U : 0U);
        printDisplayInitHeapTracePcList("pc", record.alloced_by);
        if (record.freed)
          printDisplayInitHeapTracePcList("free", record.freed_by);
        esp_rom_printf("\n");
      }
      esp_rom_printf("[heap-trace] display_init internal_events=%u printed=%u min_print=%u focus=0x%08x-0x%08x\n",
                     static_cast<unsigned>(internalEvents),
                     static_cast<unsigned>(printedEvents),
                     static_cast<unsigned>(kDisplayInitHeapTracePrintMinBytes),
                     static_cast<unsigned>(kDisplayInitHeapTraceFocusBegin),
                     static_cast<unsigned>(kDisplayInitHeapTraceFocusEnd));
    }

  private:
    bool active_ = false;
  };
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH
  constexpr const char *kFlushMode = "framebuffer-ramwrc";
#elif GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
#if GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
  constexpr const char *kFlushMode = "stream-ramwrc";
#else
  constexpr const char *kFlushMode = "stream-data";
#endif
#else
  constexpr const char *kFlushMode = "draw-bitmap";
#endif

  class DisplayBackend
  {
  public:
    static DisplayBackend &instance()
    {
      static DisplayBackend backend;
      return backend;
    }

    void flushStatsRead(int64_t *totalUs, int *callCount, int *pixelCount) const
    {
      if (totalUs)
        *totalUs = flushStats_.totalUs;
      if (callCount)
        *callCount = flushStats_.callCount;
      if (pixelCount)
        *pixelCount = flushStats_.pixelCount;
    }

    platform_display::DisplayFlushPerfStats flushPerfStatsRead() const
    {
      return flushStats_;
    }

    void flushStatsReset()
    {
      flushStats_ = {};
    }

    // Cumulative, never reset by the per-frame flushStatsReset above. The
    // per-frame counters cannot answer "has the panel stopped being painted
    // yet", because a poller sampling between frames sees them zeroed. These
    // only ever increase, so a reader can watch them go quiet.
    void flushOdometerRead(std::uint32_t &calls, std::uint64_t &pixels) const
    {
      calls = flushOdometerCalls_.load(std::memory_order_relaxed);
      pixels = flushOdometerPixels_.load(std::memory_order_relaxed);
    }

    const char *flushStageName() const
    {
      return stageName(flushStage_.load(std::memory_order_acquire));
    }

    int flushStageChunk() const
    {
      return flushStageChunk_.load(std::memory_order_acquire);
    }

    platform_display::DisplayFlushStageDetail flushStageDetail() const
    {
      return {
          flushStageX0_.load(std::memory_order_acquire),
          flushStageY0_.load(std::memory_order_acquire),
          flushStageX1_.load(std::memory_order_acquire),
          flushStageY1_.load(std::memory_order_acquire),
          flushStageChunk_.load(std::memory_order_acquire),
          flushStageRows_.load(std::memory_order_acquire),
          flushStagePixels_.load(std::memory_order_acquire),
      };
    }

    void resetClip() { replayCanvas().resetClip(); }
    void pushClip(int x, int y, int w, int h) { replayCanvas().pushClip(x, y, w, h); }
    void popClip() { replayCanvas().popClip(); }
    void clip(int *x0, int *y0, int *x1, int *y1) { replayCanvas().currentClip(x0, y0, x1, y1); }
    void setAlpha(std::uint8_t alpha) { replayCanvas().setGlobalAlpha(alpha); }
    std::uint8_t alpha() const { return onWorkerCore() ? workerCanvas_.globalAlpha() : canvas_.globalAlpha(); }

    // Coarse parallel replay: the renderer offloads the bottom row-band to a worker
    // pinned to the other CPU. The worker renders into its OWN canvas — the same
    // framebuffer pixels, but a private clip stack + dirty tracker — so the two cores
    // never race on clip/dirty state. Everything else (single-core, the panel flush,
    // console/fillRect helpers) keeps using the primary canvas_.
    bool onWorkerCore() const { return renderCoreId_ >= 0 && xPortGetCoreID() != renderCoreId_; }
    gea::framework::graphics::Canvas &replayCanvas() { return onWorkerCore() ? workerCanvas_ : canvas_; }
    void setRenderCore(int core) { renderCoreId_ = core; }
    // After a parallel band-replay joins, fold the worker canvas's dirty rects into
    // the primary so the flush (which reads canvas_) transmits both bands. These are
    // the same rects the single-core path would have produced, so the flush's per-row
    // dirty pattern is unchanged.
    void absorbWorkerDirty()
    {
      // The flush is bounding-box based (canvas_.dirty()), so fold the worker band's
      // dirty bbox into the primary via the public markDirty — same union the
      // single-core path's bbox would have accumulated.
      int x0, y0, x1, y1;
      if (workerCanvas_.dirty(&x0, &y0, &x1, &y1))
        canvas_.markDirty(x0, y0, x1, y1);
      workerCanvas_.resetDirty();
    }

    int brightness() const
    {
      return brightness_;
    }

    // Opt-in tearing sync. Off by default so a board that wires TE doesn't pay the
    // VBlank wait (or change its frame cadence) unless an app asks for it. bubble-grid
    // and other smooth-pan canvas apps call Display.setVSync(true); apps that want the
    // free-running (fastest, may tear) cadence call Display.setVSync(false) — e.g.
    // bouncing-balls-jsx, which runs the frame loop back-to-back and is NOT capped at
    // the panel's ~60Hz TE rate. When enabled, present() aligns each flush's GRAM write
    // to the panel's VBlank edge. TE IS routed on the AMOLED boards (e.g. amoled-2.06:
    // board.te = GPIO_NUM_13; setupTeVsync creates teSync_), so on those the only gate
    // is this opt-in; `on` is ANDed with (teSync_ != nullptr) purely to no-op on boards
    // that leave board.te = GPIO_NUM_NC (e.g. amoled-1.8). The flush stays on the app
    // frame task: the CO5300 serializes all panel SPI, so offloading the DMA to a
    // background task wedges the bus (verified on-device) — vsync here trades tearing
    // for cadence, not a parallel pipeline.
    void setVSync(bool on)
    {
      vsyncEnabled_ = on && (teSync_ != nullptr);
      // Single-clock: when TE-sync is active, the frame loop runs back-to-back so the
      // present's VBlank wait (the panel TE) is the one frame clock; otherwise the
      // scheduler's own timer would race the TE (two ~60Hz clocks → unstable pan fps).
      gea::framework::services::FrameScheduler::setVsyncDriven(vsyncEnabled_);
      // The TE edge is only a clock while vsync is on. Left armed it is a
      // ~60 Hz interrupt whose handler preempts whatever else runs on this
      // core, for nothing.
      if (teSync_)
      {
        const gpio_num_t te = gea::platform::board::display.te;
        if (vsyncEnabled_)
          gpio_intr_enable(te);
        else
          gpio_intr_disable(te);
      }
    }
    bool vsyncEnabled() const { return vsyncEnabled_; }

    // Damage-all: the next present treats the whole panel as changed and skips both
    // the dirtyRects compare and the persistent previous-frame copy (see present()).
    // Apps call Display.invalidate() once per frame while panning the whole screen.
    void invalidate() { invalidateRequested_ = true; }

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
    void bindCanvasesForActiveOrientation()
    {
      if (!frameBuffer_)
        return;
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      if (!softwareLandscapeActive())
      {
        const int width = logicalDisplayWidth();
        const int height = logicalDisplayHeight();
        const int stride = physicalFramebufferStridePixels();
        canvas_.bindPixels(frameBuffer_, width, height, stride);
        workerCanvas_.bindPixels(frameBuffer_, width, height, stride);
        return;
      }
#endif
      canvas_.bindPixelsRotatedLandscape(
          frameBuffer_, logicalDisplayWidth(), logicalDisplayHeight());
      workerCanvas_.bindPixelsRotatedLandscape(
          frameBuffer_, logicalDisplayWidth(), logicalDisplayHeight());
    }

    void applyOrientation(gea::framework::display::DisplayOrientation orientation)
    {
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      if (orientation != gea::framework::display::DisplayOrientation::LandscapePrimary &&
          orientation != gea::framework::display::DisplayOrientation::PortraitPrimary)
        return;
#else
      if (orientation != gea::framework::display::DisplayOrientation::LandscapePrimary)
        return;
#endif
      if (panel().initialized() && !waitForFlushCompleteSpin())
      {
        ESP_LOGE(kTag, "orientation switch timed out waiting for LCD DMA");
        return;
      }
      previousPresentValid_ = false;
      lastPanelUpdateWasPresent_ = false;
      externalPresentDamageCount_ = 0;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
      regionY_ = 0;
      regionH_ = 0;
      scrollOffsetY_ = 0;
#endif
      if (frameBuffer_)
      {
        bindCanvasesForActiveOrientation();
        std::memset(frameBuffer_, 0,
                    static_cast<std::size_t>(platform_display::kNativeWidth) *
                        platform_display::kNativeHeight * sizeof(std::uint16_t));
        canvas_.markDirty(0, 0, logicalDisplayWidth() - 1, logicalDisplayHeight() - 1);
      }
      ESP_LOGI(kTag,
               "Software orientation: %s logical=%dx%d native=%dx%d",
               gea::framework::display::detail::DisplayOrientationState::orientationString().c_str(),
               logicalDisplayWidth(),
               logicalDisplayHeight(),
               platform_display::kNativeWidth,
               platform_display::kNativeHeight);
    }
#endif

    // Kept as a no-harm API surface (declared in the platform Display facade); the
    // active vsync path waits inside present()'s flush. No-op unless enabled.
    void vsyncWaitForFrame()
    {
      if (vsyncEnabled_)
        waitForVsync();
    }

    void setBrightness(int brightnessPercent)
    {
      if (brightnessPercent < 0)
        brightnessPercent = 0;
      if (brightnessPercent > 100)
        brightnessPercent = 100;
      brightness_ = brightnessPercent;
      if (!panel().initialized())
        return;

      const esp_err_t err = panel().setBrightness(brightness_);
      if (err != ESP_OK)
      {
        ESP_LOGW(kTag, "Failed to set LCD brightness to %d%%: %s", brightness_, esp_err_to_name(err));
      }
    }

    // High-brightness mode. Separate from setBrightness(): brightness is a level
    // within the normal range, HBM switches the panel to a different, higher
    // ceiling. Kept as an explicit toggle rather than folded into a >100 percent
    // value so it can never be reached by accident from a settings slider.
    bool setHighBrightnessMode(bool enabled)
    {
      if (!panel().initialized())
        return false;
      const esp_err_t err = panel().setHighBrightnessMode(enabled);
      if (err != ESP_OK)
      {
        ESP_LOGW(kTag, "Failed to %s high-brightness mode: %s", enabled ? "enter" : "leave",
                 esp_err_to_name(err));
        return false;
      }
      highBrightnessMode_ = enabled;
      ESP_LOGI(kTag, "High-brightness mode %s", enabled ? "ON" : "OFF");
      return true;
    }
    bool highBrightnessMode() const { return highBrightnessMode_; }

    void setFlushConfig(int chunkRows, int queueDepth)
    {
      requestedFlushChunkRows_ = normalizeFlushChunkRows(chunkRows);
      requestedFlushQueueDepth_ = normalizeFlushQueueDepth(queueDepth);
      const esp_err_t err = configureFlushPipeline(requestedFlushChunkRows_, requestedFlushQueueDepth_);
      if (err != ESP_OK)
      {
        ESP_LOGE(kTag, "Failed to configure LCD flush pipeline: %s", esp_err_to_name(err));
      }
    }

    int flushChunkRows() const { return flushChunkRows_; }
    int flushQueueDepth() const { return flushQueueDepth_; }

    // Render-scale flag (1 = native, 2 = half-res + 2x upscale in presentRegion).
    // Per-present, not a compile switch: the present path is shared with full-res
    // apps, so a global flag would upscale their correct output. An app opts in by
    // rendering into a half-panel viewport and calling setPresentScale(2).
    void setPresentScale(int scale) { presentScale_ = scale >= 2 ? 2 : 1; }
    int presentScale() const { return presentScale_; }

    // Elastic-RAM arbiter entry point (callable from any task — e.g. the WiFi/BLE
    // bring-up worker). Reserve `bytes` of internal RAM by shrinking the staging
    // budget; the shrink is applied on the frame task at the next flush entry.
    // reserveInternal(0) releases the reservation (staging grows back).
    void reserveInternal(std::size_t bytes)
    {
      // Reserve against the RAM the staging actually holds, not the compile-time
      // cap: an app-configured pipeline (weather's 36-row/2-deep = 59 KB) sits far
      // below kFlushBufferMaxBytes, so a budget of (max - bytes) can be ABOVE the
      // live allocation and the "shrink" frees zero bytes — esp_wifi_init then
      // NO_MEMs with the reserve nominally applied.
      //
      // live == 0 means the staging pipeline is NOT YET allocated. The WiFi
      // bring-up worker can call this BEFORE the app's first flush allocates the
      // pipeline (maps: reserveInternal fires ~2 ms before the first staging
      // config). With live unknown the old code fell back to the 136 KB cap, so
      // (cap - bytes) = 80 KB landed far ABOVE the real ≤13 KB staging and froze
      // NOTHING — the staging then allocated full-size and esp_wifi_init NO_MEM'd
      // (the maps "still no tiles" path). When the size is unknown, drive the
      // budget straight to the floor: the not-yet-allocated pipeline then carves
      // at the floor (1-deep), and an already-allocated one shrinks to it. A
      // connectivity reservation only ever wants the staging at its floor for the
      // brief bring-up window; reserveInternal(0) releases it and the pipeline
      // grows back.
      const std::size_t live = static_cast<std::size_t>(flushBufferBytes());
      const std::size_t base = (live > 0 && live < kFlushBufferMaxBytes) ? live : kFlushBufferMaxBytes;
      const std::size_t budget = (bytes == 0)
                                     ? kFlushBufferMaxBytes
                                     : (live == 0 || bytes + kFlushBufferMinBytes >= base)
                                           ? kFlushBufferMinBytes
                                           : (base - bytes);
      if (budget != g_flushBudgetBytes)
      {
        g_flushBudgetBytes = budget;
        g_flushBudgetDirty = true;
      }
    }
    // Apply a pending reserveInternal() staging resize. MUST run on the frame task
    // (configureFlushPipeline tears down/reallocates staging and waits for DMA
    // idle internally). Called once per frame so the shrink lands even when the
    // app isn't flushing: a static screen never reaches flushFramebufferRectFrom's
    // budget check (it early-returns on an empty region), so without this a WiFi/BLE
    // bring-up reserve would never free its internal RAM -> esp_wifi_init NO_MEM.
    void applyReservation()
    {
      if (!g_flushBudgetDirty)
        return;
      g_flushBudgetDirty = false;
      configuredRequestFlushChunkRows_ = 0;
      configuredRequestFlushQueueDepth_ = 0;
      configureFlushPipeline(requestedFlushChunkRows_, requestedFlushQueueDepth_);
    }
    int flushBufferBytes() const
    {
      if (!flushBuffers_[0] || flushQueueDepth_ <= 0 || flushBufferCapacity_ <= 0)
        return 0;
      return flushBufferCapacity_ * flushQueueDepth_ * static_cast<int>(sizeof(std::uint16_t));
    }

    enum class DirectFlushResult : std::uint8_t
    {
      Done,
      Fallback,
      Failed,
    };

    bool flushFramebufferRect(int x0, int y0, int x1, int y1, bool allowPerChunkDrain = true,
                              platform_display::DisplayStreamRasterFn raster = nullptr, void *rasterUser = nullptr)
    {
      // waitAtEnd=false leaves tail DMA chunks in flight when flush() returns.
      // The next call's pre-wait inside flushFramebufferRectFrom drains them
      // before issuing setWindow, so the rAF callback of the next frame runs
      // in parallel with the prior frame's SPI drain. Net: ~6ms saved on
      // full-screen scrolling frames in single-buffer software-scroll mode.
      // Display::present() has its own entry-time drain (below) so a flush
      // tail can't collide with a subsequent canvas-direct present batch.
      return flushFramebufferRectFrom(frameBuffer_, x0, y0, x1, y1, /*waitAtEnd=*/false, /*allowDirect=*/true, allowPerChunkDrain, raster, rasterUser);
    }

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
    // The AXS15231B glass is physically 180x640 and has no reliable hardware
    // XY-swap mode. Canvas writes logical landscape pixels directly into native
    // physical row order, so flush only copies contiguous panel scanlines. Every
    // transfer spans all 180 native columns: partial-width GRAM writes are the
    // exact pattern that produced horizontal corruption on the connected board.
    bool flushFramebufferRectRotatedLandscapePrimary(const std::uint16_t *source,
                                                      int x0,
                                                      int y0,
                                                      int x1,
                                                      int y1,
                                                      bool waitAtEnd,
                                                      platform_display::DisplayStreamRasterFn raster)
    {
      if (raster)
      {
        ESP_LOGE(kTag, "Fused replay cannot feed rotated landscape scanout");
        return false;
      }

      const present::Rect logical = present::clampAndAlign(
          {x0, y0, x1, y1}, logicalDisplayWidth(), logicalDisplayHeight());
      if (!present::valid(logical))
        return true;

      lastPanelUpdateWasPresent_ = false;
      setFlushStage(FlushStage::WaitComplete, 0);
      const int64_t entryWaitStartUs = esp_timer_get_time();
      if (!waitForFlushCompleteSpin())
      {
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }
      {
        const int64_t waitUs = esp_timer_get_time() - entryWaitStartUs;
        flushStats_.completeWaitUs += waitUs;
        flushStats_.entryWaitUs += waitUs;
      }
      applyReservation();

      constexpr int nativeWidth = platform_display::kNativeWidth;
      constexpr int nativeHeight = platform_display::kNativeHeight;
      const int physicalY0 = nativeHeight - 1 - logical.x1;
      const int physicalY1 = nativeHeight - 1 - logical.x0;
#if GEA_EMBEDDED_DISPLAY_LANDSCAPE_COLUMN_WINDOWS
      const int colStart = logical.y0;
      const int colEnd = logical.y1;
#else
      const int colStart = 0;
      const int colEnd = nativeWidth - 1;
#endif
      const int colCount = colEnd - colStart + 1;

      // Program one full-scanline window for the entire native row band. Color
      // chunks continue with RAMWRC, avoiding repeated CASET/RASET while keeping
      // every command strictly serialized with the preceding DMA.
      setFlushStage(FlushStage::SetWindow, physicalY0);
      const int64_t setWindowStartUs = esp_timer_get_time();
      const esp_err_t windowErr =
          panel().setWindow(colStart, physicalY0, colEnd, physicalY1);
      flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
      if (windowErr != ESP_OK)
      {
        ESP_LOGE(kTag, "LCD rotated window setup failed: %s", esp_err_to_name(windowErr));
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }

      // Rows per chunk come from what the staging slot HOLDS, not from a row count
      // fixed for full-width transfers. A narrowed window makes each row colCount
      // pixels instead of nativeWidth, so the same buffer takes proportionally more
      // rows -- and rows per chunk is what sets the chunk COUNT, which is what the
      // per-chunk cost multiplies. Measured on a dot toggle under load: narrowing
      // the columns alone cut the pixels 14x (78,720 -> 5,568) but left 25 chunks
      // and moved inflush only 272ms -> 220ms, because setwin+tx are per chunk, not
      // per pixel. For a full-width window this expression is exactly
      // flushChunkRows_ (capacity == kFlushScanlineWidth * flushChunkRows_), so the
      // full-screen path is bit-for-bit unchanged.
      const int rotatedChunkRows = [&] {
        if (colCount <= 0)
          return flushChunkRows_;
        const int fits = flushBufferCapacity_ / colCount;
        return fits < 1 ? 1 : fits;
      }();

      auto rowsAt = [&](int row)
      {
        int rows = rotatedChunkRows;
        if (row + rows > physicalY1 + 1)
          rows = physicalY1 - row + 1;
        return rows;
      };

      auto nextBufferAndCopy = [&](int row, int rows) -> std::uint16_t *
      {
        const int pixelCount = colCount * rows;
        if (pixelCount > flushBufferCapacity_)
        {
          ESP_LOGE(kTag, "LCD rotated chunk too large: %d > %d", pixelCount, flushBufferCapacity_);
          return nullptr;
        }
        std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
        flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
        setFlushStage(FlushStage::Copy, row);
        const int64_t copyStartUs = esp_timer_get_time();
        // A full-column chunk is one contiguous run of native rows, so it stays a
        // single memcpy. A narrowed one is `rows` runs of colCount, packed tight
        // for the panel's auto-increment inside the programmed window.
        if (colCount == nativeWidth)
        {
#if GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
          std::memcpy(buffer,
                      &source[static_cast<std::size_t>(row) * nativeWidth],
                      static_cast<std::size_t>(pixelCount) * sizeof(std::uint16_t));
#else
          const std::uint16_t *sourcePixels =
              &source[static_cast<std::size_t>(row) * nativeWidth];
          for (int i = 0; i < pixelCount; ++i)
            buffer[i] = pixel::byteSwap16(sourcePixels[i]);
#endif
        }
        else
        {
          for (int r = 0; r < rows; ++r)
          {
            const std::uint16_t *sourceRow =
                &source[static_cast<std::size_t>(row + r) * nativeWidth + colStart];
            std::uint16_t *destRow = buffer + static_cast<std::size_t>(r) * colCount;
#if GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
            std::memcpy(destRow, sourceRow, static_cast<std::size_t>(colCount) * sizeof(std::uint16_t));
#else
            for (int i = 0; i < colCount; ++i)
              destRow[i] = pixel::byteSwap16(sourceRow[i]);
#endif
          }
        }
        flushStats_.copyUs += esp_timer_get_time() - copyStartUs;
        return buffer;
      };

      std::uint16_t *prefetchedBuffer = nullptr;
      int prefetchedRow = -1;
      int prefetchedRows = 0;
      bool firstChunk = true;
      for (int physicalRow = physicalY0; physicalRow <= physicalY1;)
      {
        const int rows = rowsAt(physicalRow);
        const int pixelCount = colCount * rows;
        setFlushDetail(colStart, physicalRow, colEnd, physicalRow + rows - 1, rows, pixelCount);

        std::uint16_t *buffer = nullptr;
        if (prefetchedBuffer && prefetchedRow == physicalRow && prefetchedRows == rows)
        {
          buffer = prefetchedBuffer;
          prefetchedBuffer = nullptr;
          prefetchedRow = -1;
          prefetchedRows = 0;
        }
        else
        {
          const int64_t slotWaitStartUs = esp_timer_get_time();
          setFlushStage(FlushStage::WaitSlot, physicalRow);
          bool gotSlot = xSemaphoreTake(flushSlots_, 0) == pdTRUE;
          while (!gotSlot && esp_timer_get_time() - slotWaitStartUs < 4000)
          {
            if (uxSemaphoreGetCount(flushSlots_) > 0)
              gotSlot = xSemaphoreTake(flushSlots_, 0) == pdTRUE;
          }
          if (!gotSlot && xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
          {
            ESP_LOGE(kTag, "LCD rotated draw timed out waiting for slot");
            setFlushStage(FlushStage::Idle, 0);
            return false;
          }
          flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;
          buffer = nextBufferAndCopy(physicalRow, rows);
          if (!buffer)
          {
            xSemaphoreGive(flushSlots_);
            setFlushStage(FlushStage::Idle, 0);
            return false;
          }
        }

        setFlushStage(FlushStage::Tx, physicalRow);
        const int64_t txStartUs = esp_timer_get_time();
        const esp_err_t err = panel().txColor(
            firstChunk ? LCD_CMD_RAMWR : LCD_CMD_RAMWRC,
            buffer,
            static_cast<std::size_t>(pixelCount) * sizeof(std::uint16_t));
        flushStats_.txUs += esp_timer_get_time() - txStartUs;
        flushStats_.chunkCount++;
        if (err != ESP_OK)
        {
          ESP_LOGE(kTag, "LCD rotated draw failed: %s", esp_err_to_name(err));
          xSemaphoreGive(flushSlots_);
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
        firstChunk = false;

        // While this chunk streams from internal DMA RAM, pull the next native
        // rows out of PSRAM into the other staging slot. We still wait before
        // issuing the next RAMWRC, so panel commands remain fully serialized.
        const int nextRow = physicalRow + rows;
        if (nextRow <= physicalY1 && flushQueueDepth_ > 1 &&
            xSemaphoreTake(flushSlots_, 0) == pdTRUE)
        {
          const int nextRows = rowsAt(nextRow);
          prefetchedBuffer = nextBufferAndCopy(nextRow, nextRows);
          if (prefetchedBuffer)
          {
            prefetchedRow = nextRow;
            prefetchedRows = nextRows;
          }
          else
          {
            xSemaphoreGive(flushSlots_);
          }
        }

        // The staging buffer is internal DMA RAM, independent of the native
        // PSRAM framebuffer. Let the final color DMA drain underneath the next
        // frame's JS/layout/raster work; the next flush entry waits before it
        // sends any panel command or reuses this slot.
        if (nextRow > physicalY1 && !waitAtEnd)
        {
          // This free-running animation otherwise keeps the high-priority frame
          // task continuously ready and starves IDLE0's watchdog feed. One 1 ms
          // tick runs while the tail DMA is on the wire, so it costs almost no
          // wall time and keeps the scheduler healthy.
          vTaskDelay(1);
          physicalRow = nextRow;
          continue;
        }

        setFlushStage(FlushStage::WaitComplete, physicalRow);
        const int64_t chunkWaitStartUs = esp_timer_get_time();
        bool chunkComplete = false;
        if (prefetchedBuffer)
        {
          // One slot is intentionally held by the prefetched chunk, so the
          // normal all-slots-free drain can never succeed here. Wait for the
          // current DMA's slot, then put it straight back for the next prefetch.
          while (!chunkComplete && esp_timer_get_time() - chunkWaitStartUs < 4000)
            chunkComplete = uxSemaphoreGetCount(flushSlots_) > 0;
          if (!chunkComplete && xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) == pdTRUE)
          {
            xSemaphoreGive(flushSlots_);
            chunkComplete = true;
          }
        }
        else
        {
          chunkComplete = waitForFlushCompleteSpin();
        }
        if (!chunkComplete)
        {
          if (prefetchedBuffer)
          {
            xSemaphoreGive(flushSlots_);
            prefetchedBuffer = nullptr;
          }
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
        const int64_t waitUs = esp_timer_get_time() - chunkWaitStartUs;
        flushStats_.completeWaitUs += waitUs;
        flushStats_.chunkWaitUs += waitUs;
        physicalRow = nextRow;
      }

      flushStats_.pixelCount += colCount * (physicalY1 - physicalY0 + 1);
      flushOdometerPixels_.fetch_add(static_cast<std::uint64_t>(colCount * (physicalY1 - physicalY0 + 1)), std::memory_order_relaxed);
      flushOdometerCalls_.fetch_add(1, std::memory_order_relaxed);
      setFlushStage(FlushStage::Idle, 0);
      return true;
    }
#endif

    bool flushFramebufferRectFrom(const std::uint16_t *source, int x0, int y0, int x1, int y1, bool waitAtEnd, bool allowDirect = true, bool allowPerChunkDrain = true,
                                  platform_display::DisplayStreamRasterFn raster = nullptr, void *rasterUser = nullptr)
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      if (softwareLandscapeActive())
#endif
      {
        (void)allowDirect;
        (void)allowPerChunkDrain;
        (void)rasterUser;
        return flushFramebufferRectRotatedLandscapePrimary(
            source, x0, y0, x1, y1, waitAtEnd, raster);
      }
#endif
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      // AXS15231B partial-width writes intermittently lose horizontal pixel
      // alignment. Landscape already projects every logical-X damage band to
      // a complete 180-pixel native scanline in the rotated path above.
      // Portrait must preserve the same hardware invariant: keep the dirty Y
      // band, but always transmit all native columns. Enforce it here as a
      // last line of defence for callers other than flushRects().
      x0 = 0;
      x1 = logicalDisplayWidth() - 1;
#endif
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
      if (allowDirect && directPsramStaticTxProbe_)
      {
        x0 = 0;
        y0 = 0;
        x1 = logicalDisplayWidth() - 1;
        y1 = logicalDisplayHeight() - 1;
      }
#endif
      const present::Rect region = present::clampAndAlign(
          {x0, y0, x1, y1}, logicalDisplayWidth(), logicalDisplayHeight());
      if (!present::valid(region))
        return true;
      // A framebuffer rect is about to reach the panel — its content is no
      // longer canvas-present-sourced. present() re-asserts the flag after
      // its regions transmit (they pass through here).
      lastPanelUpdateWasPresent_ = false;
      x0 = region.x0;
      y0 = region.y0;
      x1 = region.x1;
      y1 = region.y1;
      const int width = x1 - x0 + 1;
      const int totalRows = y1 - y0 + 1;

      // Drain any tail DMA left in flight by a prior flushFramebufferRectFrom
      // that returned with waitAtEnd=false. Cheap (semaphore takes return
      // immediately) when nothing is pending. Must run before setWindow so
      // command frames don't interleave with in-flight color DMAs. Spin (not block)
      // so the deferred-drain pipeline doesn't pay the cross-core wakeup latency per
      // rect — that's what capped the fused path at 35fps.
      setFlushStage(FlushStage::WaitComplete, 0);
      const int64_t entryWaitStartUs = esp_timer_get_time();
      if (!waitForFlushCompleteSpin())
      {
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }
      {
        const int64_t waitUs = esp_timer_get_time() - entryWaitStartUs;
        flushStats_.completeWaitUs += waitUs;
        flushStats_.entryWaitUs += waitUs;
      }

      // Elastic-RAM arbiter: a connectivity reserve/release changed the staging
      // budget. We are now past the in-flight-DMA drain and on the frame task, so
      // it is safe to tear down and re-allocate the staging pipeline to the new
      // budget here (configureFlushPipeline also waits for idle internally).
      applyReservation();
#if GEA_EMBEDDED_DISPLAY_CO5300_RECT_PER_CHUNK_DRAIN
      const bool drainPerChunk = allowPerChunkDrain && !canDeferFramebufferRectChunkDrain(x0, width, totalRows, waitAtEnd);
#else
      const bool drainPerChunk = false;
#endif

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH && GEA_EMBEDDED_PIXEL_PANEL_ENDIAN && GEA_EMBEDDED_DISPLAY_HAS_ESP_CACHE
      // The direct-from-PSRAM path DMAs the framebuffer itself; it can't be used when a
      // raster callback is generating each chunk in place (there is no source buffer).
      if (!raster && allowDirect && canDirectFramebufferFlush(source, x0, width, totalRows, waitAtEnd))
      {
        const std::uint16_t *directSource = directPsramSourceFor(source);
        const DirectFlushResult directResult = flushFramebufferRectDirect(directSource, x0, y0, x1, y1);
        if (directResult == DirectFlushResult::Done)
          return true;
        if (directResult == DirectFlushResult::Failed)
          return false;
      }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM
      bool csHeldStream = false;
      auto closeCsHeldStream = [&]()
      {
        if (csHeldStream)
        {
          panel().endColorStream();
          csHeldStream = false;
        }
      };
      if (canUseFramebufferColorStream(x0, width, totalRows, waitAtEnd))
      {
        setFlushStage(FlushStage::SetWindow, 0);
        const int64_t setWindowStartUs = esp_timer_get_time();
        csHeldStream = panel().setWindow(x0, y0, x1, y1) == ESP_OK &&
                       panel().beginColorStream(LCD_CMD_RAMWR) == ESP_OK;
        flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
        if (!csHeldStream)
          ESP_LOGW(kTag, "CO5300 framebuffer CS-held stream setup failed, falling back");
      }
#else
      auto closeCsHeldStream = []() {};
      constexpr bool csHeldStream = false;
#endif

#if (GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH || GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH) && \
    !GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM
      bool streamFlush = false;
      if (!csHeldStream && canUseFramebufferColorStream(x0, width, totalRows, waitAtEnd))
      {
#if GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
        setFlushStage(FlushStage::SetWindow, 0);
        const int64_t setWindowStartUs = esp_timer_get_time();
        streamFlush = panel().setWindow(x0, y0, x1, y1) == ESP_OK;
        flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
        if (!streamFlush)
          ESP_LOGW(kTag, "CO5300 framebuffer window setup failed, falling back to draw_bitmap");
#else
        streamFlush = openColorStream(x0, y0, x1, y1, "framebuffer");
#endif
      }
#endif

#if (GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH || GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH) && \
    GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
      int streamChunksSincePace = 0;
#endif
      for (int row = y0; row <= y1; row += flushChunkRows_)
      {
        int chunkRows = flushChunkRows_;
        if (row + chunkRows > y1 + 1)
          chunkRows = y1 - row + 1;
        if (chunkRows <= 0)
          break;

        const int pixelCount = width * chunkRows;
        setFlushDetail(x0, row, x1, row + chunkRows - 1, chunkRows, pixelCount);
        if (pixelCount > flushBufferCapacity_)
        {
          ESP_LOGE(kTag, "LCD chunk too large: %d > %d", pixelCount, flushBufferCapacity_);
          return false;
        }

        const int64_t slotWaitStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::WaitSlot, row);
        // Streaming paths (CS-held / RAMWRC continue) queue their chunk DMAs async and
        // recycle buffers as the completion ISR frees slots. That ISR runs on the SPI
        // host core while this frame task is pinned to APP_FRAME_TASK_CORE, so a BLOCKING
        // xSemaphoreTake here pays a ~700us cross-core reschedule per chunk (the frame
        // task sleeps and only wakes on the next scheduler tick, long after the DMA is
        // actually done) — measured ~6ms of "slot wait" across 14 chunks that is latency,
        // not wire time. Busy-poll the slot count instead: the real DMA frees a slot within
        // its wire time (~0.5ms/chunk) and we grab it immediately, no reschedule. Keep the
        // blocking take as a safety net if a chunk runs unexpectedly long. The draining
        // path (txColor blocks) always has a free slot, so this spin exits on the first read.
        bool gotSlot = (xSemaphoreTake(flushSlots_, 0) == pdTRUE);
        while (!gotSlot && (esp_timer_get_time() - slotWaitStartUs) < 4000)
        {
          if (uxSemaphoreGetCount(flushSlots_) > 0)
            gotSlot = (xSemaphoreTake(flushSlots_, 0) == pdTRUE);
        }
        if (!gotSlot && xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
          ESP_LOGE(kTag, "LCD draw timed out waiting for slot");
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
        flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

        std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
        flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
        setFlushStage(FlushStage::Copy, row);
        const int64_t copyStartUs = esp_timer_get_time();
        if (raster)
          // Fused path: rasterize this chunk's content directly into the DMA buffer
          // (no PSRAM framebuffer round-trip). The callback replays the dirty
          // display-list clipped to [x0,row]..[x1,row+chunkRows-1] into `buffer`.
          raster(buffer, width, chunkRows, x0, row, rasterUser);
        else
          copyFlushRows(buffer, source, x0, width, row, chunkRows);
        flushStats_.copyUs += esp_timer_get_time() - copyStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
        const int64_t byteSwapStartUs = esp_timer_get_time();
        for (int i = 0; i < pixelCount; i++)
        {
          buffer[i] = pixel::byteSwap16(buffer[i]);
        }
        flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

        const int64_t txStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::Tx, row);
#if GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM
        esp_err_t err;
        if (csHeldStream)
        {
          // Data-only continuation: no panel command here, so nothing drains the
          // SPI queue — this chunk's raster ran while the previous chunk's DMA
          // streamed. Mirrors the present CS-held path (queueColorStreamData).
          const bool lastChunk = row + chunkRows > y1;
          err = panel().queueColorStreamData(buffer,
                                             pixelCount * sizeof(std::uint16_t),
                                             /*keepCsActiveAfter=*/!lastChunk,
                                             /*signalCompletion=*/true);
        }
        else
        {
          err = panel().drawBitmap(x0, row, x1 + 1, row + chunkRows, buffer);
        }
#elif GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH || GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
        esp_err_t err;
        if (streamFlush)
        {
          const int command = row == y0 ? LCD_CMD_RAMWR : LCD_CMD_RAMWRC;
          err = panel().txColor(command, buffer, pixelCount * sizeof(std::uint16_t));
        }
        else
        {
          err = panel().drawBitmap(x0, row, x1 + 1, row + chunkRows, buffer);
        }
#else
        esp_err_t err = panel().drawBitmap(x0, row, x1 + 1, row + chunkRows, buffer);
#endif
        flushStats_.txUs += esp_timer_get_time() - txStartUs;
        flushStats_.chunkCount++;
        if (err != ESP_OK)
        {
          ESP_LOGE(kTag, "LCD draw failed: %s", esp_err_to_name(err));
          xSemaphoreGive(flushSlots_);
          closeCsHeldStream();
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }

#if (GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH || GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH) && \
    GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
        if (streamFlush)
        {
          streamChunksSincePace++;
          const int nextRow = row + chunkRows;
          if (streamChunksSincePace >= GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS && nextRow <= y1)
          {
            setFlushStage(FlushStage::WaitComplete, row);
            const int64_t paceWaitStartUs = esp_timer_get_time();
            if (!waitForFlushComplete())
            {
              setFlushStage(FlushStage::Idle, 0);
              return false;
            }
            flushStats_.completeWaitUs += esp_timer_get_time() - paceWaitStartUs;
            streamChunksSincePace = 0;
          }
        }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_RECT_PER_CHUNK_DRAIN
        if (drainPerChunk)
        {
          setFlushStage(FlushStage::WaitComplete, row);
          const int64_t chunkWaitStartUs = esp_timer_get_time();
          // Spin-wait for the chunk's DMA to complete rather than blocking on the
          // semaphore. A per-chunk drawBitmap DMA is tiny (a narrow dirty rect, ~35us
          // on the wire), but the completion ISR runs on the SPI host's core while the
          // frame task is pinned to APP_FRAME_TASK_CORE — so a BLOCKING xSemaphoreTake
          // here pays a cross-core reschedule that waits for the next scheduler tick
          // (~700us/chunk, measured). Across ~27 dirty-rect chunks/frame that turned a
          // ~1.6ms flush into ~20ms and dropped bouncing-balls from 60 to 28fps. Busy-
          // polling the slot count returns within the real DMA time (~35us). Fall back
          // to the blocking wait if the DMA runs long (keeps the cross-core safety net
          // for unexpectedly large chunks without burning a core indefinitely).
          bool drained = false;
          while (esp_timer_get_time() - chunkWaitStartUs < 2000)
          {
            if (uxSemaphoreGetCount(flushSlots_) >= static_cast<UBaseType_t>(flushQueueDepth_))
            {
              drained = true;
              break;
            }
          }
          if (!drained && !waitForFlushComplete())
          {
            setFlushStage(FlushStage::Idle, 0);
            return false;
          }
          {
            const int64_t waitUs = esp_timer_get_time() - chunkWaitStartUs;
            flushStats_.completeWaitUs += waitUs;
            flushStats_.chunkWaitUs += waitUs;
          }
        }
#endif
      }

      flushStats_.pixelCount += width * (y1 - y0 + 1);
      flushOdometerPixels_.fetch_add(static_cast<std::uint64_t>(width * (y1 - y0 + 1)), std::memory_order_relaxed);
      flushOdometerCalls_.fetch_add(1, std::memory_order_relaxed);

#if GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_CS_HELD_STREAM
      if (csHeldStream)
      {
        // Region tail: spin for the last queued chunks' DMA (cross-core rationale
        // as the present path), then drop the held bus. Must always drain+close
        // here — the CS-held stream can't survive across frames (deferring the close
        // to the next frame's entry, with the bus held across the gap, WEDGES the
        // panel — verified on-device 2026-07-08).
        setFlushStage(FlushStage::WaitComplete, y1);
        const int64_t waitStartUs = esp_timer_get_time();
        const bool ok = waitForFlushCompleteSpin();
        closeCsHeldStream();
        flushStats_.completeWaitUs += esp_timer_get_time() - waitStartUs;
        setFlushStage(FlushStage::Idle, 0);
        return ok;
      }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_RECT_PER_CHUNK_DRAIN
      if (drainPerChunk)
      {
        setFlushStage(FlushStage::Idle, 0);
        return true;
      }
#endif
      if (!waitAtEnd)
      {
        // Double-buffered path: tail DMAs are still draining. They'll be
        // awaited at the start of the next flush via waitForFlushComplete.
        setFlushStage(FlushStage::Idle, 0);
        return true;
      }

      setFlushStage(FlushStage::WaitComplete, y1);
      const int64_t waitStartUs = esp_timer_get_time();
      if (!waitForFlushComplete())
      {
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }
      {
        const int64_t waitUs = esp_timer_get_time() - waitStartUs;
        flushStats_.completeWaitUs += waitUs;
        flushStats_.tailWaitUs += waitUs;
      }
      setFlushStage(FlushStage::Idle, 0);
      return true;
    }

    void flush()
    {
      if (!panel().initialized() || !frameBuffer_ || !flushBuffers_[0] || !flushSlots_ || flushQueueDepth_ <= 0)
        return;

      int x0;
      int y0;
      int x1;
      int y1;
      bool forcedStaticTxProbeFlush = false;
      if (!canvas_.dirty(&x0, &y0, &x1, &y1))
      {
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE_IDLE_FLUSH
        if (!directPsramStaticTxProbe_)
          return;
        x0 = 0;
        y0 = 0;
        x1 = platform_display::kWidth - 1;
        y1 = platform_display::kHeight - 1;
        forcedStaticTxProbeFlush = true;
#else
        return;
#endif
      }

      // CSS animation dirty-region replay can call the single-rect flush()
      // path directly (instead of flushRects). Apply the same CO5300 edge
      // padding here so vertical position animations do not leave stale panel
      // rows at the dirty-window boundary.
      present::Rect padded = padPartialFlushWindowForCO5300({x0, y0, x1, y1});
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      if (!softwareLandscapeActive())
      {
        padded.x0 = 0;
        padded.x1 = logicalDisplayWidth() - 1;
      }
#elif GEA_EMBEDDED_DISPLAY_FLUSH_FULL_WIDTH_STRIPS
      // Same contract as flushRects: the panel drops partial-width column edges,
      // so a window that does not span every native column is not just wasteful,
      // it stalls. Widen here too, or the single-rect flush() path silently
      // bypasses the strip policy that flushRects applies.
      padded.x0 = 0;
      padded.x1 = logicalDisplayWidth() - 1;
#endif
      x0 = padded.x0;
      y0 = padded.y0;
      x1 = padded.x1;
      y1 = padded.y1;

      flushStats_.callCount++;
      const int64_t profileStartUs = esp_timer_get_time();

#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      bool ok;
      if (backFrameBuffer_)
      {
        // 1. Drain prev frame's still-in-flight tail SPI so the buffer that's
        //    about to become the new draw target is no longer being read.
        const int64_t prevWaitStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::WaitComplete, 0);
        if (!waitForFlushComplete())
        {
          setFlushStage(FlushStage::Idle, 0);
          flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
          return;
        }
        flushStats_.completeWaitUs += esp_timer_get_time() - prevWaitStartUs;

        // 2. Swap: just-drawn buffer becomes the SPI source; the previously-sent
        //    buffer becomes the canvas-bound draw target. Canvas rebind is
        //    cheap and only updates the pixel pointer.
        std::uint16_t *const justDrawn = frameBuffer_;
        frameBuffer_ = backFrameBuffer_;
        backFrameBuffer_ = justDrawn;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
        bindCanvasesForActiveOrientation();
#else
        canvas_.bindPixels(frameBuffer_, platform_display::kWidth, platform_display::kHeight, kFramebufferStridePx);
        // Keep the worker band-canvas pointed at the live draw buffer across swaps.
        workerCanvas_.bindPixels(frameBuffer_, platform_display::kWidth, platform_display::kHeight, kFramebufferStridePx);
#endif

        // 3. Kick async send from the just-drawn buffer (backFrameBuffer_).
        //    Don't wait at the end — the next flush() will drain at step 1.
        ok = flushFramebufferRectFrom(backFrameBuffer_, x0, y0, x1, y1, /*waitAtEnd=*/false, /*allowDirect=*/true, /*allowPerChunkDrain=*/false);
      }
      else
      {
        ok = flushFramebufferRect(x0, y0, x1, y1, /*allowPerChunkDrain=*/false);
      }
#else
      const bool ok = flushFramebufferRect(x0, y0, x1, y1, /*allowPerChunkDrain=*/false);
#endif

      if (ok && !forcedStaticTxProbeFlush)
      {
        canvas_.resetDirty();
        addPresentDamageRect(x0, y0, x1, y1);
      }
      flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
    }

    void flushRects(const platform_display::DisplayFlushRect *rects, int count, bool allowPerChunkDrain,
                    platform_display::DisplayStreamRasterFn raster = nullptr, void *rasterUser = nullptr)
    {
      if (!rects || count <= 0)
        return;
      if (!panel().initialized() || !frameBuffer_ || !flushBuffers_[0] || !flushSlots_ || flushQueueDepth_ <= 0)
        return;

      flushStats_.callCount++;
      const int64_t profileStartUs = esp_timer_get_time();

      // Build flush windows, THEN coalesce the ones that overlap. Two modes:
      //  - full-width strips (panels that drop partial-width column edges, e.g. SH8601):
      //    widen each dirty rect to the full panel width so there is no mid-screen
      //    column edge to drop, and no pad is needed. Strips that share rows merge.
      //  - padded windows (default): pad each window for the panel's edge drop, then
      //    merge. Padding old+new positions of a moving element makes their windows
      //    overlap; flushing each separately re-sends that overlap every frame
      //    (~2.5x pixel inflation). Merging flushes each pixel once. Pad-then-coalesce,
      //    not coalesce-then-pad, so the pad participates in the merge.
      constexpr int kFlushMergeCap = 32;  // == DirtyRegions::kMaxRects
      present::Rect win[kFlushMergeCap];
      int n = 0;
      for (int i = 0; i < count; i++)
      {
        const platform_display::DisplayFlushRect &r = rects[i];
        if (r.x0 > r.x1 || r.y0 > r.y1)
          continue;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
        // The logical X axis is the native panel's row axis. Project damage
        // onto that axis and merge it as 1-D row bands below. Each resulting
        // transfer still spans every native column (required by AXS15231B for
        // stable scanout), but unrelated X bands no longer inflate into a
        // nearly full-height native window just because their logical Y
        // positions differ.
        const present::Rect w =
            softwareLandscapeActive()
#if GEA_EMBEDDED_DISPLAY_LANDSCAPE_COLUMN_WINDOWS
                // Keep the real box on BOTH axes; the rotated flush now programs a
                // column window from logical Y instead of every native column. Pad
                // it the same way the non-rotated path does, so the panel's edge
                // drop cannot eat the boundary pixels.
                ? padPartialFlushWindowForCO5300({r.x0, r.y0, r.x1, r.y1})
#else
                ? present::Rect{r.x0, 0, r.x1, logicalDisplayHeight() - 1}
#endif
                : padPartialFlushWindowForCO5300(
                      {0, r.y0, logicalDisplayWidth() - 1, r.y1});
#elif GEA_EMBEDDED_DISPLAY_FLUSH_FULL_WIDTH_STRIPS
        const present::Rect w{0, r.y0, platform_display::kWidth - 1, r.y1};
#else
        const present::Rect w = padPartialFlushWindowForCO5300({r.x0, r.y0, r.x1, r.y1});
#endif
        if (n < kFlushMergeCap)
          win[n++] = w;
        else if (n > 0)
          win[n - 1] = present::unite(win[n - 1], w);  // overflow: fold into last
      }
      // Greedy merge of overlapping windows until stable (n <= 32, cheap).
      bool changed = true;
      while (changed)
      {
        changed = false;
        for (int i = 0; i < n && !changed; i++)
          for (int j = i + 1; j < n; j++)
            if (flushWindowsOverlap(win[i], win[j]))
            {
              win[i] = present::unite(win[i], win[j]);
              win[j] = win[n - 1];
              n--;
              changed = true;
              break;
            }
      }

#if GEA_EMBEDDED_FLUSH_ADAPTIVE_COALESCE
      // Adaptive: if there are many windows AND they densely fill their bounding
      // box, replace them with that single box. dense ⇒ merging adds few extra
      // (unchanged) pixels but slashes chunk count, so the per-chunk node-walk +
      // DMA-slot overhead drops while pixel volume barely grows. sparse ⇒ leave
      // the windows, since the box would be mostly unchanged gap pixels.
      if (n >= GEA_EMBEDDED_FLUSH_ADAPTIVE_MIN_WINDOWS)
      {
        present::Rect bbox = win[0];
        long sumArea = 0;
        for (int i = 0; i < n; i++)
        {
          bbox = present::unite(bbox, win[i]);
          sumArea += static_cast<long>(win[i].x1 - win[i].x0 + 1) * (win[i].y1 - win[i].y0 + 1);
        }
        const long bboxArea = static_cast<long>(bbox.x1 - bbox.x0 + 1) * (bbox.y1 - bbox.y0 + 1);
        if (bboxArea > 0 && sumArea * 100 >= static_cast<long>(GEA_EMBEDDED_FLUSH_ADAPTIVE_COVERAGE_PCT) * bboxArea)
        {
          win[0] = bbox;
          n = 1;
        }
      }
#endif

#if GEA_EMBEDDED_FLUSH_UNITE_ALL
      // DIAGNOSTIC: fold every window into one bounding box → one window, so the
      // flush issues only that window's row-chunks (min chunk count) regardless
      // of how scattered the dirty rects are.
      for (int i = 1; i < n; i++)
        win[0] = present::unite(win[0], win[i]);
      if (n > 1)
        n = 1;
#endif

      bool ok = true;
      for (int i = 0; i < n; i++)
      {
        if (!flushFramebufferRect(win[i].x0, win[i].y0, win[i].x1, win[i].y1, allowPerChunkDrain, raster, rasterUser))
        {
          ok = false;
          break;
        }
      }
      if (ok)
      {
        canvas_.resetDirty();
        for (int i = 0; i < n; i++)
          addPresentDamageRect(win[i].x0, win[i].y0, win[i].x1, win[i].y1);
      }
      flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
    }

    // Restore the draw canvas to the PSRAM framebuffer. The fused flush (flushRects
    // with a raster callback) rebinds the canvas onto each DMA chunk buffer; the
    // render layer calls this after the flush to put it back.
    void rebindCanvasToFramebuffer()
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
      bindCanvasesForActiveOrientation();
#else
      canvas_.bindPixels(frameBuffer_, platform_display::kWidth, platform_display::kHeight, kFramebufferStridePx);
#endif
    }

    bool present(const platform_display::DisplayPresentCommand *commands, int commandCount)
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
#if GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION
      // Runtime software orientation always keeps the retained framebuffer as
      // the source of truth. Portrait and landscape share one native-order
      // allocation, so replay through the actively bound Canvas in both modes.
#endif
      // Let CanvasRenderingContext2D replay this batch through the rotated
      // Canvas binding into the native-order framebuffer.
      // The direct present path rasterizes native row bands and therefore
      // cannot preserve landscape coordinates by itself.
      (void)commands;
      (void)commandCount;
      return false;
#endif
      if (!commands || commandCount <= 0)
        return false;
      if (!panel().initialized() || !flushBuffers_[0] || !flushSlots_ || flushQueueDepth_ <= 0)
        return false;

      present::Frame current;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      const int64_t splitExStart = esp_timer_get_time();
#endif
      const bool extracted = present::extractFrame(commands, commandCount, current);
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      splitExtractUs_ += esp_timer_get_time() - splitExStart;
#endif
      if (!extracted)
        return false;
      // Half-res present: the frame's opaque base covers only the half-panel
      // viewport it was rendered into; check against that, not the full panel.
      const int __obW = presentScale_ == 2 ? platform_display::kWidth / 2 : platform_display::kWidth;
      const int __obH = presentScale_ == 2 ? platform_display::kHeight / 2 : platform_display::kHeight;
      if (!present::frameHasOpaqueBase(current, __obW, __obH))
        return false;

      // Flush on the calling (app) frame task. When the app opted into tearing sync
      // (Display.setVSync(true)) the flush aligns its GRAM write to the panel's VBlank
      // (TE) edge; otherwise it flushes immediately (the pre-vsync behavior). The flush
      // MUST stay on this task: the CO5300 serializes all panel SPI (CS held across the
      // chunked DMA), so a background present task's DMA would wedge the bus against the
      // app frame's own residual panel access (a deadlock observed on-device).
      // Single-clock: NO mid-frame VBlank wait. When TE-sync is on, the frame was already
      // triggered by the TE edge (teEdgeIsr → frame post), so the GRAM write starts ~one
      // compute (~1.4ms) after the TE — inside the tear-scanline lead window — and races
      // ahead of scanout tear-free WITHOUT blocking here. Blocking for the NEXT TE would
      // double the period (→30fps); the TE trigger IS the alignment.
      return flushFrame(current, /*doVsyncWait=*/false);
    }

    // Flush one fully-extracted frame to the panel: drain the prior DMA tail, optionally
    // align the GRAM write to VBlank (TE), diff against the previous frame, collapse the
    // dirty regions, and transmit them. Runs on the app frame task (sync path) or on the
    // dedicated present task (vsync path). All present state it touches — previousPresentFrame_,
    // the flush slot pool, flushStats_ — is, in vsync mode, owned solely by the present task.
    bool flushFrame(present::Frame &current, bool doVsyncWait)
    {
      // Drain any tail DMA left in flight by a prior flushFramebufferRect()
      // that returned with waitAtEnd=false. Mirrors the entry-time drain
      // inside flushFramebufferRectFrom — without it, a canvas-direct app
      // (Display::present) that runs after JSX (Display::flush) sends its
      // first drawBitmap while the previous flush's color DMA is still on
      // the bus, and the panel ends up showing nothing.
      setFlushStage(FlushStage::WaitComplete, 0);
      const int64_t entryWaitStartUs = esp_timer_get_time();
      // Spin (not block) for the prior frame's tail DMA to drain. A blocking
      // xSemaphoreTake here is woken by the SPI-completion ISR on the SPI host's
      // core while this frame task is pinned to APP_FRAME_TASK_CORE, so it pays a
      // cross-core reschedule (~700us). Mirrors the JSX flush entry drain
      // (flushFramebufferRectFrom). The spin still waits for DMA-idle before the
      // first panel command, preserving the CO5300 TX-wedge serialization.
      if (!waitForFlushCompleteSpin())
      {
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      const int64_t splitDrainDelta = esp_timer_get_time() - entryWaitStartUs;
      flushStats_.completeWaitUs += splitDrainDelta;
      splitDrainUs_ += splitDrainDelta;
#else
      flushStats_.completeWaitUs += esp_timer_get_time() - entryWaitStartUs;
#endif

      // vsync: align the START of this frame's GRAM write to the panel's VBlank (TE)
      // edge so the write races ahead of scanout instead of tearing through it. On the
      // present task this sits immediately before the flush (no app compute in between),
      // so the TE→first-write lead window is preserved. No-op if TE isn't wired.
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      const int64_t splitVsyncStart = esp_timer_get_time();
#endif
      if (doVsyncWait)
        waitForVsync();
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      splitVsyncUs_ += esp_timer_get_time() - splitVsyncStart;
#endif

      present::Rect regions[kMaxPresentRects];
      // Damage-all (Display.invalidate(), set while panning): the change is full-screen,
      // so the dirtyRects compare would just conclude "all of it" and full-flush anyway —
      // and that diff is the SOLE reason the persistent previous-frame copy exists. Skip
      // both: flush the whole panel, store no copy. extractFrame already ran above (the
      // raster replays from it), so only the compare + copy are saved (~0.3ms). One
      // full-flush on settle (previousPresentValid_ stays false) then the cheap diff resumes.
      // Half-res present: the frame commands cover only the half-panel viewport,
      // so the dirty diff would flush just the top-left quarter. Force a full-panel
      // present (every chunk upscales its half-res rows) by skipping the diff.
      const bool skipDiff = invalidateRequested_ || presentScale_ == 2;
      invalidateRequested_ = false;
      int regionCount;
      if (skipDiff)
      {
        regions[0] = present::clampAndAlign(
            present::Rect{0, 0, platform_display::kWidth - 1, platform_display::kHeight - 1},
            platform_display::kWidth,
            platform_display::kHeight);
        regionCount = 1;
      }
      else
      {
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        const int64_t splitDiffStart = esp_timer_get_time();
#endif
        regionCount = present::dirtyRects(previousPresentValid_ ? &previousPresentFrame_ : nullptr,
                                          current,
                                          regions,
                                          kMaxPresentRects,
                                          platform_display::kWidth,
                                          platform_display::kHeight);
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        splitDiffUs_ += esp_timer_get_time() - splitDiffStart;
#endif
        for (int i = 0; i < externalPresentDamageCount_; i++)
        {
          present::addRect(regions,
                           &regionCount,
                           kMaxPresentRects,
                           externalPresentDamage_[i],
                           platform_display::kWidth,
                           platform_display::kHeight);
        }
      }
      // Collapse several scattered dirty regions into one full-width band when that's
      // cheaper than flushing them individually. The band is re-rasterized from the
      // frame's commands (rasterFrameRows handles every command type), so this is NOT
      // restricted to circles — any content rides along. present()'s entry guard already
      // requires a full-screen opaque base (frameHasOpaqueBase), which is the real
      // correctness precondition: it guarantees the band can be rebuilt from scratch.
      // The remaining decision is pure geometry, mirroring the JSX dirty-region path:
      // take the band only when there are several regions AND the band it merges into
      // wouldn't flush far more than the regions' real combined area (else we'd be
      // re-sending mostly-unchanged pixels — a loss for a few small scattered updates).
      if (regionCount >= 2)
      {
        long long sumArea = 0;
        int y0 = platform_display::kHeight;
        int y1 = -1;
        for (int i = 0; i < regionCount; i++)
        {
          if (!present::valid(regions[i]))
            continue;
          sumArea += present::area(regions[i]);
          if (regions[i].y0 < y0)
            y0 = regions[i].y0;
          if (regions[i].y1 > y1)
            y1 = regions[i].y1;
        }
        const long long bandArea =
            static_cast<long long>(platform_display::kWidth) * (y1 - y0 + 1);
        if (y0 <= y1 && sumArea > 0 && bandArea <= sumArea * 3)
        {
          regions[0] = present::clampAndAlign(
              present::Rect{0, y0, platform_display::kWidth - 1, y1},
              platform_display::kWidth,
              platform_display::kHeight);
          regionCount = 1;
        }
      }
      if (regionCount <= 0)
      {
        previousPresentFrame_ = std::move(current);
        previousPresentValid_ = true;
        lastPanelUpdateWasPresent_ = true;
        externalPresentDamageCount_ = 0;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        accountPresentSplit(0, 0); // no-op frame (nothing changed) — still counts toward the window
#endif
        return true;
      }

      long long flushedPx = 0;
      for (int i = 0; i < regionCount; i++)
        flushedPx += present::area(regions[i]);

      flushStats_.callCount++;
      const int64_t profileStartUs = esp_timer_get_time();
      bool ok = true;
      for (int i = 0; i < regionCount; i++)
      {
        if (!presentRegion(regions[i], current))
        {
          ok = false;
          break;
        }
      }
      if (ok)
      {
        canvas_.resetDirty();
        if (skipDiff)
        {
          // Damage-all: store no previous-frame copy (the saving). Next present has no
          // baseline → it full-flushes once on settle, then the diff resumes from there.
          previousPresentValid_ = false;
        }
        else
        {
          previousPresentFrame_ = std::move(current);
          previousPresentValid_ = true;
        }
        // Set AFTER presentRegion's transmits: they route through
        // flushFramebufferRectFrom, which clears the flag.
        lastPanelUpdateWasPresent_ = true;
        externalPresentDamageCount_ = 0;
      }
      const int64_t flushUs = esp_timer_get_time() - profileStartUs;
      flushStats_.totalUs += flushUs;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      accountPresentSplit(flushUs, flushedPx);
#endif
      return ok;
    }

    // [present.split] diagnostic: accumulate the present-path timing breakdown and dump
    // a per-frame average every 30 frames. extract/diff/flush are real work; drain/vsync
    // are waits. JS app compute is NOT here (it runs in the app callback before present) —
    // derive it as appnoflush (main perf line) minus extract+diff+drain+vsync.
    void accountPresentSplit(int64_t flushUs, long long flushedPx)
    {
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      splitFlushUs_ += flushUs;
      splitPx_ += flushedPx;
      if (++splitCount_ < 30)
        return;
      std::printf("[present.split] n=%d extract=%lldus diff=%lldus drain=%lldus vsync=%lldus flush=%lldus px=%lld | work(ex+diff+flush)=%lldus idle(drain+vsync)=%lldus\n"
                  "[flush.split]   raster: bg(fillRect)=%lldus(%d) circles(fillCircle)=%lldus(%d) | txkick=%lldus(%d) slotwait=%lldus tail=%lldus window=%lldus | region x0=%d w=%d(of %d) rows=%d drainPerChunk=%d [avg/frame over n=%d]\n",
                  splitCount_,
                  splitExtractUs_ / splitCount_, splitDiffUs_ / splitCount_,
                  splitDrainUs_ / splitCount_, splitVsyncUs_ / splitCount_,
                  splitFlushUs_ / splitCount_, splitPx_ / splitCount_,
                  (splitExtractUs_ + splitDiffUs_ + splitFlushUs_) / splitCount_,
                  (splitDrainUs_ + splitVsyncUs_) / splitCount_,
                  splitRasterClearUs_ / splitCount_, splitRasterClearN_ / splitCount_,
                  splitRasterFillUs_ / splitCount_, splitRasterFillN_ / splitCount_,
                  splitTxKickUs_ / splitCount_, splitTxKickN_ / splitCount_,
                  splitSlotWaitUs_ / splitCount_, splitCompleteWaitUs_ / splitCount_,
                  splitWindowUs_ / splitCount_,
                  splitRegionX0_, splitRegionW_, platform_display::kWidth, splitRegionRows_, splitDrainChunk_,
                  splitCount_);
      splitExtractUs_ = splitDiffUs_ = splitDrainUs_ = splitVsyncUs_ = splitFlushUs_ = 0;
      splitRasterClearUs_ = splitRasterFillUs_ = splitTxKickUs_ = 0;
      splitRasterClearN_ = splitRasterFillN_ = splitTxKickN_ = 0;
      splitSlotWaitUs_ = splitCompleteWaitUs_ = splitWindowUs_ = 0;
      splitPx_ = 0;
      splitCount_ = 0;
#else
      (void)flushUs;
      (void)flushedPx;
#endif
    }

    bool streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user)
    {
      if (!raster || w <= 0 || h <= 0)
        return false;
      if (!panel().initialized() || !flushBuffers_[0] || !flushSlots_ || flushQueueDepth_ <= 0)
        return false;

      int x0 = x;
      int y0 = y;
      int x1 = x + w - 1;
      int y1 = y + h - 1;
      if (x0 < 0)
        x0 = 0;
      if (y0 < 0)
        y0 = 0;
      if (x1 >= platform_display::kWidth)
        x1 = platform_display::kWidth - 1;
      if (y1 >= platform_display::kHeight)
        y1 = platform_display::kHeight - 1;
      if (x0 > x1 || y0 > y1)
        return true;

      flushStats_.callCount++;
      const int64_t profileStartUs = esp_timer_get_time();
      const int width = x1 - x0 + 1;

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
      const int totalRows = y1 - y0 + 1;
      bool streamFlush = false;
      if (totalRows > flushChunkRows_)
      {
#if GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
        setFlushStage(FlushStage::StreamSetWindow, 0);
        const int64_t setWindowStartUs = esp_timer_get_time();
        streamFlush = panel().setWindow(x0, y0, x1, y1) == ESP_OK;
        flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
        if (!streamFlush)
          ESP_LOGW(kTag, "CO5300 stream window setup failed, falling back to draw_bitmap");
#else
        setFlushStage(FlushStage::StreamSetWindow, 0);
        streamFlush = openColorStream(x0, y0, x1, y1, "stream");
#endif
      }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH && GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
      int streamChunksSincePace = 0;
#endif
      for (int row = y0; row <= y1; row += flushChunkRows_)
      {
        int chunkRows = flushChunkRows_;
        if (row + chunkRows > y1 + 1)
          chunkRows = y1 - row + 1;
        if (chunkRows <= 0)
          break;

        const int pixelCount = width * chunkRows;
        setFlushDetail(x0, row, x1, row + chunkRows - 1, chunkRows, pixelCount);
        if (pixelCount > flushBufferCapacity_)
        {
          ESP_LOGE(kTag, "LCD stream chunk too large: %d > %d", pixelCount, flushBufferCapacity_);
          setFlushStage(FlushStage::Idle, 0);
          flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
          return false;
        }

        const int64_t slotWaitStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::StreamWaitSlot, row);
        if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
          ESP_LOGE(kTag, "LCD stream timed out waiting for slot");
          setFlushStage(FlushStage::Idle, 0);
          flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
          return false;
        }
        flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

        std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
        flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
        setFlushStage(FlushStage::StreamRaster, row);
        const int64_t rasterStartUs = esp_timer_get_time();
        raster(buffer, width, chunkRows, x0, row, user);
        flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
        const int64_t byteSwapStartUs = esp_timer_get_time();
        for (int i = 0; i < pixelCount; i++)
        {
          buffer[i] = pixel::byteSwap16(buffer[i]);
        }
        flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

        const int64_t txStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::StreamTx, row);
#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
        esp_err_t err;
        if (streamFlush)
        {
          const int command = row == y0 ? LCD_CMD_RAMWR : LCD_CMD_RAMWRC;
          err = panel().txColor(command, buffer, pixelCount * sizeof(std::uint16_t));
        }
        else
        {
          err = panel().drawBitmap(x0, row, x1 + 1, row + chunkRows, buffer);
        }
#else
        esp_err_t err = panel().drawBitmap(x0, row, x1 + 1, row + chunkRows, buffer);
#endif
        flushStats_.txUs += esp_timer_get_time() - txStartUs;
        flushStats_.chunkCount++;
        if (err != ESP_OK)
        {
          ESP_LOGE(kTag, "LCD stream failed: %s", esp_err_to_name(err));
          xSemaphoreGive(flushSlots_);
          setFlushStage(FlushStage::Idle, 0);
          flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
          return false;
        }

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH && GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
        if (streamFlush)
        {
          streamChunksSincePace++;
          const int nextRow = row + chunkRows;
          if (streamChunksSincePace >= GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS && nextRow <= y1)
          {
            setFlushStage(FlushStage::StreamWaitComplete, row);
            const int64_t paceWaitStartUs = esp_timer_get_time();
            if (!waitForFlushComplete())
            {
              setFlushStage(FlushStage::Idle, 0);
              flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
              return false;
            }
            flushStats_.completeWaitUs += esp_timer_get_time() - paceWaitStartUs;
            streamChunksSincePace = 0;
          }
        }
#endif
      }

      setFlushStage(FlushStage::StreamWaitComplete, y0);
      const int64_t waitStartUs = esp_timer_get_time();
      if (!waitForFlushComplete())
      {
        setFlushStage(FlushStage::Idle, 0);
        flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
        return false;
      }
      flushStats_.completeWaitUs += esp_timer_get_time() - waitStartUs;
      flushStats_.pixelCount += width * (y1 - y0 + 1);
      flushOdometerPixels_.fetch_add(static_cast<std::uint64_t>(width * (y1 - y0 + 1)), std::memory_order_relaxed);
      flushOdometerCalls_.fetch_add(1, std::memory_order_relaxed);
      flushStats_.totalUs += esp_timer_get_time() - profileStartUs;
      addPresentDamageRect(x0, y0, x1, y1);
      setFlushStage(FlushStage::Idle, 0);
      return true;
    }

    void clear()
    {
      if (!frameBuffer_)
        return;
      clearNoFlush();
      flush();
      cursorRow_ = 0;
      cursorColumn_ = 0;
    }

    void clearNoFlush()
    {
      if (!frameBuffer_)
        return;
      previousPresentValid_ = false;
      canvas_.clear(kConsoleBackground);
      cursorRow_ = 0;
      cursorColumn_ = 0;
    }

    void print(const char *text)
    {
      if (!panel().initialized() || !frameBuffer_ || !text)
        return;
      for (const char *p = text; *p; p++)
      {
        if (*p == '\n')
        {
          flushRegion(0, cursorRow_ * platform_display::kConsoleGlyphHeight, platform_display::kWidth, platform_display::kConsoleGlyphHeight);
          newline();
          continue;
        }
        renderCharToFramebuffer(cursorColumn_, cursorRow_, *p);
        cursorColumn_++;
        if (cursorColumn_ >= platform_display::kConsoleColumns)
        {
          flushRegion(0, cursorRow_ * platform_display::kConsoleGlyphHeight, platform_display::kWidth, platform_display::kConsoleGlyphHeight);
          newline();
        }
      }
      if (cursorColumn_ > 0)
      {
        flushRegion(0, cursorRow_ * platform_display::kConsoleGlyphHeight, platform_display::kWidth, platform_display::kConsoleGlyphHeight);
      }
    }

    esp_err_t init()
    {
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
      DisplayInitHeapTraceScope displayInitHeapTraceScope;
#endif
      ESP_LOGI(kTag,
               "Initializing %s QSPI display (logical=%dx%d native=%dx%d, %dx scale)",
               qspi_panel::qspiPanelDriverName(),
               logicalDisplayWidth(),
               logicalDisplayHeight(),
               platform_display::kNativeWidth,
               platform_display::kNativeHeight,
               platform_display::kConsoleFontScale);
      ESP_LOGI(kTag, "Panel flush mode: %s, QSPI clock: %d Hz", kFlushMode, kQspiPclkHz);

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
      const std::size_t framebufferBytes =
          static_cast<std::size_t>(platform_display::kNativeWidth) * platform_display::kNativeHeight * sizeof(std::uint16_t);
#else
      const std::size_t framebufferBytes =
          static_cast<std::size_t>(kFramebufferStridePx) * platform_display::kHeight * sizeof(std::uint16_t);
#endif
      const std::size_t framebufferAllocBytes = (framebufferBytes + 63U) & ~static_cast<std::size_t>(63U);

      // Allocate framebuffer with 1 MB alignment so it lands on a known-good
      // PSRAM region for direct-PSRAM GDMA. The default 64-byte alignment
      // places frameBuffer_ in the very-low PSRAM range (~0x91b40), which
      // GDMA cannot read correctly when SPI bursts it as direct DMA — the
      // panel just receives all zeros (black screen) even though the data
      // is in PSRAM. Shifting frameBuffer_ past the first MB by aligning to
      // 1 MB makes direct PSRAM full-width work. Root cause is PSRAM-side
      // (likely bank/MMU-page boundary interaction with the GDMA burst
      // engine on the AHB path), not fixable in software except by avoiding
      // the bad addresses.
      frameBuffer_ = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(1024 * 1024, framebufferAllocBytes, MALLOC_CAP_SPIRAM));
      if (!frameBuffer_)
      {
        ESP_LOGE(kTag, "Failed to allocate framebuffer in PSRAM");
        return ESP_ERR_NO_MEM;
      }
      std::memset(frameBuffer_, 0, framebufferAllocBytes);
      // Canvas sees the logical panel width (kWidth) but uses
      // kFramebufferStridePx for row arithmetic. Clip bounds, dirty rect, and
      // width() all return the panel width; only pixel indexing uses the
      // stride. This keeps the framework's coordinate system untouched.
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
      bindCanvasesForActiveOrientation();
#else
      canvas_.bindPixels(frameBuffer_, platform_display::kWidth, platform_display::kHeight, kFramebufferStridePx);
      workerCanvas_.bindPixels(frameBuffer_, platform_display::kWidth, platform_display::kHeight, kFramebufferStridePx);
#endif
      ESP_LOGI(kTag,
               "Framebuffer allocated: %d bytes in PSRAM (stride=%d px, %s)",
               static_cast<int>(framebufferBytes),
               kFramebufferStridePx,
               kFramebufferStrideMode);
      ESP_LOGI(kTag,
               "Framebuffer source: ptr=%p external=%d dma_capable=%d align64=%d",
               frameBuffer_,
               esp_ptr_external_ram(frameBuffer_) ? 1 : 0,
               esp_ptr_dma_capable(frameBuffer_) ? 1 : 0,
               (reinterpret_cast<std::uintptr_t>(frameBuffer_) & 63U) == 0 ? 1 : 0);

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
      directPsramStaticTxProbe_ = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(64, framebufferAllocBytes, MALLOC_CAP_SPIRAM));
      if (!directPsramStaticTxProbe_)
      {
        ESP_LOGE(kTag, "Failed to allocate direct PSRAM TX probe buffer");
        return ESP_ERR_NO_MEM;
      }
      fillDirectPsramStaticTxProbe();
      ESP_LOGW(kTag,
               "Direct PSRAM static TX probe enabled: direct LCD flushes use immutable PSRAM source ptr=%p",
               directPsramStaticTxProbe_);
#endif

#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      backFrameBuffer_ = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(64, framebufferAllocBytes, MALLOC_CAP_SPIRAM));
      if (!backFrameBuffer_)
      {
        ESP_LOGW(kTag,
                 "Failed to allocate second framebuffer for double-buffer mode; "
                 "falling back to single-buffer pipeline");
      }
      else
      {
        std::memset(backFrameBuffer_, 0, framebufferAllocBytes);
        ESP_LOGI(kTag,
                 "Double-buffer enabled: 2 x %d bytes PSRAM (padded stride for GDMA)",
                 static_cast<int>(framebufferBytes));
      }
      // async_memcpy install is deferred to AFTER panel().init() below.
      //
      // Why: the M2M trigger allocator picks the lowest free instance bit from
      // a mask shared with real peripheral types. Installing first would claim
      // bit 0, which collides with SPI2's instance_id (also 0) when the LCD
      // SPI driver tries to bind its GDMA channel. The SPI driver ignores
      // gdma_connect's return value, so the conflict silently breaks panel
      // output — no SPI trigger means DMA never fires, screen stays blank.
      //
      // Installing after panel init means SPI2 grabs bit 0 first; M2M picks
      // the next free bit (1+). No conflict, panel works, GDMA works.
#endif

      qspi_panel::QspiPanelConfig config{};
      config.spiHost = gea::platform::board::display.spiHost;
      config.pinCs = gea::platform::board::display.cs;
      config.pinPclk = gea::platform::board::display.pclk;
      config.pinData0 = gea::platform::board::display.data0;
      config.pinData1 = gea::platform::board::display.data1;
      config.pinData2 = gea::platform::board::display.data2;
      config.pinData3 = gea::platform::board::display.data3;
      config.pinReset = gea::platform::board::display.reset;
      config.qspiPclkHz = kQspiPclkHz;
      // max_transfer_sz sizes the SPI bus DMA-descriptor pool, NOT the per-
      // transaction cap (that's hardware: 32KB / 2^18 bits). At 1x it holds
      // exactly one 65.6KB chunk's descriptors, so a second chunk can't be queued
      // until the first's DMA frees them — the SPI stalls between chunks (~22MB/s
      // effective vs 40MB/s peak). 4x gives descriptors for several chunks so the
      // CPU queues ahead and the DMA streams continuously. Costs only ~KB of
      // descriptors (no bounce buffer — our flush buffers are DMA-capable SRAM).
      config.maxTransferBytes = kPanelMaxTransferBytes * 8;
      config.transactionQueueDepth = kSpiTransactionQueueDepth;
      config.onColorTransferDone = &DisplayBackend::flushDoneCallback;
      config.callbackContext = this;

#if GEA_BOARD_PREPARE_DISPLAY_PANEL
      // A board whose panel reset is not a GPIO -- behind an I2C expander, say
      // -- supplies prepareDisplayPanel() to pulse it here. Such a panel only
      // accepts its init sequence after that reset, and a QSPI write reports
      // success either way, so without this the screen stays dark while every
      // log line says the display came up.
      gea::platform::board::prepareDisplayPanel();
#endif

      const esp_err_t panelErr = panel().init(config);
      if (panelErr != ESP_OK)
      {
        return panelErr;
      }
      setBrightness(brightness_);
      setupTeVsync();

#if GEA_BOARD_PANEL_BRINGUP_FILL
      // Bring-up only: paint a solid band straight through the panel binding,
      // bypassing the canvas and the damage tracker. It answers one question a
      // new board's logs cannot -- whether the QSPI path really reaches the
      // glass -- because every call above reports success whether or not the
      // panel is listening.
      {
        constexpr int kRows = 32;
        const int width = platform_display::kNativeWidth;
        auto *row = static_cast<std::uint16_t *>(
            heap_caps_malloc(static_cast<std::size_t>(width) * kRows * sizeof(std::uint16_t), MALLOC_CAP_DMA));
        if (row)
        {
          for (int i = 0; i < width * kRows; ++i) row[i] = 0xF800;  // red, RGB565
          const esp_err_t fill = panel().drawBitmap(0, 0, width, kRows, row);
          ESP_LOGW(kTag, "bring-up fill %dx%d -> %s", width, kRows, esp_err_to_name(fill));
          heap_caps_free(row);
        }
        else
        {
          ESP_LOGW(kTag, "bring-up fill: no DMA memory");
        }
      }
#endif

#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      // Now that SPI2 has claimed peripheral instance 0 in the GDMA in-use
      // mask, the async_memcpy install will pick a non-conflicting M2M
      // trigger ID. Burst=32 is empirically stable in this install order
      // (burst=64 panics in gdma_ll_tx_set_burst_size — unexplained).
      async_memcpy_config_t mcpConfig = ASYNC_MEMCPY_DEFAULT_CONFIG();
      mcpConfig.backlog = 4;
      mcpConfig.dma_burst_size = kAsyncMemcpyBurstBytes;
      const esp_err_t mcpErr = esp_async_memcpy_install_gdma_ahb(&mcpConfig, &asyncMemcpy_);
      if (mcpErr != ESP_OK)
      {
        ESP_LOGW(kTag,
                 "async_memcpy install failed (%s); scrollRect will use CPU memcpy",
                 esp_err_to_name(mcpErr));
        asyncMemcpy_ = nullptr;
      }
      else
      {
        asyncMemcpyDoneSem_ = xSemaphoreCreateBinary();
        if (!asyncMemcpyDoneSem_)
        {
          ESP_LOGW(kTag, "async_memcpy semaphore alloc failed; uninstalling driver");
          esp_async_memcpy_uninstall(asyncMemcpy_);
          asyncMemcpy_ = nullptr;
        }
        else
        {
          ESP_LOGI(kTag,
                   "async_memcpy installed for scrollRect (AHB GDMA, burst=%d)",
                   static_cast<int>(mcpConfig.dma_burst_size));
        }
      }
#endif

      // Claim the minimum viable flush staging NOW, while the panel is the only
      // thing that has asked for internal DMA RAM. Everything that competes for it
      // — the WiFi driver's static RX frames, the BT controller, the HTTP server —
      // comes up after this point and never gives any of it back, so a pipeline
      // first requested at Display::start() can find nothing left and the device
      // boots to a dark panel with a perfectly correct framebuffer behind it.
      // start() still asks for the board's full profile; this only guarantees a
      // floor it can fall back to.
      const esp_err_t floorErr = configureFlushPipeline(kFlushChunkMin, 1);
      if (floorErr != ESP_OK)
      {
        ESP_LOGW(kTag, "LCD flush floor unavailable at init: %s", esp_err_to_name(floorErr));
      }

      // Then grow to the board's profile immediately, instead of leaving that to
      // start(). The floor above is a 1-row, 1-DEEP pipeline, and 1-deep is not
      // merely slower: it disables the color-stream path (which requires depth >= 2)
      // and the rotated flush then times out waiting for a slot, so the panel paints
      // NOTHING while the pipeline sits there. An app whose own native bring-up
      // paints before the runtime reaches start() -- the NAM pedalboard mounts its
      // panel UI during gea_init, on a boot where the amp graph has already taken
      // most of the internal RAM -- therefore spends that whole window flushing into
      // 1000ms timeouts and never gets far enough for start() to lift it.
      // configureFlushPipeline descends through every smaller candidate and, if all
      // of them fail, re-takes the configuration that was live on entry, so asking
      // for the profile here can only improve on the floor we just claimed.
      const esp_err_t profileErr = configureFlushPipeline(requestedFlushChunkRows_, requestedFlushQueueDepth_);
      if (profileErr != ESP_OK)
      {
        ESP_LOGW(kTag, "LCD flush profile unavailable at init: %s", esp_err_to_name(profileErr));
      }

      ESP_LOGI(kTag, "Display panel ready (%d cols x %d rows)", platform_display::kConsoleColumns, platform_display::kConsoleRows);
      return ESP_OK;
    }

    esp_err_t start()
    {
      if (!panel().initialized() || !frameBuffer_)
      {
        return ESP_ERR_INVALID_STATE;
      }

      const esp_err_t flushErr = configureFlushPipeline(requestedFlushChunkRows_, requestedFlushQueueDepth_);
      if (flushErr != ESP_OK)
      {
        return flushErr;
      }

      if (!displayOn_)
      {
        const esp_err_t onErr = panel().setDisplayOn(true);
        if (onErr != ESP_OK)
        {
          ESP_LOGE(kTag, "display on failed: %s", esp_err_to_name(onErr));
          return onErr;
        }
        displayOn_ = true;
      }

      int x0;
      int y0;
      int x1;
      int y1;
      if (!canvas_.dirty(&x0, &y0, &x1, &y1))
      {
        canvas_.markDirty(0, 0, logicalDisplayWidth() - 1, logicalDisplayHeight() - 1);
      }
      flush();
      if (!waitForFlushComplete())
      {
        return ESP_ERR_TIMEOUT;
      }

      ESP_LOGI(kTag, "Display ready (%d cols x %d rows)", platform_display::kConsoleColumns, platform_display::kConsoleRows);
      return ESP_OK;
    }

    gea::framework::graphics::Canvas *canvas()
    {
      return &replayCanvas();
    }

    bool copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height, bool presented)
    {
      const int logicalWidth = logicalDisplayWidth();
      const int logicalHeight = logicalDisplayHeight();
      if (!dst || !frameBuffer_ || pixelCapacity < logicalWidth * logicalHeight)
        return false;
      if (presented && !waitForFlushComplete())
        return false;

      // Display-backed canvas fast path: present() DMAs canvas commands
      // straight to the panel and the CPU framebuffer never sees them, so a
      // framebuffer copy would return the stale boot frame forever. Rebuild
      // the snapshot from the retained last-presented frame instead (callers
      // hold AppState::lock(), so the frame task isn't mutating it; bitmap
      // pointers reference live image-store slots).
      if (lastPanelUpdateWasPresent_ && previousPresentValid_)
      {
        present::rasterFrameRowsStrided(dst,
                                        logicalWidth,
                                        0,
                                        logicalWidth,
                                        0,
                                        logicalHeight,
                                        logicalHeight,
                                        previousPresentFrame_);
        const int pixelCount = logicalWidth * logicalHeight;
        for (int i = 0; i < pixelCount; i++)
          dst[i] = pixel::toRgb565(dst[i]);
        if (width)
          *width = logicalWidth;
        if (height)
          *height = logicalHeight;
        return true;
      }

      const std::uint16_t *source = frameBuffer_;
#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      if (presented && backFrameBuffer_)
        source = backFrameBuffer_;
#else
      (void)presented;
#endif

      if (width)
        *width = logicalWidth;
      if (height)
        *height = logicalHeight;

      for (int y = 0; y < logicalHeight; y++)
      {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
        if (softwareLandscapeActive())
        {
          for (int x = 0; x < logicalWidth; x++)
          {
            const int physicalRow = platform_display::kNativeHeight - 1 - x;
            dst[y * logicalWidth + x] =
                pixel::toRgb565(source[physicalRow * platform_display::kNativeWidth + y]);
          }
          continue;
        }
#endif
        const int physRow = sourceFramebufferRow(source, y);
        const std::uint16_t *row =
            &source[physRow * physicalFramebufferStridePixels()];
        for (int x = 0; x < logicalWidth; x++)
        {
          dst[y * logicalWidth + x] = pixel::toRgb565(row[x]);
        }
      }
      return true;
    }

    int countNonBlackPixels(bool presented)
    {
      if (!frameBuffer_)
        return 0;
      if (presented && !waitForFlushComplete())
        return 0;

      const std::uint16_t *source = frameBuffer_;
#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      if (presented && backFrameBuffer_)
        source = backFrameBuffer_;
#else
      (void)presented;
#endif

      int count = 0;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
      const int pixelCount = platform_display::kNativeWidth * platform_display::kNativeHeight;
      for (int i = 0; i < pixelCount; ++i)
      {
        if (pixel::toRgb565(source[i]) != 0)
          count++;
      }
#else
      for (int y = 0; y < platform_display::kHeight; y++)
      {
        const int physRow = sourceFramebufferRow(source, y);
        const std::uint16_t *row = &source[physRow * kFramebufferStridePx];
        for (int x = 0; x < platform_display::kWidth; x++)
        {
          if (pixel::toRgb565(row[x]) != 0)
            count++;
        }
      }
#endif
      return count;
    }

    void setWorldOverlay(const std::uint16_t *pixels, int width, int height, int panelTop, int panelHeight)
    {
      worldPixels_ = pixels;
      worldWidth_ = width;
      worldHeight_ = height;
      worldPanelTop_ = panelTop;
      worldPanelHeight_ = panelHeight;
      worldScrollX_ = 0;
    }

    void setWorldScroll(int scrollX)
    {
      worldScrollX_ = scrollX;
    }

    void setPixel(int x, int y, std::uint16_t color) { replayCanvas().fillRect(x, y, 1, 1, color); }
    void fillRect(int x, int y, int w, int h, std::uint16_t color)
    {
      auto &targetCanvas = replayCanvas();
      if (targetCanvas.globalAlpha() == 255)
      {
        targetCanvas.fillRectOpaque(x, y, w, h, color);
        return;
      }
      targetCanvas.fillRect(x, y, w, h, color);
    }
    void resetScrollRegion()
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
      regionY_ = 0;
      regionH_ = 0;
      scrollOffsetY_ = 0;
      canvas_.setScrollRegion(0, 0, 0);
#endif
    }

    void scrollRect(int x, int y, int w, int h, int dx, int dy)
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
      // Full-width vertical scroll (any rect height): no pixels move. We
      // rotate the logical → physical FB row mapping within the rect by dy.
      // The framework draws the newly-exposed strip via replayDisplayRegion;
      // those writes land at translated rows. Untouched FB rows inside the
      // region keep their old content, which represents the prior frame's
      // logical rows — under the new offset, they slide into their new
      // on-screen positions. Mark the rect dirty so flush sends the rotated
      // view.
      //
      // Sub-rect scrolls (e.g. a list pinned below a header) work as long
      // as the scroll container spans the full panel width — the row
      // translation only kicks in for rows inside [y, y+h).
      if (dx == 0 && dy != 0 && x == 0 && w >= logicalDisplayWidth() && h > 0 &&
          y >= 0 && y + h <= logicalDisplayHeight())
      {
        // If the active region changed (different rect), reset the offset:
        // the prior rotation belonged to a different rect and would be junk
        // applied to a new region.
        if (regionY_ != y || regionH_ != h)
        {
          regionY_ = y;
          regionH_ = h;
          scrollOffsetY_ = 0;
        }
        scrollOffsetY_ -= dy;
        scrollOffsetY_ %= regionH_;
        if (scrollOffsetY_ < 0)
          scrollOffsetY_ += regionH_;
        canvas_.setScrollRegion(regionY_, regionH_, scrollOffsetY_);
        canvas_.markDirty(0, y, logicalDisplayWidth() - 1, y + h - 1);
        return;
      }
#endif
#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
      if (backFrameBuffer_ && frameBuffer_)
      {
        // Read source pixels from backFrameBuffer_ (= last-drawn frame, the
        // content that's currently on the panel) and write into frameBuffer_
        // (= the canvas-bound buffer being prepared for the next frame).
        // The active draw buffer was the SPI source two frames ago, so its
        // current content is stale and unusable as a scroll source.
        scrollRectCrossBuffer(backFrameBuffer_, frameBuffer_, x, y, w, h, dx, dy);
        return;
      }
#endif
      canvas_.scrollRect(x, y, w, h, dx, dy);
    }
    // All per-pixel drawing routes through replayCanvas() so the 2nd-core worker
    // draws into ITS band-canvas (own clip = bottom band, own dirty) — not the main
    // canvas (whose clip is the top band). On the main core replayCanvas()==canvas_,
    // so single-core behavior is unchanged. (Without this, the worker's backdrop-clear
    // blit + labels were clipped to the top band → stale pixels below the split line.)
    void strokeRect(int x, int y, int w, int h, std::uint16_t color) { replayCanvas().strokeRect(x, y, w, h, color); }
    void fillCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().fillCircle(cx, cy, r, color); }
    void strokeCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().strokeCircle(cx, cy, r, color); }
    void drawLine(int x0, int y0, int x1, int y1, std::uint16_t color) { replayCanvas().drawLine(x0, y0, x1, y1, color); }
    void drawArc(int cx, int cy, int r, int startDeg, int endDeg, std::uint16_t color) { replayCanvas().drawArc(cx, cy, r, startDeg, endDeg, color); }
    void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, std::uint16_t color) { replayCanvas().fillTriangle(x0, y0, x1, y1, x2, y2, color); }
    void drawText(const char *text, int x, int y, std::uint16_t color, float scale) { replayCanvas().drawText(text, x, y, color, scale); }
    void drawTextFont(const char *text, int x, int y, std::uint16_t color, int fontId)
    {
#ifdef GEA_EMBEDDED_HAS_GENERATED_FONTS
      replayCanvas().drawTextFont(text, x, y, color, fontId);
#else
      (void)text;
      (void)x;
      (void)y;
      (void)color;
      (void)fontId;
#endif
    }
    void drawTextFontFamily(const char *text, int x, int y, std::uint16_t color, int familyId, int sizePx)
    {
#ifdef GEA_EMBEDDED_HAS_GENERATED_FONTS
      replayCanvas().drawTextFontFamily(text, x, y, color, familyId, sizePx);
#else
      (void)text;
      (void)x;
      (void)y;
      (void)color;
      (void)familyId;
      (void)sizePx;
#endif
    }
    void fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, std::uint16_t color) { replayCanvas().fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
    void fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count,
                                    int w, int h, int tl, int tr, int br, int bl,
                                    const std::uint16_t *colors)
    {
      replayCanvas().fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
    }
    void strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lineWidth, std::uint16_t color) { replayCanvas().strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lineWidth, color); }
    void blitImage(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy) { replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy); }
    void blitImageScaled(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy, int dstWidth, int dstHeight)
    {
      replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy, dstWidth, dstHeight);
    }

  private:
#if GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT
    // Not defaulted and never inlined: with the constant member initializers
    // folded in, the backend is a .data object, and .data is internal RAM. A
    // dynamic initializer keeps it in .bss for the external-RAM sweep (see
    // ui/state_init.h). Nothing in it is DMA'd: the flush buffers it points
    // at are allocated separately with MALLOC_CAP_DMA.
    __attribute__((noinline)) DisplayBackend() {}
#else
    DisplayBackend() = default;
#endif

    static bool IRAM_ATTR flushDoneCallback(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *eventData, void *userContext)
    {
      (void)io;
      (void)eventData;
      auto *self = static_cast<DisplayBackend *>(userContext);
      if (!self || !self->flushSlots_)
        return false;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      // TEMP wire probe (gea3d present-path work): per-chunk DMA completion
      // timestamps; paired with kick timestamps to derive real chunk service
      // time under load with zero flow change.
      if (self->splitProbeArm_ && self->splitDoneN_ < kSplitProbeSlots)
      {
        // Non-compound increment: '++' on a volatile-qualified type is deprecated
        // under C++20 (-Werror=volatile).
        const int n = self->splitDoneN_;
        self->splitDoneTs_[n] = esp_timer_get_time();
        self->splitDoneN_ = n + 1;
      }
#endif
      BaseType_t needYield = pdFALSE;
      xSemaphoreGiveFromISR(self->flushSlots_, &needYield);
      return needYield == pdTRUE;
    }

    // --- TE / vsync -----------------------------------------------------------
    // The CO5300 pulses its TE line at the start of vertical blanking (TEON is
    // enabled in the panel init). On each edge the ISR posts one frame to the scheduler
    // (the single frame clock when TE-sync is on) and releases teSync_.
    // NOT IRAM_ATTR: the GPIO ISR service is installed with flag 0 (non-IRAM), so this
    // handler never runs while flash cache is disabled — it may live in flash and call
    // flash-resident helpers (FrameScheduler::notifyVsyncFromISR). Keeping IRAM_ATTR here
    // forced an l32r literal load to that flash function → "dangerous relocation" link error.
    static void teEdgeIsr(void *arg)
    {
      auto *self = static_cast<DisplayBackend *>(arg);
      if (!self || !self->teSync_)
        return;
      // Single-clock: when TE-sync is on, the TE edge IS the frame producer — post one
      // (coalesced) frame to the scheduler. The scheduler's own timer steps out (see
      // queueFrameEvent), so the panel TE is the only frame clock. The semaphore give
      // below is kept as a harmless legacy signal (waitForVsync is no longer in the
      // present path when TE drives), so nothing else that may take it is starved.
      if (self->vsyncEnabled_)
        gea::framework::services::FrameScheduler::notifyVsyncFromISR();
      BaseType_t needYield = pdFALSE;
      xSemaphoreGiveFromISR(self->teSync_, &needYield);
      if (needYield == pdTRUE)
        portYIELD_FROM_ISR();
    }

    void setupTeVsync()
    {
      const gpio_num_t te = gea::platform::board::display.te;
      if (te == GPIO_NUM_NC)
        return; // board does not route TE
      teSync_ = xSemaphoreCreateBinary();
      if (!teSync_)
      {
        ESP_LOGW(kTag, "TE vsync: semaphore alloc failed; tearing-sync disabled");
        return;
      }
      gpio_config_t cfg = {};
      // `te` is a real GPIO here (GPIO_NUM_NC handled by the early return above),
      // but on boards where board::display.te is a constexpr GPIO_NUM_NC the
      // compiler still constant-folds this otherwise-unreachable shift and
      // -Werror=shift-count-overflow rejects `1ULL << (unsigned)-1`. Mask the
      // count to 0-63: a no-op for real pins (all < 64), and a defined (unused)
      // value on the dead NC path. Without this, any app fails to compile on a
      // TE-less board (e.g. amoled-1.8).
      cfg.pin_bit_mask = 1ULL << (static_cast<unsigned>(te) & 63u);
      cfg.mode = GPIO_MODE_INPUT;
      cfg.pull_up_en = GPIO_PULLUP_DISABLE;
      cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
      cfg.intr_type = GPIO_INTR_POSEDGE; // TE asserts high at VBlank start
      if (gpio_config(&cfg) != ESP_OK)
      {
        ESP_LOGW(kTag, "TE vsync: gpio_config(GPIO%d) failed; disabled", static_cast<int>(te));
        vSemaphoreDelete(teSync_);
        teSync_ = nullptr;
        return;
      }
      // The touch driver may already own the ISR service; INVALID_STATE = already installed.
      const esp_err_t isrErr = gpio_install_isr_service(0);
      if (isrErr != ESP_OK && isrErr != ESP_ERR_INVALID_STATE)
        ESP_LOGW(kTag, "TE vsync: gpio_install_isr_service failed (%s)", esp_err_to_name(isrErr));
      if (gpio_isr_handler_add(te, &DisplayBackend::teEdgeIsr, this) != ESP_OK)
      {
        ESP_LOGW(kTag, "TE vsync: isr_handler_add(GPIO%d) failed; disabled", static_cast<int>(te));
        vSemaphoreDelete(teSync_);
        teSync_ = nullptr;
        return;
      }
      // Vsync starts off; setVSync(true) arms the interrupt.
      if (!vsyncEnabled_)
        gpio_intr_disable(te);
      // Lead time: fire TE ~kVsyncLeadLines before the frame's last line so the
      // host has time to write the NEXT frame's top rows into GRAM before scanout
      // wraps to row 0 — closes the residual top-row tear that a pure VBlank TE
      // leaves (the TE→first-write latency races scanout at the very top). Tune
      // up if the top still tears, down if a tear appears lower.
      // Lead widened from 80→120 lines (~2.6ms→~4ms) because the single-clock path no
      // longer flushes AT the TE edge — the TE triggers the frame, then ~1.4ms of compute
      // runs before the GRAM write begins. The wider lead keeps that write comfortably
      // ahead of scanout (tear-free) despite the compute delay + frame-to-frame jitter.
      constexpr int kVsyncLeadLines = 120;
      const int tearLine = platform_display::kHeight - kVsyncLeadLines;
      if (panel().setTearScanline(tearLine) != ESP_OK)
        ESP_LOGW(kTag, "TE vsync: set tear scanline %d failed", tearLine);
      ESP_LOGI(kTag, "TE vsync enabled on GPIO%d (tear scanline %d)", static_cast<int>(te), tearLine);
    }

    // Block until the next TE (VBlank) edge, with a >1-frame timeout fallback so a
    // missing/garbled TE can never freeze presentation. No-op if TE isn't wired.
    void waitForVsync()
    {
      if (!teSync_)
        return;
      xSemaphoreTake(teSync_, 0); // drop any edge that arrived since last frame
      xSemaphoreTake(teSync_, pdMS_TO_TICKS(25));
    }

    qspi_panel::QspiPanel &panel()
    {
      return qspi_panel::qspiPanel();
    }

    void flushRegion(int x, int y, int w, int h)
    {
      canvas_.markDirty(x, y, x + w - 1, y + h - 1);
      flush();
    }

    int normalizeFlushChunkRows(int rows) const
    {
      if (rows <= 0)
        rows = kFlushChunkDefault;
      if (rows < kFlushChunkMin)
        rows = kFlushChunkMin;
      if (rows > kFlushChunkLimit)
        rows = kFlushChunkLimit;
      return rows;
    }

    int normalizeFlushQueueDepth(int depth) const
    {
      if (depth <= 0)
        depth = kFlushQueueDepthDefault;
      if (depth < 1)
        depth = 1;
      if (depth > kFlushQueueDepthLimit)
        depth = kFlushQueueDepthLimit;
      return depth;
    }

    int nextSmallerChunk(int chunk) const
    {
      if (chunk <= kFlushChunkMin)
        return 0;
      chunk -= 4;
      return chunk < kFlushChunkMin ? kFlushChunkMin : chunk;
    }

    void freeFlushBuffers()
    {
      for (int slot = 0; slot < kFlushQueueDepthLimit; slot++)
      {
        if (flushBuffers_[slot])
        {
          // Pool-carved slots are views into the static reservation, not
          // heap allocations.
          if (!flushBuffersFromPool_)
            heap_caps_free(flushBuffers_[slot]);
          flushBuffers_[slot] = nullptr;
        }
      }
      flushBuffersFromPool_ = false;
      flushBufferCapacity_ = 0;
      flushBufferIndex_ = 0;
      configuredRequestFlushChunkRows_ = 0;
      configuredRequestFlushQueueDepth_ = 0;
    }

    bool waitForFlushIdle()
    {
      if (!flushSlots_ || flushQueueDepth_ <= 0)
        return true;

      int taken = 0;
      for (int i = 0; i < flushQueueDepth_; i++)
      {
        if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
          for (int j = 0; j < taken; j++)
            xSemaphoreGive(flushSlots_);
          ESP_LOGE(kTag, "Timed out waiting for LCD flush pipeline to idle");
          return false;
        }
        taken++;
      }
      return true;
    }

    bool waitForFlushComplete()
    {
      if (!flushSlots_ || flushQueueDepth_ <= 0)
        return true;

      int taken = 0;
      for (int i = 0; i < flushQueueDepth_; i++)
      {
        if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
          for (int j = 0; j < taken; j++)
            xSemaphoreGive(flushSlots_);
          ESP_LOGE(kTag, "Timed out waiting for LCD flush pipeline to complete");
          return false;
        }
        taken++;
      }
      for (int i = 0; i < taken; i++)
        xSemaphoreGive(flushSlots_);
      return true;
    }

    // Like waitForFlushComplete(), but busy-polls the slot count for a short budget
    // before falling back to the blocking take. A blocking xSemaphoreTake here is woken
    // by the SPI-completion ISR on the SPI host's core, while the frame task is pinned to
    // APP_FRAME_TASK_CORE — so the wakeup pays a cross-core reschedule (~700us). For the
    // tiny tail DMAs this drain waits on (a few chunks), polling returns within the real
    // ~35-150us completion time. The blocking fallback keeps the cross-core safety net
    // for an unexpectedly long drain without spinning a core indefinitely.
    bool waitForFlushCompleteSpin(int64_t spinBudgetUs = 2000)
    {
      if (!flushSlots_ || flushQueueDepth_ <= 0)
        return true;
      const int64_t start = esp_timer_get_time();
      while (esp_timer_get_time() - start < spinBudgetUs)
      {
        if (uxSemaphoreGetCount(flushSlots_) >= static_cast<UBaseType_t>(flushQueueDepth_))
          return true;
      }
      return waitForFlushComplete();
    }

    esp_err_t tryConfigureFlushPipelineCandidate(int chunkRows, int depth, int targetRows, int targetDepth)
    {
      const std::size_t slotBytes = static_cast<std::size_t>(kFlushScanlineWidth) * chunkRows * sizeof(std::uint16_t);
      const std::size_t totalBytes = slotBytes * depth;
      if (totalBytes > g_flushBudgetBytes)
        return ESP_ERR_INVALID_SIZE;

      bool ok = true;
      flushBuffersFromPool_ = false;
#if GEA_EMBEDDED_DISPLAY_FLUSH_POOL_BYTES > 0
      // Carve from the reserved pool when the whole pipeline fits (64-byte
      // slot stride). Falls through to per-slot mallocs otherwise.
      const std::size_t slotStride = (slotBytes + 63) & ~static_cast<std::size_t>(63);
      if (slotStride * depth <= sizeof(g_flushPool))
      {
        for (int slot = 0; slot < depth; slot++)
        {
          flushBuffers_[slot] = reinterpret_cast<std::uint16_t *>(
              reinterpret_cast<std::uint8_t *>(g_flushPool) + slotStride * static_cast<std::size_t>(slot));
        }
        flushBuffersFromPool_ = true;
      }
#endif
      if (!flushBuffersFromPool_)
      {
        const std::size_t freeDma = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (freeDma < totalBytes + kFlushDmaReserveBytes)
          return ESP_ERR_NO_MEM;
        for (int slot = 0; slot < depth; slot++)
        {
          flushBuffers_[slot] = static_cast<std::uint16_t *>(heap_caps_malloc(slotBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
          if (!flushBuffers_[slot])
          {
            ok = false;
            break;
          }
        }
      }
      if (!ok)
      {
        freeFlushBuffers();
        return ESP_ERR_NO_MEM;
      }

      flushSlots_ = xSemaphoreCreateCounting(depth, depth);
      if (!flushSlots_)
      {
        ESP_LOGW(kTag, "Failed to create LCD flush semaphore for %d rows, %d-deep pipeline", chunkRows, depth);
        freeFlushBuffers();
        return ESP_ERR_NO_MEM;
      }

      flushChunkRows_ = chunkRows;
      flushQueueDepth_ = depth;
      flushBufferCapacity_ = kFlushScanlineWidth * chunkRows;
      flushBufferIndex_ = 0;
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
      for (int slot = 0; slot < depth; slot++)
      {
        ESP_LOGI(kTag, "LCD flush slot: index=%d ptr=%p bytes=%u",
                 slot,
                 static_cast<void *>(flushBuffers_[slot]),
                 static_cast<unsigned>(slotBytes));
      }
#endif
      if (chunkRows != targetRows || depth != targetDepth)
      {
        ESP_LOGW(kTag, "LCD flush pipeline reduced to %d rows, %d-deep pipeline", chunkRows, depth);
      }
      ESP_LOGI(kTag, "LCD flush pipeline: %d rows, %d-deep pipeline", chunkRows, depth);
      configuredRequestFlushChunkRows_ = targetRows;
      configuredRequestFlushQueueDepth_ = targetDepth;
      return ESP_OK;
    }

    void teardownFlushPipeline()
    {
      if (flushSlots_)
      {
        vSemaphoreDelete(flushSlots_);
        flushSlots_ = nullptr;
      }
      freeFlushBuffers();
    }

    esp_err_t configureFlushPipeline(int requestedRows, int requestedDepth)
    {
      const int targetRows = normalizeFlushChunkRows(requestedRows);
      const int targetDepth = normalizeFlushQueueDepth(requestedDepth);

      if (flushSlots_ && configuredRequestFlushChunkRows_ == targetRows &&
          configuredRequestFlushQueueDepth_ == targetDepth && flushBuffers_[0])
        return ESP_OK;
      if (!waitForFlushIdle())
        return ESP_ERR_TIMEOUT;

      // The descent below starts by FREEING the pipeline it is trying to replace,
      // because a bigger candidate can only fit in the memory the current one
      // holds. That is fine when a candidate succeeds and fatal when none does:
      // the panel is then left with no staging at all and the runtime parks with a
      // dark screen. Remember what was working so the failure path can put it back.
      const int previousRows = flushBuffers_[0] ? flushChunkRows_ : 0;
      const int previousDepth = flushBuffers_[0] ? flushQueueDepth_ : 0;

      teardownFlushPipeline();

      // TEMP MEASUREMENT (gea3d flush-pool sizing): remove after profiling.
      ESP_LOGI(kTag, "flush-configure heap: internal-dma free=%u largest=%u | internal free=%u largest=%u (target %d rows x %d)",
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)),
               static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)),
               static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
               static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
               targetRows, targetDepth);

      // Prefer keeping the requested DEPTH over a bigger single chunk: a 2-deep
      // pipeline overlaps rasterize with DMA (that overlap is most of the flush
      // win), whereas a bigger 1-deep chunk runs them serially. So exhaust every
      // chunk size at the current depth before dropping to a shallower pipeline.
      // (The old logic dropped depth at target/2 rows, so a 1-deep big chunk beat
      // a 2-deep smaller chunk and silently lost the overlap.) With a reserved
      // pool the first candidate — the exact target — carves directly and wins;
      // this descent only runs when the pool is unavailable and we are back to
      // fragmented per-slot mallocs.
      for (int depth = targetDepth; depth >= 1; depth--)
      {
        for (int chunk = targetRows; chunk >= kFlushChunkMin; chunk = nextSmallerChunk(chunk))
        {
          if (tryConfigureFlushPipelineCandidate(chunk, depth, targetRows, targetDepth) == ESP_OK)
            return ESP_OK;
          if (chunk == kFlushChunkMin)
            break;
        }
      }

      // Nothing in the requested family fits. Re-take the configuration that was
      // live on entry rather than leaving the panel without staging: it fitted
      // moments ago against the same heap, and a smaller-than-requested pipeline
      // renders correctly — it is only slower per flush.
      if (previousRows > 0 && previousDepth > 0 &&
          tryConfigureFlushPipelineCandidate(previousRows, previousDepth, targetRows, targetDepth) == ESP_OK)
      {
        ESP_LOGW(kTag, "LCD flush pipeline kept at %d rows, %d-deep pipeline (requested %d x %d)",
                 previousRows, previousDepth, targetRows, targetDepth);
        return ESP_OK;
      }

      ESP_LOGE(kTag, "Failed to allocate LCD flush buffers");
      return ESP_ERR_NO_MEM;
    }

    bool openColorStream(int x0, int y0, int x1, int y1, const char *operation)
    {
      const int64_t setWindowStartUs = esp_timer_get_time();
      const esp_err_t err = panel().setWindow(x0, y0, x1, y1);
      flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
      if (err != ESP_OK)
      {
        ESP_LOGW(kTag,
                 "CO5300 %s stream setup failed: %s, falling back to draw_bitmap",
                 operation ? operation : "color",
                 esp_err_to_name(err));
        return false;
      }
      return true;
    }

    bool canDeferFramebufferRectChunkDrain(int x0, int width, int totalRows, bool waitAtEnd) const
    {
      // Defer completion only for the large full-width framebuffer stream. Tiny
      // retained-tree dirty rects should use the historical CO5300 behavior:
      // set a window, send that rect, drain it locally, then move to the next
      // window. Deferring those one-chunk rects just shifts the same wait into
      // the next rect's entry drain and makes the frame look like panel idle.
      return canUseFramebufferColorStream(x0, width, totalRows, waitAtEnd) &&
             x0 == 0 &&
             width == logicalDisplayWidth() &&
             totalRows > flushChunkRows_;
    }

    bool canUseFramebufferColorStream(int x0, int width, int totalRows, bool /*waitAtEnd*/) const
    {
      if (flushQueueDepth_ < 2)
        return false;
      if (totalRows <= flushChunkRows_)
        return false;
      // Stream any WIDE, multi-chunk-TALL region as one CS-held window (window set once,
      // RAMWR + RAMWRC continuations), so its chunks pipeline — the raster/copy of chunk
      // N+1 overlaps chunk N's DMA — instead of the per-chunk drain that stacks raster on
      // top of the wire. This used to require x0==0 && width==kWidth, but that rejected the
      // bouncing-balls united dirty-bbox on every frame the leftmost ball had drifted off
      // column 0 (i.e. almost always), silently dropping it to the serial draining path at
      // ~13ms vs the ~10.4ms wire floor. Streaming is safe for any windowed rect: setWindow
      // addresses [x0..x1] and RAMWRC wraps x1→x0 per row regardless of x0, and the raster
      // fills the whole chunk. Mirror canPipelinePresentRegion, which was already relaxed to
      // width-only for exactly this reason (the centered honeycomb starts mid-screen). Narrow
      // partial-width flushes (e.g. the ~42-row FPS-text region) stay on the serial path via
      // the totalRows>flushChunkRows_ gate above.
      (void)x0;
      return width >= (logicalDisplayWidth() * 3) / 4;
    }

    // A full-width region flushes as one streamed window (window set once, RAMWRC
    // continue), so its chunks pipeline safely without a per-chunk drain — independent
    // of what is drawn in it. Narrow/partial-width regions re-address per chunk and must
    // drain. (Previously this also required the circle frame pattern; the gating is the
    // full-width stream shape, not the command types.)
    bool canPipelinePresentRegion(const present::Rect &region, int width, int totalRows) const
    {
      if (flushQueueDepth_ < 2)
        return false;
      if (totalRows <= flushChunkRows_)
        return false;
      // Pipeline (overlap raster with DMA, drain once at the region tail) for any WIDE,
      // TALL region — not just exactly-full-width ones. A full-screen canvas pan (the
      // blessed clear+circles fast path) dirties ~the whole screen, but the honeycomb is
      // CENTERED so the merged band starts mid-screen (x0~10–100), not at column 0. The old
      // strict `x0==0 && width==kWidth` check rejected that, forcing the per-chunk serial
      // drain that stacks ~5ms of raster on top of the DMA instead of hiding it underneath.
      // Pipelining safety is about the content (full-chunk clear + circles) and width, not
      // where the band starts, so gate on width only. The `totalRows > flushChunkRows_`
      // check above already keeps genuinely small partial flushes (e.g. the FPS-text
      // region, ~42 rows) on the safe serial path.
      return width >= (platform_display::kWidth * 3) / 4;
    }

    // Raster one present chunk into `buffer` — the fill work presentRegion used to
    // do inline, extracted so the cross-core worker can run it on the idle core
    // while the present task streams the previous chunk. presentScale_==2 fills a
    // half-size scratch and 2x-upscales; else rasters full-res. Touches only
    // `buffer` (+ a function-static half-scratch used by one core at a time), no
    // SPI — safe to run on the worker core alongside the present task's streaming.
    void rasterPresentChunkInto(std::uint16_t *buffer, int row, int chunkRows, int width,
                                const present::Frame &frame, std::size_t presentCommandCount, int regionX0)
    {
      if (presentScale_ == 2)
      {
        alignas(16) static std::uint16_t s_halfScratch[(platform_display::kWidth / 2) *
                                                       (GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX / 2 + 2)];
        const int hw = width / 2;
        const int hRows = chunkRows / 2;
        const int hRow = row / 2;
        gea::framework::graphics::Canvas __halfCanvas;
        __halfCanvas.bindPixels(s_halfScratch, hw, hRows);
        const int __hChunkY1 = hRow + hRows - 1;
        for (std::size_t __ci = 0; __ci < presentCommandCount; __ci++)
        {
          if (presentCmdRowY1_[__ci] < hRow || presentCmdRowY0_[__ci] > __hChunkY1)
            continue;
          present::rasterCommandRows(__halfCanvas, s_halfScratch, 0, hw, hRow, hRows,
                                     platform_display::kHeight / 2, frame.commands[__ci]);
        }
        for (int __hy = 0; __hy < hRows; __hy++)
        {
          const std::uint16_t *__src = s_halfScratch + __hy * hw;
          std::uint32_t *__d0 = reinterpret_cast<std::uint32_t *>(buffer + (2 * __hy) * width);
          std::uint32_t *__d1 = reinterpret_cast<std::uint32_t *>(buffer + (2 * __hy + 1) * width);
          for (int __hx = 0; __hx < hw; __hx++)
          {
            const std::uint32_t __c = __src[__hx];
            const std::uint32_t __cc = __c | (__c << 16);
            __d0[__hx] = __cc;
            __d1[__hx] = __cc;
          }
        }
      }
      else
      {
        gea::framework::graphics::Canvas __rasterCanvas;
        __rasterCanvas.bindPixels(buffer, width, chunkRows);
        const int __chunkY1 = row + chunkRows - 1;
        for (std::size_t __ci = 0; __ci < presentCommandCount; __ci++)
        {
          if (presentCmdRowY1_[__ci] < row || presentCmdRowY0_[__ci] > __chunkY1)
            continue;
          const present::Command &__cmd = frame.commands[__ci];
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
          const int64_t __cs = esp_timer_get_time();
#endif
          present::rasterCommandRows(__rasterCanvas, buffer, regionX0, width, row, chunkRows,
                                     platform_display::kHeight, __cmd);
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
          const int64_t __cd = esp_timer_get_time() - __cs;
          using __Ct = platform_display::DisplayPresentCommandType;
          if (__cmd.type == __Ct::Clear || __cmd.type == __Ct::FillRectRgb565 || __cmd.type == __Ct::StrokeRectRgb565)
          {
            splitRasterClearUs_ += __cd;
            splitRasterClearN_++;
          }
          else
          {
            splitRasterFillUs_ += __cd;
            splitRasterFillN_++;
          }
#endif
        }
      }
    }

    // Worker-core job for cross-core chunk raster. The trampoline matches the
    // gea_render_parallel_submit fn signature and forwards to rasterPresentChunkInto.
    struct ChunkRasterJob {
      DisplayBackend *self;
      std::uint16_t *buffer;
      const present::Frame *frame;
      std::size_t cmdCount;
      int row, chunkRows, width, regionX0;
    };
    static void chunkRasterTrampoline(void *p, int /*y0*/, int /*y1*/)
    {
      auto *j = static_cast<ChunkRasterJob *>(p);
      j->self->rasterPresentChunkInto(j->buffer, j->row, j->chunkRows, j->width, *j->frame,
                                      j->cmdCount, j->regionX0);
    }

    bool presentRegion(const present::Rect &dirtyRegion, const present::Frame &frame)
    {
      present::Rect region = present::clampAndAlign(dirtyRegion, platform_display::kWidth, platform_display::kHeight);
      if (!present::valid(region))
        return true;
      // CO5300 partial flush windows can drop an edge pixel on the panel (not in
      // the framebuffer). Pad one alignment unit around the dirty content so both
      // horizontal and vertical motion land that dropped edge on already-correct
      // surrounding pixels. rasterFrameRows fills the padded area from the current
      // frame commands, so the widened window stays pixel-correct.
      region = padPartialFlushWindowForCO5300(region);
      const int width = region.x1 - region.x0 + 1;
      const int totalRows = region.y1 - region.y0 + 1;
      const bool drainPresentChunk = !canPipelinePresentRegion(region, width, totalRows);
      // [region.split] capture the dirty-region geometry + whether it pipelined, so we can
      // see WHY a near-full-screen pan serializes (per-chunk drain) instead of overlapping.
      splitRegionX0_ = region.x0;
      splitRegionW_ = width;
      splitRegionRows_ = totalRows;
      splitDrainChunk_ = drainPresentChunk ? 1 : 0;

#if GEA_EMBEDDED_DISPLAY_CO5300_PRESENT_CS_HELD_STREAM
      bool csHeldStream = false;
      auto closePresentCsStream = [&]()
      {
        if (csHeldStream)
        {
          panel().endColorStream();
          csHeldStream = false;
        }
      };
      // CS-held data-only stream for any full-region present (both the gea3d
      // half-res path and full-res canvas apps). Per-chunk draw_bitmap/tx_color
      // drains ALL in-flight DMA before its command, which serializes the chunk
      // raster against the wire (~18MB/s effective, measured overlap=0). The
      // CS-held stream frames window+RAMWR ONCE then queues each chunk as a
      // data-only continuation — no mid-stream command drains the queue, so a
      // chunk's raster overlaps the previous chunk's DMA and the flush approaches
      // the 40MB/s wire floor. The historic CO5300 wedge (a command issued behind
      // in-flight color DMA) cannot occur: the only command is the opening RAMWR
      // on an idle queue.
      // NOTE: the autonomous stream (beginColorStreamAsync, co5300_async_stream.inc)
      // is WIP — esp_lcd rejects a no-acquire queue_trans (INVALID_ARG), so we
      // stay on the working acquired-bus CS-held stream here for now.
      if (!drainPresentChunk && totalRows > flushChunkRows_)
      {
        setFlushStage(FlushStage::PresentSetWindow, 0);
        const int64_t setWindowStartUs = esp_timer_get_time();
        csHeldStream = panel().setWindow(region.x0, region.y0, region.x1, region.y1) == ESP_OK &&
                       panel().beginColorStream(LCD_CMD_RAMWR) == ESP_OK;
        const int64_t windowDeltaUs = esp_timer_get_time() - setWindowStartUs;
        flushStats_.setWindowUs += windowDeltaUs;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        splitWindowUs_ += windowDeltaUs;
#endif
        if (!csHeldStream)
          ESP_LOGW(kTag, "CO5300 present CS-held stream setup failed, falling back to draw_bitmap");
      }
#else
      // The macro is off, so the CS-held present stream never runs. csHeldStream is
      // still referenced unconditionally by the gea3d full-frame PSRAM path below
      // (dead code while kGea3dTryFullFramePsram is false, but still compiled), so
      // it must be in scope here as a compile-time false.
      constexpr bool csHeldStream = false;
      auto closePresentCsStream = []() {};
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
      bool streamFlush = false;
      if (totalRows > flushChunkRows_)
      {
#if GEA_EMBEDDED_DISPLAY_CO5300_RAMWRC_CONTINUE
        setFlushStage(FlushStage::PresentSetWindow, 0);
        const int64_t setWindowStartUs = esp_timer_get_time();
        streamFlush = panel().setWindow(region.x0, region.y0, region.x1, region.y1) == ESP_OK;
        flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
        if (!streamFlush)
          ESP_LOGW(kTag, "CO5300 present window setup failed, falling back to draw_bitmap");
#else
        streamFlush = openColorStream(region.x0, region.y0, region.x1, region.y1, "present");
#endif
      }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH && GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
      int streamChunksSincePace = 0;
#endif
      // Hoist each command's row extent out of the chunk loop: the replay
      // visits every command once per chunk, and at ~360 commands x ~19
      // chunks the extent switch inside rasterCommandRows (type dispatch +
      // field loads) dominates the rejected visits. One pass here, then a
      // two-compare skip per (chunk, command).
      const std::size_t presentCommandCount = frame.commands.size();
      presentCmdRowY0_.resize(presentCommandCount);
      presentCmdRowY1_.resize(presentCommandCount);
      for (std::size_t ci = 0; ci < presentCommandCount; ci++)
      {
        int cy0 = 0;
        int cy1 = platform_display::kHeight - 1;
        present::commandRowExtent(frame.commands[ci], platform_display::kHeight, cy0, cy1);
        presentCmdRowY0_[ci] = cy0;
        presentCmdRowY1_[ci] = cy1;
      }
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      // TEMP [chunk.probe] (gea3d present-path work): arm per-chunk kick/done
      // capture for the first few pipelined full-width regions, but only after
      // the [wire.probe] runs are done (their mid-loop chunk-0 drains would
      // serialize the pipeline). Pairing each chunk's kick timestamp with its
      // DMA-completion timestamp (stamped in flushDoneCallback) reveals whether
      // chunk N+1's raster overlapped chunk N's DMA (kick[N+1] < done[N]) or the
      // pipeline serialized (kick[N+1] > done[N]) — i.e. exactly where the
      // missing raster/DMA overlap went.
      static int s_splitProbeRuns = 0;
      const bool splitProbeThisRegion = s_splitProbeRuns < 3 && wireProbeRuns_ >= 3 &&
                                        !drainPresentChunk && totalRows > flushChunkRows_;
      if (splitProbeThisRegion)
      {
        splitKickN_ = 0;
        splitDoneN_ = 0;
        splitProbeArm_ = true;
      }
#endif
      // gea3d full-frame continuous-DMA path (presentScale_==2, CS-held): raster
      // the ENTIRE upscaled frame into one 1MB-aligned PACKED PSRAM buffer (cross-
      // core), then queue it as ONE CS-held stream. queueColorStreamData splits it
      // into 32KB sub-transactions and the SPI-done ISR chains them back-to-back,
      // so the wire runs continuously (~40MB/s) instead of stalling one chunk per
      // raster. Every other present path takes the serial loop below.
      if (kGea3dTryFullFramePsram && presentScale_ == 2 && csHeldStream && !gea3dStreamFrame_)
      {
        // Round the buffer up to 64 bytes so the tail sub-transaction's length is
        // 64-aligned (a full frame's byte count usually isn't) — direct PSRAM DMA
        // needs addr AND len 64-aligned. The few pad pixels stream past the window.
        std::size_t allocBytes = static_cast<std::size_t>(platform_display::kWidth) *
                                 platform_display::kHeight * sizeof(std::uint16_t);
        allocBytes = (allocBytes + 63) & ~static_cast<std::size_t>(63);
        const std::size_t freeInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const std::size_t largeInt = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        const std::size_t freeDma = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        const std::size_t largeDma = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_LOGW(kTag, "gea3d full-frame: need=%u  internal free=%u largest=%u  DMA free=%u largest=%u",
                 static_cast<unsigned>(allocBytes), static_cast<unsigned>(freeInt),
                 static_cast<unsigned>(largeInt), static_cast<unsigned>(freeDma),
                 static_cast<unsigned>(largeDma));
        // Internal SRAM is dma_capable; try there first (no PSRAM DMA problem).
        gea3dStreamFrame_ = static_cast<std::uint16_t *>(
            heap_caps_aligned_alloc(64, allocBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
        if (!gea3dStreamFrame_)
          ESP_LOGE(kTag, "gea3d full-frame: 411KB internal alloc FAILED (largest DMA block=%u)",
                   static_cast<unsigned>(largeDma));
        if (gea3dStreamFrame_)
          std::memset(gea3dStreamFrame_, 0, allocBytes);
        if (!gea3dStreamFrame_)
          ESP_LOGE(kTag, "gea3d full-frame PSRAM alloc failed; using chunked path");
      }
      const bool fullFrame = kGea3dTryFullFramePsram && presentScale_ == 2 && csHeldStream && gea3dStreamFrame_ != nullptr;
      if (fullFrame)
      {
        const int frameW = width;
        const int frameH = region.y1 - region.y0 + 1;
        const std::size_t framePixBytes = static_cast<std::size_t>(frameW) * frameH * sizeof(std::uint16_t);
        // Stream a 64-aligned length (pad pixels are zero, past the panel window).
        const std::size_t frameBytes = (framePixBytes + 63) & ~static_cast<std::size_t>(63);
        // 1. Raster every chunk into the packed frame (stride == frameW); cross-core.
        for (int row = region.y0; row <= region.y1; row += flushChunkRows_)
        {
          int chunkRows = flushChunkRows_;
          if (row + chunkRows > region.y1 + 1)
            chunkRows = region.y1 - row + 1;
          if (chunkRows <= 0)
            break;
          std::uint16_t *dst = gea3dStreamFrame_ + static_cast<std::size_t>(row - region.y0) * frameW;
          setFlushStage(FlushStage::PresentRaster, row);
          ChunkRasterJob job{this, dst, &frame, presentCommandCount, row, chunkRows, frameW, region.x0};
          const bool submitted = gea_render_parallel_submit(chunkRasterTrampoline, &job, 0, 0);
          if (!submitted)
            rasterPresentChunkInto(dst, row, chunkRows, frameW, frame, presentCommandCount, region.x0);
          if (submitted)
            gea_render_parallel_wait_blocking();
        }
        // 2. Flush the CPU's writes out of cache so the DMA reads real pixels.
        esp_cache_msync(gea3dStreamFrame_, frameBytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
        // 3. One continuous CS-held stream of the whole frame; take a slot for the
        //    completion callback (the shared region tail below spins until it returns).
        setFlushStage(FlushStage::PresentTx, region.y0);
        const int64_t txStart = esp_timer_get_time();
        xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000));
        const esp_err_t err = panel().queueColorStreamData(gea3dStreamFrame_, frameBytes,
                                                           /*keepCsActiveAfter=*/false,
                                                           /*signalCompletion=*/true);
        flushStats_.chunkCount++;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        splitTxKickUs_ += esp_timer_get_time() - txStart;
        splitTxKickN_++;
#endif
        if (err != ESP_OK)
        {
          ESP_LOGE(kTag, "gea3d full-frame stream failed: %s", esp_err_to_name(err));
          xSemaphoreGive(flushSlots_);
          closePresentCsStream();
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
      }
      else
      for (int row = region.y0; row <= region.y1; row += flushChunkRows_)
      {
        int chunkRows = flushChunkRows_;
        if (row + chunkRows > region.y1 + 1)
          chunkRows = region.y1 - row + 1;
        if (chunkRows <= 0)
          break;

        const int pixelCount = width * chunkRows;
        setFlushDetail(region.x0, row, region.x1, row + chunkRows - 1, chunkRows, pixelCount);
        if (pixelCount > flushBufferCapacity_)
        {
          ESP_LOGE(kTag, "LCD present chunk too large: %d > %d", pixelCount, flushBufferCapacity_);
          closePresentCsStream();
          return false;
        }

        const int64_t slotWaitStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::PresentWaitSlot, row);
        if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
          ESP_LOGE(kTag, "LCD present timed out waiting for slot");
          closePresentCsStream();
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
        const int64_t slotWaitDeltaUs = esp_timer_get_time() - slotWaitStartUs;
        flushStats_.slotWaitUs += slotWaitDeltaUs;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        splitSlotWaitUs_ += slotWaitDeltaUs;
#endif

        std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
        flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
        setFlushStage(FlushStage::PresentRaster, row);
        const int64_t rasterStartUs = esp_timer_get_time();
        rasterPresentChunkInto(buffer, row, chunkRows, width, frame, presentCommandCount, region.x0);
        flushStats_.rasterUs += esp_timer_get_time() - rasterStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
        const int64_t byteSwapStartUs = esp_timer_get_time();
        for (int i = 0; i < pixelCount; i++)
        {
          buffer[i] = pixel::byteSwap16(buffer[i]);
        }
        flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

        const int64_t txStartUs = esp_timer_get_time();
        setFlushStage(FlushStage::PresentTx, row);
#if GEA_EMBEDDED_DISPLAY_CO5300_PRESENT_CS_HELD_STREAM
        esp_err_t err;
        if (csHeldStream)
        {
          // Data-only continuation: nothing here sends a panel command, so
          // nothing drains the SPI queue — this chunk's raster already ran
          // while the previous chunk's DMA streamed.
          const bool lastChunk = row + chunkRows > region.y1;
          err = panel().queueColorStreamData(buffer,
                                             pixelCount * sizeof(std::uint16_t),
                                             /*keepCsActiveAfter=*/!lastChunk,
                                             /*signalCompletion=*/true);
        }
        else
        {
          err = panel().drawBitmap(region.x0, row, region.x1 + 1, row + chunkRows, buffer);
        }
#elif GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH
        esp_err_t err;
        if (streamFlush)
        {
          const int command = row == region.y0 ? LCD_CMD_RAMWR : LCD_CMD_RAMWRC;
          err = panel().txColor(command, buffer, pixelCount * sizeof(std::uint16_t));
        }
        else
        {
          err = panel().drawBitmap(region.x0, row, region.x1 + 1, row + chunkRows, buffer);
        }
#else
        esp_err_t err = panel().drawBitmap(region.x0, row, region.x1 + 1, row + chunkRows, buffer);
#endif
        const int64_t txKickDelta = esp_timer_get_time() - txStartUs;
        flushStats_.txUs += txKickDelta;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        splitTxKickUs_ += txKickDelta;
        splitTxKickN_++;
        // [chunk.probe] kick side: the DMA is now in flight (queueColorStreamData
        // returns after enqueue, not completion). done side is stamped in the ISR.
        if (splitProbeArm_ && splitKickN_ < kSplitProbeSlots)
          splitKickTs_[splitKickN_++] = esp_timer_get_time();
#endif
        flushStats_.chunkCount++;
        if (err != ESP_OK)
        {
          ESP_LOGE(kTag, "LCD present failed: %s", esp_err_to_name(err));
          xSemaphoreGive(flushSlots_);
          closePresentCsStream();
          setFlushStage(FlushStage::Idle, 0);
          return false;
        }
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
        // TEMP one-shot wire probe (gea3d present-path work): with the queue
        // empty (region's first chunk), kick→drain isolates one chunk's raw
        // link time. 65.6KB at a true 40MB/s ≈ 1.7ms; ≈3.3ms means the link
        // runs at half rate and per-chunk drains were never the bottleneck.
        {
          if (row == region.y0 && wireProbeRuns_ < 3)
          {
            const int64_t probeStart = esp_timer_get_time();
            if (waitForFlushCompleteSpin())
            {
              const int64_t drainUs = esp_timer_get_time() - probeStart;
              std::printf("[wire.probe] chunk0 %dpx %dB kick=%lldus drain=%lldus total=%lldus\n",
                          pixelCount, pixelCount * static_cast<int>(sizeof(std::uint16_t)),
                          static_cast<long long>(txKickDelta), static_cast<long long>(drainUs),
                          static_cast<long long>(txKickDelta + drainUs));
              wireProbeRuns_++;
            }
          }
        }
#endif

#if GEA_EMBEDDED_DISPLAY_CO5300_STREAM_FLUSH && GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS > 0
        if (streamFlush)
        {
          streamChunksSincePace++;
          const int nextRow = row + chunkRows;
          if (streamChunksSincePace >= GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS && nextRow <= region.y1)
          {
            setFlushStage(FlushStage::PresentWaitComplete, row);
            const int64_t paceWaitStartUs = esp_timer_get_time();
            if (!waitForFlushComplete())
            {
              setFlushStage(FlushStage::Idle, 0);
              return false;
            }
            flushStats_.completeWaitUs += esp_timer_get_time() - paceWaitStartUs;
            streamChunksSincePace = 0;
          }
        }
#endif

        // General canvas presents stay serialized: complex command streams have
        // wedged the CO5300 when drawBitmap/RAMWRC commands were issued behind
        // in-flight color DMA. The full-screen clear + batched-circles path is
        // the older measured fast path, so let it keep queued chunks and drain
        // once at the region tail.
        if (drainPresentChunk)
        {
          setFlushStage(FlushStage::PresentWaitComplete, row);
          const int64_t chunkDrainStartUs = esp_timer_get_time();
          // Spin-wait for this chunk's DMA rather than blocking. A narrow
          // present chunk's DMA is tiny (~tens of us on the wire), but a
          // BLOCKING xSemaphoreTake is woken by the SPI-completion ISR on the
          // SPI host's core while this frame task is pinned to APP_FRAME_TASK_CORE
          // — paying a cross-core reschedule (~700us/chunk). Across the 5-6
          // narrow chunks a canvas-direct app (canvas-3d) flushes per frame, that
          // turned ~1ms of real DMA into ~5-6ms of pure wakeup latency. Busy-poll
          // the slot count instead; it returns within the real DMA time, then
          // falls back to blocking past the 2ms budget. The spin still waits for
          // DMA-idle before the next panel command, so 569873c1's CO5300 TX-wedge
          // serialization is preserved (it polls the same drain condition).
          if (!waitForFlushCompleteSpin())
          {
            setFlushStage(FlushStage::Idle, 0);
            return false;
          }
          flushStats_.completeWaitUs += esp_timer_get_time() - chunkDrainStartUs;
        }
      }

      setFlushStage(FlushStage::PresentWaitComplete, region.y0);
      const int64_t waitStartUs = esp_timer_get_time();
      // Region tail: spin for the last queued chunks' DMA (same cross-core
      // rationale as the per-chunk drain above). For a full-width pipelined
      // present the spin busy-polls until the real tail DMA completes (within
      // the 2ms budget); for narrow regions it returns almost immediately.
      if (!waitForFlushCompleteSpin())
      {
        closePresentCsStream();
        setFlushStage(FlushStage::Idle, 0);
        return false;
      }
      // Bus release only after the tail DMA drained: endColorStream drops the
      // acquired bus without waiting for in-flight transactions itself.
      closePresentCsStream();
      const int64_t completeWaitDeltaUs = esp_timer_get_time() - waitStartUs;
      flushStats_.completeWaitUs += completeWaitDeltaUs;
#if GEA_EMBEDDED_DISPLAY_PRESENT_SPLIT_LOG
      splitCompleteWaitUs_ += completeWaitDeltaUs;
      // [chunk.probe] tail: the region tail drain above guarantees every chunk's
      // done callback has fired, so kick[i]/done[i] pair by index (single SPI
      // channel completes in submission order). wire = one chunk's real DMA time;
      // gap = prevDone->kick latency (raster not overlapped); overlap=1 means
      // chunk i's kick beat chunk i-1's completion (the pipeline is working).
      if (splitProbeThisRegion)
      {
        splitProbeArm_ = false;
        const int pairs = splitKickN_ < splitDoneN_ ? splitKickN_ : splitDoneN_;
        std::printf("[chunk.probe] region rows=%d kicks=%d dones=%d chunkRows=%d\n",
                    region.y1 - region.y0 + 1, splitKickN_, splitDoneN_, flushChunkRows_);
        for (int i = 0; i < pairs; i++)
        {
          const int64_t wireUs = splitDoneTs_[i] - splitKickTs_[i];
          const int64_t gapUs = i > 0 ? splitKickTs_[i] - splitDoneTs_[i - 1] : 0;
          const int overlap = (i > 0 && splitKickTs_[i] < splitDoneTs_[i - 1]) ? 1 : 0;
          std::printf("[chunk.probe]  #%d wire=%lldus gap(prevDone->kick)=%lldus overlap=%d\n",
                      i, static_cast<long long>(wireUs), static_cast<long long>(gapUs), overlap);
        }
        s_splitProbeRuns++;
      }
#endif
      flushStats_.pixelCount += width * (region.y1 - region.y0 + 1);
      setFlushStage(FlushStage::Idle, 0);
      return true;
    }

    void renderCharToFramebuffer(int column, int row, char c)
    {
      if (!frameBuffer_)
        return;
      const std::uint8_t *glyph = gea::framework::graphics::FontRegistry::bitmap8x16().glyphRows(c);
      const int originX = column * platform_display::kConsoleGlyphWidth;
      const int originY = row * platform_display::kConsoleGlyphHeight;
      for (int fontRow = 0; fontRow < platform_display::kConsoleFontHeight; fontRow++)
      {
        const std::uint8_t bits = glyph[fontRow];
        for (int fontBit = 0; fontBit < platform_display::kConsoleFontWidth; fontBit++)
        {
          const std::uint16_t color = (bits & (0x80 >> fontBit)) ? kConsoleForeground : kConsoleBackground;
          for (int sy = 0; sy < platform_display::kConsoleFontScale; sy++)
          {
            const int py = originY + fontRow * platform_display::kConsoleFontScale + sy;
            if (py >= logicalDisplayHeight())
              break;
            for (int sx = 0; sx < platform_display::kConsoleFontScale; sx++)
            {
              const int px = originX + fontBit * platform_display::kConsoleFontScale + sx;
              if (px < logicalDisplayWidth())
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
              {
                if (softwareLandscapeActive())
                  frameBuffer_[(platform_display::kNativeHeight - 1 - px) *
                                   platform_display::kNativeWidth +
                               py] = color;
                else
                  frameBuffer_[py * physicalFramebufferStridePixels() + px] = color;
              }
#else
                frameBuffer_[py * kFramebufferStridePx + px] = color;
#endif
            }
          }
        }
      }
    }

    void scrollUp()
    {
      cursorRow_ = platform_display::kConsoleRows - 1;
      cursorColumn_ = 0;
      canvas_.clear(kConsoleBackground);
      flush();
    }

    void newline()
    {
      cursorColumn_ = 0;
      cursorRow_++;
      if (cursorRow_ >= platform_display::kConsoleRows)
        scrollUp();
    }

    int physicalFramebufferRow(int absoluteRow) const
    {
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
      if (regionH_ > 0 && absoluteRow >= regionY_ && absoluteRow < regionY_ + regionH_)
      {
        int rel = (absoluteRow - regionY_) + scrollOffsetY_;
        rel %= regionH_;
        if (rel < 0)
          rel += regionH_;
        return regionY_ + rel;
      }
#endif
      return absoluteRow;
    }

    int sourceFramebufferRow(const std::uint16_t *source, int absoluteRow) const
    {
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
      // The diagnostic PSRAM source is already in physical panel row order.
      // Do not apply the software-scroll framebuffer row rotation to it.
      if (source == directPsramStaticTxProbe_)
        return absoluteRow;
#endif
      return physicalFramebufferRow(absoluteRow);
    }

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH && GEA_EMBEDDED_PIXEL_PANEL_ENDIAN && GEA_EMBEDDED_DISPLAY_HAS_ESP_CACHE
    bool canDirectFramebufferFlush(const std::uint16_t *source, int x0, int width, int totalRows, bool waitAtEnd) const
    {
      if (directPsramFlushDisabled_)
        return false;
      if (worldPixels_)
        return false;
      if (!canUseFramebufferColorStream(x0, width, totalRows, waitAtEnd))
        return false;
      if (source != frameBuffer_
#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
          && source != backFrameBuffer_
#endif
      )
      {
        return false;
      }
      if (x0 != 0 || width != platform_display::kWidth)
        return false;
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_PADDED_ROW_STREAM
      return kFramebufferStridePx >= platform_display::kWidth;
#else
      return kFramebufferStridePx == platform_display::kWidth;
#endif
    }

    const std::uint16_t *directPsramSourceFor(const std::uint16_t *source) const
    {
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
      if (directPsramStaticTxProbe_)
        return directPsramStaticTxProbe_;
#endif
      return source;
    }

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
    void fillDirectPsramStaticTxProbe()
    {
      if (!directPsramStaticTxProbe_)
        return;

      for (int y = 0; y < platform_display::kHeight; y++)
      {
        for (int x = 0; x < kFramebufferStridePx; x++)
        {
          if (x >= platform_display::kWidth)
          {
            directPsramStaticTxProbe_[y * kFramebufferStridePx + x] = pixel::fromRgb565(0x0000);
            continue;
          }

          const int r = (x * 31) / (platform_display::kWidth - 1);
          const int g = (y * 63) / (platform_display::kHeight - 1);
          const int checker = ((x / 16) ^ (y / 16)) & 1;
          const int b = checker ? 31 : ((x + y) & 31);
          const std::uint16_t rgb565 = static_cast<std::uint16_t>((r << 11) | (g << 5) | b);
          directPsramStaticTxProbe_[y * kFramebufferStridePx + x] = pixel::fromRgb565(rgb565);
        }
      }
    }
#endif

    DirectFlushResult flushFramebufferRectDirect(const std::uint16_t *source, int x0, int y0, int x1, int y1)
    {
      bool sentAny = false;
      int directPixelsSent = 0;
      const int width = x1 - x0 + 1;
      int row = y0;

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
      bool colorStreamOpen = false;
      auto closeColorStream = [&]()
      {
        if (colorStreamOpen)
        {
          panel().endColorStream();
          colorStreamOpen = false;
        }
      };
#else
      auto closeColorStream = []() {};
#endif
      auto finishDirect = [&](DirectFlushResult result)
      {
        closeColorStream();
        setFlushStage(FlushStage::Idle, 0);
        return result;
      };

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
      setFlushStage(FlushStage::SetWindow, 0);
      const int64_t setWindowStartUs = esp_timer_get_time();
      const esp_err_t windowErr = panel().setWindow(x0, y0, x1, y1);
      flushStats_.setWindowUs += esp_timer_get_time() - setWindowStartUs;
      if (windowErr != ESP_OK)
      {
        ESP_LOGW(kTag, "LCD direct PSRAM stream window setup failed: %s; falling back to staging buffer", esp_err_to_name(windowErr));
        return finishDirect(DirectFlushResult::Fallback);
      }
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM
      setFlushStage(FlushStage::Tx, y0);
      const int64_t streamStartUs = esp_timer_get_time();
      const esp_err_t streamErr = panel().beginColorStream(LCD_CMD_RAMWR);
      flushStats_.txUs += esp_timer_get_time() - streamStartUs;
      if (streamErr != ESP_OK)
      {
        ESP_LOGW(kTag, "LCD direct PSRAM CS-held stream start failed: %s; falling back to staging buffer", esp_err_to_name(streamErr));
        return finishDirect(DirectFlushResult::Fallback);
      }
      colorStreamOpen = true;
#endif
#endif

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS > 0 && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
      int streamChunksSincePace = 0;
#endif

      while (row <= y1)
      {
        int chunkRows = flushChunkRows_;
        if (row + chunkRows > y1 + 1)
          chunkRows = y1 - row + 1;
        if (chunkRows <= 0)
          break;

        int spanRows = 1;
        const int physRow = sourceFramebufferRow(source, row);
        while (spanRows < chunkRows)
        {
          const int nextLogicalRow = row + spanRows;
          const int nextPhysRow = sourceFramebufferRow(source, nextLogicalRow);
          if (nextPhysRow != physRow + spanRows)
            break;
          spanRows++;
        }

        const int pixelCount = width * spanRows;
        setFlushDetail(x0, row, x1, row + spanRows - 1, spanRows, pixelCount);

        const int rowBytes = width * static_cast<int>(sizeof(std::uint16_t));
        const bool sourceRowsAreContiguous = kFramebufferStridePx == width;
        const int maxRowsPerDirectSegment =
            sourceRowsAreContiguous ? (kDirectPsramMaxTransferBytes / rowBytes) : 0;
        if (sourceRowsAreContiguous && maxRowsPerDirectSegment <= 0)
        {
          ESP_LOGW(kTag, "LCD direct PSRAM transfer cap too small for one row; falling back to staging buffer");
          directPsramFlushDisabled_ = true;
          return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
        }

        auto drainPendingDirect = [&](int drainRow) -> bool
        {
          if (!sentAny)
            return true;
          setFlushStage(FlushStage::WaitComplete, drainRow);
          const int64_t waitStartUs = esp_timer_get_time();
          if (!waitForFlushComplete())
          {
            return false;
          }
          flushStats_.completeWaitUs += esp_timer_get_time() - waitStartUs;
          sentAny = false;
          return true;
        };
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
        (void)drainPendingDirect;
#else
        auto sendStagedExactRows = [&](int stagedRow, int stagedRows) -> bool
        {
          if (stagedRows <= 0)
            return true;
          if (!drainPendingDirect(stagedRow))
            return false;

          const int stagedPixels = width * stagedRows;
          if (stagedPixels > flushBufferCapacity_)
          {
            ESP_LOGE(kTag, "LCD direct staged exact chunk too large: %d > %d", stagedPixels, flushBufferCapacity_);
            return false;
          }

          const int64_t slotWaitStartUs = esp_timer_get_time();
          setFlushStage(FlushStage::WaitSlot, stagedRow);
          if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
          {
            ESP_LOGE(kTag, "LCD direct staged exact stream timed out waiting for slot");
            return false;
          }
          flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

          std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
          flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
          setFlushStage(FlushStage::Copy, stagedRow);
          const int64_t copyStartUs = esp_timer_get_time();
          copyFlushRows(buffer, source, x0, width, stagedRow, stagedRows);
          flushStats_.copyUs += esp_timer_get_time() - copyStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
          const int64_t byteSwapStartUs = esp_timer_get_time();
          for (int i = 0; i < stagedPixels; i++)
          {
            buffer[i] = pixel::byteSwap16(buffer[i]);
          }
          flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

          setFlushStage(FlushStage::SetWindow, stagedRow);
          const int64_t windowStartUs = esp_timer_get_time();
          const esp_err_t windowErr =
              panel().setWindow(x0, stagedRow, x1, stagedRow + stagedRows - 1);
          flushStats_.setWindowUs += esp_timer_get_time() - windowStartUs;
          if (windowErr != ESP_OK)
          {
            ESP_LOGW(kTag, "LCD direct staged exact window setup failed: %s", esp_err_to_name(windowErr));
            xSemaphoreGive(flushSlots_);
            directPsramFlushDisabled_ = true;
            return false;
          }

          setFlushStage(FlushStage::Tx, stagedRow);
          const int64_t txStartUs = esp_timer_get_time();
          const esp_err_t err =
              panel().txColor(LCD_CMD_RAMWR, buffer, static_cast<std::size_t>(stagedPixels) * sizeof(std::uint16_t));
          flushStats_.txUs += esp_timer_get_time() - txStartUs;
          flushStats_.chunkCount++;
          if (err != ESP_OK)
          {
            ESP_LOGW(kTag, "LCD direct staged exact stream rejected: %s", esp_err_to_name(err));
            xSemaphoreGive(flushSlots_);
            directPsramFlushDisabled_ = true;
            return false;
          }

          sentAny = true;
          directPixelsSent += stagedPixels;
          return true;
        };
#endif

        int spanRowsSent = 0;
        while (spanRowsSent < spanRows)
        {
          const int segmentRow = row + spanRowsSent;
          const int segmentPhysRow = physRow + spanRowsSent;

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STAGED_PREFIX_ROWS > 0 && \
    !GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
          if (segmentRow == y0 && spanRowsSent == 0 && kDirectPsramStagedPrefixRows > 0)
          {
            int stagedRows = kDirectPsramStagedPrefixRows;
            if (stagedRows > spanRows)
              stagedRows = spanRows;
            if (stagedRows > flushChunkRows_)
              stagedRows = flushChunkRows_;
            // Stage the first panel transfer inside the direct flush sequence.
            // Do not call the normal fallback flusher here; mixing two flush
            // engines before the direct PSRAM body can leave the panel black.
            if (!sendStagedExactRows(segmentRow, stagedRows))
              return finishDirect(DirectFlushResult::Failed);
            spanRowsSent += stagedRows;
            continue;
          }
#endif

          if (!sourceRowsAreContiguous)
          {
            int segmentRows = spanRows - spanRowsSent;
            if (segmentRows > flushChunkRows_)
              segmentRows = flushChunkRows_;
            const int segmentPixels = width * segmentRows;
            if (segmentPixels > flushBufferCapacity_)
            {
              ESP_LOGE(kTag, "LCD direct packed chunk too large: %d > %d", segmentPixels, flushBufferCapacity_);
              return finishDirect(DirectFlushResult::Failed);
            }

            const int64_t slotWaitStartUs = esp_timer_get_time();
            setFlushStage(FlushStage::WaitSlot, segmentRow);
            if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
              ESP_LOGE(kTag, "LCD direct packed stream timed out waiting for slot");
              return finishDirect(DirectFlushResult::Failed);
            }
            flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

            std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
            flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
            setFlushStage(FlushStage::Copy, segmentRow);
            const int64_t copyStartUs = esp_timer_get_time();
            copyFlushRows(buffer, source, x0, width, segmentRow, segmentRows);
            flushStats_.copyUs += esp_timer_get_time() - copyStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
            const int64_t byteSwapStartUs = esp_timer_get_time();
            for (int i = 0; i < segmentPixels; i++)
            {
              buffer[i] = pixel::byteSwap16(buffer[i]);
            }
            flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

            setFlushStage(FlushStage::Tx, segmentRow);
            const int64_t txStartUs = esp_timer_get_time();
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
            const bool hasMoreDirectWork = spanRowsSent + segmentRows < spanRows || row + spanRows <= y1;
            const esp_err_t err = panel().queueColorStreamData(buffer,
                                                               static_cast<std::size_t>(segmentPixels) * sizeof(std::uint16_t),
                                                               hasMoreDirectWork,
                                                               /*signalCompletion=*/true);
#else
            const int command = sentAny ? LCD_CMD_RAMWRC : LCD_CMD_RAMWR;
            const esp_err_t err =
                panel().txColor(command, buffer, static_cast<std::size_t>(segmentPixels) * sizeof(std::uint16_t));
#endif
            flushStats_.txUs += esp_timer_get_time() - txStartUs;
            flushStats_.chunkCount++;
            if (err != ESP_OK)
            {
              ESP_LOGW(kTag, "LCD direct packed stream rejected: %s; falling back to staging buffer", esp_err_to_name(err));
              xSemaphoreGive(flushSlots_);
              directPsramFlushDisabled_ = true;
              return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
            }

            sentAny = true;
            directPixelsSent += segmentPixels;
            spanRowsSent += segmentRows;

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS > 0 && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
            streamChunksSincePace++;
            const bool hasMoreWork = spanRowsSent < spanRows || row + spanRows <= y1;
            if (streamChunksSincePace >= GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS && hasMoreWork)
            {
              setFlushStage(FlushStage::WaitComplete, segmentRow);
              const int64_t paceWaitStartUs = esp_timer_get_time();
              if (!waitForFlushComplete())
              {
                return finishDirect(DirectFlushResult::Failed);
              }
              flushStats_.completeWaitUs += esp_timer_get_time() - paceWaitStartUs;
              streamChunksSincePace = 0;
            }
#endif
            continue;
          }

          const auto *directBytes = reinterpret_cast<const std::uint8_t *>(
              &source[segmentPhysRow * kFramebufferStridePx]);

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGN_TRANSFERS
          if ((reinterpret_cast<std::uintptr_t>(directBytes) & (kDirectPsramAlignmentBytes - 1)) != 0)
          {
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
            int stagedRows = 1;
            while (spanRowsSent + stagedRows < spanRows)
            {
              const auto *nextBytes = reinterpret_cast<const std::uint8_t *>(
                  &source[(segmentPhysRow + stagedRows) * kFramebufferStridePx]);
              if ((reinterpret_cast<std::uintptr_t>(nextBytes) & (kDirectPsramAlignmentBytes - 1)) == 0)
                break;
              stagedRows++;
            }
            if (stagedRows > flushChunkRows_)
              stagedRows = flushChunkRows_;

            const int segmentPixels = width * stagedRows;
            if (segmentPixels > flushBufferCapacity_)
            {
              ESP_LOGE(kTag, "LCD direct aligned-prefix chunk too large: %d > %d", segmentPixels, flushBufferCapacity_);
              return finishDirect(DirectFlushResult::Failed);
            }

            const int64_t slotWaitStartUs = esp_timer_get_time();
            setFlushStage(FlushStage::WaitSlot, segmentRow);
            if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
              ESP_LOGE(kTag, "LCD direct aligned-prefix stream timed out waiting for slot");
              return finishDirect(DirectFlushResult::Failed);
            }
            flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

            std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
            flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
            setFlushStage(FlushStage::Copy, segmentRow);
            const int64_t copyStartUs = esp_timer_get_time();
            // WARNING: packed 410px RGB565 rows are 820 bytes, so physical
            // framebuffer row starts are only 64-byte aligned every 16 rows.
            // Do not direct-DMA an unaligned PSRAM row here; it causes the
            // random horizontal colour bands seen during scroll. Stage only
            // the unaligned prefix/tail, then return to aligned PSRAM DMA.
            copyFlushRows(buffer, source, x0, width, segmentRow, stagedRows);
            flushStats_.copyUs += esp_timer_get_time() - copyStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
            const int64_t byteSwapStartUs = esp_timer_get_time();
            for (int i = 0; i < segmentPixels; i++)
            {
              buffer[i] = pixel::byteSwap16(buffer[i]);
            }
            flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

            setFlushStage(FlushStage::Tx, segmentRow);
            const int64_t txStartUs = esp_timer_get_time();
            const bool hasMoreDirectWork = spanRowsSent + stagedRows < spanRows || row + spanRows <= y1;
            const esp_err_t err = panel().queueColorStreamData(buffer,
                                                               static_cast<std::size_t>(segmentPixels) * sizeof(std::uint16_t),
                                                               hasMoreDirectWork,
                                                               /*signalCompletion=*/true);
            flushStats_.txUs += esp_timer_get_time() - txStartUs;
            flushStats_.chunkCount++;
            if (err != ESP_OK)
            {
              ESP_LOGW(kTag, "LCD direct aligned-prefix stream rejected: %s; falling back to staging buffer", esp_err_to_name(err));
              xSemaphoreGive(flushSlots_);
              directPsramFlushDisabled_ = true;
              return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
            }

            sentAny = true;
            directPixelsSent += segmentPixels;
            spanRowsSent += stagedRows;
            continue;
#else
            int stagedRows = 1;
            while (spanRowsSent + stagedRows < spanRows)
            {
              const auto *nextBytes = reinterpret_cast<const std::uint8_t *>(
                  &source[(segmentPhysRow + stagedRows) * kFramebufferStridePx]);
              if ((reinterpret_cast<std::uintptr_t>(nextBytes) & (kDirectPsramAlignmentBytes - 1)) == 0)
                break;
              stagedRows++;
            }

            if (!sendStagedExactRows(segmentRow, stagedRows))
              return finishDirect(DirectFlushResult::Failed);
            spanRowsSent += stagedRows;
            continue;
#endif
          }
#endif

          int segmentRows = spanRows - spanRowsSent;
          if (segmentRows > maxRowsPerDirectSegment)
            segmentRows = maxRowsPerDirectSegment;

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGN_TRANSFERS && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_REQUIRE_LENGTH_ALIGNMENT
          while (segmentRows > 0 &&
                 ((static_cast<std::size_t>(rowBytes) * segmentRows) & (kDirectPsramAlignmentBytes - 1)) != 0)
          {
            segmentRows--;
          }
          if (segmentRows <= 0)
          {
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
            int stagedRows = spanRows - spanRowsSent;
            if (stagedRows > flushChunkRows_)
              stagedRows = flushChunkRows_;

            const int segmentPixels = width * stagedRows;
            if (segmentPixels > flushBufferCapacity_)
            {
              ESP_LOGE(kTag, "LCD direct aligned-tail chunk too large: %d > %d", segmentPixels, flushBufferCapacity_);
              return finishDirect(DirectFlushResult::Failed);
            }

            const int64_t slotWaitStartUs = esp_timer_get_time();
            setFlushStage(FlushStage::WaitSlot, segmentRow);
            if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
              ESP_LOGE(kTag, "LCD direct aligned-tail stream timed out waiting for slot");
              return finishDirect(DirectFlushResult::Failed);
            }
            flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

            std::uint16_t *buffer = flushBuffers_[flushBufferIndex_];
            flushBufferIndex_ = (flushBufferIndex_ + 1) % flushQueueDepth_;
            setFlushStage(FlushStage::Copy, segmentRow);
            const int64_t copyStartUs = esp_timer_get_time();
            // WARNING: keep short aligned tails inside the active RAMWR stream.
            // Calling the normal fallback flush here injects commands while the
            // panel is expecting color bytes, which shows up as random bands.
            copyFlushRows(buffer, source, x0, width, segmentRow, stagedRows);
            flushStats_.copyUs += esp_timer_get_time() - copyStartUs;

#if !GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
            const int64_t byteSwapStartUs = esp_timer_get_time();
            for (int i = 0; i < segmentPixels; i++)
            {
              buffer[i] = pixel::byteSwap16(buffer[i]);
            }
            flushStats_.byteSwapUs += esp_timer_get_time() - byteSwapStartUs;
#endif

            setFlushStage(FlushStage::Tx, segmentRow);
            const int64_t txStartUs = esp_timer_get_time();
            const bool hasMoreDirectWork = spanRowsSent + stagedRows < spanRows || row + spanRows <= y1;
            const esp_err_t err = panel().queueColorStreamData(buffer,
                                                               static_cast<std::size_t>(segmentPixels) * sizeof(std::uint16_t),
                                                               hasMoreDirectWork,
                                                               /*signalCompletion=*/true);
            flushStats_.txUs += esp_timer_get_time() - txStartUs;
            flushStats_.chunkCount++;
            if (err != ESP_OK)
            {
              ESP_LOGW(kTag, "LCD direct aligned-tail stream rejected: %s; falling back to staging buffer", esp_err_to_name(err));
              xSemaphoreGive(flushSlots_);
              directPsramFlushDisabled_ = true;
              return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
            }

            sentAny = true;
            directPixelsSent += segmentPixels;
            spanRowsSent += stagedRows;
            continue;
#else
            const int stagedRows = spanRows - spanRowsSent;
            if (!sendStagedExactRows(segmentRow, stagedRows))
              return finishDirect(DirectFlushResult::Failed);
            spanRowsSent += stagedRows;
            continue;
#endif
          }
#endif

          const std::size_t segmentBytes = static_cast<std::size_t>(rowBytes) * segmentRows;

#if !GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
          if (sentAny)
          {
            setFlushStage(FlushStage::WaitComplete, segmentRow);
            const int64_t windowWaitStartUs = esp_timer_get_time();
            if (!waitForFlushComplete())
            {
              return finishDirect(DirectFlushResult::Failed);
            }
            flushStats_.completeWaitUs += esp_timer_get_time() - windowWaitStartUs;
            sentAny = false;
          }
          setFlushStage(FlushStage::SetWindow, segmentRow);
          const int64_t segmentWindowStartUs = esp_timer_get_time();
          const esp_err_t segmentWindowErr =
              panel().setWindow(x0, segmentRow, x1, segmentRow + segmentRows - 1);
          flushStats_.setWindowUs += esp_timer_get_time() - segmentWindowStartUs;
          if (segmentWindowErr != ESP_OK)
          {
            ESP_LOGW(kTag, "LCD direct PSRAM segment window setup failed: %s; falling back to staging buffer", esp_err_to_name(segmentWindowErr));
            directPsramFlushDisabled_ = true;
            return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
          }
#endif

          setFlushStage(FlushStage::Copy, segmentRow);
          const int64_t syncStartUs = esp_timer_get_time();
          // Direct PSRAM-to-SPI DMA must see memory, not a dirty CPU cache
          // line. These spans are already forced to cache-line-aligned address
          // and length above. Do not invalidate here: the framebuffer remains
          // live, and the SPI driver also performs its own C2M sync for DMA.
          const esp_err_t syncErr = esp_cache_msync(const_cast<std::uint8_t *>(directBytes),
                                                    segmentBytes,
                                                    ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
          flushStats_.copyUs += esp_timer_get_time() - syncStartUs;
          if (syncErr != ESP_OK)
          {
            ESP_LOGW(kTag, "LCD direct PSRAM cache sync failed: %s; falling back to staging buffer", esp_err_to_name(syncErr));
            directPsramFlushDisabled_ = true;
            return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
          }

          const int64_t slotWaitStartUs = esp_timer_get_time();
          setFlushStage(FlushStage::WaitSlot, segmentRow);
          if (xSemaphoreTake(flushSlots_, pdMS_TO_TICKS(1000)) != pdTRUE)
          {
            ESP_LOGE(kTag, "LCD direct PSRAM stream timed out waiting for slot");
            return finishDirect(DirectFlushResult::Failed);
          }
          flushStats_.slotWaitUs += esp_timer_get_time() - slotWaitStartUs;

          setFlushStage(FlushStage::Tx, segmentRow);
          const int64_t txStartUs = esp_timer_get_time();
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
          const bool hasMoreDirectWork = spanRowsSent + segmentRows < spanRows || row + spanRows <= y1;
          const esp_err_t err = panel().queueColorStreamData(directBytes,
                                                             segmentBytes,
                                                             hasMoreDirectWork,
                                                             /*signalCompletion=*/true);
#else
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
          const int command = sentAny ? LCD_CMD_RAMWRC : LCD_CMD_RAMWR;
#else
          const int command = LCD_CMD_RAMWR;
#endif
          const esp_err_t err = panel().txColor(command, directBytes, segmentBytes);
#endif
          flushStats_.txUs += esp_timer_get_time() - txStartUs;
          flushStats_.chunkCount++;
          if (err != ESP_OK)
          {
            ESP_LOGW(kTag, "LCD direct PSRAM stream rejected: %s; falling back to staging buffer", esp_err_to_name(err));
            xSemaphoreGive(flushSlots_);
            directPsramFlushDisabled_ = true;
            return finishDirect(sentAny ? DirectFlushResult::Failed : DirectFlushResult::Fallback);
          }

          sentAny = true;
          directPixelsSent += width * segmentRows;
          spanRowsSent += segmentRows;

#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS > 0 && \
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE
          streamChunksSincePace++;
          const bool hasMoreWork = spanRowsSent < spanRows || row + spanRows <= y1;
          if (streamChunksSincePace >= GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS && hasMoreWork)
          {
            setFlushStage(FlushStage::WaitComplete, segmentRow);
            const int64_t paceWaitStartUs = esp_timer_get_time();
            if (!waitForFlushComplete())
            {
              return finishDirect(DirectFlushResult::Failed);
            }
            flushStats_.completeWaitUs += esp_timer_get_time() - paceWaitStartUs;
            streamChunksSincePace = 0;
          }
#endif
        }

        row += spanRows;
      }

      if (sentAny)
      {
        setFlushStage(FlushStage::WaitComplete, y1);
        const int64_t waitStartUs = esp_timer_get_time();
        if (!waitForFlushComplete())
        {
          return finishDirect(DirectFlushResult::Failed);
        }
        flushStats_.completeWaitUs += esp_timer_get_time() - waitStartUs;
      }

      flushStats_.pixelCount += directPixelsSent;
      closeColorStream();

      return finishDirect(DirectFlushResult::Done);
    }
#endif

    void copyFlushRows(std::uint16_t *buffer, const std::uint16_t *source, int x0, int width, int row, int chunkRows)
    {
      // The framebuffer is laid out with row stride = kFramebufferStridePx
      // while the SPI staging buffer is packed at row stride = width (the
      // dirty-rect width). Full-width packed framebuffers can be copied as
      // contiguous spans; padded or partial-width regions still copy per row.
      //
      // When the software-scroll register is active, the panel row we're
      // assembling for the SPI tx (logical row `absoluteRow`) lives at a
      // rotated physical FB row. Translate per row; wrap is handled naturally
      // because each row's source pointer is computed independently.
      const int logicalWidth = logicalDisplayWidth();
      const int framebufferStride = physicalFramebufferStridePixels();
      if (!worldPixels_ && x0 == 0 && width == logicalWidth && framebufferStride == logicalWidth)
      {
        int ry = 0;
        while (ry < chunkRows)
        {
          const int absoluteRow = row + ry;
          const int physRow = sourceFramebufferRow(source, absoluteRow);
          int spanRows = 1;
          while (ry + spanRows < chunkRows)
          {
            const int nextAbsoluteRow = absoluteRow + spanRows;
            const int nextPhysRow = sourceFramebufferRow(source, nextAbsoluteRow);
            if (nextPhysRow != physRow + spanRows)
              break;
            spanRows++;
          }
          std::memcpy(&buffer[ry * width],
                      &source[physRow * framebufferStride],
                      static_cast<std::size_t>(width) * spanRows * sizeof(std::uint16_t));
          ry += spanRows;
        }
        return;
      }

      for (int ry = 0; ry < chunkRows; ry++)
      {
        const int absoluteRow = row + ry;
        if (worldPixels_ && absoluteRow >= worldPanelTop_ && absoluteRow < worldPanelTop_ + worldPanelHeight_)
        {
          const int worldRow = absoluteRow - worldPanelTop_;
          int sourceX = x0 + worldScrollX_;
          if (sourceX < 0)
            sourceX = 0;
          if (sourceX + width > worldWidth_)
            sourceX = worldWidth_ - width;
          if (sourceX < 0)
            sourceX = 0;
          std::memcpy(&buffer[ry * width], &worldPixels_[worldRow * worldWidth_ + sourceX], width * sizeof(std::uint16_t));
        }
        else
        {
          const int physRow = sourceFramebufferRow(source, absoluteRow);
          std::memcpy(&buffer[ry * width],
                      &source[physRow * framebufferStride + x0],
                      width * sizeof(std::uint16_t));
        }
      }
    }

#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
    static bool IRAM_ATTR asyncMemcpyDoneCb(async_memcpy_handle_t mcp,
                                            async_memcpy_event_t *event,
                                            void *cb_args)
    {
      (void)mcp;
      (void)event;
      auto *self = static_cast<DisplayBackend *>(cb_args);
      if (!self || !self->asyncMemcpyDoneSem_)
        return false;
      BaseType_t needYield = pdFALSE;
      xSemaphoreGiveFromISR(self->asyncMemcpyDoneSem_, &needYield);
      return needYield == pdTRUE;
    }

    bool asyncMemcpySync(void *dst, const void *src, std::size_t bytes)
    {
      if (!asyncMemcpy_ || !asyncMemcpyDoneSem_)
        return false;
      if (!asyncMemcpyTransferAligned(dst, src, bytes))
        return false;
      const esp_err_t err = esp_async_memcpy(asyncMemcpy_,
                                             dst,
                                             const_cast<void *>(src),
                                             bytes,
                                             &DisplayBackend::asyncMemcpyDoneCb,
                                             this);
      if (err != ESP_OK)
      {
        ESP_LOGW(kTag, "async_memcpy kick failed: %s (size=%zu)", esp_err_to_name(err), bytes);
        return false;
      }
      if (xSemaphoreTake(asyncMemcpyDoneSem_, pdMS_TO_TICKS(200)) != pdTRUE)
      {
        ESP_LOGE(kTag, "async_memcpy timeout (size=%zu)", bytes);
        return false;
      }
      return true;
    }

    static bool asyncMemcpyTransferAligned(const void *dst, const void *src, std::size_t bytes)
    {
      if (kAsyncMemcpyBurstBytes == 0)
        return true;
      const std::uintptr_t mask = kAsyncMemcpyBurstBytes - 1;
      return ((reinterpret_cast<std::uintptr_t>(dst) | reinterpret_cast<std::uintptr_t>(src) | bytes) & mask) == 0;
    }

    void scrollRectCrossBuffer(const std::uint16_t *src, std::uint16_t *dst,
                               int x, int y, int w, int h, int dx, int dy)
    {
      if (!src || !dst || w <= 0 || h <= 0)
        return;
      if (dx == 0 && dy == 0)
        return;

      int cx0;
      int cy0;
      int cx1;
      int cy1;
      canvas_.currentClip(&cx0, &cy0, &cx1, &cy1);

      int x0 = x;
      int y0 = y;
      int x1 = x + w - 1;
      int y1 = y + h - 1;
      if (x0 < cx0)
        x0 = cx0;
      if (y0 < cy0)
        y0 = cy0;
      if (x1 > cx1)
        x1 = cx1;
      if (y1 > cy1)
        y1 = cy1;
      if (x0 < 0)
        x0 = 0;
      if (y0 < 0)
        y0 = 0;
      if (x1 >= platform_display::kWidth)
        x1 = platform_display::kWidth - 1;
      if (y1 >= platform_display::kHeight)
        y1 = platform_display::kHeight - 1;
      if (x0 > x1 || y0 > y1)
        return;

      const int rect_w = x1 - x0 + 1;
      const int rect_h = y1 - y0 + 1;
      if (dx <= -rect_w || dx >= rect_w)
        return;
      if (dy <= -rect_h || dy >= rect_h)
        return;

      const int rowStride = kFramebufferStridePx;

      // Fast path: full-panel-width vertical scroll. When dx == 0 and the
      // rect covers all kWidth columns starting at x0 == 0, the valid
      // destination rows form a contiguous memory range (using the padded
      // stride) shifted from the source. One big memcpy/GDMA does the work.
      //
      // We deliberately copy the FULL row stride (including the unused
      // padding columns past kWidth) — that keeps the source/dest pointers
      // and length 64-byte aligned, which is what GDMA burst=64 needs to
      // accept the transfer. The padding columns never reach the panel
      // because copyFlushRows only emits kWidth pixels per row.
      //
      // The exposed strip (rows where src_row falls outside the rect) is
      // left for the framework's replayDisplayRegion to overdraw.
      if (dx == 0 && x0 == 0 && rect_w >= platform_display::kWidth)
      {
        int valid_dst_y0;
        int valid_src_y0;
        int valid_rows;
        if (dy >= 0)
        {
          valid_dst_y0 = y0 + dy;
          valid_src_y0 = y0;
          valid_rows = rect_h - dy;
        }
        else
        {
          valid_dst_y0 = y0;
          valid_src_y0 = y0 - dy;
          valid_rows = rect_h + dy;
        }
        if (valid_rows > 0)
        {
          const std::size_t bytes =
              static_cast<std::size_t>(valid_rows) * rowStride * sizeof(std::uint16_t);
          std::uint16_t *dstPtr = &dst[valid_dst_y0 * rowStride];
          const std::uint16_t *srcPtr = &src[valid_src_y0 * rowStride];

          bool copied = false;
          if (asyncMemcpy_ && asyncMemcpyDoneSem_)
          {
            copied = asyncMemcpySync(dstPtr, srcPtr, bytes);
          }
          if (!copied)
          {
            std::memcpy(dstPtr, srcPtr, bytes);
          }
        }
        canvas_.markDirty(x0, y0, x1, y1);
        return;
      }

      for (int row = y0; row <= y1; row++)
      {
        const int src_row = row - dy;
        if (src_row < y0 || src_row > y1)
        {
          // Exposed row: caller must redraw — matches Canvas::scrollRect semantics
          // (in single-buffer that strip kept its prior pixels, here it keeps dst's
          // stale content, but the framework's replayDisplayRegion overwrites it
          // immediately after the scrollRect call).
          continue;
        }
        if (dx == 0)
        {
          std::memcpy(&dst[row * rowStride + x0],
                      &src[src_row * rowStride + x0],
                      static_cast<std::size_t>(rect_w) * sizeof(std::uint16_t));
        }
        else if (dx > 0)
        {
          std::memcpy(&dst[row * rowStride + x0 + dx],
                      &src[src_row * rowStride + x0],
                      static_cast<std::size_t>(rect_w - dx) * sizeof(std::uint16_t));
        }
        else
        {
          std::memcpy(&dst[row * rowStride + x0],
                      &src[src_row * rowStride + x0 - dx],
                      static_cast<std::size_t>(rect_w + dx) * sizeof(std::uint16_t));
        }
      }

      canvas_.markDirty(x0, y0, x1, y1);
    }
#endif

    static constexpr const char *kTag = "display";
    static constexpr int kMaxPresentRects = kPresentRectLimit > 0 ? kPresentRectLimit : 1;

    enum class FlushStage : int
    {
      Idle,
      SetWindow,
      WaitSlot,
      Copy,
      Tx,
      WaitComplete,
      PresentSetWindow,
      PresentWaitSlot,
      PresentRaster,
      PresentTx,
      PresentWaitComplete,
      StreamSetWindow,
      StreamWaitSlot,
      StreamRaster,
      StreamTx,
      StreamWaitComplete,
    };

    static const char *stageName(FlushStage stage)
    {
      switch (stage)
      {
      case FlushStage::SetWindow:
        return "set_window";
      case FlushStage::WaitSlot:
        return "wait_slot";
      case FlushStage::Copy:
        return "copy";
      case FlushStage::Tx:
        return "tx";
      case FlushStage::WaitComplete:
        return "wait_complete";
      case FlushStage::PresentSetWindow:
        return "present_set_window";
      case FlushStage::PresentWaitSlot:
        return "present_wait_slot";
      case FlushStage::PresentRaster:
        return "present_raster";
      case FlushStage::PresentTx:
        return "present_tx";
      case FlushStage::PresentWaitComplete:
        return "present_wait_complete";
      case FlushStage::StreamSetWindow:
        return "stream_set_window";
      case FlushStage::StreamWaitSlot:
        return "stream_wait_slot";
      case FlushStage::StreamRaster:
        return "stream_raster";
      case FlushStage::StreamTx:
        return "stream_tx";
      case FlushStage::StreamWaitComplete:
        return "stream_wait_complete";
      case FlushStage::Idle:
        return "idle";
      }
      return "idle";
    }

    void setFlushStage(FlushStage stage, int chunk)
    {
      flushStageChunk_.store(chunk, std::memory_order_release);
      flushStage_.store(stage, std::memory_order_release);
    }

    void setFlushDetail(int x0, int y0, int x1, int y1, int rows, int pixels)
    {
      flushStageX0_.store(x0, std::memory_order_release);
      flushStageY0_.store(y0, std::memory_order_release);
      flushStageX1_.store(x1, std::memory_order_release);
      flushStageY1_.store(y1, std::memory_order_release);
      flushStageRows_.store(rows, std::memory_order_release);
      flushStagePixels_.store(pixels, std::memory_order_release);
    }

    void addPresentDamageRect(int x0, int y0, int x1, int y1)
    {
      present::addRect(externalPresentDamage_,
                       &externalPresentDamageCount_,
                       kMaxPresentRects,
                       present::Rect{x0, y0, x1, y1},
                       logicalDisplayWidth(),
                       logicalDisplayHeight());
    }

    platform_display::DisplayFlushPerfStats flushStats_{};
    std::atomic<std::uint32_t> flushOdometerCalls_{0};
    std::atomic<std::uint64_t> flushOdometerPixels_{0};
    gea::framework::graphics::Canvas canvas_{};
    // 2nd-core band canvas: same framebuffer pixels as canvas_, private clip+dirty.
    gea::framework::graphics::Canvas workerCanvas_{};
    // CPU the main render task runs on; -1 until the worker is created (single-core).
    volatile int renderCoreId_ = -1;
    std::uint16_t *frameBuffer_ = nullptr;
#if GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_REGISTER
    // Mirror of canvas's scroll region. Display owns the source of truth so
    // copyFlushRows can translate panel rows to physical FB rows without
    // reaching into Canvas. Updated by Display::scrollRect when a
    // software-register-eligible scroll happens and propagated to canvas.
    // regionH_ == 0 means the register is inactive (no translation).
    int regionY_ = 0;
    int regionH_ = 0;
    int scrollOffsetY_ = 0;
#endif
#if GEA_EMBEDDED_DISPLAY_DOUBLE_BUFFER
    // Second PSRAM framebuffer for the swap-on-flush pipeline. When the
    // compile flag is off, this stays nullptr and flush() takes the
    // synchronous single-buffer path.
    std::uint16_t *backFrameBuffer_ = nullptr;
    // AHB GDMA backed async_memcpy driver, used by scrollRectCrossBuffer to
    // copy PSRAM→PSRAM at bus rate (~5 ms for a full screen) instead of CPU
    // memcpy through cache (~25 ms). The semaphore is given by the GDMA
    // completion ISR; the caller takes it after kicking the transfer.
    async_memcpy_handle_t asyncMemcpy_ = nullptr;
    SemaphoreHandle_t asyncMemcpyDoneSem_ = nullptr;
#endif
    std::uint16_t *flushBuffers_[kFlushQueueDepthLimit] = {};
    bool flushBuffersFromPool_ = false;
    int flushBufferCapacity_ = 0;
    // gea3d full-frame continuous-DMA path: a 1MB-aligned PACKED (stride==width)
    // PSRAM buffer holding the whole upscaled frame, streamed in one CS-held call
    // so the SPI ISR chains all its 32KB sub-transactions back-to-back. 1MB align
    // dodges the PSRAM/GDMA burst bug (see the frameBuffer_ alloc comment).
    std::uint16_t *gea3dStreamFrame_ = nullptr;
    int flushBufferIndex_ = 0;
    // Per-command row extents, filled once per presentRegion and reused by
    // every flush chunk (see the hoist comment at the chunk loop). Members so
    // the vectors' capacity persists across frames instead of reallocating.
    std::vector<int> presentCmdRowY0_;
    std::vector<int> presentCmdRowY1_;
    int requestedFlushChunkRows_ = kFlushChunkDefault;
    int requestedFlushQueueDepth_ = kFlushQueueDepthDefault;
    int configuredRequestFlushChunkRows_ = 0;
    int configuredRequestFlushQueueDepth_ = 0;
    SemaphoreHandle_t flushSlots_ = nullptr;
    SemaphoreHandle_t teSync_ = nullptr; // released by teEdgeIsr on each TE (VBlank) edge
    bool vsyncEnabled_ = false;          // opt-in (Display.setVSync); gates the in-present VBlank wait
    // [present.split] per-present-path timing accumulators (dumped every 30 frames).
    int64_t splitExtractUs_ = 0, splitDiffUs_ = 0, splitDrainUs_ = 0, splitVsyncUs_ = 0, splitFlushUs_ = 0;
    long long splitPx_ = 0;
    int splitCount_ = 0;
    // Raster sub-split (filled by presentRegion's per-command timed loop): Clear (memset
    // the chunk to black) vs everything else (the circle fills). + a per-chunk txColor
    // (DMA-kick) breakdown so the 2.1ms "tx" is attributable. Reset alongside the others.
    int64_t splitRasterClearUs_ = 0, splitRasterFillUs_ = 0, splitTxKickUs_ = 0;
    int splitRasterClearN_ = 0, splitRasterFillN_ = 0, splitTxKickN_ = 0;
    int64_t splitSlotWaitUs_ = 0, splitCompleteWaitUs_ = 0, splitWindowUs_ = 0;
    // TEMP wire probe (gea3d present-path work): kick/done timestamp pairs per
    // chunk; done side is stamped in flushDoneCallback (ISR).
    static constexpr int kSplitProbeSlots = 12;
    volatile bool splitProbeArm_ = false;
    int64_t splitKickTs_[kSplitProbeSlots] = {};
    int splitKickN_ = 0;
    int64_t splitDoneTs_[kSplitProbeSlots] = {};
    volatile int splitDoneN_ = 0;
    // One-shot [wire.probe] region counter (was a static local): promoted to a
    // member so the [chunk.probe] overlap capture can start only AFTER the
    // wire probe's mid-loop chunk-0 drains are done — otherwise those forced
    // drains serialize the pipeline and poison the overlap reading.
    int wireProbeRuns_ = 0;
    // [region.split] last dirty-region geometry + pipeline decision (snapshot, not summed).
    int splitRegionX0_ = 0, splitRegionW_ = 0, splitRegionRows_ = 0, splitDrainChunk_ = 0;
    int flushChunkRows_ = kFlushChunkDefault;
    int flushQueueDepth_ = kFlushQueueDepthDefault;
    int presentScale_ = 1;
    int cursorRow_ = 0;
    int cursorColumn_ = 0;
    int brightness_ = 100;
    bool highBrightnessMode_ = false;
    bool displayOn_ = false;
    bool previousPresentValid_ = false;
    // Set by Display.invalidate(): the next present is a full-screen damage-all that
    // skips the dirtyRects compare AND the persistent previous-frame copy. Cleared
    // each present. Apps assert it per-frame while panning the whole screen.
    bool invalidateRequested_ = false;
    // True while the panel's content came from Display::present() canvas
    // commands (display-backed canvas fast path) rather than a framebuffer
    // flush. The CPU framebuffer is stale in that state, so snapshots must
    // re-rasterize previousPresentFrame_ instead of copying the framebuffer.
    bool lastPanelUpdateWasPresent_ = false;
    bool directPsramFlushDisabled_ = false;
#if GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE
    std::uint16_t *directPsramStaticTxProbe_ = nullptr;
#endif
    present::Frame previousPresentFrame_{};
    present::Rect externalPresentDamage_[kMaxPresentRects] = {};
    int externalPresentDamageCount_ = 0;
    std::atomic<FlushStage> flushStage_{FlushStage::Idle};
    std::atomic<int> flushStageChunk_{0};
    std::atomic<int> flushStageX0_{0};
    std::atomic<int> flushStageY0_{0};
    std::atomic<int> flushStageX1_{-1};
    std::atomic<int> flushStageY1_{-1};
    std::atomic<int> flushStageRows_{0};
    std::atomic<int> flushStagePixels_{0};
    const std::uint16_t *worldPixels_ = nullptr;
    int worldWidth_ = 0;
    int worldHeight_ = 0;
    int worldPanelTop_ = 0;
    int worldPanelHeight_ = 0;
    int worldScrollX_ = 0;
  };

} // namespace gea::platform::esp32::display

namespace
{
  using DisplayBackend = gea::platform::esp32::display::DisplayBackend;
}

void platform_display::Display::flushStatsRead(int64_t *totalUs, int *callCount, int *pixelCount)
{
  DisplayBackend::instance().flushStatsRead(totalUs, callCount, pixelCount);
}

platform_display::DisplayFlushPerfStats platform_display::Display::flushPerfStatsRead()
{
  return DisplayBackend::instance().flushPerfStatsRead();
}

void platform_display::Display::flushOdometerRead(uint32_t &calls, uint64_t &pixels)
{
  DisplayBackend::instance().flushOdometerRead(calls, pixels);
}

void platform_display::Display::flushStatsReset()
{
  DisplayBackend::instance().flushStatsReset();
}

const char *platform_display::Display::flushStageName()
{
  return DisplayBackend::instance().flushStageName();
}

int platform_display::Display::flushStageChunk()
{
  return DisplayBackend::instance().flushStageChunk();
}

platform_display::DisplayFlushStageDetail platform_display::Display::flushStageDetail()
{
  return DisplayBackend::instance().flushStageDetail();
}

void platform_display::Display::resetClip() { DisplayBackend::instance().resetClip(); }
void platform_display::Display::pushClip(int x, int y, int w, int h) { DisplayBackend::instance().pushClip(x, y, w, h); }
void platform_display::Display::popClip() { DisplayBackend::instance().popClip(); }
void platform_display::Display::clip(int *x0, int *y0, int *x1, int *y1) { DisplayBackend::instance().clip(x0, y0, x1, y1); }
void platform_display::Display::setAlpha(std::uint8_t alpha) { DisplayBackend::instance().setAlpha(alpha); }
std::uint8_t platform_display::Display::alpha() { return DisplayBackend::instance().alpha(); }
int platform_display::Display::brightness() { return DisplayBackend::instance().brightness(); }
void platform_display::Display::setBrightness(int brightnessPercent) { DisplayBackend::instance().setBrightness(brightnessPercent); }
bool platform_display::Display::setHighBrightnessMode(bool enabled) { return DisplayBackend::instance().setHighBrightnessMode(enabled); }
bool platform_display::Display::highBrightnessMode() { return DisplayBackend::instance().highBrightnessMode(); }
void platform_display::Display::setVSync(bool on) { DisplayBackend::instance().setVSync(on); }
void platform_display::Display::invalidate() { DisplayBackend::instance().invalidate(); }
bool platform_display::Display::vsyncEnabled() { return DisplayBackend::instance().vsyncEnabled(); }
void platform_display::Display::vsyncWaitForFrame() { DisplayBackend::instance().vsyncWaitForFrame(); }
void platform_display::Display::setFlushConfig(int chunkRows, int queueDepth) { DisplayBackend::instance().setFlushConfig(chunkRows, queueDepth); }
void platform_display::Display::setPresentScale(int scale) { DisplayBackend::instance().setPresentScale(scale); }
void platform_display::Display::reserveInternal(std::size_t bytes) { DisplayBackend::instance().reserveInternal(bytes); }
void platform_display::Display::applyPendingInternalReserve() { DisplayBackend::instance().applyReservation(); }
int platform_display::Display::flushChunkRows() { return DisplayBackend::instance().flushChunkRows(); }
int platform_display::Display::flushQueueDepth() { return DisplayBackend::instance().flushQueueDepth(); }
int platform_display::Display::flushBufferBytes() { return DisplayBackend::instance().flushBufferBytes(); }
void platform_display::Display::flush() { DisplayBackend::instance().flush(); }
void platform_display::Display::flushRects(const platform_display::DisplayFlushRect *rects, int count, bool allowPerChunkDrain) { DisplayBackend::instance().flushRects(rects, count, allowPerChunkDrain); }
void platform_display::Display::flushRectsRasterized(const platform_display::DisplayFlushRect *rects, int count, platform_display::DisplayStreamRasterFn raster, void *user, bool allowPerChunkDrain) { DisplayBackend::instance().flushRects(rects, count, allowPerChunkDrain, raster, user); }
void platform_display::Display::rebindCanvasToFramebuffer() { DisplayBackend::instance().rebindCanvasToFramebuffer(); }
bool platform_display::Display::streamRect(int x, int y, int w, int h, platform_display::DisplayStreamRasterFn raster, void *user) { return DisplayBackend::instance().streamRect(x, y, w, h, raster, user); }
bool platform_display::Display::present(const platform_display::DisplayPresentCommand *commands, int commandCount) { return DisplayBackend::instance().present(commands, commandCount); }
void platform_display::Display::clear() { DisplayBackend::instance().clear(); }
void platform_display::Display::clearNoFlush() { DisplayBackend::instance().clearNoFlush(); }
void platform_display::Display::print(const char *text) { DisplayBackend::instance().print(text); }
bool platform_display::Display::init() { return DisplayBackend::instance().init() == ESP_OK; }
bool platform_display::Display::start() { return DisplayBackend::instance().start() == ESP_OK; }
gea::framework::graphics::Canvas *platform_display::Display::canvas() { return DisplayBackend::instance().canvas(); }
bool platform_display::Display::copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height, bool presented) { return DisplayBackend::instance().copySnapshotRgb565(dst, pixelCapacity, width, height, presented); }
int platform_display::Display::countNonBlackPixels(bool presented) { return DisplayBackend::instance().countNonBlackPixels(presented); }
void platform_display::Display::setWorldOverlay(const std::uint16_t *pixels, int width, int height, int panelTop, int panelHeight) { DisplayBackend::instance().setWorldOverlay(pixels, width, height, panelTop, panelHeight); }
void platform_display::Display::setWorldScroll(int scrollX) { DisplayBackend::instance().setWorldScroll(scrollX); }
void platform_display::Display::setPixel(int x, int y, std::uint16_t color) { DisplayBackend::instance().setPixel(x, y, color); }
void platform_display::Display::fillRect(int x, int y, int w, int h, std::uint16_t color) { DisplayBackend::instance().fillRect(x, y, w, h, color); }
void platform_display::Display::scrollRect(int x, int y, int w, int h, int dx, int dy) { DisplayBackend::instance().scrollRect(x, y, w, h, dx, dy); }
void platform_display::Display::resetScrollRegion() { DisplayBackend::instance().resetScrollRegion(); }
void platform_display::Display::strokeRect(int x, int y, int w, int h, std::uint16_t color) { DisplayBackend::instance().strokeRect(x, y, w, h, color); }
void platform_display::Display::fillCircle(int cx, int cy, int r, std::uint16_t color) { DisplayBackend::instance().fillCircle(cx, cy, r, color); }
void platform_display::Display::strokeCircle(int cx, int cy, int r, std::uint16_t color) { DisplayBackend::instance().strokeCircle(cx, cy, r, color); }
void platform_display::Display::drawLine(int x0, int y0, int x1, int y1, std::uint16_t color) { DisplayBackend::instance().drawLine(x0, y0, x1, y1, color); }
void platform_display::Display::drawArc(int cx, int cy, int r, int startDeg, int endDeg, std::uint16_t color) { DisplayBackend::instance().drawArc(cx, cy, r, startDeg, endDeg, color); }
void platform_display::Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, std::uint16_t color) { DisplayBackend::instance().fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void platform_display::Display::drawText(const char *text, int x, int y, std::uint16_t color, float scale) { DisplayBackend::instance().drawText(text, x, y, color, scale); }
void platform_display::Display::drawTextFont(const char *text, int x, int y, std::uint16_t color, int fontId) { DisplayBackend::instance().drawTextFont(text, x, y, color, fontId); }
void platform_display::Display::drawTextFontFamily(const char *text, int x, int y, std::uint16_t color, int familyId, int sizePx) { DisplayBackend::instance().drawTextFontFamily(text, x, y, color, familyId, sizePx); }
void platform_display::Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, std::uint16_t color) { DisplayBackend::instance().fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void platform_display::Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count,
                                                           int w, int h, int tl, int tr, int br, int bl,
                                                           const std::uint16_t *colors)
{
  DisplayBackend::instance().fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void platform_display::Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lineWidth, std::uint16_t color) { DisplayBackend::instance().strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lineWidth, color); }
void platform_display::Display::blitImage(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy) { DisplayBackend::instance().blitImage(src, alpha, srcWidth, srcHeight, dx, dy); }
void platform_display::Display::blitImageScaled(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy, int dstWidth, int dstHeight)
{
  DisplayBackend::instance().blitImageScaled(src, alpha, srcWidth, srcHeight, dx, dy, dstWidth, dstHeight);
}

#if GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY
namespace gea::platform::display
{
void applyOrientation(gea::framework::display::DisplayOrientation orientation)
{
  gea::platform::esp32::display::DisplayBackend::instance().applyOrientation(orientation);
}
} // namespace gea::platform::display
#endif

// Full-screen PSRAM scratch for the UI's static-background gradient cache. The
// gradient is rendered once into this buffer and the dirty sub-region is blitted
// each frame instead of re-rasterizing the whole backdrop. Lazily allocated; if
// it fails the renderer falls back to rendering the gradient every frame.
extern "C" std::uint16_t *gea_bg_cache(int *cap_px)
{
  static std::uint16_t *buffer = nullptr;
  static bool attempted = false;
  constexpr int kCapPx = platform_display::kWidth * platform_display::kHeight;
  if (!attempted)
  {
    attempted = true;
    buffer = static_cast<std::uint16_t *>(heap_caps_malloc(
        static_cast<std::size_t>(kCapPx) * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM));
  }
  if (cap_px)
    *cap_px = buffer ? kCapPx : 0;
  return buffer;
}

// ── PIE/SIMD throughput probe ──────────────────────────────────────────────
// Decisive question before investing in a full RGB565 blend kernel: does the S3
// PIE actually deliver multiply-add throughput over a PSRAM buffer (the per-pixel
// blend is compute-bound, so it should)? Compute-heavy (4 mul-adds/elem) so it's
// not memory-bound, using only the confirmed mnemonics. Timing-only — SAR isn't
// set, so the numeric result is meaningless; we're measuring op throughput.
static void pieMulAddBuf(std::int16_t *dst, std::int16_t *src, std::int16_t *c, std::int16_t *d, int count8)
{
  asm volatile(
      "ee.vld.128.ip q1, %[c], 16\n"
      "ee.vld.128.ip q2, %[d], 16\n"
      "1:\n"
      "ee.vld.128.ip q0, %[s], 16\n"
      "ee.vmul.s16 q0, q0, q1\n"
      "ee.vadds.s16 q0, q0, q2\n"
      "ee.vmul.s16 q0, q0, q1\n"
      "ee.vadds.s16 q0, q0, q2\n"
      "ee.vmul.s16 q0, q0, q1\n"
      "ee.vadds.s16 q0, q0, q2\n"
      "ee.vmul.s16 q0, q0, q1\n"
      "ee.vadds.s16 q0, q0, q2\n"
      "ee.vst.128.ip q0, %[dd], 16\n"
      "addi %[n], %[n], -1\n"
      "bnez %[n], 1b\n"
      : [s] "+r"(src), [dd] "+r"(dst), [c] "+r"(c), [d] "+r"(d), [n] "+r"(count8)
      :
      : "memory");
}

// Constant-color RGB565 over-blend, 8 px/op, in PIE vector ops. NORMAL RGB565
// (panel-endian handled later). out_c = (fg_c*a5 + bg_c*(32-a5)) >> 5 per channel.
// Channel extract/repack uses the VMUL-shifts-by-SAR trick: ×1 with SAR=k is >>k,
// ×2^k with SAR=0 is <<k. cb is a 16-byte-aligned block of 9 broadcast vectors
// (8×int16 each): [0]=1, [1]=32-a5, [2]=fgR*a5, [3]=fgG*a5, [4]=fgB*a5, [5]=0x3F,
// [6]=0x1F, [7]=2048, [8]=32. q1=ones and q2=inv are held; q7 is the per-load
// scratch; q3/q4/q5 are the R/G/B channels.
static void pieBlendConst(std::uint16_t *dst, const std::uint16_t *src, int count8, const std::int16_t *cb)
{
  int t, ad;
  asm volatile(
      "ee.vld.128.ip q1, %[cb], 0\n"                         // q1 = ones  (cb[0])
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q2, %[ad], 0\n" // q2 = inv   (cb[1])
      "1:\n"
      "ee.vld.128.ip q0, %[s], 16\n" // q0 = 8 src px
      // R5 = (px >> 11) & 0x1F  (arith >>11 sign-extends a bit15-set px; mask cleans it)
      "movi %[t], 11\n wsr.sar %[t]\n ee.vmul.s16 q3, q0, q1\n"
      "addi %[ad], %[cb], 96\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q3, q3, q7\n"
      // G6 = (px >> 5) & 0x3F
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q6, q0, q1\n"
      "addi %[ad], %[cb], 80\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q4, q6, q7\n"
      // B5 = px & 0x1F
      "addi %[ad], %[cb], 96\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q5, q0, q7\n"
      // blend each channel: c*inv + fg_c*a5   (SAR=0 → raw product)
      "movi %[t], 0\n wsr.sar %[t]\n"
      "ee.vmul.s16 q3, q3, q2\n addi %[ad], %[cb], 32\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q3, q3, q7\n"
      "ee.vmul.s16 q4, q4, q2\n addi %[ad], %[cb], 48\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q4, q4, q7\n"
      "ee.vmul.s16 q5, q5, q2\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q5, q5, q7\n"
      // >>5  (×1 with SAR=5)
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q3, q3, q1\n ee.vmul.s16 q4, q4, q1\n ee.vmul.s16 q5, q5, q1\n"
      // repack: R<<11, G<<5, then OR in B  (SAR=0)
      "movi %[t], 0\n wsr.sar %[t]\n"
      "addi %[ad], %[cb], 112\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q3, q3, q7\n"
      "addi %[ad], %[cb], 128\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q4, q4, q7\n"
      "ee.orq q0, q3, q4\n ee.orq q0, q0, q5\n"
      "ee.vst.128.ip q0, %[d], 16\n"
      "addi %[n], %[n], -1\n bnez %[n], 1b\n"
      : [s] "+r"(src), [d] "+r"(dst), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
      : [cb] "r"(cb)
      : "memory");
}

// General over-blend for the face drawer: per-pixel fg color (gradient) + per-pixel
// alpha, panel-endian framebuffer. dst holds panel-endian bg in place and receives
// the panel-endian result. fgN = normal-RGB565 fg colors (pre-swapped in the gather),
// a5 = per-pixel alpha (0..32). Lerp form oC = bgC + ((fgC-bgC)*a5)>>5 — bit-identical
// to (fgC*a5 + bgC*(32-a5))>>5 but needs only a5, which keeps us inside 8 q-registers.
// cb2 = aligned block: [0]=1, [1]=0x1F, [2]=0x3F, [3]=0x00FF, [4]=256, [5]=2048, [6]=32.
// Held: q0=bgN, q1=ones, q2=fgN, q3=a5, q6=accum. Scratch: q4, q5(const/mask), q7.
static void pieBlendSpan8(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8,
                          const std::int16_t *cb2)
{
  int t, ad;
  asm volatile(
      "ee.vld.128.ip q1, %[cb], 0\n" // q1 = ones (cb2[0])
      "1:\n"
      "ee.vld.128.ip q0, %[d], 0\n" // q0 = bg (panel), no increment (stored back)
      // bgN = ((bg>>8)&0xFF) | (bg<<8)
      "movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n"
      "addi %[ad], %[cb], 48\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q4, q5\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q7, q0, q7\n"
      "ee.orq q0, q4, q7\n"          // q0 = bgN (normal)
      "ee.vld.128.ip q2, %[f], 16\n" // q2 = fgN
      "ee.vld.128.ip q3, %[a], 16\n" // q3 = a5
      // B channel (bits 0-4): oB = bgB + ((fgB-bgB)*a5)>>5
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n"                            // q5 = mask1F
      "ee.andq q4, q0, q5\n"                                                            // bgB
      "ee.andq q7, q2, q5\n"                                                            // fgB
      "ee.vsubs.s16 q7, q7, q4\n"                                                       // fgB-bgB
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                          // diff*a5 (raw)
      "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n" // +16 (round)
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                          // >>5
      "ee.vadds.s16 q6, q7, q4\n"                                                       // oB (accum)
      // G channel (bits 5-10): mask 0x3F, repack <<5
      "addi %[ad], %[cb], 32\n ee.vld.128.ip q5, %[ad], 0\n"                                                        // q5 = mask3F
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n"                                 // bgG
      "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n"                                                               // fgG (SAR still 5)
      "ee.vsubs.s16 q7, q7, q4\n"                                                                                   // diff
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                                                      // diff*a5 (raw)
      "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"                             // +16
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                                                      // >>5
      "ee.vadds.s16 q7, q7, q4\n"                                                                                   // oG
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 96\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n" // oG<<5 (c32)
      "ee.orq q6, q6, q7\n"
      // R channel (bits 11-15): mask 0x1F, repack <<11
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n"                         // q5 = mask1F
      "movi %[t], 11\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n" // bgR
      "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n"                                // fgR (SAR still 11)
      "ee.vsubs.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                                                      // diff*a5 (raw)
      "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"                             // +16
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                                                      // >>5
      "ee.vadds.s16 q7, q7, q4\n"                                                                                   // oR
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 80\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n" // oR<<11 (c2048)
      "ee.orq q6, q6, q7\n"                                                                                         // q6 = oN (normal)
      // swap oN -> panel, store
      "movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q6, q1\n addi %[ad], %[cb], 48\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q4, q5\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q7, q6, q7\n ee.orq q6, q4, q7\n"
      "ee.vst.128.ip q6, %[d], 16\n" // store result + advance dst by 8 px
      "addi %[n], %[n], -1\n bnez %[n], 1b\n"
      : [d] "+r"(dst), [f] "+r"(fgN), [a] "+r"(a5), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
      : [cb] "r"(cb2)
      : "memory");
}

// ── face-drawer hooks (consumed by render.cpp via weak symbols) ──────────────
// The PIE vector regs aren't preserved across FreeRTOS context switches, so a
// mid-kernel preemption corrupts the blend. The critical section below makes each
// kernel call run uninterrupted on this core (no context switch) → PIE state safe.
// Fast-RAM scratch for the renderer's hot lookup tables (see gea_render_fast_scratch
// in core/packages/engine/ui/render.cpp). This board sets
// CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=0, so a plain `new` goes to the PSRAM heap --
// which put the transformed-gradient LUT bank, read ONCE PER PIXEL in the engine's
// hottest loop, behind a cache miss. It used to live in .bss (internal SRAM) before
// it was made lazily heap-allocated. Hand back internal RAM, but only while a
// comfortable DMA reserve remains: the display staging buffers, USB host enumeration
// and the WiFi RX path all come out of the same internal heap, and starving them is
// worse than a slow gather. Allocated once, never freed.
extern "C" void *gea_render_fast_scratch(int bytes, int align)
{
  if (bytes <= 0)
    return nullptr;
  constexpr std::size_t kInternalReserveBytes = 48 * 1024;
  const std::size_t want = static_cast<std::size_t>(bytes);
  const std::size_t freeInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (freeInternal < want + kInternalReserveBytes)
  {
    std::printf("[fastram] declined %u B (internal free %u B)\n", static_cast<unsigned>(want),
                static_cast<unsigned>(freeInternal));
    return nullptr;
  }
  std::size_t alignment = align > 0 ? static_cast<std::size_t>(align) : sizeof(void *);
  if (alignment < sizeof(void *))
    alignment = sizeof(void *);
  void *p = heap_caps_aligned_alloc(alignment, want, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  std::printf("[fastram] %s %u B at %p (internal free %u B)\n", p ? "granted" : "failed",
              static_cast<unsigned>(want), p, static_cast<unsigned>(freeInternal));
  return p;
}

extern "C" bool gea_pie_blend_available() { return true; }
extern "C" void gea_pie_blend_span8(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8)
{
  // Statically initialized (each row broadcasts one constant across all 8 lanes):
  // {ones, mask5, mask6, mask8, <<8, <<11, 32, +16-round}. A runtime lazy fill would
  // race when both render cores first call this concurrently (one sees inited=true
  // while the other is mid-fill → reads a half-written table).
  alignas(16) static const std::int16_t cb2[8][8] = {
      {1, 1, 1, 1, 1, 1, 1, 1},
      {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F},
      {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F},
      {0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF},
      {256, 256, 256, 256, 256, 256, 256, 256},
      {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048},
      {32, 32, 32, 32, 32, 32, 32, 32},
      {16, 16, 16, 16, 16, 16, 16, 16},
  };
  // PER-CORE critical section — REQUIRED (proven empirically). The renderer offloads
  // the bottom row-band to the worker on core 0, where ESP-IDF pins WiFi/system tasks
  // that preempt it; without this guard the blend's PIE vector regs (q0-q7, not saved
  // across a FreeRTOS context switch) get clobbered mid-span → horizontal corruption
  // stripes in the worker band (0/4 clean captures vs. clean frames with the guard).
  // Per-core lock (indexed by core id): the two render cores never serialize on it,
  // so the parallel fill is preserved. Cost is only ~2fps; a fully-dedicated render
  // core would avoid it but isn't possible (system tasks are pinned to core 0).
  static portMUX_TYPE pieMux[2] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
  portMUX_TYPE *mux = &pieMux[xPortGetCoreID() & 1];
  taskENTER_CRITICAL(mux);
  pieBlendSpan8(dst, fgN, a5, count8, &cb2[0][0]);
  taskEXIT_CRITICAL(mux);
}

// ── FUSED gradient-colour + over-blend (8 px/op) ─────────────────────────────
// Computes the gradient colour INLINE via a per-channel integer DDA (no pieFg
// gather, no scratch round-trip) and over-blends it into the framebuffer in ONE
// SIMD pass — collapsing the scalar 1-px/iter LUT gather (the proven translucent
// bottleneck) into the existing 8-px/op blend. For a 2-stop, constant-alpha linear
// gradient (the glassy cube faces) the DDA colour is RGB565-identical to the LUT to
// within ≤1 LSB/channel (sub-perceptual). cb layout (each row = one s16 lane vector):
//   [0]ones [1]0x1F [2]0x3F [3]0x00FF [4]<<8(256) [5]<<11(2048) [6]<<5(32) [7]+16round
//   [8]a5(0..32, broadcast)  [9]rVec [10]gVec [11]bVec (Q8 seeds, MUTATED in place)
//   [12]rStep8 [13]gStep8 [14]bStep8 (per-group advance = 8*per-px-step, broadcast)
static void pieGradBlendSpan8(std::uint16_t *dst, std::int16_t *cb, int count8)
{
  int t, ad;
  asm volatile(
      "addi %[ad], %[cb], 0\n ee.vld.128.ip q1, %[ad], 0\n"   // q1 = ones (persist)
      "addi %[ad], %[cb], 128\n ee.vld.128.ip q3, %[ad], 0\n" // q3 = a5 const (persist)
      "1:\n"
      // ---- compute fgN (gradient colour) into q2 via per-channel DDA ----
      "addi %[ad], %[cb], 144\n ee.vld.128.ip q4, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q4, q1\n"   // rVec>>8
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q4, q5\n"                                      // &0x1F
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 80\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q2, q4, q5\n"     // r<<11 -> q2
      "addi %[ad], %[cb], 160\n ee.vld.128.ip q6, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q6, q6, q1\n"    // gVec>>8
      "addi %[ad], %[cb], 32\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q6, q6, q5\n"                                      // &0x3F
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 96\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q6, q6, q5\n"     // g<<5
      "ee.orq q2, q2, q6\n"
      "addi %[ad], %[cb], 176\n ee.vld.128.ip q6, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q6, q6, q1\n"    // bVec>>8
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q6, q6, q5\n"                                      // &0x1F
      "ee.orq q2, q2, q6\n"                                                                                            // q2 = fgN (normal)
      // ---- load + swap dst (bg, panel) -> bgN in q0 ----
      "ee.vld.128.ip q0, %[d], 0\n"
      "movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n addi %[ad], %[cb], 48\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q4, q5\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q7, q0, q7\n ee.orq q0, q4, q7\n" // q0 = bgN
      // ---- over-blend (identical math to pieBlendSpan8): oCh = bgCh + ((fgCh-bgCh)*a5+16)>>5 ----
      // B channel (bits 0-4)
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q0, q5\n ee.andq q7, q2, q5\n ee.vsubs.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n ee.vadds.s16 q6, q7, q4\n"
      // G channel (bits 5-10)
      "addi %[ad], %[cb], 32\n ee.vld.128.ip q5, %[ad], 0\n movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n"
      "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n ee.vsubs.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n ee.vadds.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 96\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n ee.orq q6, q6, q7\n"
      // R channel (bits 11-15)
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n movi %[t], 11\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n"
      "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n ee.vsubs.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"
      "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n ee.vadds.s16 q7, q7, q4\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 80\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n ee.orq q6, q6, q7\n"
      // swap oN(q6) -> panel, store, advance dst
      "movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q6, q1\n addi %[ad], %[cb], 48\n ee.vld.128.ip q5, %[ad], 0\n ee.andq q4, q4, q5\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q7, q6, q7\n ee.orq q6, q4, q7\n"
      "ee.vst.128.ip q6, %[d], 16\n"
      // ---- advance the per-channel DDA accumulators by 8*step (in cb) ----
      "addi %[ad], %[cb], 144\n ee.vld.128.ip q4, %[ad], 0\n addi %[ad], %[cb], 192\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q4, q4, q5\n addi %[ad], %[cb], 144\n ee.vst.128.ip q4, %[ad], 0\n"
      "addi %[ad], %[cb], 160\n ee.vld.128.ip q4, %[ad], 0\n addi %[ad], %[cb], 208\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q4, q4, q5\n addi %[ad], %[cb], 160\n ee.vst.128.ip q4, %[ad], 0\n"
      "addi %[ad], %[cb], 176\n ee.vld.128.ip q4, %[ad], 0\n addi %[ad], %[cb], 224\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q4, q4, q5\n addi %[ad], %[cb], 176\n ee.vst.128.ip q4, %[ad], 0\n"
      "addi %[n], %[n], -1\n bnez %[n], 1b\n"
      : [d] "+r"(dst), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
      : [cb] "r"(cb)
      : "memory");
}

// Scalar reference for the fused kernel (over-blend of a DDA colour onto dst), used
// once at boot to prove the asm matches before trusting it (no screen to eyeball).
static void gradBlendSelfCheck()
{
  alignas(16) std::int16_t cb[15][8];
  alignas(16) std::uint16_t dst[64], ref[64];
  auto bc = [&](int idx, int v) { for (int i = 0; i < 8; i++) cb[idx][i] = static_cast<std::int16_t>(v); };
  bc(0, 1); bc(1, 0x1F); bc(2, 0x3F); bc(3, 0x00FF); bc(4, 256); bc(5, 2048); bc(6, 32); bc(7, 16);
  const int a5 = 16; // α=0.5
  bc(8, a5);
  const int rS = 4 << 8, gS = 50 << 8, bS = 28 << 8;
  const int rStep = ((20 - 4) << 8) / 63, gStep = ((10 - 50) << 8) / 63, bStep = ((6 - 28) << 8) / 63;
  bc(12, rStep * 8); bc(13, gStep * 8); bc(14, bStep * 8);
  for (int i = 0; i < 8; i++) { cb[9][i] = static_cast<std::int16_t>(rS + i * rStep); cb[10][i] = static_cast<std::int16_t>(gS + i * gStep); cb[11][i] = static_cast<std::int16_t>(bS + i * bStep); }
  // dst seeded panel-order (swap of a normal pattern); ref computed in normal space.
  for (int i = 0; i < 64; i++) { const std::uint16_t n = static_cast<std::uint16_t>((i * 1031) & 0xFFFF); dst[i] = static_cast<std::uint16_t>((n >> 8) | (n << 8)); ref[i] = n; }
  for (int i = 0; i < 64; i++)
  {
    const int fr = ((rS + i * rStep) >> 8) & 0x1F, fg = ((gS + i * gStep) >> 8) & 0x3F, fb = ((bS + i * bStep) >> 8) & 0x1F;
    const int br = (ref[i] >> 11) & 0x1F, bgc = (ref[i] >> 5) & 0x3F, bb = ref[i] & 0x1F;
    const int oR = br + (((fr - br) * a5 + 16) >> 5), oG = bgc + (((fg - bgc) * a5 + 16) >> 5), oB = bb + (((fb - bb) * a5 + 16) >> 5);
    ref[i] = static_cast<std::uint16_t>((oR << 11) | (oG << 5) | oB);
  }
  pieGradBlendSpan8(dst, &cb[0][0], 8);
  int maxErr = 0;
  for (int i = 0; i < 64; i++)
  {
    const std::uint16_t got = static_cast<std::uint16_t>((dst[i] >> 8) | (dst[i] << 8)); // panel->normal
    const int er = std::abs(((got >> 11) & 0x1F) - ((ref[i] >> 11) & 0x1F));
    const int eg = std::abs(((got >> 5) & 0x3F) - ((ref[i] >> 5) & 0x3F));
    const int eb = std::abs((got & 0x1F) - (ref[i] & 0x1F));
    const int e = er > eg ? (er > eb ? er : eb) : (eg > eb ? eg : eb);
    if (e > maxErr) maxErr = e;
  }
  std::printf("[gradblend] selfcheck maxErr=%d %s\n", maxErr, maxErr <= 1 ? "PASS" : "FAIL");
}

extern "C" bool gea_pie_grad_blend_available() { return true; }

// Fused gradient-colour + over-blend of `count8`*8 px into dst (panel-order framebuffer).
// a5 = 0..32 alpha; r/g/b StartQ = Q8 channel value at px0; *StepQ = per-px Q8 delta.
extern "C" void gea_pie_grad_blend_span8(std::uint16_t *dst, int a5, int rStartQ, int rStepQ,
                                         int gStartQ, int gStepQ, int bStartQ, int bStepQ, int count8)
{
  static volatile bool checked = false;
  if (!checked) { checked = true; gradBlendSelfCheck(); }
  const int core = xPortGetCoreID() & 1;
  alignas(16) static std::int16_t cbStore[2][15][8]; // 16-byte aligned: ee.vld.128 requires it
  static bool cinit[2] = {false, false};
  std::int16_t(*cb)[8] = cbStore[core];
  auto bc = [&](int idx, int v) { for (int i = 0; i < 8; i++) cb[idx][i] = static_cast<std::int16_t>(v); };
  if (!cinit[core]) { bc(0, 1); bc(1, 0x1F); bc(2, 0x3F); bc(3, 0x00FF); bc(4, 256); bc(5, 2048); bc(6, 32); bc(7, 16); cinit[core] = true; }
  bc(8, a5);
  bc(12, rStepQ * 8); bc(13, gStepQ * 8); bc(14, bStepQ * 8);
  for (int i = 0; i < 8; i++) { cb[9][i] = static_cast<std::int16_t>(rStartQ + i * rStepQ); cb[10][i] = static_cast<std::int16_t>(gStartQ + i * gStepQ); cb[11][i] = static_cast<std::int16_t>(bStartQ + i * bStepQ); }
  static portMUX_TYPE gbMux[2] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
  portMUX_TYPE *mux = &gbMux[xPortGetCoreID() & 1];
  taskENTER_CRITICAL(mux);
  pieGradBlendSpan8(dst, &cb[0][0], count8);
  taskEXIT_CRITICAL(mux);
}

// ── OPAQUE gradient SIMD store (8 px/op) ─────────────────────────────────────
// The opaque-overwrite path (row[x]=colorNative[bk]) is pure SCALAR 1-px/iter — the
// only un-vectorized fill in the engine. This computes 8 colours via the same DDA
// and stores them straight to the framebuffer (panel order) — no dst read, no blend,
// no scratch round-trip (the clean case the translucent gather wasn't). cb layout
// reuses the blend table: [0]ones [1]0x1F [2]0x3F [3]mask8 [4]<<8 [5]<<11 [6]<<5
// [9]rVec [10]gVec [11]bVec [12]rStep8 [13]gStep8 [14]bStep8.
static void pieGradStoreSpan8(std::uint16_t *dst, const std::int16_t *cb, int count8)
{
  int t, ad;
  asm volatile(
      "addi %[ad], %[cb], 0\n ee.vld.128.ip q1, %[ad], 0\n"   // q1 = ones (persist)
      "addi %[ad], %[cb], 144\n ee.vld.128.ip q0, %[ad], 0\n" // q0 = rVec
      "addi %[ad], %[cb], 160\n ee.vld.128.ip q5, %[ad], 0\n" // q5 = gVec
      "addi %[ad], %[cb], 176\n ee.vld.128.ip q6, %[ad], 0\n" // q6 = bVec
      "1:\n"
      // R: q3 = ((rVec>>8)&0x1F)<<11
      "addi %[ad], %[cb], 0\n ee.vld.128.ip q7, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q3, q0, q7\n"
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q3, q3, q7\n"
      "addi %[ad], %[cb], 80\n ee.vld.128.ip q7, %[ad], 0\n movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q3, q3, q7\n"
      // G: q4 = ((gVec>>8)&0x3F)<<5 ; q3 |= q4
      "addi %[ad], %[cb], 0\n ee.vld.128.ip q7, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q5, q7\n"
      "addi %[ad], %[cb], 32\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q4, q4, q7\n"
      "addi %[ad], %[cb], 96\n ee.vld.128.ip q7, %[ad], 0\n movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q4, q4, q7\n"
      "ee.orq q3, q3, q4\n"
      // B: q4 = (bVec>>8)&0x1F ; q3 |= q4
      "addi %[ad], %[cb], 0\n ee.vld.128.ip q7, %[ad], 0\n movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q6, q7\n"
      "addi %[ad], %[cb], 16\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q4, q4, q7\n"
      "ee.orq q3, q3, q4\n" // q3 = NORMAL RGB565
      // swap normal -> panel: q3 = ((q3>>8)&0xFF) | (q3<<8)
      "movi %[t], 8\n wsr.sar %[t]\n ee.vmul.s16 q4, q3, q1\n addi %[ad], %[cb], 48\n ee.vld.128.ip q7, %[ad], 0\n ee.andq q4, q4, q7\n"
      "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 64\n ee.vld.128.ip q7, %[ad], 0\n ee.vmul.s16 q7, q3, q7\n ee.orq q3, q4, q7\n"
      "ee.vst.128.ip q3, %[d], 16\n" // store 8 panel-order px, advance dst
      // advance DDA accumulators by 8*step
      "addi %[ad], %[cb], 192\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q0, q0, q7\n"
      "addi %[ad], %[cb], 208\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q5, q5, q7\n"
      "addi %[ad], %[cb], 224\n ee.vld.128.ip q7, %[ad], 0\n ee.vadds.s16 q6, q6, q7\n"
      "addi %[n], %[n], -1\n bnez %[n], 1b\n"
      : [d] "+r"(dst), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
      : [cb] "r"(cb)
      : "memory");
}

// Validate the store kernel matches a scalar reference (panel-order) within 1 LSB.
static void gradStoreSelfCheck()
{
  alignas(16) std::int16_t cb[15][8];
  alignas(16) std::uint16_t out[64];
  auto bc = [&](int idx, int v) { for (int i = 0; i < 8; i++) cb[idx][i] = static_cast<std::int16_t>(v); };
  bc(0, 1); bc(1, 0x1F); bc(2, 0x3F); bc(3, 0x00FF); bc(4, 256); bc(5, 2048); bc(6, 32);
  const int rS = 3 << 8, gS = 55 << 8, bS = 30 << 8;
  const int rStep = ((25 - 3) << 8) / 63, gStep = ((5 - 55) << 8) / 63, bStep = ((2 - 30) << 8) / 63;
  bc(12, rStep * 8); bc(13, gStep * 8); bc(14, bStep * 8);
  for (int i = 0; i < 8; i++) { cb[9][i] = static_cast<std::int16_t>(rS + i * rStep); cb[10][i] = static_cast<std::int16_t>(gS + i * gStep); cb[11][i] = static_cast<std::int16_t>(bS + i * bStep); }
  pieGradStoreSpan8(out, &cb[0][0], 8);
  int maxErr = 0;
  for (int i = 0; i < 64; i++)
  {
    const int r = ((rS + i * rStep) >> 8) & 0x1F, g = ((gS + i * gStep) >> 8) & 0x3F, b = ((bS + i * bStep) >> 8) & 0x1F;
    const std::uint16_t refN = static_cast<std::uint16_t>((r << 11) | (g << 5) | b);
    const std::uint16_t gotN = static_cast<std::uint16_t>((out[i] >> 8) | (out[i] << 8)); // panel->normal
    const int er = std::abs(((gotN >> 11) & 0x1F) - r), eg = std::abs(((gotN >> 5) & 0x3F) - g), eb = std::abs((gotN & 0x1F) - b);
    const int e = er > eg ? (er > eb ? er : eb) : (eg > eb ? eg : eb);
    if (e > maxErr) maxErr = e;
    (void)refN;
  }
  std::printf("[gradstore] selfcheck maxErr=%d %s\n", maxErr, maxErr <= 1 ? "PASS" : "FAIL");
}

extern "C" bool gea_pie_grad_store_available() { return true; }

// Opaque gradient SIMD store: writes count8*8 panel-order px straight to the framebuffer.
extern "C" void gea_pie_grad_store_span8(std::uint16_t *dst, int rStartQ, int rStepQ,
                                         int gStartQ, int gStepQ, int bStartQ, int bStepQ, int count8)
{
  static volatile bool checked = false;
  if (!checked) { checked = true; gradStoreSelfCheck(); }
  const int core = xPortGetCoreID() & 1;
  alignas(16) static std::int16_t cbStore[2][15][8];
  static bool cinit[2] = {false, false};
  std::int16_t(*cb)[8] = cbStore[core];
  auto bc = [&](int idx, int v) { for (int i = 0; i < 8; i++) cb[idx][i] = static_cast<std::int16_t>(v); };
  if (!cinit[core]) { bc(0, 1); bc(1, 0x1F); bc(2, 0x3F); bc(3, 0x00FF); bc(4, 256); bc(5, 2048); bc(6, 32); cinit[core] = true; }
  bc(12, rStepQ * 8); bc(13, gStepQ * 8); bc(14, bStepQ * 8);
  for (int i = 0; i < 8; i++) { cb[9][i] = static_cast<std::int16_t>(rStartQ + i * rStepQ); cb[10][i] = static_cast<std::int16_t>(gStartQ + i * gStepQ); cb[11][i] = static_cast<std::int16_t>(bStartQ + i * bStepQ); }
  static portMUX_TYPE gsMux[2] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
  portMUX_TYPE *mux = &gsMux[xPortGetCoreID() & 1];
  taskENTER_CRITICAL(mux);
  pieGradStoreSpan8(dst, &cb[0][0], count8);
  taskEXIT_CRITICAL(mux);
}

extern "C" void gea_pie_bench()
{
  constexpr int N = 8192;
  auto *a = static_cast<std::int16_t *>(heap_caps_aligned_alloc(16, N * sizeof(std::int16_t), MALLOC_CAP_SPIRAM));
  auto *b = static_cast<std::int16_t *>(heap_caps_aligned_alloc(16, N * sizeof(std::int16_t), MALLOC_CAP_SPIRAM));
  auto *ref = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(16, N * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM));
  if (!a || !b || !ref)
  {
    std::printf("[pie] alloc fail\n");
    if (a)
      heap_caps_free(a);
    if (b)
      heap_caps_free(b);
    if (ref)
      heap_caps_free(ref);
    return;
  }
  alignas(16) std::int16_t c8[8] = {3, 3, 3, 3, 3, 3, 3, 3};
  alignas(16) std::int16_t d8[8] = {7, 7, 7, 7, 7, 7, 7, 7};
  for (int i = 0; i < N; i++)
    a[i] = static_cast<std::int16_t>(i & 0x7FFF);
  volatile std::int16_t sink = 0;
  const int reps = 100;
  std::int64_t t0 = esp_timer_get_time();
  for (int r = 0; r < reps; r++)
    for (int i = 0; i < N; i++)
    {
      std::int16_t v = a[i];
      v = static_cast<std::int16_t>(v * 3 + 7);
      v = static_cast<std::int16_t>(v * 3 + 7);
      v = static_cast<std::int16_t>(v * 3 + 7);
      v = static_cast<std::int16_t>(v * 3 + 7);
      b[i] = v;
    }
  std::int64_t scalarUs = esp_timer_get_time() - t0;
  sink = b[N - 1];
  t0 = esp_timer_get_time();
  for (int r = 0; r < reps; r++)
    pieMulAddBuf(b, a, c8, d8, N / 8);
  std::int64_t pieUs = esp_timer_get_time() - t0;
  sink = b[N - 1];
  (void)sink;
  std::printf("[pie] %dpx x%d (4 mul-add/elem)  scalar=%lldus  pie=%lldus  ratio=%.2fx\n", N, reps,
              static_cast<long long>(scalarUs), static_cast<long long>(pieUs),
              pieUs ? static_cast<double>(scalarUs) / static_cast<double>(pieUs) : 0.0);

  // ── real RGB565 over-blend: bit-exact vs scalar + speed ──────────────────
  {
    const std::uint16_t fg = 0xF800; // bright red — R5=31 (bit 15 set) exercises sign handling
    const int a5 = 20;               // ~0.625 alpha, 0..32
    const int fgR = fg >> 11, fgG = (fg >> 5) & 0x3F, fgB = fg & 0x1F, inv = 32 - a5;
    alignas(16) std::int16_t cb[9][8];
    auto fillv = [&](int idx, int v)
    {
      for (int i = 0; i < 8; i++)
        cb[idx][i] = static_cast<std::int16_t>(v);
    };
    fillv(0, 1);
    fillv(1, inv);
    fillv(2, fgR * a5);
    fillv(3, fgG * a5);
    fillv(4, fgB * a5);
    fillv(5, 0x3F);
    fillv(6, 0x1F);
    fillv(7, 2048);
    fillv(8, 32);
    auto *sp = reinterpret_cast<std::uint16_t *>(a);
    auto *dp = reinterpret_cast<std::uint16_t *>(b);
    for (int i = 0; i < N; i++)
      sp[i] = static_cast<std::uint16_t>((static_cast<unsigned>(i) * 2654435761u) >> 16);
    for (int i = 0; i < N; i++)
    {
      int R = sp[i] >> 11, G = (sp[i] >> 5) & 0x3F, B = sp[i] & 0x1F;
      int oR = (fgR * a5 + R * inv) >> 5, oG = (fgG * a5 + G * inv) >> 5, oB = (fgB * a5 + B * inv) >> 5;
      ref[i] = static_cast<std::uint16_t>((oR << 11) | (oG << 5) | oB);
    }
    pieBlendConst(dp, sp, N / 8, &cb[0][0]);
    int mism = 0, firstBad = -1;
    for (int i = 0; i < N; i++)
      if (dp[i] != ref[i])
      {
        mism++;
        if (firstBad < 0)
          firstBad = i;
      }
    std::int64_t st0 = esp_timer_get_time();
    for (int r = 0; r < reps; r++)
      for (int i = 0; i < N; i++)
      {
        int R = sp[i] >> 11, G = (sp[i] >> 5) & 0x3F, B = sp[i] & 0x1F;
        int oR = (fgR * a5 + R * inv) >> 5, oG = (fgG * a5 + G * inv) >> 5, oB = (fgB * a5 + B * inv) >> 5;
        dp[i] = static_cast<std::uint16_t>((oR << 11) | (oG << 5) | oB);
      }
    std::int64_t bScalarUs = esp_timer_get_time() - st0;
    st0 = esp_timer_get_time();
    for (int r = 0; r < reps; r++)
      pieBlendConst(dp, sp, N / 8, &cb[0][0]);
    std::int64_t bPieUs = esp_timer_get_time() - st0;
    std::printf("[pie-blend] mismatch=%d/%d (first@%d)  scalar=%lldus pie=%lldus ratio=%.2fx\n", mism, N, firstBad,
                static_cast<long long>(bScalarUs), static_cast<long long>(bPieUs),
                bPieUs ? static_cast<double>(bScalarUs) / static_cast<double>(bPieUs) : 0.0);
  }

  // ── general span: per-pixel fg gradient + per-pixel alpha + panel-endian ──
  {
    constexpr int M = 256;
    alignas(16) std::uint16_t bg[M], fgN[M], ref[M];
    auto *work = reinterpret_cast<std::uint16_t *>(b); // PSRAM, in-place — matches the drawer (framebuffer)
    alignas(16) std::int16_t a5[M];
    alignas(16) std::int16_t cb2[8][8];
    auto fv = [&](int idx, int v)
    {
      for (int i = 0; i < 8; i++)
        cb2[idx][i] = static_cast<std::int16_t>(v);
    };
    fv(0, 1);
    fv(1, 0x1F);
    fv(2, 0x3F);
    fv(3, 0x00FF);
    fv(4, 256);
    fv(5, 2048);
    fv(6, 32);
    fv(7, 16);
    for (int i = 0; i < M; i++)
    {
      fgN[i] = static_cast<std::uint16_t>((static_cast<unsigned>(i) * 1103515245u) >> 16); // fg (normal RGB565)
      bg[i] = static_cast<std::uint16_t>((static_cast<unsigned>(i) * 2654435761u) >> 16);  // bg (panel-endian)
      a5[i] = static_cast<std::int16_t>(i % 33);                                           // per-pixel alpha 0..32 (incl. 32)
      work[i] = bg[i];
    }
    for (int i = 0; i < M; i++)
    {
      std::uint16_t bgn = static_cast<std::uint16_t>((bg[i] >> 8) | (bg[i] << 8));
      int bR = bgn >> 11, bG = (bgn >> 5) & 0x3F, bB = bgn & 0x1F;
      int fR = fgN[i] >> 11, fG = (fgN[i] >> 5) & 0x3F, fB = fgN[i] & 0x1F;
      int A = a5[i], iv = 32 - A;
      int oR = (fR * A + bR * iv + 16) >> 5, oG = (fG * A + bG * iv + 16) >> 5, oB = (fB * A + bB * iv + 16) >> 5;
      std::uint16_t on = static_cast<std::uint16_t>((oR << 11) | (oG << 5) | oB);
      ref[i] = static_cast<std::uint16_t>((on >> 8) | (on << 8));
    }
    pieBlendSpan8(work, fgN, a5, M / 8, &cb2[0][0]);
    int mism = 0, firstBad = -1;
    for (int i = 0; i < M; i++)
      if (work[i] != ref[i])
      {
        mism++;
        if (firstBad < 0)
          firstBad = i;
      }
    std::printf("[pie-span] mismatch=%d/%d (first@%d)  [%04x vs %04x]\n", mism, M, firstBad,
                firstBad >= 0 ? work[firstBad] : 0, firstBad >= 0 ? ref[firstBad] : 0);
  }
  heap_caps_free(a);
  heap_caps_free(b);
  heap_caps_free(ref);
}

// Full-screen PSRAM scratch for the static-backdrop cache (bg gradient + stage +
// dithered floor baked once). Separate from gea_bg_cache so the Phase-1 gradient
// cache is undisturbed. Bound as a width-strided canvas during the bake, blitted
// per frame thereafter.
extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
#if !GEA_EMBEDDED_DISPLAY_STATIC_BACKDROP
  // Backdrop cache off for this board: returning nullptr is the engine's own weak
  // default and switches off the whole path -- bake, per-frame blit, AND the
  // transform reproject (which is gated on staticBackdropActive()). css-3d-cube's
  // 62-74fps July measurements ran exactly this shape: full display-list record
  // every frame (recprobe fullrebuilds:90/90) with a partial ~22k px dirty flush,
  // and no [bdrop] line in any log.
  //
  // Do NOT read "no [bdrop] line in the log" as "the path never engaged" -- it means
  // the opposite. The cache bakes once at settle and a kQuiet frame keeps it, so a
  // healthy steady state logs nothing; a line appears only on a bake or a drop.
  // Measured on css-3d-cube translucent (2026-09-15): cache ON 50-56 fps, OFF 31-44.
  // Turning it off also switches off the transform reproject. An earlier note here
  // blamed the per-frame blit window for stranding the CO5300's dropped column along
  // the cube's seams -- that was WRONG; those pixels were a byte-swapped write in
  // render.cpp's scalarPx. See the board CMakeLists for the full account.
  if (cap_px)
    *cap_px = 0;
  return nullptr;
#else
  // (gea_pie_bench() was a one-time SIMD validation probe — disabled now that the
  // SIMD path is parked; left in the TU for reference / future re-enable.)
  static std::uint16_t *buffer = nullptr;
  static bool attempted = false;
  constexpr int kCapPx = platform_display::kWidth * platform_display::kHeight;
  if (!attempted)
  {
    attempted = true;
    buffer = static_cast<std::uint16_t *>(heap_caps_malloc(
        static_cast<std::size_t>(kCapPx) * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM));
  }
  if (cap_px)
    *cap_px = buffer ? kCapPx : 0;
  return buffer;
#endif
}

// ── 2nd-core render worker ─────────────────────────────────────────────────
// The renderer is single-core (main task on CPU0); CPU1 is idle. For a scanline
// fill the renderer offloads the bottom row-band to this worker (pinned to the
// other core) via gea_render_parallel_submit(), fills the top band itself, then
// gea_render_parallel_wait()s. The worker BLOCKS on its own task-notification
// when idle (so the worker core's idle task still runs — keeps the task watchdog
// fed); the renderer SPINS on an atomic done flag for the sub-millisecond join.
// The submitted band callback must be re-entrant (it runs concurrently with the
// renderer over a DISJOINT row range — disjoint framebuffer rows + its own dirty
// accumulator; all other state it touches is read-only during the fill).
namespace
{
  struct GeaRenderWorker
  {
    TaskHandle_t task = nullptr;
    void (*volatile fn)(void *, int, int) = nullptr;
    void *volatile ctx = nullptr;
    volatile int y0 = 0;
    volatile int y1 = 0;
    std::atomic<std::uint32_t> jobSeq{0}; // bumped per submit; the worker spins on it
    std::atomic<bool> done{true};
    TaskHandle_t submitter = nullptr;     // notified on done for the blocking wait path
    bool attempted = false;
  };
  GeaRenderWorker g_geaRenderWorker;
  // DIAG: concurrency timing — startup latency (submit→worker-start), band time,
  // and the main's wait time. Distinguishes "worker starts late" from "contention".
  volatile std::int64_t g_wkSubmitUs = 0;
  std::int64_t g_wkStartLatUs = 0, g_wkBandUs = 0, g_wkWaitUs = 0;
  std::int64_t g_wkStartLatLast = 0; // this-frame worker pickup latency (submit→start)
  int g_mainSharePermille = 500;     // adaptive main-core row share (see the controller in wait())
  int g_wkJobs = 0;
  constexpr std::uint32_t kRenderWorkerStackBytes = 8192;
// GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL=1 takes the worker's stack from
// external RAM. The worker only replays raster bands into the framebuffer and
// never runs with the flash cache disabled, so the stack may live there; on a
// board whose internal SRAM is committed elsewhere the internal allocation
// otherwise fails and all raster work silently stays on the render core.
// Needs CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY.
#ifndef GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL
#define GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL 0
#endif

  void geaRenderWorkerTask(void *)
  {
    std::uint32_t seen = 0;
    std::int64_t idleSince = esp_timer_get_time();
    for (;;)
    {
      // Wait for the next band. SPIN so a band submitted mid-frame is picked up with
      // ~zero latency — a cross-core notify wake measured ~500us (≈ half a tick), which
      // wasted most of the parallel win. After a few ms idle (between frames) fall back
      // to blocking so CPU1's idle task + the task watchdog still run.
      while (g_geaRenderWorker.jobSeq.load(std::memory_order_acquire) == seen)
      {
        if (esp_timer_get_time() - idleSince > 4000)
        {
          ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
          idleSince = esp_timer_get_time();
        }
      }
      seen = g_geaRenderWorker.jobSeq.load(std::memory_order_acquire);
      const std::int64_t s = esp_timer_get_time();
      g_wkStartLatUs += s - g_wkSubmitUs;
      g_wkStartLatLast = s - g_wkSubmitUs;
      void (*fn)(void *, int, int) = g_geaRenderWorker.fn;
      if (fn)
        fn(g_geaRenderWorker.ctx, g_geaRenderWorker.y0, g_geaRenderWorker.y1);
      g_wkBandUs += esp_timer_get_time() - s;
      g_geaRenderWorker.done.store(true, std::memory_order_release);
      // Wake a submitter parked in the blocking wait (the gea3d present pipeline,
      // which yields core 0 to the app task while a chunk rasters). Harmless for
      // spin-waiters — they never park. Latches, so no lost wakeup if it wasn't
      // waiting yet.
      if (TaskHandle_t sub = g_geaRenderWorker.submitter)
        xTaskNotifyGive(sub);
      idleSince = esp_timer_get_time(); // start the spin window after finishing a band
    }
  }
} // namespace

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
  if (!g_geaRenderWorker.attempted)
  {
    g_geaRenderWorker.attempted = true;
    const BaseType_t renderCore = xPortGetCoreID();
    const BaseType_t workerCore = (renderCore == 0) ? 1 : 0;
    const UBaseType_t prio = uxTaskPriorityGet(nullptr); // symmetric with the render task
    // 8KB stack (bytes): the worker runs only band replay, not the app loop. On
    // bubble-grid-jsx the largest free internal block at first moving replay is
    // ~9.7KB, so a 12KB worker silently disables the split path and leaves all
    // rounded-circle raster work on the render core.
#if GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL
    const BaseType_t created = xTaskCreatePinnedToCoreWithCaps(geaRenderWorkerTask, "gea_rwrk",
                                                               kRenderWorkerStackBytes, nullptr, prio,
                                                               &g_geaRenderWorker.task, workerCore,
                                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    const BaseType_t created = xTaskCreatePinnedToCore(geaRenderWorkerTask, "gea_rwrk",
                                                       kRenderWorkerStackBytes, nullptr, prio,
                                                       &g_geaRenderWorker.task, workerCore);
#endif
    std::printf("[render-worker] task=%p create=%d renderCore=%d workerCore=%d prio=%u stack=%u\n",
                static_cast<void *>(g_geaRenderWorker.task), static_cast<int>(created),
                static_cast<int>(renderCore),
                static_cast<int>(workerCore), static_cast<unsigned>(prio),
                static_cast<unsigned>(kRenderWorkerStackBytes));
    if (created != pdPASS || !g_geaRenderWorker.task)
    {
      multi_heap_info_t info{};
      heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      std::printf("[render-worker] create failed: create=%d stack=%u internal_free=%u internal_largest=%u internal_min=%u free_blocks=%u alloc_blocks=%u total_blocks=%u\n",
                  static_cast<int>(created), static_cast<unsigned>(kRenderWorkerStackBytes),
                  static_cast<unsigned>(info.total_free_bytes),
                  static_cast<unsigned>(info.largest_free_block),
                  static_cast<unsigned>(info.minimum_free_bytes),
                  static_cast<unsigned>(info.free_blocks),
                  static_cast<unsigned>(info.allocated_blocks),
                  static_cast<unsigned>(info.total_blocks));
    }
  }
  if (!g_geaRenderWorker.task)
    return false;
  // Record the render core (the submitting core) so the display backend routes the
  // worker core's replay to its own band-canvas. Re-set each submit in case the
  // render task ever migrates cores.
  gea::platform::esp32::display::DisplayBackend::instance().setRenderCore(static_cast<int>(xPortGetCoreID()));
  g_geaRenderWorker.fn = fn;
  g_geaRenderWorker.ctx = ctx;
  g_geaRenderWorker.y0 = y0;
  g_geaRenderWorker.y1 = y1;
  g_geaRenderWorker.submitter = xTaskGetCurrentTaskHandle();
  g_geaRenderWorker.done.store(false, std::memory_order_release);
  g_wkSubmitUs = esp_timer_get_time();
  g_geaRenderWorker.jobSeq.fetch_add(1, std::memory_order_release); // publish job (worker spins on this)
  xTaskNotifyGive(g_geaRenderWorker.task);                          // also wake it if it fell back to blocking
  return true;
}

extern "C" void gea_render_parallel_merge_dirty()
{
  gea::platform::esp32::display::DisplayBackend::instance().absorbWorkerDirty();
}

// Render core id for per-core scratch indexing in the shared renderer (render.cpp).
extern "C" int gea_current_render_core() { return static_cast<int>(xPortGetCoreID()); }

extern "C" void gea_render_parallel_wait()
{
  if (!g_geaRenderWorker.task)
    return;
  // The worker's band is sub-millisecond; spin (the render core's idle task only
  // needs to run within the ~5s watchdog window, so a short spin is fine). The
  // esp_timer deadline guards against a wedged worker so we can never hang.
  const std::int64_t w0 = esp_timer_get_time();
  const std::int64_t deadline = w0 + 100000; // 100ms safety net
  while (!g_geaRenderWorker.done.load(std::memory_order_acquire))
  {
    if (esp_timer_get_time() > deadline)
    {
      std::printf("[render-worker] WAIT TIMEOUT — worker wedged?\n");
      break;
    }
  }
  const std::int64_t waitThis = esp_timer_get_time() - w0;
  g_wkWaitUs += waitThis;
  // Adaptive row-share controller (visual-neutral: only shifts WHICH rows each core
  // renders, never the output). Balance point = main's top-band time ≈ worker's wall
  // time, i.e. this-frame main-wait ≈ worker startup latency. wait>startLat ⇒ the worker
  // (bottom band) is the tall pole ⇒ shift rows off it (raise the main share); wait<startLat
  // ⇒ the worker idled waiting on the main ⇒ give the worker more rows. A small clamped
  // step per frame converges in a few frames and continuously tracks the cube's rotation
  // (which shifts the projected pixel mass between the top and bottom bands frame to frame).
  {
    const int errUs = static_cast<int>(waitThis - g_wkStartLatLast);
    int step = errUs / 40; // ~1 permille per 40us of imbalance
    if (step > 8)
      step = 8;
    else if (step < -8)
      step = -8;
    g_mainSharePermille += step;
    if (g_mainSharePermille < 350)
      g_mainSharePermille = 350;
    else if (g_mainSharePermille > 700)
      g_mainSharePermille = 700;
  }
  if (++g_wkJobs >= 300)
  {
    std::printf("[worker] startLat=%dus band=%dus wait=%dus share=%d (avg/300)\n",
                static_cast<int>(g_wkStartLatUs / 300), static_cast<int>(g_wkBandUs / 300),
                static_cast<int>(g_wkWaitUs / 300), g_mainSharePermille);
    g_wkStartLatUs = g_wkBandUs = g_wkWaitUs = 0;
    g_wkJobs = 0;
  }
}

extern "C" void gea_render_parallel_wait_blocking()
{
  if (!g_geaRenderWorker.task)
    return;
  // Unlike the spin wait above, PARK the caller (yield the core) until the worker
  // finishes — so a lower-priority task sharing this core (the app frame task,
  // pinned to core 0 alongside the gea3d present) runs during the chunk raster
  // instead of the present spinning it away. The worker notifyGives us on done;
  // the short timeout re-checks in case the notify landed before we parked.
  const std::int64_t deadline = esp_timer_get_time() + 100000; // 100ms safety net
  while (!g_geaRenderWorker.done.load(std::memory_order_acquire))
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
    if (esp_timer_get_time() > deadline)
    {
      std::printf("[render-worker] BLOCKING WAIT TIMEOUT — worker wedged?\n");
      break;
    }
  }
}

// Adaptive main-core row share for the 2-core replay split (overrides the renderer's
// weak 500-permille default). Updated each frame by the controller in gea_render_parallel_wait().
extern "C" int gea_render_parallel_main_share_permille() { return g_mainSharePermille; }
