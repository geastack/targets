// Generic parallel-RGB (DPI) panel backend for the ESP32-S3 LCD peripheral.
//
// Selected by composing a board whose `chips.display` is `rgb_panel`: the panel
// has no controller IC to talk to, the S3's LCD_CAM scans a PSRAM framebuffer
// out continuously and the only board-specific facts are timings and pins,
// which board.h supplies. The backlight is either a GPIO driven by LEDC PWM or
// a line on the board's I/O expander (Waveshare's 7" routes it through a
// CH422G); a panel reset line, when the board has one, is likewise an expander
// pin. The raster/present pipeline below is shared with the Elecrow rotary's
// ST7701 target, which is where it was measured and tuned.
#include "esp_cpu.h"
#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#if GEA_BOARD_HAS_EXPANDER
#include "chip_bindings/expanders/io_expander.h"
#endif

#ifndef GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
// Strip-raster fused present: compose each horizontal strip in INTERNAL SRAM
// (fast, cached, no contention with the RGB panel's continuous PSRAM scanout),
// then bulk-copy the finished strip straight to the live scanout framebuffer.
// Replaces the slow "raster 1000 scattered circles directly into the PSRAM back
// buffer (75ms under scanout contention) + memcpy back->scanout (36ms)" path for
// full-screen-redraw frames. Still composes each strip fully off-screen, so it is
// tear-equivalent to the double buffer (never exposes clear-then-draw intermediates).
#define GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER 1
#endif

// Bounce buffers vs direct PSRAM scanout: with bounce, a core-0 ISR memcpys
// every scanline batch from PSRAM into internal SRAM -- a standing CPU tax on
// the ISR's core. With 0 the GDMA reads the PSRAM framebuffer directly (no CPU
// in the scanout path); the risk is scanout underrun (visible drift) when CPU
// PSRAM traffic starves the DMA.
#ifndef GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
#define GEA_EMBEDDED_DISPLAY_RGB_BOUNCE 1
#endif

// Tear-free present: two driver-owned scanout framebuffers plus the persistent
// draw buffer. A frame is published into whichever scanout buffer the panel is
// NOT reading, then handed to the driver, which switches to it at the next frame
// boundary. With a single scanout buffer the publish copy wrote rows the panel
// was scanning at that moment, so every animated frame tore (and a ~17ms copy at
// 44Hz refresh spans most of a scan). 0 restores the single-buffer path.
// Core for the panel's interrupts (the bounce-buffer copy ISR); -1 = the core
// that initialises the display.
// Core 1: the renderer runs on core 0, and the bounce copy costs it ~10% there.
// With three scanout buffers, svg-charts on the CrowPanel 7 measured 26.6 fps
// with the ISR on core 1 against 25.0 on core 0. (With two buffers, core 1 was
// worse: a swap from the other core waits two frame ends, so frames snapped to
// panel-rate / 3.)
#ifndef GEA_EMBEDDED_DISPLAY_RGB_ISR_CORE
#define GEA_EMBEDDED_DISPLAY_RGB_ISR_CORE 1
#endif

#ifndef GEA_EMBEDDED_DISPLAY_RGB_SWAP_FBS
#define GEA_EMBEDDED_DISPLAY_RGB_SWAP_FBS 1
#endif
// Scanout buffers in swap mode (2 or 3). With three, the renderer draws into a
// buffer the panel released one frame earlier, so a publish never waits for the
// second vsync after the previous one: the frame rate follows the frame work
// instead of snapping to panel-rate / 2.
// The board picks the count (chips.display.scanFramebuffers); a build can still
// force it with this macro.

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_cache.h"
#include "esp_private/esp_cache_private.h"
#include "hal/cache_hal.h"
#include "esp_heap_caps.h"
#include "esp_ipc.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "soc/clk_tree_defs.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/cache.h"
#include "soc/gdma_struct.h"
#endif

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

// Second-core worker (defined later in this file). Used by the strip-raster path to
// copy a finished strip to the PSRAM scanout buffer on the other core while this
// core rasters the next strip into SRAM.
extern "C" bool gea_render_parallel_rows_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1);
extern "C" void gea_render_parallel_wait();

namespace {

constexpr const char *kTag = "rgb_panel";
constexpr int kWidth = gea::platform::display::kWidth;
constexpr int kHeight = gea::platform::display::kHeight;
constexpr std::size_t kPixelCount = static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);
constexpr std::size_t kFramebufferBytes = kPixelCount * sizeof(std::uint16_t);
#ifdef GEA_EMBEDDED_DISPLAY_RGB_PCLK_HZ
constexpr int kPixelClockHz = GEA_EMBEDDED_DISPLAY_RGB_PCLK_HZ;
static_assert(kPixelClockHz > 0, "RGB pixel clock must be positive");
#else
constexpr int kPixelClockHz = gea::platform::board::display.pclkHz;
#endif
// Bounce buffers are refilled from PSRAM while the other core renders into
// PSRAM too; a refill that overruns its window lets the LCD DMA send a half
// old buffer (a tear band). Larger buffers average the render's PSRAM bursts
// over a longer window: 20-row refills peaked at 1.29 ms of a 1.09 ms window
// on a busy screen. 40 rows (128 KB) does not fit in internal RAM by default;
// a board whose screens need it sets chips.display.bounceRows (see
// bounceBufferRows), taking that RAM from the strip staging allocated after.
#ifndef GEA_EMBEDDED_DISPLAY_RGB_BOUNCE_MAX_ROWS
#define GEA_EMBEDDED_DISPLAY_RGB_BOUNCE_MAX_ROWS 30
#endif
	// Bounce rows: the largest candidate that divides the panel height, so the
	// framebuffer splits into whole bounce buffers whatever the geometry is.
	constexpr int pickBounceRows(int height)
	{
		constexpr int candidates[] = {GEA_EMBEDDED_DISPLAY_RGB_BOUNCE_MAX_ROWS, 48, 40, 30, 24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1};
		for (int rows : candidates) {
			if (rows <= GEA_EMBEDDED_DISPLAY_RGB_BOUNCE_MAX_ROWS && rows > 0 && height % rows == 0) return rows;
		}
		return 1;
	}
	constexpr int kRgbBounceRows = pickBounceRows(kHeight);
	constexpr std::size_t kRgbBounceBufferPixels = static_cast<std::size_t>(kWidth) * kRgbBounceRows;
	static_assert(kPixelCount % kRgbBounceBufferPixels == 0, "RGB bounce buffer must divide the framebuffer evenly");
	// The board's bounce depth when it names one that divides the panel height
	// (the driver needs whole buffers per frame), else the default above. Not
	// a free win: on the CrowPanel 7.0 a deeper bounce pair left no internal
	// RAM for the strip staging, and rasterizing straight into PSRAM made the
	// picture flicker far worse than the overruns it was meant to absorb.
	int bounceBufferRows()
	{
		const int rows = gea::platform::board::display.bounceRows;
		return rows > 0 && kHeight % rows == 0 ? rows : kRgbBounceRows;
	}
constexpr ledc_mode_t kBacklightLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_0;
constexpr ledc_timer_bit_t kBacklightLedcDutyResolution = LEDC_TIMER_10_BIT;
constexpr std::uint32_t kBacklightLedcMaxDuty = (1u << 10) - 1u;

namespace present = gea::framework::display_present;

// Internal-SRAM scratch strips that the fused present path composes into before
// copying to the PSRAM scanout buffer. Double-buffered: while core0 rasters strip
// N+1 into one half (SRAM), the worker core copies strip N from the other half to
// scanout (PSRAM) — different buffers, different buses, so they overlap with no
// contention. Two 24-row strips fit the same 46KB block one 48-row strip used.
constexpr int kStripRows = 24;
constexpr int kStripBufferRows = kStripRows * 2;
constexpr std::size_t kStripBufferPixels = static_cast<std::size_t>(kWidth) * kStripBufferRows;
// Rows per strip half actually allocated: kStripRows when internal RAM allows,
// fewer on an app whose static data already took part of it (0 = no strips).
// A shorter strip costs a few more raster passes; no strip at all puts every
// raster write on the PSRAM bus the scanout DMA shares, and the panel slips.
constexpr int kStripRowChoices[] = {kStripRows, 16, 12, 8};
int g_stripRows = 0;
// 2: a strip per core (the fused present double-buffers across both). 1: internal
// RAM held only one, enough for the main core to stage its replay.
int g_stripHalves = 0;

gea::framework::graphics::Canvas g_canvas;
// Band canvas for the renderer's coarse 2-core replay split: bound to the SAME
// back buffer as g_canvas but with a private clip stack + dirty accumulator, so
// the worker core's band replay never races the main core on clip/dirty state.
// g_renderCoreId is the core the renderer submits from (recorded per
// gea_render_parallel_submit); Display draw calls on any OTHER core are the
// worker's band replay and route here via replayCanvas().
gea::framework::graphics::Canvas g_workerCanvas;
int g_renderCoreId = -1;

bool onWorkerCore()
{
	return g_renderCoreId >= 0 && static_cast<int>(xPortGetCoreID()) != g_renderCoreId;
}

using gea::platform::display::DisplayFlushRect;
bool g_swapFbs = false;
bool g_backReady = true;
void ensureBackReady(const DisplayFlushRect *skip = nullptr, int skipCount = 0);

// Every Display draw entry point and Display::canvas() come through here, so this
// is where the back buffer is made safe to draw into after a swap (a flag test
// on the fast path). The worker core's band replay is covered by the main core
// readying the buffer before it submits the band.
gea::framework::graphics::Canvas &replayCanvas()
{
	if (onWorkerCore()) return g_workerCanvas;
	if (!g_backReady) ensureBackReady();
	return g_canvas;
}
// The same canvas without readying the back buffer, for state that touches no
// pixels (clip, alpha, rebinding). Readying early would patch the whole stale
// area before the fused flush can say which rects it repaints, and would block
// on the panel releasing the buffer right after every swap.
gea::framework::graphics::Canvas &stateCanvas()
{
	return onWorkerCore() ? g_workerCanvas : g_canvas;
}
esp_lcd_panel_handle_t g_panel = nullptr;
std::uint16_t *g_framebuffer = nullptr;
std::uint16_t *g_drawFramebuffer = nullptr;
std::uint16_t *g_stripBuffer = nullptr;

std::size_t stripBufferPixels() { return static_cast<std::size_t>(kWidth) * g_stripRows * g_stripHalves; }

// Allocates the two strip halves at the tallest height internal RAM holds while
// leaving `headroom` bytes free (radios reserve some after boot).
void allocStripBuffer(std::size_t headroom)
{
	g_stripRows = 0;
	g_stripHalves = 0;
	// Two halves when they fit at full height; otherwise the tallest strip,
	// one half before two: the main core is the one that stages, and a staged
	// repaint re-walks the display list once per strip (a board whose bounce
	// buffers take most of the internal RAM -- the CrowPanel 7.0 -- has room
	// for a single 16-row strip or two 8-row ones).
	for (int rows : kStripRowChoices) {
		for (int halves = 2; halves >= 1; --halves) {
			const std::size_t bytes = static_cast<std::size_t>(kWidth) * rows * halves * sizeof(std::uint16_t);
			if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < bytes ||
			    heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < bytes + headroom)
				continue;
			g_stripBuffer = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (g_stripBuffer) {
				g_stripRows = rows;
				g_stripHalves = halves;
				return;
			}
		}
	}
}
std::atomic<std::size_t> g_internalReservation{0};
std::size_t g_appliedInternalReservation = 0;
std::size_t g_internalCacheHeadroom = 0;
	// The strip path publishes straight to scanout and leaves g_drawFramebuffer
	// untouched. If a later frame falls back to the per-command path (partial
	// present with no full-screen base), re-sync the back buffer once first.
	bool g_drawBufferStale = false;
	bool g_initialized = false;
	bool g_backlightPwmInitialized = false;
	int g_brightness = 100;
	int g_flushRows = kHeight;
	int g_flushDepth = 1;
	gea::platform::display::DisplayFlushPerfStats g_stats = {};

	bool clipRect(int *x0, int *y0, int *x1, int *y1);

	void cacheWritebackAll()
	{
		if (!g_framebuffer) return;
		esp_cache_msync(g_framebuffer,
		                kFramebufferBytes,
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
	}

	// Write a scanout region back from the data cache so the LCD's GDMA sees it.
	// Only direct-PSRAM scanout needs this: in bounce-buffer mode the driver's ISR
	// memcpys the framebuffer through the same (shared, single) S3 data cache, so
	// dirty lines are already coherent for it -- IDF's own draw_bitmap skips the
	// sync in that mode for the same reason. Skipping it saves a cache walk over
	// every published row.
	void cacheWritebackRect(std::uint16_t *buffer, int x0, int y0, int x1, int y1)
	{
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
		(void)buffer; (void)x0; (void)y0; (void)x1; (void)y1;
#else
		if (!buffer || !clipRect(&x0, &y0, &x1, &y1)) return;
		const std::size_t begin = static_cast<std::size_t>(y0) * kWidth + static_cast<std::size_t>(x0);
		const std::size_t end = static_cast<std::size_t>(y1) * kWidth + static_cast<std::size_t>(x1) + 1;
		if (end <= begin || end > kPixelCount) return;
		esp_cache_msync(&buffer[begin],
		                (end - begin) * sizeof(std::uint16_t),
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif
	}

	void cacheWritebackRect(int x0, int y0, int x1, int y1) { cacheWritebackRect(g_framebuffer, x0, y0, x1, y1); }

void addFlushStats(int64_t totalUs, int pixelCount)
{
	g_stats.totalUs += totalUs;
	g_stats.callCount += 1;
	g_stats.pixelCount += pixelCount;
}

void addFlushStageStats(int64_t copyUs, int64_t cacheUs)
{
	g_stats.copyUs += copyUs;
	g_stats.completeWaitUs += cacheUs;
}

// ── Swapped scanout buffers (direct mode) ────────────────────────────────────
// Two driver-owned scanout buffers and no separate draw buffer: the renderer
// draws straight into the back buffer (g_drawFramebuffer points at it), and a
// publish hands that buffer to the panel, which switches at the next frame wrap.
// g_scanFb[g_frontIndex] is the buffer most recently handed over (g_framebuffer;
// screenshots read it).
//
// After a swap the new back buffer lags by whatever the frame just published
// changed (its stale list). Before anything draws into it, ensureBackReady()
// waits for the panel to release it and patches those regions in from the front
// -- the previous frame's dirty rects only, never the whole frame. The fused
// raster path (flushRectsRasterized) knows which rects it is about to repaint
// completely, so it patches only the stale area OUTSIDE them: when an animation
// redraws the same region every frame, almost nothing is copied and each dirty
// pixel reaches PSRAM exactly once, from an internal-RAM strip.
struct StaleRect {
	int x0, y0, x1, y1;
};
// Many small animations (64 bouncing balls: an old and a new box each) must stay
// separate rects: with 8, the list folded into one screen-sized box and every
// frame copied ~300k PSRAM pixels front to back before drawing.
constexpr int kMaxStaleRects = 160;
struct StaleList {
	StaleRect rects[kMaxStaleRects];
	int count = 0;
};

constexpr int kMaxScanFbs = 3;
std::uint16_t *g_scanFb[kMaxScanFbs] = {nullptr, nullptr, nullptr};
int g_scanFbCount = 1;
int g_frontIndex = 0;
// The buffer the renderer draws into: the one after the front in rotation.
int g_drawIndex = 1;
StaleList g_stale[kMaxScanFbs];
// New pixels reached the back buffer since the last swap.
bool g_publishPending = false;
// g_backReady (declared above replayCanvas): the back buffer is released by the
// panel and patched up to date, so drawing into it is safe. Cleared by every swap.
// Serializes readying, in case a draw on another task races the frame task.
SemaphoreHandle_t g_readyMutex = nullptr;
// A sideways scroll copy is running on the row worker (gea_display_scroll_rect_begin).
bool g_scrollCopyInFlight = false;
// The viewport a finished scroll copy wrote back row by row this frame (empty:
// x0 > x1). publishSwap need not write back the frame's draws inside it.
DisplayFlushRect g_cleanRect = {0, 0, -1, -1};
// An unstaged replay drew while the scroll copy ran (gea_display_note_direct_draw).
bool g_directDrawDuringCopy = false;
// Every pixel this frame drew went out through scanoutRowCopy,
// already written back from the cache (a fused frame with no direct draws).
bool g_frameWrittenBack = false;
// Per buffer, per screen row: the signature (gea_display_scroll_rect_rows) of
// what the last frame drawn into that buffer left in its scroll viewport, valid
// only while every frame since was a sideways scroll copy of the same viewport.
// A row whose new signature matches the back buffer's already holds the
// shifted picture, so the copy skips it.
std::uint32_t *g_rowSig[kMaxScanFbs] = {nullptr, nullptr, nullptr};
DisplayFlushRect g_rowSigRect[kMaxScanFbs] = {};
bool g_rowSigValid[kMaxScanFbs] = {false, false, false};
// The coming copy's signatures (screen rows from g_nextSigY0), and whether the
// frame being drawn is a sideways scroll copy.
const std::uint32_t *g_nextSig = nullptr;
int g_nextSigY0 = 0, g_nextSigRows = 0;
bool g_frameIsScrollCopy = false;
std::uint8_t g_skipRow[kHeight];
// Counted by the driver's frame-complete callback, which runs right after the
// bounce ISR has latched the driver's current buffer for the next frame.
volatile std::uint32_t g_frameSeq = 0;
// DIAG: panel VSYNCs. In steady state there is exactly one frame-buffer wrap per
// VSYNC; a difference means the bounce ISR fell behind and the driver had to
// restart the scan (the visible "shift").
volatile std::uint32_t g_vsyncSeq = 0;
std::uint32_t g_diagVsync0 = 0, g_diagFrame0 = 0;
// Buffer i is released by the panel once g_frameSeq reaches g_freeSeq[i].
std::uint32_t g_freeSeq[kMaxScanFbs] = {0, 0, 0};
int g_panelIsrCore = -1;
// Set while the bounce ISR refills a buffer (see scanoutRowCopy).
DRAM_ATTR volatile bool g_bounceFilling = false;
// Refill collisions, read from the hardware: the GDMA channel feeding the
// panel reports the descriptor it is on, and so the buffer it is sending. If
// that is the buffer the ISR is refilling, the panel shows some of its old
// rows (thin glitch lines): at the start of a fill it means the fills are a
// buffer out of step with the DMA, at the end that the fill ran late.
// GEADEV PANEL reads the counts.
struct RefillStats {
	// [0]: the channel's current descriptor; [1]: the one before it (the
	// registers' meaning is loosely documented, so both are read).
	std::uint32_t fills, atStart[2], atEnd[2], unknown, corrected, fallback;
	std::uint16_t atStartPos[32], atEndPos[32];  // per bounce position, [0] reading
};
DRAM_ATTR RefillStats g_refill = {};
DRAM_ATTR int g_panelDmaChannel = -1;

// A private semaphore, not a task notification: the frame task's notification
// slot belongs to whoever else wakes it, and there is only one slot.
SemaphoreHandle_t g_frameSem = nullptr;
volatile bool g_swapWaiting = false;
portMUX_TYPE g_swapMux = portMUX_INITIALIZER_UNLOCKED;
// DIAG: publish cost decomposition, printed every kSwapDiagFrames swaps.
constexpr int kSwapDiagFrames = 100;
struct SwapDiag {
	std::int64_t copyUs = 0, waitUs = 0, totalUs = 0, rasterUs = 0;
	std::int64_t px = 0, patchPx = 0, preparePx = 0;
	int prepares = 0;
	int frames = 0, waits = 0, timeouts = 0, unready = 0, late = 0, regions = 0, rings = 0, fallbacks = 0;
	int paths[4] = {0, 0, 0, 0};  // flush, flushRects, rasterized, strip
	std::int64_t windowStartUs = 0;
} g_swapDiag;
std::int64_t g_lastBlockUs = 0;

// ── Ring scroll (bounce mode) ────────────────────────────────────────────────
// The S3's LCD peripheral has no scroll register, but in bounce mode the CPU
// fills every scanline batch, and this file does that fill itself (the driver
// owns no framebuffer: `no_fb` + on_bounce_empty). So a vertical scroll of a
// full-width region is the Canvas's ring mapping (Canvas::rowToPhysical) plus
// a per-buffer offset the fill applies: no pixel moves, only the rows the
// scroll exposes are drawn.
//
// Each scanout buffer carries its own mapping, because the buffers hold
// different frames. The draw buffer's mapping is the canvas's; the others lag
// by the rotations scrolled since they were last current (g_pendingRot), and
// their stale rects are shifted with every scroll so they stay in the current
// frame's coordinates. A buffer that is fully stale adopts the front's mapping
// when it is readied, and is then patched in full.
struct ScanMap {
	int y = 0, h = 0, off = 0;  // region rows [y, y+h); off in [0, h)
};
ScanMap g_map[kMaxScanFbs];
int g_pendingRot[kMaxScanFbs] = {0, 0, 0};
bool g_mapReset[kMaxScanFbs] = {false, false, false};
int g_regionY = 0, g_regionH = 0;
bool g_ringScroll = false;
// The buffer the panel is scanning and the one it switches to at the next frame
// wrap, with the scanned buffer's mapping latched at that wrap.
volatile int g_scanIndex = 0;
volatile int g_pendingScan = 0;
ScanMap g_scanMap;

__attribute__((always_inline)) inline int mapRow(const ScanMap &m, int y)
{
	if (m.h <= 0 || y < m.y || y >= m.y + m.h) return y;
	int rel = y - m.y + m.off;
	if (rel >= m.h) rel -= m.h;
	return m.y + rel;
}

int normOffset(int value, int h) { return h > 0 ? ((value % h) + h) % h : 0; }

std::uint16_t *drawRow(int y) { return g_drawFramebuffer + static_cast<std::size_t>(mapRow(g_map[g_drawIndex], y)) * kWidth; }

void applyCanvasMap()
{
	const ScanMap &m = g_map[g_drawIndex];
	if (g_canvas.pixels() == g_drawFramebuffer) g_canvas.setScrollRegion(m.y, m.h, m.off);
	if (g_workerCanvas.pixels() == g_drawFramebuffer) g_workerCanvas.setScrollRegion(m.y, m.h, m.off);
}

// When the panel last started displaying a frame, and when the bounce ISR last
// latched a buffer for the next one: together they say how far down the frame in
// flight the panel has read (see backRowReleased).
volatile std::int64_t g_vsyncUs = 0;
volatile std::int64_t g_frameCompleteUs = 0;

// VSYNC interrupt lateness: the interval between VSYNC interrupts against
// the frame period (a late one is a late DMA restart).
DRAM_ATTR std::int32_t g_vsyncLateMaxUs = 0;
DRAM_ATTR std::uint32_t g_vsyncLate200 = 0;
// Where the DMA sits at each VSYNC: bin = buffer * 16 + descriptor index.
DRAM_ATTR std::uint32_t g_vsyncAt[32] = {};
DRAM_ATTR std::uint32_t g_vsyncRestarts = 0;
DRAM_ATTR std::uint32_t g_vsyncSkipped = 0;
DRAM_ATTR std::uint32_t g_vsyncRealigned = 0;
DRAM_ATTR std::uint32_t g_frameEofFills = 0;     // refills from DMA EOFs since the last VSYNC
DRAM_ATTR std::uint32_t g_shortFrames = 0;       // VSYNCs that found fewer: the driver restarts there
DRAM_ATTR std::uint32_t g_shortFramesLate = 0;   // ... on a late interrupt
// The last few VSYNCs that found the DMA out of place, with their context.
struct SlipEvent {
	std::uint32_t ms;
	std::uint16_t bin;
	std::uint16_t lateUs;
	std::uint16_t prevLateUs;
	std::uint8_t eofFills;
	std::uint8_t prevEofFills;
};
DRAM_ATTR SlipEvent g_slips[6] = {};
DRAM_ATTR std::uint32_t g_slipCount = 0;
DRAM_ATTR std::uint16_t g_prevLateUs = 0;
DRAM_ATTR std::uint8_t g_prevEofFills = 0;
DRAM_ATTR std::uint32_t g_vsyncPhaseQ4 = 0;
// A VSYNC interrupt this late (past the hardware event, in 1/16 us) may
// already be inside the next frame's first lines: no restart then.
constexpr std::uint32_t kVsyncOnTimeQ4 = 40 * 16;
int panelDmaBin();
IRAM_ATTR bool onPanelVsync(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *, void *)
{
	const std::int64_t now = esp_timer_get_time();
	{
		// Realign the DMA with the panel at every VSYNC that comes in time: the
		// DMA is then still parked on buffer 0 just past its first descriptor
		// (the FIFO's preserved pixels), waiting out the blanking. A later
		// interrupt finds it already further in, sending the next frame, and a
		// restart then would shift that frame sideways, so it is skipped; a
		// slip waits for the next VSYNC in time. (The driver's own
		// CONFIG_LCD_RGB_RESTART_IN_VSYNC restarts at every VSYNC, late or not:
		// the screen shook while scrolling. With no restart at all a slip stayed
		// on screen, the picture split and shifted.)
		// The descriptor alone leaves ~140 us of the next frame unseen (the
		// DMA moves through descriptor 1 slowly at first), so the interrupt's
		// own lateness gates it too. VSYNC is a fixed hardware period: track its
		// phase from the interrupts that come in time and measure this one
		// against it.
		// Phase in 1/16 us, 32-bit: the anchor moves every frame in time, so
		// the distance from it stays small.
		constexpr auto &d = gea::platform::board::display;
		constexpr std::uint32_t kPeriodQ4 = static_cast<std::uint32_t>(
			(static_cast<std::uint64_t>(kWidth + d.hsyncPulseWidth + d.hsyncBackPorch + d.hsyncFrontPorch) *
			 (kHeight + d.vsyncPulseWidth + d.vsyncBackPorch + d.vsyncFrontPorch) * 16000000ULL) / kPixelClockHz);
		const std::uint32_t nowQ4 = static_cast<std::uint32_t>(now) << 4;
		std::uint32_t lateQ4 = 0;
		const std::uint32_t sinceQ4 = nowQ4 - g_vsyncPhaseQ4;
		if (g_vsyncPhaseQ4 == 0 || sinceQ4 > 0x40000000u) {
			g_vsyncPhaseQ4 = nowQ4;
		} else {
			const std::uint32_t n = (sinceQ4 + kPeriodQ4 / 2) / kPeriodQ4;
			const std::uint32_t expected = g_vsyncPhaseQ4 + n * kPeriodQ4;
			const std::int32_t late = static_cast<std::int32_t>(nowQ4 - expected);
			if (late < 0) {
				g_vsyncPhaseQ4 = nowQ4;  // the estimate ran late: this one is the earliest seen
			} else {
				lateQ4 = static_cast<std::uint32_t>(late);
				if (lateQ4 <= kVsyncOnTimeQ4) g_vsyncPhaseQ4 = expected + lateQ4 / 4;  // follow drift slowly
			}
		}
		const int bin = panelDmaBin();
		if (bin >= 0 && bin < 32) g_vsyncAt[bin]++;
		// An interrupt in time that finds the DMA anywhere but buffer 0 /
		// descriptor 1 means the DMA itself is out of step with the panel
		// (behind after an underrun, or ahead): that is the slip a restart
		// exists for, so it restarts too.
		{
			const std::uint32_t fills = g_frameEofFills;
			g_frameEofFills = 0;
			const std::uint16_t lateUs = static_cast<std::uint16_t>(std::min<std::uint32_t>(lateQ4 >> 4, 65535));
			if (fills < static_cast<std::uint32_t>(kHeight / kRgbBounceRows)) {
				g_shortFrames++;
				if (lateQ4 > kVsyncOnTimeQ4) g_shortFramesLate++;
			}
			if (bin != 1 && g_vsyncSeq > 200) {
				SlipEvent &e = g_slips[g_slipCount % 6];
				e = {static_cast<std::uint32_t>(now / 1000), static_cast<std::uint16_t>(bin), lateUs, g_prevLateUs,
				     static_cast<std::uint8_t>(std::min<std::uint32_t>(fills, 255)), g_prevEofFills};
				g_slipCount++;
			}
			g_prevLateUs = lateUs;
			g_prevEofFills = static_cast<std::uint8_t>(std::min<std::uint32_t>(fills, 255));
		}
		if (lateQ4 <= kVsyncOnTimeQ4) {
			if (bin != 1) g_vsyncRealigned++;
			if (esp_lcd_rgb_panel_restart(panel) == ESP_OK) {
				g_vsyncRestarts++;
			}
		} else {
			g_vsyncSkipped++;
		}
	}
	if (g_vsyncUs != 0) {
		const auto &d = gea::platform::board::display;
		const std::int64_t frameUs = static_cast<std::int64_t>(kWidth + d.hsyncPulseWidth + d.hsyncBackPorch + d.hsyncFrontPorch) *
		                             (kHeight + d.vsyncPulseWidth + d.vsyncBackPorch + d.vsyncFrontPorch) * 1000000 / kPixelClockHz;
		std::int32_t dev = static_cast<std::int32_t>(now - g_vsyncUs - frameUs);
		if (dev < 0) dev = -dev;
		if (dev > g_vsyncLateMaxUs) g_vsyncLateMaxUs = dev;
		if (dev > 200) g_vsyncLate200++;
	}
	g_vsyncSeq = g_vsyncSeq + 1;
	g_vsyncUs = now;
	return false;
}

// The panel stops reading the old front once the refill starts on the next
// frame's first band: switch the scanned buffer there.
IRAM_ATTR void latchScanBuffer()
{
	portENTER_CRITICAL_SAFE(&g_swapMux);
	g_scanIndex = g_pendingScan;
	g_scanMap = g_map[g_scanIndex];
	g_frameSeq = g_frameSeq + 1;
	portEXIT_CRITICAL_SAFE(&g_swapMux);
	g_frameCompleteUs = esp_timer_get_time();
}

IRAM_ATTR bool wakeSwapWaiter()
{
	if (!g_swapWaiting) return false;
	if (!xPortInIsrContext()) {
		xSemaphoreGive(g_frameSem);
		return false;
	}
	BaseType_t woken = pdFALSE;
	xSemaphoreGiveFromISR(g_frameSem, &woken);
	return woken == pdTRUE;
}

// Set by each refill that chose its own band (see onBounceEmpty); the driver
// calls this right after the refill, and its frame count is then not used.
DRAM_ATTR bool g_bandManaged = false;

IRAM_ATTR bool onPanelFrameComplete(esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t *, void *)
{
	if (g_bandManaged) return false;
	latchScanBuffer();
	return wakeSwapWaiter();
}

// Fill one bounce buffer from the scanned buffer through its ring mapping:
// contiguous physical runs, then a cache preload of the next batch's first run
// (the driver's own memcpy path does the same preload).
// Which of the GDMA channel's descriptors point into [buf, buf+len): bit 0
// the current one, bit 1 the previous; 4 when the channel is unknown.
IRAM_ATTR int panelDmaSending(const void *buf, int len)
{
#if CONFIG_IDF_TARGET_ESP32S3
	int ch = g_panelDmaChannel;
	if (ch < 0) {
		for (int i = 0; i < 5; ++i)
			if (GDMA.channel[i].out.peri_sel.sel == 5 && GDMA.channel[i].out.dscr != 0) ch = i;  // 5: LCD_CAM
		if (ch < 0) return 4;
		g_panelDmaChannel = ch;
	}
	const auto lo = reinterpret_cast<std::uintptr_t>(buf);
	const auto hi = lo + static_cast<std::uintptr_t>(len);
	auto inside = [&](std::uint32_t desc) {
		if (desc < SOC_DRAM_LOW || desc >= SOC_DRAM_HIGH) return false;
		const auto b = static_cast<std::uintptr_t>(reinterpret_cast<const volatile std::uint32_t *>(desc)[1]);
		return b >= lo && b < hi;
	};
	return (inside(GDMA.channel[ch].out.dscr) ? 1 : 0) | (inside(GDMA.channel[ch].out.dscr_bf0) ? 2 : 0);
#else
	return 4;
#endif
}

// The two bounce buffers; [0] is the one the driver restarts the DMA on at
// every VSYNC, and the first it fills (at position 0) when transmission starts.
DRAM_ATTR std::uint8_t *g_bounceBuf[2] = {nullptr, nullptr};
// The newest frame (by VSYNC count) the scanned buffer has been switched for.
DRAM_ATTR std::uint32_t g_latchedFor = 0;

// The bounce buffer the GDMA channel is sending now, or -1 if unknown.
IRAM_ATTR int panelDmaBuffer(int len)
{
	for (int i = 0; i < 2; ++i)
		if (g_bounceBuf[i] && (panelDmaSending(g_bounceBuf[i], len) & 6) == 2) return i;
	return -1;
}

IRAM_ATTR int panelDmaBin()
{
#if CONFIG_IDF_TARGET_ESP32S3
	if (g_panelDmaChannel < 0 || !g_bounceBuf[1]) return -1;
	const std::uint32_t desc = GDMA.channel[g_panelDmaChannel].out.dscr_bf0;
	if (desc < SOC_DRAM_LOW || desc >= SOC_DRAM_HIGH) return -1;
	const auto b = static_cast<std::uintptr_t>(reinterpret_cast<const volatile std::uint32_t *>(desc)[1]);
	for (int i = 0; i < 2; ++i) {
		const auto lo = reinterpret_cast<std::uintptr_t>(g_bounceBuf[i]);
		if (b >= lo && b < lo + 24u * kWidth * 2u) return i * 16 + static_cast<int>((b - lo) / 4095u);
	}
#endif
	return -1;
}

// Refill one bounce buffer from the scanned buffer through its ring mapping:
// contiguous physical runs, then a cache preload of the next band's first run.
//
// The buffer and band are chosen here, not taken from the driver. The driver
// picks the buffer from a running count of DMA EOF interrupts, and with
// CONFIG_LCD_RGB_RESTART_IN_VSYNC (the DMA restarts on buffer 0 at every
// VSYNC) that count is never reset: one EOF lost to interrupt latency leaves
// every later refill writing the buffer the DMA is sending, one band late,
// until another loss flips it back. The panel then shows the head of each
// 24-row band from the old contents -- thin lines, on a still screen too. On
// the bench it slipped at most boots and under load, and stayed slipped for
// seconds at a time. So: the channel's current descriptor says which buffer
// the DMA is sending (refill the other), and the time since VSYNC, snapped to
// that buffer's parity (band k is in buffer k % 2 after the restart), says
// which band it is on (refill the next).
IRAM_ATTR bool refillBand(std::uint8_t *given, int posPx, int lenBytes)
{
	g_bounceFilling = true;
	constexpr std::size_t kRowBytes = static_cast<std::size_t>(kWidth) * sizeof(std::uint16_t);
	const int rows = static_cast<int>(static_cast<std::size_t>(lenBytes) / kRowBytes);
	if (esp_timer_get_time() - g_vsyncUs >= 40) g_frameEofFills = g_frameEofFills + 1;
	const int bands = kHeight / rows;

	std::uint8_t *dst = given;
	int band = posPx / (kWidth * rows);
	bool managed = false;
	std::uint32_t frame = 0;
	const std::uint32_t vseq = g_vsyncSeq;
	if (vseq != 0 && g_bounceBuf[1]) {
		const auto &d = gea::platform::board::display;
		const std::int64_t lineClocks = kWidth + d.hsyncPulseWidth + d.hsyncBackPorch + d.hsyncFrontPorch;
		const int frameLines = kHeight + d.vsyncPulseWidth + d.vsyncBackPorch + d.vsyncFrontPorch;
		const std::int64_t line = (esp_timer_get_time() - g_vsyncUs) * kPixelClockHz / (lineClocks * 1000000);
		if (line <= frameLines + rows) {
			if (posPx == 0 && line < 2) {
				// The driver refilling buffer 0 inside the VSYNC interrupt, just
				// before it restarts the DMA there: buffer 0 already holds band 0
				// (refilled while the DMA sent band 19), and a copy here would hold
				// the restart back.
				g_bandManaged = true;
				g_bounceFilling = false;
				return false;
			} else if (const int p = panelDmaBuffer(lenBytes); p >= 0) {
				// The DMA moves to band k about 4 lines after line 24k (measured).
				constexpr int kLeadLines = 4;
				const int num = static_cast<int>(line) - kLeadLines - rows * p + rows / 2;
				const int span = 2 * rows;
				const int k = p + 2 * (num >= 0 ? num / span : -((span - 1 - num) / span));
				int t = k + 1;
				frame = vseq;
				if (t >= bands) {
					t -= bands;
					frame = vseq + 1;
				}
				managed = t >= 0;
				band = t;
				dst = g_bounceBuf[1 - p];
			}
		}
	}
	g_bandManaged = managed;
	bool yield = false;
	if (managed && static_cast<std::int32_t>(frame - g_latchedFor) > 0) {
		g_latchedFor = frame;
		latchScanBuffer();
		yield = wakeSwapWaiter();
	}
	const int sendingAtStart = panelDmaSending(dst, lenBytes);

	const int row0 = band * rows;
	const std::uint16_t *fb = g_scanFb[g_scanIndex];
	const ScanMap m = g_scanMap;
	for (int r = 0; r < rows;) {
		const int phys = mapRow(m, row0 + r);
		int run = 1;
		while (r + run < rows && mapRow(m, row0 + r + run) == phys + run) ++run;
		std::memcpy(dst + static_cast<std::size_t>(r) * kRowBytes, fb + static_cast<std::size_t>(phys) * kWidth, static_cast<std::size_t>(run) * kRowBytes);
		r += run;
	}
#if CONFIG_IDF_TARGET_ESP32S3
	const int next = row0 + rows;
	if (next < kHeight) {
		const int phys = mapRow(m, next);
		int run = 1;
		while (run < rows && next + run < kHeight && mapRow(m, next + run) == phys + run) ++run;
		const std::uint32_t bytes = static_cast<std::uint32_t>(static_cast<std::size_t>(run) * kRowBytes);
		Cache_Start_DCache_Preload(reinterpret_cast<std::uint32_t>(fb + static_cast<std::size_t>(phys) * kWidth), bytes, 0);
	}
#endif
	{
		const int sendingAtEnd = panelDmaSending(dst, lenBytes);
		g_refill.fills++;
		if (!managed) g_refill.fallback++;
		else if (dst != given || band != posPx / (kWidth * rows)) g_refill.corrected++;
		if (sendingAtStart & 4) g_refill.unknown++;
		for (int k = 0; k < 2; ++k) {
			if (!(sendingAtStart & 4) && (sendingAtStart & (2 >> k))) g_refill.atStart[k]++;
			if (!(sendingAtEnd & 4) && (sendingAtEnd & (2 >> k))) g_refill.atEnd[k]++;
		}
		if (band >= 0 && band < 32) {
			if (sendingAtStart & 2) g_refill.atStartPos[band]++;
			if (sendingAtEnd & 2) g_refill.atEndPos[band]++;
		}
	}
	g_bounceFilling = false;
	return yield;
}

IRAM_ATTR bool onBounceEmpty(esp_lcd_panel_handle_t, void *bounce, int posPx, int lenBytes, void *)
{
	auto *given = static_cast<std::uint8_t *>(bounce);
	if (!g_bounceBuf[0]) {
		if (posPx == 0) g_bounceBuf[0] = given;
	} else if (!g_bounceBuf[1] && given != g_bounceBuf[0]) {
		g_bounceBuf[1] = given;
	}
	return refillBand(given, posPx, lenBytes);
}

int rectArea(int x0, int y0, int x1, int y1) { return (x1 - x0 + 1) * (y1 - y0 + 1); }

void staleAdd(StaleList &list, int x0, int y0, int x1, int y1)
{
	for (int i = 0; i < list.count; ++i) {
		StaleRect &r = list.rects[i];
		if (x0 >= r.x0 && y0 >= r.y0 && x1 <= r.x1 && y1 <= r.y1) return;  // already covered
		// Merge when the bounding box costs no more than copying both: an
		// animation's dirty box drifts a little frame to frame, and copying the two
		// overlapping boxes separately would move the overlap twice.
		const int ux0 = std::min(x0, r.x0), uy0 = std::min(y0, r.y0);
		const int ux1 = std::max(x1, r.x1), uy1 = std::max(y1, r.y1);
		if (rectArea(ux0, uy0, ux1, uy1) <= rectArea(x0, y0, x1, y1) + rectArea(r.x0, r.y0, r.x1, r.y1)) {
			list.rects[i] = list.rects[--list.count];
			staleAdd(list, ux0, uy0, ux1, uy1);  // the grown box may now swallow others
			return;
		}
	}
	if (list.count < kMaxStaleRects) {
		list.rects[list.count++] = {x0, y0, x1, y1};
		return;
	}
	// Full: fold everything into one bounding box (correct, just copies more).
	StaleRect box = {x0, y0, x1, y1};
	for (int i = 0; i < list.count; ++i) {
		const StaleRect &r = list.rects[i];
		box = {std::min(box.x0, r.x0), std::min(box.y0, r.y0), std::max(box.x1, r.x1), std::max(box.y1, r.y1)};
	}
	list.rects[0] = box;
	list.count = 1;
}

// A region the back buffer now holds new pixels for: the front lags there.
void markStale(int x0, int y0, int x1, int y1)
{
	for (int i = 0; i < g_scanFbCount; ++i)
		if (i != g_drawIndex) staleAdd(g_stale[i], x0, y0, x1, y1);
	g_publishPending = true;
}

// Block (not spin) until the panel has stopped reading the back buffer. Normally
// the swap latched long ago and this returns at once; it only waits when frames
// are published faster than the panel refreshes.
void waitBackBufferFree()
{
	const std::uint32_t freeSeq = g_freeSeq[g_drawIndex];
	if (static_cast<std::int32_t>(g_frameSeq - freeSeq) >= 0) return;
	const std::int64_t w0 = esp_timer_get_time();
	xSemaphoreTake(g_frameSem, 0);  // drop a stale give from an earlier wait
	g_swapWaiting = true;
	while (static_cast<std::int32_t>(g_frameSeq - freeSeq) < 0) {
		xSemaphoreTake(g_frameSem, pdMS_TO_TICKS(5));
		if (esp_timer_get_time() - w0 > 100000) {
			// The panel stopped scanning (or never started): don't hang the frame.
			g_swapDiag.timeouts++;
			break;
		}
	}
	g_swapWaiting = false;
	const std::int64_t now = esp_timer_get_time();
	g_swapDiag.waitUs += now - w0;
	g_swapDiag.waits++;
	g_lastBlockUs = now;
}

// Hand the back buffer to the panel. The driver switches to it when the bounce
// ISR wraps to the next frame; g_freeSeq says when the OLD front is released.
// If this task runs on the ISR's core, masking interrupts around the switch
// guarantees the next frame-complete already scans the new buffer; from the other
// core the ISR may be mid-wrap with the old index, so one more frame is required.
void commitSwap(int back)
{
	std::uint32_t seq;
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	// The bounce fill reads g_scanFb[g_scanIndex], which the frame-complete ISR
	// switches to g_pendingScan at the wrap; the shared spinlock makes "the next
	// wrap takes it" exact from either core, so the old front is released at the
	// very next frame end.
	taskENTER_CRITICAL(&g_swapMux);
	g_pendingScan = back;
	seq = g_frameSeq + 1;
	taskEXIT_CRITICAL(&g_swapMux);
#else
	// Direct scanout: draw_bitmap writes the whole frame back from the cache
	// (too long for a critical section), and the GDMA may prefetch one more pass
	// of the old buffer, so always allow two frame ends.
	esp_lcd_panel_draw_bitmap(g_panel, 0, 0, kWidth, kHeight, g_scanFb[back]);
	seq = g_frameSeq + 2;
#endif
	g_freeSeq[g_frontIndex] = seq;
	std::uint16_t *oldBack = g_scanFb[back];
	g_frontIndex = back;
	g_drawIndex = (back + 1) % g_scanFbCount;
	g_framebuffer = g_scanFb[back];
	g_drawFramebuffer = g_scanFb[g_drawIndex];
	g_backReady = false;
	// Follow the swap with the draw canvases -- unless the render layer has one
	// bound to a scratch buffer right now (it rebinds via rebindCanvasToFramebuffer).
	if (g_canvas.pixels() == oldBack) g_canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	if (g_workerCanvas.pixels() == oldBack) g_workerCanvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	applyCanvasMap();
}

// One line per kSwapDiagFrames published frames, the same in both modes:
// publish = whole Display flush call, raster = fused raster inside it, copy =
// pixel copies into scanout memory, wait = blocked on the panel releasing the back buffer.
void noteFrameDiag(int path, std::int64_t totalUs, int px)
{
	if (px <= 0) return;
	g_swapDiag.paths[path]++;
	g_swapDiag.totalUs += totalUs;
	g_swapDiag.px += px;
	if (g_swapDiag.windowStartUs == 0) g_swapDiag.windowStartUs = esp_timer_get_time();
	if (++g_swapDiag.frames >= kSwapDiagFrames) {
		const std::int64_t now = esp_timer_get_time();
		const int n = g_swapDiag.frames;
		const std::uint32_t vs = g_vsyncSeq, fr = g_frameSeq;
		const double secs = static_cast<double>(now - g_swapDiag.windowStartUs) / 1e6;
		std::printf("[rgb-swap] panel vsync/s=%.2f wraps/s=%.2f slip=%d | swap=%d frames=%d paths=%d/%d/%d/%d publish=%dus raster=%dus copy=%dus wait=%dus(%d waits, %d timeouts) px=%d patch_px=%d unready=%d late=%d regions=%d rings=%d fallbacks=%d prepares=%d prepare_px=%d frames/s=%.1f\n",
		            (vs - g_diagVsync0) / secs, (fr - g_diagFrame0) / secs,
		            static_cast<int>((vs - g_diagVsync0) - (fr - g_diagFrame0)),
		            g_swapFbs ? 1 : 0,
		            n,
		            g_swapDiag.paths[0], g_swapDiag.paths[1], g_swapDiag.paths[2], g_swapDiag.paths[3],
		            static_cast<int>(g_swapDiag.totalUs / n),
		            static_cast<int>(g_swapDiag.rasterUs / n),
		            static_cast<int>(g_swapDiag.copyUs / n),
		            static_cast<int>(g_swapDiag.waitUs / n),
		            g_swapDiag.waits,
		            g_swapDiag.timeouts,
		            static_cast<int>(g_swapDiag.px / n),
		            static_cast<int>(g_swapDiag.patchPx / n),
		            g_swapDiag.unready,
		            g_swapDiag.late,
		            g_swapDiag.regions, g_swapDiag.rings, g_swapDiag.fallbacks,
		            g_swapDiag.prepares, static_cast<int>(g_swapDiag.preparePx / n),
		            n * 1000000.0 / static_cast<double>(now - g_swapDiag.windowStartUs));
		g_swapDiag = {};
		g_swapDiag.windowStartUs = now;
		g_diagVsync0 = vs;
		g_diagFrame0 = fr;
	}
}

// One row's copy into a scanout buffer, kept out of the bounce refills'
// way. A refill reads its rows from PSRAM through the data cache it shares
// with every core, and has one bounce period to finish or the LCD DMA sends a
// half-refilled band (a tear). Two things land in that window: the other
// core's PSRAM traffic, and -- worse -- dirty framebuffer lines a draw left in
// the cache, which the refill's own reads evict and so write back inside its
// window. So a copy on the other core waits out a refill in progress, and every
// copy writes its own lines back at once, while the refill is not running
// (on the ISR's core it cannot be); publishSwap does the same for the frame's
// direct draws. Scrolling Profile, 30-row refills overran 1.3% of their
// 1.64 ms windows (to 1.84 ms) before this; after it, none in ~12k refills per
// tab on Profile, Shop and Analytics, for about one frame per second.
// Spins (no PSRAM access: the flag is in internal RAM) while a refill runs on
// the ISR's core; bounded, so a stopped panel cannot hang the caller.
inline void waitOutRefill()
{
	const std::uint32_t t0 = esp_cpu_get_cycle_count();
	while (g_bounceFilling && esp_cpu_get_cycle_count() - t0 < 240u * 3000u) {
	}
}
// One framebuffer row written back from the data cache. esp_cache_msync does the
// same under the same lock, but also takes a mutex and validates its arguments
// on every call: ~5 us a row, which a frame of 64 bouncing balls (~2,900 short
// rows) paid ~14 ms for. The rows here are always cacheable PSRAM.
inline void writebackRow(const void *dst, std::size_t bytes)
{
	esp_cache_sync_ops_enter_critical_section();
	cache_hal_writeback_addr(reinterpret_cast<std::uint32_t>(dst), bytes);
	esp_cache_sync_ops_exit_critical_section();
}
// Row-repeat records (see g_rowSum): a write they did not see forgets them.
void forgetRowSums();
inline void forgetRowSumAt(const void *dst);
void scanoutRowCopy(void *dst, const void *src, std::size_t bytes)
{
	forgetRowSumAt(dst);
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	if (static_cast<int>(xPortGetCoreID()) != g_panelIsrCore) waitOutRefill();
	std::memcpy(dst, src, bytes);
	writebackRow(dst, bytes);
#else
	std::memcpy(dst, src, bytes);
#endif
}

// Copy rect r of the front buffer into the back buffer.
void patchRect(const StaleRect &r)
{
	const std::uint16_t *src = g_scanFb[g_frontIndex];
	std::uint16_t *dst = g_scanFb[g_drawIndex];
	const std::size_t bytes = static_cast<std::size_t>(r.x1 - r.x0 + 1) * sizeof(std::uint16_t);
	const ScanMap &ms = g_map[g_frontIndex];
	const ScanMap &md = g_map[g_drawIndex];
	for (int y = r.y0; y <= r.y1; ++y) {
		scanoutRowCopy(&dst[static_cast<std::size_t>(mapRow(md, y)) * kWidth + r.x0],
		               &src[static_cast<std::size_t>(mapRow(ms, y)) * kWidth + r.x0], bytes);
	}
	cacheWritebackRect(dst, r.x0, r.y0, r.x1, r.y1);
}

// a minus b, appended to out (up to 4 pieces). Returns false if out is full.
bool subtractRect(const StaleRect &a, const StaleRect &b, StaleRect *out, int &n, int cap)
{
	if (b.x1 < a.x0 || b.x0 > a.x1 || b.y1 < a.y0 || b.y0 > a.y1) {
		if (n >= cap) return false;
		out[n++] = a;
		return true;
	}
	StaleRect pieces[4];
	int k = 0;
	if (b.y0 > a.y0) pieces[k++] = {a.x0, a.y0, a.x1, b.y0 - 1};
	if (b.y1 < a.y1) pieces[k++] = {a.x0, b.y1 + 1, a.x1, a.y1};
	const int my0 = std::max(a.y0, b.y0), my1 = std::min(a.y1, b.y1);
	if (b.x0 > a.x0) pieces[k++] = {a.x0, my0, b.x0 - 1, my1};
	if (b.x1 < a.x1) pieces[k++] = {b.x1 + 1, my0, a.x1, my1};
	if (n + k > cap) return false;
	for (int i = 0; i < k; ++i) out[n++] = pieces[i];
	return true;
}

// Bring the back buffer up to date from the front, except inside `skip` (rects
// the caller is about to repaint completely). Skipping is only an optimisation:
// the patch always runs before the repaint, so copying extra is still correct.
int patchBack(const DisplayFlushRect *skip, int skipCount)
{
	StaleList &list = g_stale[g_drawIndex];
	// Each stale rect is cut by the skip rects on its own, so a long stale list
	// needs no more scratch than one rect's fragments.
	constexpr int kCap = 64;
	StaleRect work[2][kCap];
	int px = 0;
	for (int r = 0; r < list.count; ++r) {
		const StaleRect &a = list.rects[r];
		int n = 1, cur = 0;
		work[0][0] = a;
		for (int s = 0; s < skipCount && n > 0; ++s) {
			const StaleRect b = {skip[s].x0, skip[s].y0, skip[s].x1, skip[s].y1};
			if (b.x1 < a.x0 || b.x0 > a.x1 || b.y1 < a.y0 || b.y0 > a.y1) continue;
			int m = 0;
			bool ok = true;
			for (int i = 0; i < n && ok; ++i) ok = subtractRect(work[cur][i], b, work[cur ^ 1], m, kCap);
			if (!ok) break;  // too fragmented: copy what we have, still correct
			cur ^= 1;
			n = m;
		}
		for (int i = 0; i < n; ++i) {
			patchRect(work[cur][i]);
			px += rectArea(work[cur][i].x0, work[cur][i].y0, work[cur][i].x1, work[cur][i].y1);
		}
	}
	list.count = 0;
	return px;
}

// Rows [first, first+count) of buffer `fb`, rotated left by `by` rows, through
// one row of internal RAM (juggling: every row moves once).
void rotateRows(std::uint16_t *fb, int first, int count, int by)
{
	by = normOffset(by, count);
	if (count <= 1 || by == 0) return;
	forgetRowSums();
	static std::uint16_t temp[kWidth];
	constexpr std::size_t kRowBytes = static_cast<std::size_t>(kWidth) * sizeof(std::uint16_t);
	auto row = [&](int i) { return fb + static_cast<std::size_t>(first + i) * kWidth; };
	int a = count, b = by;
	while (b) { const int t = a % b; a = b; b = t; }
	for (int start = 0; start < a; ++start) {
		std::memcpy(temp, row(start), kRowBytes);
		int i = start;
		for (;;) {
			int j = i + by;
			if (j >= count) j -= count;
			if (j == start) break;
			std::memcpy(row(i), row(j), kRowBytes);
			i = j;
		}
		std::memcpy(row(i), temp, kRowBytes);
	}
}

// Physically re-lay buffer i (not being scanned) from its mapping to `target`
// without changing what any logical row holds.
void remapBuffer(int i, const ScanMap &target)
{
	ScanMap &m = g_map[i];
	if (m.y == target.y && m.h == target.h && m.off == target.off) return;
	if (m.h > 0) rotateRows(g_scanFb[i], m.y, m.h, m.off);                  // to identity
	if (target.h > 0) rotateRows(g_scanFb[i], target.y, target.h, -target.off);  // to target
	m = target;
}

// Bring the draw buffer's mapping up to the current frame before it is patched.
void bringMapCurrent(int i)
{
	ScanMap &m = g_map[i];
	if (g_mapReset[i]) {
		// Fully stale: any layout works, and matching the front's makes the full
		// patch a straight copy.
		const ScanMap &front = g_map[g_frontIndex];
		m = (front.y == g_regionY && front.h == g_regionH) ? front : ScanMap{g_regionY, g_regionH, 0};
		g_mapReset[i] = false;
	} else if (g_pendingRot[i] != 0) {
		m.off = normOffset(m.off - g_pendingRot[i], m.h);
	}
	g_pendingRot[i] = 0;
	applyCanvasMap();
}

// A new ring region (or none). The draw buffer, ready and current, is re-laid
// in place; every other buffer is marked fully stale and adopts a mapping when
// it is next readied.
void changeRegion(int y, int h)
{
	if (g_regionY == y && g_regionH == h) return;
	g_swapDiag.regions++;
	g_regionY = h > 0 ? y : 0;
	g_regionH = h > 0 ? h : 0;
	const std::int64_t c0 = esp_timer_get_time();
	remapBuffer(g_drawIndex, ScanMap{g_regionY, g_regionH, 0});
	g_swapDiag.copyUs += esp_timer_get_time() - c0;
	for (int i = 0; i < g_scanFbCount; ++i) {
		if (i == g_drawIndex) continue;
		g_mapReset[i] = true;
		g_pendingRot[i] = 0;
		g_stale[i].count = 0;
		staleAdd(g_stale[i], 0, 0, kWidth - 1, kHeight - 1);
	}
	applyCanvasMap();
}

// Shift a lagging buffer's stale rects by a scroll of the region: the part
// inside the region moves with the content; rows pushed out of the region wrap
// to the exposed side, which the scroll frame repaints and marks stale itself.
void shiftStale(StaleList &list, int ry, int rh, int dy)
{
	static StaleRect old[kMaxStaleRects];  // render task only, under g_readyMutex
	const int n = list.count;
	for (int i = 0; i < n; ++i) old[i] = list.rects[i];
	list.count = 0;
	const int ry1 = ry + rh - 1;
	for (int i = 0; i < n; ++i) {
		const StaleRect r = old[i];
		if (r.y0 < ry) staleAdd(list, r.x0, r.y0, r.x1, std::min(r.y1, ry - 1));
		if (r.y1 > ry1) staleAdd(list, r.x0, std::max(r.y0, ry1 + 1), r.x1, r.y1);
		const int y0 = std::max(r.y0, ry) + dy, y1 = std::min(r.y1, ry1) + dy;
		if (std::max(r.y0, ry) <= std::min(r.y1, ry1)) {
			const int cy0 = std::max(y0, ry), cy1 = std::min(y1, ry1);
			if (cy0 <= cy1) staleAdd(list, r.x0, cy0, r.x1, cy1);
		}
	}
}

// Make the back buffer safe to draw into: released by the panel and current.
__attribute__((noinline)) void ensureBackReady(const DisplayFlushRect *skip, int skipCount)
{
	if (!g_swapFbs || g_backReady) return;
	xSemaphoreTake(g_readyMutex, portMAX_DELAY);
	if (!g_backReady) {
		// Pixels already drawn since the swap went in ahead of the gate: into a
		// buffer the panel may still have been scanning, and the patch below may
		// now overwrite them.
		int dx0, dy0, dx1, dy1;
		if (g_canvas.dirty(&dx0, &dy0, &dx1, &dy1) || g_workerCanvas.dirty(&dx0, &dy0, &dx1, &dy1)) g_swapDiag.late++;
		waitBackBufferFree();
		bringMapCurrent(g_drawIndex);
		const std::int64_t c0 = esp_timer_get_time();
		g_swapDiag.patchPx += patchBack(skip, skipCount);
		g_swapDiag.copyUs += esp_timer_get_time() - c0;
		g_backReady = true;
	}
	xSemaphoreGive(g_readyMutex);
}

bool g_vsyncWanted = true;
int publishSwap();
// Switch between swapped buffers and direct draw (see Display::setVSync) when
// nothing is in flight; a ring-scroll region or a scroll copy defers it to the
// next call.
void applyVSync()
{
	if (!g_framebuffer || g_scanFbCount < 2 || g_vsyncWanted == g_swapFbs) return;
	if (g_regionH > 0 || g_scrollCopyInFlight) return;
	xSemaphoreTake(g_readyMutex, portMAX_DELAY);
	if (!g_vsyncWanted) {
		// The last frame drawn becomes the front, then is drawn into in place
		// once the panel scans it.
		publishSwap();
		waitBackBufferFree();
		g_drawIndex = g_frontIndex;
		g_swapFbs = false;
		g_ringScroll = false;
	} else {
		g_drawIndex = (g_frontIndex + 1) % g_scanFbCount;
		g_stale[g_drawIndex].count = 0;
		staleAdd(g_stale[g_drawIndex], 0, 0, kWidth - 1, kHeight - 1);
		g_swapFbs = true;
		g_ringScroll = GEA_EMBEDDED_DISPLAY_RGB_BOUNCE;
	}
	g_drawFramebuffer = g_scanFb[g_drawIndex];
	g_backReady = !g_swapFbs;
	g_canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	g_workerCanvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	applyCanvasMap();
	xSemaphoreGive(g_readyMutex);
}

// Swap in everything drawn since the last swap. No pixels move here.
int publishSwap()
{
	if (!g_publishPending) return 0;
	g_publishPending = false;
	// Something drew into the back buffer without going through replayCanvas /
	// ensureBackReady: it may have written rows the panel was still scanning.
	if (!g_backReady) g_swapDiag.unready++;
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	// The frame's direct draws left dirty lines in the cache; written back now,
	// between refills, they are not evicted inside one (see scanoutRowCopy).
	// Every other buffer's stale list holds what this frame drew; one of them
	// is enough.
	if (g_scanFbCount > 1 && !g_frameWrittenBack) {
		const StaleList &drawn = g_stale[(g_drawIndex + 1) % g_scanFbCount];
		const ScanMap &m = g_map[g_drawIndex];
		const bool otherCore = static_cast<int>(xPortGetCoreID()) != g_panelIsrCore;
		for (int k = 0; k < drawn.count; ++k) {
			const StaleRect &r = drawn.rects[k];
			// Inside a scroll copy's viewport every row already left the cache:
			// the copy wrote each back, and so did every staged repaint in it.
			// The per-row walk (and refill wait) cost ~1 ms of a chart scroll frame.
			if (r.x0 >= g_cleanRect.x0 && r.y0 >= g_cleanRect.y0 && r.x1 <= g_cleanRect.x1 && r.y1 <= g_cleanRect.y1) continue;
			const std::size_t bytes = static_cast<std::size_t>(r.x1 - r.x0 + 1) * sizeof(std::uint16_t);
			for (int y = r.y0; y <= r.y1; ++y) {
				if (otherCore) waitOutRefill();
				writebackRow(g_drawFramebuffer + static_cast<std::size_t>(mapRow(m, y)) * kWidth + r.x0, bytes);
			}
		}
	}
#endif
	g_frameWrittenBack = false;
	g_cleanRect = {0, 0, -1, -1};
	// Any other kind of frame may have changed what any buffer's viewport holds.
	if (!g_frameIsScrollCopy)
		for (bool &v : g_rowSigValid) v = false;
	g_frameIsScrollCopy = false;
	g_nextSig = nullptr;  // signatures name this frame's list only
	commitSwap(g_drawIndex);
	return 1;
}

// The frame task never blocks on an RGB panel (no transfer to wait for), so on a
// long animation IDLE never runs on its core and the task watchdog fires every
// 5s -- and its ISR prints a backtrace over the 115200-baud console with that
// core's interrupts held off long enough to starve the bounce-buffer ISR, a
// visible glitch. Give IDLE one tick if nothing has blocked for a while.
void yieldToIdleIfStarved()
{
	const std::int64_t now = esp_timer_get_time();
	if (now - g_lastBlockUs < 500000) return;
	g_lastBlockUs = now;
	vTaskDelay(1);
}

#if GEA_BOARD_HAS_EXPANDER
constexpr int kExpanderBacklightPin = gea::platform::board::expander.backlight;
constexpr int kExpanderDisplayResetPin = gea::platform::board::expander.displayReset;
#else
constexpr int kExpanderBacklightPin = -1;
constexpr int kExpanderDisplayResetPin = -1;
#endif
constexpr gpio_num_t kBacklightPin = gea::platform::board::display.backlight;

esp_err_t updateBacklightDuty()
{
	if (!g_backlightPwmInitialized) return ESP_OK;
	const int brightness = gea::platform::board::display.backlightActiveLow ? 100 - g_brightness : g_brightness;
	const std::uint32_t duty = (kBacklightLedcMaxDuty * static_cast<std::uint32_t>(brightness)) / 100u;
	esp_err_t err = ledc_set_duty(kBacklightLedcMode, kBacklightLedcChannel, duty);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to set backlight duty: %s", esp_err_to_name(err));
		return err;
	}
	err = ledc_update_duty(kBacklightLedcMode, kBacklightLedcChannel);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to update backlight duty: %s", esp_err_to_name(err));
		return err;
	}
	return ESP_OK;
}

// An expander line is on/off (any non-zero brightness lights the panel) unless
// the expander carries a level on it, as a board-control MCU does.
esp_err_t applyBacklight()
{
#if GEA_BOARD_HAS_EXPANDER
	if constexpr (kExpanderBacklightPin >= 0) {
		auto &expander = gea::platform::esp32::chip_bindings::expanders::ioExpander();
		if (expander.setLevel(kExpanderBacklightPin, g_brightness)) return ESP_OK;
		return expander.writePin(kExpanderBacklightPin, g_brightness > 0) ? ESP_OK : ESP_FAIL;
	}
#endif
	return updateBacklightDuty();
}

esp_err_t initBacklight()
{
	if constexpr (kBacklightPin < 0) {
#if GEA_BOARD_HAS_EXPANDER
		if constexpr (kExpanderBacklightPin >= 0) {
			auto &expander = gea::platform::esp32::chip_bindings::expanders::ioExpander();
			if (!expander.init()) {
				ESP_LOGE(kTag, "I/O expander for the backlight did not come up");
				return ESP_FAIL;
			}
			// Dark until the framebuffer holds a cleared frame; start() lights it.
			return expander.writePin(kExpanderBacklightPin, false) ? ESP_OK : ESP_FAIL;
		}
#endif
		ESP_LOGW(kTag, "no backlight line declared; the panel is expected to light itself");
		return ESP_OK;
	} else {
		ledc_timer_config_t timerConfig = {};
		timerConfig.speed_mode = kBacklightLedcMode;
		timerConfig.timer_num = kBacklightLedcTimer;
		timerConfig.duty_resolution = kBacklightLedcDutyResolution;
		timerConfig.freq_hz = 5000;
		timerConfig.clk_cfg = LEDC_AUTO_CLK;
		esp_err_t err = ledc_timer_config(&timerConfig);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "failed to configure backlight LEDC timer: %s", esp_err_to_name(err));
			return err;
		}

		ledc_channel_config_t channelConfig = {};
		channelConfig.gpio_num = kBacklightPin;
		channelConfig.speed_mode = kBacklightLedcMode;
		channelConfig.channel = kBacklightLedcChannel;
		channelConfig.timer_sel = kBacklightLedcTimer;
		channelConfig.duty = gea::platform::board::display.backlightActiveLow ? kBacklightLedcMaxDuty : 0;
		channelConfig.hpoint = 0;
		err = ledc_channel_config(&channelConfig);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "failed to configure backlight LEDC channel: %s", esp_err_to_name(err));
			return err;
		}

		g_backlightPwmInitialized = true;
		return ESP_OK;
	}
}

// A panel whose reset sits on the expander gets a pulse before the LCD
// peripheral starts scanning; one with no reset line relies on power-on.
esp_err_t resetPanel()
{
#if GEA_BOARD_HAS_EXPANDER
	if constexpr (kExpanderDisplayResetPin >= 0) {
		auto &expander = gea::platform::esp32::chip_bindings::expanders::ioExpander();
		if (!expander.writePin(kExpanderDisplayResetPin, false)) return ESP_FAIL;
		vTaskDelay(pdMS_TO_TICKS(10));
		if (!expander.writePin(kExpanderDisplayResetPin, true)) return ESP_FAIL;
		vTaskDelay(pdMS_TO_TICKS(120));
	}
#endif
	return ESP_OK;
}

// Optional controller commands over 9-bit, mode-0 SPI on the expander.
// The pixel stream still uses the shared RGB DMA/rendering path.
esp_err_t initRgbController()
{
#if GEA_BOARD_HAS_RGB_CONTROLLER
  auto &io = gea::platform::esp32::chip_bindings::expanders::ioExpander();
  const auto &control = gea::platform::board::rgbController;
  if (!io.init()) return ESP_FAIL;
  if (control.addressSelect >= 0) {
    if (!io.writePin(control.addressSelect, false) || !io.setInput(control.addressSelect, false)) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  if (control.reset >= 0) {
    if (!io.writePin(control.reset, false) || !io.setInput(control.reset, false)) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!io.writePin(control.reset, true)) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  if (control.addressSelect >= 0 && !io.setInput(control.addressSelect, true)) return ESP_FAIL;
  if (!io.writePin(control.cs, true) || !io.writePin(control.clock, false)) return ESP_FAIL;
  for (const auto &command : gea::platform::board::rgbControllerCommands) {
    if (!io.writePin(control.cs, false)) return ESP_FAIL;
    for (int index = -1; index < command.length; ++index) {
      const unsigned word = index < 0 ? command.command : 0x100u | command.data[index];
      for (int bit = 8; bit >= 0; --bit) {
        if (!io.writePin(control.clock, false) || !io.writePin(control.data, (word & (1u << bit)) != 0) ||
            !io.writePin(control.clock, true)) return ESP_FAIL;
      }
    }
    if (!io.writePin(control.clock, false) || !io.writePin(control.cs, true)) return ESP_FAIL;
    if (command.delayMs) vTaskDelay(pdMS_TO_TICKS(command.delayMs));
  }
  ESP_LOGI(kTag, "RGB controller initialized through I/O expander");
#endif
  return ESP_OK;
}

struct PanelCreate {
	esp_lcd_rgb_panel_config_t *config;
	esp_err_t err;
	int core;  // where the driver installed its interrupts (the calling core)
	SemaphoreHandle_t done;
};

void createPanel(PanelCreate &c)
{
	c.core = static_cast<int>(xPortGetCoreID());
	c.err = esp_lcd_new_rgb_panel(c.config, &g_panel);
	// Bounce buffers live in internal DMA RAM, which other services share: when
	// the preferred size does not fit, step down to the next whole-row divisor
	// rather than leaving the panel dark.
	while (c.err == ESP_ERR_NO_MEM && c.config->bounce_buffer_size_px > static_cast<std::size_t>(kWidth)) {
		int rows = static_cast<int>(c.config->bounce_buffer_size_px / kWidth) - 1;
		while (rows > 1 && kHeight % rows != 0) --rows;
		ESP_LOGW(kTag, "no room for %d-row bounce buffers; trying %d", int(c.config->bounce_buffer_size_px / kWidth), rows);
		c.config->bounce_buffer_size_px = static_cast<std::size_t>(kWidth) * rows;
		c.err = esp_lcd_new_rgb_panel(c.config, &g_panel);
	}
	while (c.err != ESP_OK && c.config->num_fbs > 1) {
		ESP_LOGW(kTag, "no room for %d scanout buffers (%s); trying %d", int(c.config->num_fbs),
		         esp_err_to_name(c.err), int(c.config->num_fbs) - 1);
		c.config->num_fbs -= 1;
		c.err = esp_lcd_new_rgb_panel(c.config, &g_panel);
	}
}

esp_err_t initRgbPanel()
{
	const auto &display = gea::platform::board::display;
	esp_lcd_rgb_panel_config_t config = {};
	config.clk_src = LCD_CLK_SRC_PLL160M;
	config.timings.pclk_hz = kPixelClockHz;
	config.timings.h_res = kWidth;
	config.timings.v_res = kHeight;
	config.timings.hsync_pulse_width = display.hsyncPulseWidth;
	config.timings.hsync_back_porch = display.hsyncBackPorch;
	config.timings.hsync_front_porch = display.hsyncFrontPorch;
	config.timings.vsync_pulse_width = display.vsyncPulseWidth;
	config.timings.vsync_back_porch = display.vsyncBackPorch;
	config.timings.vsync_front_porch = display.vsyncFrontPorch;
	config.timings.flags.pclk_active_neg = display.pclkActiveNeg ? 1 : 0;
	config.data_width = 16;
	config.in_color_format = LCD_COLOR_FMT_RGB565;
	config.out_color_format = LCD_COLOR_FMT_RGB565;
#ifdef GEA_EMBEDDED_DISPLAY_RGB_SCAN_FBS
	config.num_fbs = GEA_EMBEDDED_DISPLAY_RGB_SWAP_FBS ? GEA_EMBEDDED_DISPLAY_RGB_SCAN_FBS : 1;
#else
	config.num_fbs = GEA_EMBEDDED_DISPLAY_RGB_SWAP_FBS ? gea::platform::board::display.scanFramebuffers : 1;
#endif
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	config.bounce_buffer_size_px = static_cast<std::size_t>(kWidth) * bounceBufferRows();
	// This file fills the bounce buffers itself (onBounceEmpty) so it can apply
	// the ring-scroll mapping, which the driver only allows when it owns no
	// framebuffer: the scanout buffers are allocated below instead.
	const int wantFbs = static_cast<int>(config.num_fbs > 0 ? config.num_fbs : 1);
	config.num_fbs = 0;
	config.flags.no_fb = 1;
	int ownFbs = 0;
	for (; ownFbs < wantFbs; ++ownFbs) {
		g_scanFb[ownFbs] = static_cast<std::uint16_t *>(heap_caps_aligned_alloc(64, kFramebufferBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!g_scanFb[ownFbs]) break;
		std::memset(g_scanFb[ownFbs], 0, kFramebufferBytes);
	}
	if (ownFbs == 0) {
		ESP_LOGE(kTag, "no PSRAM for a scanout buffer");
		return ESP_ERR_NO_MEM;
	}
	if (ownFbs < wantFbs) ESP_LOGW(kTag, "no room for %d scanout buffers; using %d", wantFbs, ownFbs);
#else
	config.bounce_buffer_size_px = 0;
#endif
	config.dma_burst_size = 64;
	config.hsync_gpio_num = display.hsync;
	config.vsync_gpio_num = display.vsync;
	config.de_gpio_num = display.de;
	config.pclk_gpio_num = display.pclk;
	config.disp_gpio_num = display.disp;
	for (int i = 0; i < 16; ++i) config.data_gpio_nums[i] = display.data[i];
	config.flags.fb_in_psram = 1;

	// The driver allocates its GDMA EOF and LCD interrupts on the calling core;
	// commitSwap() needs to know which one that is.
	PanelCreate create = {&config, ESP_FAIL, -1, nullptr};
#if GEA_EMBEDDED_DISPLAY_RGB_ISR_CORE >= 0
	// Install the panel's interrupts (the bounce-buffer copy ISR) on a chosen
	// core, off the runtime's, by creating the panel from a task pinned there.
	create.done = xSemaphoreCreateBinary();
	if (create.done &&
	    xTaskCreatePinnedToCore([](void *arg) {
		    auto *c = static_cast<PanelCreate *>(arg);
		    createPanel(*c);
		    xSemaphoreGive(c->done);
		    vTaskDelete(nullptr);
	    }, "rgb_init", 4096, &create, configMAX_PRIORITIES - 2, nullptr, GEA_EMBEDDED_DISPLAY_RGB_ISR_CORE) == pdPASS) {
		xSemaphoreTake(create.done, portMAX_DELAY);
	} else {
		createPanel(create);
	}
	if (create.done) vSemaphoreDelete(create.done);
#else
	createPanel(create);
#endif
	g_panelIsrCore = create.core;
	esp_err_t err = create.err;
	ESP_LOGI(kTag, "panel interrupts on core %d", g_panelIsrCore);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to create RGB panel: %s", esp_err_to_name(err));
		return err;
	}
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	g_scanFbCount = ownFbs;
	{
		ESP_LOGI(kTag, "bounce buffers: %d rows", static_cast<int>(config.bounce_buffer_size_px / kWidth));
	}
#else
	g_scanFbCount = static_cast<int>(config.num_fbs);
#endif
	g_swapFbs = g_scanFbCount > 1;
	if (g_swapFbs || GEA_EMBEDDED_DISPLAY_RGB_BOUNCE) {
		if (g_swapFbs) {
			g_frameSem = xSemaphoreCreateBinary();
			g_readyMutex = xSemaphoreCreateMutex();
			if (!g_frameSem || !g_readyMutex) return ESP_ERR_NO_MEM;
		}
		esp_lcd_rgb_panel_event_callbacks_t callbacks = {};
		callbacks.on_frame_buf_complete = onPanelFrameComplete;
		callbacks.on_vsync = onPanelVsync;
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
		callbacks.on_bounce_empty = onBounceEmpty;
#endif
		err = esp_lcd_rgb_panel_register_event_callbacks(g_panel, &callbacks, nullptr);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "failed to register RGB frame callback: %s", esp_err_to_name(err));
			return err;
		}
	}
	err = esp_lcd_panel_reset(g_panel);
	if (err != ESP_OK) return err;
	err = esp_lcd_panel_init(g_panel);
	if (err != ESP_OK) return err;

#if !GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	void *fb0 = nullptr;
	void *fb1 = nullptr;
	void *fb2 = nullptr;
	err = g_scanFbCount >= 3 ? esp_lcd_rgb_panel_get_frame_buffer(g_panel, 3, &fb0, &fb1, &fb2)
	    : g_scanFbCount == 2 ? esp_lcd_rgb_panel_get_frame_buffer(g_panel, 2, &fb0, &fb1)
	                         : esp_lcd_rgb_panel_get_frame_buffer(g_panel, 1, &fb0);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to get RGB framebuffer: %s", esp_err_to_name(err));
		return err;
	}
	g_scanFb[0] = static_cast<std::uint16_t *>(fb0);
	g_scanFb[1] = static_cast<std::uint16_t *>(fb1);
	g_scanFb[2] = static_cast<std::uint16_t *>(fb2);
#endif
	g_ringScroll = GEA_EMBEDDED_DISPLAY_RGB_BOUNCE && g_swapFbs;
	g_frontIndex = 0;
	g_drawIndex = g_scanFbCount > 1 ? 1 : 0;
	g_framebuffer = g_scanFb[0];
	if (!g_framebuffer || (g_swapFbs && !g_scanFb[1])) return ESP_ERR_NO_MEM;

	// Double buffer: the panel framebuffer (g_framebuffer) is continuously scanned out,
	// so drawing directly into it exposes every intermediate step (e.g. a tick's
	// background clears, THEN the tick paints — a visible flash). Render into a separate
	// PSRAM back buffer instead; publishDrawRect() copies the fully-composited dirty
	// region into the scanout framebuffer in one shot, so the panel only ever sees
	// finished pixels. Falls back to single-buffered direct draw if PSRAM is exhausted.
	// With swapped scanout buffers the renderer draws straight into the back one.
	g_drawFramebuffer = g_swapFbs ? g_scanFb[1]
	                              : static_cast<std::uint16_t *>(heap_caps_aligned_alloc(
	                                    64, kFramebufferBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!g_drawFramebuffer) {
		ESP_LOGW(kTag, "no PSRAM for back buffer; falling back to single-buffered draw");
		g_drawFramebuffer = g_framebuffer;
		g_swapFbs = false;  // drawing straight into the front buffer: nothing to swap
	}

	std::memset(g_framebuffer, 0, kFramebufferBytes);
	if (g_drawFramebuffer != g_framebuffer) std::memset(g_drawFramebuffer, 0, kFramebufferBytes);
	cacheWritebackAll();
	for (int i = 1; i < g_scanFbCount; ++i) {
		std::memset(g_scanFb[i], 0, kFramebufferBytes);
		cacheWritebackRect(g_scanFb[i], 0, 0, kWidth - 1, kHeight - 1);
	}
	g_canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	g_workerCanvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);

#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
	// Internal SRAM (not PSRAM): the whole point is to raster off the scanout bus.
	// 16-byte aligned: rows are 960B (16-multiple), so an aligned base keeps every
	// staged-replay row 16-aligned — maximal PIE-SIMD span coverage per scanline.
	allocStripBuffer(0);
	ESP_LOGI(kTag, "strip-raster %s strip_buf=%p rows=%d halves=%d (internal free=%u largest=%u)",
	         g_stripBuffer ? "ON" : "OFF (no internal SRAM)", g_stripBuffer, g_stripRows, g_stripHalves,
	         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
	         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
#endif
	ESP_LOGI(kTag, "RGB panel ready %dx%d pclk=%d bounce_rows=%d fb=%p fb2=%p swap=%d isr_core=%d draw_fb=%p single_fb=%d row_bytes=%u fb_alignment64=%u vsync_restart=%d",
	         kWidth,
	         kHeight,
	         kPixelClockHz,
	         int(config.bounce_buffer_size_px / kWidth),
	         g_framebuffer,
	         g_scanFb[1],
	         g_swapFbs ? 1 : 0,
	         g_panelIsrCore,
	         g_drawFramebuffer,
	         g_framebuffer == g_drawFramebuffer ? 1 : 0,
	         static_cast<unsigned>(kWidth * sizeof(std::uint16_t)),
	         static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(g_framebuffer) % 64),
#if CONFIG_LCD_RGB_RESTART_IN_VSYNC
	         1);
#else
	         0);
#endif
	applyVSync();  // the app may have asked before the panel was up
	return ESP_OK;
}

void replayPresentCommand(const gea::platform::display::DisplayPresentCommand &command)
{
	using Type = gea::platform::display::DisplayPresentCommandType;
	switch (command.type) {
		case Type::Clear:
			g_canvas.clear(command.clear.color);
			break;
		case Type::FillRectRgb565:
			g_canvas.setGlobalAlpha(command.fillRectRgb565.alpha);
			g_canvas.fillRect(command.fillRectRgb565.x,
			                  command.fillRectRgb565.y,
			                  command.fillRectRgb565.w,
			                  command.fillRectRgb565.h,
			                  command.fillRectRgb565.color);
			break;
		case Type::StrokeRectRgb565:
			g_canvas.setGlobalAlpha(command.strokeRectRgb565.alpha);
			g_canvas.strokeRect(command.strokeRectRgb565.x,
			                    command.strokeRectRgb565.y,
			                    command.strokeRectRgb565.w,
			                    command.strokeRectRgb565.h,
			                    command.strokeRectRgb565.color);
			break;
		case Type::FillTriangleRgb565:
			g_canvas.setGlobalAlpha(command.fillTriangleRgb565.alpha);
			g_canvas.fillTriangle(command.fillTriangleRgb565.x0,
			                      command.fillTriangleRgb565.y0,
			                      command.fillTriangleRgb565.x1,
			                      command.fillTriangleRgb565.y1,
			                      command.fillTriangleRgb565.x2,
			                      command.fillTriangleRgb565.y2,
			                      command.fillTriangleRgb565.color);
			break;
		case Type::FillTrianglesRgb565: {
			// Depth-ordered batch (farthest-first). Opaque batches take the span-occlusion
			// path so each pixel is written once; translucent ones must stay painter's order.
			const auto &batch = command.fillTrianglesRgb565;
			if (!batch.entries) break;
			g_canvas.setGlobalAlpha(batch.alpha);
			if (batch.alpha == 255) {
				g_canvas.fillTrianglesOpaqueOccluded(batch.entries, batch.count, 0, 0);
			} else {
				for (int i = 0; i < batch.count; i++) {
					const auto &t = batch.entries[i];
					g_canvas.fillTriangle(t.x0, t.y0, t.x1, t.y1, t.x2, t.y2, t.color);
				}
			}
			break;
		}
		case Type::FillCircleRgb565:
			g_canvas.setGlobalAlpha(command.fillCircleRgb565.alpha);
			g_canvas.fillCircle(command.fillCircleRgb565.x,
			                    command.fillCircleRgb565.y,
			                    command.fillCircleRgb565.radius,
			                    command.fillCircleRgb565.color);
			break;
		case Type::StrokeCircleRgb565:
			g_canvas.setGlobalAlpha(command.strokeCircleRgb565.alpha);
			g_canvas.strokeCircle(command.strokeCircleRgb565.x,
			                      command.strokeCircleRgb565.y,
			                      command.strokeCircleRgb565.radius,
			                      command.strokeCircleRgb565.color);
			break;
			case Type::FillCirclesRgb565:
				g_canvas.setGlobalAlpha(command.fillCirclesRgb565.alpha);
				g_canvas.fillCirclesRgb565(command.fillCirclesRgb565.xs,
				                           command.fillCirclesRgb565.ys,
				                           command.fillCirclesRgb565.count,
				                           command.fillCirclesRgb565.radius,
				                           command.fillCirclesRgb565.colors);
				break;
			case Type::DrawImage:
				g_canvas.setGlobalAlpha(command.drawImage.alpha);
			g_canvas.drawImage(command.drawImage.pixels,
			                   command.drawImage.alphaPixels,
			                   command.drawImage.srcWidth,
			                   command.drawImage.srcHeight,
			                   command.drawImage.x,
			                   command.drawImage.y);
			break;
		case Type::DrawImageScaled:
			g_canvas.setGlobalAlpha(command.drawImageScaled.alpha);
			g_canvas.drawImage(command.drawImageScaled.pixels,
			                   command.drawImageScaled.alphaPixels,
			                   command.drawImageScaled.srcWidth,
			                   command.drawImageScaled.srcHeight,
			                   command.drawImageScaled.x,
			                   command.drawImageScaled.y,
			                   command.drawImageScaled.w,
			                   command.drawImageScaled.h);
			break;
		case Type::DrawImageRotated90CW:
			g_canvas.setGlobalAlpha(command.drawImageRotated90CW.alpha);
			g_canvas.drawImageRotated90CW(command.drawImageRotated90CW.pixels,
			                              command.drawImageRotated90CW.alphaPixels,
			                              command.drawImageRotated90CW.srcWidth,
			                              command.drawImageRotated90CW.srcHeight,
			                              command.drawImageRotated90CW.x,
			                              command.drawImageRotated90CW.y,
			                              command.drawImageRotated90CW.w,
			                              command.drawImageRotated90CW.h);
			break;
		case Type::DrawImageTiledX:
			g_canvas.setGlobalAlpha(command.drawImageTiledX.alpha);
			g_canvas.drawImageTiledX(command.drawImageTiledX.pixels,
			                         command.drawImageTiledX.alphaPixels,
			                         command.drawImageTiledX.srcWidth,
			                         command.drawImageTiledX.srcHeight,
			                         command.drawImageTiledX.x,
			                         command.drawImageTiledX.y,
			                         command.drawImageTiledX.w);
			break;
		case Type::FillText:
			g_canvas.setGlobalAlpha(command.fillText.alpha);
			// A baked family draws in that font, as display_present.h's replay
			// does; only a family-less command takes the built-in bitmap font.
			if (command.fillText.fontFamilyId >= 0)
				g_canvas.drawTextFontFamily(command.fillText.text,
				                            command.fillText.x,
				                            command.fillText.y,
				                            command.fillText.color,
				                            command.fillText.fontFamilyId,
				                            command.fillText.fontSizePx);
			else
				g_canvas.drawText(command.fillText.text,
				                  command.fillText.x,
				                  command.fillText.y,
				                  command.fillText.color,
				                  command.fillText.scale);
			break;
	}
}

bool clipRect(int *x0, int *y0, int *x1, int *y1)
{
	if (!x0 || !y0 || !x1 || !y1) return false;
	if (*x0 < 0) *x0 = 0;
	if (*y0 < 0) *y0 = 0;
	if (*x1 >= kWidth) *x1 = kWidth - 1;
	if (*y1 >= kHeight) *y1 = kHeight - 1;
	return *x0 <= *x1 && *y0 <= *y1;
}

	int publishDrawRect(int x0, int y0, int x1, int y1, int *outX0 = nullptr, int *outY0 = nullptr, int *outX1 = nullptr, int *outY1 = nullptr)
	{
		if (!g_framebuffer || !g_drawFramebuffer || !clipRect(&x0, &y0, &x1, &y1)) return 0;
		if (outX0) *outX0 = x0;
		if (outY0) *outY0 = y0;
		if (outX1) *outX1 = x1;
		if (outY1) *outY1 = y1;
		const int width = x1 - x0 + 1;
		if (g_framebuffer == g_drawFramebuffer) return width * (y1 - y0 + 1);
		if (g_swapFbs) {
			// Deferred: recorded against both scanout buffers, copied by publishSwap().
			markStale(x0, y0, x1, y1);
			return width * (y1 - y0 + 1);
		}
		const std::int64_t c0 = esp_timer_get_time();
		for (int y = y0; y <= y1; ++y) {
		const std::size_t offset = static_cast<std::size_t>(y) * kWidth + static_cast<std::size_t>(x0);
		std::memcpy(&g_framebuffer[offset], &g_drawFramebuffer[offset], static_cast<std::size_t>(width) * sizeof(std::uint16_t));
		forgetRowSumAt(&g_framebuffer[offset]);
	}
	g_swapDiag.copyUs += esp_timer_get_time() - c0;
	return width * (y1 - y0 + 1);
}

struct RenderRowWorker {
	TaskHandle_t task = nullptr;
	void (*volatile fn)(void *, int, int) = nullptr;
	void *volatile ctx = nullptr;
	volatile int y0 = 0;
	volatile int y1 = 0;
	std::atomic<std::uint32_t> jobSeq{0};
	std::atomic<bool> done{true};
	// DIAG: cumulative on-worker band execution time (read by the wait-side print;
	// only ever written between done=false and done=true, so the read is stable).
	std::int64_t bandUs = 0;
	bool attempted = false;
};

RenderRowWorker g_renderRowWorker;
std::atomic<bool> g_renderWorkerBlockIdle{false};
// 12KB (was 8KB for the memcpy-only strip copies): the worker now also runs the
// renderer's full band replay (gradient projection math + rounded-rect batch +
// command handlers), which is much deeper than a strip copy.
constexpr std::uint32_t kRenderRowWorkerStackBytes = 12288;

#ifndef GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL
#define GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL 0
#endif

void renderRowWorkerTask(void *)
{
	// MUST start at 0 (jobSeq's initial value), NOT jobSeq.load(): submit() creates
	// this task and increments jobSeq immediately after, so by the time this task
	// gets its first instructions in, jobSeq is usually already 1 — loading it here
	// would swallow job 1 (the caller times out and the first parallel band is
	// silently dropped; for the in-place recolor that band then stays stale forever,
	// since later recolors only match the CURRENT oldColor).
	std::uint32_t seen = 0;
	std::int64_t idleSince = esp_timer_get_time();
	for (;;) {
		while (g_renderRowWorker.jobSeq.load(std::memory_order_acquire) == seen) {
			// Every submit notifies, so blocking at once loses no job; spinning
			// first only shaves the wake-up, and starves a lower-priority task
			// that shares this core (gea_render_worker_block_when_idle).
			if (g_renderWorkerBlockIdle.load(std::memory_order_relaxed) || esp_timer_get_time() - idleSince > 4000) {
				ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
				idleSince = esp_timer_get_time();
			}
		}
		seen = g_renderRowWorker.jobSeq.load(std::memory_order_acquire);
		void (*fn)(void *, int, int) = g_renderRowWorker.fn;
		const std::int64_t b0 = esp_timer_get_time();
		if (fn) fn(g_renderRowWorker.ctx, g_renderRowWorker.y0, g_renderRowWorker.y1);
		g_renderRowWorker.bandUs += esp_timer_get_time() - b0;
		g_renderRowWorker.done.store(true, std::memory_order_release);
		idleSince = esp_timer_get_time();
	}
}

#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
// Runs on the worker core: copy a finished SRAM strip (ctx) to PSRAM scanout rows
// [y0,y1] and write it back so the panel's scanout DMA sees it.
// Where presentStripFused publishes: the live buffer on a single-buffer panel,
// the off-screen back buffer when scanout buffers swap.
std::uint16_t *g_stripTarget = nullptr;

void copyStripToScanoutThunk(void *ctx, int y0, int y1)
{
	const std::uint16_t *src = static_cast<const std::uint16_t *>(ctx);
	// Row by row through the scanout guard: one memcpy of the whole strip left
	// up to ~38 KB of dirty lines in the 64 KB cache for the next refill to
	// evict inside its window (the overruns on every full-screen redraw, such
	// as a tab switch).
	for (int y = y0; y <= y1; ++y)
		scanoutRowCopy(&g_stripTarget[static_cast<std::size_t>(y) * kWidth],
		               &src[static_cast<std::size_t>(y - y0) * kWidth], kWidth * sizeof(std::uint16_t));
	cacheWritebackRect(g_stripTarget, 0, y0, kWidth - 1, y1);
}

// Compose the whole frame strip-by-strip in internal SRAM, copying each finished
// strip to the live scanout buffer. Used only for full-screen-redraw frames
// (frameHasOpaqueBase), so each strip is fully composited before it is published —
// tear-equivalent to the PSRAM double buffer, but the 1000-circle raster runs in
// SRAM instead of fighting the panel's continuous scanout for PSRAM bandwidth.
//
// Pipelined across cores: this core rasters strip N+1 into one SRAM half while the
// worker core copies strip N out of the other half to scanout. Raster (SRAM-only)
// and copy (PSRAM) hit different buses, so the ~6.5ms raster hides under the ~20ms
// copy instead of adding to it.
bool presentStripFused(const present::Frame &frame)
{
	if (!g_framebuffer || !g_stripBuffer || g_stripHalves < 2) return false;
	int stripRows = g_flushRows > 0 ? g_flushRows : g_stripRows;
	if (stripRows > g_stripRows) stripRows = g_stripRows;
	const std::int64_t start = esp_timer_get_time();
	int back = 0;
	if (g_swapFbs) {
		// The whole frame lands in the back buffer, so its pending regions are moot:
		// wait for the panel to release it, but patch nothing.
		back = g_drawIndex;
		if (!g_backReady) {
			waitBackBufferFree();
			g_stale[back].count = 0;
			g_backReady = true;
		}
		// Every row is written below, so the buffer can take the region's identity
		// layout (offset 0) for free.
		g_map[back] = ScanMap{g_regionY, g_regionH, 0};
		g_pendingRot[back] = 0;
		g_mapReset[back] = false;
		g_stripTarget = g_scanFb[back];
	} else {
		g_stripTarget = g_framebuffer;
	}
	bool inFlight = false;
	int idx = 0;
	for (int row = 0; row < kHeight; row += stripRows, ++idx) {
		int rows = stripRows;
		if (row + rows > kHeight) rows = kHeight - row;
		std::uint16_t *buf = g_stripBuffer + static_cast<std::size_t>(idx & 1) * g_stripRows * kWidth;
		// Raster strip `idx` into its SRAM half — overlaps the worker copying the
		// previous strip out of the OTHER half (different buffer + bus, no contention).
		present::rasterFrameRowsStrided(buf, kWidth, 0, kWidth, row, rows, kHeight, frame);
		// The single worker handles one copy at a time. Waiting here also guarantees
		// the half we'll reuse two strips from now is free before we raster into it.
		if (inFlight) { gea_render_parallel_wait(); inFlight = false; }
		if (gea_render_parallel_rows_submit(copyStripToScanoutThunk, buf, row, row + rows - 1)) {
			inFlight = true;
		} else {
			copyStripToScanoutThunk(buf, row, row + rows - 1);  // no worker: inline copy
		}
	}
	if (inFlight) gea_render_parallel_wait();
	if (g_swapFbs) {
		g_stale[back].count = 0;
		g_publishPending = false;
		commitSwap(back);
		// The new back buffer lags by a whole frame; a later partial frame patches
		// it in full once (a following full-frame present needs no patch at all).
		for (int i = 0; i < g_scanFbCount; ++i) {
			if (i == back) continue;
			g_stale[i].count = 0;
			staleAdd(g_stale[i], 0, 0, kWidth - 1, kHeight - 1);
			g_mapReset[i] = true;
			g_pendingRot[i] = 0;
		}
	} else {
		g_drawBufferStale = (g_drawFramebuffer != g_framebuffer);
	}
	addFlushStats(esp_timer_get_time() - start, kWidth * kHeight);
	noteFrameDiag(3, esp_timer_get_time() - start, kWidth * kHeight);
	return true;
}
#endif

	}  // namespace

namespace {

// DIAG: band-replay split engagement — submit count + the main core's join-wait
// time, printed every 300 draw-band jobs (mirrors the esp32 backend's [worker]
// line). Tells "split never engages" apart from "split engages but doesn't help".
int g_drawBandJobs = 0;
std::int64_t g_drawBandWaitUs = 0;
bool g_lastSubmitWasDrawBand = false;

// Shared single-job submit behind both worker entry points: the strip-copy path
// (gea_render_parallel_rows_submit) and the renderer's coarse band replay
// (gea_render_parallel_submit). One job in flight at a time — both producers run
// on the frame task and join via gea_render_parallel_wait before resubmitting.
bool submitRenderWorkerJob(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	if (!g_renderRowWorker.attempted) {
		g_renderRowWorker.attempted = true;
		const BaseType_t renderCore = xPortGetCoreID();
		const BaseType_t workerCore = renderCore == 0 ? 1 : 0;
		const UBaseType_t priority = uxTaskPriorityGet(nullptr);
#if GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL
		// This persistent task only replays render commands and copies pixels;
		// neither its stack nor its local variables are DMA buffers. Match the
		// SPI display worker's external-stack option and retain internal RAM for
		// audio DMA, lwIP and FreeRTOS control structures.
		const BaseType_t created = xTaskCreatePinnedToCoreWithCaps(renderRowWorkerTask,
		                                                   "gea_rowwrk",
		                                                   kRenderRowWorkerStackBytes,
		                                                   nullptr,
		                                                   priority,
		                                                   &g_renderRowWorker.task,
		                                                   workerCore,
		                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
		BaseType_t created = xTaskCreatePinnedToCore(renderRowWorkerTask,
		                                                   "gea_rowwrk",
		                                                   kRenderRowWorkerStackBytes,
		                                                   nullptr,
		                                                   priority,
		                                                   &g_renderRowWorker.task,
		                                                   workerCore);
#endif
#if !GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL
		if (created != pdPASS || !g_renderRowWorker.task) {
			// Internal RAM is tight on an RGB board (bounce buffers, strip staging);
			// the stack holds no DMA buffers, so PSRAM is a correct second choice.
			g_renderRowWorker.task = nullptr;
			created = xTaskCreatePinnedToCoreWithCaps(renderRowWorkerTask,
			                                          "gea_rowwrk",
			                                          kRenderRowWorkerStackBytes,
			                                          nullptr,
			                                          priority,
			                                          &g_renderRowWorker.task,
			                                          workerCore,
			                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		}
#endif
		if (created != pdPASS || !g_renderRowWorker.task) {
			ESP_LOGW(kTag, "row worker create failed: create=%d stack=%u",
			         static_cast<int>(created),
			         static_cast<unsigned>(kRenderRowWorkerStackBytes));
		} else {
			std::printf("[render-worker] task=%p renderCore=%d workerCore=%d prio=%u stack=%u\n",
			            static_cast<void *>(g_renderRowWorker.task), static_cast<int>(renderCore),
			            static_cast<int>(workerCore), static_cast<unsigned>(priority),
			            static_cast<unsigned>(kRenderRowWorkerStackBytes));
		}
	}
	if (!g_renderRowWorker.task) return false;
	g_renderRowWorker.fn = fn;
	g_renderRowWorker.ctx = ctx;
	g_renderRowWorker.y0 = y0;
	g_renderRowWorker.y1 = y1;
	g_renderRowWorker.done.store(false, std::memory_order_release);
	g_renderRowWorker.jobSeq.fetch_add(1, std::memory_order_release);
	xTaskNotifyGive(g_renderRowWorker.task);
	return true;
}

}  // namespace

// A task that shares the worker's core at a lower priority (gea3d's frame
// build) asks the worker to sleep between jobs instead of spinning.
extern "C" void gea_render_worker_block_when_idle(bool on)
{
	g_renderWorkerBlockIdle.store(on, std::memory_order_relaxed);
}

extern "C" bool gea_render_parallel_rows_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	if (g_scrollCopyInFlight) return false;  // the worker is copying a scroll
	g_lastSubmitWasDrawBand = false;
	return submitRenderWorkerJob(fn, ctx, y0, y1);
}

// NOTE: a 410-permille biased split (worker takes more rows; both bands finish
// together) was measured and REVERTED: with the cores fully overlapped the pair
// is bus-limited, so rebalancing rows doesn't shorten the wall — it only deepens
// the PSRAM saturation window, and the bounce-fill ISR (which keeps the panel
// synced) starts missing its deadline → dropped/restarted frames. Even split
// (the weak default 500) leaves the main core a solo tail that the ISR rides.

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	// Record the render core so Display draw calls on the OTHER core (the worker's
	// band replay) route to the band canvas. Re-set each submit in case the frame
	// task ever migrates cores.
	if (g_scrollCopyInFlight) return false;  // the worker is copying a scroll
	g_renderCoreId = static_cast<int>(xPortGetCoreID());
	ensureBackReady();  // the band draws into the back buffer
	const bool ok = submitRenderWorkerJob(fn, ctx, y0, y1);
	g_lastSubmitWasDrawBand = ok;
	return ok;
}

// After a coarse band-replay joins, fold the worker canvas's dirty bbox into the
// primary so the publish copy (which reads g_canvas.dirty()) transmits both bands.
extern "C" void gea_render_parallel_merge_dirty()
{
	int x0, y0, x1, y1;
	if (g_workerCanvas.dirty(&x0, &y0, &x1, &y1)) g_canvas.markDirty(x0, y0, x1, y1);
	g_workerCanvas.resetDirty();
}

namespace {

// SRAM-staged dirty-region replay (gea_replay_stage_* hooks): the region replay
// rasters into a per-core half of g_stripBuffer instead of read-modify-writing the
// PSRAM back buffer under the RGB panel's continuous scanout. Measured before this:
// a HALF-band replay cost the same ~20ms as the full region — the PSRAM bus, not
// the CPU, was the limit, so the 2-core split alone bought nothing. Per-core halves:
// with the band split each core stages its own (disjoint) band rows, so the halves
// never overlap; presentStripFused also uses both halves but only on canvas-present
// frames, never concurrently with a region replay.
struct ReplayStageState {
	bool active = false;
	int x0 = 0, y0 = 0, x1 = -1, y1 = -1;  // staged window (screen coords)
	bool hadDirty = false;                 // canvas dirty union accumulated before the
	int dx0 = 0, dy0 = 0, dx1 = -1, dy1 = -1;  // rebind (bindPixels resets it)
	std::uint8_t alpha = 255;
	bool unseeded = false;  // the window was not seeded: every pixel must be copied back
	int stride = 0;         // strip row stride in pixels: the window width, 16-byte rounded
	std::uint16_t *strip = nullptr;  // the window's buffer (a strip half or a pipeline slot)
	// DIAG: per-strip cost decomposition (seed copy-in / replay body / copy-back),
	// printed every kStageDiagStrips strips per core.
	std::int64_t beginDoneUs = 0;
	std::int64_t seedUs = 0, bodyUs = 0, cbUs = 0;
	int strips = 0, px = 0;
};
ReplayStageState g_replayStage[2];
constexpr int kStageDiagStrips = 600;
// DIAG: PIE SIMD blend engagement (calls + 8-px groups), printed with [stage].
volatile int g_pieCalls = 0;
volatile int g_pieCount8 = 0;

}  // namespace

// Staged-replay gate. Measured on css-3d-cube: staging the replay through SRAM
// strips LOST ~6fps — the per-strip re-replay multiplies commands whose raster
// cost is NOT row-proportional (the transformed face-label text re-rasters its
// full quad in every overlapping 24-row strip; rUs text went to ~10ms/frame),
// and the per-pixel blend turned out compute-bound, not PSRAM-bound, so the
// SRAM dst saved nothing. Keep the machinery (hooks + per-core windows) gated
// off for A/B; flip to 1 to re-measure.
#ifndef GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE
#define GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE 0
#endif

// Direct replay is about to repaint `rects` completely (see tree_render): ready
// the back buffer now, patching only the stale area outside them.
extern "C" void gea_display_prepare_repaint(const gea::platform::display::DisplayFlushRect *rects, int count)
{
	if (rects && count > 0) {
		g_swapDiag.prepares++;
		for (int i = 0; i < count; ++i) g_swapDiag.preparePx += (rects[i].x1 - rects[i].x0 + 1) * (rects[i].y1 - rects[i].y0 + 1);
		ensureBackReady(rects, count);
	}
}

// In-place frames: an app that repaints every pixel each frame can render
// straight into the swap buffers instead of into its own frame that present()
// then copies over (a whole frame of PSRAM reads and writes, ~40 ms at 800x480).
// Only with swapped buffers laid out as plain rows (no ring-scroll region): the
// app writes row-major pixels.
static bool panelBuffersUsable()
{
	return g_swapFbs && g_scanFbCount >= 2 && g_regionH == 0;
}

extern "C" int gea_display_panel_buffer_count(int *width, int *height)
{
	if (width) *width = kWidth;
	if (height) *height = kHeight;
	return panelBuffersUsable() ? g_scanFbCount : 0;
}

extern "C" std::uint16_t *gea_display_panel_buffer(int index)
{
	return index >= 0 && index < g_scanFbCount ? g_scanFb[index] : nullptr;
}

// The buffer the next frame draws into, once the panel has released it. The
// caller repaints all of it, so nothing is carried over from the front.
extern "C" int gea_display_acquire_panel_buffer()
{
	if (!panelBuffersUsable()) return -1;
	const DisplayFlushRect whole{0, 0, kWidth - 1, kHeight - 1};
	ensureBackReady(&whole, 1);
	g_drawBufferStale = false;
	return g_drawIndex;
}

// State-only view of the draw canvas (dirty box, scroll region): no pixels move
// through it, so it does not ready the back buffer -- the replay readies it once
// it can name the rects it repaints whole (gea_display_prepare_repaint).
extern "C" gea::framework::graphics::Canvas *gea_display_state_canvas()
{
	return &stateCanvas();
}

extern "C" int gea_replay_stage_rows()
{
#if GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE
	return g_stripBuffer ? g_stripRows : 0;
#else
	return 0;
#endif
}

// Whole-display-list replays clipped to a large region (a scroller's
// overlay/extra repaint, the conservative full replay) are the bursts that
// starve a bounce refill: tens of thousands of read-modify-written PSRAM
// pixels with no guard. They stage through the strip even where the per-region
// direct replay above does not (that one is cheap per strip and was measured
// tear-free; these are rare, so the per-strip list walk costs little).
extern "C" int gea_bulk_replay_stage_rows()
{
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	return g_stripBuffer ? g_stripRows : 0;
#else
	return gea_replay_stage_rows();
#endif
}

namespace {
// A staged window packs its rows at its own width, so a narrow window (a
// scrollbar thumb's 10 px column) holds many more rows than a full-width strip:
// one strip, one display-list walk, instead of one per 16 rows. Measured on the
// Analytics tab: the 10x337 thumb repaint took ~5 ms as 21 strips (160 us list
// walk + 116 us copy-back each).
int stageStrideFor(int width)
{
	int stride = (width + 7) & ~7;
	if (stride > kWidth) stride = kWidth;
	return stride < 8 ? 8 : stride;
}
// Pipelined staging (gea_replay_stage_pipeline): the main core replays into one
// slot of the strip buffer while the worker core copies the previous slot back
// into the framebuffer. The copy-back is PSRAM-write bound (~1.3 ms per 24-row
// full-width strip, ~27 ms a full frame), the replay mostly SRAM work, so they
// overlap instead of adding up. With one strip half the strip is split into
// two half-height slots.
bool g_stagePipeline = false;
int g_pipeSlot = 0;
bool g_pipeInFlight = false;
struct StageCopyJob {
	const std::uint16_t *strip;  // the slot, addressed at (dx0, dy0)
	int stride;                  // slot row stride in pixels
	int x0;                      // first framebuffer column
	std::size_t rowBytes;
	int buffer;                  // scanout buffer index written
	bool sums;                   // full-width rows: skip the ones that repeat
};

// Row repeats. A full repaint staged strip by strip (a 3D frame under the UI)
// often leaves many rows exactly as the same buffer already holds them -- the
// sky above Neon Drift's horizon, half the screen -- and the copy-back is
// PSRAM-write bound (~84 us a full row). Per buffer and physical row: a hash of
// what the last staged copy wrote there, trusted only while nothing else has
// written the row since. Every other writer forgets: scanoutRowCopy (patches,
// scroll and rect copies, strip publishes) per row, an unstaged replay
// (gea_display_note_direct_draw), a frame flushed without a staged repaint, and
// whole-buffer copies and rotations entirely. Hashing a row is ~7 us of
// internal-RAM reads on the worker core.
std::uint64_t *g_rowSum[kMaxScanFbs] = {nullptr, nullptr, nullptr};
std::uint8_t *g_rowSumOk[kMaxScanFbs] = {nullptr, nullptr, nullptr};
// A pipelined staged repaint ran since the last flush.
bool g_frameStagedRepaint = false;

void forgetRowSums()
{
	for (int b = 0; b < kMaxScanFbs; ++b)
		if (g_rowSumOk[b]) std::memset(g_rowSumOk[b], 0, kHeight);
}

// Forgets the row a write to `dst` lands in, if it is in a scanout buffer.
inline void forgetRowSumAt(const void *dst)
{
	const auto *p = static_cast<const std::uint16_t *>(dst);
	for (int b = 0; b < kMaxScanFbs; ++b) {
		if (!g_rowSumOk[b] || p < g_scanFb[b] || p >= g_scanFb[b] + kPixelCount) continue;
		g_rowSumOk[b][(p - g_scanFb[b]) / kWidth] = 0;
		return;
	}
}

bool ensureRowSums(int buffer)
{
	if (buffer < 0 || buffer >= kMaxScanFbs) return false;
	if (!g_rowSum[buffer]) g_rowSum[buffer] = static_cast<std::uint64_t *>(heap_caps_calloc(kHeight, sizeof(std::uint64_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!g_rowSumOk[buffer]) g_rowSumOk[buffer] = static_cast<std::uint8_t *>(heap_caps_calloc(kHeight, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	return g_rowSum[buffer] && g_rowSumOk[buffer];
}

// Four independent multiply chains over interleaved words, so the in-order
// core never waits on one multiply's result for the next.
inline std::uint64_t rowSum(const std::uint16_t *row)
{
	static_assert(kWidth % 8 == 0, "rows hash as groups of four words");
	const std::uint32_t *w = reinterpret_cast<const std::uint32_t *>(row);
	constexpr std::uint32_t kPrime = 0x01000193u;
	std::uint32_t a = 0x9E3779B9u, b = 0x85EBCA6Bu, c = 0xC2B2AE35u, d = 0x27D4EB2Fu;
	for (int i = 0; i < kWidth / 2; i += 4) {
		a = (a + w[i]) * kPrime;
		b = (b + w[i + 1]) * kPrime;
		c = (c + w[i + 2]) * kPrime;
		d = (d + w[i + 3]) * kPrime;
	}
	return (static_cast<std::uint64_t>((a * kPrime) ^ b) << 32) | ((c * kPrime) ^ d);
}

// True when physical row `phys` of `buffer` already holds `src` (a full row);
// otherwise records `src` as what the caller is about to write there.
inline bool rowRepeats(int buffer, int phys, const std::uint16_t *src)
{
	const std::uint64_t sum = rowSum(src);
	if (g_rowSumOk[buffer][phys] && g_rowSum[buffer][phys] == sum) return true;
	g_rowSum[buffer][phys] = sum;
	g_rowSumOk[buffer][phys] = 1;
	return false;
}
StageCopyJob g_stageCopyJobs[2];
int pipeSlotRows() { return g_stripHalves >= 2 ? g_stripRows : g_stripRows / 2; }
std::uint16_t *pipeSlotBase(int slot) { return g_stripBuffer + static_cast<std::size_t>(slot) * pipeSlotRows() * kWidth; }
int stageRowsFor(int width)
{
	if (!g_stripBuffer || g_stripRows <= 0) return 0;
	const int slotRows = g_stagePipeline ? pipeSlotRows() : g_stripRows;
	const int rows = slotRows * kWidth / stageStrideFor(width);
	return rows < kHeight ? rows : kHeight;
}
}  // namespace

// Rows a staged window of `width` pixels may span (0 = no staging).
extern "C" int gea_bulk_replay_stage_rows_for_width(int width)
{
	if (gea_bulk_replay_stage_rows() <= 0) return 0;
	return stageRowsFor(width);
}

extern "C" bool gea_replay_stage_active()
{
	return g_replayStage[xPortGetCoreID() & 1].active;
}

// The next stages need no seed: the replay paints every pixel of them opaquely.
bool g_stageSkipSeed = false;
extern "C" void gea_replay_stage_skip_seed(bool skip) { g_stageSkipSeed = skip; }

extern "C" bool gea_replay_stage_begin(int x0, int y0, int x1, int y1)
{
	if (!g_stripBuffer || !g_drawFramebuffer) return false;
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= kWidth) x1 = kWidth - 1;
	if (y1 >= kHeight) y1 = kHeight - 1;
	const int rows = y1 - y0 + 1;
	if (x0 > x1 || rows <= 0 || rows > stageRowsFor(x1 - x0 + 1)) return false;
	const int core = xPortGetCoreID() & 1;
	const bool pipelined = g_stagePipeline && core == 0;
	if (!pipelined && core >= g_stripHalves) return false;
	ReplayStageState &st = g_replayStage[core];
	if (st.active) return false;  // staging never nests; fall back to direct if it ever would
	// Only a replay into the draw framebuffer stages: one into a scratch buffer
	// (a retained snapshot, a fused raster chunk) would have its strip copied
	// back into the framebuffer instead of the buffer it was drawing.
	if (stateCanvas().pixels() != g_drawFramebuffer) return false;
	std::uint16_t *strip = pipelined ? pipeSlotBase(g_pipeSlot) : g_stripBuffer + static_cast<std::size_t>(core) * g_stripRows * kWidth;
	// Ready the back buffer first: until then it holds a frame-old picture under
	// a ring mapping that has not taken the scrolls since, so a seed read now
	// comes from the wrong rows and the copy-back writes them over the patched
	// frame -- on a scrolling Analytics tab, bands of the page (the top rows
	// among them) landed elsewhere on screen.
	auto &canvas = replayCanvas();
	// Seed the window from the back buffer so alpha blends and partial paints meet
	// the same dst bytes the direct path would have.
	const std::int64_t s0 = esp_timer_get_time();
	const int width = x1 - x0 + 1;
	const int stride = stageStrideFor(width);
	for (int y = y0; y <= y1 && !g_stageSkipSeed; ++y) {
		std::memcpy(&strip[static_cast<std::size_t>(y - y0) * stride],
		            drawRow(y) + x0,
		            static_cast<std::size_t>(width) * sizeof(std::uint16_t));
	}
	st.seedUs += esp_timer_get_time() - s0;
	st.px += width * rows;
	st.hadDirty = canvas.dirty(&st.dx0, &st.dy0, &st.dx1, &st.dy1);
	st.alpha = canvas.globalAlpha();
	// Window binding: claim rows [0..y1] with the pixel base rewound by y0 rows and
	// x0 columns, so screen pixel (x, y) lands at strip[(y - y0) * stride + x - x0]
	// -- the replay keeps drawing in screen coordinates. Everything outside the
	// window is unbacked (or another row's) memory; the guard clip below keeps
	// every drawer inside the window (all drawers clamp to the clip, and the
	// replay never reads outside it).
	canvas.bindPixels(strip - static_cast<std::size_t>(y0) * stride - x0, kWidth, y1 + 1, stride);
	canvas.setGlobalAlpha(st.alpha);
	canvas.pushClip(x0, y0, width, rows);
	st.active = true;
	st.unseeded = g_stageSkipSeed;
	st.strip = strip;
	st.stride = stride;
	st.x0 = x0;
	st.y0 = y0;
	st.x1 = x1;
	st.y1 = y1;
	st.beginDoneUs = esp_timer_get_time();
	return true;
}

namespace {
// Worker-core half of a pipelined strip: copies rows [y0, y1] of the slot back
// into the framebuffer, one cache write-back per run of consecutive rows.
void stageCopyRows(void *ctx, int y0, int y1)
{
	const StageCopyJob &job = *static_cast<const StageCopyJob *>(ctx);
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	const bool waitRefill = static_cast<int>(xPortGetCoreID()) != g_panelIsrCore;
	std::uint16_t *runStart = nullptr;
	std::uint16_t *runLast = nullptr;
	int runRows = 0;
	auto flushRun = [&]() {
		if (!runStart) return;
		esp_cache_msync(runStart,
		                static_cast<std::size_t>(runLast - runStart) * sizeof(std::uint16_t) + job.rowBytes,
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
		runStart = runLast = nullptr;
		runRows = 0;
	};
	for (int y = y0; y <= y1; ++y) {
		std::uint16_t *dst = drawRow(y) + job.x0;
		const std::uint16_t *src = job.strip + static_cast<std::ptrdiff_t>(y) * job.stride;
		if (job.sums && rowRepeats(job.buffer, static_cast<int>((dst - job.x0 - g_scanFb[job.buffer]) / kWidth), src)) continue;
		if (runStart && (dst != runLast + kWidth || runRows >= 64)) flushRun();
		if (waitRefill) waitOutRefill();
		std::memcpy(dst, src, job.rowBytes);
		if (!runStart) runStart = dst;
		runLast = dst;
		++runRows;
	}
	flushRun();
#else
	for (int y = y0; y <= y1; ++y)
		scanoutRowCopy(drawRow(y) + job.x0, job.strip + static_cast<std::ptrdiff_t>(y) * job.stride, job.rowBytes);
#endif
}
}  // namespace

// Turns pipelined staging on for a run of full-replay strips on the main core
// (see g_stagePipeline), or off, waiting out the last copy-back so the
// framebuffer is whole before anything reads or publishes it.
extern "C" void gea_replay_stage_pipeline(bool on)
{
	if (on) {
		if (g_stagePipeline || !g_stripBuffer || pipeSlotRows() < 4 || (xPortGetCoreID() & 1) != 0) return;
		if (g_replayStage[0].active) return;
		g_stagePipeline = true;
		g_pipeSlot = 0;
		g_frameStagedRepaint = true;
		return;
	}
	if (!g_stagePipeline) return;
	if (g_pipeInFlight) {
		gea_render_parallel_wait();
		g_pipeInFlight = false;
	}
	g_stagePipeline = false;
}

extern "C" void gea_replay_stage_end()
{
	const int core = xPortGetCoreID() & 1;
	ReplayStageState &st = g_replayStage[core];
	if (!st.active) return;
	st.active = false;
	const std::int64_t e0 = esp_timer_get_time();
	st.bodyUs += e0 - st.beginDoneUs;
	auto &canvas = replayCanvas();
	int dx0, dy0, dx1, dy1;
	bool dirty = canvas.dirty(&dx0, &dy0, &dx1, &dy1);
	if (st.unseeded) {
		// No seed: the dirty bbox no longer bounds what differs from the back
		// buffer. A fill trims pixels that already hold its colour and leaves
		// them out of the dirty rect -- in an unseeded strip those are leftovers
		// of an earlier strip, matching the fill but not the back buffer. Copied
		// back by bbox, a scroll frame left the blit-dragged corner button's edge
		// standing in the page gutter (a blue bar beside the scrollbar). The
		// cover painted the whole window, so copy all of it.
		dx0 = st.x0; dy0 = st.y0; dx1 = st.x1; dy1 = st.y1;
		dirty = true;
	} else if (dirty) {
		// Copy back only the painted bbox (clamped to the window) — unpainted strip
		// pixels are seed-identical, so skipping them saves PSRAM write traffic.
		if (dx0 < st.x0) dx0 = st.x0;
		if (dy0 < st.y0) dy0 = st.y0;
		if (dx1 > st.x1) dx1 = st.x1;
		if (dy1 > st.y1) dy1 = st.y1;
		dirty = dx0 <= dx1 && dy0 <= dy1;
	}
	bool copied = false;
	if (dirty && g_stagePipeline && core == 0) {
		// Hand the copy-back to the worker core; the next strip replays into the
		// other slot meanwhile. One job in flight: when the worker is still busy
		// with the previous one (its core also runs the panel ISR and the 3D
		// frame build), this core copies the slot itself rather than wait, and
		// the next strip reuses it.
		StageCopyJob &job = g_stageCopyJobs[g_pipeSlot];
		job.strip = st.strip + static_cast<std::size_t>(dy0 - st.y0) * st.stride + (dx0 - st.x0) - static_cast<std::ptrdiff_t>(dy0) * st.stride;
		job.stride = st.stride;
		job.x0 = dx0;
		job.rowBytes = static_cast<std::size_t>(dx1 - dx0 + 1) * sizeof(std::uint16_t);
		job.buffer = g_drawIndex;
		job.sums = dx0 == 0 && dx1 == kWidth - 1 && st.stride == kWidth && ensureRowSums(g_drawIndex);
		if (g_pipeInFlight && !g_renderRowWorker.done.load(std::memory_order_acquire)) {
			stageCopyRows(&job, dy0, dy1);
			copied = true;
		} else if (g_pipeInFlight) {
			gea_render_parallel_wait();
			g_pipeInFlight = false;
		}
		if (!copied && gea_render_parallel_rows_submit(stageCopyRows, &job, dy0, dy1)) {
			g_pipeInFlight = true;
			g_pipeSlot ^= 1;
			copied = true;
		}
	}
	if (dirty && !copied) {
		const std::uint16_t *strip = st.strip;
		const int width = dx1 - dx0 + 1;
		const std::size_t rowBytes = static_cast<std::size_t>(width) * sizeof(std::uint16_t);
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
		// One cache write-back per run of physically consecutive rows, not one per
		// row: a narrow window (a scrollbar thumb's 10 px x 337 rows) paid ~7 us of
		// esp_cache_msync per 20-byte row, ~2.4 ms of a 3 ms repaint. The run's
		// span also covers the other columns' lines between its rows; writing
		// those back early is harmless.
		const bool waitRefill = static_cast<int>(xPortGetCoreID()) != g_panelIsrCore;
		std::uint16_t *runStart = nullptr;
		std::uint16_t *runLast = nullptr;
		int runRows = 0;
		auto flushRun = [&]() {
			if (!runStart) return;
			esp_cache_msync(runStart,
			                static_cast<std::size_t>(runLast - runStart) * sizeof(std::uint16_t) + rowBytes,
			                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
			runStart = runLast = nullptr;
			runRows = 0;
		};
		for (int y = dy0; y <= dy1; ++y) {
			std::uint16_t *dst = drawRow(y) + dx0;
			if (runStart && (dst != runLast + kWidth || runRows >= 64)) flushRun();
			if (waitRefill) waitOutRefill();
			std::memcpy(dst, &strip[static_cast<std::size_t>(y - st.y0) * st.stride + (dx0 - st.x0)], rowBytes);
			forgetRowSumAt(dst);
			if (!runStart) runStart = dst;
			runLast = dst;
			++runRows;
		}
		flushRun();
#else
		for (int y = dy0; y <= dy1; ++y)
			scanoutRowCopy(drawRow(y) + dx0, &strip[static_cast<std::size_t>(y - st.y0) * st.stride + (dx0 - st.x0)], rowBytes);
#endif
	}
	canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	applyCanvasMap();
	canvas.setGlobalAlpha(st.alpha);
	// bindPixels reset the dirty tracker: restore the frame's accumulated union,
	// then add this strip's painted bbox so the publish copy transmits it.
	if (st.hadDirty) canvas.markDirty(st.dx0, st.dy0, st.dx1, st.dy1);
	if (dirty) canvas.markDirty(dx0, dy0, dx1, dy1);
	st.cbUs += esp_timer_get_time() - e0;
	if (++st.strips >= kStageDiagStrips) {
		std::printf("[stage] core%d strips=%d px=%d seed=%dus body=%dus cb=%dus (avg/strip) pie=%d/%dpx\n",
		            core, st.strips, st.px / st.strips,
		            static_cast<int>(st.seedUs / st.strips),
		            static_cast<int>(st.bodyUs / st.strips),
		            static_cast<int>(st.cbUs / st.strips),
		            g_pieCalls, g_pieCount8 * 8);
		g_pieCalls = 0;
		g_pieCount8 = 0;
		st.seedUs = st.bodyUs = st.cbUs = 0;
		st.strips = 0;
		st.px = 0;
	}
}

namespace {

// ── PIE SIMD over-blend (gea_pie_blend hooks) ────────────────────────────────
// Same S3 vector kernel as targets/esp32/display.cpp's pieBlendSpan8, minus the
// panel<->normal byte-swap steps: this board stores native-order RGB565
// (GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=0), so the framebuffer bytes already ARE the
// "normal" layout the channel masks need. Contract unchanged: dst = bg in /
// result out (storage order), fgN = normal-RGB565 fg, a5 = per-pixel 0..32;
// lerp oC = bgC + ((fgC-bgC)*a5)>>5 per channel, +16 round-to-nearest.
// cb2 rows (16B apart): [0]=1, [1]=0x1F, [2]=0x3F, [5]=2048(<<11), [6]=32(<<5),
// [7]=16(round); rows 3/4 (0xFF mask, <<8) are the swap constants — unused here
// but kept so the table layout matches the esp32 kernel's documented block.
// Held: q0=bg, q1=ones, q2=fgN, q3=a5, q6=accum. Scratch: q4, q5(const/mask), q7.
void pieBlendSpan8Native(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8,
                         const std::int16_t *cb2)
{
	int t, ad;
	asm volatile(
	    "ee.vld.128.ip q1, %[cb], 0\n" // q1 = ones (cb2[0])
	    "1:\n"
	    "ee.vld.128.ip q0, %[d], 0\n"  // q0 = bg (native order), no increment (stored back)
	    "ee.vld.128.ip q2, %[f], 16\n" // q2 = fgN
	    "ee.vld.128.ip q3, %[a], 16\n" // q3 = a5
	    // B channel (bits 0-4): oB = bgB + ((fgB-bgB)*a5 + 16)>>5
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
	    "ee.orq q6, q6, q7\n"                                                                                         // q6 = result (native order)
	    "ee.vst.128.ip q6, %[d], 16\n" // store result + advance dst by 8 px
	    "addi %[n], %[n], -1\n bnez %[n], 1b\n"
	    : [d] "+r"(dst), [f] "+r"(fgN), [a] "+r"(a5), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
	    : [cb] "r"(cb2)
	    : "memory");
}

}  // namespace

// Like-for-like on css-3d-cube (same app build, even split, no staging): scalar
// gradient replay = 28-41ms/frame, SIMD = ~13.5ms — the kernel halves the
// dominant replay cost. (An earlier "no difference" reading compared across two
// different app revisions — the example was being reworked in a parallel session
// mid-measurement.)
extern "C" bool gea_pie_blend_available() { return true; }
extern "C" void gea_pie_blend_span8(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8)
{
	// Same constant block layout as the esp32 kernel (rows 3/4 unused here).
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
	// PER-CORE critical section — the PIE q-registers are not saved across FreeRTOS
	// context switches, so a mid-kernel preemption (WiFi/system tasks on core 0,
	// the render worker's own preemptors on core 1) corrupts the blend. Per-core
	// mux so the two render cores never serialize on it (proven pattern on the
	// esp32 target, where corruption stripes appeared without the guard).
	static portMUX_TYPE pieMux[2] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
	portMUX_TYPE *mux = &pieMux[xPortGetCoreID() & 1];
	g_pieCalls = g_pieCalls + 1;
	g_pieCount8 = g_pieCount8 + count8;
	taskENTER_CRITICAL(mux);
	pieBlendSpan8Native(dst, fgN, a5, count8, &cb2[0][0]);
	taskEXIT_CRITICAL(mux);
}

extern "C" int gea_current_render_core()
{
	return static_cast<int>(xPortGetCoreID());
}

extern "C" void gea_render_parallel_wait()
{
	if (!g_renderRowWorker.task) return;
	const std::int64_t w0 = esp_timer_get_time();
	const std::int64_t deadline = w0 + 100000;
	while (!g_renderRowWorker.done.load(std::memory_order_acquire)) {
		if (esp_timer_get_time() > deadline) {
			ESP_LOGW(kTag, "row worker wait timeout");
			break;
		}
	}
	if (g_lastSubmitWasDrawBand) {
		g_drawBandWaitUs += esp_timer_get_time() - w0;
		if (++g_drawBandJobs >= 300) {
			std::printf("[worker] draw-band jobs=300 wait=%dus band=%dus avg\n",
			            static_cast<int>(g_drawBandWaitUs / 300),
			            static_cast<int>(g_renderRowWorker.bandUs / 300));
			g_drawBandWaitUs = 0;
			g_drawBandJobs = 0;
			g_renderRowWorker.bandUs = 0;
		}
	}
}

namespace gea::platform::display {

bool Display::init()
{
	if (g_initialized) return true;
	esp_err_t err = initBacklight();
	if (err != ESP_OK) return false;
	err = resetPanel();
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "panel reset failed: %s", esp_err_to_name(err));
		return false;
	}
	err = initRgbController();
	if (err != ESP_OK) return false;
	err = initRgbPanel();
	if (err != ESP_OK) return false;
	g_initialized = true;
	return true;
}

bool Display::start()
{
	if (!g_initialized && !init()) return false;
	setBrightness(g_brightness);
	flush();
	return true;
}

// Routed per-core: the worker core's band replay (and render.cpp helpers that
// draw raw pixels through canvas()) must hit the band canvas — its own clip +
// dirty accumulator over the same framebuffer. On the main core this is g_canvas.
gea::framework::graphics::Canvas *Display::canvas()
{
#if GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH
	// Fused builds: the engine asks for canvas() early each frame only to read
	// its dirty box and scroll region, and draws inside the fused raster
	// callbacks. Readying here would patch the whole stale area before
	// flushRectsRasterized can say which rects it repaints (measured on the
	// CrowPanel 7: 85k px / 12ms per frame, 13 fps vs 15.4). Still wait for the
	// panel to let go of the buffer, so a raw draw through this pointer can never
	// hit rows being scanned; one that lands before the patch is counted as late.
	if (g_swapFbs && !g_backReady && !onWorkerCore()) waitBackBufferFree();
	return &stateCanvas();
#else
	return &replayCanvas();
#endif
}

// The render layer binds canvas() onto a scratch buffer (retained snapshots for
// screenshots/OTA, fused rasterized flushes) and calls this to point it back at
// the draw framebuffer. bindPixels resets the dirty tracker, which is right
// here: the union it held describes the scratch buffer, not the framebuffer.
void Display::rebindCanvasToFramebuffer()
{
	stateCanvas().bindPixels(g_drawFramebuffer, kWidth, kHeight);
	applyCanvasMap();
}

// The RGB panel scans out of g_framebuffer continuously, so it holds exactly
// what the panel shows: a screenshot copies it rather than re-rendering the
// display list on the device-control task (which also overflowed that task's
// stack rasterizing triangles).
extern "C" bool geaDisplaySnapshotPrefersPresented() { return true; }

bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height, bool)
{
	if (width) *width = kWidth;
	if (height) *height = kHeight;
	if (!dst || pixelCapacity < static_cast<int>(kPixelCount)) return false;
	const std::uint16_t *source = g_framebuffer ? g_framebuffer : g_drawFramebuffer;
	if (!source) return false;
	const ScanMap map = g_framebuffer ? g_map[g_frontIndex] : g_map[g_drawIndex];
	// Native pixels are panel byte order; a snapshot is plain RGB565. Rows go
	// through the buffer's ring mapping, as the panel shows them.
	for (int y = 0; y < kHeight; ++y) {
		const std::uint16_t *row = source + static_cast<std::size_t>(mapRow(map, y)) * kWidth;
		std::uint16_t *out = dst + static_cast<std::size_t>(y) * kWidth;
		for (int x = 0; x < kWidth; ++x) out[x] = gea::framework::graphics::pixel::toRgb565(row[x]);
	}
	return true;
}

int Display::countNonBlackPixels(bool)
{
	const std::uint16_t *source = g_framebuffer ? g_framebuffer : g_drawFramebuffer;
	if (!source) return 0;
	int count = 0;
	for (std::size_t i = 0; i < kPixelCount; ++i) {
		if (source[i] != 0) ++count;
	}
	return count;
}

void Display::clear()
{
	ensureBackReady();
	g_canvas.clear(0x0000);
	flush();
}

void Display::clearNoFlush()
{
	ensureBackReady();
	g_canvas.clear(0x0000);
}
void Display::print(const char *) {}

void Display::flush()
{
	// Whatever drew this frame outside a staged repaint left no record. (A
	// flush with nothing drawn since the last one is not a frame of its own.)
	{
		int fx0, fy0, fx1, fy1;
		if (!g_frameStagedRepaint && g_canvas.dirty(&fx0, &fy0, &fx1, &fy1)) forgetRowSums();
	}
	g_frameStagedRepaint = false;
	if (!g_framebuffer || !g_drawFramebuffer) return;
	const int64_t start = esp_timer_get_time();
	int64_t copyUs = 0;
	int64_t cacheUs = 0;
	int x0 = 0;
	int y0 = 0;
	int x1 = -1;
	int y1 = -1;
	int pixels = 0;
		if (g_canvas.dirty(&x0, &y0, &x1, &y1)) {
			const int64_t copyStart = esp_timer_get_time();
			pixels = publishDrawRect(x0, y0, x1, y1, &x0, &y0, &x1, &y1);
			copyUs += esp_timer_get_time() - copyStart;
		}
		if (pixels > 0) {
			const int64_t cacheStart = esp_timer_get_time();
			cacheWritebackRect(x0, y0, x1, y1);
			cacheUs += esp_timer_get_time() - cacheStart;
		}
	g_canvas.resetDirty();
	if (g_swapFbs) {
		const int64_t swapStart = esp_timer_get_time();
		publishSwap();
		copyUs += esp_timer_get_time() - swapStart;
		yieldToIdleIfStarved();
	}
	noteFrameDiag(0, esp_timer_get_time() - start, pixels);
	addFlushStats(esp_timer_get_time() - start, pixels);
	addFlushStageStats(copyUs, cacheUs);
}

void Display::flushRects(const DisplayFlushRect *rects, int count, bool)
{
	// Whatever drew this frame outside a staged repaint left no record. (A
	// flush with nothing drawn since the last one is not a frame of its own.)
	{
		int fx0, fy0, fx1, fy1;
		if (!g_frameStagedRepaint && g_canvas.dirty(&fx0, &fy0, &fx1, &fy1)) forgetRowSums();
	}
	g_frameStagedRepaint = false;
	if (!g_framebuffer || !g_drawFramebuffer || !rects || count <= 0) return;
	const int64_t start = esp_timer_get_time();
	int64_t copyUs = 0;
	int64_t cacheUs = 0;
		int pixels = 0;
		for (int i = 0; i < count; ++i) {
			int x0 = 0;
			int y0 = 0;
			int x1 = -1;
			int y1 = -1;
			const int64_t copyStart = esp_timer_get_time();
			const int rectPixels = publishDrawRect(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1, &x0, &y0, &x1, &y1);
			copyUs += esp_timer_get_time() - copyStart;
			if (rectPixels > 0) {
				pixels += rectPixels;
				const int64_t cacheStart = esp_timer_get_time();
				cacheWritebackRect(x0, y0, x1, y1);
				cacheUs += esp_timer_get_time() - cacheStart;
			}
		}
		if (g_swapFbs) {
			// The back buffer is the live draw target, so anything drawn outside the
			// rects is about to be shown too: record it, or the front would never
			// catch up and the pixels would flicker between frames.
			int dx0, dy0, dx1, dy1;
			if (g_canvas.dirty(&dx0, &dy0, &dx1, &dy1) && clipRect(&dx0, &dy0, &dx1, &dy1)) {
				// The dirty tracker is one bounding box, so test it against the rects'
				// bounding box: the renderer clips its drawing to the rects themselves.
				int bx0 = rects[0].x0, by0 = rects[0].y0, bx1 = rects[0].x1, by1 = rects[0].y1;
				for (int i = 1; i < count; ++i) {
					bx0 = std::min(bx0, rects[i].x0);
					by0 = std::min(by0, rects[i].y0);
					bx1 = std::max(bx1, rects[i].x1);
					by1 = std::max(by1, rects[i].y1);
				}
				if (dx0 < bx0 || dy0 < by0 || dx1 > bx1 || dy1 > by1) markStale(dx0, dy0, dx1, dy1);
			}
		}
		g_canvas.resetDirty();
	if (g_swapFbs) {
		const int64_t swapStart = esp_timer_get_time();
		publishSwap();
		copyUs += esp_timer_get_time() - swapStart;
		yieldToIdleIfStarved();
	}
	noteFrameDiag(1, esp_timer_get_time() - start, pixels);
	addFlushStats(esp_timer_get_time() - start, pixels);
	addFlushStageStats(copyUs, cacheUs);
}

}  // namespace gea::platform::display

namespace {
bool streamRectNoSwap(int x, int y, int w, int h, gea::platform::display::DisplayStreamRasterFn raster, void *user)
{
	if (!g_framebuffer || !g_drawFramebuffer || !raster || w <= 0 || h <= 0) return false;
	int x0 = x;
	int y0 = y;
	int x1 = x + w - 1;
	int y1 = y + h - 1;
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= kWidth) x1 = kWidth - 1;
	if (y1 >= kHeight) y1 = kHeight - 1;
	if (x0 > x1 || y0 > y1) return true;

	const int width = x1 - x0 + 1;
	const int height = y1 - y0 + 1;
	ensureBackReady();  // no-op when flushRectsRasterized already readied it
	const int64_t start = esp_timer_get_time();
	if (g_stripBuffer) {
		// Raster the region band by band into internal RAM (fast, off the PSRAM
		// bus, blends never read PSRAM) and write each finished band out once. The
		// raster contract is per-chunk -- the QSPI backend calls it per DMA chunk.
		const int bandRows = std::max(1, static_cast<int>(stripBufferPixels() / static_cast<std::size_t>(width)));
		for (int row = 0; row < height; row += bandRows) {
			const int rows = std::min(bandRows, height - row);
			const std::int64_t r0 = esp_timer_get_time();
			raster(g_stripBuffer, width, rows, x0, y0 + row, user);
			const std::int64_t r1 = esp_timer_get_time();
			for (int i = 0; i < rows; ++i) {
				scanoutRowCopy(drawRow(y0 + row + i) + x0, &g_stripBuffer[static_cast<std::size_t>(i) * width],
				               static_cast<std::size_t>(width) * sizeof(std::uint16_t));
			}
			g_swapDiag.rasterUs += r1 - r0;
			g_swapDiag.copyUs += esp_timer_get_time() - r1;
		}
	} else {
		const std::size_t bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * sizeof(std::uint16_t);
		auto *scratch = static_cast<std::uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (!scratch) scratch = static_cast<std::uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
		if (!scratch) return false;
		raster(scratch, width, height, x0, y0, user);
		g_swapDiag.rasterUs += esp_timer_get_time() - start;
		for (int row = 0; row < height; ++row) {
			std::copy_n(&scratch[static_cast<std::size_t>(row) * width],
			            width,
			            drawRow(y0 + row) + x0);
		}
		heap_caps_free(scratch);
	}
		int publishedX0 = 0;
		int publishedY0 = 0;
		int publishedX1 = -1;
		int publishedY1 = -1;
		const int pixels = publishDrawRect(x0, y0, x1, y1, &publishedX0, &publishedY0, &publishedX1, &publishedY1);
		if (pixels > 0) cacheWritebackRect(publishedX0, publishedY0, publishedX1, publishedY1);
		g_canvas.resetDirty();
		addFlushStats(esp_timer_get_time() - start, pixels);
	return true;
}
}  // namespace

namespace gea::platform::display {

bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user)
{
	if (raster && w > 0 && h > 0) {
		const DisplayFlushRect rect = {x, y, x + w - 1, y + h - 1};
		ensureBackReady(&rect, 1);
	}
	const bool ok = streamRectNoSwap(x, y, w, h, raster, user);
	if (g_swapFbs) publishSwap();
	return ok;
}

// Many small rects (64 bouncing balls): copying a rect's short rows out to
// PSRAM is bound by bus latency, not bandwidth -- each row is a few cache-line
// round trips while scanout and PSRAM-resident code share the bus -- and cost
// ~2x the raster. So the worker core copies rect i out of one half of the SRAM
// strip while this core rasters rect i+1 into the other half.
struct RectCopyJob {
	const std::uint16_t *src;
	int x0, y0, width, height;
};
RectCopyJob g_rectCopyJob{};

void copyRectJob(void *ctx, int, int)
{
	const RectCopyJob &j = *static_cast<const RectCopyJob *>(ctx);
	const std::size_t bytes = static_cast<std::size_t>(j.width) * sizeof(std::uint16_t);
	for (int i = 0; i < j.height; ++i)
		scanoutRowCopy(drawRow(j.y0 + i) + j.x0, j.src + static_cast<std::size_t>(i) * j.width, bytes);
}

void waitRectCopy()
{
	if (!g_scrollCopyInFlight) return;
	const std::int64_t w0 = esp_timer_get_time();
	while (!g_renderRowWorker.done.load(std::memory_order_acquire)) {
		if (esp_timer_get_time() - w0 > 100000) break;  // a stuck worker must not hang the frame
	}
	g_scrollCopyInFlight = false;
	g_swapDiag.copyUs += esp_timer_get_time() - w0;
}

bool smallRectsPipelinable(const DisplayFlushRect *rects, int count)
{
	if (!g_stripBuffer || count < 4 || g_scrollCopyInFlight) return false;
	const std::size_t slot = stripBufferPixels() / 2;
	for (int i = 0; i < count; ++i) {
		const std::size_t w = static_cast<std::size_t>(rects[i].x1 - rects[i].x0 + 1);
		const std::size_t h = static_cast<std::size_t>(rects[i].y1 - rects[i].y0 + 1);
		if (w * h > slot) return false;
	}
	return true;
}

// The pipelined path for flushRectsRasterized (smallRectsPipelinable said so).
void flushSmallRectsPipelined(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, int *pxOut)
{
	const std::size_t slot = stripBufferPixels() / 2;
	ensureBackReady(rects, count);
	int px = 0, half = 0;
	for (int i = 0; i < count; ++i) {
		int x0 = rects[i].x0, y0 = rects[i].y0, x1 = rects[i].x1, y1 = rects[i].y1;
		if (!clipRect(&x0, &y0, &x1, &y1)) continue;
		const int width = x1 - x0 + 1, height = y1 - y0 + 1;
		std::uint16_t *buf = g_stripBuffer + half * slot;
		const std::int64_t r0 = esp_timer_get_time();
		raster(buf, width, height, x0, y0, user);
		g_swapDiag.rasterUs += esp_timer_get_time() - r0;
		waitRectCopy();  // the other half is free again, and the worker idle
		g_rectCopyJob = {buf, x0, y0, width, height};
		g_scrollCopyInFlight = true;  // band replays stay off the busy worker
		if (!submitRenderWorkerJob(copyRectJob, &g_rectCopyJob, y0, y1)) {
			g_scrollCopyInFlight = false;
			copyRectJob(&g_rectCopyJob, y0, y1);
		}
		publishDrawRect(x0, y0, x1, y1);
		px += width * height;
		half ^= 1;
	}
	waitRectCopy();
	g_canvas.resetDirty();
	*pxOut = px;
}

// Fused flush: each rect is rastered from the display list into internal-RAM
// bands and written to the back buffer once. Because these rects are repainted
// completely, readying the back buffer patches only the stale area outside them.
void Display::flushRectsRasterized(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, bool)
{
	forgetRowSums();
	g_frameStagedRepaint = false;
	if (!rects || count <= 0 || !raster) return;
	const std::int64_t start = esp_timer_get_time();
	// Nothing drew around the strips (no canvas or worker damage yet; the strip
	// writes below reset it): then every row of this frame leaves the cache as
	// it is written, and the publish need not walk them again.
	int dx0, dy0, dx1, dy1;
	const bool writtenBack = !g_canvas.dirty(&dx0, &dy0, &dx1, &dy1) && !g_workerCanvas.dirty(&dx0, &dy0, &dx1, &dy1);
	int px = 0;
	if (smallRectsPipelinable(rects, count)) {
		flushSmallRectsPipelined(rects, count, raster, user, &px);
	} else {
		ensureBackReady(rects, count);
		for (int i = 0; i < count; ++i) {
			streamRectNoSwap(rects[i].x0, rects[i].y0, rects[i].x1 - rects[i].x0 + 1, rects[i].y1 - rects[i].y0 + 1, raster, user);
			px += (rects[i].x1 - rects[i].x0 + 1) * (rects[i].y1 - rects[i].y0 + 1);
		}
	}
	if (g_swapFbs) {
		g_frameWrittenBack = writtenBack;
		publishSwap();
		yieldToIdleIfStarved();
	}
	noteFrameDiag(2, esp_timer_get_time() - start, px);
	rebindCanvasToFramebuffer();
}

bool Display::present(const DisplayPresentCommand *commands, int commandCount)
{
	if (!commands || commandCount <= 0) return false;
	// A frame the app rendered in place (gea_display_acquire_panel_buffer) opens
	// with a draw of the back buffer onto itself: already there, so only what is
	// drawn over it replays.
	const DisplayPresentCommand &base = commands[0];
	if (base.type == gea::platform::display::DisplayPresentCommandType::DrawImage && g_drawFramebuffer &&
	    base.drawImage.pixels == g_drawFramebuffer && base.drawImage.x == 0 && base.drawImage.y == 0 &&
	    base.drawImage.srcWidth == kWidth && base.drawImage.srcHeight == kHeight && !base.drawImage.alphaPixels &&
	    base.drawImage.alpha == 255 && panelBuffersUsable()) {
		const DisplayFlushRect whole{0, 0, kWidth - 1, kHeight - 1};
		ensureBackReady(&whole, 1);
		g_drawBufferStale = false;
		const std::int64_t replayStart = esp_timer_get_time();
		for (int i = 1; i < commandCount; ++i) replayPresentCommand(commands[i]);
		g_swapDiag.rasterUs += esp_timer_get_time() - replayStart;
		g_canvas.setGlobalAlpha(255);
		g_canvas.markDirty(0, 0, kWidth - 1, kHeight - 1);
		flush();
		return true;
	}
#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
	// Fast path: full-screen-redraw frames compose in SRAM strips and publish
	// straight to scanout. Anything without a full-screen opaque base needs the
	// persistent back buffer, so it falls through.
	if (g_stripBuffer && g_stripHalves == 2) {
		present::Frame frame;
		if (present::extractFrame(commands, commandCount, frame) &&
		    present::frameHasOpaqueBase(frame, kWidth, kHeight)) {
			return presentStripFused(frame);
		}
	}
#endif
	// Fallback: per-command replay into the PSRAM back buffer, then publish. If
	// the last frame went through the strip path, the back buffer is stale —
	// re-seed it from the live scanout before drawing the partial update on top.
	// A frame whose first command paints every pixel (a whole-screen
	// putImageData or fill) needs none of the old pixels: bringing the back
	// buffer current from the front would copy the whole frame (~47 ms of PSRAM
	// traffic at 800x480) only for the replay to paint over it.
	present::Frame frame;
	if (present::extractFrame(commands, commandCount, frame) && present::frameHasOpaqueBase(frame, kWidth, kHeight)) {
		const DisplayFlushRect whole{0, 0, kWidth - 1, kHeight - 1};
		ensureBackReady(&whole, 1);
		g_drawBufferStale = false;
	}
	ensureBackReady();
	if (g_drawBufferStale && g_drawFramebuffer != g_framebuffer) {
		std::memcpy(g_drawFramebuffer, g_framebuffer, kFramebufferBytes);
		forgetRowSums();
		g_drawBufferStale = false;
	}
	const std::int64_t replayStart = esp_timer_get_time();
	for (int i = 0; i < commandCount; ++i) {
		replayPresentCommand(commands[i]);
	}
	g_swapDiag.rasterUs += esp_timer_get_time() - replayStart;
	g_canvas.setGlobalAlpha(255);
	flush();
	return true;
}

void Display::setFlushConfig(int chunkRows, int queueDepth)
{
	g_flushRows = chunkRows;
	g_flushDepth = queueDepth;
}

void Display::reserveInternal(std::size_t bytes)
{
  g_internalReservation.store(bytes, std::memory_order_release);
}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
// The optional strip-raster cache is internal staging too. Yield it on the
// frame task before radio/audio bring-up; wait for its row worker before freeing.
// RGB DMA's bounce buffers remain owned by the LCD driver throughout.
void Display::applyPendingInternalReserve()
{
  const std::size_t requested = g_internalReservation.load(std::memory_order_acquire);
  if (requested == g_appliedInternalReservation) return;
  g_appliedInternalReservation = requested;
  if (requested) {
    g_internalCacheHeadroom = std::max(g_internalCacheHeadroom, requested);
    if (g_stripBuffer) {
      gea_render_parallel_wait();
      const unsigned bytes = static_cast<unsigned>(stripBufferPixels() * sizeof(std::uint16_t));
      heap_caps_free(g_stripBuffer);
      g_stripBuffer = nullptr;
      g_stripRows = 0;
      g_stripHalves = 0;
      ESP_LOGI(kTag, "Released %u bytes of strip staging for internal-memory reservation", bytes);
    }
  } else {
#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
    // Restoring an optimization must not consume the headroom just requested
    // by the radios. The PSRAM back-buffer path remains fully functional.
    if (!g_stripBuffer) allocStripBuffer(g_internalCacheHeadroom);
#endif
  }
}
// An app's own internal-RAM allocations (Memory.beginInternal/endInternal, on
// the frame task like the reservation above): the strip yields its block, the
// app takes what it asked for, and the strip comes back at the tallest height
// left -- shorter strips cost a few raster passes, a table read from PSRAM per
// pixel costs far more.
extern "C" void gea_display_release_internal(std::size_t)
{
  if (!g_stripBuffer) return;
  gea_render_parallel_wait();
  heap_caps_free(g_stripBuffer);
  g_stripBuffer = nullptr;
  g_stripRows = 0;
  g_stripHalves = 0;
}
extern "C" void gea_display_restore_internal()
{
#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
  // Before init() the panel allocates its own strip.
  if (!g_initialized || g_stripBuffer || g_appliedInternalReservation) return;
  allocStripBuffer(g_internalCacheHeadroom);
  ESP_LOGI(kTag, "strip-raster rows=%d halves=%d after app internal allocation (internal free=%u largest=%u)", g_stripRows, g_stripHalves,
           static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
           static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
#endif
}
int Display::flushChunkRows() { return g_flushRows; }
int Display::flushQueueDepth() { return g_flushDepth; }
int Display::flushBufferBytes() { return 0; }
// All clip/alpha/draw entry points below route through replayCanvas() so the
// 2nd-core band replay lands on the worker canvas (its own clip + dirty over
// the same framebuffer). On the main core replayCanvas() == g_canvas, so the
// single-core paths are unchanged.
void Display::pushClip(int x, int y, int w, int h) { stateCanvas().pushClip(x, y, w, h); }
void Display::popClip() { stateCanvas().popClip(); }
void Display::resetClip() { stateCanvas().resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { stateCanvas().setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return stateCanvas().globalAlpha(); }
int Display::brightness() { return g_brightness; }

void Display::setBrightness(int brightnessPercent)
{
	if (brightnessPercent < 0) brightnessPercent = 0;
	if (brightnessPercent > 100) brightnessPercent = 100;
	g_brightness = brightnessPercent;
	applyBacklight();
}

// VSync off (Display.setVSync(false)): draw straight into the buffer the panel
// scans, as the single-buffer fallback does. No swap, so no waiting for the
// panel to release the back buffer and no patching it from the front -- on 64
// bouncing balls those cost ~9 ms and a PSRAM copy of every changed pixel a
// frame. The app accepts that a moving object may show split across a scan.
void Display::setVSync(bool on)
{
	g_vsyncWanted = on;
	applyVSync();
}
void Display::invalidate() {}
bool Display::vsyncEnabled() { return g_swapFbs; }
void Display::vsyncWaitForFrame() {}

void Display::clip(int *x0, int *y0, int *x1, int *y1) { stateCanvas().currentClip(x0, y0, x1, y1); }

void Display::fillRect(int x, int y, int w, int h, std::uint16_t color) { replayCanvas().fillRect(x, y, w, h, color); }
namespace {
// Whether the panel is done reading `row` of the back buffer. It is released as a
// whole one frame end after the swap; before that, the panel is in the buffer's
// last frame, and a row it has already DISPLAYED in that frame is never read
// again (the bounce ISR reads ahead of the display, never behind it). The
// display position is counted from that frame's VSYNC, less the rows before the
// active area and a few rows of margin.
bool backRowReleased(std::uint32_t freeSeq, int row)
{
	const std::uint32_t seq = g_frameSeq;
	if (static_cast<std::int32_t>(seq - freeSeq) >= 0) return true;
	if (seq + 1 != freeSeq) return false;
	const std::int64_t vsyncUs = g_vsyncUs, latchUs = g_frameCompleteUs;
	if (g_frameSeq != seq) return backRowReleased(freeSeq, row);
	if (vsyncUs <= latchUs) return false;  // the last frame has not started displaying yet
	const auto &d = gea::platform::board::display;
	const std::int64_t lineClocks = kWidth + d.hsyncPulseWidth + d.hsyncBackPorch + d.hsyncFrontPorch;
	const std::int64_t lines = (esp_timer_get_time() - vsyncUs) * kPixelClockHz / (lineClocks * 1000000);
	return row < lines - d.vsyncPulseWidth - d.vsyncBackPorch - 4;
}
}  // namespace

namespace {
struct ScrollCopy {
	const std::uint16_t *src;
	std::uint16_t *dst;
	int dstX, srcX;
	std::size_t bytes;
	int y0, y1, dy;
	std::uint32_t freeSeq;
	// Rows (by screen row) the back buffer already holds shifted, or null.
	const std::uint8_t *skip = nullptr;
};

void scrollCopyRows(void *ctx, int, int)
{
	auto &c = *static_cast<ScrollCopy *>(ctx);
	bool released = false;
	for (int row = c.y0; row <= c.y1; ++row) {
		const int srcRow = row - c.dy;
		if (srcRow < c.y0 || srcRow > c.y1) continue;
		if (c.skip && c.skip[row]) continue;
		if (!released) {
			const std::int64_t r0 = esp_timer_get_time();
			while (!(released = static_cast<std::int32_t>(g_frameSeq - c.freeSeq) >= 0) && !backRowReleased(c.freeSeq, row)) {
				if (esp_timer_get_time() - r0 > 100000) break;  // panel stopped: don't hang
				esp_rom_delay_us(20);
			}
		}
		// Row by row, each written back at once (see scanoutRowCopy): batching
		// the write-back left dirty lines for the bounce refills to evict, and
		// the panel showed thin glitch lines.
		scanoutRowCopy(&c.dst[static_cast<std::size_t>(row) * kWidth + c.dstX],
		               &c.src[static_cast<std::size_t>(srcRow) * kWidth + c.srcX], c.bytes);
	}
}
}  // namespace

void Display::scrollRect(int x, int y, int w, int h, int dx, int dy)
{
	if (g_ringScroll && !onWorkerCore()) {
		if (dx == 0 && dy != 0 && x <= 0 && x + w >= kWidth && y >= 0 && h > 0 && y + h <= kHeight && dy > -h && dy < h) {
			// The ring path: rotate the draw buffer's mapping. The rows the shift
			// exposes keep pixels from the opposite edge, exactly as an in-place
			// scroll leaves stale rows; the caller repaints them and flushes only
			// what it repainted (gea_display_scroll_rect_self_flushing).
			g_swapDiag.rings++;
			ensureBackReady();
			changeRegion(y, h);
			ScanMap &m = g_map[g_drawIndex];
			m.off = normOffset(m.off - dy, m.h);
			applyCanvasMap();
			for (int i = 0; i < g_scanFbCount; ++i) {
				if (i == g_drawIndex || g_mapReset[i]) continue;
				g_pendingRot[i] += dy;
				shiftStale(g_stale[i], y, h, dy);
			}
			g_publishPending = true;
			return;
		}
		// Anything else moves pixels on an identity layout.
		g_swapDiag.fallbacks++;
		if (g_regionH > 0) {
			ensureBackReady();
			changeRegion(0, 0);
		}
	}
	// Swapped scanout buffers, back not yet readied since the swap: its pixels
	// are a frame old, so readying it would copy the whole viewport from the
	// front only for the in-place scroll to move it all again. Ready everything
	// outside the viewport, then copy the viewport from the front, shifted --
	// one pass over the pixels instead of two. Rows the shift exposes keep stale
	// pixels, exactly like an in-place scroll; the caller repaints them.
	if (g_swapFbs && !g_backReady && !onWorkerCore() && (dx == 0 || dy == 0) && (dx != 0 || dy != 0)) {
		int x0 = x, y0 = y, x1 = x + w - 1, y1 = y + h - 1;
		int cx0, cy0, cx1, cy1;
		g_canvas.currentClip(&cx0, &cy0, &cx1, &cy1);
		x0 = std::max(x0, cx0); y0 = std::max(y0, cy0);
		x1 = std::min(x1, cx1); y1 = std::min(y1, cy1);
		if (!clipRect(&x0, &y0, &x1, &y1)) return;
		const int rw = x1 - x0 + 1, rh = y1 - y0 + 1;
		if (dx <= -rw || dx >= rw || dy <= -rh || dy >= rh) {
			replayCanvas();  // nothing survives the shift; just ready the buffer
			return;
		}
		const DisplayFlushRect skip = {x0, y0, x1, y1};
		const std::int64_t c0 = esp_timer_get_time();
		const std::uint16_t *src = g_framebuffer;
		std::uint16_t *dst = g_drawFramebuffer;
		const int sx0 = dx > 0 ? x0 : x0 - dx;           // first source column
		const int dxs = dx > 0 ? x0 + dx : x0;           // first destination column
		const std::size_t bytes = static_cast<std::size_t>(rw - (dx > 0 ? dx : -dx)) * sizeof(std::uint16_t);
		// Copy top down behind the panel's scan of the buffer's last frame instead of
		// first waiting for the whole buffer: the copy outruns nothing (the scan is
		// faster), so the wait for the release overlaps the copy. The copy runs on
		// the panel ISR's core when the row worker lives there: a PSRAM-to-PSRAM
		// copy at full speed from the other core starves the bounce ISR's refill of
		// the same bus, and the panel shows stale bounce rows (short glitch lines);
		// on the ISR's core the ISR simply preempts it.
		ScrollCopy job{src, dst, dxs, sx0, bytes, y0, y1, dy, g_freeSeq[g_drawIndex]};
		if (g_panelIsrCore >= 0 && g_panelIsrCore != static_cast<int>(xPortGetCoreID()) &&
		    submitRenderWorkerJob(scrollCopyRows, &job, y0, y1)) {
			g_lastSubmitWasDrawBand = false;
			gea_render_parallel_wait();
		} else {
			scrollCopyRows(&job, y0, y1);
		}
		cacheWritebackRect(dst, x0, y0, x1, y1);
		const std::int64_t c1 = esp_timer_get_time();
		g_swapDiag.copyUs += c1 - c0;
		ensureBackReady(&skip, 1);  // the rest of the buffer, outside the viewport
		g_canvas.markDirty(x0, y0, x1, y1);
		// The whole viewport changed: hand it to the other buffers here. A frame
		// published with flushRects records the canvas box only when it reaches
		// past the rects' bounding box, and a strip on one edge with an overlay
		// on the other spans the viewport: a tab swipe's last step then never
		// reached the other buffer, and every later swap flickered between them.
		markStale(x0, y0, x1, y1);
		return;
	}
	replayCanvas().scrollRect(x, y, w, h, dx, dy);
}
// A sideways scroll's shifted copy, started without waiting for it: the
// worker on the panel ISR's core copies the viewport while this core repaints
// the strip the scroll exposes, which the copy never writes. On the CrowPanel
// 7.0 the copy alone took ~30 ms of a tab swipe's frame and the strip as long
// again, one after the other. Both halves of a scroll's work then fit in one.
namespace {
struct AsyncScroll {
	ScrollCopy job;
	DisplayFlushRect skip;
	std::int64_t startUs;
};
AsyncScroll g_asyncScroll{};

}  // namespace

extern "C" bool gea_display_scroll_rect_begin(int x, int y, int w, int h, int dx, int dy)
{
	// Taken here whatever happens: a later begin must never see these.
	const std::uint32_t *const nextSig = g_nextSig;
	g_nextSig = nullptr;
	if (!g_swapFbs || g_backReady || onWorkerCore() || dy != 0 || dx == 0 || g_regionH > 0) return false;
	if (g_panelIsrCore < 0 || g_panelIsrCore == static_cast<int>(xPortGetCoreID())) return false;
	int x0 = x, y0 = y, x1 = x + w - 1, y1 = y + h - 1;
	int cx0, cy0, cx1, cy1;
	g_canvas.currentClip(&cx0, &cy0, &cx1, &cy1);
	x0 = std::max(x0, cx0); y0 = std::max(y0, cy0);
	x1 = std::min(x1, cx1); y1 = std::min(y1, cy1);
	if (!clipRect(&x0, &y0, &x1, &y1)) return false;
	const int rw = x1 - x0 + 1;
	if (dx <= -rw || dx >= rw) return false;
	xSemaphoreTake(g_readyMutex, portMAX_DELAY);
	// The strip repaint draws into the back buffer straight away, so the panel
	// must be done with it first (it nearly always is by now).
	waitBackBufferFree();
	bringMapCurrent(g_drawIndex);
	const int sx0 = dx > 0 ? x0 : x0 - dx;
	const int dxs = dx > 0 ? x0 + dx : x0;
	const std::size_t bytes = static_cast<std::size_t>(rw - (dx > 0 ? dx : -dx)) * sizeof(std::uint16_t);
	g_asyncScroll.job = ScrollCopy{g_framebuffer, g_drawFramebuffer, dxs, sx0, bytes, y0, y1, 0, g_freeSeq[g_drawIndex]};
	g_asyncScroll.skip = {x0, y0, x1, y1};
	// Skip the rows whose content the shift leaves alone and the back buffer
	// already holds; record the new signatures for this buffer's next turn.
	const int b = g_drawIndex;
	if (!g_rowSig[b]) g_rowSig[b] = static_cast<std::uint32_t *>(heap_caps_calloc(kHeight, sizeof(std::uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (g_rowSig[b] && nextSig && g_nextSigY0 <= y0 && g_nextSigY0 + g_nextSigRows - 1 >= y1) {
		const DisplayFlushRect &pr = g_rowSigRect[b];
		const bool same = g_rowSigValid[b] && pr.x0 == x0 && pr.y0 == y0 && pr.x1 == x1 && pr.y1 == y1;
		int skipped = 0;
		for (int row = y0; row <= y1; ++row) {
			const std::uint32_t sig = nextSig[row - g_nextSigY0];
			g_skipRow[row] = same && sig != 0 && g_rowSig[b][row] == sig;
			skipped += g_skipRow[row];
			g_rowSig[b][row] = sig;
		}
		g_rowSigRect[b] = {x0, y0, x1, y1};
		g_rowSigValid[b] = true;
		if (skipped) g_asyncScroll.job.skip = g_skipRow;
	} else {
		g_rowSigValid[b] = false;
	}
	g_frameIsScrollCopy = true;
	g_asyncScroll.startUs = esp_timer_get_time();
	if (!submitRenderWorkerJob(scrollCopyRows, &g_asyncScroll.job, y0, y1)) {
		g_rowSigValid[b] = false;
		g_frameIsScrollCopy = false;
		xSemaphoreGive(g_readyMutex);
		return false;
	}
	g_lastSubmitWasDrawBand = false;
	g_scrollCopyInFlight = true;
	g_directDrawDuringCopy = false;
	// Ready for drawing inside the viewport; the end call brings the rest of
	// the buffer up to date once the copy is done.
	g_backReady = true;
	xSemaphoreGive(g_readyMutex);
	return true;
}

// GEADEV PANEL: the refill counters since the last read, then reset.
// corrected: refills whose buffer or band differed from the driver's choice;
// fallback: refills that took the driver's (no VSYNC yet, or the DMA position
// unreadable); at_start/at_end: refills that began or ended while the DMA
// was sending that buffer (current/next descriptor) -- at_start must stay 0.
extern "C" void gea_display_panel_stats(char *out, int size)
{
	const RefillStats r = g_refill;
	std::memset(&g_refill, 0, sizeof(g_refill));
	int n = std::snprintf(out, static_cast<std::size_t>(size), "vsync_dev_max_us=%d vsync_dev_gt200=%u ", static_cast<int>(g_vsyncLateMaxUs), static_cast<unsigned>(g_vsyncLate200));
	g_vsyncLateMaxUs = 0;
	g_vsyncLate200 = 0;
	n += std::snprintf(out + n, static_cast<std::size_t>(size - n), "restarts=%u skipped=%u realigned=%u short=%u/%u vs_at:", static_cast<unsigned>(g_vsyncRestarts), static_cast<unsigned>(g_vsyncSkipped), static_cast<unsigned>(g_vsyncRealigned), static_cast<unsigned>(g_shortFrames), static_cast<unsigned>(g_shortFramesLate));
	for (int i = 0; i < 32 && n > 0 && n < size; ++i)
		if (g_vsyncAt[i]) n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " %d.%d:%u", i / 16, i % 16, static_cast<unsigned>(g_vsyncAt[i]));
	std::memset(g_vsyncAt, 0, sizeof(g_vsyncAt));
	g_vsyncRestarts = 0;
	g_vsyncSkipped = 0;
	g_vsyncRealigned = 0;
	g_shortFrames = 0;
	g_shortFramesLate = 0;
	// Slips: uptime_ms:bin/late_us/prev_late_us/eof_fills/prev_eof_fills (kept across reads).
	n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " slips=%u", static_cast<unsigned>(g_slipCount));
	for (std::uint32_t i = g_slipCount > 6 ? g_slipCount - 6 : 0; i < g_slipCount && n > 0 && n < size; ++i) {
		const SlipEvent &e = g_slips[i % 6];
		n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " %u:%u.%u/%u/%u/%u/%u", static_cast<unsigned>(e.ms), e.bin / 16u, e.bin % 16u,
		                   static_cast<unsigned>(e.lateUs), static_cast<unsigned>(e.prevLateUs), static_cast<unsigned>(e.eofFills), static_cast<unsigned>(e.prevEofFills));
	}
	n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " ");
	n += std::snprintf(out + n, static_cast<std::size_t>(size - n), "fills=%u corrected=%u fallback=%u at_start=%u/%u at_end=%u/%u unknown=%u",
	                      static_cast<unsigned>(r.fills), static_cast<unsigned>(r.corrected), static_cast<unsigned>(r.fallback), static_cast<unsigned>(r.atStart[0]),
	                      static_cast<unsigned>(r.atStart[1]), static_cast<unsigned>(r.atEnd[0]),
	                      static_cast<unsigned>(r.atEnd[1]), static_cast<unsigned>(r.unknown));
	for (int i = 0; i < 32 && n > 0 && n < size; ++i)
		if (r.atStartPos[i]) n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " s%d:%u", i, static_cast<unsigned>(r.atStartPos[i]));
	for (int i = 0; i < 32 && n > 0 && n < size; ++i)
		if (r.atEndPos[i]) n += std::snprintf(out + n, static_cast<std::size_t>(size - n), " e%d:%u", i, static_cast<unsigned>(r.atEndPos[i]));
}

// The svg painter asks before splitting a paint across both cores.
extern "C" bool gea_display_second_core_busy() { return g_scrollCopyInFlight; }

extern "C" void gea_display_scroll_rect_end()
{
	if (!g_scrollCopyInFlight) return;
	// Only the worker copies: from this core, a copy competes with the bounce
	// refills for PSRAM and the panel shows thin glitch lines.
	gea_render_parallel_wait();
	g_scrollCopyInFlight = false;
	const DisplayFlushRect &r = g_asyncScroll.skip;  // each row left the cache as it was copied
	g_swapDiag.copyUs += esp_timer_get_time() - g_asyncScroll.startUs;
	xSemaphoreTake(g_readyMutex, portMAX_DELAY);
	g_swapDiag.patchPx += patchBack(&r, 1);  // the rest of the buffer, outside the viewport
	xSemaphoreGive(g_readyMutex);
	g_canvas.markDirty(r.x0, r.y0, r.x1, r.y1);
	markStale(r.x0, r.y0, r.x1, r.y1);  // see Display::scrollRect
	if (!g_directDrawDuringCopy) g_cleanRect = r;
	// An unstaged draw may have left rows the signatures do not describe.
	else g_rowSigValid[g_drawIndex] = false;
}

extern "C" void gea_display_scroll_rect_rows(const std::uint32_t *sig, int y0, int rows)
{
	g_nextSig = sig;
	g_nextSigY0 = y0;
	g_nextSigRows = rows;
}

// An unstaged replay drew into the framebuffer through the cache: the publish
// must write the scroll viewport's rows back after all.
extern "C" void gea_display_note_direct_draw()
{
	forgetRowSums();
	g_directDrawDuringCopy = true;
	g_cleanRect.x1 = -1;
}

void Display::resetScrollRegion()
{
	if (g_ringScroll) {
		if (g_regionH > 0) {
			ensureBackReady();
			changeRegion(0, 0);
		}
		return;
	}
	replayCanvas().setScrollRegion(0, 0, 0);
}
void Display::strokeRect(int x, int y, int w, int h, std::uint16_t color) { replayCanvas().strokeRect(x, y, w, h, color); }
void Display::fillCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().fillCircle(cx, cy, r, color); }
void Display::strokeCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().strokeCircle(cx, cy, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, std::uint16_t color) { replayCanvas().drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int cx, int cy, int r, int startDeg, int endDeg, std::uint16_t color) { replayCanvas().drawArc(cx, cy, r, startDeg, endDeg, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, std::uint16_t color) { replayCanvas().fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, std::uint16_t color, float scale) { replayCanvas().drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, std::uint16_t color, int fontId) { replayCanvas().drawTextFont(text, x, y, color, fontId); }
void Display::drawTextFontFamily(const char *text, int x, int y, std::uint16_t color, int familyId, int sizePx) { replayCanvas().drawTextFontFamily(text, x, y, color, familyId, sizePx); }
void Display::setPixel(int x, int y, std::uint16_t color) { replayCanvas().fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, std::uint16_t color) { replayCanvas().fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count,
                                         int w, int h, int tl, int tr, int br, int bl,
                                         const std::uint16_t *colors)
{
	replayCanvas().fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lineWidth, std::uint16_t color) { replayCanvas().strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lineWidth, color); }
void Display::blitImage(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy) { replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy); }
void Display::blitImageScaled(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy, int dstWidth, int dstHeight) { replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy, dstWidth, dstHeight); }
void Display::setWorldOverlay(const std::uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}

void Display::flushStatsRead(int64_t *totalUs, int *callCount, int *pixelCount)
{
	if (totalUs) *totalUs = g_stats.totalUs;
	if (callCount) *callCount = g_stats.callCount;
	if (pixelCount) *pixelCount = g_stats.pixelCount;
}

DisplayFlushPerfStats Display::flushPerfStatsRead() { return g_stats; }
void Display::flushStatsReset() { g_stats = {}; }
const char *Display::flushStageName() { return "idle"; }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return {}; }

}  // namespace gea::platform::display

extern "C" bool gea_display_flush_presents_frame() { return g_swapFbs; }

// The ring scroll moves no pixels and publishes the rotation itself, so a scroll
// frame needs to flush only what it repainted -- flushing the whole viewport
// would mark it stale and copy it into the next back buffer.
extern "C" bool gea_display_scroll_rect_self_flushing() { return g_ringScroll; }

// Every framebuffer writer here maps rows through the ring, so content drawn on
// a non-scroll frame lands correctly under a rotated region: the engine keeps
// the region instead of resetting it (and repainting it) on such frames.
extern "C" bool gea_display_scroll_region_persists() { return g_ringScroll; }

extern "C" const std::uint16_t *gea_display_presented_framebuffer(int *width, int *height)
{
	if (width) *width = gea::platform::display::kWidth;
	if (height) *height = gea::platform::display::kHeight;
	return g_framebuffer ? g_framebuffer : g_drawFramebuffer;
}

// Full-screen PSRAM scratch for the static-backdrop cache. Without this hook the
// weak default in ui/render.cpp returns nullptr and the bake SILENTLY never runs —
// an animated-transform scene (css-3d-cube) then full-rebuilds + re-rasterizes the
// whole static stage every frame (~17fps at 480x480) instead of reprojecting over
// a baked backdrop. Same shape as the targets/esp32/display.cpp implementation.
extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	static std::uint16_t *buffer = nullptr;
	static bool attempted = false;
	constexpr int kCapPx = gea::platform::display::kWidth * gea::platform::display::kHeight;
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
