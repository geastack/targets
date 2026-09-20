#include "rp2350_gea_platform.h"

#include <rp2350_panel.h>

#include "app.h"
#include "audio.h"
#include "board.h"
#include "canvas.h"
#include "display.h"
#include "display_present.h"
#include "event.h"
#include "events.h"
#include "host/audio.h"
#include "host/backends.h"
#include "host/display_orientation.h"
#include "host/timers.h"
#include "input.h"
#include "memory.h"
#include "pixel.h"
#include "services/frame_scheduler.h"
#include "services/storage_service.h"
#include "touch.h"
#ifndef GEA_RP2350_HAS_TOUCH
#define GEA_RP2350_HAS_TOUCH 1
#endif
#if GEA_RP2350_HAS_TOUCH
#include "touch/ft6336/ft6336.h"
#endif
#include "ui/document.h"
#include "ui/internal.h"
#include "ui/style.h"
#include "ui/tree_internal.h"

#if __has_include("gea_embedded_app_config.h")
#include "gea_embedded_app_config.h"
#endif

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <new>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include "rp_pico_alloc.h"
}

#ifndef GEA_RP2350_HAS_BUTTONS
#define GEA_RP2350_HAS_BUTTONS 0
#endif

// Opt-in: make the hardware Up/Down buttons scroll the primary scrollable
// container by default (browser-style arrow-key page scroll). Off by default;
// enabled only by boards that want it (e.g. the touchless Tufty 2350).
#ifndef GEA_RP2350_DEFAULT_BUTTON_SCROLL
#define GEA_RP2350_DEFAULT_BUTTON_SCROLL 0
#endif

#ifndef GEA_RP2350_FRAMEBUFFER_IN_SRAM
#define GEA_RP2350_FRAMEBUFFER_IN_SRAM 0
#endif

#ifndef GEA_RP2350_BACKGROUND_CACHE_IN_SRAM
#ifdef GEA_EMBEDDED_DISPLAY_BACKGROUND_CACHE_IN_SRAM
#define GEA_RP2350_BACKGROUND_CACHE_IN_SRAM GEA_EMBEDDED_DISPLAY_BACKGROUND_CACHE_IN_SRAM
#elif defined(GEA_RP2350_DEFAULT_BACKGROUND_CACHE_IN_SRAM)
#define GEA_RP2350_BACKGROUND_CACHE_IN_SRAM GEA_RP2350_DEFAULT_BACKGROUND_CACHE_IN_SRAM
#else
#define GEA_RP2350_BACKGROUND_CACHE_IN_SRAM 0
#endif
#endif

#ifndef GEA_RP2350_DISPLAY_COMMAND_BUFFER_BYTES
#define GEA_RP2350_DISPLAY_COMMAND_BUFFER_BYTES 0
#endif

#ifndef GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS
#ifdef GEA_EMBEDDED_DISPLAY_COMMAND_BUFFER_COMMANDS
#define GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS GEA_EMBEDDED_DISPLAY_COMMAND_BUFFER_COMMANDS
#elif defined(GEA_RP2350_DEFAULT_DISPLAY_COMMAND_BUFFER_COMMANDS)
#define GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS GEA_RP2350_DEFAULT_DISPLAY_COMMAND_BUFFER_COMMANDS
#else
#define GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS 0
#endif
#endif

#ifndef GEA_RP2350_PIE_BLEND
#define GEA_RP2350_PIE_BLEND 0
#endif

#ifndef GEA_RP2350_EARLY_PSRAM_ALLOC
#define GEA_RP2350_EARLY_PSRAM_ALLOC 0
#endif

#ifndef GEA_RP2350_RENDER_CORE1
// Second M33 rasters the bottom half of each present chunk in parallel with the
// first core's top half — the present is raster-bound, not bus-bound, so this is
// the main fps lever. See rasterPresentChunk.
#define GEA_RP2350_RENDER_CORE1 1
#endif

#ifndef GEA_RP2350_RENDER_CORE1_STACK_BYTES
// Dedicated stack: the worker runs present::rasterFrameRows (local Canvas + the
// per-command raster recursion), which needs more than the 2KB pico-sdk default.
#define GEA_RP2350_RENDER_CORE1_STACK_BYTES 8192
#endif

#ifndef GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US
#define GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US 0
#endif

#ifndef GEA_RP2350_SOFTWARE_SCROLL_REGISTER
#define GEA_RP2350_SOFTWARE_SCROLL_REGISTER 1
#endif

// Use a panel's DCS vertical-scroll register (VSCRDEF/VSCSAD) for full-screen
// vertical scrolls: the framebuffer keeps its circular row layout, GRAM mirrors
// it 1:1, and the panel applies the display-order rotation itself — so a scroll
// frame only streams the revealed strip (plus scrollbar/slot damage) instead of
// re-streaming the whole region. Requires GEA_RP2350_SOFTWARE_SCROLL_REGISTER.
// ⚠ MUST STAY OFF for the Waveshare 2.41 RM690B0: this panel IGNORES
// VSCRDEF/VSCSAD (verified 2026-07-02 — the display shows stale content while
// fps soars because nothing streams). That's exactly why the software scroll
// register exists. Only enable on a panel proven to honor 0x33/0x37.
#ifndef GEA_RP2350_PANEL_HW_VSCROLL
#define GEA_RP2350_PANEL_HW_VSCROLL 0
#endif

namespace gea::framework::app::generated {
void drainMicrotasks();
}  // namespace gea::framework::app::generated

namespace {

namespace board = gea::platform::board;
constexpr std::uint32_t kAllocMagic = 0x47454152u;

i2c_inst_t *boardI2c()
{
	return board::i2c.bus == 1 ? i2c1 : i2c0;
}

struct AllocHeader {
	std::uint32_t magic = kAllocMagic;
	std::uint32_t flags = 0;
	std::size_t size = 0;
	std::size_t alignment = 0;
	void *raw = nullptr;
};

bool gUseRpAllocator = false;

struct PoolAllocationStats {
	std::uint32_t allocCount = 0;
	std::uint32_t freeCount = 0;
	std::size_t liveBytes = 0;
	std::size_t peakBytes = 0;
};

PoolAllocationStats gSramAllocStats;
PoolAllocationStats gPsramAllocStats;

PoolAllocationStats &poolStats(bool rp)
{
	return rp ? gPsramAllocStats : gSramAllocStats;
}

void noteAllocation(bool rp, std::size_t size)
{
	auto &stats = poolStats(rp);
	stats.allocCount++;
	stats.liveBytes += size;
	if (stats.liveBytes > stats.peakBytes) stats.peakBytes = stats.liveBytes;
}

void noteFree(bool rp, std::size_t size)
{
	auto &stats = poolStats(rp);
	stats.freeCount++;
	stats.liveBytes = size <= stats.liveBytes ? stats.liveBytes - size : 0;
}

struct RawAllocation {
	void *ptr = nullptr;
	bool rp = false;
};

RawAllocation rawAlloc(std::size_t size)
{
	if (gUseRpAllocator) {
		if (void *ptr = rp_mem_malloc(size)) return {ptr, true};
	}
#if GEA_RP2350_EARLY_PSRAM_ALLOC
	if (void *ptr = rp_mem_malloc(size)) return {ptr, true};
#endif
	return {std::malloc(size), false};
}

void rawFree(void *ptr, bool rp)
{
	if (!ptr) return;
	if (rp) rp_mem_free(ptr);
	else std::free(ptr);
}

void *allocBlock(std::size_t size, std::size_t alignment)
{
	if (size == 0) size = 1;
	if (alignment < alignof(void *)) alignment = alignof(void *);
	const std::size_t total = size + alignment - 1 + sizeof(AllocHeader);
	RawAllocation raw = rawAlloc(total);
	if (!raw.ptr) {
		reset_usb_boot(0, 0);
		std::abort();
	}
	const auto base = reinterpret_cast<std::uintptr_t>(raw.ptr) + sizeof(AllocHeader);
	const auto aligned = (base + alignment - 1) & ~(static_cast<std::uintptr_t>(alignment) - 1);
	auto *header = reinterpret_cast<AllocHeader *>(aligned - sizeof(AllocHeader));
	header->magic = kAllocMagic;
	header->flags = raw.rp ? 1u : 0u;
	header->size = size;
	header->alignment = alignment;
	header->raw = raw.ptr;
	noteAllocation(raw.rp, size);
	return reinterpret_cast<void *>(aligned);
}

AllocHeader *headerFor(void *ptr)
{
	if (!ptr) return nullptr;
	auto *header = reinterpret_cast<AllocHeader *>(reinterpret_cast<std::uintptr_t>(ptr) - sizeof(AllocHeader));
	return header->magic == kAllocMagic ? header : nullptr;
}

void freeBlock(void *ptr) noexcept
{
	if (!ptr) return;
	AllocHeader *header = headerFor(ptr);
	if (!header) {
		std::free(ptr);
		return;
	}
	void *raw = header->raw;
	const bool rp = (header->flags & 1u) != 0;
	const std::size_t size = header->size;
	header->magic = 0;
	noteFree(rp, size);
	rawFree(raw, rp);
}

void *reallocBlock(void *ptr, std::size_t size)
{
	if (!ptr) return allocBlock(size, alignof(std::max_align_t));
	AllocHeader *header = headerFor(ptr);
	if (!header) return std::realloc(ptr, size);
	void *next = allocBlock(size, header->alignment);
	std::memcpy(next, ptr, std::min(size, header->size));
	freeBlock(ptr);
	return next;
}

using Pixel = gea::framework::graphics::pixel::native_t;
namespace present = gea::framework::display_present;
gea::framework::graphics::Canvas gDisplayCanvas;
#if GEA_RP2350_RENDER_CORE1
gea::framework::graphics::Canvas gRenderWorkerCanvas;
#endif
#if GEA_RP2350_FRAMEBUFFER_IN_SRAM
alignas(4) Pixel gSramFramebuffer[gea::rp2350::kPanelWidth * gea::rp2350::kPanelHeight]
    __attribute__((section(".uninitialized_data")));
#endif
#if GEA_RP2350_BACKGROUND_CACHE_IN_SRAM
alignas(4) Pixel gSramBackgroundCache[gea::rp2350::kPanelWidth * gea::rp2350::kPanelHeight]
    __attribute__((section(".uninitialized_data")));
#endif
#if GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS > 0
alignas(gea::embedded::ui::DisplayCommand) gea::embedded::ui::DisplayCommand
    gSramDisplayCommandBuffer[GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS]
    __attribute__((section(".uninitialized_data")));
#elif GEA_RP2350_DISPLAY_COMMAND_BUFFER_BYTES > 0
alignas(16) std::uint8_t gSramDisplayCommandBuffer[GEA_RP2350_DISPLAY_COMMAND_BUFFER_BYTES]
    __attribute__((section(".uninitialized_data")));
#endif
Pixel *gFramebuffer = nullptr;
std::uint8_t gDisplayAlpha = 255;
int gFlushRows = 64;
int gFlushDepth = 2;
int gFrameIntervalMs = gea::framework::services::FrameScheduler::kDefaultFrameIntervalMs;
int gFlushCalls = 0;
int gFlushPixels = 0;
int gBrightness = 100;
bool gVsyncDriven = false;
bool gInvalidated = true;
present::Frame gPreviousPresentFrame;
bool gPreviousPresentValid = false;
// Present render-scale (1 = native, 2 = app renders a half-panel viewport and the
// present 2x-upscales each chunk). gea3d opts in via Display::setPresentScale(2)
// when it renders into a width/2 x height/2 viewport — half the fill/overdraw,
// full-res out. Matches the ESP32 backend's presentScale_ behaviour.
int gPresentScale = 1;
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
int gScrollRegionY = 0;
int gScrollRegionH = 0;
int gScrollOffsetY = 0;
#endif

#if GEA_RP2350_RENDER_CORE1
struct RenderWorkerTask {
	void (*fn)(void *, int, int) = nullptr;
	void *ctx = nullptr;
	int y0 = 0;
	int y1 = -1;
};

constexpr std::uint32_t kRenderWorkerRun = 0x47455231u;   // "GER1"
constexpr std::uint32_t kRenderWorkerDone = 0x47455244u;  // "GERD"
RenderWorkerTask gRenderWorkerTask;
volatile bool gRenderWorkerReady = false;
volatile bool gRenderWorkerInFlight = false;
#if GEA_RP2350_RENDER_CORE1_STACK_BYTES > 0
static_assert((GEA_RP2350_RENDER_CORE1_STACK_BYTES % sizeof(std::uint32_t)) == 0,
              "Core1 render stack size must be a multiple of 4 bytes");
alignas(8) std::uint32_t gRenderWorkerStack[GEA_RP2350_RENDER_CORE1_STACK_BYTES / sizeof(std::uint32_t)]
    __attribute__((section(".uninitialized_data")));
#endif
#endif

bool onRenderWorkerCore()
{
#if GEA_RP2350_RENDER_CORE1
	return get_core_num() == 1;
#else
	return false;
#endif
}

gea::framework::graphics::Canvas &activeDisplayCanvas()
{
#if GEA_RP2350_RENDER_CORE1
	if (onRenderWorkerCore()) return gRenderWorkerCanvas;
#endif
	return gDisplayCanvas;
}

#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
void setScrollRegionAll(int regionY, int regionH, int scrollOffsetY)
{
	gScrollRegionY = regionY;
	gScrollRegionH = regionH;
	gScrollOffsetY = scrollOffsetY;
	gDisplayCanvas.setScrollRegion(regionY, regionH, scrollOffsetY);
#if GEA_RP2350_RENDER_CORE1
	gRenderWorkerCanvas.setScrollRegion(regionY, regionH, scrollOffsetY);
#endif
#if GEA_RP2350_PANEL_HW_VSCROLL
	// Full-screen regions ride the panel's scroll register; partial-height
	// regions keep VSCSAD at identity and flush through the software remap.
	if (regionH > 0 && regionY == 0 && regionH == gea::rp2350::kPanelHeight) {
		gea::rp2350::panelSetVerticalScroll(scrollOffsetY);
	} else {
		gea::rp2350::panelSetVerticalScroll(0);
	}
#endif
}
#endif

#if GEA_RP2350_RENDER_CORE1
void bindRenderWorkerCanvas()
{
	if (gFramebuffer) {
		gRenderWorkerCanvas.bindPixels(gFramebuffer, gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
		gRenderWorkerCanvas.setScrollRegion(gScrollRegionY, gScrollRegionH, gScrollOffsetY);
#endif
		gRenderWorkerCanvas.resetDirty();
		gRenderWorkerCanvas.resetClip();
		gRenderWorkerCanvas.setGlobalAlpha(255);
	}
}

void renderWorkerMain()
{
	bindRenderWorkerCanvas();
	__dmb();
	gRenderWorkerReady = true;
	while (true) {
		const std::uint32_t cmd = multicore_fifo_pop_blocking();
		if (cmd != kRenderWorkerRun) continue;
		__dmb();
		bindRenderWorkerCanvas();
		RenderWorkerTask task = gRenderWorkerTask;
		if (task.fn && task.y0 <= task.y1) {
			task.fn(task.ctx, task.y0, task.y1);
		}
		__dmb();
		multicore_fifo_push_blocking(kRenderWorkerDone);
	}
}

void startRenderWorker()
{
	static bool launched = false;
	if (launched) return;
	launched = true;
#if GEA_RP2350_RENDER_CORE1_STACK_BYTES > 0
	multicore_launch_core1_with_stack(renderWorkerMain, gRenderWorkerStack, sizeof(gRenderWorkerStack));
#else
	multicore_launch_core1(renderWorkerMain);
#endif
	for (int i = 0; i < 200000 && !gRenderWorkerReady; ++i) {
		tight_loop_contents();
	}
}
#endif

struct FlushWindow {
	int x0 = 0;
	int y0 = 0;
	int x1 = 0;
	int y1 = 0;
};

constexpr int kMaxCollectedWindows = 64;
constexpr int kWindowMergeGapShift = 2;
constexpr int kDenseWindowMergeThresholdPercent = 75;

struct ScheduledTimer {
	double id = 0;
	gea::host::TimerCallback callback;
	double delayMs = 0;
	double nextFireMs = 0;
	bool interval = false;
	bool active = false;
};

std::array<ScheduledTimer, 16> gTimers{};
std::array<gea::host::AnimationFrameCallback, 16> gPendingRaf{};
std::array<gea::host::AnimationFrameCallback, 16> gActiveRaf{};
int gPendingRafCount = 0;
double gNextTimerId = 1;
double gLastTimestampMs = 0;

std::array<gea::framework::events::Event, gea::framework::services::FrameScheduler::kEventQueueDepth> gEvents{};
int gEventHead = 0;
int gEventTail = 0;
bool gEventQueueStarted = false;

#if GEA_RP2350_HAS_BUTTONS
std::array<bool, board::buttons.size()> gButtonDown{};
// Next time (µs since boot) a held auto-repeat button re-fires its keydown.
std::array<std::uint64_t, board::buttons.size()> gButtonNextRepeatUs{};

void dispatchKeyInput()
{
	auto &tree = gea::embedded::ui::Tree::instance();
	for (int keyCode = gea::framework::input::consumeKeyDown(); keyCode != 0;
	     keyCode = gea::framework::input::consumeKeyDown()) {
		gea::framework::events::PointerEvent event{};
		event.type = gea::framework::events::PointerEventType::KeyDown;
		event.keyCode = keyCode;
		event.bubbles = true;
		event.cancelable = true;

		const int inputId = tree.activeInputId();
		bool stopped = false;
		if (inputId >= 0) {
			event.targetId = inputId;
			event.currentTargetId = inputId;
			tree.dispatchEvent(event);
			stopped = event.propagationStopped;
		}
		if (!stopped) {
			event.propagationStopped = false;
			gea::embedded::ui::dispatchDocumentKeyDown(event);
		}

#if GEA_RP2350_DEFAULT_BUTTON_SCROLL
		// Default action for the hardware Up/Down buttons on this touchless
		// board: scroll the primary scrollable container, mirroring a browser's
		// arrow-key page scroll. Skipped when a text input is focused (arrows
		// move the caret there) or the app consumed the key via
		// preventDefault().
		constexpr int kKeyCodeArrowUp = 38;    // web keyCode for ArrowUp
		constexpr int kKeyCodeArrowDown = 40;  // web keyCode for ArrowDown
		constexpr int kButtonScrollStepPx = 12;
		if (inputId < 0 && !event.defaultPrevented) {
			if (keyCode == kKeyCodeArrowDown) {
				tree.scrollByKeyStep(kButtonScrollStepPx);
			} else if (keyCode == kKeyCodeArrowUp) {
				tree.scrollByKeyStep(-kButtonScrollStepPx);
			}
		}
#endif
	}
}
#endif

#if GEA_RP2350_HAS_TOUCH
class Ft6336Bus final : public gea::chips::ft6336::RegisterBus {
public:
	bool writeRegister(std::uint8_t reg, std::uint8_t value) override
	{
		const std::uint8_t data[2] = {reg, value};
		return i2c_write_blocking(boardI2c(), board::touch.address, data, 2, false) == 2;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override
	{
		if (i2c_write_blocking(boardI2c(), board::touch.address, &reg, 1, true) != 1) return false;
		return i2c_read_blocking(boardI2c(), board::touch.address, &value, 1, false) == 1;
	}

	bool readRegisters(std::uint8_t reg, std::uint8_t *data, std::size_t length) override
	{
		if (!data || length == 0) return false;
		if (i2c_write_blocking(boardI2c(), board::touch.address, &reg, 1, true) != 1) return false;
		return i2c_read_blocking(boardI2c(), board::touch.address, data, length, false) ==
		       static_cast<int>(length);
	}
};

Ft6336Bus gTouchBus;
gea::chips::ft6336::ControllerCore gTouchCore;
gea::platform::touch::Touchscreen::Observer gTouchObserver = nullptr;
bool gTouchInitialized = false;
bool gTouching = false;
int gTouchX = 0;
int gTouchY = 0;

// Synthetic drag driven by GEADEV DRAG: a per-frame state machine advanced by
// pollTouch() that feeds gTouchObserver the same Down/Move/Up sequence a
// finger would, at delayUs cadence. While active, hardware samples are
// ignored so a benchmark run can't be disturbed by an accidental touch.
struct SyntheticDrag {
	bool active = false;
	std::uint64_t startUs = 0;
	std::uint64_t delayUs = 0;
	int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	int steps = 0;
	int lastStep = -1;
};
SyntheticDrag gSyntheticDrag;

void pollSyntheticDrag()
{
	SyntheticDrag &drag = gSyntheticDrag;
	const std::uint64_t elapsedUs = time_us_64() - drag.startUs;
	const int step = drag.delayUs > 0 ? static_cast<int>(elapsedUs / drag.delayUs) : drag.steps + 1;
	if (drag.lastStep < 0) {
		gTouchObserver(gea::platform::touch::Phase::Down, true, drag.x1, drag.y1);
		gTouching = true;
		gTouchX = drag.x1;
		gTouchY = drag.y1;
		drag.lastStep = 0;
		return;
	}
	if (step > drag.steps) {
		gTouchObserver(gea::platform::touch::Phase::Up, false, gTouchX, gTouchY);
		gTouching = false;
		drag.active = false;
		std::printf("GEADEV:OK DRAG x1=%d y1=%d x2=%d y2=%d steps=%d\n",
		            drag.x1, drag.y1, drag.x2, drag.y2, drag.steps);
		std::fflush(stdout);
		return;
	}
	if (step > drag.lastStep) {
		const int clamped = std::min(step, drag.steps);
		const int x = drag.x1 + ((drag.x2 - drag.x1) * clamped) / drag.steps;
		const int y = drag.y1 + ((drag.y2 - drag.y1) * clamped) / drag.steps;
		if (x != gTouchX || y != gTouchY) {
			gTouchObserver(gea::platform::touch::Phase::Move, true, x, y);
			gTouchX = x;
			gTouchY = y;
		}
		drag.lastStep = step;
	}
}
#endif

void flushRectClipped(int x0, int y0, int x1, int y1)
{
	if (!gFramebuffer) return;
	x0 = std::max(0, x0);
	y0 = std::max(0, y0);
	x1 = std::min(gea::rp2350::kPanelWidth - 1, x1);
	y1 = std::min(gea::rp2350::kPanelHeight - 1, y1);
	if (x0 > x1 || y0 > y1) return;
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
	if (gDisplayCanvas.scrollRegionH() > 0) {
#if GEA_RP2350_PANEL_HW_VSCROLL
		if (gScrollRegionY == 0 && gScrollRegionH == gea::rp2350::kPanelHeight) {
			// GRAM mirrors the framebuffer's circular layout and the panel
			// rotates scanout via VSCSAD, so flush each logical rect as its
			// contiguous physical row spans (≤2: a rect crossing the wrap).
			int y = y0;
			while (y <= y1) {
				const int physY = gDisplayCanvas.rowToPhysical(y);
				int run = 1;
				while (y + run <= y1 && gDisplayCanvas.rowToPhysical(y + run) == physY + run) run++;
				gea::rp2350::panelFlushRect(gFramebuffer, gea::rp2350::kPanelWidth, x0, physY, x1, physY + run - 1);
				y += run;
			}
			gFlushCalls++;
			gFlushPixels += (x1 - x0 + 1) * (y1 - y0 + 1);
			return;
		}
#endif
		if (x0 == 0 && x1 == gea::rp2350::kPanelWidth - 1) {
			// Full-width flush of a circularly remapped region: the display-order
			// rows form contiguous framebuffer spans — DMA them straight from
			// PSRAM (panelFlushFullWidthSpans), no per-row CPU copies. Round rows
			// to the panel's window alignment first (extra rows are clean pixels).
			y0 &= ~1;
			y1 = std::min(gea::rp2350::kPanelHeight - 1, y1 | 1);
			int y = y0;
			while (y <= y1) {
				const int phys0 = gDisplayCanvas.rowToPhysical(y);
				int run0 = 1;
				while (y + run0 <= y1 && gDisplayCanvas.rowToPhysical(y + run0) == phys0 + run0) run0++;
				const int yNext = y + run0;
				int run1 = 0;
				int phys1 = 0;
				if (yNext <= y1) {
					phys1 = gDisplayCanvas.rowToPhysical(yNext);
					run1 = 1;
					while (yNext + run1 <= y1 && gDisplayCanvas.rowToPhysical(yNext + run1) == phys1 + run1) run1++;
				}
				gea::rp2350::panelFlushFullWidthSpans(
				    gFramebuffer + static_cast<std::size_t>(phys0) * gea::rp2350::kPanelWidth,
				    run0,
				    run1 > 0 ? gFramebuffer + static_cast<std::size_t>(phys1) * gea::rp2350::kPanelWidth : nullptr,
				    run1,
				    y);
				y = yNext + run1;
			}
			gFlushCalls++;
			gFlushPixels += (x1 - x0 + 1) * (y1 - y0 + 1);
			return;
		}
		gea::rp2350::panelStreamRect(x0,
		                             y0,
		                             x1,
		                             y1,
		                             [](std::uint16_t *dst, int width, int height, int originX, int originY, void *) {
			                             if (!gFramebuffer) return;
			                             for (int row = 0; row < height; ++row) {
				                             const int physicalY = gDisplayCanvas.rowToPhysical(originY + row);
				                             std::copy_n(gFramebuffer + static_cast<std::size_t>(physicalY) * gea::rp2350::kPanelWidth + originX,
				                                         width,
				                                         dst + static_cast<std::size_t>(row) * width);
			                             }
		                             },
		                             nullptr);
	} else
#endif
	gea::rp2350::panelFlushRect(gFramebuffer, gea::rp2350::kPanelWidth, x0, y0, x1, y1);
	gFlushCalls++;
	gFlushPixels += (x1 - x0 + 1) * (y1 - y0 + 1);
}

int flushWindowArea(const FlushWindow &window)
{
	if (window.x0 > window.x1 || window.y0 > window.y1) return 0;
	return (window.x1 - window.x0 + 1) * (window.y1 - window.y0 + 1);
}

FlushWindow uniteFlushWindows(const FlushWindow &a, const FlushWindow &b)
{
	if (flushWindowArea(a) <= 0) return b;
	if (flushWindowArea(b) <= 0) return a;
	return {std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1)};
}

bool flushWindowsOverlapOrTouch(const FlushWindow &a, const FlushWindow &b)
{
	return a.x0 <= b.x1 + 1 && a.x1 + 1 >= b.x0 && a.y0 <= b.y1 + 1 && a.y1 + 1 >= b.y0;
}

FlushWindow alignedFlushWindow(const gea::platform::display::DisplayFlushRect &rect)
{
	const present::Rect aligned = present::clampAndAlign({rect.x0, rect.y0, rect.x1, rect.y1},
	                                                     gea::rp2350::kPanelWidth,
	                                                     gea::rp2350::kPanelHeight);
	return {aligned.x0, aligned.y0, aligned.x1, aligned.y1};
}

int collectFlushWindows(const gea::platform::display::DisplayFlushRect *rects,
                        int count,
                        std::array<FlushWindow, kMaxCollectedWindows> &windows)
{
	int windowCount = 0;
	for (int i = 0; i < count; ++i) {
		FlushWindow window = alignedFlushWindow(rects[i]);
		if (flushWindowArea(window) <= 0) continue;

		bool merged = false;
		for (int existing = 0; existing < windowCount; ++existing) {
			if (!flushWindowsOverlapOrTouch(windows[static_cast<std::size_t>(existing)], window)) continue;
			windows[static_cast<std::size_t>(existing)] =
			    uniteFlushWindows(windows[static_cast<std::size_t>(existing)], window);
			merged = true;
			break;
		}
		if (merged) continue;

		if (windowCount < static_cast<int>(windows.size())) {
			windows[static_cast<std::size_t>(windowCount++)] = window;
			continue;
		}

		int best = 0;
		int bestCost = 0x7fffffff;
		for (int existing = 0; existing < windowCount; ++existing) {
			const FlushWindow united = uniteFlushWindows(windows[static_cast<std::size_t>(existing)], window);
			const int cost = flushWindowArea(united) - flushWindowArea(windows[static_cast<std::size_t>(existing)]);
			if (cost < bestCost) {
				bestCost = cost;
				best = existing;
			}
		}
		windows[static_cast<std::size_t>(best)] = uniteFlushWindows(windows[static_cast<std::size_t>(best)], window);
	}

	bool changed = true;
	while (changed) {
		changed = false;
		for (int i = 0; i < windowCount && !changed; ++i) {
			for (int j = i + 1; j < windowCount; ++j) {
				const int areaI = flushWindowArea(windows[static_cast<std::size_t>(i)]);
				const int areaJ = flushWindowArea(windows[static_cast<std::size_t>(j)]);
				const FlushWindow united =
				    uniteFlushWindows(windows[static_cast<std::size_t>(i)], windows[static_cast<std::size_t>(j)]);
				const int extra = flushWindowArea(united) - areaI - areaJ;
				const int allowance = std::min(areaI, areaJ) >> kWindowMergeGapShift;
				if (extra > allowance) continue;
				windows[static_cast<std::size_t>(i)] = united;
				windows[static_cast<std::size_t>(j)] = windows[static_cast<std::size_t>(--windowCount)];
				changed = true;
				break;
			}
		}
	}

	if (windowCount >= 2) {
		FlushWindow bounds = windows[0];
		int areaSum = 0;
		for (int i = 0; i < windowCount; ++i) {
			bounds = uniteFlushWindows(bounds, windows[static_cast<std::size_t>(i)]);
			areaSum += flushWindowArea(windows[static_cast<std::size_t>(i)]);
		}
		const int boundsArea = flushWindowArea(bounds);
		if (boundsArea > 0 &&
		    areaSum * 100 >= boundsArea * kDenseWindowMergeThresholdPercent) {
			windows[0] = bounds;
			windowCount = 1;
		} else {
			const FlushWindow band{0, bounds.y0, gea::rp2350::kPanelWidth - 1, bounds.y1};
			const int bandArea = flushWindowArea(band);
			if (bandArea > 0 && bandArea <= areaSum * 3) {
				windows[0] = band;
				windowCount = 1;
			}
		}
	}

	std::sort(windows.begin(), windows.begin() + windowCount, [](const FlushWindow &a, const FlushWindow &b) {
		if (a.y0 != b.y0) return a.y0 < b.y0;
		if (a.x0 != b.x0) return a.x0 < b.x0;
		if (a.y1 != b.y1) return a.y1 < b.y1;
		return a.x1 < b.x1;
	});
	return windowCount;
}

void setGlobalAlpha(std::uint8_t alpha)
{
	gDisplayAlpha = alpha;
	gDisplayCanvas.setGlobalAlpha(alpha);
}

struct PresentRasterContext {
	const present::Frame *frame = nullptr;
};

// Half-res upscale scratch, one bank per core so core 0 (top rows) and core 1
// (bottom rows) never share it. Sized for a full DMA chunk (64 rows) at half-res.
constexpr int kMaxHalfChunkRows = 64 / 2 + 2;
alignas(4) std::uint16_t gUpscaleScratch[2][(gea::rp2350::kPanelWidth / 2 + 2) * kMaxHalfChunkRows];

struct ChunkRasterJob {
	const present::Frame *frame = nullptr;
	std::uint16_t *pixels = nullptr;
	int width = 0;
	int height = 0;
	int originX = 0;
	int originY = 0;
};

// Rasters chunk rows [ry0, ry1] (inclusive, chunk-local). Safe to run on either
// core: it reads the shared read-only Frame and writes only its own rows of
// `pixels` and its own per-core scratch bank.
void rasterChunkRows(const ChunkRasterJob &job, int ry0, int ry1)
{
	if (ry1 < ry0) return;
	const int rows = ry1 - ry0 + 1;
	if (gPresentScale == 2) {
		// The frame's commands are in half-panel (viewport) coordinates. Raster the
		// half-res source region covering these rows into the scratch, then
		// nearest-2x-expand it into the full-res chunk that streams to the panel.
		const int hx0 = job.originX / 2;
		const int hy0 = (job.originY + ry0) / 2;
		const int hx1 = (job.originX + job.width - 1) / 2;
		const int hy1 = (job.originY + ry1) / 2;
		const int hw = hx1 - hx0 + 1;
		const int hh = hy1 - hy0 + 1;
		std::uint16_t *scratch = gUpscaleScratch[onRenderWorkerCore() ? 1 : 0];
		present::rasterFrameRows(scratch, hx0, hw, hy0, hh,
		                         gea::rp2350::kPanelHeight / 2, *job.frame);
		for (int ly = ry0; ly <= ry1; ++ly) {
			const int sy = (job.originY + ly) / 2 - hy0;
			const std::uint16_t *srow = scratch + static_cast<std::size_t>(sy) * hw;
			std::uint16_t *drow = job.pixels + static_cast<std::size_t>(ly) * job.width;
			for (int lx = 0; lx < job.width; ++lx) {
				drow[lx] = srow[(job.originX + lx) / 2 - hx0];
			}
		}
		return;
	}
	present::rasterFrameRows(job.pixels + static_cast<std::size_t>(ry0) * job.width,
	                         job.originX, job.width, job.originY + ry0, rows,
	                         gea::rp2350::kPanelHeight, *job.frame);
}

#if GEA_RP2350_RENDER_CORE1
void chunkRowsTrampoline(void *ctx, int y0, int y1)
{
	rasterChunkRows(*static_cast<const ChunkRasterJob *>(ctx), y0, y1);
}
#endif

void rasterPresentChunk(std::uint16_t *pixels, int width, int height, int originX, int originY, void *user)
{
	auto *context = static_cast<PresentRasterContext *>(user);
	if (!context || !context->frame) return;
	const ChunkRasterJob job{context->frame, pixels, width, height, originX, originY};
#if GEA_RP2350_RENDER_CORE1
	// Split the chunk: core 1 rasters the bottom half while this core does the top.
	// The FIFO handshake only pays off for chunks tall enough to halve; `job` lives
	// on this core's stack and stays valid because we block here until core 1 is done.
	if (gRenderWorkerReady && height >= 16) {
		const int mid = height / 2;
		gRenderWorkerTask.fn = chunkRowsTrampoline;
		gRenderWorkerTask.ctx = const_cast<ChunkRasterJob *>(&job);
		gRenderWorkerTask.y0 = mid;
		gRenderWorkerTask.y1 = height - 1;
		__dmb();
		multicore_fifo_push_blocking(kRenderWorkerRun);
		rasterChunkRows(job, 0, mid - 1);
		while (multicore_fifo_pop_blocking() != kRenderWorkerDone) {
		}
		__dmb();
		return;
	}
#endif
	rasterChunkRows(job, 0, height - 1);
}

bool presentDirectToPanel(const gea::platform::display::DisplayPresentCommand *commands, int commandCount)
{
#if GEA_RP2350_PANEL_HW_VSCROLL
	// With the panel scroll register engaged, GRAM rows are circularly offset
	// from logical rows; the direct present rasters at logical coordinates, so
	// route those frames through the framebuffer path instead.
	if (gDisplayCanvas.scrollRegionH() > 0 && gScrollOffsetY != 0) return false;
#endif
#if GEA_EMBEDDED_TTF_RUNTIME_FONTS
	for (int i = 0; i < commandCount; ++i) {
		if (commands[i].type == gea::platform::display::DisplayPresentCommandType::FillText) {
			return false;
		}
	}
#endif
	present::Frame current;
	if (!present::extractFrame(commands, commandCount, current)) return false;

	std::array<present::Rect, kMaxCollectedWindows> dirty{};
	int dirtyCount = 0;
	if (gInvalidated || gPresentScale == 2) {
		// scale==2: frame commands are half-res, so a command-space diff can't map to
		// full-res panel windows — always repaint the whole panel (upscaled). For a
		// full-screen 3D flythrough every frame is fully dirty anyway.
		dirty[0] = present::fullScreen(gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
		dirtyCount = 1;
	} else {
		dirtyCount = present::dirtyRects(gPreviousPresentValid ? &gPreviousPresentFrame : nullptr,
		                                 current,
		                                 dirty.data(),
		                                 static_cast<int>(dirty.size()),
		                                 gea::rp2350::kPanelWidth,
		                                 gea::rp2350::kPanelHeight);
	}

	if (dirtyCount > 0) {
		std::array<gea::platform::display::DisplayFlushRect, kMaxCollectedWindows> rects{};
		for (int i = 0; i < dirtyCount; ++i) {
			rects[static_cast<std::size_t>(i)] = {dirty[static_cast<std::size_t>(i)].x0,
			                                     dirty[static_cast<std::size_t>(i)].y0,
			                                     dirty[static_cast<std::size_t>(i)].x1,
			                                     dirty[static_cast<std::size_t>(i)].y1};
		}

		std::array<FlushWindow, kMaxCollectedWindows> windows{};
		const int mergedCount = collectFlushWindows(rects.data(), dirtyCount, windows);
		PresentRasterContext context{&current};
		for (int i = 0; i < mergedCount; ++i) {
			const FlushWindow &window = windows[static_cast<std::size_t>(i)];
			gea::rp2350::panelStreamRect(window.x0,
			                             window.y0,
			                             window.x1,
			                             window.y1,
			                             rasterPresentChunk,
			                             &context);
			gFlushCalls++;
			gFlushPixels += flushWindowArea(window);
		}
	}

	gPreviousPresentFrame = std::move(current);
	gPreviousPresentValid = true;
	gDisplayCanvas.resetDirty();
	gDisplayCanvas.setGlobalAlpha(gDisplayAlpha);
	gInvalidated = false;
	return true;
}

}  // namespace

extern "C" gea::framework::graphics::pixel::native_t *gea_bg_cache(int *cap_px)
{
#if GEA_RP2350_BACKGROUND_CACHE_IN_SRAM
	if (cap_px) *cap_px = gea::rp2350::kPanelWidth * gea::rp2350::kPanelHeight;
	return gSramBackgroundCache;
#else
	if (cap_px) *cap_px = 0;
	return nullptr;
#endif
}

extern "C" void *gea_display_command_buffer(int *cap_commands, int command_size, int command_align)
{
#if GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS > 0
	if (command_size != static_cast<int>(sizeof(gea::embedded::ui::DisplayCommand)) ||
	    command_align > static_cast<int>(alignof(gea::embedded::ui::DisplayCommand))) {
		if (cap_commands) *cap_commands = 0;
		return nullptr;
	}
	if (cap_commands) *cap_commands = GEA_RP2350_DISPLAY_COMMAND_BUFFER_COMMANDS;
	return gSramDisplayCommandBuffer;
#elif GEA_RP2350_DISPLAY_COMMAND_BUFFER_BYTES > 0
	if (command_size <= 0) {
		if (cap_commands) *cap_commands = 0;
		return nullptr;
	}
	if (command_align <= 0) command_align = 1;
	const std::uintptr_t raw = reinterpret_cast<std::uintptr_t>(gSramDisplayCommandBuffer);
	const std::uintptr_t aligned = (raw + static_cast<std::uintptr_t>(command_align - 1)) &
	                               ~static_cast<std::uintptr_t>(command_align - 1);
	const std::size_t offset = static_cast<std::size_t>(aligned - raw);
	const std::size_t usable = offset < sizeof(gSramDisplayCommandBuffer) ? sizeof(gSramDisplayCommandBuffer) - offset : 0;
	if (cap_commands) *cap_commands = static_cast<int>(usable / static_cast<std::size_t>(command_size));
	return usable >= static_cast<std::size_t>(command_size) ? reinterpret_cast<void *>(aligned) : nullptr;
#else
	(void)command_size;
	(void)command_align;
	if (cap_commands) *cap_commands = 0;
	return nullptr;
#endif
}

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
#if GEA_RP2350_RENDER_CORE1
	if (!fn || y0 > y1 || !gRenderWorkerReady || gRenderWorkerInFlight || onRenderWorkerCore())
		return false;
	gRenderWorkerTask = {fn, ctx, y0, y1};
	__dmb();
	gRenderWorkerInFlight = true;
	multicore_fifo_push_blocking(kRenderWorkerRun);
	return true;
#else
	(void)fn;
	(void)ctx;
	(void)y0;
	(void)y1;
	return false;
#endif
}

extern "C" void gea_render_parallel_wait()
{
#if GEA_RP2350_RENDER_CORE1
	if (!gRenderWorkerInFlight)
		return;
#if GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US > 0
	const std::uint64_t startUs = time_us_64();
	while (true)
	{
		if (multicore_fifo_rvalid() && multicore_fifo_pop_blocking() == kRenderWorkerDone)
			break;
		if (time_us_64() - startUs >= GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US)
		{
			gRenderWorkerReady = false;
			gRenderWorkerInFlight = false;
			return;
		}
		tight_loop_contents();
	}
#else
	while (multicore_fifo_pop_blocking() != kRenderWorkerDone)
	{
	}
#endif
	__dmb();
	gRenderWorkerInFlight = false;
#endif
}

extern "C" void gea_render_parallel_merge_dirty()
{
#if GEA_RP2350_RENDER_CORE1
	int x0 = 0, y0 = 0, x1 = -1, y1 = -1;
	if (gRenderWorkerCanvas.dirty(&x0, &y0, &x1, &y1)) {
		gDisplayCanvas.markDirty(x0, y0, x1, y1);
		gRenderWorkerCanvas.resetDirty();
	}
#endif
}

extern "C" int gea_current_render_core()
{
#if GEA_RP2350_RENDER_CORE1
	return get_core_num();
#else
	return 0;
#endif
}

extern "C" int gea_render_parallel_main_share_permille()
{
#if GEA_RP2350_RENDER_CORE1
	return 500;
#else
	return 500;
#endif
}

extern "C" bool gea_pie_blend_available()
{
	return false;
}

extern "C" bool gea_rgb565_lut_blend_available()
{
	return GEA_RP2350_PIE_BLEND != 0;
}

extern "C" bool gea_rgb565_lut_blend_const_alpha_available()
{
	return GEA_RP2350_PIE_BLEND != 0;
}

extern "C" void __not_in_flash_func(gea_rgb565_lut_blend_const_alpha_span)(std::uint16_t *dst,
                                                                            const std::uint16_t *colorRgb565,
                                                                            std::int16_t a5,
                                                                            std::int32_t bucketFx,
                                                                            std::int32_t dBucketFx,
                                                                            int count)
{
#if GEA_RP2350_PIE_BLEND
	if (count <= 0)
		return;
	const std::uint32_t av = static_cast<std::uint32_t>(a5);
	if (av == 0)
		return;
	auto swap16 = [](std::uint16_t v) -> std::uint16_t {
#if GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
		return __builtin_bswap16(v);
#else
		return v;
#endif
	};
	auto clampBucket = [](std::int32_t fx) -> int {
		int bucket = fx >> 16;
		if (bucket < 0)
			return 0;
		if (bucket > 250)
			return 250;
		return bucket;
	};
	auto splitInsideBuckets = [](std::int32_t fx0, std::int32_t step32, int n, int &insideStart, int &insideEnd) {
		insideStart = 0;
		insideEnd = 0;
		if (n <= 0)
			return;
		constexpr std::int64_t kBucketHi = 251LL << 16;
		const std::int64_t start = fx0;
		const std::int64_t step = step32;
		const std::int64_t last = start + static_cast<std::int64_t>(n - 1) * step;
		if (step == 0) {
			if (start >= 0 && start < kBucketHi)
				insideEnd = n;
			return;
		}
		auto ceilDivPositive = [](std::int64_t a, std::int64_t b) -> std::int64_t {
			return (a + b - 1) / b;
		};
		std::int64_t first = 0;
		std::int64_t end = n;
		if (step > 0) {
			if (start < 0)
				first = ceilDivPositive(-start, step);
			if (first >= n)
				return;
			if (start >= kBucketHi)
				end = 0;
			else if (last >= kBucketHi)
				end = (kBucketHi - 1 - start) / step + 1;
		} else {
			const std::int64_t down = -step;
			if (start >= kBucketHi)
				first = ceilDivPositive(start - (kBucketHi - 1), down);
			if (first >= n)
				return;
			if (start < 0)
				end = 0;
			else if (last < 0)
				end = start / down + 1;
		}
		if (end > n)
			end = n;
		if (end <= first)
			return;
		insideStart = static_cast<int>(first);
		insideEnd = static_cast<int>(end);
	};
		auto advanceFx = [](std::int32_t fx, std::int32_t step, int n) -> std::int32_t {
			return static_cast<std::int32_t>(static_cast<std::int64_t>(fx) + static_cast<std::int64_t>(step) * n);
		};
		auto halfBlendRgb565 = [](std::uint16_t fg, std::uint16_t bg) -> std::uint16_t {
			const std::uint32_t x = static_cast<std::uint32_t>(fg ^ bg);
			return static_cast<std::uint16_t>((fg & bg) + ((x & 0xF7DEu) >> 1) + (x & 0x0821u));
		};
		auto halfBlendRgb565Pair = [](std::uint32_t fg, std::uint32_t bg) -> std::uint32_t {
			const std::uint32_t x = fg ^ bg;
			return ((fg & bg) + ((x & 0xF7DEF7DEu) >> 1) + (x & 0x08210821u));
		};
		auto swap16Pair = [](std::uint32_t v) -> std::uint32_t {
#if GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
			return ((v & 0x00FF00FFu) << 8) | ((v & 0xFF00FF00u) >> 8);
#else
			return v;
#endif
		};
		auto loadPairNormal = [&](const std::uint16_t *p) -> std::uint32_t {
			std::uint32_t stored = 0;
			std::memcpy(&stored, p, sizeof(stored));
			return swap16Pair(stored);
		};
		auto storePairNormal = [&](std::uint16_t *p, std::uint32_t normal) {
			const std::uint32_t stored = swap16Pair(normal);
			std::memcpy(p, &stored, sizeof(stored));
		};
		auto halfBlend = [&](std::uint16_t fc, std::uint16_t storedDst) -> std::uint16_t {
			return swap16(halfBlendRgb565(fc, swap16(storedDst)));
		};
		auto paintHalfConst = [&](int bucket, int n) {
			const std::uint16_t fc = colorRgb565[bucket];
			if ((reinterpret_cast<std::uintptr_t>(dst) & 2u) && n > 0) {
				*dst = halfBlend(fc, *dst);
				dst++;
				n--;
			}
			const std::uint32_t fgPair = static_cast<std::uint32_t>(fc) |
			                             (static_cast<std::uint32_t>(fc) << 16);
			while (n >= 4) {
				storePairNormal(dst, halfBlendRgb565Pair(fgPair, loadPairNormal(dst)));
				storePairNormal(dst + 2, halfBlendRgb565Pair(fgPair, loadPairNormal(dst + 2)));
				dst += 4;
				n -= 4;
			}
			while (n >= 2) {
				storePairNormal(dst, halfBlendRgb565Pair(fgPair, loadPairNormal(dst)));
				dst += 2;
				n -= 2;
			}
			if (n > 0) {
				*dst = halfBlend(fc, *dst);
				dst++;
			}
		};
		auto paintHalfUnclamped = [&](std::int32_t &fx, int n) {
			if ((reinterpret_cast<std::uintptr_t>(dst) & 2u) && n > 0) {
				const std::uint16_t fc = colorRgb565[fx >> 16];
				*dst = halfBlend(fc, *dst);
				dst++;
				fx += dBucketFx;
				n--;
			}
			while (n >= 4) {
				const std::uint16_t fc0 = colorRgb565[fx >> 16];
				const std::int32_t fx1 = fx + dBucketFx;
				const std::uint16_t fc1 = colorRgb565[fx1 >> 16];
				const std::int32_t fx2 = fx1 + dBucketFx;
				const std::uint16_t fc2 = colorRgb565[fx2 >> 16];
				const std::int32_t fx3 = fx2 + dBucketFx;
				const std::uint16_t fc3 = colorRgb565[fx3 >> 16];
				const std::uint32_t fgPair0 = static_cast<std::uint32_t>(fc0) |
				                              (static_cast<std::uint32_t>(fc1) << 16);
				const std::uint32_t fgPair1 = static_cast<std::uint32_t>(fc2) |
				                              (static_cast<std::uint32_t>(fc3) << 16);
				storePairNormal(dst, halfBlendRgb565Pair(fgPair0, loadPairNormal(dst)));
				storePairNormal(dst + 2, halfBlendRgb565Pair(fgPair1, loadPairNormal(dst + 2)));
				dst += 4;
				fx = fx3 + dBucketFx;
				n -= 4;
			}
			while (n >= 2) {
				const std::uint16_t fc0 = colorRgb565[fx >> 16];
				const std::int32_t fx1 = fx + dBucketFx;
				const std::uint16_t fc1 = colorRgb565[fx1 >> 16];
				const std::uint32_t fgPair = static_cast<std::uint32_t>(fc0) |
				                             (static_cast<std::uint32_t>(fc1) << 16);
				storePairNormal(dst, halfBlendRgb565Pair(fgPair, loadPairNormal(dst)));
				dst += 2;
				fx = fx1 + dBucketFx;
				n -= 2;
			}
			if (n > 0) {
				const std::uint16_t fc = colorRgb565[fx >> 16];
				*dst = halfBlend(fc, *dst);
				dst++;
				fx += dBucketFx;
			}
		};
	if (av == 16) {
		int insideStart = 0;
		int insideEnd = 0;
		splitInsideBuckets(bucketFx, dBucketFx, count, insideStart, insideEnd);
		if (insideEnd > insideStart) {
			if (insideStart > 0) {
				paintHalfConst(clampBucket(bucketFx), insideStart);
				bucketFx = advanceFx(bucketFx, dBucketFx, insideStart);
			}
			if (dBucketFx == 0)
				paintHalfConst(bucketFx >> 16, insideEnd - insideStart);
			else
				paintHalfUnclamped(bucketFx, insideEnd - insideStart);
			if (insideEnd < count)
				paintHalfConst(clampBucket(bucketFx), count - insideEnd);
			return;
		}
		paintHalfConst(clampBucket(bucketFx), count);
		return;
	}

	const std::uint32_t inv = 32u - av;
	while (count-- > 0) {
		int bucket = bucketFx >> 16;
		if (bucket < 0)
			bucket = 0;
		else if (bucket > 250)
			bucket = 250;
		const std::uint16_t fc = colorRgb565[bucket];
		const std::uint16_t dc = swap16(*dst);
		const std::uint32_t f = (static_cast<std::uint32_t>(fc) | (static_cast<std::uint32_t>(fc) << 16)) & 0x07E0F81Fu;
		const std::uint32_t b = (static_cast<std::uint32_t>(dc) | (static_cast<std::uint32_t>(dc) << 16)) & 0x07E0F81Fu;
		const std::uint32_t o = ((f * av + b * inv + 0x02008010u) >> 5) & 0x07E0F81Fu;
		const std::uint16_t rn = static_cast<std::uint16_t>(o | (o >> 16));
		*dst++ = swap16(rn);
		bucketFx += dBucketFx;
	}
#else
	(void)dst;
	(void)colorRgb565;
	(void)a5;
	(void)bucketFx;
	(void)dBucketFx;
	(void)count;
#endif
}

extern "C" void __not_in_flash_func(gea_rgb565_lut_blend_span)(std::uint16_t *dst,
                                                                const std::uint16_t *colorRgb565,
                                                                const std::uint8_t *a5,
                                                                std::int32_t bucketFx,
                                                                std::int32_t dBucketFx,
                                                                int count)
{
#if GEA_RP2350_PIE_BLEND
	auto swap16 = [](std::uint16_t v) -> std::uint16_t {
#if GEA_EMBEDDED_PIXEL_PANEL_ENDIAN
		return __builtin_bswap16(v);
#else
		return v;
#endif
	};

	while (count-- > 0) {
		int bucket = bucketFx >> 16;
		if (bucket < 0)
			bucket = 0;
		else if (bucket > 250)
			bucket = 250;
		const std::uint32_t av = static_cast<std::uint32_t>(a5[bucket]);
		if (av == 0) {
			dst++;
			bucketFx += dBucketFx;
			continue;
		}
		const std::uint16_t fc = colorRgb565[bucket];
		const std::uint16_t dc = swap16(*dst);
		const std::uint32_t f = (static_cast<std::uint32_t>(fc) | (static_cast<std::uint32_t>(fc) << 16)) & 0x07E0F81Fu;
		const std::uint32_t b = (static_cast<std::uint32_t>(dc) | (static_cast<std::uint32_t>(dc) << 16)) & 0x07E0F81Fu;
		const std::uint32_t o = ((f * av + b * (32u - av) + 0x02008010u) >> 5) & 0x07E0F81Fu;
		const std::uint16_t rn = static_cast<std::uint16_t>(o | (o >> 16));
		*dst++ = swap16(rn);
		bucketFx += dBucketFx;
	}
#else
	(void)dst;
	(void)colorRgb565;
	(void)a5;
	(void)bucketFx;
	(void)dBucketFx;
	(void)count;
#endif
}

void *operator new(std::size_t size)
{
	return allocBlock(size, alignof(std::max_align_t));
}

void *operator new[](std::size_t size)
{
	return allocBlock(size, alignof(std::max_align_t));
}

void *operator new(std::size_t size, std::align_val_t alignment)
{
	return allocBlock(size, static_cast<std::size_t>(alignment));
}

void *operator new[](std::size_t size, std::align_val_t alignment)
{
	return allocBlock(size, static_cast<std::size_t>(alignment));
}

void *operator new(std::size_t size, const std::nothrow_t &) noexcept
{
	return allocBlock(size, alignof(std::max_align_t));
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
	return allocBlock(size, alignof(std::max_align_t));
}

void operator delete(void *ptr) noexcept
{
	freeBlock(ptr);
}

void operator delete[](void *ptr) noexcept
{
	freeBlock(ptr);
}

void operator delete(void *ptr, std::size_t) noexcept
{
	freeBlock(ptr);
}

void operator delete[](void *ptr, std::size_t) noexcept
{
	freeBlock(ptr);
}

void operator delete(void *ptr, std::align_val_t) noexcept
{
	freeBlock(ptr);
}

void operator delete[](void *ptr, std::align_val_t) noexcept
{
	freeBlock(ptr);
}

void operator delete(void *ptr, std::size_t, std::align_val_t) noexcept
{
	freeBlock(ptr);
}

void operator delete[](void *ptr, std::size_t, std::align_val_t) noexcept
{
	freeBlock(ptr);
}

namespace gea::rp2350 {

void memoryInit()
{
	const size_t maxFree = rp_mem_max_free_size();
	gUseRpAllocator = maxFree > 0;
	std::printf("rp2350 psram allocator: max_free=%u enabled=%d\n", static_cast<unsigned>(maxFree), gUseRpAllocator ? 1 : 0);
}

void boardIoInit()
{
	if (board::power.powerHold >= 0) {
		gpio_init(board::power.powerHold);
		gpio_set_dir(board::power.powerHold, GPIO_OUT);
		gpio_put(board::power.powerHold, 1);
	}

	if (board::power.powerKey >= 0) {
		gpio_init(board::power.powerKey);
		gpio_pull_up(board::power.powerKey);
		gpio_set_dir(board::power.powerKey, GPIO_IN);
	}

	adc_init();
	if (board::power.batteryAdc >= 0) {
		adc_gpio_init(board::power.batteryAdc);
		adc_select_input(board::power.batteryAdcChannel);
	}

	i2c_init(boardI2c(), board::i2c.frequencyHz);
	gpio_set_function(board::i2c.sda, GPIO_FUNC_I2C);
	gpio_set_function(board::i2c.scl, GPIO_FUNC_I2C);
	gpio_pull_up(board::i2c.sda);
	gpio_pull_up(board::i2c.scl);

#if GEA_RP2350_HAS_BUTTONS
	for (std::size_t i = 0; i < board::buttons.size(); ++i) {
		const auto &button = board::buttons[i];
		gpio_init(button.pin);
		gpio_set_dir(button.pin, GPIO_IN);
		if (button.pullUp) gpio_pull_up(button.pin);
		gButtonDown[i] = button.activeLow ? gpio_get(button.pin) == 0 : gpio_get(button.pin) != 0;
	}
#endif
}

void pollTouch()
{
#if GEA_RP2350_HAS_TOUCH
	if (!gTouchInitialized || !gTouchObserver) return;
	if (gSyntheticDrag.active) {
		pollSyntheticDrag();
		return;
	}
	gea::chips::ft6336::MultiTouchSample sample{};
	const bool ok = gTouchCore.readMulti(gTouchBus, sample);
	if (ok && sample.count > 0) {
		const int x = std::clamp(sample.x[0], 0, kPanelWidth - 1);
		const int y = std::clamp(sample.y[0], 0, kPanelHeight - 1);
		const auto phase = gTouching ? gea::platform::touch::Phase::Move : gea::platform::touch::Phase::Down;
		if (!gTouching || x != gTouchX || y != gTouchY) {
			gTouchObserver(phase, true, x, y);
		}
		gTouching = true;
		gTouchX = x;
		gTouchY = y;
		return;
	}
	if (gTouching) {
		gTouchObserver(gea::platform::touch::Phase::Up, false, gTouchX, gTouchY);
		gTouching = false;
	}
#endif
}

int scanoutRowToPhysical(int row)
{
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
	if (gDisplayCanvas.scrollRegionH() > 0) return gDisplayCanvas.rowToPhysical(row);
#endif
	return row;
}

bool startSyntheticDrag(int x1, int y1, int x2, int y2, int steps, int delayMs)
{
#if GEA_RP2350_HAS_TOUCH
	if (!gTouchInitialized || !gTouchObserver) return false;
	if (gSyntheticDrag.active) return false;
	gSyntheticDrag.x1 = std::clamp(x1, 0, kPanelWidth - 1);
	gSyntheticDrag.y1 = std::clamp(y1, 0, kPanelHeight - 1);
	gSyntheticDrag.x2 = std::clamp(x2, 0, kPanelWidth - 1);
	gSyntheticDrag.y2 = std::clamp(y2, 0, kPanelHeight - 1);
	gSyntheticDrag.steps = std::clamp(steps, 1, 256);
	gSyntheticDrag.delayUs = static_cast<std::uint64_t>(std::clamp(delayMs, 1, 250)) * 1000u;
	gSyntheticDrag.lastStep = -1;
	gSyntheticDrag.startUs = time_us_64();
	gSyntheticDrag.active = true;
	return true;
#else
	(void)x1; (void)y1; (void)x2; (void)y2; (void)steps; (void)delayMs;
	return false;
#endif
}

void pollButtons()
{
#if GEA_RP2350_HAS_BUTTONS
	// Press edge always queues one keydown. Buttons flagged `repeat` also
	// auto-repeat while held — an initial delay, then a steady interval —
	// matching a keyboard's typematic repeat, so holding Up/Down keeps the
	// view scrolling instead of nudging once per press.
	constexpr std::uint64_t kButtonRepeatInitialDelayUs = 300000;  // 300 ms hold before repeat
	constexpr std::uint64_t kButtonRepeatIntervalUs = 55000;       // ~18 repeats/sec
	const std::uint64_t nowUs = time_us_64();
	for (std::size_t i = 0; i < board::buttons.size(); ++i) {
		const auto &button = board::buttons[i];
		const bool down = button.activeLow ? gpio_get(button.pin) == 0 : gpio_get(button.pin) != 0;
		if (down && !gButtonDown[i]) {
			gea::framework::input::queueKeyDown(button.keyCode);
			gButtonNextRepeatUs[i] = nowUs + kButtonRepeatInitialDelayUs;
		} else if (down && button.repeat && nowUs >= gButtonNextRepeatUs[i]) {
			gea::framework::input::queueKeyDown(button.keyCode);
			gButtonNextRepeatUs[i] = nowUs + kButtonRepeatIntervalUs;
		}
		gButtonDown[i] = down;
	}
#endif
}

void dispatchQueuedEvents()
{
	gea::framework::events::Event event{};
	while (gea::framework::services::FrameScheduler::receiveEvent(&event)) {
		if (event.type == gea::framework::events::EventType::Touch) {
			gea::framework::events::TouchRuntime::dispatchEvent(event);
		}
	}
#if GEA_RP2350_HAS_BUTTONS
	dispatchKeyInput();
#endif
}

}  // namespace gea::rp2350

namespace gea::framework::memory {

void *Allocator::allocatePreferSpiram(std::size_t size, std::size_t alignment)
{
	return allocBlock(size, alignment);
}

void *Allocator::reallocatePreferSpiram(void *ptr, std::size_t size)
{
	return reallocBlock(ptr, size);
}

void Allocator::free(void *ptr) noexcept
{
	freeBlock(ptr);
}

double MemoryBackend::internalFree() { return 0.0; }
double MemoryBackend::internalLargestFreeBlock() { return 0.0; }
double MemoryBackend::internalMinimumFree() { return 0.0; }
double MemoryBackend::psramFree() { return static_cast<double>(rp_mem_max_free_size()); }
double MemoryBackend::currentTaskStackHighWaterMark() { return 0.0; }
double MemoryBackend::geaMainStackBytes() { return 0.0; }
double MemoryBackend::geaInitStackBytes() { return 0.0; }
double MemoryBackend::appFrameStackWords() { return 0.0; }
double MemoryBackend::appFrameStackBytes() { return 0.0; }
double MemoryBackend::displayFlushConfiguredRows() { return static_cast<double>(gFlushRows); }
double MemoryBackend::displayFlushConfiguredDepth() { return static_cast<double>(gFlushDepth); }
double MemoryBackend::displayFlushBufferMaxBytes() { return static_cast<double>(gea::rp2350::kPanelWidth * gFlushRows * 2 * gFlushDepth); }
double MemoryBackend::displayFlushRows() { return static_cast<double>(gFlushRows); }
double MemoryBackend::displayFlushDepth() { return static_cast<double>(gFlushDepth); }
double MemoryBackend::displayFlushBufferBytes() { return static_cast<double>(gea::rp2350::kPanelWidth * gFlushRows * 2 * gFlushDepth); }
double MemoryBackend::allocationSramCount() { return static_cast<double>(gSramAllocStats.allocCount); }
double MemoryBackend::allocationPsramCount() { return static_cast<double>(gPsramAllocStats.allocCount); }
double MemoryBackend::allocationSramBytes() { return static_cast<double>(gSramAllocStats.liveBytes); }
double MemoryBackend::allocationPsramBytes() { return static_cast<double>(gPsramAllocStats.liveBytes); }
double MemoryBackend::allocationSramPeakBytes() { return static_cast<double>(gSramAllocStats.peakBytes); }
double MemoryBackend::allocationPsramPeakBytes() { return static_cast<double>(gPsramAllocStats.peakBytes); }

}  // namespace gea::framework::memory

namespace gea::host {

double setTimeout(TimerCallback callback, double delayMs)
{
	if (!callback) return 0;
	for (auto &timer : gTimers) {
		if (timer.active) continue;
		timer.id = gNextTimerId++;
		timer.callback = std::move(callback);
		timer.delayMs = std::max(0.0, delayMs);
		timer.nextFireMs = gLastTimestampMs + timer.delayMs;
		timer.interval = false;
		timer.active = true;
		return timer.id;
	}
	return 0;
}

double setInterval(TimerCallback callback, double delayMs)
{
	if (!callback) return 0;
	for (auto &timer : gTimers) {
		if (timer.active) continue;
		timer.id = gNextTimerId++;
		timer.callback = std::move(callback);
		timer.delayMs = std::max(1.0, delayMs);
		timer.nextFireMs = gLastTimestampMs + timer.delayMs;
		timer.interval = true;
		timer.active = true;
		return timer.id;
	}
	return 0;
}

void clearTimeout(double id)
{
	for (auto &timer : gTimers) {
		if (timer.id == id) timer.active = false;
	}
}

void clearInterval(double id)
{
	clearTimeout(id);
}

void resetScheduledTimers()
{
	for (auto &timer : gTimers) timer.active = false;
}

double requestAnimationFrame(AnimationFrameCallback callback)
{
	if (!callback || gPendingRafCount >= static_cast<int>(gPendingRaf.size())) return 0;
	const double id = gNextTimerId++;
	gPendingRaf[static_cast<std::size_t>(gPendingRafCount++)] = std::move(callback);
	return id;
}

void runAnimationFrameCallbacks(double timestampMs)
{
	gLastTimestampMs = timestampMs;
	for (auto &timer : gTimers) {
		if (!timer.active || timer.nextFireMs > timestampMs || !timer.callback) continue;
		timer.callback();
		if (timer.interval) timer.nextFireMs = timestampMs + timer.delayMs;
		else timer.active = false;
	}

	const int count = gPendingRafCount;
	for (int i = 0; i < count; ++i) {
		gActiveRaf[static_cast<std::size_t>(i)] = std::move(gPendingRaf[static_cast<std::size_t>(i)]);
		gPendingRaf[static_cast<std::size_t>(i)] = nullptr;
	}
	gPendingRafCount = 0;
	for (int i = 0; i < count; ++i) {
		auto &callback = gActiveRaf[static_cast<std::size_t>(i)];
		if (callback) callback(timestampMs);
		callback = nullptr;
	}
}

void resetAnimationFrameCallbacks()
{
	for (auto &callback : gPendingRaf) callback = nullptr;
	for (auto &callback : gActiveRaf) callback = nullptr;
	gPendingRafCount = 0;
}

void animationFramePerfStatsReset() {}
AnimationFramePerfStats animationFramePerfStatsRead() { return {}; }

AudioDestinationProperty::operator AudioDestinationNode() const { return AudioDestinationNode(); }
AudioDestinationProperty::operator double() const { return 0.0; }
AudioContextCurrentTimeProperty::operator double() const { return 0.0; }
const AudioParamValueProperty &AudioParamValueProperty::operator=(double) const { return *this; }
AudioParamValueProperty::operator double() const { return 0.0; }
const AudioParam &AudioParam::operator=(double) const { return *this; }
void AudioParam::setValueAtTime(double, double) const {}
const OscillatorTypeProperty &OscillatorTypeProperty::operator=(double) const { return *this; }
const OscillatorTypeProperty &OscillatorTypeProperty::operator=(const char *) const { return *this; }
const OscillatorTypeProperty &OscillatorTypeProperty::operator=(const std::string &) const { return *this; }
void OscillatorNode::connect(AudioDestinationNode) const {}
void OscillatorNode::connect(AudioDestinationProperty) const {}
void OscillatorNode::connect(double) const {}
void OscillatorNode::start(double) const {}
void OscillatorNode::stop(double) const {}
AudioDestinationNode AudioBufferSourceNode::connect(AudioDestinationNode destination) const
{
	connected = true;
	return destination;
}
AudioDestinationNode AudioBufferSourceNode::connect(AudioDestinationProperty destination) const
{
	return connect(static_cast<AudioDestinationNode>(destination));
}
AudioDestinationNode AudioBufferSourceNode::connect(double destinationHandle) const
{
	return connect(AudioDestinationNode(destinationHandle));
}
void AudioBufferSourceNode::start(double) const {}
void AudioBufferSourceNode::stop(double) const {}
OscillatorNode AudioContext::createOscillator() const { return OscillatorNode(); }
AudioBufferSourceNode AudioContext::createBufferSource() const { return AudioBufferSourceNode(); }
AudioBuffer AudioContext::decodeAudioData(const std::vector<std::uint8_t> &) const { return AudioBuffer(); }

HTMLAudioElement::HTMLAudioElement(const char *src) : src_(src ? src : "") {}
HTMLAudioElement::HTMLAudioElement(const std::string &src) : src_(src) {}
HTMLAudioElement::HTMLAudioElement(const gea::embedded::ui::NodeHandle &node) : nodeId_(node.id()) {}
std::string HTMLAudioElement::src() const { return src_; }
void HTMLAudioElement::setSrc(const std::string &src) { src_ = src; }
bool HTMLAudioElement::play() const { return false; }
void HTMLAudioElement::pause() const {}

}  // namespace gea::host

// gea_host_image_load_asset_path now comes from core/packages/host/host/image.cpp
// (real embedded-asset lookup + ImageStore decode), compiled into this target.

namespace gea::framework::services {

EventQueue FrameScheduler::createEventQueue()
{
	gEventQueueStarted = true;
	return EventQueue(&gEvents);
}

EventQueue FrameScheduler::eventQueue()
{
	return gEventQueueStarted ? EventQueue(&gEvents) : EventQueue();
}

bool FrameScheduler::sendEvent(const gea::framework::events::Event &event, int)
{
	const int next = (gEventTail + 1) % static_cast<int>(gEvents.size());
	if (next == gEventHead) return false;
	gEvents[static_cast<std::size_t>(gEventTail)] = event;
	gEventTail = next;
	return true;
}

bool FrameScheduler::receiveEvent(gea::framework::events::Event *event)
{
	if (gEventHead == gEventTail) return false;
	if (event) *event = gEvents[static_cast<std::size_t>(gEventHead)];
	gEventHead = (gEventHead + 1) % static_cast<int>(gEvents.size());
	return true;
}

void FrameScheduler::start(EventQueue queue)
{
	gEventQueueStarted = static_cast<bool>(queue);
}

void FrameScheduler::runFrame(const FrameCallbacks &callbacks)
{
	if (callbacks.frame) callbacks.frame(nowMs(), callbacks.context);
}

bool FrameScheduler::takeCatchUpRequest() { return false; }
bool FrameScheduler::drainFramesAndCheckInput() { return gEventHead != gEventTail; }
void FrameScheduler::setVsyncDriven(bool driven) { gVsyncDriven = driven; }
bool FrameScheduler::vsyncDriven() { return gVsyncDriven; }
void FrameScheduler::notifyVsyncFromISR() {}

void FrameScheduler::setFrameIntervalMs(int intervalMs)
{
	if (intervalMs < kMinFrameIntervalMs) intervalMs = kMinFrameIntervalMs;
	if (intervalMs > kMaxFrameIntervalMs) intervalMs = kMaxFrameIntervalMs;
	gFrameIntervalMs = intervalMs;
}

int FrameScheduler::frameIntervalMs()
{
	return gFrameIntervalMs;
}

void FrameScheduler::setFrameRate(double fps)
{
	if (fps <= 0.0) return;
	setFrameIntervalMs(static_cast<int>((1000.0 / fps) + 0.5));
}

double FrameScheduler::frameRate()
{
	return gFrameIntervalMs > 0 ? 1000.0 / static_cast<double>(gFrameIntervalMs) : 0.0;
}

int FrameScheduler::nowMs()
{
	return static_cast<int>(time_us_64() / 1000u);
}

bool StorageService::init() { return true; }
bool StorageService::getString(const char *, char *, unsigned) { return false; }
bool StorageService::setString(const char *, const char *) { return false; }
bool StorageService::loadKv(std::string &out)
{
	out.clear();
	return false;
}
void StorageService::saveKv(const std::string &) {}

}  // namespace gea::framework::services

namespace gea::framework::app {

void Application::init(int width, int height, double devicePixelRatio)
{
	gea::embedded::ui::Document::setPreferredMountSize(width, height);
	gea::embedded::ui::setViewportMetrics(width, height, devicePixelRatio);
}

void Application::frame(int timestampMs)
{
	generated::drainMicrotasks();
	gea::host::runAnimationFrameCallbacks(static_cast<double>(timestampMs));
	gea::embedded::ui::Document::instance().frame(timestampMs);
	generated::drainMicrotasks();
	gea::embedded::ui::Document::instance().refreshMountedIfDirty();
}

void Application::toggleSettings() {}
void applicationFramePerfStatsReset() {}
void applicationFramePerfStatsAdd(ApplicationFramePhase, std::int64_t) {}
void applicationFramePerfStatsAdd(ApplicationFramePerfStats &, ApplicationFramePhase, std::int64_t) {}
ApplicationFramePerfStats applicationFramePerfStatsRead() { return {}; }
void applicationFramePhaseSet(ApplicationFramePhase) {}
ApplicationFramePhase applicationFramePhaseRead() { return ApplicationFramePhase::Idle; }
const char *applicationFramePhaseName(ApplicationFramePhase phase)
{
	switch (phase) {
	case ApplicationFramePhase::DrainMicrotasks:
		return "drainMicrotasks";
	case ApplicationFramePhase::AnimationFrameCallbacks:
		return "animationFrameCallbacks";
	case ApplicationFramePhase::StyleRecompute:
		return "styleRecompute";
	case ApplicationFramePhase::DocumentFrame:
		return "documentFrame";
	case ApplicationFramePhase::RefreshMounted:
		return "refreshMounted";
	case ApplicationFramePhase::Idle:
	default:
		return "idle";
	}
}

}  // namespace gea::framework::app

namespace gea::platform::display {

bool Display::init()
{
	if (!gFramebuffer) {
		const auto bytes = static_cast<std::size_t>(gea::rp2350::kPanelWidth) * gea::rp2350::kPanelHeight * sizeof(Pixel);
#if GEA_RP2350_FRAMEBUFFER_IN_SRAM
		gFramebuffer = gSramFramebuffer;
#else
		gFramebuffer = static_cast<Pixel *>(gea::framework::memory::Allocator::allocatePreferSpiram(bytes, alignof(Pixel)));
		if (!gFramebuffer) return false;
#endif
		std::memset(gFramebuffer, 0, bytes);
		gDisplayCanvas.bindPixels(gFramebuffer, gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
	}
#if GEA_RP2350_RENDER_CORE1
	startRenderWorker();
#endif
	const bool ok = gea::rp2350::panelInit();
	gea::rp2350::panelSetBrightness(gBrightness);
	return ok;
}

bool Display::start() { return init(); }
gea::framework::graphics::Canvas *Display::canvas() { return &activeDisplayCanvas(); }
bool Display::framebufferIsPanelDirect() { return true; }
bool Display::panelScanoutSurface(std::uint16_t **outBuffer, int *outWidth, int *outHeight)
{
	if (outBuffer) *outBuffer = gFramebuffer;
	if (outWidth) *outWidth = gea::rp2350::kPanelWidth;
	if (outHeight) *outHeight = gea::rp2350::kPanelHeight;
	return gFramebuffer != nullptr;
}
bool Display::panelDirectTarget(int, int, int, int, std::uint16_t **, int *, int *, int *, int *, int *, int *, int *, int *) { return false; }
void Display::flipPanelToBack() {}
bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height, bool)
{
	if (width) *width = gea::rp2350::kPanelWidth;
	if (height) *height = gea::rp2350::kPanelHeight;
	const int pixels = gea::rp2350::kPanelWidth * gea::rp2350::kPanelHeight;
	if (!dst || pixelCapacity < pixels || !gFramebuffer) return false;
	std::memcpy(dst, gFramebuffer, static_cast<std::size_t>(pixels) * sizeof(std::uint16_t));
	return true;
}
int Display::countNonBlackPixels(bool)
{
	if (!gFramebuffer) return 0;
	int count = 0;
	const int pixels = gea::rp2350::kPanelWidth * gea::rp2350::kPanelHeight;
	for (int i = 0; i < pixels; ++i) {
		if (gFramebuffer[i] != 0) ++count;
	}
	return count;
}

void Display::clear()
{
	gDisplayCanvas.clear(0);
	flush();
}
void Display::clearNoFlush() { gDisplayCanvas.clear(0); }
void Display::print(const char *) {}
void Display::flush()
{
	int x0 = 0;
	int y0 = 0;
	int x1 = -1;
	int y1 = -1;
	if (gInvalidated) {
		x0 = 0;
		y0 = 0;
		x1 = gea::rp2350::kPanelWidth - 1;
		y1 = gea::rp2350::kPanelHeight - 1;
		gInvalidated = false;
	} else if (!gDisplayCanvas.dirty(&x0, &y0, &x1, &y1)) {
		return;
	}
	flushRectClipped(x0, y0, x1, y1);
	gDisplayCanvas.resetDirty();
	gPreviousPresentValid = false;
}

void Display::flushRects(const DisplayFlushRect *rects, int count, bool)
{
	if (!rects || count <= 0) return;
	std::array<FlushWindow, kMaxCollectedWindows> windows{};
	const int mergedCount = collectFlushWindows(rects, count, windows);
	for (int i = 0; i < mergedCount; ++i) {
		const FlushWindow &window = windows[static_cast<std::size_t>(i)];
		flushRectClipped(window.x0, window.y0, window.x1, window.y1);
	}
	gDisplayCanvas.resetDirty();
	gInvalidated = false;
	gPreviousPresentValid = false;
}

void Display::flushRectsRasterized(const DisplayFlushRect *rects, int count, DisplayStreamRasterFn raster, void *user, bool)
{
	if (!rects || count <= 0 || !raster) return;
	std::array<FlushWindow, kMaxCollectedWindows> windows{};
	const int mergedCount = collectFlushWindows(rects, count, windows);
#if GEA_RP2350_PANEL_HW_VSCROLL
	if (gDisplayCanvas.scrollRegionH() > 0 && gScrollOffsetY != 0 && gFramebuffer) {
		// GRAM is circularly offset (panel scroll register engaged): the fused
		// stream would land at wrong rows. Raster row-by-row through the
		// framebuffer at its physical rows, then flush translated.
		for (int i = 0; i < mergedCount; ++i) {
			const FlushWindow &window = windows[static_cast<std::size_t>(i)];
			const int rowWidth = window.x1 - window.x0 + 1;
			std::uint16_t rowBuf[gea::rp2350::kPanelWidth];
			for (int yy = window.y0; yy <= window.y1; ++yy) {
				raster(rowBuf, rowWidth, 1, window.x0, yy, user);
				const int physY = gDisplayCanvas.rowToPhysical(yy);
				std::copy_n(rowBuf,
				            rowWidth,
				            gFramebuffer + static_cast<std::size_t>(physY) * gea::rp2350::kPanelWidth + window.x0);
			}
			flushRectClipped(window.x0, window.y0, window.x1, window.y1);
		}
		gDisplayCanvas.resetDirty();
		gInvalidated = false;
		gPreviousPresentValid = false;
		return;
	}
#endif
	for (int i = 0; i < mergedCount; ++i) {
		const FlushWindow &window = windows[static_cast<std::size_t>(i)];
		streamRect(window.x0,
		           window.y0,
		           window.x1 - window.x0 + 1,
		           window.y1 - window.y0 + 1,
		           raster,
		           user);
	}
	gDisplayCanvas.resetDirty();
	gInvalidated = false;
	gPreviousPresentValid = false;
}

void Display::rebindCanvasToFramebuffer()
{
	if (gFramebuffer) gDisplayCanvas.bindPixels(gFramebuffer, gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
#if GEA_RP2350_RENDER_CORE1
	if (gFramebuffer) gRenderWorkerCanvas.bindPixels(gFramebuffer, gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
#endif
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
	setScrollRegionAll(gScrollRegionY, gScrollRegionH, gScrollOffsetY);
#endif
}

bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user)
{
	if (!raster || w <= 0 || h <= 0) return false;
#if GEA_RP2350_PANEL_HW_VSCROLL
	// GRAM rows are circularly offset while the panel scroll register is
	// engaged; a raster stream addressed at logical rows would land wrong.
	if (gDisplayCanvas.scrollRegionH() > 0 && gScrollOffsetY != 0) return false;
#endif
	int x0 = std::max(0, x);
	int y0 = std::max(0, y);
	int x1 = std::min(gea::rp2350::kPanelWidth - 1, x + w - 1);
	int y1 = std::min(gea::rp2350::kPanelHeight - 1, y + h - 1);
	if (x0 > x1 || y0 > y1) return true;
	gea::rp2350::panelStreamRect(x0, y0, x1, y1, raster, user);
	gPreviousPresentValid = false;
	return true;
}

bool Display::present(const DisplayPresentCommand *commands, int commandCount)
{
	if (!commands || commandCount <= 0) return false;
	if (presentDirectToPanel(commands, commandCount)) return true;
	gPreviousPresentValid = false;
	for (int i = 0; i < commandCount; ++i) {
		const auto &command = commands[i];
		switch (command.type) {
		case DisplayPresentCommandType::Clear:
			gDisplayCanvas.clear(command.clear.color);
			break;
		case DisplayPresentCommandType::FillRectRgb565:
			gDisplayCanvas.setGlobalAlpha(command.fillRectRgb565.alpha);
			gDisplayCanvas.fillRect(command.fillRectRgb565.x, command.fillRectRgb565.y,
			                        command.fillRectRgb565.w, command.fillRectRgb565.h,
			                        command.fillRectRgb565.color);
			break;
		case DisplayPresentCommandType::StrokeRectRgb565:
			gDisplayCanvas.setGlobalAlpha(command.strokeRectRgb565.alpha);
			gDisplayCanvas.strokeRect(command.strokeRectRgb565.x, command.strokeRectRgb565.y,
			                          command.strokeRectRgb565.w, command.strokeRectRgb565.h,
			                          command.strokeRectRgb565.color);
			break;
		case DisplayPresentCommandType::FillTriangleRgb565:
			gDisplayCanvas.setGlobalAlpha(command.fillTriangleRgb565.alpha);
			gDisplayCanvas.fillTriangle(command.fillTriangleRgb565.x0, command.fillTriangleRgb565.y0,
			                            command.fillTriangleRgb565.x1, command.fillTriangleRgb565.y1,
			                            command.fillTriangleRgb565.x2, command.fillTriangleRgb565.y2,
			                            command.fillTriangleRgb565.color);
			break;
		case DisplayPresentCommandType::FillCircleRgb565:
			gDisplayCanvas.setGlobalAlpha(command.fillCircleRgb565.alpha);
			gDisplayCanvas.fillCircle(command.fillCircleRgb565.x, command.fillCircleRgb565.y,
			                          command.fillCircleRgb565.radius, command.fillCircleRgb565.color);
			break;
		case DisplayPresentCommandType::StrokeCircleRgb565:
			gDisplayCanvas.setGlobalAlpha(command.strokeCircleRgb565.alpha);
			gDisplayCanvas.strokeCircle(command.strokeCircleRgb565.x, command.strokeCircleRgb565.y,
			                            command.strokeCircleRgb565.radius, command.strokeCircleRgb565.color);
			break;
			case DisplayPresentCommandType::FillCirclesRgb565:
				gDisplayCanvas.setGlobalAlpha(command.fillCirclesRgb565.alpha);
				gDisplayCanvas.fillCirclesRgb565(command.fillCirclesRgb565.xs, command.fillCirclesRgb565.ys,
				                                 command.fillCirclesRgb565.count, command.fillCirclesRgb565.radius,
				                                 command.fillCirclesRgb565.colors);
				break;
			case DisplayPresentCommandType::DrawImage:
				gDisplayCanvas.setGlobalAlpha(command.drawImage.alpha);
			gDisplayCanvas.drawImage(command.drawImage.pixels, command.drawImage.alphaPixels,
			                         command.drawImage.srcWidth, command.drawImage.srcHeight,
			                         command.drawImage.x, command.drawImage.y);
			break;
		case DisplayPresentCommandType::DrawImageScaled:
			gDisplayCanvas.setGlobalAlpha(command.drawImageScaled.alpha);
			gDisplayCanvas.drawImage(command.drawImageScaled.pixels, command.drawImageScaled.alphaPixels,
			                         command.drawImageScaled.srcWidth, command.drawImageScaled.srcHeight,
			                         command.drawImageScaled.x, command.drawImageScaled.y,
			                         command.drawImageScaled.w, command.drawImageScaled.h);
			break;
		case DisplayPresentCommandType::DrawImageRotated90CW:
			gDisplayCanvas.setGlobalAlpha(command.drawImageRotated90CW.alpha);
			gDisplayCanvas.drawImageRotated90CW(command.drawImageRotated90CW.pixels,
			                                    command.drawImageRotated90CW.alphaPixels,
			                                    command.drawImageRotated90CW.srcWidth,
			                                    command.drawImageRotated90CW.srcHeight,
			                                    command.drawImageRotated90CW.x,
			                                    command.drawImageRotated90CW.y,
			                                    command.drawImageRotated90CW.w,
			                                    command.drawImageRotated90CW.h);
			break;
		case DisplayPresentCommandType::DrawImageTiledX:
			gDisplayCanvas.setGlobalAlpha(command.drawImageTiledX.alpha);
			gDisplayCanvas.drawImageTiledX(command.drawImageTiledX.pixels, command.drawImageTiledX.alphaPixels,
			                               command.drawImageTiledX.srcWidth, command.drawImageTiledX.srcHeight,
			                               command.drawImageTiledX.x, command.drawImageTiledX.y,
			                               command.drawImageTiledX.w);
			break;
		case DisplayPresentCommandType::FillText:
			gDisplayCanvas.setGlobalAlpha(command.fillText.alpha);
			if (command.fillText.fontFamilyId >= 0) {
				gDisplayCanvas.drawTextFontFamily(command.fillText.text, command.fillText.x, command.fillText.y,
				                                  command.fillText.color, command.fillText.fontFamilyId,
				                                  command.fillText.fontSizePx);
			} else {
				gDisplayCanvas.drawText(command.fillText.text, command.fillText.x, command.fillText.y,
				                        command.fillText.color, command.fillText.scale);
			}
			break;
		}
	}
	gDisplayCanvas.setGlobalAlpha(gDisplayAlpha);
	flush();
	return true;
}

void Display::setFlushConfig(int chunkRows, int queueDepth)
{
	if (chunkRows > 0) gFlushRows = chunkRows;
	if (queueDepth > 0) gFlushDepth = queueDepth;
	gea::rp2350::panelSetFlushConfig(gFlushRows, gFlushDepth);
}
void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return gFlushRows; }
int Display::flushQueueDepth() { return gFlushDepth; }
int Display::flushBufferBytes() { return gea::rp2350::kPanelWidth * gFlushRows * 2 * gFlushDepth; }
void Display::pushClip(int x, int y, int w, int h) { activeDisplayCanvas().pushClip(x, y, w, h); }
void Display::popClip() { activeDisplayCanvas().popClip(); }
void Display::resetClip() { activeDisplayCanvas().resetClip(); }
void Display::setAlpha(std::uint8_t alpha)
{
	activeDisplayCanvas().setGlobalAlpha(alpha);
	if (!onRenderWorkerCore()) gDisplayAlpha = alpha;
}
std::uint8_t Display::alpha() { return activeDisplayCanvas().globalAlpha(); }
int Display::brightness() { return gBrightness; }
void Display::setBrightness(int brightnessPercent)
{
	gBrightness = std::clamp(brightnessPercent, 0, 100);
	gea::rp2350::panelSetBrightness(gBrightness);
}
void Display::setVSync(bool on) { gVsyncDriven = on; }
bool Display::vsyncEnabled() { return gVsyncDriven; }
void Display::invalidate() { gInvalidated = true; }
void Display::setPresentScale(int scale) { gPresentScale = scale >= 2 ? 2 : 1; }
void Display::vsyncWaitForFrame() {}
void Display::clip(int *x0, int *y0, int *x1, int *y1) { activeDisplayCanvas().currentClip(x0, y0, x1, y1); }
void Display::fillRect(int x, int y, int w, int h, Pixel color) { activeDisplayCanvas().fillRect(x, y, w, h, color); }
void Display::scrollRect(int x, int y, int w, int h, int dx, int dy)
{
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
	if (!onRenderWorkerCore() &&
	    dx == 0 &&
	    dy != 0 &&
	    x == 0 &&
	    w >= gea::rp2350::kPanelWidth &&
	    h > 0 &&
	    y >= 0 &&
	    y + h <= gea::rp2350::kPanelHeight) {
		if (gScrollRegionY != y || gScrollRegionH != h) {
			gScrollRegionY = y;
			gScrollRegionH = h;
			gScrollOffsetY = 0;
		}
		gScrollOffsetY -= dy;
		gScrollOffsetY %= gScrollRegionH;
		if (gScrollOffsetY < 0) gScrollOffsetY += gScrollRegionH;
		setScrollRegionAll(gScrollRegionY, gScrollRegionH, gScrollOffsetY);
#if GEA_RP2350_PANEL_HW_VSCROLL
		if (gScrollRegionY == 0 && gScrollRegionH == gea::rp2350::kPanelHeight) {
			// The panel rotates scanout itself (VSCSAD in setScrollRegionAll);
			// only the revealed strip needs pixels, and its replay marks its
			// own dirty rect.
			return;
		}
#endif
		gDisplayCanvas.markDirty(0, y, gea::rp2350::kPanelWidth - 1, y + h - 1);
		return;
	}
#endif
	activeDisplayCanvas().scrollRect(x, y, w, h, dx, dy);
}
void Display::resetScrollRegion()
{
#if GEA_RP2350_SOFTWARE_SCROLL_REGISTER
	setScrollRegionAll(0, 0, 0);
#else
	activeDisplayCanvas().setScrollRegion(0, 0, 0);
#endif
}
void Display::strokeRect(int x, int y, int w, int h, Pixel color) { activeDisplayCanvas().strokeRect(x, y, w, h, color); }
void Display::fillCircle(int cx, int cy, int r, Pixel color) { activeDisplayCanvas().fillCircle(cx, cy, r, color); }
void Display::strokeCircle(int cx, int cy, int r, Pixel color) { activeDisplayCanvas().strokeCircle(cx, cy, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, Pixel color) { activeDisplayCanvas().drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int cx, int cy, int r, int startDeg, int endDeg, Pixel color) { activeDisplayCanvas().drawArc(cx, cy, r, startDeg, endDeg, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, Pixel color) { activeDisplayCanvas().fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, Pixel color, float scale) { activeDisplayCanvas().drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, Pixel color, int fontId) { activeDisplayCanvas().drawTextFont(text, x, y, color, fontId); }
void Display::drawTextFontFamily(const char *text, int x, int y, Pixel color, int familyId, int sizePx) { activeDisplayCanvas().drawTextFontFamily(text, x, y, color, familyId, sizePx); }
void Display::setPixel(int x, int y, Pixel color) { activeDisplayCanvas().fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, Pixel color) { activeDisplayCanvas().fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count, int w, int h, int tl, int tr, int br, int bl, const Pixel *colors)
{
	activeDisplayCanvas().fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lw, Pixel color) { activeDisplayCanvas().strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lw, color); }
void Display::blitImage(const Pixel *src, const std::uint8_t *alpha, int srcW, int srcH, int dx, int dy) { activeDisplayCanvas().drawImage(src, alpha, srcW, srcH, dx, dy); }
void Display::blitImageScaled(const Pixel *src, const std::uint8_t *alpha, int srcW, int srcH, int dx, int dy, int dstW, int dstH) { activeDisplayCanvas().drawImage(src, alpha, srcW, srcH, dx, dy, dstW, dstH); }
void Display::setWorldOverlay(const std::uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}
void Display::flushStatsRead(std::int64_t *totalUs, int *callCount, int *pixelCount)
{
	const auto stats = gea::rp2350::panelFlushStatsRead();
	if (totalUs) *totalUs = stats.totalUs;
	if (callCount) *callCount = stats.callCount;
	if (pixelCount) *pixelCount = stats.pixelCount;
}
DisplayFlushPerfStats Display::flushPerfStatsRead()
{
	const auto panelStats = gea::rp2350::panelFlushStatsRead();
	DisplayFlushPerfStats stats{};
	stats.totalUs = panelStats.totalUs;
	stats.setWindowUs = panelStats.setWindowUs;
	stats.copyUs = panelStats.copyUs;
	stats.txUs = panelStats.txWaitUs;
	stats.completeWaitUs = panelStats.txWaitUs;
	stats.callCount = panelStats.callCount;
	stats.chunkCount = panelStats.chunkCount;
	stats.pixelCount = panelStats.pixelCount;
	return stats;
}
void Display::presentPathDebug(int *calls, int *direct, int *general, int *rejected, int *tileShapeFailKind, int *tileShapeFailType, int *tileShapeFailCount)
{
	if (calls) *calls = 0;
	if (direct) *direct = 0;
	if (general) *general = 0;
	if (rejected) *rejected = 0;
	if (tileShapeFailKind) *tileShapeFailKind = 0;
	if (tileShapeFailType) *tileShapeFailType = 0;
	if (tileShapeFailCount) *tileShapeFailCount = 0;
}
void Display::landFrameDebug(int *total, int *align, int *raster, int *text, int *flip)
{
	if (total) *total = 0;
	if (align) *align = 0;
	if (raster) *raster = 0;
	if (text) *text = 0;
	if (flip) *flip = 0;
}
void Display::landPanDebug(int *detect, int *kick, int *strips, int *wait, int *interior)
{
	if (detect) *detect = 0;
	if (kick) *kick = 0;
	if (strips) *strips = 0;
	if (wait) *wait = 0;
	if (interior) *interior = 0;
}
void Display::flushStatsReset()
{
	gFlushCalls = 0;
	gFlushPixels = 0;
	gea::rp2350::panelFlushStatsReset();
}
const char *Display::flushStageName() { return "idle"; }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return {}; }

}  // namespace gea::platform::display

namespace gea::framework::display {

double DisplayBackend::brightness() { return static_cast<double>(gea::platform::display::Display::brightness()); }
void DisplayBackend::setBrightness(double brightness) { gea::platform::display::Display::setBrightness(static_cast<int>(brightness)); }
void DisplayBackend::setAA(double samples) { gea::platform::display::Display::setAA(static_cast<int>(samples)); }
void DisplayBackend::setFlushConfig(double rows, double depth) { gea::platform::display::Display::setFlushConfig(static_cast<int>(rows), static_cast<int>(depth)); }
void DisplayBackend::setEpaperRefreshConfig(double, double, double, const std::vector<std::uint8_t> &, bool, const std::vector<std::uint8_t> &, bool) {}
void DisplayBackend::epaperFullRefresh() {}
void DisplayBackend::setEpaperGrayscale(bool) {}
double DisplayBackend::nativeWidth() { return static_cast<double>(detail::DisplayOrientationState::nativeWidth()); }
double DisplayBackend::nativeHeight() { return static_cast<double>(detail::DisplayOrientationState::nativeHeight()); }
std::string DisplayBackend::orientation() { return detail::DisplayOrientationState::orientationString(); }
void DisplayBackend::setOrientation(const std::string &orientation) { detail::DisplayOrientationState::setOrientation(orientation); }
std::vector<std::string> DisplayBackend::supportedOrientations() { return detail::DisplayOrientationState::supportedOrientations(); }
void DisplayBackend::setSupportedOrientations(const std::vector<std::string> &orientations) { detail::DisplayOrientationState::setSupportedOrientations(orientations); }
void DisplayBackend::setSupportedOrientations(const std::string &orientation) { detail::DisplayOrientationState::setSupportedOrientations(orientation); }
bool DisplayBackend::autoRotate() { return detail::DisplayOrientationState::autoRotate(); }
void DisplayBackend::setAutoRotate(bool enabled) { detail::DisplayOrientationState::setAutoRotate(enabled); }
void DisplayBackend::setVSync(bool on) { gea::platform::display::Display::setVSync(on); }
void DisplayBackend::setTextRasterCache(bool on) { gea::framework::graphics::Canvas::setTextRasterCacheEnabled(on); }
void DisplayBackend::invalidate() { gea::platform::display::Display::invalidate(); }
std::string DisplayBackend::pixelFormat() { return "rgb565"; }
void DisplayBackend::setPixelFormat(const std::string &) {}
std::string DisplayBackend::panelPixelFormat() { return "rgb565"; }
std::vector<std::string> DisplayBackend::supportedPixelFormats() { return {"rgb565"}; }
void DisplayBackend::updateAutoRotationFromAccelerometer() { detail::DisplayOrientationState::updateAutoRotationFromAccelerometer(); }
void DisplayBackend::updateAutoRotation(double x, double y, double z) { detail::DisplayOrientationState::updateAutoRotation(x, y, z); }

}  // namespace gea::framework::display

namespace gea::platform::touch {

void Touchscreen::setObserver(Observer observer)
{
#if GEA_RP2350_HAS_TOUCH
	gTouchObserver = observer;
#else
	(void)observer;
#endif
}

bool Touchscreen::init()
{
#if GEA_RP2350_HAS_TOUCH
	gpio_init(board::touch.reset);
	gpio_set_dir(board::touch.reset, GPIO_OUT);
	gpio_put(board::touch.reset, 1);
	sleep_ms(10);
	gpio_put(board::touch.reset, 0);
	sleep_ms(10);
	gpio_put(board::touch.reset, 1);
	sleep_ms(300);

	gpio_init(board::touch.interrupt);
	gpio_set_dir(board::touch.interrupt, GPIO_IN);
	gpio_pull_up(board::touch.interrupt);

	std::uint8_t chipId = 0;
	const bool idOk = gTouchCore.readChipId(gTouchBus, chipId);
	gTouchInitialized = idOk && chipId == gea::chips::ft6336::kExpectedChipId &&
	                    gTouchCore.configure(gTouchBus, false);
	std::printf("ft6336 init=%d id_ok=%d id=0x%02x\n",
	            gTouchInitialized ? 1 : 0,
	            idOk ? 1 : 0,
	            chipId);
	return gTouchInitialized;
#else
	return false;
#endif
}

int Touchscreen::read(int *x, int *y)
{
#if GEA_RP2350_HAS_TOUCH
	gea::chips::ft6336::TouchSample sample{};
	const bool ok = gTouchCore.read(gTouchBus, sample);
	if (ok && sample.touching) {
		gTouchX = std::clamp(sample.x, 0, gea::rp2350::kPanelWidth - 1);
		gTouchY = std::clamp(sample.y, 0, gea::rp2350::kPanelHeight - 1);
		gTouching = true;
		if (x) *x = gTouchX;
		if (y) *y = gTouchY;
		return 1;
	}
	if (x) *x = gTouchX;
	if (y) *y = gTouchY;
	return 0;
#else
	if (x) *x = 0;
	if (y) *y = 0;
	return 0;
#endif
}

int Touchscreen::readCached(int *x, int *y)
{
#if GEA_RP2350_HAS_TOUCH
	if (x) *x = gTouchX;
	if (y) *y = gTouchY;
	return gTouching ? 1 : 0;
#else
	if (x) *x = 0;
	if (y) *y = 0;
	return 0;
#endif
}

void Touchscreen::consumeLatestMove(int *x, int *y)
{
#if GEA_RP2350_HAS_TOUCH
	if (x) *x = gTouchX;
	if (y) *y = gTouchY;
#else
	if (x) *x = 0;
	if (y) *y = 0;
#endif
}

void Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
#if GEA_RP2350_HAS_TOUCH
	gTouchX = x;
	gTouchY = y;
	gTouching = touching;
	if (gTouchObserver) gTouchObserver(phase, touching, x, y);
#else
	(void)phase;
	(void)touching;
	(void)x;
	(void)y;
#endif
}

}  // namespace gea::platform::touch

namespace gea::framework::sensors {
void AccelerometerBackend::init() {}
void AccelerometerBackend::close() {}
void AccelerometerBackend::calibrateBias() {}
double AccelerometerBackend::tiltX() { return 0.0; }
double AccelerometerBackend::tiltY() { return 0.0; }
double AccelerometerBackend::accelerationX() { return 0.0; }
double AccelerometerBackend::accelerationY() { return 0.0; }
double AccelerometerBackend::accelerationZ() { return 9.8; }
double AccelerometerBackend::gyroscopeX() { return 0.0; }
double AccelerometerBackend::gyroscopeY() { return 0.0; }
double AccelerometerBackend::gyroscopeZ() { return 0.0; }
}  // namespace gea::framework::sensors

namespace gea::framework::network {
bool WifiBackend::enabled() { return false; }
void WifiBackend::setEnabled(bool) {}
bool WifiBackend::connected() { return false; }
double WifiBackend::rssi() { return 0.0; }
std::string WifiBackend::ssid() { return {}; }
std::string WifiBackend::ip() { return {}; }
std::string WifiBackend::mac() { return {}; }
void WifiBackend::configure(const std::string &, const std::string &) {}
bool WifiBackend::waitForConnection(double) { return false; }
void WifiBackend::startScan() {}
bool WifiBackend::scanning() { return false; }
double WifiBackend::scanCount() { return 0.0; }
std::string WifiBackend::scanSsidAt(double) { return {}; }
double WifiBackend::scanRssiAt(double) { return 0.0; }
bool WifiBackend::scanSecuredAt(double) { return false; }
}  // namespace gea::framework::network

namespace gea::framework::camera {
bool CameraBackend::isAvailable() { return false; }
bool CameraBackend::hasPermission() { return false; }
bool CameraBackend::requestPermission() { return false; }
bool CameraBackend::open(const std::string &, double, double) { return false; }
void CameraBackend::close() {}
bool CameraBackend::isOpen() { return false; }
double CameraBackend::width() { return 0.0; }
double CameraBackend::height() { return 0.0; }
double CameraBackend::orientation() { return 0.0; }
std::string CameraBackend::facing() { return {}; }
double CameraBackend::deviceCount() { return 0.0; }
std::string CameraBackend::deviceIdAt(double) { return {}; }
std::string CameraBackend::deviceFacingAt(double) { return {}; }
void CameraBackend::draw(double, double, double, double) {}
double CameraBackend::capture() { return -1.0; }
double CameraBackend::captureMirrored() { return -1.0; }
bool CameraBackend::startRecording(const std::string &, double) { return false; }
double CameraBackend::stopRecording() { return -1.0; }
bool CameraBackend::isRecording() { return false; }
void CameraBackend::setFlash(const std::string &) {}
void CameraBackend::setZoom(double) {}
void CameraBackend::setMirror(bool) {}
void CameraBackend::setExposure(const std::string &, double, double, double) {}
void CameraBackend::setWhiteBalance(const std::string &, double, double) {}
void CameraBackend::setFocus(const std::string &, double, double) {}
void CameraBackend::setTorch(const std::string &, double) {}
void registerCameraSurface() {}
}  // namespace gea::framework::camera

namespace gea::framework::geolocation {
bool GeolocationBackend::hasFix() { return false; }
GeolocationPosition GeolocationBackend::currentPosition() { return {}; }
double GeolocationBackend::latitude() { return 0.0; }
double GeolocationBackend::longitude() { return 0.0; }
double GeolocationBackend::accuracy() { return -1.0; }
}  // namespace gea::framework::geolocation

namespace gea::framework::bluetooth {
void HidBackend::init(const std::string &, double, const std::string &) {}
bool HidBackend::enabled() { return false; }
void HidBackend::setEnabled(bool) {}
void HidBackend::startAdvertising() {}
void HidBackend::stopAdvertising() {}
bool HidBackend::connected() { return false; }
bool HidBackend::bound() { return false; }
double HidBackend::batteryLevel() { return 0.0; }
std::string HidBackend::mac() { return {}; }
std::string HidBackend::deviceName() { return {}; }
void HidBackend::keyTap(double) {}
void HidBackend::keyDown(double, double) {}
void HidBackend::keyUp() {}
void HidBackend::mouseMove(double, double, double, double) {}
void HidBackend::mouseClick(double) {}
}  // namespace gea::framework::bluetooth

namespace gea::framework::audio {
double AudioBackend::volume() { return 0.0; }
void AudioBackend::setVolume(double) {}
}  // namespace gea::framework::audio

namespace gea::framework::input {
bool InputBackend::consumeBackButton() { return false; }
}  // namespace gea::framework::input

namespace gea::platform::audio {
AudioParam::AudioParam(NativeAudioHandle oscillator) : oscillator_(oscillator) {}
double AudioParam::value() const { return 0.0; }
void AudioParam::setValue(double) {}
void AudioParam::setValueAtTime(double, double) {}
AudioNode::AudioNode(NativeAudioHandle native) : native_(native) {}
NativeAudioHandle AudioNode::nativeId() const { return native_; }
AudioDestinationNode::AudioDestinationNode(NativeAudioHandle native) : AudioNode(native) {}
OscillatorNode::OscillatorNode(NativeAudioHandle native) : AudioNode(native), frequency(native) {}
OscillatorType OscillatorNode::type() const { return OscillatorType::Sine; }
void OscillatorNode::setType(OscillatorType) {}
void OscillatorNode::connect(const AudioDestinationNode &) {}
void OscillatorNode::start(double) {}
void OscillatorNode::stop(double) {}
double AudioContext::currentTime() const { return 0.0; }
AudioDestinationNode AudioContext::destination() const { return AudioDestinationNode(); }
OscillatorNode AudioContext::createOscillator() const { return OscillatorNode(); }
AudioContext AudioSystem::sharedContext() { return AudioContext(); }
int AudioSystem::volume() { return 0; }
void AudioSystem::setVolume(int) {}
bool AudioSystem::playFile(const std::string &) { return false; }
bool AudioSystem::playPcm(const std::int16_t *, std::size_t, int, int) { return false; }
void AudioSystem::stopPlayback() {}
}  // namespace gea::platform::audio
