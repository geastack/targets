#include <rp2350_gea_platform.h>
#include <rp2350_panel.h>

#include "display.h"
#include "events.h"
#include "image.h"
#include "css/declarative.h"
#include "css/engine.h"
#include "host/timers.h"
#include "host/memory.h"
#include "pixel.h"
#include "services/frame_scheduler.h"
#include "ui/document.h"
#include "ui/tree_internal.h"
#include "ui/refresh_perf.h"
#include "ui/style.h"

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/structs/timer.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>

extern void __gea_top_level();
// Deferred CSS/prelude registration: the codegen emits it as this function
// (via --cpp-prelude-symbol) so it runs after memoryInit() and the stylesheet
// allocates from PSRAM, not the small pre-main SRAM heap. Weak + C++ linkage
// (the emitter writes a plain C++ function); bringup builds without an app
// don't define it.
void gea_plugin_cpp_register_prelude() __attribute__((weak));

namespace gea::framework::app::generated {
void drainMicrotasks();
}  // namespace gea::framework::app::generated

// ---- Pre-main crash logger (boot-debug aid) --------------------------------
// A HardFault before main() leaves the board dark on USB with nothing to
// observe. The handler stashes the faulting PC/LR in watchdog scratch (which
// survives a reboot) and resets; the priority-101 constructor below runs
// before all other C++ static constructors, brings USB up, and reports the
// previous run's fault (or a hang marker) for ~8 s so a host capture sees it.
extern "C" void isr_hardfault(void)
{
	std::uint32_t *sp;
	__asm volatile("mrs %0, msp" : "=r"(sp));
	watchdog_hw->scratch[4] = 0xFA17FA17u;
	watchdog_hw->scratch[5] = sp[6];  // stacked PC
	watchdog_hw->scratch[6] = sp[5];  // stacked LR
	watchdog_reboot(0, 0, 0);
	while (true) {
		tight_loop_contents();
	}
}

// Panic override (boot-debug aid): the SDK default prints once and hits a
// breakpoint, which without a debugger lands in a lockup-reset before the USB
// buffer drains — the message is lost. Broadcast it for a while instead, then
// reboot cleanly.
extern "C" void __attribute__((noreturn)) gea_panic(const char *fmt, ...)
{
	char buf[256];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(buf, sizeof buf, fmt, args);
	va_end(args);
	const std::uint64_t untilUs = time_us_64() + 8000000u;
	while (time_us_64() < untilUs) {
		std::printf("GEA_PANIC %s\n", buf);
		std::fflush(stdout);
		sleep_ms(250);
	}
	watchdog_reboot(0, 0, 0);
	while (true) {
		tight_loop_contents();
	}
}

// PC sampler: a raw TIMER0 alarm IRQ fires every ~1 ms and stores the
// interrupted instruction address straight into watchdog scratch[5] (which
// survives the mystery lockup-reset). The next boot reports it — pointing at
// the exact code the previous boot was stuck in.
extern "C" {
volatile std::uint32_t *gea_pc_sample_slot = &watchdog_hw->scratch[5];
// Boot profile ring: the same 1ms IRQ also appends each PC here until full
// (~4s — covers the whole boot). Dumped over GEADEV BOOTPROF and folded into
// a function histogram host-side with addr2line.
constexpr int kPcRingEntries = 4096;
std::uint32_t gea_pc_ring[kPcRingEntries];
std::uint32_t *gea_pc_ring_ptr = gea_pc_ring;
std::uint32_t *gea_pc_ring_limit = gea_pc_ring + kPcRingEntries;
void gea_pc_sampler_rearm(void);
// Last CSS recompute-batch profile line (style.cpp, GEA_RECPROF builds; null otherwise).
const char *gea_recprof_last(void);

__attribute__((naked)) void gea_pc_sampler_irq(void)
{
	__asm volatile(
	    "mrs r0, msp\n"
	    "ldr r1, [r0, #24]\n"  // stacked PC of the interrupted code
	    "ldr r2, =gea_pc_sample_slot\n"
	    "ldr r2, [r2]\n"
	    "str r1, [r2]\n"
	    "ldr r2, =gea_pc_ring_ptr\n"
	    "ldr r3, [r2]\n"
	    "ldr r0, =gea_pc_ring_limit\n"
	    "ldr r0, [r0]\n"
	    "cmp r3, r0\n"
	    "bcs 1f\n"  // ring full: keep only the hang slot updated
	    "str r1, [r3]\n"
	    "adds r3, #4\n"
	    "str r3, [r2]\n"
	    "1:\n"
	    "b gea_pc_sampler_rearm\n");
}

void gea_pc_sampler_rearm(void)
{
	timer0_hw->intr = 1u;  // ack alarm 0
	timer0_hw->alarm[0] = timer0_hw->timerawl + 1000u;
}
}

namespace {

struct EarlyBootBeacon {
	EarlyBootBeacon()
	{
		stdio_init_all();
		const bool faulted = watchdog_hw->scratch[4] == 0xFA17FA17u;
		const bool hungInCtors = !faulted && watchdog_hw->scratch[7] == 0xC702C702u;
		if (faulted || hungInCtors) {
			const std::uint32_t pc = watchdog_hw->scratch[5];
			const std::uint32_t lr = watchdog_hw->scratch[6];
			const std::uint64_t untilUs = time_us_64() + 8000000u;
			while (time_us_64() < untilUs) {
				if (faulted) {
					std::printf("GEA_FAULT pre-main pc=%08x lr=%08x\n",
					            static_cast<unsigned>(pc),
					            static_cast<unsigned>(lr));
				} else {
					std::printf("GEA_HANG pre-main, last sampled pc=%08x\n",
					            static_cast<unsigned>(pc));
				}
				std::fflush(stdout);
				sleep_ms(250);
			}
			watchdog_hw->scratch[4] = 0;
		}
		watchdog_hw->scratch[7] = 0xC702C702u;  // phase: static ctors running
		watchdog_hw->scratch[5] = 0;

		// Start the PC sampler for the remainder of the boot.
		irq_set_exclusive_handler(TIMER0_IRQ_0, gea_pc_sampler_irq);
		timer0_hw->inte |= 1u;
		timer0_hw->alarm[0] = timer0_hw->timerawl + 1000u;
		irq_set_enabled(TIMER0_IRQ_0, true);
	}
};

__attribute__((init_priority(101))) EarlyBootBeacon gEarlyBootBeacon;

}  // namespace
// ----------------------------------------------------------------------------

namespace {

#ifndef GEA_RP2350_SYS_CLOCK_KHZ
// 300 MHz: the software raster/present is pure integer compute and scales ~linearly
// with clock. Stable at 1.20V (below the ~1.30V that 400MHz needs). The QSPI PIO
// divides from clk_sys, so rp2350_panel.cpp pins the panel wire clock with a
// runtime divider to keep the bus at its tuned speed regardless of this overclock.
#define GEA_RP2350_SYS_CLOCK_KHZ 300000
#endif

#ifndef GEA_RP2350_VREG_VOLTAGE
#define GEA_RP2350_VREG_VOLTAGE VREG_VOLTAGE_1_20
#endif

#ifndef GEA_RP2350_HAS_TOUCH
#define GEA_RP2350_HAS_TOUCH 1
#endif

#ifndef GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO
#define GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO 2.0
#endif

#ifndef GEA_RP2350_STARTUP_TRACE
#define GEA_RP2350_STARTUP_TRACE 0
#endif

#ifndef GEA_RP2350_FRAME_PHASE_PERF
#define GEA_RP2350_FRAME_PHASE_PERF 0
#endif

#ifndef GEA_RP2350_REFRESH_DETAIL_PERF
#define GEA_RP2350_REFRESH_DETAIL_PERF 0
#endif

#ifndef GEA_RP2350_PERF_LOG
#define GEA_RP2350_PERF_LOG 1
#endif

#if GEA_RP2350_FRAME_PHASE_PERF
struct FramePhaseStats {
	std::uint64_t workUs = 0;
	std::uint64_t inputUs = 0;
	std::uint64_t microtaskUs = 0;
	std::uint64_t rafUs = 0;
	std::uint64_t cssUs = 0;
	std::uint64_t documentUs = 0;
	std::uint64_t refreshUs = 0;
	std::uint64_t sleepUs = 0;
	std::uint32_t frames = 0;
	std::uint32_t overBudgetFrames = 0;
	std::uint32_t worstWorkUs = 0;
};

FramePhaseStats gFramePhaseStats;
std::uint64_t gFramePhaseStartUs = 0;

void phaseBeginFrame(std::uint64_t startUs)
{
	gFramePhaseStartUs = startUs;
}

void phaseEndFrame(std::uint64_t endUs, std::uint32_t budgetUs)
{
	const auto workUs = static_cast<std::uint32_t>(endUs - gFramePhaseStartUs);
	gFramePhaseStats.workUs += workUs;
	++gFramePhaseStats.frames;
	if (budgetUs > 0 && workUs > budgetUs) ++gFramePhaseStats.overBudgetFrames;
	if (workUs > gFramePhaseStats.worstWorkUs) gFramePhaseStats.worstWorkUs = workUs;
}

void phaseAddSleep(std::uint32_t sleepUs)
{
	gFramePhaseStats.sleepUs += sleepUs;
}

void phaseLogAndReset()
{
	const auto frames = gFramePhaseStats.frames;
	const std::uint64_t avgWorkUs = frames > 0 ? (gFramePhaseStats.workUs + frames / 2u) / frames : 0;
	const std::uint64_t avgRefreshUs = frames > 0 ? (gFramePhaseStats.refreshUs + frames / 2u) / frames : 0;
	std::printf("gea.rp2350.phase frames=%u work_ms=%llu avg_work_us=%llu worst_work_us=%u over_budget=%u input_ms=%llu micro_ms=%llu raf_ms=%llu css_ms=%llu doc_ms=%llu refresh_ms=%llu avg_refresh_us=%llu sleep_ms=%llu\n",
	            static_cast<unsigned>(frames),
	            static_cast<unsigned long long>(gFramePhaseStats.workUs / 1000u),
	            static_cast<unsigned long long>(avgWorkUs),
	            static_cast<unsigned>(gFramePhaseStats.worstWorkUs),
	            static_cast<unsigned>(gFramePhaseStats.overBudgetFrames),
	            static_cast<unsigned long long>(gFramePhaseStats.inputUs / 1000u),
	            static_cast<unsigned long long>(gFramePhaseStats.microtaskUs / 1000u),
	            static_cast<unsigned long long>(gFramePhaseStats.rafUs / 1000u),
	            static_cast<unsigned long long>(gFramePhaseStats.cssUs / 1000u),
	            static_cast<unsigned long long>(gFramePhaseStats.documentUs / 1000u),
	            static_cast<unsigned long long>(gFramePhaseStats.refreshUs / 1000u),
	            static_cast<unsigned long long>(avgRefreshUs),
	            static_cast<unsigned long long>(gFramePhaseStats.sleepUs / 1000u));
	gFramePhaseStats = {};
}
#endif

#if GEA_RP2350_REFRESH_DETAIL_PERF
void refreshDetailLogAndReset(int frames)
{
	const auto stats = gea::embedded::ui::refreshPerfStatsRead();
	const int denom = frames > 0 ? frames : 1;
	std::printf("gea.rp2350.refresh frames=%d layout_us=%lld dlist_us=%lld record_us=%lld dirty_us=%lld coal_us=%lld replay_us=%lld regions=%d origin_regions=%d checks=%d fill_us=%lld circle_us=%lld text_us=%lld grad_us=%lld img_us=%lld other_us=%lld flush_us=%lld snap_us=%lld full_records=%d reprojects=%d recorded_nodes=%d recorded_cmds=%d\n",
	            frames,
	            static_cast<long long>(stats.treeLayoutUs / denom),
	            static_cast<long long>(stats.treeDisplayListUs / denom),
	            static_cast<long long>(stats.treeRecordNodeUs / denom),
	            static_cast<long long>(stats.treeDirtyCollectUs / denom),
	            static_cast<long long>(stats.treeDirtyCoalesceUs / denom),
	            static_cast<long long>(stats.treeReplayUs / denom),
	            stats.treeReplayRegions / denom,
	            stats.treeReplayOriginRegions / denom,
	            stats.treeReplayCommandChecks / denom,
	            static_cast<long long>(stats.treeReplayFillUs / denom),
	            static_cast<long long>(stats.treeReplayCircleUs / denom),
	            static_cast<long long>(stats.treeReplayTextUs / denom),
	            static_cast<long long>(stats.treeReplayGradientUs / denom),
	            static_cast<long long>(stats.treeReplayImageUs / denom),
	            static_cast<long long>(stats.treeReplayOtherUs / denom),
	            static_cast<long long>(stats.treeFlushRectsUs / denom),
	            static_cast<long long>(stats.treeSnapshotUs / denom),
	            stats.treeFullRecords,
	            stats.treeReprojects,
	            stats.treeRecordedNodes / denom,
	            stats.treeRecordedCommands / denom);
	std::printf("gea.rp2350.replay_detail frames=%d direct1=%d directN=%d simple1=%d parallel_try=%d parallel_ok=%d split_bg_us=%lld split_dyn_us=%lld split_raster_us=%lld bg_restore_us=%lld node_walk_us=%lld filter_us=%lld filter_calls=%d clip_us=%lld clip_calls=%d draw_counts=fill:%d circle:%d round:%d xround:%d text:%d other:%d\n",
	            frames,
	            stats.treeReplayDirectRegionCalls / denom,
	            stats.treeReplayDirectRegionsCalls / denom,
	            stats.treeReplaySimpleRegionCalls / denom,
	            stats.treeReplayParallelAttempts / denom,
	            stats.treeReplayParallelSuccesses / denom,
	            static_cast<long long>(stats.treeReplaySplitBgUs / denom),
	            static_cast<long long>(stats.treeReplaySplitDynUs / denom),
	            static_cast<long long>(stats.treeReplaySplitRasterUs / denom),
	            static_cast<long long>(stats.treeReplayBgRestoreUs / denom),
	            static_cast<long long>(stats.treeReplayNodeWalkUs / denom),
	            static_cast<long long>(stats.treeReplayCommandFilterUs / denom),
	            stats.treeReplayCommandFilterCalls / denom,
	            static_cast<long long>(stats.treeReplayCommandClipUs / denom),
	            stats.treeReplayCommandClipCalls / denom,
	            stats.treeReplayFillRectCommands / denom,
	            stats.treeReplayCircleCommands / denom,
	            stats.treeReplayRoundedRectCommands / denom,
	            stats.treeReplayTransformedRoundedRectCommands / denom,
	            stats.treeReplayTextCommands / denom,
	            stats.treeReplayOtherCommands / denom);
	std::printf("gea.rp2350.layout frames=%d node_calls=%d memo_hits=%d scoped=%d repositions=%d text_us=%lld set_style=%d/%d set_text=%d/%d layout_changed=%d\n",
	            frames,
	            stats.treeLayoutNodeCalls,
	            stats.treeLayoutMemoHits,
	            stats.treeScopedLayouts,
	            stats.treeLayoutRepositionCalls,
	            static_cast<long long>(stats.treeLayoutTextUs),
	            stats.treeSetStyleCalls,
	            stats.treeSetStyleChanged,
	            stats.treeSetTextCalls,
	            stats.treeSetTextChanged,
	            stats.treeSetStyleLayoutChanged);
	std::printf("gea.rp2350.abs_mode frames=%d calls=%d fast=%d full=%d noop=%d dirty=%d reject_node=%d reject_reason=%d scoped_reject=%d/%d\n",
	            frames,
	            stats.treeAbsModeCalls,
	            stats.treeAbsModeFast,
	            stats.treeAbsModeFull,
	            stats.treeAbsModeNoop,
	            stats.treeAbsModeDirtyNodes,
	            stats.treeAbsModeRejectNode,
	            stats.treeAbsModeRejectReason,
	            stats.treeScopedRejectReason,
	            stats.treeScopedRejectNode);
	std::printf("gea.rp2350.root_scroll frames=%d calls=%d accepted=%d rejected=%d scan_us=%lld rebuild_us=%lld scrollrect_us=%lld strip_us=%lld scrollbar_us=%lld full_replay_us=%lld stream_us=%lld extra_us=%lld flush_us=%lld move_px=%d strip_px=%d viewport_px=%d\n",
	            frames,
	            stats.rootScrollCalls,
	            stats.rootScrollAccepted,
	            stats.rootScrollRejected,
	            static_cast<long long>(stats.rootScrollScanUs / denom),
	            static_cast<long long>(stats.rootScrollRebuildUs / denom),
	            static_cast<long long>(stats.rootScrollScrollRectUs / denom),
	            static_cast<long long>(stats.rootScrollStripReplayUs / denom),
	            static_cast<long long>(stats.rootScrollScrollbarReplayUs / denom),
	            static_cast<long long>(stats.rootScrollFullReplayUs / denom),
	            static_cast<long long>(stats.rootScrollStreamUs / denom),
	            static_cast<long long>(stats.rootScrollExtraReplayUs / denom),
	            static_cast<long long>(stats.rootScrollFlushUs / denom),
	            stats.rootScrollMovePx,
	            stats.rootScrollStripPx,
	            stats.rootScrollViewportPx);
	std::printf("gea.rp2350.text_cache frames=%d calls=%d hits=%d misses=%d fallback=%d sram_uses=%d psram_uses=%d cached_bytes=%d sram_bytes=%d max_entry=%d\n",
	            frames,
	            stats.projectedTextCacheCalls,
	            stats.projectedTextCacheHits,
	            stats.projectedTextCacheMisses,
	            stats.projectedTextCacheFallbacks,
	            stats.projectedTextCacheSramUses,
	            stats.projectedTextCachePsramUses,
	            stats.projectedTextCacheBytes,
	            stats.projectedTextCacheSramBytes,
	            stats.projectedTextCacheMaxEntryBytes);
	gea::embedded::ui::refreshPerfStatsReset();
}
#endif

void configureClock()
{
	constexpr std::uint32_t kSysClockKhz = GEA_RP2350_SYS_CLOCK_KHZ;
	if constexpr (kSysClockKhz > 150000) {
		vreg_set_voltage(GEA_RP2350_VREG_VOLTAGE);
		sleep_ms(2);
	}
	set_sys_clock_khz(kSysClockKhz, true);
	clock_configure(clk_peri,
	                0,
	                CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
	                kSysClockKhz * 1000,
	                kSysClockKhz * 1000);
}

void runFrame(int timestampMs)
{
	static bool cssAnimationsStarted = false;
	const auto nowMs = static_cast<std::uint32_t>(timestampMs);
#if GEA_RP2350_FRAME_PHASE_PERF
	std::uint64_t phaseUs = time_us_64();
#endif
	if (!cssAnimationsStarted) {
		gea::css::DeclarativeAnimations::scanAndStart(nowMs);
		gea::embedded::ui::StyleSheet::instance().startCssAnimations(nowMs);
		cssAnimationsStarted = true;
	}

	gea::rp2350::pollTouch();
	gea::rp2350::pollButtons();
	gea::rp2350::dispatchQueuedEvents();
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.inputUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	// Coalesce class-style recomputes from this frame's reactive updates
	// (microtasks + rAF): a city/theme switch flips a near-root class plus a
	// few descendant classes, and unbatched each flip re-walks its subtree
	// against the stylesheet. endStyleMountBatch() below runs one coalesced,
	// dependency-filtered pass before layout/paint. Same bracketing as
	// gea_app_entry.cpp's frame().
	gea::embedded::ui::beginStyleMountBatch();
	gea::framework::app::generated::drainMicrotasks();
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.microtaskUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	gea::host::runAnimationFrameCallbacks(static_cast<double>(timestampMs));
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.rafUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	gea::css::AnimationEngine::instance().tick(nowMs);
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.cssUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	gea::rp2350::dispatchQueuedEvents();
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.inputUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	// Apply the coalesced recompute so layout/paint below see final styles.
	gea::embedded::ui::endStyleMountBatch();
	gea::embedded::ui::Document::instance().frame(timestampMs);
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.documentUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	gea::framework::app::generated::drainMicrotasks();
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.microtaskUs += nowUs - phaseUs;
		phaseUs = nowUs;
	}
#endif
	gea::embedded::ui::Document::instance().refreshMountedIfDirty();
#if GEA_RP2350_FRAME_PHASE_PERF
	{
		const std::uint64_t nowUs = time_us_64();
		gFramePhaseStats.refreshUs += nowUs - phaseUs;
	}
#endif
}

void logFlushStats(int timestampMs)
{
#if GEA_RP2350_PERF_LOG
	static int lastLogMs = -1;
	static int frames = 0;
	if (lastLogMs < 0) {
		lastLogMs = timestampMs;
		gea::rp2350::panelFlushStatsReset();
		return;
	}

	++frames;
	const int elapsedMs = timestampMs - lastLogMs;
	if (elapsedMs < 1000) return;

	const auto stats = gea::rp2350::panelFlushStatsRead();
	const int fps = elapsedMs > 0 ? (frames * 1000 + elapsedMs / 2) / elapsedMs : 0;
	std::printf("gea.rp2350.perf fps=%d frames=%d ms=%d flush_calls=%d flush_kpx=%d chunks=%d total_ms=%lld window_ms=%lld copy_ms=%lld txwait_ms=%lld\n",
	            fps,
	            frames,
	            elapsedMs,
	            stats.callCount,
	            (stats.pixelCount + 999) / 1000,
	            stats.chunkCount,
	            static_cast<long long>(stats.totalUs / 1000),
	            static_cast<long long>(stats.setWindowUs / 1000),
	            static_cast<long long>(stats.copyUs / 1000),
	            static_cast<long long>(stats.txWaitUs / 1000));
	std::printf("gea.rp2350.alloc sram_allocs=%u psram_allocs=%u sram_live=%u psram_live=%u sram_peak=%u psram_peak=%u psram_free=%u\n",
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationSramCount()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationPsramCount()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationSramBytes()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationPsramBytes()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationSramPeakBytes()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::allocationPsramPeakBytes()),
	            static_cast<unsigned>(gea::framework::memory::MemoryBackend::psramFree()));
#if GEA_RP2350_FRAME_PHASE_PERF
	phaseLogAndReset();
#endif
#if GEA_RP2350_REFRESH_DETAIL_PERF
	refreshDetailLogAndReset(frames);
#endif
	gea::rp2350::panelFlushStatsReset();
	lastLogMs = timestampMs;
	frames = 0;
#else
	(void)timestampMs;
#endif
}

#if GEA_RP2350_STARTUP_TRACE
void waitForUsbConsole(std::uint32_t timeoutMs = 8000)
{
	const std::uint64_t deadlineUs = time_us_64() + static_cast<std::uint64_t>(timeoutMs) * 1000u;
	while (!stdio_usb_connected() && time_us_64() < deadlineUs) {
		sleep_ms(10);
	}
}

void startupLog(const char *stage, std::uint64_t startUs = 0)
{
	const std::uint64_t nowUs = time_us_64();
	if (startUs > 0) {
		std::printf("GEA_START %s t=%llu ms delta=%llu ms\n",
		            stage,
		            static_cast<unsigned long long>(nowUs / 1000u),
		            static_cast<unsigned long long>((nowUs - startUs) / 1000u));
	} else {
		std::printf("GEA_START %s t=%llu ms\n",
		            stage,
		            static_cast<unsigned long long>(nowUs / 1000u));
	}
	std::fflush(stdout);
}
#endif

class Base64Writer {
public:
	void writeByte(std::uint8_t byte)
	{
		pending_[pendingLength_++] = byte;
		if (pendingLength_ == 3) flushTriplet(3);
	}

	void finish()
	{
		if (pendingLength_ > 0) flushTriplet(pendingLength_);
		flushLine();
	}

private:
	void flushTriplet(int length)
	{
		const std::uint32_t a = pending_[0];
		const std::uint32_t b = length > 1 ? pending_[1] : 0;
		const std::uint32_t c = length > 2 ? pending_[2] : 0;
		const std::uint32_t value = (a << 16) | (b << 8) | c;

		append(kAlphabet[(value >> 18) & 0x3f]);
		append(kAlphabet[(value >> 12) & 0x3f]);
		append(length > 1 ? kAlphabet[(value >> 6) & 0x3f] : '=');
		append(length > 2 ? kAlphabet[value & 0x3f] : '=');
		pendingLength_ = 0;
	}

	void append(char ch)
	{
		line_[lineLength_++] = ch;
		if (lineLength_ >= kLineLength) flushLine();
	}

	void flushLine()
	{
		if (lineLength_ <= 0) return;
		std::printf("GEADEV:DATA %.*s\n", lineLength_, line_);
		std::fflush(stdout);
		lineLength_ = 0;
	}

	static constexpr int kLineLength = 76;
	static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	std::uint8_t pending_[3]{};
	int pendingLength_ = 0;
	char line_[kLineLength]{};
	int lineLength_ = 0;
};

void writeU16(Base64Writer &writer, std::uint16_t value)
{
	writer.writeByte(static_cast<std::uint8_t>(value & 0xff));
	writer.writeByte(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

void writeRun(Base64Writer &writer, std::uint16_t count, std::uint16_t rgb565)
{
	writeU16(writer, count);
	writeU16(writer, gea::framework::graphics::pixel::toRgb565(rgb565));
}

void writeScreenshot()
{
	using gea::platform::display::Display;

	std::uint16_t *pixels = nullptr;
	int width = 0;
	int height = 0;
	if (!Display::panelScanoutSurface(&pixels, &width, &height) || !pixels || width <= 0 || height <= 0) {
		std::printf("GEADEV:ERR SCREENSHOT display-unavailable\n");
		std::fflush(stdout);
		return;
	}

	const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
	std::printf("GEADEV:SCREENSHOT BEGIN width=%d height=%d encoding=rgb565-rle-v1 app=css-3d-cube\n",
	            width,
	            height);
	std::fflush(stdout);

	Base64Writer writer;
	if (pixelCount > 0) {
		// Serialize in LOGICAL row order (the scroll register's circular remap
		// makes the raw buffer order scroll-dependent) so captures compare
		// across scroll positions.
		auto pixelAt = [&](std::size_t i) {
			const int row = static_cast<int>(i) / width;
			const int col = static_cast<int>(i) % width;
			return pixels[static_cast<std::size_t>(gea::rp2350::scanoutRowToPhysical(row)) * width + col];
		};
		std::uint16_t runValue = pixelAt(0);
		std::uint16_t runCount = 1;
		for (std::size_t i = 1; i < pixelCount; ++i) {
			const std::uint16_t value = pixelAt(i);
			if (value == runValue && runCount < 65535) {
				runCount++;
				continue;
			}
			writeRun(writer, runCount, runValue);
			runValue = value;
			runCount = 1;
		}
		writeRun(writer, runCount, runValue);
	}
	writer.finish();
	std::printf("GEADEV:SCREENSHOT END\n");
	std::fflush(stdout);
}

// Boot-time snapshot, queryable over GEADEV BOOTSTATS: the boot prints happen
// before a host console can attach, so main() stores them here instead.
struct BootStatsSnapshot {
	std::uint32_t topLevelUs = 0;
	std::uint32_t initialRefreshUs = 0;
	std::uint32_t loopEnterUs = 0;
	// PC-ring indices bracketing the boot phases (1 sample/ms).
	int profTopLevelStart = 0;
	int profTopLevelEnd = 0;
	int profLoopEnter = 0;
#if GEA_RP2350_REFRESH_DETAIL_PERF
	int nodeCalls = 0;
	int memoHits = 0;
	long long textUs = 0;
	int setTextCalls = 0;
	int setTextChanged = 0;
	int refreshCalls = 0;
	int fullRecords = 0;
	long long replayUs = 0;
	int parallelTries = 0;
	int parallelOks = 0;
	int directReplays = 0;
#endif
};
BootStatsSnapshot gBootStats;

void printBootStats()
{
	std::printf("GEADEV:BOOTSTATS top_level_us=%lu initial_refresh_us=%lu loop_enter_us=%lu",
	            static_cast<unsigned long>(gBootStats.topLevelUs),
	            static_cast<unsigned long>(gBootStats.initialRefreshUs),
	            static_cast<unsigned long>(gBootStats.loopEnterUs));
#if GEA_RP2350_REFRESH_DETAIL_PERF
	std::printf(" node_calls=%d memo_hits=%d text_us=%lld set_text=%d/%d refreshes=%d full_records=%d replay_us=%lld par=%d/%d direct=%d",
	            gBootStats.nodeCalls,
	            gBootStats.memoHits,
	            gBootStats.textUs,
	            gBootStats.setTextCalls,
	            gBootStats.setTextChanged,
	            gBootStats.refreshCalls,
	            gBootStats.fullRecords,
	            gBootStats.replayUs,
	            gBootStats.parallelTries,
	            gBootStats.parallelOks,
	            gBootStats.directReplays);
#endif
	std::printf("\n");
	if (const char *rec = gea_recprof_last()) std::printf("GEADEV:RECPROF %s\n", rec);
}

void printBootProfile()
{
	const int count = static_cast<int>(gea_pc_ring_ptr - gea_pc_ring);
	std::printf("GEADEV:BOOTPROF BEGIN count=%d tl0=%d tl1=%d loop=%d\n",
	            count, gBootStats.profTopLevelStart, gBootStats.profTopLevelEnd,
	            gBootStats.profLoopEnter);
	for (int i = 0; i < count; i += 16) {
		std::printf("GEADEV:DATA");
		for (int j = i; j < i + 16 && j < count; j++) std::printf(" %08x", static_cast<unsigned>(gea_pc_ring[j]));
		std::printf("\n");
	}
	std::printf("GEADEV:BOOTPROF END\n");
}

void handleDevCommand(char *line)
{
	while (*line == ' ' || *line == '\t') ++line;
	if (std::strncmp(line, "GEADEV", 6) != 0) return;
	line += 6;
	while (*line == ' ' || *line == '\t') ++line;
	if (std::strncmp(line, "PING", 4) == 0) {
		std::printf("GEADEV:PONG app=css-3d-cube\n");
		std::fflush(stdout);
	} else if (std::strncmp(line, "BOOTSTATS", 9) == 0) {
		printBootStats();
		std::fflush(stdout);
	} else if (std::strncmp(line, "BOOTPROF", 8) == 0) {
		printBootProfile();
		std::fflush(stdout);
	} else if (std::strncmp(line, "TREE", 4) == 0) {
		// Dump visible node boxes for layout debugging.
		auto &tree = gea::embedded::ui::Tree::instance();
		std::printf("GEADEV:TREE count=%d root=%d\n", tree.nodeCount(), tree.mountedRoot());
		for (int i = 0; i < tree.nodeCount(); i++) {
			const auto &n = tree.node(i);
			if (n.style.display == 1) continue;
			if (n.layout.width <= 0 || n.layout.height <= 0) continue;
			std::printf("GEADEV:DATA id=%d p=%d t=%d x=%d y=%d w=%d h=%d sy=%d ov=%d\n",
			            i, n.parent, static_cast<int>(n.type),
			            n.layout.x, n.layout.y, n.layout.width, n.layout.height,
			            n.layout.scroll_y, n.style.overflow_y);
		}
		std::printf("GEADEV:TREE END\n");
		std::fflush(stdout);
	} else if (std::strncmp(line, "PROFARM", 7) == 0) {
		// Re-arm the PC sample ring: records the next ~4s of 1ms samples.
		gea_pc_ring_ptr = gea_pc_ring;
		std::printf("GEADEV:OK PROFARM\n");
		std::fflush(stdout);
	} else if (std::strncmp(line, "SCREENSHOT", 10) == 0) {
		writeScreenshot();
	} else if (std::strncmp(line, "VSCROLL", 7) == 0) {
		// Direct panel scroll-register test: the visible image should rotate
		// vertically by <rows> without any pixel data being streamed.
		int rows = 0;
		if (std::sscanf(line + 7, "%d", &rows) != 1) {
			std::printf("GEADEV:ERR VSCROLL usage=GEADEV_VSCROLL_rows\n");
		} else {
			gea::rp2350::panelSetVerticalScroll(rows);
			std::printf("GEADEV:OK VSCROLL rows=%d\n", rows);
		}
		std::fflush(stdout);
	} else if (std::strncmp(line, "DRAG", 4) == 0) {
		int x1 = 0, y1 = 0, x2 = 0, y2 = 0, steps = 24, delayMs = 16;
		const int parsed = std::sscanf(line + 4, "%d %d %d %d %d %d", &x1, &y1, &x2, &y2, &steps, &delayMs);
		if (parsed < 4) {
			std::printf("GEADEV:ERR DRAG usage=GEADEV_DRAG_x1_y1_x2_y2_[steps]_[delay_ms]\n");
		} else if (!gea::rp2350::startSyntheticDrag(x1, y1, x2, y2, steps, delayMs)) {
			std::printf("GEADEV:ERR DRAG touch-unavailable-or-busy\n");
		}
		std::fflush(stdout);
	} else {
		std::printf("GEADEV:ERR unknown-command\n");
		std::fflush(stdout);
	}
}

void pollDevControl()
{
	static char line[96]{};
	static int length = 0;
	while (true) {
		const int ch = getchar_timeout_us(0);
		if (ch == PICO_ERROR_TIMEOUT) break;
		if (ch == '\r') continue;
		if (ch == '\n') {
			line[length] = '\0';
			if (length > 0) handleDevCommand(line);
			length = 0;
			continue;
		}
		if (length < static_cast<int>(sizeof(line)) - 1) {
			line[length++] = static_cast<char>(ch);
		} else {
			length = 0;
		}
	}
}

}  // namespace

int main()
{
	// USB console first, before any clock/board/PSRAM work: if a boot stage
	// crashes, the beacon below tells us we made it out of the C++ static
	// constructors, and the stage logs narrow the rest. (Boot-debug aid; the
	// default clocks are USB-capable, and set_sys_clock_khz leaves clk_usb's
	// PLL alone.)
	stdio_init_all();
	watchdog_hw->scratch[7] = 0;  // phase: static ctors completed, main reached
#if GEA_RP2350_STARTUP_TRACE
	sleep_ms(2500);
	std::printf("GEA_START beacon (pre-clock)\n");
	std::fflush(stdout);
#endif
	configureClock();
#if GEA_RP2350_STARTUP_TRACE
	std::uint64_t stageUs = time_us_64();
	startupLog("clock");
#endif
	gea::rp2350::boardIoInit();
	// Brief settle for the I2C pull-ups / ADC; the old 1500ms here was only a
	// USB-console courtesy and cost every boot (trace builds wait for the
	// console explicitly below).
	sleep_ms(50);
#if GEA_RP2350_STARTUP_TRACE
	startupLog("boardIoInit", stageUs);
	waitForUsbConsole();
	startupLog("stdio");
	stageUs = time_us_64();
#endif
	gea::rp2350::memoryInit();
#if GEA_RP2350_STARTUP_TRACE
	startupLog("memoryInit", stageUs);
#endif
	if (gea_plugin_cpp_register_prelude) gea_plugin_cpp_register_prelude();

	std::printf("Gea RP2350 launching app\n");
#if GEA_RP2350_STARTUP_TRACE
	stageUs = time_us_64();
#endif
	gea::platform::display::Display::init();
#if GEA_RP2350_STARTUP_TRACE
	startupLog("displayInit", stageUs);
#endif
	gea::embedded::ui::Document::setPreferredMountSize(gea::rp2350::kPanelWidth, gea::rp2350::kPanelHeight);
	gea::embedded::ui::setViewportMetrics(gea::rp2350::kPanelWidth,
	                                       gea::rp2350::kPanelHeight,
	                                       GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO);
#if GEA_RP2350_STARTUP_TRACE
	stageUs = time_us_64();
#endif
	auto queue = gea::framework::services::FrameScheduler::createEventQueue();
	gea::framework::services::FrameScheduler::start(queue);
#if GEA_RP2350_HAS_TOUCH
	gea::framework::events::TouchRuntime::start();
#endif
#if GEA_RP2350_STARTUP_TRACE
	startupLog("scheduler", stageUs);
	stageUs = time_us_64();
#endif

	const std::uint64_t topLevelStartUs = time_us_64();
	gBootStats.profTopLevelStart = static_cast<int>(gea_pc_ring_ptr - gea_pc_ring);
	// Defer per-node class-style recomputes for the whole initial mount:
	// without the batch every setClassName/appendChild re-walks the growing
	// subtree against the stylesheet — O(nodes^2 x rules) (this was ~420ms of
	// weather's boot). endStyleMountBatch() runs ONE coalesced pass instead.
	// Same bracketing as gea_app_entry.cpp's launcher path.
	gea::embedded::ui::beginStyleMountBatch();
	__gea_top_level();
	gea::embedded::ui::endStyleMountBatch();
	gBootStats.profTopLevelEnd = static_cast<int>(gea_pc_ring_ptr - gea_pc_ring);
	gBootStats.topLevelUs = static_cast<std::uint32_t>(time_us_64() - topLevelStartUs);
#if GEA_RP2350_STARTUP_TRACE
	startupLog("topLevel", stageUs);
	stageUs = time_us_64();
#endif
	gea::framework::app::generated::drainMicrotasks();
#if GEA_RP2350_STARTUP_TRACE
	startupLog("initialDrain", stageUs);
	stageUs = time_us_64();
#endif
	const std::uint64_t initialRefreshStartUs = time_us_64();
	// The mount inside __gea_top_level() ran while the style batch was open, so
	// Tree::mount skipped its layout/paint (that frame would have been
	// unstyled). Re-mount now for the first styled present; the refresh below
	// is then a no-op unless the drain above dirtied something.
	{
		auto &tree = gea::embedded::ui::Tree::instance();
		const int root = tree.mountedRoot();
		if (root >= 0) tree.mount(root, tree.mountedWidth(), tree.mountedHeight());
	}
	gea::embedded::ui::Document::instance().refreshMountedIfDirty();
	gBootStats.initialRefreshUs = static_cast<std::uint32_t>(time_us_64() - initialRefreshStartUs);
	gBootStats.loopEnterUs = static_cast<std::uint32_t>(time_us_64());
	gBootStats.profLoopEnter = static_cast<int>(gea_pc_ring_ptr - gea_pc_ring);
#if GEA_RP2350_STARTUP_TRACE
	startupLog("initialRefresh", stageUs);
#endif
	gea::rp2350::panelFlushStatsReset();
#if GEA_RP2350_REFRESH_DETAIL_PERF
	{
		// Boot totals (mount + first layout), queryable later via GEADEV BOOTSTATS
		// (this runs before a host console can attach, so printing is useless).
		const auto bootPerf = gea::embedded::ui::refreshPerfStatsRead();
		gBootStats.nodeCalls = bootPerf.treeLayoutNodeCalls;
		gBootStats.memoHits = bootPerf.treeLayoutMemoHits;
		gBootStats.textUs = static_cast<long long>(bootPerf.treeLayoutTextUs);
		gBootStats.setTextCalls = bootPerf.treeSetTextCalls;
		gBootStats.setTextChanged = bootPerf.treeSetTextChanged;
		gBootStats.refreshCalls = bootPerf.treeRefreshCalls;
		gBootStats.fullRecords = bootPerf.treeFullRecords;
		gBootStats.replayUs = static_cast<long long>(bootPerf.treeReplayUs);
		gBootStats.parallelTries = bootPerf.treeReplayParallelAttempts;
		gBootStats.parallelOks = bootPerf.treeReplayParallelSuccesses;
		gBootStats.directReplays = bootPerf.treeDirectReplayCalls;
	}
	gea::embedded::ui::refreshPerfStatsReset();
#endif
#if GEA_RP2350_STARTUP_TRACE
	startupLog("loop");
#endif

	while (true) {
		const std::uint64_t frameStartUs = time_us_64();
		const int nowMs = static_cast<int>(frameStartUs / 1000u);
#if GEA_RP2350_FRAME_PHASE_PERF
		phaseBeginFrame(frameStartUs);
#endif
		runFrame(nowMs);
		pollDevControl();
		const int intervalMs = gea::framework::services::FrameScheduler::frameIntervalMs();
		if (intervalMs > 0) {
			const std::uint64_t frameBudgetUs = static_cast<std::uint64_t>(intervalMs) * 1000u;
			std::uint64_t elapsedUs = time_us_64() - frameStartUs;
			// Idle-time image pre-decode: drain one deferred decode per mostly-idle
			// frame so lazy-loaded assets (decodeDeferred) never bunch their decode
			// cost into a later first-draw frame. Runs on core 0 while the core-1
			// raster worker is parked, so ImageStore access stays single-threaded.
			if (elapsedUs * 2 < frameBudgetUs &&
			    gea::framework::graphics::ImageStore::instance().materializeOneDeferred()) {
				elapsedUs = time_us_64() - frameStartUs;
			}
#if GEA_RP2350_FRAME_PHASE_PERF
			phaseEndFrame(frameStartUs + elapsedUs, static_cast<std::uint32_t>(frameBudgetUs));
#endif
			logFlushStats(nowMs);
			if (elapsedUs < frameBudgetUs) {
				const auto sleepDurationUs = static_cast<std::uint32_t>(frameBudgetUs - elapsedUs);
				sleep_us(sleepDurationUs);
#if GEA_RP2350_FRAME_PHASE_PERF
				phaseAddSleep(sleepDurationUs);
#endif
			}
		} else {
#if GEA_RP2350_FRAME_PHASE_PERF
			phaseEndFrame(time_us_64(), 0);
#endif
			logFlushStats(nowMs);
			sleep_us(1000);
#if GEA_RP2350_FRAME_PHASE_PERF
			phaseAddSleep(1000);
#endif
		}
	}
}
