#include "services/frame_scheduler.h"

#include "app.h"
#include "display.h"
#include "host/timers.h"
#include "ui/refresh_trace.h"
#include "ui/refresh_perf.h"
#include "ui/document.h"
#include "memory_config.h"
#include "services/app_state.h"
#include "ui/canvas_element.h"

#include <atomic>
#include <cstdint>
#include <limits>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace gea::framework::services {

namespace {

// [DIAG A/B] fused-gradient-blend optimisation phase. Pinned to 0 = fused path OFF
// (ship the proven constant-alpha gather; the fused kernel measured +0). Flip the
// initial value (and uncomment the ^=1 below) to re-A/B fused vs gather.
volatile int g_diagAbPhase = 0;

constexpr const char *kTag = "gea_esp32_frame";
constexpr int kTaskStackWords = GEA_EMBEDDED_APP_FRAME_TASK_STACK_WORDS;
// GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL=1 allocates the frame task's stack
// from external RAM in start() instead of carrying it inside this static
// engine, which is internal SRAM. The task never runs with the flash cache
// disabled when the app executes from external RAM (flash writes are
// serialized elsewhere), so the stack may live outside; the control block and
// the ISR-fed event queue stay in the engine, where FreeRTOS requires them.
// Needs CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY.
#ifndef GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL
#define GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL 0
#endif
constexpr int kTaskPriority = GEA_EMBEDDED_APP_FRAME_TASK_PRIORITY;
#ifndef GEA_EMBEDDED_APP_FRAME_TASK_CORE
// -1 = tskNO_AFFINITY (run on either core). Boards with BT/WiFi pinned to a
// specific core can override this to put the frame task on the other core,
// eliminating preemption from BT host / NimBLE / Wi-Fi background tasks.
#define GEA_EMBEDDED_APP_FRAME_TASK_CORE -1
#endif
constexpr int kTaskCore = GEA_EMBEDDED_APP_FRAME_TASK_CORE;

#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES
#define GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES 0
#endif

#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_IDLE_YIELD_TICKS
#define GEA_EMBEDDED_FRAME_SCHEDULER_IDLE_YIELD_TICKS 1
#endif

#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD
#define GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD 0
#endif

#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
#define GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER 0
#endif

// Per-drop TE-miss flush-breakdown diagnostic (kept for profiling flush spikes).
// Default OFF: it emits an ESP_LOGW on the frame task for EVERY missed VBlank, and on a
// frame that only just overruns the TE that log can itself cascade further drops —
// perturbing the very cadence it measures. Set to 1 to profile; leave 0 for clean fps.
#ifndef GEA_EMBEDDED_TE_MISS_LOG
#define GEA_EMBEDDED_TE_MISS_LOG 0
#endif

enum class FrameStage {
	Idle,
	FrameBegin,
	WaitAppLock,
	AppFrame,
	AppUnlock,
	ReadInFrameStats,
	TrailFlush,
	ReadTrailStats,
	FrameDone,
};

class FrameSchedulerEngine {
public:
	static FrameSchedulerEngine &instance()
	{
		static FrameSchedulerEngine engine;
		return engine;
	}

	EventQueue createEventQueue()
	{
		eventQueue_ = xQueueCreateStatic(
			FrameScheduler::kEventQueueDepth,
			sizeof(gea::framework::events::Event),
			eventQueueBuffer_,
			&eventQueueStorage_);
		return EventQueue(eventQueue_);
	}

	EventQueue eventQueue() const
	{
		return EventQueue(eventQueue_);
	}

	bool sendEvent(const gea::framework::events::Event &event, int waitMs)
	{
		const TickType_t wait = waitMs <= 0 ? 0 : pdMS_TO_TICKS(waitMs);
		return eventQueue_ && xQueueSend(eventQueue_, &event, wait) == pdPASS;
	}

	bool receiveEvent(gea::framework::events::Event *event)
	{
		return eventQueue_ && event && xQueueReceive(eventQueue_, event, portMAX_DELAY) == pdPASS;
	}

	// Graceful-overrun catch-up: the event loop calls takeCatchUpRequest() after a
	// frame; if the frame overran the budget it runs the next one back-to-back.
	bool takeCatchUpRequest() { return catchUpRequest_.exchange(false, std::memory_order_acq_rel); }

	void setVsyncDriven(bool driven) { vsyncDriven_.store(driven, std::memory_order_release); }
	bool vsyncDriven() const { return vsyncDriven_.load(std::memory_order_acquire); }

	// Called from the panel's TE (VBlank) edge ISR when TE-sync is enabled: this is the
	// SOLE frame producer in that mode. Posts one Frame event per edge, coalesced — if a
	// frame is still queued or running (overrun), the edge is dropped rather than piling
	// up. Records the post time so the timer can detect a dead TE and fall back.
	void notifyVsyncFromISR()
	{
		if (!eventQueue_)
			return;
		lastVsyncPostUs_.store(static_cast<uint32_t>(esp_timer_get_time()), std::memory_order_release);
		if (eventPending_.load(std::memory_order_acquire))
			return;
		if (frameInProgress_.load(std::memory_order_acquire))
			return;
		gea::framework::events::Event event{};
		event.type = gea::framework::events::EventType::Frame;
		eventPending_.store(true, std::memory_order_release);
		BaseType_t needYield = pdFALSE;
		if (xQueueSendFromISR(eventQueue_, &event, &needYield) != pdTRUE)
			eventPending_.store(false, std::memory_order_release);
		else if (needYield == pdTRUE)
			portYIELD_FROM_ISR();
	}

	// Before each back-to-back catch-up frame: discard the redundant Frame events
	// the timer keeps posting (we're driving frames ourselves), and return true the
	// moment a non-Frame event (touch/input) is waiting so the caller stops catching
	// up and lets the normal event loop service it — input is never starved.
	bool drainFramesAndCheckInput()
	{
		if (!eventQueue_) return false;
		gea::framework::events::Event peeked{};
		while (xQueuePeek(eventQueue_, &peeked, 0) == pdPASS) {
			if (peeked.type != gea::framework::events::EventType::Frame) return true;
			gea::framework::events::Event discard{};
			xQueueReceive(eventQueue_, &discard, 0);
			eventPending_.store(false, std::memory_order_release);
		}
		return false;
	}

	void start(EventQueue queue)
	{
		eventQueue_ = static_cast<QueueHandle_t>(queue.nativeHandle());
#if GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
		startTimer();
#else
		if (task_) return;
#if GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL
		if (!taskStack_) {
			const std::size_t bytes = static_cast<std::size_t>(kTaskStackWords) * sizeof(StackType_t);
			taskStack_ = static_cast<StackType_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
			if (!taskStack_) taskStack_ = static_cast<StackType_t *>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (!taskStack_) {
				ESP_LOGW(kTag, "Failed to allocate the app frame task stack");
				return;
			}
		}
#endif
		if (kTaskCore < 0) {
			task_ = xTaskCreateStatic(
				&FrameSchedulerEngine::taskMain,
				"app_frame",
				kTaskStackWords,
				this,
				kTaskPriority,
				taskStack_,
				&taskStorage_);
		} else {
			task_ = xTaskCreateStaticPinnedToCore(
				&FrameSchedulerEngine::taskMain,
				"app_frame",
				kTaskStackWords,
				this,
				kTaskPriority,
				taskStack_,
				&taskStorage_,
				static_cast<BaseType_t>(kTaskCore));
		}
		if (!task_) {
			ESP_LOGW(kTag, "Failed to start app frame task");
			return;
		}
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		ESP_LOGI(kTag,
			"stack probe [frame_scheduler:after_app_frame_task] task=app_frame priority=%d stack_arg_words=%d hwm=%u",
			kTaskPriority,
			kTaskStackWords,
			static_cast<unsigned>(uxTaskGetStackHighWaterMark(task_)));
#endif
#endif
	}

	void runFrame(const FrameScheduler::FrameCallbacks &callbacks)
	{
		// frameStartUs is FUNCTIONAL (drives lastFrameStartUs_, frameStartedMs_, and
		// the watchdog), so it stays. Only the perf period accumulation is gated.
		const int64_t frameStartUs = esp_timer_get_time();
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		if (lastFrameStartUs_ > 0) {
			perf_.framePeriodUs += frameStartUs - lastFrameStartUs_;
			perf_.framePeriodCount++;
		}
#endif
		lastFrameStartUs_ = frameStartUs;
		frameSequence_.fetch_add(1, std::memory_order_relaxed);
		frameStartedMs_.store(static_cast<int>(frameStartUs / 1000), std::memory_order_relaxed);
		setFrameStage(FrameStage::FrameBegin);
		frameInProgress_.store(true, std::memory_order_release);
		eventPending_.store(false, std::memory_order_release);

		gea::platform::display::Display::flushStatsReset();
		// Apply any pending elastic-RAM staging resize requested off-task (e.g. the
		// WiFi/BLE bring-up worker's reserveInternal). Done here, on the frame task,
		// every frame so the shrink lands even on a fully static screen that never
		// enters a content flush — otherwise esp_wifi_init can NO_MEM waiting for
		// internal DMA RAM the staging buffers are still holding.
		gea::platform::display::Display::applyPendingInternalReserve();
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		// These per-frame stat accumulators are only WRITTEN when perf is on (raf
		// under GEA_EMBEDDED_RAF_PERF, canvas/refresh via their detail flags,
		// gFramePerfStats via recordFramePhase) and only READ by the harvest below —
		// all governed by GEA_EMBEDDED_PERF. Resetting them with the flag off is
		// pure dead work, so the resets are gated too.
		gea::host::animationFramePerfStatsReset();
		gea::embedded::ui::canvasPerfStatsReset();
		gea::embedded::ui::refreshPerfStatsReset();
		gea::framework::app::applicationFramePerfStatsReset();
#endif
		setFrameStage(FrameStage::WaitAppLock);
		AppState::lock();
		setFrameStage(FrameStage::AppFrame);
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		const int64_t appStartUs = esp_timer_get_time();
#endif
		if (callbacks.frame) callbacks.frame(FrameScheduler::nowMs(), callbacks.context);
		if (vsyncDriven_.load(std::memory_order_acquire)) {
#if !defined(GEA_EMBEDDED_DIRECT_CANVAS_CONTEXT) || !GEA_EMBEDDED_DIRECT_CANVAS_CONTEXT
			// The phase-set is kept (watchdog stall report); only the timing reads +
			// the perf accumulation around the real refresh are perf machinery.
			gea::framework::app::applicationFramePhaseSet(gea::framework::app::ApplicationFramePhase::RefreshMounted);
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
			const int64_t refreshStartUs = esp_timer_get_time();
#endif
			gea::embedded::ui::Document::instance().refreshMountedIfDirty();
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
			gea::framework::app::applicationFramePerfStatsAdd(
				gea::framework::app::ApplicationFramePhase::RefreshMounted,
				esp_timer_get_time() - refreshStartUs);
#endif
			gea::framework::app::applicationFramePhaseSet(gea::framework::app::ApplicationFramePhase::Idle);
#endif
		}
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		perf_.appTotalUs += esp_timer_get_time() - appStartUs;
		const gea::framework::app::ApplicationFramePerfStats appStats = gea::framework::app::applicationFramePerfStatsRead();
		perf_.drainMicrotasksUs += appStats.drainMicrotasksUs;
		perf_.animationFrameUs += appStats.animationFrameUs;
		perf_.styleRecomputeUs += appStats.styleRecomputeUs;
		perf_.documentFrameUs += appStats.documentFrameUs;
		perf_.refreshMountedUs += appStats.refreshMountedUs;
		const gea::host::AnimationFramePerfStats rafStats = gea::host::animationFramePerfStatsRead();
		perf_.rafTotalUs += rafStats.totalUs;
		perf_.rafPrepareUs += rafStats.prepareUs;
		perf_.rafCallbackUs += rafStats.callbackUs;
		if (rafStats.maxCallbackUs > perf_.rafMaxCallbackUs) perf_.rafMaxCallbackUs = rafStats.maxCallbackUs;
		perf_.rafPendingAtRun += rafStats.pendingAtRun;
		perf_.rafCallbackCount += rafStats.callbackCount;
		perf_.rafRequestCount += rafStats.requestCount;
		perf_.rafDroppedCount += rafStats.droppedCount;
		const gea::embedded::ui::CanvasPerfStats canvasStats = gea::embedded::ui::canvasPerfStatsRead();
		perf_.canvasLookupUs += canvasStats.canvasLookupUs;
		perf_.canvasLookupCalls += canvasStats.canvasLookupCalls;
		perf_.markDirtyUs += canvasStats.markDirtyUs;
		perf_.markDirtyCalls += canvasStats.markDirtyCalls;
		perf_.clearRectUs += canvasStats.clearRectUs;
		perf_.clearRectCalls += canvasStats.clearRectCalls;
		perf_.fillRectUs += canvasStats.fillRectUs;
		perf_.fillRectCalls += canvasStats.fillRectCalls;
		perf_.fillCircleUs += canvasStats.fillCircleUs;
		perf_.fillCircleCalls += canvasStats.fillCircleCalls;
		perf_.fillTriangleUs += canvasStats.fillTriangleUs;
		perf_.fillTriangleCalls += canvasStats.fillTriangleCalls;
		perf_.drawImageUs += canvasStats.drawImageUs;
		perf_.drawImageCalls += canvasStats.drawImageCalls;
		perf_.fillTextUs += canvasStats.fillTextUs;
		perf_.fillTextCalls += canvasStats.fillTextCalls;
		perf_.batchFlushUs += canvasStats.batchFlushUs;
		perf_.batchFlushCalls += canvasStats.batchFlushCalls;
		const gea::embedded::ui::RefreshPerfStats refreshStats = gea::embedded::ui::refreshPerfStatsRead();
		const bool hadScrollUpdate =
			refreshStats.rootScrollMovePx != 0 ||
			refreshStats.rootScrollViewportPx > 0 ||
			refreshStats.rootScrollStripPx > 0 ||
			refreshStats.rootScrollAccepted > 0 ||
			refreshStats.rootScrollStreamFrames > 0 ||
			refreshStats.rootScrollSyncFrames > 0;
		perf_.rootScrollScanUs += refreshStats.rootScrollScanUs;
		perf_.rootScrollRebuildUs += refreshStats.rootScrollRebuildUs;
		perf_.rootScrollScrollRectUs += refreshStats.rootScrollScrollRectUs;
		perf_.rootScrollStripReplayUs += refreshStats.rootScrollStripReplayUs;
		perf_.rootScrollScrollbarReplayUs += refreshStats.rootScrollScrollbarReplayUs;
		perf_.rootScrollFullReplayUs += refreshStats.rootScrollFullReplayUs;
		perf_.rootScrollStreamUs += refreshStats.rootScrollStreamUs;
		perf_.rootScrollExtraReplayUs += refreshStats.rootScrollExtraReplayUs;
		perf_.rootScrollFlushUs += refreshStats.rootScrollFlushUs;
		perf_.rootScrollSnapshotUs += refreshStats.rootScrollSnapshotUs;
		perf_.virtualListRegionFillUs += refreshStats.virtualListRegionFillUs;
		perf_.virtualListRegionTextUs += refreshStats.virtualListRegionTextUs;
		perf_.virtualListRegionThumbUs += refreshStats.virtualListRegionThumbUs;
		perf_.treeLayoutModeUs += refreshStats.treeLayoutModeUs;
		perf_.treeLayoutUs += refreshStats.treeLayoutUs;
		perf_.treeDisplayListUs += refreshStats.treeDisplayListUs;
		perf_.treeRecordNodeUs += refreshStats.treeRecordNodeUs;
		perf_.treeSetStyleUs += refreshStats.treeSetStyleUs;
		perf_.treeSetTextUs += refreshStats.treeSetTextUs;
		perf_.treeAbsModeUs += refreshStats.treeAbsModeUs;
		perf_.treeDirtyCollectUs += refreshStats.treeDirtyCollectUs;
		perf_.treeDirtyCoalesceUs += refreshStats.treeDirtyCoalesceUs;
			perf_.treeReplayUs += refreshStats.treeReplayUs;
			perf_.treeReplayFillUs += refreshStats.treeReplayFillUs;
			perf_.treeReplayCircleUs += refreshStats.treeReplayCircleUs;
			perf_.treeReplayTextUs += refreshStats.treeReplayTextUs;
			perf_.treeReplayGradientUs += refreshStats.treeReplayGradientUs;
			perf_.treeReplayImageUs += refreshStats.treeReplayImageUs;
			perf_.treeReplayOtherUs += refreshStats.treeReplayOtherUs;
				perf_.treeReplaySplitBgUs += refreshStats.treeReplaySplitBgUs;
				perf_.treeReplaySplitDynUs += refreshStats.treeReplaySplitDynUs;
				perf_.treeReplaySplitRasterUs += refreshStats.treeReplaySplitRasterUs;
				perf_.treeReplayBgRestoreUs += refreshStats.treeReplayBgRestoreUs;
				perf_.treeReplayNodeWalkUs += refreshStats.treeReplayNodeWalkUs;
				perf_.treeReplayCommandFilterUs += refreshStats.treeReplayCommandFilterUs;
				perf_.treeReplayCommandClipUs += refreshStats.treeReplayCommandClipUs;
				perf_.treeFlushRectsUs += refreshStats.treeFlushRectsUs;
				perf_.treeSnapshotUs += refreshStats.treeSnapshotUs;
				perf_.treeReplayRegions += refreshStats.treeReplayRegions;
				perf_.treeReplayOriginRegions += refreshStats.treeReplayOriginRegions;
				perf_.treeReplayCommandChecks += refreshStats.treeReplayCommandChecks;
				perf_.treeReplayDirectRegionCalls += refreshStats.treeReplayDirectRegionCalls;
				perf_.treeReplayDirectRegionsCalls += refreshStats.treeReplayDirectRegionsCalls;
				perf_.treeReplaySimpleRegionCalls += refreshStats.treeReplaySimpleRegionCalls;
				perf_.treeReplayCommandFilterCalls += refreshStats.treeReplayCommandFilterCalls;
				perf_.treeReplayCommandClipCalls += refreshStats.treeReplayCommandClipCalls;
				perf_.treeReplayCircleCommands += refreshStats.treeReplayCircleCommands;
			perf_.treeBgRecolorCalls += refreshStats.treeBgRecolorCalls;
			perf_.treeBgRecolorPixels += refreshStats.treeBgRecolorPixels;
			perf_.treeSetStyleCalls += refreshStats.treeSetStyleCalls;
			perf_.treeSetStyleChanged += refreshStats.treeSetStyleChanged;
			perf_.treeSetStyleNoop += refreshStats.treeSetStyleNoop;
			perf_.treeSetStyleLayoutChanged += refreshStats.treeSetStyleLayoutChanged;
			perf_.treeSetStyleTransformChanged += refreshStats.treeSetStyleTransformChanged;
			perf_.treeSetStylePaintChanged += refreshStats.treeSetStylePaintChanged;
			perf_.treeSetTextCalls += refreshStats.treeSetTextCalls;
			perf_.treeSetTextChanged += refreshStats.treeSetTextChanged;
			perf_.treeSetTextStable += refreshStats.treeSetTextStable;
			perf_.treeSetTextHidden += refreshStats.treeSetTextHidden;
			perf_.treeMarkDisplayListDirtyCalls += refreshStats.treeMarkDisplayListDirtyCalls;
			perf_.treeMarkDisplayListContentDirtyCalls += refreshStats.treeMarkDisplayListContentDirtyCalls;
			perf_.treeMarkNodeCommandDirtyCalls += refreshStats.treeMarkNodeCommandDirtyCalls;
			perf_.treeAbsModeCalls += refreshStats.treeAbsModeCalls;
			perf_.treeAbsModeFast += refreshStats.treeAbsModeFast;
			perf_.treeAbsModeFull += refreshStats.treeAbsModeFull;
			perf_.treeAbsModeNoop += refreshStats.treeAbsModeNoop;
			perf_.treeAbsModeDirtyNodes += refreshStats.treeAbsModeDirtyNodes;
			if (refreshStats.treeAbsModeRejectReason != 0) {
				perf_.treeAbsModeRejectNode = refreshStats.treeAbsModeRejectNode;
				perf_.treeAbsModeRejectReason = refreshStats.treeAbsModeRejectReason;
			}
			perf_.rootScrollCalls += refreshStats.rootScrollCalls;
		perf_.rootScrollAccepted += refreshStats.rootScrollAccepted;
		perf_.rootScrollRejected += refreshStats.rootScrollRejected;
		perf_.rootScrollMovePx += refreshStats.rootScrollMovePx;
		perf_.rootScrollViewportPx += refreshStats.rootScrollViewportPx;
		perf_.rootScrollStripPx += refreshStats.rootScrollStripPx;
		perf_.rootScrollExtraDirtyNodes += refreshStats.rootScrollExtraDirtyNodes;
		perf_.rootScrollStreamFrames += refreshStats.rootScrollStreamFrames;
		perf_.rootScrollSyncFrames += refreshStats.rootScrollSyncFrames;
		perf_.virtualListRegionCalls += refreshStats.virtualListRegionCalls;
		perf_.virtualListRegionRows += refreshStats.virtualListRegionRows;
		perf_.virtualListRegionTextCalls += refreshStats.virtualListRegionTextCalls;
		perf_.treeRefreshCalls += refreshStats.treeRefreshCalls;
		perf_.treeDirectReplayCalls += refreshStats.treeDirectReplayCalls;
		perf_.treeRecordCalls += refreshStats.treeRecordCalls;
		perf_.treeRecordedNodes += refreshStats.treeRecordedNodes;
		perf_.treeRecordedCommands += refreshStats.treeRecordedCommands;
#endif  // GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG (perf harvest R1)
		setFrameStage(FrameStage::AppUnlock);
		AppState::unlock();

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		setFrameStage(FrameStage::ReadInFrameStats);
		const gea::platform::display::DisplayFlushPerfStats inFrameStats =
			gea::platform::display::Display::flushPerfStatsRead();
		perf_.inFrameFlushUs += inFrameStats.totalUs;
		perf_.inFrameFlushCalls += inFrameStats.callCount;
		perf_.inFrameFlushPixels += inFrameStats.pixelCount;
		perf_.inFrameFlushChunks += inFrameStats.chunkCount;
		perf_.inFrameSetWindowUs += inFrameStats.setWindowUs;
		perf_.inFrameSlotWaitUs += inFrameStats.slotWaitUs;
		perf_.inFrameCopyUs += inFrameStats.copyUs;
		perf_.inFrameRasterUs += inFrameStats.rasterUs;
		perf_.inFrameByteSwapUs += inFrameStats.byteSwapUs;
		perf_.inFrameTxUs += inFrameStats.txUs;
		perf_.inFrameCompleteWaitUs += inFrameStats.completeWaitUs;
		perf_.inFrameEntryWaitUs += inFrameStats.entryWaitUs;
		perf_.inFrameChunkWaitUs += inFrameStats.chunkWaitUs;
		perf_.inFrameTailWaitUs += inFrameStats.tailWaitUs;
#endif  // GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG (perf harvest R2 inFrame)

		gea::platform::display::Display::flushStatsReset();
		setFrameStage(FrameStage::TrailFlush);
		const int64_t trailStartUs = esp_timer_get_time();
		gea::platform::display::Display::flush();
		perf_.trailFlushUs += esp_timer_get_time() - trailStartUs;

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		setFrameStage(FrameStage::ReadTrailStats);
		const gea::platform::display::DisplayFlushPerfStats trailStats =
			gea::platform::display::Display::flushPerfStatsRead();
		perf_.trailFlushCalls += trailStats.callCount;
		perf_.trailFlushPixels += trailStats.pixelCount;
		perf_.trailFlushChunks += trailStats.chunkCount;
		perf_.trailSetWindowUs += trailStats.setWindowUs;
		perf_.trailSlotWaitUs += trailStats.slotWaitUs;
		perf_.trailCopyUs += trailStats.copyUs;
		perf_.trailRasterUs += trailStats.rasterUs;
		perf_.trailByteSwapUs += trailStats.byteSwapUs;
		perf_.trailTxUs += trailStats.txUs;
		perf_.trailCompleteWaitUs += trailStats.completeWaitUs;
		perf_.trailEntryWaitUs += trailStats.entryWaitUs;
		perf_.trailChunkWaitUs += trailStats.chunkWaitUs;
		perf_.trailTailWaitUs += trailStats.tailWaitUs;
#endif  // GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG (perf harvest R3 trail)

		setFrameStage(FrameStage::FrameDone);
		frameInProgress_.store(false, std::memory_order_release);
		setFrameStage(FrameStage::Idle);
		const int64_t frameDoneUs = esp_timer_get_time();
		// Production-fps probe: deliberately outside GEA_EMBEDDED_PERF, so it still
		// reports the TRUE production frame rate when every other perf subsystem is
		// stripped. It has its own switch because it is otherwise a once-a-second
		// INFO line for the life of the device: that is fine while measuring and
		// wrong for an app that keeps a log ring or a serial capture, where it
		// overwrites everything else within minutes. Default stays on so no existing
		// board changes behaviour; an app turns it off with
		// GEA_EMBEDDED_FRAME_SCHEDULER_FPS_LOG=0 in gea.defines.
#if GEA_EMBEDDED_FRAME_SCHEDULER_FPS_LOG
		{
			static int64_t fpsWinStartUs = 0;
			static int fpsWinFrames = 0;
			if (fpsWinStartUs == 0) fpsWinStartUs = frameDoneUs;
			fpsWinFrames++;
			const int64_t fpsElapsed = frameDoneUs - fpsWinStartUs;
			if (fpsElapsed >= 1000000) {
				ESP_LOGI("geafps", "PRODFPS=%.1f (%d frames/%lldms)",
				         fpsWinFrames * 1000000.0 / static_cast<double>(fpsElapsed), fpsWinFrames, fpsElapsed / 1000);
				fpsWinStartUs = frameDoneUs;
				fpsWinFrames = 0;
				// g_diagAbPhase ^= 1; // [DIAG A/B] re-enable to flip fused vs gather each window
			}
		}
#endif  // GEA_EMBEDDED_FRAME_SCHEDULER_FPS_LOG
		perf_.frameTotalUs += frameDoneUs - frameStartUs;
		perf_.frameCount++;
#if GEA_EMBEDDED_TE_MISS_LOG
		// Flush-spike profiling: under TE-sync a frame whose WORK exceeds one TE interval
		// (~16.8ms) misses its VBlank and drops. Dump THIS frame's flush sub-stage split so
		// we can see which stage (drain / raster / copy / tx) spiked — these ~2% of frames
		// are what cap the locked rate. The log fires AFTER frameDoneUs is captured, so it
		// doesn't inflate the measured frame; it only profiles already-overrun frames.
		{
			const int64_t thisFrameUs = frameDoneUs - frameStartUs;
			if (vsyncDriven_.load(std::memory_order_acquire) && thisFrameUs > 16800) {
				// Find the largest dirty region this frame (tree replay samples) so we can see
				// WHAT geometry blew the budget — a text-sized box, a full-width band, or worse.
				int bigW = 0, bigH = 0, bigX = 0, bigY = 0;
				long long bigA = 0;
				for (int i = 0; i < refreshStats.treeReplayRegionSampleCount; i++) {
					const int w = refreshStats.treeReplayRegionX1[i] - refreshStats.treeReplayRegionX0[i] + 1;
					const int h = refreshStats.treeReplayRegionY1[i] - refreshStats.treeReplayRegionY0[i] + 1;
					const long long a = static_cast<long long>(w) * h;
					if (a > bigA) {
						bigA = a; bigW = w; bigH = h;
						bigX = refreshStats.treeReplayRegionX0[i];
						bigY = refreshStats.treeReplayRegionY0[i];
					}
				}
				ESP_LOGW(kTag,
					"TE-MISS frame=%lldus inflush=%lldus(copy=%lld tx=%lld chunks=%d px=%d) regions=%d biggest=%dx%d@(%d,%d)=%lldpx",
					(long long)thisFrameUs,
					(long long)inFrameStats.totalUs,
					(long long)inFrameStats.copyUs,
					(long long)inFrameStats.txUs,
					inFrameStats.chunkCount,
					inFrameStats.pixelCount,
					refreshStats.treeReplayRegionSampleCount,
					bigW, bigH, bigX, bigY, bigA);
			}
		}
#endif
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		const bool hadDisplayUpdate =
			hadScrollUpdate ||
			inFrameStats.pixelCount > 0 ||
			trailStats.pixelCount > 0 ||
			inFrameStats.callCount > 0 ||
			trailStats.callCount > 0;
		recordCadence(frameStartUs, frameDoneUs, hadDisplayUpdate);
#endif  // GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG (perf harvest R5 cadence)

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		// Single-frame dump for any frame that overran a soft budget (50ms). Unlike
		// the windowed log (which averages over kFrameLogInterval frames and so smears
		// a lone spike into 1/30 of its real cost), this prints THIS frame's own phase
		// breakdown — the only honest view of a one-off hitch like a city/theme switch.
		{
			const int64_t thisFrameUs = frameDoneUs - frameStartUs;
			// TEMP (regression diagnosis): raised 50000 -> 300000 so the per-frame dump does
			// not fire on every instrumented frame and spiral the UART (the windowed perf:
			// line below carries the full breakdown instead). A board where the interesting
			// hitch is spread over MANY frames rather than concentrated in one lowers it,
			// so each painting frame prints its own split.
#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_SLOW_FRAME_US
#define GEA_EMBEDDED_FRAME_SCHEDULER_SLOW_FRAME_US 300000
#endif
			if (thisFrameUs > GEA_EMBEDDED_FRAME_SCHEDULER_SLOW_FRAME_US) {
				ESP_LOGW(kTag,
						"SLOW frame=%lldus | drain=%lldus raf=%lldus recompute=%lldus doc=%lldus refresh=%lldus | tree(layout=%lldus(calls:%d repos:%d txt:%lldus) dlist=%lldus dirty=%lldus replay=%lldus/%dr checks=%d draw=rect:%d circle:%d round:%d xround:%d text:%d other:%d rUs=f:%lld/c:%lld/t:%lld/g:%lld/i:%lld/o:%lld bg=%d/%d fast=%d bgUs=%lld/%lld/%lld/%lld flush=%lldus) record(nodes=%d cmds=%d us=%lldus) inflush=%lldus(%dcalls/%dchunks/%dpx setwin=%lldus slot=%lldus copy=%lldus swap=%lldus tx=%lldus wait=%lldus[entry=%lldus chunk=%lldus tail=%lldus]) trail=%lldus(%dcalls/%dpx copy=%lldus wait=%lldus)",
					thisFrameUs,
					(long long)appStats.drainMicrotasksUs,
					(long long)rafStats.totalUs,
					(long long)appStats.styleRecomputeUs,
					(long long)appStats.documentFrameUs,
					(long long)appStats.refreshMountedUs,
					(long long)refreshStats.treeLayoutUs,
					refreshStats.treeLayoutNodeCalls,
					refreshStats.treeLayoutRepositionCalls,
					(long long)refreshStats.treeLayoutTextUs,
					(long long)refreshStats.treeDisplayListUs,
					(long long)refreshStats.treeDirtyCollectUs,
					(long long)refreshStats.treeReplayUs,
					refreshStats.treeReplayRegions,
					refreshStats.treeReplayCommandChecks,
					refreshStats.treeReplayFillRectCommands,
					refreshStats.treeReplayCircleCommands,
					refreshStats.treeReplayRoundedRectCommands,
					refreshStats.treeReplayTransformedRoundedRectCommands,
					refreshStats.treeReplayTextCommands,
					refreshStats.treeReplayOtherCommands,
					(long long)refreshStats.treeReplayFillUs,
					(long long)refreshStats.treeReplayCircleUs,
					(long long)refreshStats.treeReplayTextUs,
					(long long)refreshStats.treeReplayGradientUs,
					(long long)refreshStats.treeReplayImageUs,
					(long long)refreshStats.treeReplayOtherUs,
						refreshStats.treeBgRecolorCalls,
						refreshStats.treeBgRecolorPixels,
						refreshStats.treeBgRecolorFastPixels,
						(long long)refreshStats.treeBgRecolorUs,
						(long long)refreshStats.treeBgRecolorFillUs,
						(long long)refreshStats.treeBgRecolorEdgeUs,
						(long long)refreshStats.treeBgRecolorReplayUs,
						(long long)refreshStats.treeFlushRectsUs,
					refreshStats.treeRecordedNodes,
					refreshStats.treeRecordedCommands,
					(long long)refreshStats.treeRecordNodeUs,
					(long long)inFrameStats.totalUs,
					inFrameStats.callCount,
					inFrameStats.chunkCount,
					inFrameStats.pixelCount,
					(long long)inFrameStats.setWindowUs,
					(long long)inFrameStats.slotWaitUs,
					(long long)inFrameStats.copyUs,
					(long long)inFrameStats.byteSwapUs,
					(long long)inFrameStats.txUs,
					(long long)inFrameStats.completeWaitUs,
					(long long)inFrameStats.entryWaitUs,
					(long long)inFrameStats.chunkWaitUs,
					(long long)inFrameStats.tailWaitUs,
						(long long)trailStats.totalUs,
						trailStats.callCount,
						trailStats.pixelCount,
						(long long)trailStats.copyUs,
						(long long)trailStats.completeWaitUs);
					ESP_LOGW(kTag,
						"SLOW replay-detail direct1=%d directN=%d simple1=%d bgRestore=%lldus nodeWalk=%lldus filter=%lldus/%d clip=%lldus/%d",
						refreshStats.treeReplayDirectRegionCalls,
						refreshStats.treeReplayDirectRegionsCalls,
						refreshStats.treeReplaySimpleRegionCalls,
						(long long)refreshStats.treeReplayBgRestoreUs,
						(long long)refreshStats.treeReplayNodeWalkUs,
						(long long)refreshStats.treeReplayCommandFilterUs,
						refreshStats.treeReplayCommandFilterCalls,
						(long long)refreshStats.treeReplayCommandClipUs,
						refreshStats.treeReplayCommandClipCalls);
					ESP_LOGW(kTag,
						"SLOW inv setStyle=%lldus/%d calls changed=%d noop=%d layout=%d xform=%d paint=%d setText=%lldus/%d changed=%d stable=%d hidden=%d marks=dl:%d content:%d node:%d absMode=%lldus calls:%d fast:%d full:%d noop:%d dirty:%d reject:%d/%d",
					(long long)refreshStats.treeSetStyleUs,
					refreshStats.treeSetStyleCalls,
					refreshStats.treeSetStyleChanged,
					refreshStats.treeSetStyleNoop,
					refreshStats.treeSetStyleLayoutChanged,
					refreshStats.treeSetStyleTransformChanged,
					refreshStats.treeSetStylePaintChanged,
					(long long)refreshStats.treeSetTextUs,
					refreshStats.treeSetTextCalls,
					refreshStats.treeSetTextChanged,
					refreshStats.treeSetTextStable,
					refreshStats.treeSetTextHidden,
					refreshStats.treeMarkDisplayListDirtyCalls,
					refreshStats.treeMarkDisplayListContentDirtyCalls,
					refreshStats.treeMarkNodeCommandDirtyCalls,
					(long long)refreshStats.treeAbsModeUs,
					refreshStats.treeAbsModeCalls,
					refreshStats.treeAbsModeFast,
					refreshStats.treeAbsModeFull,
					refreshStats.treeAbsModeNoop,
					refreshStats.treeAbsModeDirtyNodes,
					refreshStats.treeAbsModeRejectNode,
					refreshStats.treeAbsModeRejectReason);
			}
		}
#endif

		logPerfWindowIfReady();
		const bool timerCatchUpDue = catchUpFrameDue_.exchange(false, std::memory_order_acq_rel);
#if GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES
		catchUpRequest_.store(false, std::memory_order_release);
		vTaskDelay(1);
#else
		// Graceful overrun: if this frame exceeded the frame budget, or the frame
		// timer fired while it was still running, let the event loop run the next
		// frame back-to-back. Boards can cap long catch-up bursts so idle still
		// runs during debug logging or sustained heavy animation.
		//
		// TE-vsync single-clock: when the display drives off TE, the panel's TE edge is
		// the sole frame producer (teEdgeIsr → notifyVsyncFromISR posts each frame; the
		// timer below stops producing). So there is NO graceful-overrun catch-up in that
		// mode — one TE edge = one frame, and an overrun simply drops the next edge
		// (coalesced) rather than spinning a back-to-back loop on a second clock.
		const bool vsyncDriven = vsyncDriven_.load(std::memory_order_acquire);
		const bool catchUpRequested =
			!vsyncDriven && (timerCatchUpDue || (frameDoneUs - frameStartUs) >= frameIntervalUs());
		catchUpRequest_.store(catchUpRequested, std::memory_order_release);
		bool forceIdleYield = false;
	#if GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD > 0
		if (catchUpRequested) {
			catchUpBurstFrames_++;
			if (catchUpBurstFrames_ >= GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD) {
				catchUpBurstFrames_ = 0;
				forceIdleYield = true;
			}
		} else {
			catchUpBurstFrames_ = 0;
		}
	#endif
		if (forceIdleYield) {
			vTaskDelay(1);
		} else if (!catchUpRequested && GEA_EMBEDDED_FRAME_SCHEDULER_IDLE_YIELD_TICKS > 0) {
			vTaskDelay(GEA_EMBEDDED_FRAME_SCHEDULER_IDLE_YIELD_TICKS);
		}
#endif
		}

	void setFrameIntervalMs(int intervalMs)
	{
		setFrameIntervalUs(static_cast<int64_t>(intervalMs) * 1000);
	}

	int frameIntervalMs() const
	{
		return static_cast<int>((frameIntervalUs() + 500) / 1000);
	}

	void setFrameRate(double fps)
	{
		if (!(fps > 0.0)) return;
		setFrameIntervalUs(static_cast<int64_t>((1000000.0 / fps) + 0.5));
	}

	double frameRate() const
	{
		const int64_t intervalUs = frameIntervalUs();
		return intervalUs > 0 ? 1000000.0 / static_cast<double>(intervalUs) : 0.0;
	}

private:
	struct PerfWindow {
		int frameCount = 0;
		int64_t frameTotalUs = 0;
		int64_t framePeriodUs = 0;
		int framePeriodCount = 0;
		int64_t appTotalUs = 0;
		int64_t trailFlushUs = 0;
		int64_t inFrameFlushUs = 0;
		int inFrameFlushCalls = 0;
		int inFrameFlushPixels = 0;
		int trailFlushCalls = 0;
		int trailFlushPixels = 0;
		int inFrameFlushChunks = 0;
		int trailFlushChunks = 0;
		int64_t inFrameSetWindowUs = 0;
		int64_t inFrameSlotWaitUs = 0;
		int64_t inFrameCopyUs = 0;
		int64_t inFrameRasterUs = 0;
		int64_t inFrameByteSwapUs = 0;
		int64_t inFrameTxUs = 0;
		int64_t inFrameCompleteWaitUs = 0;
		int64_t inFrameEntryWaitUs = 0;
		int64_t inFrameChunkWaitUs = 0;
		int64_t inFrameTailWaitUs = 0;
		int64_t trailSetWindowUs = 0;
		int64_t trailSlotWaitUs = 0;
		int64_t trailCopyUs = 0;
		int64_t trailRasterUs = 0;
		int64_t trailByteSwapUs = 0;
		int64_t trailTxUs = 0;
		int64_t trailCompleteWaitUs = 0;
		int64_t trailEntryWaitUs = 0;
		int64_t trailChunkWaitUs = 0;
		int64_t trailTailWaitUs = 0;
		int64_t canvasLookupUs = 0;
		int64_t markDirtyUs = 0;
		int64_t clearRectUs = 0;
		int64_t fillRectUs = 0;
		int64_t fillCircleUs = 0;
		int64_t fillTriangleUs = 0;
		int64_t drawImageUs = 0;
		int64_t fillTextUs = 0;
		int64_t batchFlushUs = 0;
		int64_t drainMicrotasksUs = 0;
		int64_t animationFrameUs = 0;
		int64_t rafTotalUs = 0;
		int64_t rafPrepareUs = 0;
		int64_t rafCallbackUs = 0;
		int64_t rafMaxCallbackUs = 0;
		int64_t styleRecomputeUs = 0;
		int64_t documentFrameUs = 0;
		int64_t refreshMountedUs = 0;
		int64_t rootScrollScanUs = 0;
		int64_t rootScrollRebuildUs = 0;
		int64_t rootScrollScrollRectUs = 0;
		int64_t rootScrollStripReplayUs = 0;
		int64_t rootScrollScrollbarReplayUs = 0;
		int64_t rootScrollFullReplayUs = 0;
		int64_t rootScrollStreamUs = 0;
		int64_t rootScrollExtraReplayUs = 0;
		int64_t rootScrollFlushUs = 0;
		int64_t rootScrollSnapshotUs = 0;
		int64_t virtualListRegionFillUs = 0;
		int64_t virtualListRegionTextUs = 0;
		int64_t virtualListRegionThumbUs = 0;
		int64_t treeLayoutModeUs = 0;
		int64_t treeLayoutUs = 0;
		int64_t treeDisplayListUs = 0;
		int64_t treeRecordNodeUs = 0;
		int64_t treeSetStyleUs = 0;
		int64_t treeSetTextUs = 0;
		int64_t treeAbsModeUs = 0;
		int64_t treeDirtyCollectUs = 0;
		int64_t treeDirtyCoalesceUs = 0;
		int64_t treeReplayUs = 0;
		// Per-command-type replay attribution (fill/text/gradient/image/other) in the
		// windowed log too — the SLOW-frame dump already prints these, but steady-state
		// tuning (e.g. the rotary's staged-replay work) needs the 30-frame average.
		int64_t treeReplayFillUs = 0;
		int64_t treeReplayCircleUs = 0;
		int64_t treeReplayTextUs = 0;
		int64_t treeReplayGradientUs = 0;
		int64_t treeReplayImageUs = 0;
		int64_t treeReplayOtherUs = 0;
			int64_t treeReplaySplitBgUs = 0;
			int64_t treeReplaySplitDynUs = 0;
			int64_t treeReplaySplitRasterUs = 0;
			int64_t treeReplayBgRestoreUs = 0;
			int64_t treeReplayNodeWalkUs = 0;
			int64_t treeReplayCommandFilterUs = 0;
			int64_t treeReplayCommandClipUs = 0;
			int64_t treeFlushRectsUs = 0;
			int64_t treeSnapshotUs = 0;
			int treeReplayCircleCommands = 0;
		int canvasLookupCalls = 0;
		int markDirtyCalls = 0;
		int clearRectCalls = 0;
		int fillRectCalls = 0;
		int fillCircleCalls = 0;
		int fillTriangleCalls = 0;
		int drawImageCalls = 0;
		int fillTextCalls = 0;
		int batchFlushCalls = 0;
		int rafPendingAtRun = 0;
		int rafCallbackCount = 0;
		int rafRequestCount = 0;
		int rafDroppedCount = 0;
		int rootScrollCalls = 0;
		int rootScrollAccepted = 0;
		int rootScrollRejected = 0;
		int rootScrollMovePx = 0;
		int rootScrollViewportPx = 0;
		int rootScrollStripPx = 0;
		int rootScrollExtraDirtyNodes = 0;
		int rootScrollStreamFrames = 0;
		int rootScrollSyncFrames = 0;
		int virtualListRegionCalls = 0;
		int virtualListRegionRows = 0;
			int virtualListRegionTextCalls = 0;
			int treeRefreshCalls = 0;
			int treeDirectReplayCalls = 0;
			int treeBgRecolorCalls = 0;
			int treeBgRecolorPixels = 0;
				int treeReplayRegions = 0;
				int treeReplayOriginRegions = 0;
				int treeReplayCommandChecks = 0;
				int treeReplayDirectRegionCalls = 0;
				int treeReplayDirectRegionsCalls = 0;
				int treeReplaySimpleRegionCalls = 0;
				int treeReplayCommandFilterCalls = 0;
				int treeReplayCommandClipCalls = 0;
				int treeSetStyleCalls = 0;
			int treeSetStyleChanged = 0;
			int treeSetStyleNoop = 0;
			int treeSetStyleLayoutChanged = 0;
			int treeSetStyleTransformChanged = 0;
			int treeSetStylePaintChanged = 0;
			int treeSetTextCalls = 0;
			int treeSetTextChanged = 0;
			int treeSetTextStable = 0;
			int treeSetTextHidden = 0;
			int treeMarkDisplayListDirtyCalls = 0;
			int treeMarkDisplayListContentDirtyCalls = 0;
			int treeMarkNodeCommandDirtyCalls = 0;
			int treeAbsModeCalls = 0;
			int treeAbsModeFast = 0;
			int treeAbsModeFull = 0;
			int treeAbsModeNoop = 0;
			int treeAbsModeDirtyNodes = 0;
			int treeAbsModeRejectNode = -1;
			int treeAbsModeRejectReason = 0;
			int treeRecordCalls = 0;
		int treeRecordedNodes = 0;
		int treeRecordedCommands = 0;
		int cadenceActiveFrames = 0;
		int cadenceStartGapCount = 0;
		int cadenceStartGapOver20ms = 0;
		int cadenceStartGapOver33ms = 0;
		int cadenceDoneGapCount = 0;
		int cadenceDoneGapOver20ms = 0;
		int cadenceDoneGapOver33ms = 0;
		int64_t cadenceStartGapUs = 0;
		int64_t cadenceStartGapMinUs = std::numeric_limits<int64_t>::max();
		int64_t cadenceStartGapMaxUs = 0;
		int64_t cadenceDoneGapUs = 0;
		int64_t cadenceDoneGapMinUs = std::numeric_limits<int64_t>::max();
		int64_t cadenceDoneGapMaxUs = 0;
		int64_t cadenceFrameUs = 0;
		int64_t cadenceFrameMaxUs = 0;

		void reset()
		{
			*this = {};
		}
	};

	FrameSchedulerEngine() = default;

	static const char *stageName(FrameStage stage)
	{
		switch (stage) {
			case FrameStage::FrameBegin: return "frame_begin";
			case FrameStage::WaitAppLock: return "wait_app_lock";
			case FrameStage::AppFrame: return "app_frame";
			case FrameStage::AppUnlock: return "app_unlock";
			case FrameStage::ReadInFrameStats: return "read_inframe_stats";
			case FrameStage::TrailFlush: return "trail_flush";
			case FrameStage::ReadTrailStats: return "read_trail_stats";
			case FrameStage::FrameDone: return "frame_done";
			case FrameStage::Idle: return "idle";
		}
		return "idle";
	}

	void setFrameStage(FrameStage stage)
	{
		frameStage_.store(static_cast<int>(stage), std::memory_order_release);
	}

	FrameStage frameStage() const
	{
		return static_cast<FrameStage>(frameStage_.load(std::memory_order_acquire));
	}

	void setFrameIntervalUs(int64_t intervalUs)
	{
		const int64_t normalized = normalizeFrameIntervalUs(intervalUs);
		frameIntervalUs_.store(normalized, std::memory_order_relaxed);
#if GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
		restartTimer(normalized);
#endif
	}

	int64_t frameIntervalUs() const
	{
		return frameIntervalUs_.load(std::memory_order_relaxed);
	}

	static int64_t normalizeFrameIntervalUs(int64_t intervalUs)
	{
		const int64_t minUs = static_cast<int64_t>(FrameScheduler::kMinFrameIntervalMs) * 1000;
		const int64_t maxUs = static_cast<int64_t>(FrameScheduler::kMaxFrameIntervalMs) * 1000;
		if (intervalUs < minUs) return minUs;
		if (intervalUs > maxUs) return maxUs;
		return intervalUs;
	}

	static int64_t tickPeriodUs()
	{
		constexpr int64_t periodUs = 1000000LL / configTICK_RATE_HZ;
		return periodUs > 0 ? periodUs : 1000;
	}

	static TickType_t ticksForFrameIntervalUs(int64_t intervalUs)
	{
		const int64_t tickUs = tickPeriodUs();
		TickType_t ticks = static_cast<TickType_t>(normalizeFrameIntervalUs(intervalUs) / tickUs);
		return ticks < 1 ? 1 : ticks;
	}

	void checkWatchdog()
	{
		if (!frameInProgress_.load(std::memory_order_acquire)) return;

		const int nowMs = FrameScheduler::nowMs();
		const int elapsedMs = nowMs - frameStartedMs_.load(std::memory_order_relaxed);
		if (elapsedMs < FrameScheduler::kWatchdogMs) return;
		if (nowMs - watchdogLastMs_ < FrameScheduler::kWatchdogMs) return;
		watchdogLastMs_ = nowMs;
		const auto displayDetail = gea::platform::display::Display::flushStageDetail();

		ESP_LOGE(kTag,
			"frame watchdog: seq=%lu stuck=%dms stage=%s app_phase=%s ui=%s/%d display=%s/%d rect=%d,%d-%d,%d rows=%d px=%d pending=%d internal_free=%u internal_largest=%u internal_min=%u psram_free=%u",
			static_cast<unsigned long>(frameSequence_.load(std::memory_order_relaxed)),
			elapsedMs,
			stageName(frameStage()),
			gea::framework::app::applicationFramePhaseName(gea::framework::app::applicationFramePhaseRead()),
			gea::embedded::ui::refreshTraceStageName(),
			gea::embedded::ui::refreshTraceIndex(),
			gea::platform::display::Display::flushStageName(),
			gea::platform::display::Display::flushStageChunk(),
			displayDetail.x0,
			displayDetail.y0,
			displayDetail.x1,
			displayDetail.y1,
			displayDetail.rows,
			displayDetail.pixels,
			eventPending_.load(std::memory_order_relaxed) ? 1 : 0,
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
			static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
			static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
			static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
	}

	void queueFrameEvent()
	{
		timerTickCount_.fetch_add(1, std::memory_order_relaxed);
		if (!eventQueue_)
			return;
		// TE-vsync single-clock: while the panel TE is producing frames, the timer must
		// NOT also post — that's the second clock we're eliminating. Only fall back to
		// timer-posting if the TE edge has gone silent for >2 periods (dead/garbled TE),
		// so a missing TE can never freeze rendering.
		if (vsyncDriven_.load(std::memory_order_acquire)) {
			const uint32_t sincePost =
				static_cast<uint32_t>(esp_timer_get_time()) - lastVsyncPostUs_.load(std::memory_order_acquire);
			if (sincePost < 2u * static_cast<uint32_t>(frameIntervalUs()))
				return;
		}
		if (eventPending_.load(std::memory_order_acquire)) {
			catchUpFrameDue_.store(true, std::memory_order_release);
			timerPendingSkipCount_.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		if (frameInProgress_.load(std::memory_order_acquire)) {
			catchUpFrameDue_.store(true, std::memory_order_release);
			timerInProgressCount_.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		gea::framework::events::Event event{};
		event.type = gea::framework::events::EventType::Frame;
		eventPending_.store(true, std::memory_order_release);
		if (!sendEvent(event, 0)) {
			eventPending_.store(false, std::memory_order_release);
			timerSendFailCount_.fetch_add(1, std::memory_order_relaxed);
		} else {
			timerQueueCount_.fetch_add(1, std::memory_order_relaxed);
		}
	}

#if GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
	void startTimer()
	{
		if (timer_) return;
		const esp_timer_create_args_t args = {
			.callback = &FrameSchedulerEngine::timerCallback,
			.arg = this,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "gea_frame",
			.skip_unhandled_events = true,
		};
		const esp_err_t createErr = esp_timer_create(&args, &timer_);
		if (createErr != ESP_OK) {
			ESP_LOGW(kTag, "Failed to create frame scheduler timer: %d", static_cast<int>(createErr));
			return;
		}
		restartTimer(frameIntervalUs());
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		ESP_LOGI(kTag, "frame scheduler esp_timer started interval=%lldus", static_cast<long long>(timerIntervalUs_));
#endif
	}

	void restartTimer(int64_t intervalUs)
	{
		if (!timer_) return;
		if (timerRunning_) {
			const esp_err_t stopErr = esp_timer_stop(timer_);
			if (stopErr != ESP_OK && stopErr != ESP_ERR_INVALID_STATE)
				ESP_LOGW(kTag, "Failed to stop frame scheduler timer: %d", static_cast<int>(stopErr));
			timerRunning_ = false;
		}
		const esp_err_t startErr = esp_timer_start_periodic(timer_, static_cast<uint64_t>(intervalUs));
		if (startErr != ESP_OK) {
			ESP_LOGW(kTag, "Failed to start frame scheduler timer: %d", static_cast<int>(startErr));
			return;
		}
		timerIntervalUs_ = intervalUs;
		timerRunning_ = true;
	}

	void onTimer()
	{
		checkWatchdog();
		queueFrameEvent();
	}

	static void timerCallback(void *arg)
	{
		static_cast<FrameSchedulerEngine *>(arg)->onTimer();
	}
#else
	// In TE-vsync mode the panel's TE ISR produces every frame, so this timer task is only
	// a dead-TE watchdog + setFrameRate fallback — it must NOT wake every frame. Waking at
	// the ~16ms frame interval makes this prio-23 task preempt the prio-5 render task once
	// per frame, beating against the ~16.7ms TE and knocking ~2% of frames past their edge
	// (a missed VBlank → the steady rate stalls at ~58 instead of locking to ~60). At a
	// coarse watchdog cadence the render task owns core 0 between TEs and locks cleanly.
	// 48ms still catches a dead/garbled TE within ~3 frames (queueFrameEvent's fallback
	// fires once TE has been silent > 2 frame intervals).
	static constexpr int64_t kVsyncWatchdogIntervalUs = 48000;

	void runTask()
	{
		TickType_t lastWake = xTaskGetTickCount();
		int64_t lastIntervalUs = -1;
		TickType_t ticks = 1;

		while (true) {
			// Pick the wake cadence by mode: frame-rate timer when we drive frames
			// ourselves, coarse watchdog when the TE drives them.
			const int64_t intervalUs =
				vsyncDriven() ? kVsyncWatchdogIntervalUs : frameIntervalUs();
			if (intervalUs != lastIntervalUs) {
				lastIntervalUs = intervalUs;
				ticks = ticksForFrameIntervalUs(intervalUs);
				lastWake = xTaskGetTickCount();
			}
			vTaskDelayUntil(&lastWake, ticks);
			checkWatchdog();
			queueFrameEvent();
		}
	}

	static void taskMain(void *arg)
	{
		static_cast<FrameSchedulerEngine *>(arg)->runTask();
	}
#endif

	static void recordGap(int64_t gapUs,
		int *count,
		int64_t *totalUs,
		int64_t *minUs,
		int64_t *maxUs,
		int *over20ms,
		int *over33ms)
	{
		if (gapUs <= 0 || gapUs > 100000)
			return;
		(*count)++;
		*totalUs += gapUs;
		if (gapUs < *minUs) *minUs = gapUs;
		if (gapUs > *maxUs) *maxUs = gapUs;
		if (gapUs > 20000) (*over20ms)++;
		if (gapUs > 33333) (*over33ms)++;
	}

	void recordCadence(int64_t frameStartUs, int64_t frameDoneUs, bool active)
	{
		if (!active)
			return;
		perf_.cadenceActiveFrames++;
		const int64_t frameUs = frameDoneUs - frameStartUs;
		perf_.cadenceFrameUs += frameUs;
		if (frameUs > perf_.cadenceFrameMaxUs)
			perf_.cadenceFrameMaxUs = frameUs;
		if (lastActiveFrameStartUs_ > 0) {
			recordGap(frameStartUs - lastActiveFrameStartUs_,
				&perf_.cadenceStartGapCount,
				&perf_.cadenceStartGapUs,
				&perf_.cadenceStartGapMinUs,
				&perf_.cadenceStartGapMaxUs,
				&perf_.cadenceStartGapOver20ms,
				&perf_.cadenceStartGapOver33ms);
		}
		lastActiveFrameStartUs_ = frameStartUs;
		if (lastActiveFrameDoneUs_ > 0) {
			recordGap(frameDoneUs - lastActiveFrameDoneUs_,
				&perf_.cadenceDoneGapCount,
				&perf_.cadenceDoneGapUs,
				&perf_.cadenceDoneGapMinUs,
				&perf_.cadenceDoneGapMaxUs,
				&perf_.cadenceDoneGapOver20ms,
				&perf_.cadenceDoneGapOver33ms);
		}
		lastActiveFrameDoneUs_ = frameDoneUs;
	}

	void logPerfWindowIfReady()
	{
		// Window length. The ~80-arg ESP_LOGI dump below costs a few ms of vsnprintf +
		// console TX ON THE FRAME TASK when it fires; at a 30-frame cadence that lands as
		// ~1 dropped VBlank per 30 under TE-sync (the frame overruns its 16ms window). A
		// 300-frame window keeps the diagnostic but makes that perturbation rare (~once
		// per 5s) so it stops capping the steady TE-locked rate.
		// Board-tunable: a board capturing the PER-FRAME dump instead needs this
		// windowed line out of the way, or its ~80-arg output floods a small log
		// ring and evicts the frames actually being looked at.
#ifndef GEA_EMBEDDED_FRAME_SCHEDULER_PERF_WINDOW_FRAMES
#define GEA_EMBEDDED_FRAME_SCHEDULER_PERF_WINDOW_FRAMES 300
#endif
		if (perf_.frameCount < GEA_EMBEDDED_FRAME_SCHEDULER_PERF_WINDOW_FRAMES) return;
#if !GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		// Perf logging disabled at build time. Still reset counters so they
		// don't accumulate indefinitely. The ~80-arg ESP_LOGI vsnprintf below
		// costs ~1-2ms when it fires, which lands as a single per-window
		// outlier in steady-state animations — disabling lets a board measure
		// without perturbing what it's measuring.
		perf_.reset();
		return;
#else
		const int64_t appNoFlushUs = perf_.appTotalUs - perf_.inFrameFlushUs;
		const int64_t accountedCanvasUs =
			perf_.canvasLookupUs +
			perf_.markDirtyUs +
			perf_.clearRectUs +
			perf_.fillRectUs +
			perf_.fillCircleUs +
			perf_.fillTriangleUs +
			perf_.drawImageUs +
			perf_.fillTextUs;
		const int64_t otherAppUs = appNoFlushUs - accountedCanvasUs;
		const int64_t animationFrameNoFlushUs = perf_.animationFrameUs - perf_.inFrameFlushUs;
		const unsigned internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		const unsigned internalLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		const unsigned internalMin = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		const unsigned psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		const unsigned geaMainStackHwm = static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr));
		const unsigned appFrameStackHwm = task_ ? static_cast<unsigned>(uxTaskGetStackHighWaterMark(task_)) : 0;
		const int64_t avgPeriodUs = perf_.framePeriodCount > 0 ? perf_.framePeriodUs / perf_.framePeriodCount : 0;
		const int fpsX10 = avgPeriodUs > 0 ? static_cast<int>((10000000LL + avgPeriodUs / 2) / avgPeriodUs) : 0;
		const int64_t cadenceFrameAvgUs =
			perf_.cadenceActiveFrames > 0 ? perf_.cadenceFrameUs / perf_.cadenceActiveFrames : 0;
		const int64_t cadenceStartAvgUs =
			perf_.cadenceStartGapCount > 0 ? perf_.cadenceStartGapUs / perf_.cadenceStartGapCount : 0;
		const int64_t cadenceStartMinUs =
			perf_.cadenceStartGapCount > 0 ? perf_.cadenceStartGapMinUs : 0;
		const int64_t cadenceDoneAvgUs =
			perf_.cadenceDoneGapCount > 0 ? perf_.cadenceDoneGapUs / perf_.cadenceDoneGapCount : 0;
		const int64_t cadenceDoneMinUs =
			perf_.cadenceDoneGapCount > 0 ? perf_.cadenceDoneGapMinUs : 0;
		const int timerTicks = timerTickCount_.exchange(0, std::memory_order_relaxed);
		const int timerQueued = timerQueueCount_.exchange(0, std::memory_order_relaxed);
		const int timerPendingSkips = timerPendingSkipCount_.exchange(0, std::memory_order_relaxed);
		const int timerInProgress = timerInProgressCount_.exchange(0, std::memory_order_relaxed);
		const int timerSendFails = timerSendFailCount_.exchange(0, std::memory_order_relaxed);
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE
		ESP_LOGI(kTag,
			"perf-lite: frame=%lldus period=%lldus fps=%d.%01d target=%lldus timer=tick:%d queue:%d pend:%d busy:%d fail:%d app=%lldus raf=%lldus refresh=%lldus replay=%dr flush=%lldus/%dc/%dpx cadence=%lldus/%lldus/%lldus [avg over %d frames]",
			perf_.frameTotalUs / perf_.frameCount,
			avgPeriodUs,
			fpsX10 / 10,
			fpsX10 % 10,
			frameIntervalUs(),
			timerTicks,
			timerQueued,
			timerPendingSkips,
			timerInProgress,
			timerSendFails,
			perf_.appTotalUs / perf_.frameCount,
			perf_.animationFrameUs / perf_.frameCount,
			perf_.refreshMountedUs / perf_.frameCount,
			perf_.treeReplayRegions / perf_.frameCount,
			perf_.inFrameFlushUs / perf_.frameCount,
			perf_.inFrameFlushChunks / perf_.frameCount,
			perf_.inFrameFlushPixels / perf_.frameCount,
			cadenceStartMinUs,
			cadenceStartAvgUs,
			perf_.cadenceStartGapMaxUs,
			perf_.frameCount);
		perf_.reset();
		return;
#endif
		ESP_LOGI(kTag,
				"perf: frame=%lldus period=%lldus fps=%d.%01d target=%lldus timer=tick:%d queue:%d pend:%d busy:%d fail:%d cadence=active:%d frame:%lldus max:%lldus start:%lldus/%lldus/%lldus over20:%d over33:%d done:%lldus/%lldus/%lldus over20:%d over33:%d app=%lldus appnoflush=%lldus phases=drain:%lldus raf:%lldus rafnoflush:%lldus recompute:%lldus doc:%lldus refresh:%lldus rafdetail=run:%lldus prep:%lldus cb:%lldus/%d max:%lldus pending:%d req:%d drop:%d canvas=lookup:%lldus/%d dirty:%lldus/%d clear:%lldus/%d fill:%lldus/%d image:%lldus/%d text:%lldus/%d other:%lldus batchflush=%lldus/%d inflush=%lldus(%dcalls/%dchunks/%dpx setwin:%lldus slot:%lldus copy:%lldus raster:%lldus swap:%lldus tx:%lldus wait:%lldus entry:%lldus chunk:%lldus tail:%lldus) trailflush=%lldus(%dcalls/%dchunks/%dpx setwin:%lldus slot:%lldus copy:%lldus raster:%lldus swap:%lldus tx:%lldus wait:%lldus entry:%lldus chunk:%lldus tail:%lldus) ui=root(scan:%lldus/%d ok:%d rej:%d rebuild:%lldus scroll:%lldus strip:%lldus bar:%lldus full:%lldus stream:%lldus/%d sync:%d extra:%lldus flush:%lldus snap:%lldus move:%dpx viewport:%dpx strip_px:%d extra_nodes:%d) vlist=region(fill:%lldus text:%lldus/%d thumb:%lldus calls:%d rows:%d) tree(count:%d direct:%d bgrecolor:%d/%dpx mode:%lldus layout:%lldus dlist:%lldus dirty:%lldus coal:%lldus replay:%lldus/%dr/%dor/%dcc circle:%d rUs=f:%lld/c:%lld/t:%lld/g:%lld/i:%lld/o:%lld flush:%lldus snap:%lldus) heap=internal:%u largest:%u min:%u psram:%u stack=gea_hwm:%u app_frame_hwm:%u recprobe(fullrebuilds:%d recUs:%lld nodes:%d cmds:%d) [avg over %d frames]",
			perf_.frameTotalUs / perf_.frameCount,
			avgPeriodUs,
			fpsX10 / 10,
			fpsX10 % 10,
			frameIntervalUs(),
			timerTicks,
			timerQueued,
			timerPendingSkips,
			timerInProgress,
			timerSendFails,
			perf_.cadenceActiveFrames,
			cadenceFrameAvgUs,
			perf_.cadenceFrameMaxUs,
			cadenceStartMinUs,
			cadenceStartAvgUs,
			perf_.cadenceStartGapMaxUs,
			perf_.cadenceStartGapOver20ms,
			perf_.cadenceStartGapOver33ms,
			cadenceDoneMinUs,
			cadenceDoneAvgUs,
			perf_.cadenceDoneGapMaxUs,
			perf_.cadenceDoneGapOver20ms,
			perf_.cadenceDoneGapOver33ms,
			perf_.appTotalUs / perf_.frameCount,
			appNoFlushUs / perf_.frameCount,
			perf_.drainMicrotasksUs / perf_.frameCount,
			perf_.animationFrameUs / perf_.frameCount,
			animationFrameNoFlushUs / perf_.frameCount,
			perf_.styleRecomputeUs / perf_.frameCount,
			perf_.documentFrameUs / perf_.frameCount,
			perf_.refreshMountedUs / perf_.frameCount,
			perf_.rafTotalUs / perf_.frameCount,
			perf_.rafPrepareUs / perf_.frameCount,
			perf_.rafCallbackUs / perf_.frameCount,
			perf_.rafCallbackCount / perf_.frameCount,
			perf_.rafMaxCallbackUs,
			perf_.rafPendingAtRun / perf_.frameCount,
			perf_.rafRequestCount / perf_.frameCount,
			perf_.rafDroppedCount / perf_.frameCount,
			perf_.canvasLookupUs / perf_.frameCount,
			perf_.canvasLookupCalls / perf_.frameCount,
			perf_.markDirtyUs / perf_.frameCount,
			perf_.markDirtyCalls / perf_.frameCount,
			(perf_.clearRectUs + perf_.fillRectUs) / perf_.frameCount,
			(perf_.clearRectCalls + perf_.fillRectCalls) / perf_.frameCount,
			(perf_.fillCircleUs + perf_.fillTriangleUs) / perf_.frameCount,
			(perf_.fillCircleCalls + perf_.fillTriangleCalls) / perf_.frameCount,
			perf_.drawImageUs / perf_.frameCount,
			perf_.drawImageCalls / perf_.frameCount,
			perf_.fillTextUs / perf_.frameCount,
			perf_.fillTextCalls / perf_.frameCount,
			otherAppUs / perf_.frameCount,
			perf_.batchFlushUs / perf_.frameCount,
			perf_.batchFlushCalls / perf_.frameCount,
			perf_.inFrameFlushUs / perf_.frameCount,
			perf_.inFrameFlushCalls / perf_.frameCount,
			perf_.inFrameFlushChunks / perf_.frameCount,
			perf_.inFrameFlushPixels / perf_.frameCount,
			perf_.inFrameSetWindowUs / perf_.frameCount,
			perf_.inFrameSlotWaitUs / perf_.frameCount,
			perf_.inFrameCopyUs / perf_.frameCount,
			perf_.inFrameRasterUs / perf_.frameCount,
			perf_.inFrameByteSwapUs / perf_.frameCount,
			perf_.inFrameTxUs / perf_.frameCount,
			perf_.inFrameCompleteWaitUs / perf_.frameCount,
			perf_.inFrameEntryWaitUs / perf_.frameCount,
			perf_.inFrameChunkWaitUs / perf_.frameCount,
			perf_.inFrameTailWaitUs / perf_.frameCount,
			perf_.trailFlushUs / perf_.frameCount,
			perf_.trailFlushCalls / perf_.frameCount,
			perf_.trailFlushChunks / perf_.frameCount,
			perf_.trailFlushPixels / perf_.frameCount,
			perf_.trailSetWindowUs / perf_.frameCount,
			perf_.trailSlotWaitUs / perf_.frameCount,
			perf_.trailCopyUs / perf_.frameCount,
			perf_.trailRasterUs / perf_.frameCount,
			perf_.trailByteSwapUs / perf_.frameCount,
			perf_.trailTxUs / perf_.frameCount,
			perf_.trailCompleteWaitUs / perf_.frameCount,
			perf_.trailEntryWaitUs / perf_.frameCount,
			perf_.trailChunkWaitUs / perf_.frameCount,
			perf_.trailTailWaitUs / perf_.frameCount,
			perf_.rootScrollScanUs / perf_.frameCount,
			perf_.rootScrollCalls / perf_.frameCount,
			perf_.rootScrollAccepted / perf_.frameCount,
			perf_.rootScrollRejected / perf_.frameCount,
			perf_.rootScrollRebuildUs / perf_.frameCount,
			perf_.rootScrollScrollRectUs / perf_.frameCount,
			perf_.rootScrollStripReplayUs / perf_.frameCount,
			perf_.rootScrollScrollbarReplayUs / perf_.frameCount,
			perf_.rootScrollFullReplayUs / perf_.frameCount,
			perf_.rootScrollStreamUs / perf_.frameCount,
			perf_.rootScrollStreamFrames,
			perf_.rootScrollSyncFrames,
			perf_.rootScrollExtraReplayUs / perf_.frameCount,
			perf_.rootScrollFlushUs / perf_.frameCount,
			perf_.rootScrollSnapshotUs / perf_.frameCount,
			perf_.rootScrollMovePx / perf_.frameCount,
			perf_.rootScrollViewportPx / perf_.frameCount,
			perf_.rootScrollStripPx / perf_.frameCount,
			perf_.rootScrollExtraDirtyNodes / perf_.frameCount,
			perf_.virtualListRegionFillUs / perf_.frameCount,
			perf_.virtualListRegionTextUs / perf_.frameCount,
			perf_.virtualListRegionTextCalls / perf_.frameCount,
			perf_.virtualListRegionThumbUs / perf_.frameCount,
			perf_.virtualListRegionCalls / perf_.frameCount,
				perf_.virtualListRegionRows / perf_.frameCount,
				perf_.treeRefreshCalls / perf_.frameCount,
				perf_.treeDirectReplayCalls / perf_.frameCount,
				perf_.treeBgRecolorCalls,
				perf_.treeBgRecolorPixels / perf_.frameCount,
				perf_.treeLayoutModeUs / perf_.frameCount,
			perf_.treeLayoutUs / perf_.frameCount,
			perf_.treeDisplayListUs / perf_.frameCount,
			perf_.treeDirtyCollectUs / perf_.frameCount,
			perf_.treeDirtyCoalesceUs / perf_.frameCount,
			perf_.treeReplayUs / perf_.frameCount,
			perf_.treeReplayRegions / perf_.frameCount,
			perf_.treeReplayOriginRegions / perf_.frameCount,
			perf_.treeReplayCommandChecks / perf_.frameCount,
			perf_.treeReplayCircleCommands / perf_.frameCount,
			perf_.treeReplayFillUs / perf_.frameCount,
			perf_.treeReplayCircleUs / perf_.frameCount,
			perf_.treeReplayTextUs / perf_.frameCount,
			perf_.treeReplayGradientUs / perf_.frameCount,
			perf_.treeReplayImageUs / perf_.frameCount,
			perf_.treeReplayOtherUs / perf_.frameCount,
			perf_.treeFlushRectsUs / perf_.frameCount,
			perf_.treeSnapshotUs / perf_.frameCount,
			internalFree,
			internalLargest,
			internalMin,
			psramFree,
			geaMainStackHwm,
			appFrameStackHwm,
			perf_.treeRecordCalls,
			(long long)perf_.treeRecordNodeUs,
			perf_.treeRecordedNodes,
			perf_.treeRecordedCommands,
			perf_.frameCount);
			ESP_LOGI(kTag,
				"replay-split: bg=%lldus dyn=%lldus raster=%lldus [per-frame avg over %d]",
				(long long)(perf_.treeReplaySplitBgUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				(long long)(perf_.treeReplaySplitDynUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				(long long)(perf_.treeReplaySplitRasterUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				perf_.frameCount);
			ESP_LOGI(kTag,
				"replay-detail: direct1=%d directN=%d simple1=%d bgRestore=%lldus nodeWalk=%lldus filter=%lldus/%d clip=%lldus/%d [per-frame avg over %d]",
				perf_.treeReplayDirectRegionCalls / perf_.frameCount,
				perf_.treeReplayDirectRegionsCalls / perf_.frameCount,
				perf_.treeReplaySimpleRegionCalls / perf_.frameCount,
				(long long)(perf_.treeReplayBgRestoreUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				(long long)(perf_.treeReplayNodeWalkUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				(long long)(perf_.treeReplayCommandFilterUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				perf_.treeReplayCommandFilterCalls / perf_.frameCount,
				(long long)(perf_.treeReplayCommandClipUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
				perf_.treeReplayCommandClipCalls / perf_.frameCount,
				perf_.frameCount);
			ESP_LOGI(kTag,
				"layout-inv: setStyle=%lldus/%dc changed=%d noop=%d layout=%d xform=%d paint=%d setText=%lldus/%dc changed=%d stable=%d hidden=%d marks=dl:%d content:%d node:%d absMode=%lldus/%dc fast=%d full=%d noop=%d dirty=%d reject:%d/%d [avg/sum over %d]",
			(long long)(perf_.treeSetStyleUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
			perf_.treeSetStyleCalls / perf_.frameCount,
			perf_.treeSetStyleChanged / perf_.frameCount,
			perf_.treeSetStyleNoop / perf_.frameCount,
			perf_.treeSetStyleLayoutChanged / perf_.frameCount,
			perf_.treeSetStyleTransformChanged / perf_.frameCount,
			perf_.treeSetStylePaintChanged / perf_.frameCount,
			(long long)(perf_.treeSetTextUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
			perf_.treeSetTextCalls / perf_.frameCount,
			perf_.treeSetTextChanged / perf_.frameCount,
			perf_.treeSetTextStable / perf_.frameCount,
			perf_.treeSetTextHidden / perf_.frameCount,
			perf_.treeMarkDisplayListDirtyCalls / perf_.frameCount,
			perf_.treeMarkDisplayListContentDirtyCalls / perf_.frameCount,
			perf_.treeMarkNodeCommandDirtyCalls / perf_.frameCount,
			(long long)(perf_.treeAbsModeUs / (perf_.frameCount > 0 ? perf_.frameCount : 1)),
			perf_.treeAbsModeCalls / perf_.frameCount,
			perf_.treeAbsModeFast / perf_.frameCount,
			perf_.treeAbsModeFull / perf_.frameCount,
			perf_.treeAbsModeNoop / perf_.frameCount,
			perf_.treeAbsModeDirtyNodes / perf_.frameCount,
			perf_.treeAbsModeRejectNode,
			perf_.treeAbsModeRejectReason,
			perf_.frameCount);
		perf_.reset();
#endif
	}

	TaskHandle_t task_ = nullptr;
	std::atomic<bool> eventPending_{false};
	std::atomic<bool> frameInProgress_{false};
	std::atomic<bool> catchUpFrameDue_{false};
	// Set true when a frame overran the budget; the event loop runs the next frame
	// back-to-back (graceful overrun) instead of waiting for the timer's tick.
	std::atomic<bool> catchUpRequest_{false};
	// Set by the display backend when TE-vsync is enabled. In this mode the panel's TE
	// edge is the SOLE frame producer (teEdgeIsr → notifyVsyncFromISR), and the timer
	// below stops posting frames. See FrameScheduler::setVsyncDriven / notifyVsyncFromISR.
	std::atomic<bool> vsyncDriven_{false};
	// µs (32-bit, wrap-safe) of the last TE-driven frame post, written by the TE ISR.
	// The timer reads it to fall back to timer-posting only if the TE has gone silent.
	std::atomic<uint32_t> lastVsyncPostUs_{0};
	std::atomic<int> frameStartedMs_{0};
	int64_t lastFrameStartUs_ = 0;
	int64_t lastActiveFrameStartUs_ = 0;
	int64_t lastActiveFrameDoneUs_ = 0;
	int catchUpBurstFrames_ = 0;
	int watchdogLastMs_ = 0;
	std::atomic<std::uint32_t> frameSequence_{0};
	std::atomic<int> frameStage_{static_cast<int>(FrameStage::Idle)};
	std::atomic<int> timerTickCount_{0};
	std::atomic<int> timerQueueCount_{0};
	std::atomic<int> timerPendingSkipCount_{0};
	std::atomic<int> timerInProgressCount_{0};
	std::atomic<int> timerSendFailCount_{0};
	StaticQueue_t eventQueueStorage_{};
	std::uint8_t eventQueueBuffer_[FrameScheduler::kEventQueueDepth * sizeof(gea::framework::events::Event)]{};
#if GEA_EMBEDDED_FRAME_SCHEDULER_USE_ESP_TIMER
	esp_timer_handle_t timer_ = nullptr;
	int64_t timerIntervalUs_ = 0;
	bool timerRunning_ = false;
#else
	StaticTask_t taskStorage_{};
#if GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL
	StackType_t *taskStack_ = nullptr; // external RAM, allocated in start()
#else
	StackType_t taskStack_[kTaskStackWords]{};
#endif
#endif
	QueueHandle_t eventQueue_ = nullptr;
	std::atomic<int64_t> frameIntervalUs_{FrameScheduler::kDefaultFrameIntervalUs};
	PerfWindow perf_{};
};

}  // namespace

// [DIAG A/B] read by the renderer's fused-gradient-blend gate (weak symbol in render.cpp).
extern "C" int gea_diag_ab_phase() { return g_diagAbPhase; }

EventQueue FrameScheduler::createEventQueue()
{
	return FrameSchedulerEngine::instance().createEventQueue();
}

EventQueue FrameScheduler::eventQueue()
{
	return FrameSchedulerEngine::instance().eventQueue();
}

bool FrameScheduler::sendEvent(const gea::framework::events::Event &event, int waitMs)
{
	return FrameSchedulerEngine::instance().sendEvent(event, waitMs);
}

bool FrameScheduler::receiveEvent(gea::framework::events::Event *event)
{
	return FrameSchedulerEngine::instance().receiveEvent(event);
}

int FrameScheduler::nowMs()
{
	return static_cast<int>(esp_timer_get_time() / 1000);
}

void FrameScheduler::start(EventQueue queue)
{
	FrameSchedulerEngine::instance().start(queue);
}

void FrameScheduler::runFrame(const FrameCallbacks &callbacks)
{
	FrameSchedulerEngine::instance().runFrame(callbacks);
}

bool FrameScheduler::takeCatchUpRequest()
{
	return FrameSchedulerEngine::instance().takeCatchUpRequest();
}

bool FrameScheduler::drainFramesAndCheckInput()
{
	return FrameSchedulerEngine::instance().drainFramesAndCheckInput();
}

void FrameScheduler::setVsyncDriven(bool driven)
{
	FrameSchedulerEngine::instance().setVsyncDriven(driven);
}

bool FrameScheduler::vsyncDriven()
{
	return FrameSchedulerEngine::instance().vsyncDriven();
}

void FrameScheduler::notifyVsyncFromISR()
{
	FrameSchedulerEngine::instance().notifyVsyncFromISR();
}

void FrameScheduler::setFrameIntervalMs(int intervalMs)
{
	FrameSchedulerEngine::instance().setFrameIntervalMs(intervalMs);
}

int FrameScheduler::frameIntervalMs()
{
	return FrameSchedulerEngine::instance().frameIntervalMs();
}

void FrameScheduler::setFrameRate(double fps)
{
	FrameSchedulerEngine::instance().setFrameRate(fps);
}

double FrameScheduler::frameRate()
{
	return FrameSchedulerEngine::instance().frameRate();
}

}  // namespace gea::framework::services
