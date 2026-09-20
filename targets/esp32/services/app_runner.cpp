#include "services/app_runner.h"

#include <cstddef>
#include <cstdio>

#include "app.h"
#include "events.h"
#include "memory_config.h"
#include "services/app_state.h"
#include "services/diagnostics.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
#include "esp_attr.h"
#include "esp_heap_trace.h"
#include "esp_memory_utils.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace gea::framework::services {

namespace {

constexpr const char *kTag = "gea_esp32_app";

constexpr int kGeaAppInitTaskStack = GEA_EMBEDDED_GEA_INIT_TASK_STACK_BYTES;

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
constexpr std::size_t kAppInitHeapTraceRecords = 1024;
constexpr std::size_t kAppInitHeapTracePrintMinBytes = 128;

EXT_RAM_BSS_ATTR heap_trace_record_t appInitHeapTraceRecords[kAppInitHeapTraceRecords];
bool appInitHeapTraceReady = false;

void printHeapTracePcList(const void *const *pcs)
{
	for (int i = 0; i < CONFIG_HEAP_TRACING_STACK_DEPTH; ++i) {
		if (!pcs[i]) break;
		esp_rom_printf(" pc%d=0x%08x", i, static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(pcs[i])));
	}
}

void startAppInitHeapTrace()
{
	if (!appInitHeapTraceReady) {
		const esp_err_t init = heap_trace_init_standalone(appInitHeapTraceRecords, kAppInitHeapTraceRecords);
		appInitHeapTraceReady = (init == ESP_OK);
		esp_rom_printf("[heap-trace] app_init init=%d records=%u\n",
			static_cast<int>(init),
			static_cast<unsigned>(kAppInitHeapTraceRecords));
	}
	if (!appInitHeapTraceReady) return;
	const esp_err_t start = heap_trace_start(HEAP_TRACE_LEAKS);
	esp_rom_printf("[heap-trace] app_init start=%d\n", static_cast<int>(start));
}

void stopAndDumpAppInitHeapTrace()
{
	if (!appInitHeapTraceReady) return;
	const esp_err_t stop = heap_trace_stop();
	heap_trace_summary_t summary{};
	const esp_err_t summaryErr = heap_trace_summary(&summary);
	esp_rom_printf(
		"[heap-trace] app_init stop=%d summary=%d mode=%d allocs=%u frees=%u count=%u capacity=%u high=%u overflow=%u\n",
		static_cast<int>(stop),
		static_cast<int>(summaryErr),
		static_cast<int>(summary.mode),
		static_cast<unsigned>(summary.total_allocations),
		static_cast<unsigned>(summary.total_frees),
		static_cast<unsigned>(summary.count),
		static_cast<unsigned>(summary.capacity),
		static_cast<unsigned>(summary.high_water_mark),
		static_cast<unsigned>(summary.has_overflowed));

	std::uint32_t internalLiveBytes = 0;
	std::uint32_t internalLiveBlocks = 0;
	std::uint32_t printedBlocks = 0;
	const size_t count = heap_trace_get_count();
	for (size_t i = 0; i < count; ++i) {
		heap_trace_record_t record{};
		if (heap_trace_get(i, &record) != ESP_OK || !record.address || record.freed) continue;
		if (!esp_ptr_internal(record.address)) continue;
		internalLiveBytes += static_cast<std::uint32_t>(record.size);
		internalLiveBlocks += 1;
		if (record.size < kAppInitHeapTracePrintMinBytes) continue;
		printedBlocks += 1;
		esp_rom_printf("[heap-live] scope=app_init ptr=0x%08x size=%u",
			static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(record.address)),
			static_cast<unsigned>(record.size));
		printHeapTracePcList(record.alloced_by);
		esp_rom_printf("\n");
	}
	esp_rom_printf("[heap-trace] app_init internal_live_blocks=%u internal_live_bytes=%u printed=%u min_print=%u\n",
		static_cast<unsigned>(internalLiveBlocks),
		static_cast<unsigned>(internalLiveBytes),
		static_cast<unsigned>(printedBlocks),
		static_cast<unsigned>(kAppInitHeapTracePrintMinBytes));
}
#endif

#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
void logCurrentTaskStack(const char *stage, int stackBytes)
{
	TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
	ESP_LOGI(kTag,
		"stack probe [%s] task=%s stack_arg=%d hwm=%u",
		stage ? stage : "?",
		currentTask ? pcTaskGetName(currentTask) : "?",
		stackBytes,
		static_cast<unsigned>(uxTaskGetStackHighWaterMark(currentTask)));
}
#endif

struct InitTaskArgs {
	int width = 0;
	int height = 0;
	SemaphoreHandle_t done = nullptr;
	bool ok = false;
};

class GeaInitTask {
public:
	static void run(void *arg)
	{
		auto *args = static_cast<InitTaskArgs *>(arg);
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		logCurrentTaskStack("gea_init:start", kGeaAppInitTaskStack);
#endif
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		HeapProbe::log("gea_init:start");
#endif
		AppState::lock();
		bool ok = true;
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
		startAppInitHeapTrace();
#endif
		gea::framework::app::Application::init(args->width, args->height);
		AppState::afterAppInit();
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
		stopAndDumpAppInitHeapTrace();
#endif
		AppState::unlock();
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		logCurrentTaskStack("gea_init:after_app_init", kGeaAppInitTaskStack);
#endif
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		HeapProbe::log("gea_init:after_app_init");
#endif
		args->ok = ok;
		xSemaphoreGive(args->done);
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		logCurrentTaskStack("gea_init:before_delete", kGeaAppInitTaskStack);
#endif
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		HeapProbe::log("gea_init:before_delete");
#endif
		vTaskDeleteWithCaps(nullptr);
	}
};

bool runGeaInitTask(int width, int height)
{
	SemaphoreHandle_t done = xSemaphoreCreateBinary();
	if (!done) {
		ESP_LOGE(kTag, "Failed to create gea init semaphore");
		return false;
	}

	InitTaskArgs args{};
	args.width = width;
	args.height = height;
	args.done = done;

	TaskHandle_t task = nullptr;
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	HeapProbe::log("app_runner:before_gea_init_task");
#endif
	BaseType_t created = pdFAIL;
#if CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY
	// Try SPIRAM stack first. Previously gated on `!defined(GEA_EMBEDDED_APP_USES_BLE)`
	// — the assumption being that BT controller interrupts couldn't tolerate
	// stacks in PSRAM. In practice gea_init is a one-shot bootstrap task that
	// runs `__gea_top_level()` and then exits, so any BLE event handler that
	// preempted it would do so against a brief, idle stack — there's no
	// long-lived BLE-vs-PSRAM-cache contention. Removing the gate is
	// necessary for BLE applications that import `BLE` directly. The
	// analyzer correctly flags the active build as `USES_BLE`; the old
	// gate then forced gea_init into internal DRAM, and the ~64 KB stack
	// didn't fit in the post-init internal heap (largest contiguous ~31 KB
	// after WiFi/LWIP/diag/OTA init).
	//
	// Also dropped `MALLOC_CAP_8BIT` from the cap mask — under octal-PSRAM +
	// reserved-WiFi-pool, the `8BIT` cap excludes the large contiguous SPIRAM
	// chunks even though SPIRAM is genuinely byte-accessible. Stack memory is
	// only touched by FreeRTOS context save/restore (word-aligned) and user
	// code (via cache, which doesn't care about cap flags), so dropping the
	// 8BIT requirement here is safe.
	created = xTaskCreateWithCaps(
		&GeaInitTask::run,
		"gea_init",
		kGeaAppInitTaskStack,
		&args,
		5,
		&task,
		MALLOC_CAP_SPIRAM);
	ESP_LOGI("app_runner", "gea_init SPIRAM create: %s", created == pdPASS ? "ok" : "FAIL");
#endif
	if (created != pdPASS) {
		created = xTaskCreateWithCaps(
			&GeaInitTask::run,
			"gea_init",
			kGeaAppInitTaskStack,
			&args,
			5,
			&task,
			MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		ESP_LOGI("app_runner", "gea_init INTERNAL create: %s", created == pdPASS ? "ok" : "FAIL");
	}
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	HeapProbe::log(created == pdPASS ? "app_runner:after_gea_init_task" : "app_runner:after_gea_init_task_failed");
#endif
	if (created != pdPASS) {
		ESP_LOGE(kTag, "Failed to start gea init task");
		vSemaphoreDelete(done);
		return false;
	}

	xSemaphoreTake(done, portMAX_DELAY);
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
	HeapProbe::log("app_runner:after_gea_init_done");
#endif
	vSemaphoreDelete(done);
	return args.ok;
}

}  // namespace

bool AppRunner::initApplication(int width, int height)
{
	return runGeaInitTask(width, height);
}

}  // namespace gea::framework::services
