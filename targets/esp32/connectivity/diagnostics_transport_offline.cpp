#include "services/diagnostics_platform.h"

#include <cstdarg>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
#include "esp_heap_caps.h"
#endif

namespace gea::platform::esp32::diagnostics {

namespace {

constexpr char kTag[] = "gea_esp32_diag";

class OfflineListener final : public gea::framework::services::DiagnosticsListener {
public:
	bool listen(int) override { return false; }
	gea::framework::services::DiagnosticsPeer *accept() override { return nullptr; }
	int wait(gea::framework::services::DiagnosticsPeer *, bool, int, int *errOut) override
	{
		if (errOut) *errOut = 0;
		return gea::framework::services::kDiagnosticsWaitNone;
	}
};

class OfflinePlatform final : public gea::framework::services::DiagnosticsPlatform {
public:
	gea::framework::services::DiagnosticsListener &listener() override { return listener_; }
	void installLogSink(gea::framework::services::DiagnosticsVPrintSink) override {}
	void startTask(void (*)(void *), void *) override {}
	std::int64_t nowUs() override { return esp_timer_get_time(); }
	void sleepMs(int ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }

	void logInfo(const char *fmt, ...) override
	{
		std::va_list args;
		va_start(args, fmt);
		log(ESP_LOG_INFO, fmt, args);
		va_end(args);
	}
	void logWarn(const char *fmt, ...) override
	{
		std::va_list args;
		va_start(args, fmt);
		log(ESP_LOG_WARN, fmt, args);
		va_end(args);
	}
	void logError(const char *fmt, ...) override
	{
		std::va_list args;
		va_start(args, fmt);
		log(ESP_LOG_ERROR, fmt, args);
		va_end(args);
	}
	void logHeapProbe(const char *stage) override
	{
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		ESP_LOGI(kTag, "heap [%s] internal_free=%u largest=%u psram_free=%u",
		         stage ? stage : "?",
		         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
		         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
#else
		(void)stage;
#endif
	}
	void logCurrentTaskStackProbe(const char *stage) override
	{
		ESP_LOGI(kTag, "stack [%s] task=%s hwm=%u", stage ? stage : "?",
		         pcTaskGetName(nullptr), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
	}
	void logHeapTaskSummary(const char *) override {}

private:
	static void log(esp_log_level_t level, const char *fmt, std::va_list args)
	{
		char message[192];
		std::vsnprintf(message, sizeof(message), fmt, args);
		esp_log_write(level, kTag, "%s", message);
	}

	OfflineListener listener_;
};

OfflinePlatform &platform()
{
	static OfflinePlatform instance;
	return instance;
}

}  // namespace
}  // namespace gea::platform::esp32::diagnostics

namespace gea::framework::services {

DiagnosticsPlatform &diagnosticsPlatform()
{
	return gea::platform::esp32::diagnostics::platform();
}


}  // namespace gea::framework::services
