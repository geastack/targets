#include "services/diagnostics_platform.h"

#include <cerrno>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "memory_config.h"

#include "esp_err.h"
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
#include "esp_heap_caps.h"
#endif
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include "sdkconfig.h"
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TASK_TRACKING
#include "esp_heap_task_info.h"
#endif
#include <unistd.h>

namespace gea::platform::esp32::diagnostics {

namespace {

constexpr const char *kTag = "gea_esp32_diag";
constexpr int kSocketTimeoutMs = 2000;

bool isBackpressure(int err)
{
	return err == EAGAIN || err == EWOULDBLOCK;
}

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
struct HeapMapDumpState {
	const char *stage = nullptr;
	const char *capsName = nullptr;
	int heapIndex = -1;
	intptr_t heapStart = 0;
	intptr_t heapEnd = 0;
	std::uint32_t freeBytes = 0;
	std::uint32_t usedBytes = 0;
	std::uint32_t largestFree = 0;
	std::uint32_t freeBlocks = 0;
	std::uint32_t usedBlocks = 0;
	bool haveHeap = false;
};

void finishHeapMapRegion(HeapMapDumpState &state)
{
	if (!state.haveHeap) return;
	esp_rom_printf(
		"[heap-map] stage=%s caps=%s heap=%d range=0x%08x-0x%08x free=%u used=%u largest_free=%u free_blocks=%u used_blocks=%u\n",
		state.stage ? state.stage : "?",
		state.capsName ? state.capsName : "?",
		state.heapIndex,
		static_cast<unsigned>(state.heapStart),
		static_cast<unsigned>(state.heapEnd),
		static_cast<unsigned>(state.freeBytes),
		static_cast<unsigned>(state.usedBytes),
		static_cast<unsigned>(state.largestFree),
		static_cast<unsigned>(state.freeBlocks),
		static_cast<unsigned>(state.usedBlocks));
}

bool heapMapWalker(walker_heap_into_t heapInfo, walker_block_info_t blockInfo, void *userData)
{
	auto &state = *static_cast<HeapMapDumpState *>(userData);
	if (!state.haveHeap || state.heapStart != heapInfo.start || state.heapEnd != heapInfo.end) {
		finishHeapMapRegion(state);
		state.haveHeap = true;
		state.heapIndex += 1;
		state.heapStart = heapInfo.start;
		state.heapEnd = heapInfo.end;
		state.freeBytes = 0;
		state.usedBytes = 0;
		state.largestFree = 0;
		state.freeBlocks = 0;
		state.usedBlocks = 0;
	}

	if (blockInfo.used) {
		state.usedBytes += static_cast<std::uint32_t>(blockInfo.size);
		state.usedBlocks += 1;
		return true;
	}

	state.freeBytes += static_cast<std::uint32_t>(blockInfo.size);
	state.freeBlocks += 1;
	if (blockInfo.size > state.largestFree) {
		state.largestFree = static_cast<std::uint32_t>(blockInfo.size);
	}
	esp_rom_printf(
		"[heap-free] stage=%s caps=%s heap=%d ptr=0x%08x size=%u\n",
		state.stage ? state.stage : "?",
		state.capsName ? state.capsName : "?",
		state.heapIndex,
		static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(blockInfo.ptr)),
		static_cast<unsigned>(blockInfo.size));
	return true;
}

void dumpHeapMapForCaps(const char *stage, const char *capsName, std::uint32_t caps)
{
	multi_heap_info_t info{};
	heap_caps_get_info(&info, caps);
	esp_rom_printf(
		"[heap-map] stage=%s caps=%s total_free=%u largest=%u min=%u free_blocks=%u alloc_blocks=%u total_blocks=%u\n",
		stage ? stage : "?",
		capsName ? capsName : "?",
		static_cast<unsigned>(info.total_free_bytes),
		static_cast<unsigned>(info.largest_free_block),
		static_cast<unsigned>(info.minimum_free_bytes),
		static_cast<unsigned>(info.free_blocks),
		static_cast<unsigned>(info.allocated_blocks),
		static_cast<unsigned>(info.total_blocks));
	HeapMapDumpState state{};
	state.stage = stage;
	state.capsName = capsName;
	heap_caps_walk(caps, heapMapWalker, &state);
	finishHeapMapRegion(state);
}

void dumpInternalHeapMap(const char *stage)
{
	dumpHeapMapForCaps(stage, "internal8", MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	dumpHeapMapForCaps(stage, "dma_internal", MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
}

bool shouldDumpInternalHeapMap(const char *stage)
{
	if (!stage) return false;
	return std::strcmp(stage, "runtime:start") == 0 ||
		std::strcmp(stage, "runtime:after_power") == 0 ||
		std::strcmp(stage, "runtime:after_display_init") == 0 ||
		std::strcmp(stage, "runtime:after_app_init") == 0 ||
		std::strcmp(stage, "runtime:after_display_start") == 0 ||
		std::strcmp(stage, "runtime:after_frame_start") == 0 ||
		std::strcmp(stage, "gea_init:after_app_init") == 0 ||
		std::strcmp(stage, "app_runner:after_gea_init_done") == 0;
}
#endif

class SocketPeer : public gea::framework::services::DiagnosticsPeer {
public:
	explicit SocketPeer(int fd)
		: fd_(fd)
	{
		configure(fd_);
	}

	~SocketPeer() override
	{
		close("closed", 0);
	}

	int fd() const
	{
		return fd_;
	}

	int write(const unsigned char *data, int len, int *err_out) override
	{
		if (err_out) *err_out = 0;
		if (!data || len <= 0) return 0;
		while (true) {
			int sent = send(fd_, data, static_cast<std::size_t>(len), MSG_NOSIGNAL | MSG_DONTWAIT);
			if (sent < 0) {
				int err = errno;
				if (err == EINTR) continue;
				if (err_out) *err_out = err;
				return isBackpressure(err) ? 0 : -1;
			}
			if (sent == 0) {
				if (err_out) *err_out = ECONNRESET;
				return -1;
			}
			return sent;
		}
	}

	int read(char *data, int len, int *err_out) override
	{
		if (err_out) *err_out = 0;
		if (!data || len <= 0) return 0;
		while (true) {
			int received = recv(fd_, data, static_cast<std::size_t>(len), MSG_DONTWAIT);
			if (received < 0) {
				int err = errno;
				if (err == EINTR) continue;
				if (err_out) *err_out = err;
				return isBackpressure(err) ? gea::framework::services::kDiagnosticsIoWouldBlock : -1;
			}
			return received;
		}
	}

	void close(const char *reason, int err) override
	{
		if (fd_ < 0) return;
		if (err) {
			ESP_LOGI(kTag, "Diagnostics client %s: errno=%d", reason, err);
		} else {
			ESP_LOGI(kTag, "Diagnostics client %s", reason);
		}
		shutdown(fd_, SHUT_RDWR);
		::close(fd_);
		fd_ = -1;
	}

private:
	static void configure(int fd)
	{
		struct timeval timeout = {};
		timeout.tv_sec = kSocketTimeoutMs / 1000;
		timeout.tv_usec = (kSocketTimeoutMs % 1000) * 1000;
		setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
		int nodelay = 1;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
	}

	int fd_;
};

class SocketListener : public gea::framework::services::DiagnosticsListener {
public:
	~SocketListener() override
	{
		if (serverFd_ >= 0) ::close(serverFd_);
	}

	bool listen(int port) override
	{
		if (serverFd_ >= 0) return true;
		serverFd_ = socket(AF_INET, SOCK_STREAM, 0);
		if (serverFd_ < 0) return false;

		int opt = 1;
		setsockopt(serverFd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

		struct sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		addr.sin_addr.s_addr = INADDR_ANY;
		if (bind(serverFd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0 ||
		    ::listen(serverFd_, 2) < 0) {
			::close(serverFd_);
			serverFd_ = -1;
			return false;
		}
		return true;
	}

	gea::framework::services::DiagnosticsPeer *accept() override
	{
		if (serverFd_ < 0) return nullptr;
		int fd = ::accept(serverFd_, nullptr, nullptr);
		if (fd < 0) return nullptr;
		return new SocketPeer(fd);
	}

	int wait(gea::framework::services::DiagnosticsPeer *peer,
	         bool wait_for_peer_write,
	         int timeout_ms,
	         int *err_out) override
	{
		if (err_out) *err_out = 0;
		if (serverFd_ < 0) return 0;

		SocketPeer *socket_peer = static_cast<SocketPeer *>(peer);
		fd_set read_fds;
		fd_set write_fds;
		FD_ZERO(&read_fds);
		FD_ZERO(&write_fds);
		FD_SET(serverFd_, &read_fds);
		int max_fd = serverFd_;

		if (socket_peer && socket_peer->fd() >= 0) {
			FD_SET(socket_peer->fd(), &read_fds);
			if (wait_for_peer_write) FD_SET(socket_peer->fd(), &write_fds);
			if (socket_peer->fd() > max_fd) max_fd = socket_peer->fd();
		}

		struct timeval timeout = {};
		timeout.tv_sec = timeout_ms / 1000;
		timeout.tv_usec = (timeout_ms % 1000) * 1000;
		int selected = select(max_fd + 1, &read_fds, &write_fds, nullptr, &timeout);
		if (selected < 0) {
			if (errno == EINTR) return 0;
			if (err_out) *err_out = errno;
			return -1;
		}

		int events = gea::framework::services::kDiagnosticsWaitNone;
		if (selected > 0 && FD_ISSET(serverFd_, &read_fds)) {
			events |= gea::framework::services::kDiagnosticsWaitListenerReadable;
		}
		if (socket_peer && socket_peer->fd() >= 0 && selected > 0) {
			if (FD_ISSET(socket_peer->fd(), &read_fds)) {
				events |= gea::framework::services::kDiagnosticsWaitPeerReadable;
			}
			if (wait_for_peer_write && FD_ISSET(socket_peer->fd(), &write_fds)) {
				events |= gea::framework::services::kDiagnosticsWaitPeerWritable;
			}
		}
		return events;
	}

private:
	int serverFd_ = -1;
};

class Platform : public gea::framework::services::DiagnosticsPlatform {
public:
	gea::framework::services::DiagnosticsListener &listener() override
	{
		return listener_;
	}

	void installLogSink(gea::framework::services::DiagnosticsVPrintSink sink) override
	{
		esp_log_set_vprintf(sink);
	}

	void startTask(void (*entry)(void *), void *context) override
	{
		xTaskCreate(entry, "diag_srv", 6144, context, 5, nullptr);
	}

	std::int64_t nowUs() override
	{
		return esp_timer_get_time();
	}

	void sleepMs(int ms) override
	{
		vTaskDelay(pdMS_TO_TICKS(ms));
	}

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
		std::uint32_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		std::uint32_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		std::uint32_t min_internal = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		std::uint32_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		const std::int32_t delta_internal = haveLastHeapProbe_
			? static_cast<std::int32_t>(free_internal) - static_cast<std::int32_t>(lastInternalFree_)
			: 0;
		const std::int32_t delta_psram = haveLastHeapProbe_
			? static_cast<std::int32_t>(free_psram) - static_cast<std::int32_t>(lastPsramFree_)
			: 0;
		ESP_LOGI(kTag, "heap probe [%s] internal_free=%u delta=%+d internal_largest=%u internal_min=%u psram_free=%u psram_delta=%+d",
		         stage ? stage : "?",
		         static_cast<unsigned>(free_internal),
		         static_cast<int>(delta_internal),
		         static_cast<unsigned>(largest_internal),
		         static_cast<unsigned>(min_internal),
		         static_cast<unsigned>(free_psram),
		         static_cast<int>(delta_psram));
		lastInternalFree_ = free_internal;
		lastPsramFree_ = free_psram;
		haveLastHeapProbe_ = true;
		if (shouldDumpInternalHeapMap(stage)) {
			dumpInternalHeapMap(stage);
		}
#else
		(void)stage;
#endif
	}

	void logCurrentTaskStackProbe(const char *stage) override
	{
#if GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG
		TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
		ESP_LOGI(kTag,
		         "stack probe [%s] task=%s hwm=%u",
		         stage ? stage : "?",
		         currentTask ? pcTaskGetName(currentTask) : "?",
		         static_cast<unsigned>(uxTaskGetStackHighWaterMark(currentTask)));
#else
		(void)stage;
#endif
	}

	void logHeapTaskSummary(const char *stage) override
	{
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TASK_TRACKING
		constexpr std::size_t kMaxTasks = 32;
		constexpr std::size_t kMaxHeaps = 128;
		task_stat_t taskStats[kMaxTasks]{};
		heap_stat_t heapStats[kMaxHeaps]{};
		heap_all_tasks_stat_t allStats{};
		allStats.task_count = kMaxTasks;
		allStats.stat_arr = taskStats;
		allStats.heap_count = kMaxHeaps;
		allStats.heap_stat_start = heapStats;
		allStats.alloc_count = 0;
		allStats.alloc_stat_start = nullptr;

		esp_err_t err = heap_caps_get_all_task_stat(&allStats);
		if (err != ESP_OK) {
			ESP_LOGW(kTag,
			         "heap task summary [%s] unavailable err=%d",
			         stage ? stage : "?",
			         static_cast<int>(err));
			return;
		}

		ESP_LOGI(kTag,
		         "heap task summary [%s] tasks=%u heaps=%u tracked_dynamic_heap_only=1",
		         stage ? stage : "?",
		         static_cast<unsigned>(allStats.task_count),
		         static_cast<unsigned>(allStats.heap_count));

		for (std::size_t taskIndex = 0; taskIndex < allStats.task_count; ++taskIndex) {
			const task_stat_t &task = allStats.stat_arr[taskIndex];
			if (task.overall_current_usage == 0 && task.overall_peak_usage == 0) continue;

			std::size_t dramDma = 0;
			std::size_t internalOther = 0;
			std::size_t psram = 0;
			std::size_t other = 0;
			for (std::size_t heapIndex = 0; heapIndex < task.heap_count && task.heap_stat; ++heapIndex) {
				const heap_stat_t &heap = task.heap_stat[heapIndex];
				const std::uint32_t caps = heap.caps;
				if ((caps & MALLOC_CAP_SPIRAM) != 0) {
					psram += heap.current_usage;
				} else if ((caps & MALLOC_CAP_INTERNAL) != 0 && (caps & MALLOC_CAP_DMA) != 0) {
					dramDma += heap.current_usage;
				} else if ((caps & MALLOC_CAP_INTERNAL) != 0) {
					internalOther += heap.current_usage;
				} else {
					other += heap.current_usage;
				}
			}

			ESP_LOGI(kTag,
			         "heap task [%s] name=%s alive=%d current=%u peak=%u heaps=%u dram_dma=%u internal_other=%u psram=%u other=%u",
			         stage ? stage : "?",
			         task.name,
			         task.is_alive ? 1 : 0,
			         static_cast<unsigned>(task.overall_current_usage),
			         static_cast<unsigned>(task.overall_peak_usage),
			         static_cast<unsigned>(task.heap_count),
			         static_cast<unsigned>(dramDma),
			         static_cast<unsigned>(internalOther),
			         static_cast<unsigned>(psram),
			         static_cast<unsigned>(other));
		}
#elif GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG
		ESP_LOGI(kTag,
		         "heap task summary [%s] disabled; enable CONFIG_HEAP_TASK_TRACKING to attribute dynamic heap by task",
		         stage ? stage : "?");
#else
		(void)stage;
#endif
	}

private:
	void log(esp_log_level_t level, const char *fmt, std::va_list args)
	{
		char message[192];
		std::vsnprintf(message, sizeof(message), fmt, args);
		esp_log_write(level, kTag, "%s", message);
	}

	SocketListener listener_;
	bool haveLastHeapProbe_ = false;
	std::uint32_t lastInternalFree_ = 0;
	std::uint32_t lastPsramFree_ = 0;
};

}  // namespace

Platform &platform()
{
	static Platform instance;
	return instance;
}

}  // namespace gea::platform::esp32::diagnostics

namespace gea::framework::services {

DiagnosticsPlatform &diagnosticsPlatform()
{
	return gea::platform::esp32::diagnostics::platform();
}

}  // namespace gea::framework::services
