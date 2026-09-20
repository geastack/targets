#include "memory.h"

#include "display.h"
#include "memory_config.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::memory {

std::uint32_t Memory::internalFree() {
	return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

std::uint32_t Memory::internalLargestFreeBlock() {
	return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

std::uint32_t Memory::internalMinimumFree() {
	return heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

std::uint32_t Memory::psramFree() {
	return heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

std::uint32_t Memory::currentTaskStackHighWaterMark() {
	return uxTaskGetStackHighWaterMark(nullptr);
}

std::uint32_t Memory::geaMainStackBytes() {
	return GEA_EMBEDDED_GEA_MAIN_TASK_STACK_BYTES;
}

std::uint32_t Memory::geaInitStackBytes() {
	return GEA_EMBEDDED_GEA_INIT_TASK_STACK_BYTES;
}

std::uint32_t Memory::appFrameStackWords() {
	return GEA_EMBEDDED_APP_FRAME_TASK_STACK_WORDS;
}

std::uint32_t Memory::appFrameStackBytes() {
	return GEA_EMBEDDED_APP_FRAME_TASK_STACK_WORDS * sizeof(StackType_t);
}

std::uint32_t Memory::displayFlushConfiguredRows() {
	return GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX;
}

std::uint32_t Memory::displayFlushConfiguredDepth() {
	return GEA_EMBEDDED_DISPLAY_FLUSH_QUEUE_DEPTH;
}

std::uint32_t Memory::displayFlushBufferMaxBytes() {
	return GEA_EMBEDDED_DISPLAY_FLUSH_BUFFER_MAX_BYTES;
}

std::uint32_t Memory::displayFlushRows() {
	return gea::platform::display::Display::flushChunkRows();
}

std::uint32_t Memory::displayFlushDepth() {
	return gea::platform::display::Display::flushQueueDepth();
}

std::uint32_t Memory::displayFlushBufferBytes() {
	return gea::platform::display::Display::flushBufferBytes();
}


// MALLOC_CAP_DMA because that is what an SPI/LCD driver's descriptors and
// transaction pool need, and it is the capability that runs out first on a board
// whose framebuffer lives in PSRAM: the failure this exists for was 3,236 bytes
// free, largest block 1,600, and "SPI bus init failed: ESP_ERR_NO_MEM" one
// millisecond after the framebuffer landed in PSRAM.
void *Memory::reserveInternalDma(std::size_t bytes) {
	if (bytes == 0) return nullptr;
	void *reserve = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
	if (reserve == nullptr) {
		ESP_LOGW("gea_memory",
		         "internal DMA reserve of %u bytes unavailable; free=%u largest=%u",
		         static_cast<unsigned>(bytes),
		         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)),
		         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)));
	} else {
		ESP_LOGI("gea_memory", "internal DMA reserve: %u bytes held", static_cast<unsigned>(bytes));
	}
	return reserve;
}

void Memory::releaseInternalDma(void *reserve) {
	if (reserve == nullptr) return;
	heap_caps_free(reserve);
	ESP_LOGI("gea_memory",
	         "internal DMA reserve released; free=%u largest=%u",
	         static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)),
	         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)));
}

}  // namespace gea::platform::memory
