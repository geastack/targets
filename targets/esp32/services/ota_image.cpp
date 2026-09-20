// The OTA image writer, separated from the HTTP OTA server in ota.cpp.
//
// Staging an image into the next OTA partition has nothing to do with how the
// bytes arrived: BLE OTA streams them over a GATT characteristic and needs no
// network stack. ota.cpp pulls in esp_http_server and only builds when the app
// declares the network capability, so while these methods lived there a
// BLE-only app -- the default the scaffold generates, and what lesson 3 of the
// embedded tutorial asks for -- compiled ble_hid.cpp's OTA path and then failed
// to link it. This file carries the transport-agnostic half and is built
// whenever OTA is reachable at all, by network or by BLE.
#include "services/ota.h"

#include "services/ota_image.h"

#include <cstdint>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

namespace gea::framework::services {

namespace {

constexpr const char *kTag = "gea_esp32_ota";

struct OtaImageSession {
	esp_ota_handle_t handle = 0;
	const esp_partition_t *partition = nullptr;
	std::size_t expected = 0;
	std::size_t received = 0;
	bool active = false;
};

OtaImageSession g_imageSession;

StaticTimer_t g_rebootTimerStorage;
TimerHandle_t g_rebootTimer = nullptr;

void rebootTimerCallback(TimerHandle_t)
{
	esp_restart();
}

}  // namespace

OtaServer::Result OtaServer::beginImage(std::size_t imageSize)
{
	abortImage();
	const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
	if (!partition) return Result::NoPartition;
	if (imageSize == 0 || imageSize > partition->size) return Result::InvalidSize;
	const esp_partition_t *running = esp_ota_get_running_partition();
	if (running && running->address == partition->address) return Result::NoPartition;

	esp_ota_handle_t handle = 0;
	const esp_err_t err = esp_ota_begin(partition, imageSize, &handle);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "esp_ota_begin failed: %s", esp_err_to_name(err));
		return Result::BeginFailed;
	}
	g_imageSession = {handle, partition, imageSize, 0, true};
	ESP_LOGI(kTag, "stream OTA started for %s (%u bytes)", partition->label,
	         static_cast<unsigned>(imageSize));
	return Result::Ok;
}

OtaServer::Result OtaServer::writeImage(const std::uint8_t *data, std::size_t length)
{
	if (!g_imageSession.active) return Result::NotStarted;
	if (!data || length == 0 || g_imageSession.received + length > g_imageSession.expected) {
		abortImage();
		return Result::InvalidSize;
	}
	const esp_err_t err = esp_ota_write(g_imageSession.handle, data, length);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "esp_ota_write failed: %s", esp_err_to_name(err));
		abortImage();
		return Result::WriteFailed;
	}
	g_imageSession.received += length;
	return Result::Ok;
}

OtaServer::Result OtaServer::finishImage()
{
	if (!g_imageSession.active) return Result::NotStarted;
	if (g_imageSession.received != g_imageSession.expected) {
		abortImage();
		return Result::SizeMismatch;
	}

	const esp_ota_handle_t handle = g_imageSession.handle;
	const esp_partition_t *partition = g_imageSession.partition;
	g_imageSession.active = false;
	if (esp_ota_end(handle) != ESP_OK) {
		g_imageSession = {};
		return Result::ValidationFailed;
	}
	if (esp_ota_set_boot_partition(partition) != ESP_OK) {
		g_imageSession = {};
		return Result::BootSelectionFailed;
	}
	ESP_LOGI(kTag, "stream OTA complete for %s (%u bytes)", partition->label,
	         static_cast<unsigned>(g_imageSession.received));
	g_imageSession = {};
	return Result::Ok;
}

void OtaServer::abortImage()
{
	if (g_imageSession.active) esp_ota_abort(g_imageSession.handle);
	g_imageSession = {};
}

std::size_t OtaServer::receivedBytes()
{
	return g_imageSession.received;
}

std::size_t OtaServer::expectedBytes()
{
	return g_imageSession.expected;
}

void OtaServer::rebootSoon()
{
	// Do not allocate a throwaway task after a multi-megabyte OTA. Radio stacks
	// can leave too little contiguous internal heap for xTaskCreate(), and the old
	// unchecked failure left the validated image selected but never rebooted.
	// A static software timer needs no heap and lets the BLE success notification
	// drain before restarting into the selected partition.
	if (!g_rebootTimer) {
		g_rebootTimer = xTimerCreateStatic(
			"ota_reboot", pdMS_TO_TICKS(500), pdFALSE, nullptr,
			&rebootTimerCallback, &g_rebootTimerStorage);
	}
	if (g_rebootTimer && xTimerStart(g_rebootTimer, 0) == pdPASS) return;

	// The timer command queue can only be unavailable under severe scheduler
	// pressure. Reboot directly rather than report a successful OTA that never
	// activates; the image has already passed esp_ota_end and boot selection.
	esp_restart();
}

}  // namespace gea::framework::services

namespace gea::targets::esp32::ota_debug {

std::atomic<int> httpStartError{-1};
std::atomic<bool> httpStarted{false};

int startError() { return httpStartError.load(std::memory_order_relaxed); }
bool started() { return httpStarted.load(std::memory_order_relaxed); }
std::uint32_t dmaFree() { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA); }
std::uint32_t dmaLargest() { return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA); }

}  // namespace gea::targets::esp32::ota_debug
