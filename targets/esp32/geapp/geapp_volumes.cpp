// SPDX-License-Identifier: GPL-3.0-only
// The volume monitor on ESP32: a low-priority task polls every volume
// source (an SD slot, a USB drive) so a card that comes or goes is noticed
// whether Home or an app is on screen. While an app runs it only watches for
// volumes arriving and leaving; Home's own refresh rescans fully.
#include "geapp_host.h"

#include "platform/file_cache.h"
#include "program.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

extern "C" bool gea_geapp_flash_vfs_mount(const char *base, const char *label);
extern "C" const char *gea_geapp_flash_vfs_status(void);
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace gea::platform::geapp {

namespace {

constexpr char kTag[] = "geapp_vol";
constexpr TickType_t kPollPeriod = pdMS_TO_TICKS(2000);
constexpr char kFlashLabel[] = "apps";
constexpr char kFlashRoot[] = "/apps";

gea::geaos::VolumeMonitor g_monitor;
TaskHandle_t g_task = nullptr;

SemaphoreHandle_t lock()
{
	// Made on first use, from whichever task gets here first (the board's
	// start task may add a source before installHost() runs); a function
	// static is initialized once even when two tasks race to it.
	static StaticSemaphore_t storage;
	static SemaphoreHandle_t mutex = xSemaphoreCreateMutexStatic(&storage);
	return mutex;
}

bool pollLocked(bool rescanUnchanged)
{
	xSemaphoreTake(lock(), portMAX_DELAY);
	const bool changed = g_monitor.poll(rescanUnchanged);
	xSemaphoreGive(lock());
	return changed;
}

void monitorTask(void *)
{
	for (;;) {
		// A notification (card-detect edge) cuts the wait short.
		ulTaskNotifyTake(pdTRUE, kPollPeriod);
		if (pollLocked(!gea::framework::program::loaded())) ESP_LOGI(kTag, "apps changed (generation %u)", static_cast<unsigned>(gea::geaos::Launcher::generation()));
	}
}

}  // namespace

// A board with no storage of its own (no NAND, no card) keeps its apps in a
// FAT partition labelled "apps" on the boot flash, at /apps
// (geapp_flash_vfs.c: every access runs on an internal-stack worker).
const char *flashVolumeStatus() { return gea_geapp_flash_vfs_status(); }

bool mountFlashVolume()
{
	static bool mounted = false;
	if (mounted) return true;
	if (!gea_geapp_flash_vfs_mount(kFlashRoot, kFlashLabel)) return false;
	mounted = true;
	const gea::geaos::Volume volume{"flash", kFlashRoot, false, true, "Internal storage"};
	gea::geaos::recoverInstalls(gea::geaos::posixFs(), volume);
	gea::geaos::Launcher::addVolume(volume);
	return true;
}

void addVolumeSource(gea::geaos::VolumeSource *source)
{
	xSemaphoreTake(lock(), portMAX_DELAY);
	g_monitor.add(source);
	xSemaphoreGive(lock());
}

bool pollVolumes()
{
	(void)gea::platform::storage::ensureMounted();
	return pollLocked(true);
}

void wakeVolumeMonitor()
{
	if (g_task) xTaskNotifyGive(g_task);
}

void wakeVolumeMonitorFromIsr()
{
	if (!g_task) return;
	BaseType_t woken = pdFALSE;
	vTaskNotifyGiveFromISR(g_task, &woken);
	portYIELD_FROM_ISR(woken);
}

void startVolumeMonitor()
{
	if (g_task) return;
	// The scan parses manifests (JSON): 6 KB. PSRAM: internal RAM is what the
	// apps run short of, and this task never runs with the cache off.
	if (xTaskCreatePinnedToCoreWithCaps(&monitorTask, "geapp_volumes", 6144, nullptr, tskIDLE_PRIORITY + 1, &g_task, 0,
	                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
		g_task = nullptr;
		if (xTaskCreatePinnedToCore(&monitorTask, "geapp_volumes", 6144, nullptr, tskIDLE_PRIORITY + 1, &g_task, 0) != pdPASS) {
			g_task = nullptr;
			ESP_LOGW(kTag, "no volume monitor: cards are seen only on a rescan");
		}
	}
}

}  // namespace gea::platform::geapp
