#include "apps/app_manager.h"

#include <cstdio>
#include <cstring>

#include "apps.h"

#include "esp_app_desc.h"
#include "esp_err.h"
#include "esp_log.h"
#ifndef GEA_EMBEDDED_OTA_DISABLED
#include "esp_ota_ops.h"
#endif
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::esp32::apps {

namespace {

constexpr const char *kTag = "esp32_apps";

class Esp32AppLauncherPlatform final : public gea::framework::apps::AppLauncherPlatform {
public:
	const char *currentInstalledAppId() override
	{
#ifdef GEA_EMBEDDED_OTA_DISABLED
		return nullptr;
#else
		if (cachedId_[0] != '\0') return cachedId_;

		const esp_partition_t *running = esp_ota_get_running_partition();
		if (!running) return nullptr;

		esp_app_desc_t desc;
		if (esp_ota_get_partition_description(running, &desc) != ESP_OK) return nullptr;

		std::snprintf(cachedId_, sizeof(cachedId_), "%s", desc.version);
		return cachedId_[0] ? cachedId_ : nullptr;
#endif
	}

	bool runningAppIsLauncher(const char *launcherAppId) override
	{
#ifdef GEA_EMBEDDED_OTA_DISABLED
		(void)launcherAppId;
		return false;
#else
		const esp_partition_t *running = esp_ota_get_running_partition();
		return running && partitionVersionMatches(running, launcherAppId);
#endif
	}

	bool launchInstalledApp(const char *appId) override
	{
#ifdef GEA_EMBEDDED_OTA_DISABLED
		(void)appId;
		return false;
#else
		if (!appId || appId[0] == '\0') return false;

		const esp_partition_t *running = esp_ota_get_running_partition();
		const esp_partition_t *partition = findAppPartition(appId, running);
		if (!partition) {
			ESP_LOGW(kTag, "No installed app image found for '%s'", appId);
			return false;
		}

		const esp_err_t err = esp_ota_set_boot_partition(partition);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Failed to select app '%s': %s", appId, esp_err_to_name(err));
			return false;
		}

		ESP_LOGI(kTag, "Launching app '%s' from partition %s", appId, partition->label);
		vTaskDelay(pdMS_TO_TICKS(250));
		esp_restart();
		return true;
#endif
	}

	void startLauncherButtonTask() override
	{
		gea::platform::esp32::apps::startLauncherButtonTask();
	}

private:
#ifndef GEA_EMBEDDED_OTA_DISABLED
	bool partitionVersionMatches(const esp_partition_t *partition, const char *appId) const
	{
		if (!partition || !appId) return false;
		esp_app_desc_t desc;
		if (esp_ota_get_partition_description(partition, &desc) != ESP_OK) return false;
		return std::strcmp(desc.version, appId) == 0;
	}

	const esp_partition_t *findAppPartitionByVersion(const char *appId, const esp_partition_t *skip) const
	{
		esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
		const esp_partition_t *found = nullptr;

		while (it) {
			const esp_partition_t *partition = esp_partition_get(it);
			if (partition && (!skip || partition->address != skip->address) && partitionVersionMatches(partition, appId)) {
				found = partition;
				break;
			}
			it = esp_partition_next(it);
		}

		if (it) esp_partition_iterator_release(it);
		return found;
	}

	const esp_partition_t *findPlannedAppPartition(const char *appId, const esp_partition_t *skip) const
	{
		for (int i = 0; i < gea::framework::apps::InstalledApps::planCount(); i++) {
			const gea::framework::apps::InstalledAppPlanEntry *entry = gea::framework::apps::InstalledApps::planAt(i);
			if (!entry || !entry->appId || std::strcmp(entry->appId, appId) != 0) continue;

			const esp_partition_t *partition =
				esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, entry->slotLabel);
			if (!partition) {
				ESP_LOGW(kTag, "Install plan maps '%s' to missing partition %s", appId, entry->slotLabel);
				continue;
			}
			if (skip && partition->address == skip->address) continue;

			esp_app_desc_t desc;
			const esp_err_t err = esp_ota_get_partition_description(partition, &desc);
			if (err == ESP_OK) {
				if (std::strcmp(desc.version, appId) != 0) {
					ESP_LOGW(kTag, "Install plan maps '%s' to %s, but slot contains '%s'", appId, partition->label, desc.version);
					continue;
				}
			} else {
				ESP_LOGW(kTag, "Install plan maps '%s' to %s, but descriptor read failed: %s", appId, partition->label,
				         esp_err_to_name(err));
				continue;
			}

			return partition;
		}

		return nullptr;
	}

	const esp_partition_t *findAppPartition(const char *appId, const esp_partition_t *skip) const
	{
		const esp_partition_t *partition = findAppPartitionByVersion(appId, skip);
		if (partition) return partition;

		partition = findPlannedAppPartition(appId, skip);
		if (partition) {
			ESP_LOGW(kTag, "Using installed app plan fallback for '%s' in %s", appId, partition->label);
		}
		return partition;
	}
#endif

	char cachedId_[64]{};
};

Esp32AppLauncherPlatform &platformInstance()
{
	static Esp32AppLauncherPlatform platform;
	return platform;
}

struct PlatformRegistration {
	PlatformRegistration()
	{
		gea::framework::apps::AppManager::setPlatform(&platformInstance());
	}
};

PlatformRegistration registration;

}  // namespace

gea::framework::apps::AppLauncherPlatform &launcherPlatform()
{
	return platformInstance();
}

}  // namespace gea::platform::esp32::apps
