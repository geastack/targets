#include "apps/app_manager.h"

#include <cstdio>
#include <cstring>

#include "geaos.h"

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
// The image that plays Home on an OTA-slot board.
constexpr const char *kLauncherAppId = "app-launcher";
constexpr const char *kAbi = "esp32-ota";

class Esp32OtaAppHost final : public gea::geaos::AppHost {
public:
	const char *targetKey() const override { return "esp32-ota"; }
	const char *abi() const override { return kAbi; }

	const char *firmwareId() const override
	{
#ifdef GEA_EMBEDDED_OTA_DISABLED
		return "";
#else
		if (cachedId_[0] != '\0') return cachedId_;
		const esp_partition_t *running = esp_ota_get_running_partition();
		esp_app_desc_t desc;
		if (!running || esp_ota_get_partition_description(running, &desc) != ESP_OK) return "";
		std::snprintf(cachedId_, sizeof(cachedId_), "%s", desc.version);
		return cachedId_;
#endif
	}

	const char *runningAppId() const override
	{
		const char *id = firmwareId();
		return std::strcmp(id, kLauncherAppId) == 0 ? "" : id;
	}

	// Every other image in an app slot is an app; its version string is its id.
	std::vector<gea::geaos::InstalledApp> bundledApps() const override
	{
		std::vector<gea::geaos::InstalledApp> apps;
#ifndef GEA_EMBEDDED_OTA_DISABLED
		esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
		while (it) {
			const esp_partition_t *partition = esp_partition_get(it);
			esp_app_desc_t desc;
			if (partition && esp_ota_get_partition_description(partition, &desc) == ESP_OK &&
			    gea::geaos::isValidAppId(desc.version) && std::strcmp(desc.version, kLauncherAppId) != 0) {
				bool seen = false;
				for (const auto &app : apps) seen = seen || app.manifest.id == desc.version;
				if (!seen) {
					gea::geaos::InstalledApp app;
					app.manifest.id = desc.version;
					app.manifest.name = desc.project_name[0] ? desc.project_name : desc.version;
					app.manifest.abi = kAbi;
					app.volumeId = "firmware";
					app.location = partition->label;
					apps.push_back(std::move(app));
				}
			}
			it = esp_partition_next(it);
		}
		if (it) esp_partition_iterator_release(it);
#endif
		return apps;
	}

	bool launch(const gea::geaos::InstalledApp &app, std::string &error) override
	{
		return boot(app.manifest.id.c_str(), error);
	}

	bool home(std::string &error) override { return boot(kLauncherAppId, error); }

private:
	bool boot(const char *appId, std::string &error)
	{
#ifdef GEA_EMBEDDED_OTA_DISABLED
		(void)appId;
		error = "this firmware has no app slots";
		return false;
#else
		const esp_partition_t *running = esp_ota_get_running_partition();
		const esp_partition_t *partition = findAppPartition(appId, running);
		if (!partition) {
			ESP_LOGW(kTag, "No installed app image found for '%s'", appId);
			error = std::string("no app image for ") + appId;
			return false;
		}
		const esp_err_t err = esp_ota_set_boot_partition(partition);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Failed to select app '%s': %s", appId, esp_err_to_name(err));
			error = std::string("cannot select ") + appId + ": " + esp_err_to_name(err);
			return false;
		}
		ESP_LOGI(kTag, "Launching app '%s' from partition %s", appId, partition->label);
		vTaskDelay(pdMS_TO_TICKS(250));
		esp_restart();
		return true;
#endif
	}

#ifndef GEA_EMBEDDED_OTA_DISABLED
	static const esp_partition_t *findAppPartition(const char *appId, const esp_partition_t *skip)
	{
		esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr);
		const esp_partition_t *found = nullptr;
		while (it) {
			const esp_partition_t *partition = esp_partition_get(it);
			esp_app_desc_t desc;
			if (partition && (!skip || partition->address != skip->address) &&
			    esp_ota_get_partition_description(partition, &desc) == ESP_OK && std::strcmp(desc.version, appId) == 0) {
				found = partition;
				break;
			}
			it = esp_partition_next(it);
		}
		if (it) esp_partition_iterator_release(it);
		return found;
	}
#endif

	mutable char cachedId_[64]{};
};

Esp32OtaAppHost &hostInstance()
{
	static Esp32OtaAppHost host;
	return host;
}

// Registers before app_main. A board with loadable apps installs its geapp
// host later, which replaces this one.
struct HostRegistration {
	HostRegistration()
	{
		if (!gea::geaos::Launcher::host()) gea::geaos::Launcher::setHost(&hostInstance());
	}
};

HostRegistration registration;

}  // namespace

gea::geaos::AppHost &launcherPlatform()
{
	return hostInstance();
}

}  // namespace gea::platform::esp32::apps
