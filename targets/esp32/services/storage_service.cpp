#include "services/storage_service.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <cstddef>
#include <cstdio>
#include <string>

#if defined(GEA_STORAGE_SPIFFS)
#include "esp_spiffs.h"
#endif

namespace gea::framework::services {

namespace {

constexpr const char *kTag = "gea_esp32_storage";
constexpr const char *kNamespace = "gea";
// Single NVS blob holding the whole app-facing localStorage set. A blob (not a
// str) so values may be arbitrary length and contain NUL. Distinct key from any
// device-setting string (e.g. "default_app"), so they coexist in one namespace.
constexpr const char *kLocalStorageKey = "ls_kv";

#if defined(GEA_STORAGE_SPIFFS)
// Persist the whole localStorage blob to a file on a dedicated SPIFFS partition
// on the main flash. The stock NVS partition is only 24KB (shared with BLE
// bonds) and fills up, silently dropping saved config/profiles on reboot; a
// SPIFFS partition on the 32MB flash has real room. Targets without a `storage`
// partition (or without GEA_STORAGE_SPIFFS) keep using the NVS blob below.
constexpr const char *kSpiffsBase = "/storage";
constexpr const char *kSpiffsPartition = "storage";
constexpr const char *kKvPath = "/storage/ls_kv.blob";
bool g_spiffs_ok = false;
#endif

}  // namespace

bool StorageService::init()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	if (ret != ESP_OK) {
		ESP_LOGE(kTag, "NVS init failed: %s", esp_err_to_name(ret));
		return false;
	}
#if defined(GEA_STORAGE_SPIFFS)
	esp_vfs_spiffs_conf_t conf = {};
	conf.base_path = kSpiffsBase;
	conf.partition_label = kSpiffsPartition;
	conf.max_files = 4;
	conf.format_if_mount_failed = true;
	const esp_err_t sret = esp_vfs_spiffs_register(&conf);
	g_spiffs_ok = (sret == ESP_OK);
	if (g_spiffs_ok) {
		std::size_t total = 0, used = 0;
		if (esp_spiffs_info(kSpiffsPartition, &total, &used) == ESP_OK)
			ESP_LOGI(kTag, "SPIFFS mounted: %u/%u bytes used", static_cast<unsigned>(used),
			         static_cast<unsigned>(total));
	} else {
		ESP_LOGW(kTag, "SPIFFS mount failed: %s (localStorage falls back to NVS)",
		         esp_err_to_name(sret));
	}
#endif
	return true;
}

bool StorageService::getString(const char *key, char *buf, unsigned capacity)
{
	if (!key || !buf || capacity == 0) return false;
	nvs_handle_t handle;
	if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return false;
	std::size_t len = capacity;
	const esp_err_t err = nvs_get_str(handle, key, buf, &len);
	nvs_close(handle);
	return err == ESP_OK;
}

bool StorageService::setString(const char *key, const char *value)
{
	if (!key || !value) return false;
	nvs_handle_t handle;
	if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
	esp_err_t err = nvs_set_str(handle, key, value);
	if (err == ESP_OK) err = nvs_commit(handle);
	nvs_close(handle);
	if (err != ESP_OK) ESP_LOGW(kTag, "setString(%s) failed: %s", key, esp_err_to_name(err));
	return err == ESP_OK;
}

bool StorageService::loadKv(std::string &out)
{
	out.clear();
#if defined(GEA_STORAGE_SPIFFS)
	if (g_spiffs_ok) {
		FILE *f = std::fopen(kKvPath, "rb");
		if (!f) return true;  // no file yet == empty store (not an error)
		std::fseek(f, 0, SEEK_END);
		const long n = std::ftell(f);
		std::fseek(f, 0, SEEK_SET);
		if (n > 0) {
			out.resize(static_cast<std::size_t>(n));
			const std::size_t rd = std::fread(out.data(), 1, static_cast<std::size_t>(n), f);
			if (rd != static_cast<std::size_t>(n)) out.resize(rd);
		}
		std::fclose(f);
		return true;
	}
#endif
	nvs_handle_t handle;
	if (nvs_open(kNamespace, NVS_READONLY, &handle) != ESP_OK) return false;
	std::size_t len = 0;
	esp_err_t err = nvs_get_blob(handle, kLocalStorageKey, nullptr, &len);
	if (err == ESP_OK && len > 0) {
		out.resize(len);
		err = nvs_get_blob(handle, kLocalStorageKey, out.data(), &len);
		if (err != ESP_OK) out.clear();
	}
	nvs_close(handle);
	return err == ESP_OK;
}

void StorageService::saveKv(const std::string &blob)
{
#if defined(GEA_STORAGE_SPIFFS)
	if (g_spiffs_ok) {
		if (blob.empty()) {
			std::remove(kKvPath);
			return;
		}
		FILE *f = std::fopen(kKvPath, "wb");
		if (!f) {
			ESP_LOGW(kTag, "saveKv: fopen(%s) failed", kKvPath);
			return;
		}
		const std::size_t wr = std::fwrite(blob.data(), 1, blob.size(), f);
		std::fclose(f);
		if (wr != blob.size()) ESP_LOGW(kTag, "saveKv: short write %u/%u", static_cast<unsigned>(wr),
		                                static_cast<unsigned>(blob.size()));
		return;
	}
#endif
	nvs_handle_t handle;
	if (nvs_open(kNamespace, NVS_READWRITE, &handle) != ESP_OK) return;
	esp_err_t err = blob.empty() ? nvs_erase_key(handle, kLocalStorageKey)
	                             : nvs_set_blob(handle, kLocalStorageKey, blob.data(), blob.size());
	// Erasing a never-written key is fine — still commit so the store is durable.
	if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) err = nvs_commit(handle);
	nvs_close(handle);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
		ESP_LOGW(kTag, "saveKv failed: %s", esp_err_to_name(err));
}

}  // namespace gea::framework::services
