// Mounts the board's microSD (storage.microSD, SD_MMC 1-bit) at /sdcard for the persistent file cache (e.g. the
// maps OSM tile cache). Lazy + idempotent (mounts on first use).
//
// The ESP32-P4 has ONE SDMMC host controller with TWO slots. The hosted WiFi
// co-processor (esp_hosted) creates that controller at boot and uses SLOT 1.
// The microSD is a separate device on its own pins — it just needs to live on the
// OTHER slot (SLOT 0). The catch: esp_vfs_fat_sdmmc_mount() calls sdmmc_host_init(),
// which tries to create a SECOND controller and fails ("no available sd host
// controller"). So we DON'T re-init the host: we add slot 0 to the controller
// esp_hosted already made (sdmmc_host_init_slot reuses the shared static s_ctlr),
// then probe the card and mount FATFS manually. Both run on the one host at once.
#include "platform/file_cache.h"

#include <cstdlib>
#include <sys/stat.h>

#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "board.h"
#include "sdmmc_cmd.h"

namespace gea::platform::storage {

namespace {
constexpr char kTag[] = "sdcard";
bool g_attempted = false;
bool g_mounted = false;
sdmmc_card_t *g_card = nullptr;

bool mountSdCard()
{
	if (g_attempted) return g_mounted;
	g_attempted = true;
#if !GEA_BOARD_HAS_MICROSD
	return false;
#endif

	// Reuse an existing mount (e.g. the camera already mounted /sdcard).
	struct stat st {};
	if (stat("/sdcard", &st) == 0) {
		g_mounted = true;
		return true;
	}

	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.slot = SDMMC_HOST_SLOT_0;             // the SD's slot (the hosted WiFi has slot 1)
	// 20 MHz default speed: the slot is 1-bit and runs over board-length lines.
	host.max_freq_khz = SDMMC_FREQ_DEFAULT;
	// The card is powered from VDD_3V3, not an on-chip LDO channel.
	host.flags = SDMMC_HOST_FLAG_1BIT;

	// 1-bit: only CLK, CMD and D0 come from the definition.
	sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
	slot.width = 1;
	slot.clk = gea::platform::board::storage.clk;
	slot.cmd = gea::platform::board::storage.cmd;
	slot.d0 = gea::platform::board::storage.data0;
	slot.d1 = GPIO_NUM_NC;
	slot.d2 = GPIO_NUM_NC;
	slot.d3 = GPIO_NUM_NC;
	slot.cd = SDMMC_SLOT_NO_CD;
	slot.wp = SDMMC_SLOT_NO_WP;
	slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

	// Add slot 0 to the EXISTING controller (esp_hosted's). NOT sdmmc_host_init().
	esp_err_t err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot);
	if (err == ESP_ERR_INVALID_STATE) {
		// Wi-Fi has not brought the shared controller up (an app with no network
		// use never starts esp_hosted): create it here instead.
		if (sdmmc_host_init() == ESP_OK) err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot);
	}
	if (err != ESP_OK) {
		ESP_LOGW(kTag, "init slot 0 failed (%s); file cache disabled", esp_err_to_name(err));
		return false;
	}

	g_card = static_cast<sdmmc_card_t *>(std::calloc(1, sizeof(sdmmc_card_t)));
	if (!g_card) return false;
	err = sdmmc_card_init(&host, g_card);
	if (err != ESP_OK) {
		ESP_LOGW(kTag, "no card on SDMMC slot 0 (%s); file cache disabled", esp_err_to_name(err));
		return false;
	}

	// Mount FATFS manually (esp_vfs_fat_sdmmc_mount would try to re-init the host).
	BYTE pdrv = FF_DRV_NOT_USED;
	if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == FF_DRV_NOT_USED) {
		ESP_LOGW(kTag, "no free FATFS drive; file cache disabled");
		return false;
	}
	ff_diskio_register_sdmmc(pdrv, g_card);
	char drive[3] = {static_cast<char>('0' + pdrv), ':', '\0'};
	const esp_vfs_fat_conf_t conf = {
		.base_path = "/sdcard",
		.fat_drive = drive,
		.max_files = 4,
	};
	static FATFS *fs = nullptr;
	err = esp_vfs_fat_register_cfg(&conf, &fs);
	if (err != ESP_OK) {
		ESP_LOGW(kTag, "vfs_fat register failed (%s); file cache disabled", esp_err_to_name(err));
		ff_diskio_register_sdmmc(pdrv, nullptr);
		return false;
	}
	const FRESULT fr = f_mount(fs, drive, 1);
	if (fr != FR_OK) {
		ESP_LOGW(kTag, "f_mount failed (%d); file cache disabled", static_cast<int>(fr));
		esp_vfs_fat_unregister_path("/sdcard");
		ff_diskio_register_sdmmc(pdrv, nullptr);
		return false;
	}

	g_mounted = true;
	ESP_LOGI(kTag, "mounted /sdcard on SDMMC slot 0 (shared host) for file cache");
	return g_mounted;
}

}  // namespace

// Wire the microSD card as the persistent file-cache backend. Called once from
// app_main at startup — that reference is what guarantees this translation unit
// is linked, so the SD provider actually replaces the default (no-storage) one.
void installSdCardFileCache()
{
	setMountProvider(&mountSdCard);
}

}  // namespace gea::platform::storage
