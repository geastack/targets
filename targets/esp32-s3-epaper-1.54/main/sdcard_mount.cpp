#include "platform/file_cache.h"

#include <sys/stat.h>

#include "board.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace gea::platform::storage {

namespace {

constexpr char kTag[] = "sdcard";
bool g_attempted = false;
bool g_mounted = false;
sdmmc_card_t *g_card = nullptr;

bool mountSdCard()
{
	struct stat st {};
	if (stat("/sdcard", &st) == 0) {
		g_mounted = true;
		return true;
	}

	esp_vfs_fat_sdmmc_mount_config_t mountConfig = {};
	mountConfig.format_if_mount_failed = false;
	mountConfig.max_files = 8;

	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	// Match the pala_note reference (same board, same CLK/CMD/D0 pins): run the
	// 1-bit SD bus at high speed (40 MHz) instead of the conservative 20 MHz
	// default. Slow SD writes in the mic capture task are what overrun the I2S
	// RX DMA and drop recorded samples; halving the write time removes that
	// headroom pressure.
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

	sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
	slot.width = 1;
	slot.clk = gea::platform::board::Board::storage.clk;
	slot.cmd = gea::platform::board::Board::storage.cmd;
	slot.d0 = gea::platform::board::Board::storage.d0;
	slot.d1 = GPIO_NUM_NC;
	slot.d2 = GPIO_NUM_NC;
	slot.d3 = GPIO_NUM_NC;
	slot.d4 = GPIO_NUM_NC;
	slot.d5 = GPIO_NUM_NC;
	slot.d6 = GPIO_NUM_NC;
	slot.d7 = GPIO_NUM_NC;
	slot.cd = GPIO_NUM_NC;
	slot.wp = GPIO_NUM_NC;
	slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

	ESP_LOGI(kTag,
	         "mounting /sdcard on SD_MMC one-bit bus clk=%d cmd=%d d0=%d",
	         static_cast<int>(slot.clk),
	         static_cast<int>(slot.cmd),
	         static_cast<int>(slot.d0));

	esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mountConfig, &g_card);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "mount /sdcard on SD_MMC failed: %s", esp_err_to_name(err));
		sdmmc_host_deinit();
		return false;
	}

	g_mounted = true;
	ESP_LOGI(kTag, "mounted /sdcard on SD_MMC one-bit bus");
	return true;
}

bool ensureStorageMounted()
{
	if (g_attempted) return g_mounted;
	g_attempted = true;
	g_mounted = mountSdCard();
	return g_mounted;
}

}  // namespace

void installSdCardFileCache()
{
	setMountProvider(&ensureStorageMounted);
}

}  // namespace gea::platform::storage
