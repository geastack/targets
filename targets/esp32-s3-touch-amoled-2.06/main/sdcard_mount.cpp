// Mounts the onboard microSD (TF) card at /sdcard for the persistent file cache
// (the maps OSM tile cache, voice-notes recordings, GEADEV PUSH, ...). Without
// this the registered mount provider is null, gea::platform::storage::ensureMounted()
// always returns false, and every tile re-downloads instead of reading from SD.
//
// The ESP32-S3-Touch-AMOLED-2.06 wires its TF slot for SD_MMC (SDIO) 1-bit mode,
// NOT SPI — per Waveshare's own pin_config.h: SDMMC_CLK=GPIO2, SDMMC_CMD=GPIO1,
// SDMMC_DATA(D0)=GPIO3. The ESP32-S3 SDMMC host routes these through the GPIO
// matrix, so the non-default pins are fine. (An SPI-mode mount on these pins
// returns ESP_ERR_INVALID_RESPONSE — the card is wired for SDIO.)
//
// Lazy + idempotent: the card isn't touched until the first ensureMounted() (a
// tile/file op or GEADEV PUSH). A missing/unformatted card just logs and returns
// false, so the app transparently falls back to no caching.
#include "platform/file_cache.h"
#include "board.h"

#include <sys/stat.h>

#include "driver/sdmmc_host.h"
#if defined(GEA_BOARD_MICROSD_SPI) && GEA_BOARD_MICROSD_SPI
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#endif
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace gea::platform::storage {

namespace {

constexpr char kTag[] = "sdcard";

// Waveshare ESP32-S3-Touch-AMOLED-2.06 TF slot (SD_MMC 1-bit, GPIO matrix).
constexpr gpio_num_t kPinClk = gea::platform::board::storage.clk;
constexpr gpio_num_t kPinCmd = gea::platform::board::storage.cmd;
constexpr gpio_num_t kPinD0 = gea::platform::board::storage.data0;

bool g_attempted = false;
bool g_mounted = false;
sdmmc_card_t *g_card = nullptr;

#if defined(GEA_BOARD_MICROSD_SPI) && GEA_BOARD_MICROSD_SPI
// A slot wired for SPI (the Elecrow CrowPanel Advance's, whose chip select is
// hard-wired low). SD_MMC cannot drive it: DAT3 low at CMD0 puts the card in
// SPI mode, so it must be spoken to as SPI.
bool mountSdCardSpi(const esp_vfs_fat_sdmmc_mount_config_t &mountConfig)
{
	constexpr auto &spi = gea::platform::board::storageSpi;
	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	host.slot = SPI2_HOST;
	host.max_freq_khz = SDMMC_FREQ_DEFAULT;  // 20 MHz; the card shares its lines with other functions

	spi_bus_config_t bus = {};
	bus.mosi_io_num = spi.mosi;
	bus.miso_io_num = spi.miso;
	bus.sclk_io_num = spi.sck;
	bus.quadwp_io_num = GPIO_NUM_NC;
	bus.quadhd_io_num = GPIO_NUM_NC;
	bus.max_transfer_sz = 16 * 1024;
	esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(host.slot), &bus, SDSPI_DEFAULT_DMA);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
		return false;
	}

	sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
	slot.host_id = static_cast<spi_host_device_t>(host.slot);
	slot.gpio_cs = spi.cs;  // GPIO_NUM_NC when the board ties chip select

	ESP_LOGI(kTag, "mounting /sdcard SPI sck=%d mosi=%d miso=%d cs=%d", static_cast<int>(spi.sck),
	         static_cast<int>(spi.mosi), static_cast<int>(spi.miso), static_cast<int>(spi.cs));
	err = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &mountConfig, &g_card);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "mount /sdcard failed: %s (card inserted + FAT-formatted? DIP switch on SD?)", esp_err_to_name(err));
		spi_bus_free(static_cast<spi_host_device_t>(host.slot));
		return false;
	}
	ESP_LOGI(kTag, "mounted /sdcard (SPI)");
	return true;
}
#endif

bool mountSdCard()
{
#if defined(GEA_BOARD_MICROSD_SPI) && GEA_BOARD_MICROSD_SPI
	{
		struct stat st {};
		if (stat("/sdcard", &st) == 0) return true;
		esp_vfs_fat_sdmmc_mount_config_t mountConfig = {};
		mountConfig.format_if_mount_failed = false;
		mountConfig.max_files = 3;
		mountConfig.allocation_unit_size = 16 * 1024;
		return mountSdCardSpi(mountConfig);
	}
#endif
	if (kPinClk == GPIO_NUM_NC || kPinCmd == GPIO_NUM_NC || kPinD0 == GPIO_NUM_NC) {
		ESP_LOGI(kTag, "no microSD pins configured; storage cache disabled");
		return false;
	}
	struct stat st {};
	if (stat("/sdcard", &st) == 0) {
		g_mounted = true;
		return true;
	}

	esp_vfs_fat_sdmmc_mount_config_t mountConfig = {};
	mountConfig.format_if_mount_failed = false;
	mountConfig.max_files = 3;
	mountConfig.allocation_unit_size = 16 * 1024;

	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40 MHz; init still steps down to 400 kHz

	sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
	slot.width = 1;
	slot.clk = kPinClk;
	slot.cmd = kPinCmd;
	slot.d0 = kPinD0;
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

	ESP_LOGI(kTag, "mounting /sdcard SD_MMC 1-bit clk=%d cmd=%d d0=%d", static_cast<int>(slot.clk),
	         static_cast<int>(slot.cmd), static_cast<int>(slot.d0));

	esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot, &mountConfig, &g_card);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "mount /sdcard failed: %s (card inserted + FAT-formatted?)", esp_err_to_name(err));
		return false;
	}

	g_mounted = true;
	ESP_LOGI(kTag, "mounted /sdcard (SD_MMC 1-bit)");
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
