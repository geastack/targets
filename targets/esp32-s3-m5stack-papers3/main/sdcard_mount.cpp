#include "platform/file_cache.h"

#include "board.h"

#include <sys/stat.h>

#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

namespace gea::platform::storage {

namespace {

constexpr char kTag[] = "papers3_sd";
bool mounted = false;
sdmmc_card_t *card = nullptr;

bool ensureSpiBus()
{
	spi_bus_config_t config = {};
	config.mosi_io_num = gea::platform::board::storage.mosi;
	config.miso_io_num = gea::platform::board::storage.miso;
	config.sclk_io_num = gea::platform::board::storage.sclk;
	config.quadwp_io_num = -1;
	config.quadhd_io_num = -1;
	config.max_transfer_sz = 4096;
	const esp_err_t err = spi_bus_initialize(gea::platform::board::storage.spiHost, &config, SPI_DMA_CH_AUTO);
	return err == ESP_OK || err == ESP_ERR_INVALID_STATE;
}

bool mountCard()
{
	// SD is REAL on this board: SCLK 39 / MOSI 38 / MISO 40 / CS 47 (see
	// board.h), none of which collide with the panel's parallel bus. The LilyGo
	// early-out this file came from existed because that board's SD pins landed
	// on e-paper data lines; here the bus is verified working (upstream PaperBoy
	// mounts a card on these exact pins).
	struct stat st = {};
	if (stat("/sdcard", &st) == 0) return true;
	if (!ensureSpiBus()) return false;

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	host.slot = gea::platform::board::storage.spiHost;
	host.max_freq_khz = 20000;

	sdspi_device_config_t device = SDSPI_DEVICE_CONFIG_DEFAULT();
	device.host_id = gea::platform::board::storage.spiHost;
	device.gpio_cs = gea::platform::board::storage.cs;
	device.gpio_cd = SDSPI_SLOT_NO_CD;
	device.gpio_wp = SDSPI_SLOT_NO_WP;
	device.gpio_int = SDSPI_SLOT_NO_INT;

	esp_vfs_fat_mount_config_t mount = {};
	mount.format_if_mount_failed = false;
	mount.max_files = 8;
	mount.allocation_unit_size = 16 * 1024;
	const esp_err_t err = esp_vfs_fat_sdspi_mount("/sdcard", &host, &device, &mount, &card);
	if (err != ESP_OK) {
		ESP_LOGW(kTag, "microSD mount failed: %s", esp_err_to_name(err));
		return false;
	}
	ESP_LOGI(kTag, "microSD mounted on its own SPI2 bus CS=%d", static_cast<int>(device.gpio_cs));
	return true;
}

bool ensureStorageMounted()
{
	// A missing card or a card inserted too late is recoverable. Keep a
	// successful mount idempotent, but retry failures when the caller explicitly
	// asks for storage again (for example, the e-reader's RESCAN action).
	// mountCard() also accepts an already-initialized SPI bus, so the retry does
	// not require a reboot or tear down the board's dedicated SPI2 bus.
	if (!mounted) mounted = mountCard();
	return mounted;
}

}  // namespace

void installSdCardFileCache()
{
	setMountProvider(&ensureStorageMounted);
}

}  // namespace gea::platform::storage
