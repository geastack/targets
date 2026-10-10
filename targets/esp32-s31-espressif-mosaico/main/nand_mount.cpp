// The board's 1 Gbit SPI NAND as its file storage, FAT on Espressif's
// spi_nand_flash driver (Dhara wear levelling), mounted at /nand.
//
// It is where an app keeps what is too large for its app slot: the NOR holds
// the firmware and the slot caps an image near 8 MB, while assets pushed here
// (`gea devctl push <file> /nand/...`) are read back at run time through
// readCacheFile. It is the storage provider gea::platform::storage::
// ensureMounted() answers from, so every file op mounts it first.
//
// The chip is on SPI3 (SPI2 is the panel's) behind VCC_3V3, which the mount
// opens first. A blank chip has no filesystem: the first mount formats it.
#include "platform/file_cache.h"
#include "board.h"

#include <cinttypes>
#include <cstring>
#include <initializer_list>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat_nand.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace gea::platform::board {
void powerOnRails();
}

namespace gea::platform::storage {

namespace {

constexpr char kTag[] = "nand";
constexpr char kMountPoint[] = "/nand";
// The GD5F1GM7 reads at up to 133 MHz; 40 MHz single-line is what the
// driver's own examples run and is ample for loading assets once.
constexpr int kClockHz = 40 * 1000 * 1000;

StaticSemaphore_t g_lockStorage;
SemaphoreHandle_t g_lock = nullptr;
bool g_busReady = false;
spi_device_handle_t g_spi = nullptr;
spi_nand_flash_device_t *g_flash = nullptr;
bool g_mounted = false;

// Each step is kept once it succeeds, and a failed mount is retried on the
// next file op: the first attempt runs at boot, before the USB console is up,
// so its log line is lost -- a retry while a host is attached says why.
bool mountNand()
{
	gea::platform::board::powerOnRails();
	const auto &pins = gea::platform::board::nand;
	esp_err_t err = ESP_OK;
	if (!g_busReady) {
		// WP# and HOLD# are active low and stay plain GPIOs held high. Handed to
		// the SPI bus as IO2/IO3 they float to levels the chip reads as HOLD#
		// asserted, and READ ID answers zeros (seen on the board; Espressif's
		// own Mosaico bring-up does the same). Single-line needs neither.
		for (gpio_num_t pin : {pins.hold, pins.wp}) {
			gpio_set_level(pin, 1);
			gpio_config_t control = {};
			control.pin_bit_mask = 1ULL << pin;
			control.mode = GPIO_MODE_OUTPUT;
			control.pull_up_en = GPIO_PULLUP_ENABLE;
			gpio_config(&control);
			gpio_set_level(pin, 1);
		}
		spi_bus_config_t bus = {};
		bus.mosi_io_num = pins.mosi;
		bus.miso_io_num = pins.miso;
		bus.sclk_io_num = pins.clk;
		bus.quadwp_io_num = GPIO_NUM_NC;
		bus.quadhd_io_num = GPIO_NUM_NC;
		bus.max_transfer_sz = 4096 * 2;
		err = spi_bus_initialize(pins.spiHost, &bus, SPI_DMA_CH_AUTO);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(err));
			return false;
		}
		g_busReady = true;
	}
	if (g_spi == nullptr) {
		spi_device_interface_config_t device = {};
		device.clock_speed_hz = kClockHz;
		device.mode = 0;
		device.spics_io_num = pins.cs;
		device.queue_size = 10;
		device.flags = SPI_DEVICE_HALFDUPLEX;
		err = spi_bus_add_device(pins.spiHost, &device, &g_spi);
		if (err != ESP_OK) {
			g_spi = nullptr;
			ESP_LOGE(kTag, "SPI device add failed: %s", esp_err_to_name(err));
			return false;
		}
	}
	if (g_flash == nullptr) {
		spi_nand_flash_config_t config = {};
		config.device_handle = g_spi;
		config.io_mode = SPI_NAND_IO_MODE_SIO;
		config.flags = SPI_DEVICE_HALFDUPLEX;
		err = spi_nand_flash_init_device(&config, &g_flash);
		if (err != ESP_OK) {
			g_flash = nullptr;
			ESP_LOGE(kTag, "NAND init failed: %s", esp_err_to_name(err));
			return false;
		}
	}

	esp_vfs_fat_mount_config_t mount = {};
	mount.max_files = 4;
	mount.format_if_mount_failed = true;
	mount.allocation_unit_size = 16 * 1024;
	err = esp_vfs_fat_nand_mount(kMountPoint, g_flash, &mount);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "mount %s failed: %s", kMountPoint, esp_err_to_name(err));
		return false;
	}
	std::uint64_t total = 0;
	std::uint64_t free = 0;
	esp_vfs_fat_info(kMountPoint, &total, &free);
	ESP_LOGI(kTag, "mounted %s: %" PRIu64 " KB, %" PRIu64 " KB free", kMountPoint, total / 1024, free / 1024);
	return true;
}

// Serialized: the board start task mounts eagerly (GEADEV PUSH writes /nand
// without asking) while the app's first file read may ask at the same moment.
bool ensureNandMounted()
{
	xSemaphoreTake(g_lock, portMAX_DELAY);
	if (!g_mounted) g_mounted = mountNand();
	const bool mounted = g_mounted;
	xSemaphoreGive(g_lock);
	return mounted;
}

}  // namespace

}  // namespace gea::platform::storage

// GEADEV FORMAT /nand (device_control.cpp): unmount, erase the whole chip,
// and mount again, which formats the blank chip.
extern "C" int gea_board_storage_format(const char *mount)
{
	using namespace gea::platform::storage;
	if (std::strcmp(mount, kMountPoint) != 0) return -2;
	if (g_lock == nullptr) return -3;
	xSemaphoreTake(g_lock, portMAX_DELAY);
	if (!g_mounted && g_flash == nullptr) g_mounted = mountNand();
	if (g_flash == nullptr) {
		xSemaphoreGive(g_lock);
		return -4;
	}
	if (g_mounted) esp_vfs_fat_nand_unmount(kMountPoint, g_flash);
	g_mounted = false;
	const esp_err_t err = spi_nand_erase_chip(g_flash);
	if (err != ESP_OK) ESP_LOGE(kTag, "erase failed: %s", esp_err_to_name(err));
	g_mounted = mountNand();
	const bool mounted = g_mounted;
	xSemaphoreGive(g_lock);
	return err != ESP_OK ? -5 : (mounted ? 0 : -6);
}

namespace gea::platform::storage {

void installNandFileStorage()
{
	if (g_lock == nullptr) g_lock = xSemaphoreCreateMutexStatic(&g_lockStorage);
	setMountProvider(&ensureNandMounted);
}

}  // namespace gea::platform::storage
