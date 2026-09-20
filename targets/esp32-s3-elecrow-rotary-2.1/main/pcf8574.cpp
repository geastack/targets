#include "pcf8574.h"

#include "board.h"
#include "i2c.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr const char *kTag = "pcf8574";
constexpr TickType_t kTimeout = pdMS_TO_TICKS(100);

i2c_master_dev_handle_t g_device = nullptr;
std::uint8_t g_shadow = 0xff;

esp_err_t writeShadow()
{
	if (!g_device) return ESP_ERR_INVALID_STATE;
	return i2c_master_transmit(g_device, &g_shadow, 1, kTimeout);
}

}  // namespace

namespace gea::platform::elecrow {

esp_err_t Pcf8574::init()
{
	if (g_device) return ESP_OK;

	auto bus = gea::platform::i2c::Bus::primary();
	if (!bus.available()) return ESP_ERR_INVALID_STATE;

	i2c_device_config_t config = {};
	config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	config.device_address = gea::platform::board::expander.address;
	config.scl_speed_hz = 100000;

	const esp_err_t err = i2c_master_bus_add_device(
		static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()),
		&config,
		&g_device);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to add PCF8574 at 0x%02x: %s",
		         gea::platform::board::expander.address,
		         esp_err_to_name(err));
		g_device = nullptr;
		return err;
	}

	g_shadow = 0xff;
	const esp_err_t writeErr = writeShadow();
	if (writeErr != ESP_OK) {
		ESP_LOGE(kTag, "failed to initialize PCF8574 outputs: %s", esp_err_to_name(writeErr));
		return writeErr;
	}

	ESP_LOGI(kTag, "PCF8574 ready at 0x%02x", gea::platform::board::expander.address);
	return ESP_OK;
}

esp_err_t Pcf8574::writePin(int bit, bool high)
{
	if (bit < 0 || bit > 7) return ESP_ERR_INVALID_ARG;
	esp_err_t err = init();
	if (err != ESP_OK) return err;

	const std::uint8_t mask = static_cast<std::uint8_t>(1u << bit);
	if (high) {
		g_shadow = static_cast<std::uint8_t>(g_shadow | mask);
	} else {
		g_shadow = static_cast<std::uint8_t>(g_shadow & ~mask);
	}
	return writeShadow();
}

esp_err_t Pcf8574::read(std::uint8_t *value)
{
	if (!value) return ESP_ERR_INVALID_ARG;
	esp_err_t err = init();
	if (err != ESP_OK) return err;
	return i2c_master_receive(g_device, value, 1, kTimeout);
}

}  // namespace gea::platform::elecrow
