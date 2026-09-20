#include "i2c.h"

#include "board.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

namespace {

constexpr gpio_num_t kSdaPin = gea::platform::board::i2c.sda;
constexpr gpio_num_t kSclPin = gea::platform::board::i2c.scl;
constexpr const char *TAG = "i2c";

i2c_master_bus_handle_t primary_bus = nullptr;

}  // namespace

gea::platform::i2c::Bus gea::platform::i2c::Bus::primary() {
	if (primary_bus) return Bus(primary_bus);

	i2c_master_bus_config_t bus_cfg = {};
	bus_cfg.i2c_port = -1;
	bus_cfg.sda_io_num = kSdaPin;
	bus_cfg.scl_io_num = kSclPin;
	bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
	bus_cfg.flags.enable_internal_pullup = true;

	const esp_err_t err = i2c_new_master_bus(&bus_cfg, &primary_bus);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
		primary_bus = nullptr;
		return Bus(nullptr);
	}

	ESP_LOGI(TAG, "I2C bus ready (SDA=%d, SCL=%d)", static_cast<int>(kSdaPin), static_cast<int>(kSclPin));
	return Bus(primary_bus);
}
