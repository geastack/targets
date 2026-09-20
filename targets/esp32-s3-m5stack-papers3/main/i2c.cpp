#include "i2c.h"

#include "board.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

namespace {

constexpr char kTag[] = "papers3_i2c";
i2c_master_bus_handle_t primaryBus = nullptr;

}  // namespace

gea::platform::i2c::Bus gea::platform::i2c::Bus::primary()
{
	if (primaryBus) return Bus(primaryBus);

	i2c_master_bus_config_t config = {};
	config.i2c_port = -1;
	config.sda_io_num = gea::platform::board::i2c.sda;
	config.scl_io_num = gea::platform::board::i2c.scl;
	config.clk_source = I2C_CLK_SRC_DEFAULT;
	config.glitch_ignore_cnt = 7;
	config.flags.enable_internal_pullup = true;

	const esp_err_t err = i2c_new_master_bus(&config, &primaryBus);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "I2C init failed: %s", esp_err_to_name(err));
		primaryBus = nullptr;
		return Bus(nullptr);
	}
	ESP_LOGI(kTag, "I2C ready SDA=%d SCL=%d", static_cast<int>(config.sda_io_num), static_cast<int>(config.scl_io_num));
	return Bus(primaryBus);
}
