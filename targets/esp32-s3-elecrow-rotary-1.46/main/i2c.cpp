#include "auxiliary_i2c.h"
#include "board.h"
#include "esp_log.h"
#include "i2c.h"

namespace {
i2c_master_bus_handle_t openBus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl)
{
  i2c_master_bus_config_t config{};
  config.i2c_port = port;
  config.sda_io_num = sda;
  config.scl_io_num = scl;
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus = nullptr;
  const auto result = i2c_new_master_bus(&config, &bus);
  if (result != ESP_OK) {
    ESP_LOGE("crowpanel146", "I2C setup failed on GPIO%d/%d: %s", sda, scl, esp_err_to_name(result));
  }
  return bus;
}
}

gea::platform::i2c::Bus gea::platform::i2c::Bus::primary()
{
  static const auto handle = openBus(I2C_NUM_0, board::i2c.sda, board::i2c.scl);
  return Bus(handle);
}

i2c_master_bus_handle_t gea::platform::elecrow::auxiliaryI2c()
{
  static const auto handle = openBus(I2C_NUM_1, board::auxiliaryI2c.sda, board::auxiliaryI2c.scl);
  return handle;
}
