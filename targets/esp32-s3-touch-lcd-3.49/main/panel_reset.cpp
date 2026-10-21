// Bring the panel out of reset and enable its backlight before the display
// driver talks to it.
//
// On the V2 board the AXS15231B's reset is the TCA9554's EXIO5 and the
// backlight supply is gated by EXIO1 (BL_EN); neither is a GPIO, so
// board::display.reset is GPIO_NUM_NC and the panel driver has nothing to
// toggle. The vendor's V2 examples (example_lcd_exio_init / example_lcd_reset)
// drive the same two pins at the same point.
//
// Display init runs before the board's shared I2C bus is brought up, so this
// opens its own short-lived master bus and closes it again; the later bus
// creation in i2c.cpp then proceeds normally.
#include "board.h"

#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
  constexpr char kTag[] = "panel_reset";
  constexpr std::uint8_t kOutputRegister = 0x01;
  constexpr std::uint8_t kConfigRegister = 0x03;
  constexpr std::uint8_t kBacklightEnable = 1u << 1;
  constexpr std::uint8_t kPanelReset = 1u << 5;

  bool writeRegister(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t value)
  {
    const std::uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(device, bytes, sizeof(bytes), 100) == ESP_OK;
  }
}  // namespace

namespace gea::platform::board
{
  void prepareDisplayPanel()
  {
    i2c_master_bus_config_t busConfig = {};
    busConfig.i2c_port = I2C_NUM_0;
    busConfig.sda_io_num = i2c.sda;
    busConfig.scl_io_num = i2c.scl;
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t err = i2c_new_master_bus(&busConfig, &bus);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "I2C bus for the panel reset failed: %s", esp_err_to_name(err));
      return;
    }

    i2c_device_config_t deviceConfig = {};
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    deviceConfig.device_address = expander.address;
    deviceConfig.scl_speed_hz = 400000;
    i2c_master_dev_handle_t device = nullptr;
    err = i2c_master_bus_add_device(bus, &deviceConfig, &device);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "TCA9554 attach failed: %s", esp_err_to_name(err));
      i2c_del_master_bus(bus);
      return;
    }

    // Outputs first, so pins switching to output come up at the levels the
    // board expects; then the reset pulse with the backlight enabled.
    const std::uint8_t idle = static_cast<std::uint8_t>(expander.initialOutputs);
    const bool ok = writeRegister(device, kOutputRegister, idle) &&
                    writeRegister(device, kConfigRegister, static_cast<std::uint8_t>(expander.initialDirections));
    if (ok)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      writeRegister(device, kOutputRegister, static_cast<std::uint8_t>(idle & ~kPanelReset));
      vTaskDelay(pdMS_TO_TICKS(20));
      writeRegister(device, kOutputRegister, static_cast<std::uint8_t>(idle | kPanelReset | kBacklightEnable));
      vTaskDelay(pdMS_TO_TICKS(120));
      ESP_LOGI(kTag, "panel reset and backlight enable through TCA9554 (EXIO5, EXIO1)");
    }
    else
    {
      ESP_LOGE(kTag, "TCA9554 not responding at 0x%02x", expander.address);
    }

    i2c_master_bus_rm_device(device);
    i2c_del_master_bus(bus);
  }
}  // namespace gea::platform::board
