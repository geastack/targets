// Pulse the panel's reset line before the display driver talks to it.
//
// On this board (hardware V2) OLED_RESET is expander pin EXIO0 of a TCA9554 on
// the shared I2C bus, not a GPIO, so board::display.reset is GPIO_NUM_NC and
// the panel driver has nothing to toggle. The RM69080 keeps whatever state it
// was left in across a warm reset of the ESP32, and a QSPI write reports
// success whether or not the panel is listening -- so without this the display
// reports a clean init and stays dark. The vendor's own V2 example pulses the
// same line at the same point.
//
// Display init runs before the board's shared I2C bus is brought up, so this
// opens its own short-lived master bus and closes it again; the later
// bus creation in i2c.cpp then proceeds normally.
#include "board.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_io_expander_tca9554.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
  constexpr char kTag[] = "panel_reset";
  constexpr uint32_t kPanelResetPin = IO_EXPANDER_PIN_NUM_0;
  constexpr uint32_t kTouchResetPin = IO_EXPANDER_PIN_NUM_1;

  void pulseReset(esp_io_expander_handle_t expander, uint32_t pin)
  {
    esp_io_expander_set_dir(expander, pin, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(expander, pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(expander, pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(expander, pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
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

    esp_io_expander_handle_t expander = nullptr;
    err = esp_io_expander_new_i2c_tca9554(bus, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &expander);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "TCA9554 not found: %s", esp_err_to_name(err));
      i2c_del_master_bus(bus);
      return;
    }

    // Both resets are driven: the touch controller's reset is the neighbouring
    // expander pin and is equally unreachable as a GPIO.
    pulseReset(expander, kPanelResetPin);
    pulseReset(expander, kTouchResetPin);

    ESP_LOGI(kTag, "panel and touch reset through TCA9554 (EXIO0, EXIO1)");

    // The shared bus is created later by i2c.cpp; release this one so that
    // creation does not fail on an already-installed port.
    esp_io_expander_del(expander);
    i2c_del_master_bus(bus);
  }
}  // namespace gea::platform::board
