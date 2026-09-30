// The 1.8-inch board routes its peripheral reset lines through an XCA9554.
// Follow Waveshare's original 04_GFX_FT3168_Image sequence: EXIO0..2 low for
// 20 ms, then high. Preserve the remaining pins and reuse the shared I2C bus.
// https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/main/examples/arduino/examples/04_GFX_FT3168_Image/04_GFX_FT3168_Image.ino
#include "i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>

namespace gea::platform::board {
void prepareDisplayPanel()
{
    const auto bus = gea::platform::i2c::Bus::primary();
    if (!bus.available()) {
        ESP_LOGE("panel_reset", "shared I2C bus unavailable");
        return;
    }
    i2c_device_config_t config{};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = 0x20;
    config.scl_speed_hz = 400000;
    i2c_master_dev_handle_t device = nullptr;
    esp_err_t err = i2c_master_bus_add_device(
        static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device);
    if (err != ESP_OK) {
        ESP_LOGE("panel_reset", "expander attach failed: %s", esp_err_to_name(err));
        return;
    }
    constexpr std::uint8_t pins = 0x07;
    const auto read = [&](std::uint8_t reg, std::uint8_t &value) {
        return i2c_master_transmit_receive(device, &reg, 1, &value, 1, 100);
    };
    const auto write = [&](std::uint8_t reg, std::uint8_t value) {
        const std::uint8_t bytes[]{reg, value};
        return i2c_master_transmit(device, bytes, sizeof(bytes), 100);
    };
    std::uint8_t output = 0, direction = 0;
    err = read(0x01, output);
    if (err == ESP_OK) err = read(0x03, direction);
    // Set the low latch before enabling outputs to avoid an unintended pulse.
    if (err == ESP_OK) err = write(0x01, output & ~pins);
    if (err == ESP_OK) err = write(0x03, direction & ~pins);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20));
        err = write(0x01, output | pins);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    i2c_master_bus_rm_device(device);
    if (err == ESP_OK) ESP_LOGI("panel_reset", "peripherals reset through XCA9554 EXIO0..2");
    else ESP_LOGE("panel_reset", "expander reset failed: %s", esp_err_to_name(err));
}
} // namespace gea::platform::board
