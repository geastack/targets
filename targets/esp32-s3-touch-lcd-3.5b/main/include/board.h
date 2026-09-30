#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"

// Waveshare ESP32-S3-Touch-LCD-3.5B, including the enclosed 3.5B-C.
// Pin assignments: vendor ESP-IDF/01_factory/components/esp_bsp.
namespace gea::platform::board {
struct I2cBusConfig { gpio_num_t sda, scl; };
struct QspiDisplayConfig {
  spi_host_device_t spiHost;
  gpio_num_t cs, pclk, data0, data1, data2, data3, reset, te;
};
struct Es8311AudioConfig { int i2sPort; gpio_num_t mclk, bclk, ws, dout, din, powerAmplifier; };
bool setSpeakerPower(bool enabled);
struct TouchConfig { gpio_num_t reset, interrupt; };
struct SdMmcConfig { gpio_num_t clk, cmd, data0; };
struct LauncherButtonConfig { gpio_num_t pin; int activeLevel; };
inline constexpr I2cBusConfig i2c{GPIO_NUM_8, GPIO_NUM_7};
inline constexpr QspiDisplayConfig display{
  SPI2_HOST, GPIO_NUM_12, GPIO_NUM_5, GPIO_NUM_1, GPIO_NUM_2,
  GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_NC, GPIO_NUM_NC};
inline constexpr Es8311AudioConfig audio{0, GPIO_NUM_44, GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_14, GPIO_NUM_NC};
inline constexpr TouchConfig touch{GPIO_NUM_NC, GPIO_NUM_NC};
inline constexpr SdMmcConfig storage{GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9};
inline constexpr LauncherButtonConfig launcherButton{GPIO_NUM_0, 0};
}
