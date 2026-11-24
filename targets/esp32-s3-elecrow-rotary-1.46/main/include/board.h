#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace gea::platform::board {
// Elecrow DHR55146D V1.0 schematic and factory sketch.
inline constexpr struct { gpio_num_t sda, scl; } i2c{GPIO_NUM_6, GPIO_NUM_7};
inline constexpr struct { gpio_num_t sda, scl; } auxiliaryI2c{GPIO_NUM_38, GPIO_NUM_39};
inline constexpr struct {
  spi_host_device_t spiHost;
  gpio_num_t mosi, sclk, cs, dc, reset, backlight;
} display{SPI2_HOST, GPIO_NUM_11, GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_3, GPIO_NUM_14, GPIO_NUM_46};
inline constexpr struct {
  gpio_num_t reset, interrupt;
  bool swapXy, mirrorX, mirrorY;
} touch{GPIO_NUM_13, GPIO_NUM_5, false, false, false};
inline constexpr struct {
  gpio_num_t phaseA, phaseB;
  int direction;
} rotary{GPIO_NUM_45, GPIO_NUM_42, 1};
inline constexpr gpio_num_t displayPower = GPIO_NUM_1;
inline constexpr gpio_num_t panelPower = GPIO_NUM_2;
inline constexpr gpio_num_t rgbPower = GPIO_NUM_17;
inline constexpr gpio_num_t rgbData = GPIO_NUM_48;
inline constexpr int rgbPixels = 8;
inline constexpr gpio_num_t bulb = GPIO_NUM_43;
inline constexpr gpio_num_t powerIndicator = GPIO_NUM_40;
}
