#pragma once
#include "driver/gpio.h"
#include "driver/spi_master.h"

// Waveshare factory BSP, ESP-IDF-5.5.1/01_factory/components/esp_bsp.
namespace gea::platform::board {
struct I2cBusConfig { gpio_num_t sda, scl; };
struct St7789DisplayConfig { spi_host_device_t spiHost; gpio_num_t mosi, sclk, cs, dc, reset, backlight; };
struct TouchConfig { gpio_num_t reset, interrupt; bool swapXy = false, mirrorX = false, mirrorY = false; };
struct ButtonConfig { gpio_num_t key1, key2; };
struct Es8311AudioConfig {
  int i2sPort;
  gpio_num_t mclk, bclk, ws, dout, din, powerAmplifier;
  int es7210Address;
};
struct SdMmcStorageConfig { gpio_num_t clk, cmd, d0; };
struct PowerConfig { gpio_num_t enable, charging, adc; };
class WaveshareTouchLcd154 {
public:
  static constexpr I2cBusConfig i2c{GPIO_NUM_42, GPIO_NUM_41};
  static constexpr St7789DisplayConfig display{SPI2_HOST, GPIO_NUM_39, GPIO_NUM_38,
      GPIO_NUM_21, GPIO_NUM_45, GPIO_NUM_40, GPIO_NUM_46};
  static constexpr TouchConfig touch{GPIO_NUM_47, GPIO_NUM_48};
  static constexpr ButtonConfig buttons{GPIO_NUM_0, GPIO_NUM_4};
  static constexpr Es8311AudioConfig audio{0, GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10,
      GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_7, 0x40};
  static constexpr SdMmcStorageConfig storage{GPIO_NUM_16, GPIO_NUM_15, GPIO_NUM_17};
  static constexpr PowerConfig power{GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_1};
};
using Board = WaveshareTouchLcd154;
inline constexpr auto i2c = Board::i2c;
inline constexpr auto display = Board::display;
inline constexpr auto touch = Board::touch;
inline constexpr auto buttons = Board::buttons;
inline constexpr auto audio = Board::audio;
inline constexpr auto power = Board::power;
}
