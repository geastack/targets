#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "panel_power.h"

namespace gea::platform::board {

struct I2cBusConfig { gpio_num_t sda; gpio_num_t scl; };
struct Co5300DisplayConfig {
    spi_host_device_t spiHost;
    gpio_num_t cs;
    gpio_num_t pclk;
    gpio_num_t data0;
    gpio_num_t data1;
    gpio_num_t data2;
    gpio_num_t data3;
    gpio_num_t reset;
    gpio_num_t te;
};
struct Cst820TouchConfig { gpio_num_t interrupt; };

// https://docs.m5stack.com/en/core/StopWatch
// OLED and touch resets are M5IOE1 outputs, not ESP32 GPIOs.
inline constexpr I2cBusConfig i2c{GPIO_NUM_47, GPIO_NUM_48};
inline constexpr Co5300DisplayConfig display{
    SPI2_HOST, GPIO_NUM_39, GPIO_NUM_40,
    GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_46, GPIO_NUM_45,
    GPIO_NUM_NC, GPIO_NUM_38
};
inline constexpr Cst820TouchConfig touch{GPIO_NUM_13};

}  // namespace gea::platform::board
