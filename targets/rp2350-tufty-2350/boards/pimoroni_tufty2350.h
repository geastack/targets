/*
 * Pico SDK board definition for Pimoroni Tufty 2350.
 *
 * Pin values mirror Pimoroni's Tufty 2350 MicroPython board package. Keep this
 * file to preprocessor definitions so the Pico SDK can include it from C and
 * ASM contexts.
 */

#ifndef _BOARDS_PIMORONI_TUFTY2350_H
#define _BOARDS_PIMORONI_TUFTY2350_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)
pico_board_cmake_set(PICO_NUM_GPIOS, 48)
pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)

#define PIMORONI_TUFTY2350
#define PICO_RP2350A 0

#define TUFTY_RTC_I2C_SDA 4
#define TUFTY_RTC_I2C_SCL 5
#define TUFTY_PSRAM_CS 8

#define TUFTY_WL_ON 23u
#define TUFTY_WL_D 24u
#define TUFTY_WL_CS 25u
#define TUFTY_WL_CLK 29u

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN TUFTY_RTC_I2C_SDA
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN TUFTY_RTC_I2C_SCL
#endif

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#ifndef RP2350_XIP_CSI_PIN
#define RP2350_XIP_CSI_PIN TUFTY_PSRAM_CS
#endif

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#ifndef CYW43_WL_GPIO_COUNT
#define CYW43_WL_GPIO_COUNT 3
#endif

#ifndef CYW43_PIN_WL_DYNAMIC
#define CYW43_PIN_WL_DYNAMIC 0
#endif

#ifndef CYW43_DEFAULT_PIN_WL_REG_ON
#define CYW43_DEFAULT_PIN_WL_REG_ON TUFTY_WL_ON
#endif
#ifndef CYW43_DEFAULT_PIN_WL_DATA_OUT
#define CYW43_DEFAULT_PIN_WL_DATA_OUT TUFTY_WL_D
#endif
#ifndef CYW43_DEFAULT_PIN_WL_DATA_IN
#define CYW43_DEFAULT_PIN_WL_DATA_IN TUFTY_WL_D
#endif
#ifndef CYW43_DEFAULT_PIN_WL_HOST_WAKE
#define CYW43_DEFAULT_PIN_WL_HOST_WAKE TUFTY_WL_D
#endif
#ifndef CYW43_DEFAULT_PIN_WL_CLOCK
#define CYW43_DEFAULT_PIN_WL_CLOCK TUFTY_WL_CLK
#endif
#ifndef CYW43_DEFAULT_PIN_WL_CS
#define CYW43_DEFAULT_PIN_WL_CS TUFTY_WL_CS
#endif

#endif
