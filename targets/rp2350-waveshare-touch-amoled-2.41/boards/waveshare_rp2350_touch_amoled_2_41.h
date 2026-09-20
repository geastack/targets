/*
 * Pico SDK board definition for Waveshare RP2350-Touch-AMOLED-2.41.
 *
 * Derived from Waveshare's board package. Keep this file limited to
 * preprocessor definitions so the Pico SDK can include it from C and ASM
 * contexts.
 */

#ifndef _BOARDS_WAVESHARE_RP2350_TOUCH_AMOLED_2_41_H
#define _BOARDS_WAVESHARE_RP2350_TOUCH_AMOLED_2_41_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)

#define WAVESHARE_RP2350_TOUCH_AMOLED_2_41
#define PICO_RP2350A 0

#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 0
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 1
#endif

#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 6
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 7
#endif

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef RP2350_XIP_CSI_PIN
#define RP2350_XIP_CSI_PIN 47
#endif

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif
