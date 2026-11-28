# M5Stack StickS3 hardware settings for the shared ESP32 SPI panel component.
set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-s3-m5stack-sticks3")
set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 135)
set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 240)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH 135)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT 240)
# 1.14" 135x240 IPS at ~241 PPI. DPR scales explicit CSS px to physical pixels:
# 1.0 maps 1 CSS px to 1 tiny device px (illegibly small); 2.0 gives a 67x120 CSS
# canvas (legible but too cramped for label+value info rows); 1.5 gives a 90x160
# canvas that fits the info screens while staying legible — matching the small
# high-density M5 panels (amoled-1.75 / elecrow-rotary also run 1.5).
set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.5")
# Color IPS panel — do not inherit the epaper family's monochrome media flag.
set(GEA_EMBEDDED_TARGET_DISPLAY_MONOCHROME 0)
# Panel-endian framebuffer (like the amoled targets): the ST7789 latches
# RGB565 MSB-first, so storing pixels pre-swapped turns the flush byteswap
# into a straight memcpy.
set(GEA_EMBEDDED_TARGET_PIXEL_PANEL_ENDIAN 1)
# Parallel-replay thresholds stay at framework defaults: measured on this
# panel (2026-07-12), a 2-core band split LOWERED bouncing-balls-jsx from
# 54.5 to 52.3 fps — its replay cost is the per-node command walk (duplicated
# per band), not raster pixels. The default 65536-pixel gate exceeds the whole
# 135x240 panel, so the split stays off here.
# bootloader_support: the geatsc prelude seeds Math.random from the hardware
# RNG (bootloader_random.h). esp_driver_ledc: PWM backlight (display.cpp).
# spi_flash/esp_psram: put esp_flash.h/esp_psram.h on the include path so the
# diagnostics deviceInfo facade (app native/device_info.cpp) can report the 8MB
# flash + 8MB PSRAM instead of degrading to 0 (mirrors the m5paper target).
# esp_driver_i2s: board.h names the speaker and microphone pins with driver/i2s_types.h.
set(GEA_EMBEDDED_TARGET_EXTRA_REQUIRES bootloader_support esp_driver_ledc spi_flash esp_psram esp_driver_i2c esp_driver_i2s)

# IMU: BMI270 (6-axis accel+gyro) on the shared primary I2C bus at 0x68, driving
# gea::platform::sensors::Accelerometer (tilt/level tool, air-mouse, etc.).
# services/buttons.cpp: the shared board-button service watches KEY1/KEY2, which
# targets.json declares (gea_board_buttons below passes the generated table).
set(GEA_EMBEDDED_TARGET_EXTRA_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/imu/bmi270.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/services/buttons.cpp")

# ST7789V2 window on the 240x320 GRAM, 80 MHz SPI mode 0, glass powered from the
# M5PM1 L3B rail.
set(GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS
    GEA_ST7789_LOG_TAG="sticks3_display"
    GEA_ST7789_PCLK_HZ=80000000 GEA_ST7789_SPI_MODE=0
    GEA_ST7789_GAP_X=52 GEA_ST7789_GAP_Y=40
    GEA_ST7789_MIRROR_GAP_X=53 GEA_ST7789_MIRROR_GAP_Y=40
    GEA_ST7789_M5PM1_POWER=1)
set(GEA_EMBEDDED_TARGET_DISPLAY_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/displays/st7789_spi_lcd.cpp")
set(GEA_EMBEDDED_TARGET_I2C_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/board/i2c.cpp")
set(GEA_EMBEDDED_TARGET_LAUNCHER_BUTTON_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/board/launcher_button.cpp")
set(GEA_EMBEDDED_TARGET_POWER_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/power_absent.cpp")

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32/gea_spi_panel_component.cmake")

if(NOT CMAKE_SCRIPT_MODE_FILE)
    include("${CMAKE_CURRENT_LIST_DIR}/../../esp32/gea_board_buttons.cmake")
    gea_board_buttons(${COMPONENT_LIB})
endif()

# Generated app archives are linked after ESP-IDF's component archives. Force
# the hardware RNG implementation out of bootloader_support before that final
# archive introduces its references (mirrors the m5paper target).
if(NOT CMAKE_SCRIPT_MODE_FILE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-u bootloader_random_enable"
        "-u bootloader_random_disable"
    )
endif()
