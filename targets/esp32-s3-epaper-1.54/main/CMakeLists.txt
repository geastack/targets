set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-s3-epaper-1.54")
set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 200)
set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 200)
# DPR 1.0 (logical px == physical 200x200): pixel fonts (Cozette) only render
# crisp, uniform stems when baked at an integer multiple of their native size,
# which requires baked-size == CSS-size. A non-integer DPR (1.5) would force
# fractional baked sizes and reintroduce uneven 1px/2px stems on this 1-bit panel.
set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.0")
set(GEA_EMBEDDED_TARGET_DISPLAY_MONOCHROME 1)
set(GEA_EMBEDDED_TARGET_I2C_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/board/i2c.cpp")
set(GEA_EMBEDDED_TARGET_LAUNCHER_BUTTON_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/board/launcher_button.cpp")
set(GEA_EMBEDDED_TARGET_POWER_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/power_absent.cpp")
set(GEA_EMBEDDED_TARGET_SDCARD_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/sdcard_sdmmc_1bit.cpp")
include("${CMAKE_CURRENT_LIST_DIR}/../../esp32/gea_spi_panel_component.cmake")
