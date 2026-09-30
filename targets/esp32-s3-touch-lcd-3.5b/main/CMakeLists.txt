set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-lcd-3.5b")
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS pixel scale")
set(GEA_BOARD_DISPLAY_WIDTH 320)
set(GEA_BOARD_DISPLAY_HEIGHT 480)
set(GEA_BOARD_QSPI_PCLK_HZ 40000000)
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_STREAM_PACE_CHUNKS 0)
set(GEA_BOARD_EXTRA_REQUIRES esp_driver_ledc esp_codec_dev)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_EMBEDDED_AXS15231B_PANEL=1
    GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO=6
    GEA_BOARD_AXS15231B_INIT_HEADER="axs15231b_panel_init.h"
    GEA_BOARD_HAS_SDMMC=1
    GEA_BOARD_HAS_SPEAKER=1
    GEA_BOARD_SPEAKER_POWER=1
    GEA_EMBEDDED_DISPLAY_FULL_FRAME_TRANSFERS=1
)
set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/platform_stubs.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/speaker_power.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
    "${GEA_CHIPS}/power/axp2101/axp2101.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/power/axp2101.cpp"
    "${GEA_CHIPS}/displays/sh8601/sh8601.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/sh8601.cpp"
)
include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")
