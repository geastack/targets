# Waveshare ESP32-S3-Touch-AMOLED-1.8: SH8601 QSPI panel, FT3168 touch,
# AXP2101 PMIC, QMI8658 IMU, ES8311 codec. Board wiring only; the application
# component is the shared ESP32 one (targets/esp32/board/board_component.cmake).
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-amoled-1.8" CACHE STRING "Board profile passed to geatsc for C++ target selection")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

set(GEA_BOARD_PLATFORM_SOURCES
    "${GEA_EMBEDDED_ROOT}/targets/esp32/board/app_main.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/board/i2c.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/board/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/panel_reset.cpp"
    "${GEA_CHIPS}/displays/sh8601/sh8601.cpp"
    "${GEA_CHIPS}/power/axp2101/axp2101.cpp"
    "${GEA_CHIPS}/imu/qmi8658/qmi8658.cpp"
    "${GEA_CHIPS}/touch/ft3168/ft3168.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/sh8601.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/power/axp2101.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/touch/ft3168.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/imu/qmi8658.cpp"
)
set(GEA_BOARD_EXTRA_REQUIRES esp_codec_dev esp_new_jpeg)
set(GEA_BOARD_DISPLAY_WIDTH 368)
set(GEA_BOARD_DISPLAY_HEIGHT 448)
set(GEA_BOARD_QSPI_PCLK_HZ 80000000)
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
set(GEA_BOARD_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES 0)
set(GEA_BOARD_PARTIAL_PAD_X 8)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_STREAM_PACE_CHUNKS 4)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS GEA_BOARD_PREPARE_DISPLAY_PANEL=1)

include("${GEA_EMBEDDED_ROOT}/targets/esp32/board/board_component.cmake")
