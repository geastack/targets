set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-m5stack-stopwatch")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

set(GEA_BOARD_DISPLAY_WIDTH 466)
set(GEA_BOARD_DISPLAY_HEIGHT 466)
set(GEA_BOARD_QSPI_PCLK_HZ 80000000)
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_EXTRA_REQUIRES bootloader_support esp_psram esp_codec_dev)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_BOARD_PREPARE_DISPLAY_PANEL=1
    GEA_BOARD_SPEAKER_POWER=1
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
    GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL=1
    GEA_EMBEDDED_CO5300_STOPWATCH_INIT=1
    GEA_EMBEDDED_CO5300_PANEL_X_GAP=6
    GEA_EMBEDDED_CO5300_PANEL_Y_GAP=0
)

# Keep the application pipeline shared; only the native board wiring differs.
# Board bindings supply sensors, power, audio and buttons to every Gea application.
set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/buttons.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/panel_reset.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/power.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/imu/bmi270.cpp"
    "${GEA_CHIPS}/displays/co5300/co5300.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/co5300.cpp"
)
include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")

# Generated app archives can reference the hardware RNG after IDF archives
# were visited; keep the same extraction rule as other S3 targets.
if(NOT CMAKE_SCRIPT_MODE_FILE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-u bootloader_random_enable" "-u bootloader_random_disable")
endif()
