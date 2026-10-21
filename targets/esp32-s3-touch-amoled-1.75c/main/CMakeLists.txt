set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-amoled-1.75c")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

# The 1.75's CO5300 glass: 466x466 visible, six columns in, on the 1.75's
# measured 80 MHz QSPI clock and its vendor init sequence.
set(GEA_BOARD_DISPLAY_WIDTH 466)
set(GEA_BOARD_DISPLAY_HEIGHT 466)
set(GEA_BOARD_QSPI_PCLK_HZ 80000000)
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_EXTRA_REQUIRES esp_codec_dev)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_BOARD_HAS_SPEAKER=1
    GEA_EMBEDDED_CO5300_WAVESHARE_175_INIT=1
    GEA_EMBEDDED_CO5300_PANEL_X_GAP=6
    GEA_EMBEDDED_CO5300_PANEL_Y_GAP=0
    # The flush pipeline otherwise takes every internal DMA byte left at boot
    # (20 rows x 2), which starves the audio task stack and WiFi. 12 rows
    # leaves ~25 KB internal free.
    GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX=12
    # Run the runtime on a PSRAM-stacked task and let app_main return, which
    # hands the 32 KB internal main stack back to the heap (BLE, WiFi and the
    # audio task all need internal RAM). Same split as the m5stack stopwatch.
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
    GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL=1
    # App native code reads the board name from this string.
    GEA_EMBEDDED_BOARD_ID="${GEA_EMBEDDED_CPP_BOARD}"
    # The QMI8658 reads mirrored along the screen's horizontal axis here.
    GEA_EMBEDDED_IMU_INVERT_X=1
)

set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    # CST9217 glue, shared with the 1.75.
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.75/main/touch.cpp"
    "${GEA_CHIPS}/displays/co5300/co5300.cpp"
    "${GEA_CHIPS}/power/axp2101/axp2101.cpp"
    "${GEA_CHIPS}/imu/qmi8658/qmi8658.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/co5300.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/power/axp2101.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/touch/cst9217.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/imu/qmi8658.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
)
include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")
