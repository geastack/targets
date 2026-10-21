set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-lcd-3.49")
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.0" CACHE STRING "Logical CSS pixel scale")

# 172x640 glass, landscape by default. Like the T-Display-S3 Long's 180x640
# AXS15231B, the controller has no usable hardware XY swap, so the runtime
# rotates in software and every transfer spans the native 172-pixel scanline.
set(GEA_BOARD_DISPLAY_WIDTH 640)
set(GEA_BOARD_DISPLAY_HEIGHT 172)
set(GEA_BOARD_NATIVE_DISPLAY_WIDTH 172)
set(GEA_BOARD_NATIVE_DISPLAY_HEIGHT 640)
# The vendor drives this panel at 40 MHz; so does the 3.5B's AXS15231B.
set(GEA_BOARD_QSPI_PCLK_HZ 40000000)
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
# Paired with the 30 fps interval below: drop catch-up frames so the cap holds.
set(GEA_BOARD_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES 1)
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_STREAM_PACE_CHUNKS 0)
set(GEA_BOARD_EXTRA_REQUIRES esp_driver_ledc esp_codec_dev esp_adc)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_EMBEDDED_AXS15231B_PANEL=1
    # V2: backlight PWM on GPIO42, active low (the vendor's full brightness
    # is duty 0). Its supply enable and the panel reset are expander pins,
    # driven by panel_reset.cpp.
    GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO=42
    GEA_EMBEDDED_AXS15231B_BACKLIGHT_ACTIVE_LOW=1
    GEA_BOARD_PREPARE_DISPLAY_PANEL=1
    GEA_BOARD_AXS15231B_INIT_HEADER="axs15231b_panel_init.h"
    GEA_BOARD_HAS_SDMMC=1
    GEA_BOARD_HAS_SPEAKER=1
    GEA_BOARD_HAS_EXPANDER=1
    GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY=1
    GEA_EMBEDDED_DISPLAY_ROTATE_LANDSCAPE=1
    GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION=1
    GEA_EMBEDDED_DISPLAY_DEFAULT_PORTRAIT_PRIMARY=0
    GEA_EMBEDDED_FLUSH_UNITE_ALL=0
    # Like the 3.5B's AXS15231B, this glass cannot reliably restart a write at
    # a nonzero native row: a partial window lands at the wrong place and
    # leaves stale pixels behind. Rasterize damage only, but send the frame
    # from its origin.
    GEA_EMBEDDED_DISPLAY_FULL_FRAME_TRANSFERS=1
    # A full 640x172 frame takes 20-25 ms over QSPI, so free-running lands
    # between 40 and 50 fps and the uneven cadence reads as jitter. Pace a
    # steady 30 fps instead, and drop catch-up frames: a tick that lands mid-frame
    # would otherwise run the next frame back to back and undo the cap.
    GEA_EMBEDDED_DEFAULT_FRAME_INTERVAL_US=33333
    GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH=2
    # The flush pipeline otherwise takes every internal DMA byte left at boot
    # (64 rows x 2), which starves the audio task stack and WiFi.
    GEA_EMBEDDED_DISPLAY_FLUSH_CHUNK_MAX=32
    # Run the runtime on a PSRAM-stacked task and let app_main return, which
    # hands the 32 KB internal main stack back to the heap (BLE, WiFi and the
    # audio task all need internal RAM). Same split as the m5stack stopwatch.
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
    GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL=1
    # App native code reads the board name from this string.
    GEA_EMBEDDED_BOARD_ID="${GEA_EMBEDDED_CPP_BOARD}"
)

# Shared ESP32-S3 bootstrap seams; the board's own hardware is selected here
# and in include/board.h.
set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/panel_reset.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/power.cpp"
    "${GEA_CHIPS}/imu/qmi8658/qmi8658.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/imu/qmi8658.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/expanders/tca9554.cpp"
    "${GEA_CHIPS}/displays/sh8601/sh8601.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/sh8601.cpp"
)
include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")
