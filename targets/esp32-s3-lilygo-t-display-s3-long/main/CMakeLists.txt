set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-lilygo-t-display-s3-long")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.0" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_BOARD_DISPLAY_WIDTH 640)
set(GEA_BOARD_DISPLAY_HEIGHT 180)
set(GEA_BOARD_NATIVE_DISPLAY_WIDTH 180)
set(GEA_BOARD_NATIVE_DISPLAY_HEIGHT 640)
if(NOT DEFINED GEA_BOARD_QSPI_PCLK_HZ)
    # Keep color DMA serialized for panel stability and use a conservative
    # 20 MHz QSPI data phase. Full-native-scanline landscape transfers avoid the
    # partial-window corruption seen with narrower and higher-clocked writes.
    set(GEA_BOARD_QSPI_PCLK_HZ 20000000)
endif()
if(NOT DEFINED GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH)
    set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 1)
endif()
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)
set(GEA_BOARD_STREAM_PACE_CHUNKS 0)
set(GEA_BOARD_EXTRA_REQUIRES
    esp_driver_ledc
    espressif__esp_tinyusb
    espressif__tinyusb
    spi_flash
)
set(GEA_BOARD_ENABLE_BLE_HID 0)
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_EMBEDDED_AXS15231B_PANEL=1
    GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO=1
    GEA_EMBEDDED_BLE_DISABLED=1
    GEA_EMBEDDED_WIFI_DISABLED=1
    # The controller has no usable hardware XY-swap register. Runtime
    # orientation switching rebinds this one physical 180x640 framebuffer as
    # either native portrait rows or a native-order rotated landscape Canvas.
    GEA_EMBEDDED_DISPLAY_SOFTWARE_LANDSCAPE_PRIMARY=1
    GEA_EMBEDDED_DISPLAY_ROTATE_LANDSCAPE=1
    GEA_EMBEDDED_DISPLAY_RUNTIME_SOFTWARE_ORIENTATION=1
    GEA_EMBEDDED_DISPLAY_DEFAULT_PORTRAIT_PRIMARY=0
    # Keep separate native row bands. Each band still spans the full 180-pixel
    # scanline, the transfer shape measured stable on this panel.
    GEA_EMBEDDED_FLUSH_UNITE_ALL=0
    # At 120 Hz a frame that exceeds 8.3 ms intentionally enters catch-up mode.
    # Portrait can sustain that mode indefinitely, so yield once per burst to
    # service IDLE0 and its task-watchdog subscription without lowering the
    # requested frame rate.
    GEA_EMBEDDED_FRAME_SCHEDULER_MAX_CATCHUP_FRAMES_BEFORE_YIELD=60
    # Keep one color transaction prepared behind the active transfer for the
    # 20 MHz stability/performance configuration;
    # the shared depth-32 queue previously produced horizontal corruption.
    GEA_EMBEDDED_DISPLAY_SPI_TRANSACTION_QUEUE_DEPTH=2
)

# Reuse the shared ESP32-S3 bootstrap seams; all board-specific hardware is
# selected here and in include/board.h.
set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-elecrow-rotary-2.1/main/platform_stubs.cpp"
    "${GEA_CHIPS}/displays/sh8601/sh8601.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/display.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/displays/sh8601.cpp"
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")
