set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s31-espressif-mosaico")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")

# The shared base resolves the npm packages (and with them GEA_CHIPS) only
# after this file has run, so resolve them here too: the source list below
# names chip drivers by that path and would otherwise expand to `/displays/...`.
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

# 480x480 square CO5300 AMOLED, no column gap.
set(GEA_BOARD_DISPLAY_WIDTH 480)
set(GEA_BOARD_DISPLAY_HEIGHT 480)

# The board's own BSP drives the panel at 40 MHz. The panel latches at 80 MHz,
# as on the 1.75 and 2.06 CO5300 boards: bouncing-balls went from 36.5 to
# 61.6 fps, the full-width flush from 23 ms to 11.8 ms per frame.
set(GEA_BOARD_QSPI_PCLK_HZ 80000000)
set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)

# No AXP2101, no ESP-Mosaico BSP: the VCC_3V3 rail and the USB console are
# board_power.cpp's, and it gives display init its prepareDisplayPanel().
set(GEA_BOARD_EXTRA_REQUIRES
    esp_codec_dev
    esp_driver_ledc
    espressif__esp_tinyusb
)

set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS
    GEA_BOARD_PREPARE_DISPLAY_PANEL=1
    GEA_BOARD_USB_CONSOLE=1
    # `gea flash` installs apps through the running app (usbAppUpdate).
    GEA_DEVICE_CONTROL_USB_OTA=1
    # Framebuffer-free retained UI: each dirty band is rendered straight into the
    # internal-RAM DMA staging chunk and streamed while the next band renders,
    # instead of writing the PSRAM framebuffer and reading it back to flush.
    GEA_EMBEDDED_DISPLAY_BANDED_UI=1
    GEA_EMBEDDED_CO5300_MOSAICO_INIT=1
    GEA_EMBEDDED_CO5300_PANEL_X_GAP=0
    GEA_EMBEDDED_CO5300_PANEL_Y_GAP=0
    GEA_BOARD_BMI270_I2C_ADDRESS=0x69
    GEA_BOARD_ES8311_I2C_ADDRESS=0x19
    # The side "AI" button; GPIO0 is the expansion I2C bus on this board.
    GEA_BOARD_LAUNCHER_BUTTON_GPIO=7
    GEA_BOARD_TOUCH_CST9220=1
)

set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/board_power.cpp"
    "${GEA_CHIPS}/displays/co5300/co5300.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/display.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/displays/co5300.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/touch/cst9217.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/audio/es8311.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/imu/bmi270.cpp"
    # The BQ27220 fuel gauge has no binding yet; report "no battery reading"
    # rather than a number nobody measured.
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/power_absent.cpp"
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")

# IDF 6.2 brings GCC 16 + picolibc: picolibc declares fflush() nonnull, so the
# runtime's fflush(nullptr) trips -Werror=nonnull, and GCC 16 flags generated
# std::move()s as redundant. Warnings, not errors, until the compiler runtime
# and emitter are fixed for this toolchain.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=nonnull -Wno-error=redundant-move)
    # board_power.cpp keeps a crash's registers and log across the reset (no UART0 here)
    # and makes the USB console wait for the host instead of dropping output.
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler"
                          "-Wl,--wrap=tinyusb_cdcacm_write_queue_char"
                          "-Wl,--wrap=uart_hal_write_txfifo")
endif()
