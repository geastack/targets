set(GEA_BOARD_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_BOARD_TARGET_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-amoled-2.41")

# The shared base resolves the npm packages (and with them GEA_CHIPS) only
# after this file has run, so resolve them here too: the source list below
# names chip drivers by that path and would otherwise expand to `/displays/...`.
set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

# 450x600 portrait AMOLED. The vendor's own V2 example spells it the same way
# (LCD_WIDTH 450 / LCD_HEIGHT 600) even though the product page quotes the
# panel as 600x450; the driver is fed the portrait orientation it scans out.
# 80 MHz, the base default. During bring-up nothing reached the glass at this
# rate and the clock was dropped to 40 MHz to get a first picture; the cause
# was the panel reset, not the clock, and once prepareDisplayPanel pulsed
# EXIO0 the panel latched at 80 MHz as well. Worth 22ms -> 15ms of flush on a
# frame that dirties ~100k pixels.
set(GEA_BOARD_QSPI_PCLK_HZ 80000000)

# The base pads partial flush windows 8 px sideways. This panel repaints at
# the padded origin without compensating for it, so a tap moved the buttons
# and left a stale sliver. No padding, no displacement.
# Every band re-issues its own window instead of continuing the previous write.
# The RAMWRC continue path trusts the panel column counter, which wraps at this
# panel gapped width rather than the logical 450: after the first repaint the
# content walked sideways and left a stale sliver at the edge.
set(GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 0)

set(GEA_BOARD_PARTIAL_PAD_X 0)
set(GEA_BOARD_PARTIAL_PAD_Y 0)

set(GEA_BOARD_DISPLAY_WIDTH 450)
set(GEA_BOARD_DISPLAY_HEIGHT 600)

# The RM69080 speaks the SH8601 command set, so the shared SH8601 QSPI binding
# drives it -- which is what the vendor's ESP-IDF example does as well.
#
# Touch is an FT6336, on its own ESP32 binding.
# The shared base defaults this to esp_codec_dev; this board has no audio
# codec, and requiring one fails the build outright.
set(GEA_BOARD_EXTRA_REQUIRES espressif__esp_io_expander espressif__esp_io_expander_tca9554)

# panel_reset.cpp supplies board::prepareDisplayPanel(); display init only
# calls it for a board that says it has one.
# No GEA_EMBEDDED_FLUSH_UNITE_ALL here. The shift and the stale sliver that
# uniting every rect was added to hide came from the RAMWRC continue-write
# above, and GEA_BOARD_FRAMEBUFFER_STREAM_FLUSH 0 is what actually fixed them;
# present::clampAndAlign already snaps x0 down to even and x1 up to odd, which
# is exactly the two-pixel column rule this panel wants. Uniting on top of that
# only cost pixels: every frame flushed 98% of the panel, 35ms of a 60ms frame,
# and the bouncing-balls example ran at 16fps instead of 25.
set(GEA_BOARD_EXTRA_COMPILE_DEFINITIONS GEA_BOARD_PREPARE_DISPLAY_PANEL=1 GEA_EMBEDDED_RM690B0_PANEL=1)

set(GEA_BOARD_PLATFORM_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/app_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/i2c.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/launcher_button.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/touch.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/panel_reset.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-elecrow-rotary-2.1/main/platform_stubs.cpp"
    "${GEA_CHIPS}/displays/sh8601/sh8601.cpp"
    "${GEA_CHIPS}/displays/rm690b0/rm690b0.cpp"
    "${GEA_CHIPS}/touch/ft6336/ft6336.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/display.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/displays/sh8601.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/touch/ft6336.cpp"
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-touch-amoled-1.8/main/CMakeLists.txt")
