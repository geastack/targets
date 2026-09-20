# The Gea app pipeline is common to the ESP32 e-paper boards. Keep only the
# M5PaperS3 4.7" hardware/profile choices here; the included component
# definition (shared with the other e-paper targets) resolves sources relative
# to this component directory and picks up this board's own display.cpp/board.h.
set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-s3-m5stack-papers3")
set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 960)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT 960)
# DPR 1: a logical viewport of 540x960, the same as the LilyGo T5 4.7" board,
# which carries the IDENTICAL 540x960 GRAY4 panel.
#
# This was "2.0" on the theory that the panel is dense enough for DPR 1 to make
# CSS text physically tiny. What it actually did was halve the logical viewport
# to 270x480, and every app in the corpus is authored against 540x960 -- so the
# layouts did not merely look small, they OVERFLOWED. Measured on device with
# `gea devctl node`: the e-reader library's 3-up cover grid gives each column
# 132 device px while a `.library-book-row` is a fixed 150 CSS px = 300 device
# px that does not shrink, so the grid reported a 620 device px content width
# inside a 452 device px box and the covers overlapped and ran off the screen.
# The app's own arithmetic says 540: 22 + 150 + 14 + 150 + 14 + 150 + 22 = 522.
#
# If the type turns out to be too small for this panel in physical terms, that
# is a question for the app's CSS (or a board-level font scale), not for the
# logical viewport size -- shrinking the viewport moves every layout off the
# board it was authored for.
set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.0")
# Real BMI270 accelerometer/gyroscope, not the no-op default.
set(GEA_EMBEDDED_TARGET_IMU_SOURCE "imu.cpp")
set(GEA_EMBEDDED_TARGET_AUDIO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/audio_stub.cpp")
set(GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/services/audio_runtime_stub.cpp")
# papers3_epd: our board-local direct-drive panel engine (components/). The
# panel is wired straight to the SoC — there is no IT8951/controller to talk to.
set(GEA_EMBEDDED_TARGET_EXTRA_REQUIRES esp_adc bootloader_support spi_flash esp_psram papers3_epd)
set(GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS
    # No speaker/mic on this board.
    GEA_EMBEDDED_AUDIO_DISABLED=1
    # A full-page reader screen is ~100 nodes and a page turn rebuilds the page
    # subtree before it tears the old one down, so the node table needs headroom
    # the 512 default does not have on a 540x960 panel. At ~318 B/node this is
    # ~650 KB, which lives in PSRAM (see treeState()).
    GEA_EMBEDDED_MAX_NODES=1024
    # Note: unlike the classic-ESP32 M5Paper we do NOT set GEA_EMBEDDED_ROM_TJPGD
    # (its "esp32/rom/tjpgd.h" is ESP32-only). The S3's 8 MB PSRAM lets stb_image
    # decode full-page EPUB covers, matching the other S3 targets.
    # Large images decode to 1-channel grayscale (this is a grayscale e-paper).
    GEA_EMBEDDED_IMAGE_GRAYSCALE_DECODE=1
    # Render straight into a 16-level, 4-bit packed Canvas (GEA_PIXEL_GRAY4).
    # The driver maps it to the panel's four calibrated physical states.
    GEA_EMBEDDED_PIXEL_FORMAT=3
    # Calibrate the two 2-bit glyph edge coverages to gray4 levels 7 and 5 on
    # the direct-drive panel. The LUT remains endpoint-exact (0/255).
    GEA_EMBEDDED_FONT_COVERAGE_PAPERS3=1
    # Landscape-native rendering: the panel is physically 960x540 landscape but
    # apps render logical portrait 540x960. This enables the Canvas's rotation-
    # aware packed addressing (bindPixelsRotatedLandscape) so draws land directly
    # in the physical buffer — no per-frame transpose. Additive + compile-gated:
    # only this target compiles the rotated path; every other target is unaffected.
    GEA_EMBEDDED_DISPLAY_ROTATE_LANDSCAPE=1
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-epaper-1.54/main/CMakeLists.txt")
