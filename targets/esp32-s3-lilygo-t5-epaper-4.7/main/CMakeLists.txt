# The Gea app pipeline is common to the ESP32 e-paper boards. Keep only the
# LilyGo T5 4.7" hardware/profile choices here; the included component
# definition (shared with the other e-paper targets) resolves sources relative
# to this component directory and picks up this board's own display.cpp/board.h.
set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-s3-lilygo-t5-epaper-4.7")
set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 960)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT 960)
set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.0")
set(GEA_EMBEDDED_TARGET_AUDIO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/audio_stub.cpp")
set(GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/services/audio_runtime_stub.cpp")
# epd_driver: our board-local vendored ED047TC1 low-level driver (components/).
set(GEA_EMBEDDED_TARGET_EXTRA_REQUIRES esp_adc bootloader_support spi_flash esp_psram epd_driver)
set(GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS
    # No speaker/mic on this board.
    GEA_EMBEDDED_AUDIO_DISABLED=1
    # Note: unlike the classic-ESP32 M5Paper we do NOT set GEA_EMBEDDED_ROM_TJPGD
    # (its "esp32/rom/tjpgd.h" is ESP32-only). The S3's 8 MB PSRAM lets stb_image
    # decode full-page EPUB covers, matching the other S3 targets.
    # Large images decode to 1-channel grayscale (this is a grayscale e-paper).
    GEA_EMBEDDED_IMAGE_GRAYSCALE_DECODE=1
    # Render straight into the panel's native 16-level, 4-bit packed format
    # (GEA_PIXEL_GRAY4). The Canvas framebuffer IS the grayscale buffer.
    GEA_EMBEDDED_PIXEL_FORMAT=3
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-epaper-1.54/main/CMakeLists.txt")
