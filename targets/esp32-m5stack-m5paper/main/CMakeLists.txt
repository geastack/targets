# The Gea app pipeline is common to the ESP32 e-paper boards. Keep only the
# M5Paper hardware/profile choices here; the included component definition
# resolves sources relative to this component directory.
set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-m5stack-m5paper")
set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 960)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH 540)
set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT 960)
set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.0")
set(GEA_EMBEDDED_TARGET_AUDIO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/audio_stub.cpp")
set(GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/services/audio_runtime_stub.cpp")
set(GEA_EMBEDDED_TARGET_EXTRA_REQUIRES esp_adc bootloader_support spi_flash esp_psram)
set(GEA_EMBEDDED_TARGET_LDFRAGMENTS "${CMAKE_CURRENT_LIST_DIR}/m5paper_psram.lf")
set(GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS
    # M5Paper has no speaker or microphone. Keep Web Audio timing functional,
    # but do not create a PCM render task for oscillator output.
    GEA_EMBEDDED_AUDIO_DISABLED=1
    # Decode large JPEGs (e.g. EPUB covers) with the ESP32 ROM tinyjpeg
    # decompressor: block-streaming, ~a few KB of work RAM, and 1/2–1/8
    # descaling. stb_image needs several large contiguous buffers that the
    # reader's fragmented PSRAM can't satisfy for a full-page cover.
    GEA_EMBEDDED_ROM_TJPGD=1
    # Large images decode to 1-channel grayscale (this is a grayscale e-paper).
    GEA_EMBEDDED_IMAGE_GRAYSCALE_DECODE=1
    # Render straight into the panel's native 16-level, 4-bit packed format
    # (GEA_PIXEL_GRAY4). The Canvas framebuffer IS the IT8951 buffer: no RGB565
    # intermediate, no per-frame luma repack, quarter-size framebuffer — halving
    # both the fill and the flush cost of a full-screen page turn.
    GEA_EMBEDDED_PIXEL_FORMAT=3
    # app_main is only a small pre-scheduler bootstrap. The actual Gea runtime
    # task is allocated after ESP-IDF releases the ROM startup-stack D/IRAM.
    GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768
)

include("${CMAKE_CURRENT_LIST_DIR}/../../esp32-s3-epaper-1.54/main/CMakeLists.txt")

# Generated app archives are linked after ESP-IDF's component archives. Force
# the hardware RNG implementation out of bootloader_support before that final
# archive introduces its references (for example, Tetris bag shuffling).
if(NOT CMAKE_SCRIPT_MODE_FILE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-u bootloader_random_enable"
        "-u bootloader_random_disable"
    )
endif()
