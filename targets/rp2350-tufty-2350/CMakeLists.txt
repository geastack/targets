cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/boards" CACHE STRING "Pico board header search dirs")
set(PICO_BOARD pimoroni_tufty2350 CACHE STRING "Pico SDK board")
set(PICO_PLATFORM rp2350 CACHE STRING "Pico SDK platform")
set(PICO_NUM_GPIOS 48 CACHE STRING "RP2350 GPIO count for Tufty 2350")

if(DEFINED ENV{PICO_SDK_PATH})
    include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
elseif(DEFINED PICO_SDK_PATH)
    include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
else()
    message(FATAL_ERROR "PICO_SDK_PATH is required to build the RP2350 target.")
endif()

project(gea_rp2350_tufty_2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

get_filename_component(GEA_TARGETS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
foreach(_GEA_REQUIRED_ENV GEA_CORE_DIR GEA_COMPILER_DIR GEA_CHIPS_DIR GEA_ENGINE_DIR GEA_HOST_DIR GEA_ELEMENTS_DIR GEA_GEAOS_PACKAGE_DIR)
    if(NOT DEFINED ENV{${_GEA_REQUIRED_ENV}} OR "$ENV{${_GEA_REQUIRED_ENV}}" STREQUAL "")
        message(FATAL_ERROR "${_GEA_REQUIRED_ENV} is unset. Run this target through the gea CLI so npm packages are resolved first.")
    endif()
endforeach()
get_filename_component(GEA_CORE "$ENV{GEA_CORE_DIR}" REALPATH)
get_filename_component(GEA_COMPILER "$ENV{GEA_COMPILER_DIR}" REALPATH)
get_filename_component(GEA_CHIPS "$ENV{GEA_CHIPS_DIR}" REALPATH)
get_filename_component(GEA_ENGINE "$ENV{GEA_ENGINE_DIR}" REALPATH)
get_filename_component(GEA_HOST "$ENV{GEA_HOST_DIR}" REALPATH)
get_filename_component(GEA_ELEMENTS "$ENV{GEA_ELEMENTS_DIR}" REALPATH)
include("${GEA_TARGETS_ROOT}/targets/cmake/geatsc_generated_archive.cmake")
set(GEA_RP2350_SHARED_MAIN "${GEA_TARGETS_ROOT}/targets/rp2350-waveshare-touch-amoled-2.41/main")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.0" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_RP2350_TUFTY_NATIVE_WIDTH 320)
set(GEA_RP2350_TUFTY_NATIVE_HEIGHT 240)

add_executable(gea_rp2350_tufty_2350_bringup
    main/bringup.cpp
    main/rp2350_panel.cpp
    "${GEA_CHIPS}/rtc/pcf85063/pcf85063.cpp"
)
pico_generate_pio_header(gea_rp2350_tufty_2350_bringup "${CMAKE_CURRENT_LIST_DIR}/main/st7789_parallel.pio")

target_include_directories(gea_rp2350_tufty_2350_bringup PRIVATE
    main
    main/include
    "${GEA_CHIPS}"
    "${GEA_CORE}/include"
)

target_compile_definitions(gea_rp2350_tufty_2350_bringup PRIVATE
    GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=1
    GEA_EMBEDDED_DISPLAY_WIDTH=${GEA_RP2350_TUFTY_NATIVE_WIDTH}
    GEA_EMBEDDED_DISPLAY_HEIGHT=${GEA_RP2350_TUFTY_NATIVE_HEIGHT}
    GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=${GEA_RP2350_TUFTY_NATIVE_WIDTH}
    GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=${GEA_RP2350_TUFTY_NATIVE_HEIGHT}
    GEA_RP2350_HAS_TOUCH=0
    GEA_RP2350_HAS_BUTTONS=1
)

target_link_libraries(gea_rp2350_tufty_2350_bringup
    pico_stdlib
    hardware_adc
    hardware_clocks
    hardware_dma
    hardware_gpio
    hardware_i2c
    hardware_pio
    hardware_pwm
)

pico_enable_stdio_usb(gea_rp2350_tufty_2350_bringup 1)
pico_enable_stdio_uart(gea_rp2350_tufty_2350_bringup 0)
pico_add_extra_outputs(gea_rp2350_tufty_2350_bringup)

set(GEA_EMBEDDED_APP "" CACHE STRING "Gea app id to compile into the RP2350 firmware")
if(GEA_EMBEDDED_APP STREQUAL "css-3d-cube")
    set(GEA_RP2350_TUFTY_PANEL_SCALE 1)
    set(GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO "1.0")
else()
    set(GEA_RP2350_TUFTY_PANEL_SCALE 1)
    set(GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO "${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}")
endif()
math(EXPR GEA_RP2350_TUFTY_DISPLAY_WIDTH "${GEA_RP2350_TUFTY_NATIVE_WIDTH} / ${GEA_RP2350_TUFTY_PANEL_SCALE}")
math(EXPR GEA_RP2350_TUFTY_DISPLAY_HEIGHT "${GEA_RP2350_TUFTY_NATIVE_HEIGHT} / ${GEA_RP2350_TUFTY_PANEL_SCALE}")
set(GEA_RP2350_TUFTY_FRAMEBUFFER_IN_SRAM 1)

if(GEA_EMBEDDED_APP)
    find_program(NODE_EXECUTABLE NAMES node)
    if(NOT NODE_EXECUTABLE)
        message(FATAL_ERROR "Node.js is required to build Gea apps for RP2350.")
    endif()

    # App metadata (root;entry;runtime;native sources...) is resolved once by
    # the gea CLI and passed as -DGEA_EMBEDDED_APP_META (and in the environment).
    if(NOT GEA_EMBEDDED_APP_META AND DEFINED ENV{GEA_EMBEDDED_APP_META})
        set(GEA_EMBEDDED_APP_META "$ENV{GEA_EMBEDDED_APP_META}")
    endif()
    if(NOT GEA_EMBEDDED_APP_META)
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META is unset for app '${GEA_EMBEDDED_APP}'. Run this target through the gea CLI: gea build --board <alias> --app ${GEA_EMBEDDED_APP}")
    endif()
    set(GEA_APP_META_LIST "${GEA_EMBEDDED_APP_META}")
    # Element 0 is the app's absolute directory (see gea_framework.cmake).
    list(GET GEA_APP_META_LIST 0 GEA_APP_DIR)
    list(GET GEA_APP_META_LIST 1 GEA_APP_ENTRY)

    if(NOT IS_ABSOLUTE "${GEA_APP_DIR}")
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META must start with the app's absolute directory (got '${GEA_APP_DIR}')")
    endif()
    if(NOT EXISTS "${GEA_APP_DIR}/package.json")
        message(FATAL_ERROR "Gea app '${GEA_EMBEDDED_APP}' not found at ${GEA_APP_DIR}.")
    endif()

    # The `inspect ... --format cmake` metadata is `root;entry;runtime;<native
    # sources...>`. Elements 3+ are app-owned native C/C++ sources (relative to
    # the app dir) declared via the app's `gea.nativeSources` — compile them
    # into the firmware alongside the generated program, and expose the app's
    # `native/` dir so their headers (and the generated program's host-binding
    # includes, e.g. "native_bench.h") resolve.
    set(GEA_APP_NATIVE_SOURCE_RELS "")
    list(LENGTH GEA_APP_META_LIST GEA_APP_META_LENGTH)
    if(GEA_APP_META_LENGTH GREATER 3)
        list(SUBLIST GEA_APP_META_LIST 3 -1 GEA_APP_NATIVE_SOURCE_RELS)
    endif()
    set(GEA_APP_NATIVE_SOURCES "")
    set(GEA_APP_NATIVE_INCLUDE_DIRS "")
    foreach(_GEA_APP_NATIVE_SOURCE_REL IN LISTS GEA_APP_NATIVE_SOURCE_RELS)
        list(APPEND GEA_APP_NATIVE_SOURCES "${GEA_APP_DIR}/${_GEA_APP_NATIVE_SOURCE_REL}")
        get_filename_component(_GEA_APP_NATIVE_SOURCE_DIR "${GEA_APP_DIR}/${_GEA_APP_NATIVE_SOURCE_REL}" DIRECTORY)
        list(APPEND GEA_APP_NATIVE_INCLUDE_DIRS "${_GEA_APP_NATIVE_SOURCE_DIR}")
    endforeach()
    if(GEA_APP_NATIVE_SOURCES)
        list(APPEND GEA_APP_NATIVE_INCLUDE_DIRS "${GEA_APP_DIR}/native")
        list(REMOVE_DUPLICATES GEA_APP_NATIVE_INCLUDE_DIRS)
    endif()

    set(GEA_APP_BUILD_DIR "${CMAKE_BINARY_DIR}/apps/${GEA_EMBEDDED_APP}")
    set(GEA_APP_STAMP "${GEA_APP_BUILD_DIR}/.geatsc.stamp")
    set(GEA_APP_SOURCE_LIST "${GEA_APP_BUILD_DIR}/geatsc-sources.txt")
    set(GEA_APP_GEATSC_ARCHIVE "${GEA_APP_BUILD_DIR}/libgea_app_geatsc.a")
    set(GEA_APP_FONT_CPP "${GEA_APP_BUILD_DIR}/gea_embedded_font_generated.cpp")

    file(GLOB_RECURSE GEA_APP_INPUTS CONFIGURE_DEPENDS
        "${GEA_APP_DIR}/*.css"
        "${GEA_APP_DIR}/*.ts"
        "${GEA_APP_DIR}/*.tsx"
        "${GEA_APP_DIR}/package.json"
        "${GEA_APP_DIR}/vite.config.*"
    )

    add_custom_command(
        OUTPUT "${GEA_APP_STAMP}"
        BYPRODUCTS
            "${GEA_APP_SOURCE_LIST}"
            "${GEA_APP_BUILD_DIR}/entry.cpp"
            "${GEA_APP_BUILD_DIR}/generated_support.hpp"
            "${GEA_APP_FONT_CPP}"
            "${GEA_APP_BUILD_DIR}/gea_embedded_font_generated.h"
            "${GEA_APP_BUILD_DIR}/runtime_pch.h"
            "${GEA_APP_BUILD_DIR}/runtime.cpp"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${GEA_APP_BUILD_DIR}"
        COMMAND "${NODE_EXECUTABLE}"
            "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
            --app-dir "${GEA_APP_DIR}"
            --entry "${GEA_APP_ENTRY}"
            --out-dir "${GEA_APP_BUILD_DIR}"
            --gea-ir-backend
            --pixel-panel-endian 1
            --font-viewport-width ${GEA_RP2350_TUFTY_DISPLAY_WIDTH}
            --font-viewport-height ${GEA_RP2350_TUFTY_DISPLAY_HEIGHT}
            --font-device-pixel-ratio ${GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO}
            --allow-any
        COMMAND "${CMAKE_COMMAND}" -E touch "${GEA_APP_STAMP}"
        DEPENDS
            "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
            ${GEA_APP_INPUTS}
        VERBATIM
    )
    add_custom_target(gea_rp2350_tufty_2350_codegen DEPENDS "${GEA_APP_STAMP}")

    set_source_files_properties(
        "${GEA_APP_FONT_CPP}"
        PROPERTIES GENERATED TRUE
    )

    file(GLOB GEA_ENGINE_UI_SOURCES CONFIGURE_DEPENDS
        "${GEA_ENGINE}/ui/*.cpp"
    )

    set(GEA_ENGINE_SOURCES
        "${GEA_ENGINE}/vendor/AnimatedGIF/AnimatedGIF.c"
        "${GEA_ENGINE}/bitmap_font.cpp"
        "${GEA_ENGINE}/canvas.cpp"
        "${GEA_ENGINE}/image_store.cpp"
        "${GEA_ENGINE}/rasterized_font.cpp"
        "${GEA_ENGINE}/touch_runtime.cpp"
        "${GEA_CORE}/events.cpp"
        "${GEA_ELEMENTS}/ui/camera_element.cpp"
        "${GEA_ELEMENTS}/ui/virtual_list.cpp"
        "${GEA_HOST}/input.cpp"
        ${GEA_ENGINE_UI_SOURCES}
    )

    add_executable(gea_rp2350_tufty_2350_app
        "${GEA_RP2350_SHARED_MAIN}/app.cpp"
        "${GEA_RP2350_SHARED_MAIN}/rp2350_gea_platform.cpp"
        main/rp2350_panel.cpp
        "${GEA_RP2350_SHARED_MAIN}/vendor/waveshare_psram/rp_pico_alloc.c"
        "${GEA_RP2350_SHARED_MAIN}/vendor/waveshare_psram/tlsf/tlsf.c"
        "${GEA_APP_FONT_CPP}"
        ${GEA_APP_NATIVE_SOURCES}
        ${GEA_ENGINE_SOURCES}
    )
    pico_generate_pio_header(gea_rp2350_tufty_2350_app "${CMAKE_CURRENT_LIST_DIR}/main/st7789_parallel.pio")
    add_dependencies(gea_rp2350_tufty_2350_app gea_rp2350_tufty_2350_codegen)

    target_include_directories(gea_rp2350_tufty_2350_app PRIVATE
        main
        main/include
        "${GEA_RP2350_SHARED_MAIN}"
        "${GEA_RP2350_SHARED_MAIN}/vendor/waveshare_psram"
        "${GEA_APP_BUILD_DIR}"
        "${GEA_CHIPS}"
        "${GEA_CORE}/include"
        "${GEA_ENGINE}"
        "${GEA_ENGINE}/ui"
        "${GEA_ELEMENTS}"
        "${GEA_ELEMENTS}/ui"
        "${GEA_ENGINE}/vendor/AnimatedGIF"
        "${GEA_ENGINE}/vendor/stb"
        "${GEA_HOST}/include"
        ${GEA_APP_NATIVE_INCLUDE_DIRS}
    )

    target_compile_definitions(gea_rp2350_tufty_2350_app PRIVATE
        GEA_CPP_VALUE_AVAILABLE=1
        GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=1
        GEA_EMBEDDED_DISPLAY_WIDTH=${GEA_RP2350_TUFTY_DISPLAY_WIDTH}
        GEA_EMBEDDED_DISPLAY_HEIGHT=${GEA_RP2350_TUFTY_DISPLAY_HEIGHT}
        GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=${GEA_RP2350_TUFTY_NATIVE_WIDTH}
        GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=${GEA_RP2350_TUFTY_NATIVE_HEIGHT}
        GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=${GEA_RP2350_TUFTY_CSS_DEVICE_PIXEL_RATIO}
        GEA_EMBEDDED_HAS_GENERATED_FONTS=1
        GEA_EMBEDDED_DEFAULT_FRAME_INTERVAL_US=16666
        GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH=0
        GEA_EMBEDDED_DISPLAY_FUSE_INTERLEAVED_REPLAY_FLUSH=0
        GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_CLEAR_RGB565=-1
        GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS=0
        GEA_EMBEDDED_UI_TRANSFORM_CACHE_SLOTS=64
        GEA_EMBEDDED_UI_DEPTH_CACHE_SLOTS=64
        GEA_EMBEDDED_UI_CORNER_CACHE_SLOTS=64
        GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_SLOTS=8
        GEA_EMBEDDED_TRANSFORMED_GRADIENT_EDGE_CACHE=1
        GEA_EMBEDDED_RECOLOR_PIXEL_CACHE_SIZE=512
        GEA_EMBEDDED_LINE_BREAK_CACHE_SLOTS=4
        GEA_EMBEDDED_LINE_BREAK_CACHE_LINES=32
        GEA_EMBEDDED_CANVAS_CIRCLE_RADIUS_MAX=31
        GEA_EMBEDDED_CANVAS_CIRCLE_BOX_SPAN_MAX=96
        GEA_EMBEDDED_CANVAS_CIRCLE_BOX_SPAN_SLOTS=4
        GEA_EMBEDDED_PERF=1
        GEA_FRAME_PERF_LOG=0
        GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG=0
        GEA_EMBEDDED_UI_REFRESH_PERF=1
        GEA_EMBEDDED_CANVAS_PERF_DETAIL=0
        GEA_EMBEDDED_RAF_PERF=0
        GEA_EMBEDDED_SINGLE_THREAD_IMAGE_STORE=1
        GEA_PICO_SDK=1
        GEA_RP2350_HAS_TOUCH=0
        GEA_RP2350_HAS_BUTTONS=1
        GEA_RP2350_DEFAULT_BUTTON_SCROLL=1
        GEA_RP2350_FRAMEBUFFER_IN_SRAM=${GEA_RP2350_TUFTY_FRAMEBUFFER_IN_SRAM}
        GEA_RP2350_DEFAULT_BACKGROUND_CACHE_IN_SRAM=0
        GEA_RP2350_DEFAULT_DISPLAY_COMMAND_BUFFER_COMMANDS=192
        GEA_RP2350_EARLY_PSRAM_ALLOC=1
        GEA_RP2350_PIE_BLEND=1
        GEA_RP2350_RENDER_CORE1=1
        GEA_RP2350_RENDER_CORE1_STACK_BYTES=12288
        GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US=100000
        GEA_EMBEDDED_PIE_BLEND_SPAN8_GATHER=0
        GEA_EMBEDDED_TRANSFORMED_GRADIENT_A5_MIRROR=0
        GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_LOCK=1
        GEA_EMBEDDED_TRANSFORMED_GRADIENT_LUT_BANKS=2
        GEA_EMBEDDED_RENDER_PARALLEL_DIRTY_REPLAY=0
        GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS=24
        GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS=6000
        GEA_EMBEDDED_DISPLAY_NODE_SCRATCH_IN_SRAM=1
        GEA_EMBEDDED_DISPLAY_REPLAY_CLIP_STACK_IN_SRAM=1
        GEA_EMBEDDED_DISPLAY_RECORD_CHILDREN_IN_SRAM=1
        GEA_EMBEDDED_RARE_STYLE_SRAM_POOL_ENTRIES=48
        GEA_EMBEDDED_RENDER_HOT_SRAM=1
        GEA_EMBEDDED_PROJECTED_TEXT_CACHE_BANKS=2
        GEA_EMBEDDED_PROJECTED_TEXT_SRAM_CACHE_BYTES=12288
        GEA_EMBEDDED_TEXT_SPRITE_CACHE=0
        GEA_RP2350_STARTUP_TRACE=0
        GEA_RP2350_PERF_LOG=0
        GEA_RP2350_FRAME_PHASE_PERF=0
        GEA_RP2350_REFRESH_DETAIL_PERF=0
        GEA_RP2350_PANEL_SCALE=${GEA_RP2350_TUFTY_PANEL_SCALE}
        GEA_RP2350_PSRAM_SIZE_BYTES=8388608
        RP2350_XIP_CSI_PIN=8
        PICO_CORE1_STACK_SIZE=4096
        PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1
    )

    target_compile_options(gea_rp2350_tufty_2350_app PRIVATE
        -Wno-deprecated
        -Wno-unused-value
        -Wno-parentheses-equality
        -Wno-volatile
    )
    gea_geatsc_add_generated_archive(
        TARGET gea_rp2350_tufty_2350_app
        NAME gea_rp2350_tufty_2350_geatsc_archive
        SOURCE_LIST "${GEA_APP_SOURCE_LIST}"
        OUTPUT "${GEA_APP_GEATSC_ARCHIVE}"
        DEPENDS "${GEA_APP_STAMP}"
        COMPILE_OPTIONS
            -Wno-unused-variable
            -Wno-return-type
            -Wno-format-truncation
            -Os
    )

    target_link_libraries(gea_rp2350_tufty_2350_app
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_gpio
        hardware_i2c
        hardware_pio
        hardware_pwm
        hardware_spi
        hardware_sync
        hardware_vreg
        pico_multicore
        pico_rand
        m
    )

    pico_enable_stdio_usb(gea_rp2350_tufty_2350_app 1)
    pico_enable_stdio_uart(gea_rp2350_tufty_2350_app 0)
    pico_add_extra_outputs(gea_rp2350_tufty_2350_app)
endif()
