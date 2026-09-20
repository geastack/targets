cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}/boards" CACHE STRING "Pico board header search dirs")
set(PICO_BOARD waveshare_rp2350_touch_amoled_2_41 CACHE STRING "Pico SDK board")
set(PICO_PLATFORM rp2350 CACHE STRING "Pico SDK platform")

if(DEFINED ENV{PICO_SDK_PATH})
    include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
elseif(DEFINED PICO_SDK_PATH)
    include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
else()
    message(FATAL_ERROR "PICO_SDK_PATH is required to build the RP2350 target.")
endif()

project(gea_rp2350_touch_amoled_241 C CXX ASM)
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
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_EMBEDDED_CPP_BOARD "waveshare-rp2350-amoled-2.41" CACHE STRING "Board profile passed to geatsc for C++ target selection")
set(GEA_EMBEDDED_TTF_RUNTIME_FONTS OFF CACHE BOOL "Use runtime TTF rasterization instead of baked embedded font atlases")
set(GEA_RP2350_STARTUP_TRACE OFF CACHE BOOL "Log app startup milestones over USB stdio")
set(GEA_EMBEDDED_TTF_RUNTIME_TRACE OFF CACHE BOOL "Log runtime TTF font cache milestones over USB stdio")

add_executable(gea_rp2350_touch_amoled_241_bringup
    main/bringup.cpp
    "${GEA_CHIPS}/displays/rm690b0/rm690b0.cpp"
    "${GEA_CHIPS}/imu/qmi8658/qmi8658.cpp"
    "${GEA_CHIPS}/power/eta6098/eta6098.cpp"
    "${GEA_CHIPS}/rtc/pcf85063/pcf85063.cpp"
    "${GEA_CHIPS}/touch/ft6336/ft6336.cpp"
)

target_include_directories(gea_rp2350_touch_amoled_241_bringup PRIVATE
    main/include
    "${GEA_CHIPS}"
)

target_compile_definitions(gea_rp2350_touch_amoled_241_bringup PRIVATE
    GEA_EMBEDDED_DISPLAY_WIDTH=450
    GEA_EMBEDDED_DISPLAY_HEIGHT=600
    GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=450
    GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=600
)

target_link_libraries(gea_rp2350_touch_amoled_241_bringup
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_i2c
)

pico_enable_stdio_usb(gea_rp2350_touch_amoled_241_bringup 1)
pico_enable_stdio_uart(gea_rp2350_touch_amoled_241_bringup 0)
pico_add_extra_outputs(gea_rp2350_touch_amoled_241_bringup)

set(GEA_EMBEDDED_APP "" CACHE STRING "Gea app id to compile into the RP2350 firmware")

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
    set(GEA_APP_NATIVE_SOURCE_RELS "")
    list(LENGTH GEA_APP_META_LIST GEA_APP_META_LENGTH)
    if(GEA_APP_META_LENGTH GREATER 3)
        list(SUBLIST GEA_APP_META_LIST 3 -1 GEA_APP_NATIVE_SOURCE_RELS)
    endif()

    if(NOT IS_ABSOLUTE "${GEA_APP_DIR}")
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META must start with the app's absolute directory (got '${GEA_APP_DIR}')")
    endif()
    if(NOT EXISTS "${GEA_APP_DIR}/package.json")
        message(FATAL_ERROR "Gea app '${GEA_EMBEDDED_APP}' not found at ${GEA_APP_DIR}.")
    endif()

    set(GEA_APP_BUILD_DIR "${CMAKE_BINARY_DIR}/apps/${GEA_EMBEDDED_APP}")
    set(GEA_APP_STAMP "${GEA_APP_BUILD_DIR}/.geatsc.stamp")
    set(GEA_APP_SOURCE_LIST "${GEA_APP_BUILD_DIR}/geatsc-sources.txt")
    set(GEA_APP_GEATSC_ARCHIVE "${GEA_APP_BUILD_DIR}/libgea_app_geatsc.a")
    set(GEA_APP_NATIVE_HEADERS_H "${GEA_APP_BUILD_DIR}/gea_app_native_headers.h")
    set(GEA_APP_FONT_CPP "${GEA_APP_BUILD_DIR}/gea_embedded_font_generated.cpp")
    set(GEA_APP_ASSET_CPP "${GEA_APP_BUILD_DIR}/gea_embedded_assets_generated.cpp")
    set(GEA_APP_NATIVE_SOURCES "")
    set(GEA_APP_NATIVE_INCLUDE_DIRS "")
    set(GEA_APP_NATIVE_HEADER_INCLUDES "")
    foreach(GEA_APP_NATIVE_SOURCE_REL IN LISTS GEA_APP_NATIVE_SOURCE_RELS)
        set(GEA_APP_NATIVE_SOURCE "${GEA_APP_DIR}/${GEA_APP_NATIVE_SOURCE_REL}")
        if(NOT EXISTS "${GEA_APP_NATIVE_SOURCE}")
            message(FATAL_ERROR "App native source not found: ${GEA_APP_NATIVE_SOURCE}")
        endif()
        list(APPEND GEA_APP_NATIVE_SOURCES "${GEA_APP_NATIVE_SOURCE}")
        get_filename_component(GEA_APP_NATIVE_SOURCE_DIR "${GEA_APP_NATIVE_SOURCE}" DIRECTORY)
        list(APPEND GEA_APP_NATIVE_INCLUDE_DIRS "${GEA_APP_NATIVE_SOURCE_DIR}")
        get_filename_component(GEA_APP_NATIVE_SOURCE_EXT "${GEA_APP_NATIVE_SOURCE}" EXT)
        if(GEA_APP_NATIVE_SOURCE_EXT MATCHES "^\\.(cc|cpp|cxx)$")
            get_filename_component(GEA_APP_NATIVE_SOURCE_BASE "${GEA_APP_NATIVE_SOURCE}" NAME_WE)
            foreach(GEA_APP_NATIVE_HEADER_EXT h hh hpp hxx)
                set(GEA_APP_NATIVE_HEADER "${GEA_APP_NATIVE_SOURCE_DIR}/${GEA_APP_NATIVE_SOURCE_BASE}.${GEA_APP_NATIVE_HEADER_EXT}")
                if(EXISTS "${GEA_APP_NATIVE_HEADER}")
                    list(APPEND GEA_APP_NATIVE_HEADER_INCLUDES "${GEA_APP_NATIVE_HEADER}")
                    break()
                endif()
            endforeach()
        elseif(GEA_APP_NATIVE_SOURCE_EXT MATCHES "^\\.(h|hh|hpp|hxx)$")
            list(APPEND GEA_APP_NATIVE_HEADER_INCLUDES "${GEA_APP_NATIVE_SOURCE}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES GEA_APP_NATIVE_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES GEA_APP_NATIVE_HEADER_INCLUDES)

    file(WRITE "${GEA_APP_NATIVE_HEADERS_H}" "#pragma once\n")
    foreach(GEA_APP_NATIVE_HEADER IN LISTS GEA_APP_NATIVE_HEADER_INCLUDES)
        file(APPEND "${GEA_APP_NATIVE_HEADERS_H}" "#include \"${GEA_APP_NATIVE_HEADER}\"\n")
    endforeach()

    file(GLOB_RECURSE GEA_APP_INPUTS CONFIGURE_DEPENDS
        "${GEA_APP_DIR}/*.css"
        "${GEA_APP_DIR}/*.ts"
        "${GEA_APP_DIR}/*.tsx"
        "${GEA_APP_DIR}/*.ttf"
        "${GEA_APP_DIR}/*.otf"
        "${GEA_APP_DIR}/package.json"
        "${GEA_APP_DIR}/vite.config.*"
    )

    set(GEA_EMBEDDED_FONT_ARGS "")
    if(GEA_EMBEDDED_TTF_RUNTIME_FONTS)
        list(APPEND GEA_EMBEDDED_FONT_ARGS --runtime-ttf-fonts)
    endif()

    # --cpp-prelude-symbol: emit the CSS/prelude registration as a function
    # instead of a file-scope global. At static-init time only the small SRAM
    # newlib heap exists (PSRAM comes up in main's memoryInit), and a large app
    # stylesheet OOM-panics the boot; main() calls the symbol after memoryInit
    # so the registration allocates from PSRAM.
    add_custom_command(
        OUTPUT "${GEA_APP_STAMP}"
        BYPRODUCTS
            "${GEA_APP_SOURCE_LIST}"
            "${GEA_APP_BUILD_DIR}/entry.cpp"
            "${GEA_APP_BUILD_DIR}/generated_support.hpp"
            "${GEA_APP_FONT_CPP}"
            "${GEA_APP_ASSET_CPP}"
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
            --cpp-board "${GEA_EMBEDDED_CPP_BOARD}"
            --cpp-prelude-symbol gea_plugin_cpp_register_prelude
            --pixel-panel-endian 1
            --font-viewport-width 450
            --font-viewport-height 600
            --font-device-pixel-ratio ${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
            --allow-any
            ${GEA_EMBEDDED_FONT_ARGS}
        COMMAND "${CMAKE_COMMAND}" -E touch "${GEA_APP_STAMP}"
        DEPENDS
            "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
            "${GEA_CORE}/scripts/generate-gea-embedded-fonts.mjs"
            ${GEA_APP_INPUTS}
        VERBATIM
    )
    add_custom_target(gea_rp2350_touch_amoled_241_codegen DEPENDS "${GEA_APP_STAMP}")

    set_source_files_properties(
        "${GEA_APP_FONT_CPP}"
        "${GEA_APP_ASSET_CPP}"
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
        "${GEA_HOST}/host/image.cpp"
        ${GEA_ENGINE_UI_SOURCES}
    )

    add_executable(gea_rp2350_touch_amoled_241_app
        main/app.cpp
        main/rp2350_gea_platform.cpp
        main/rp2350_host_fetch_stub.cpp
        main/rp2350_panel.cpp
        main/vendor/waveshare_psram/rp_pico_alloc.c
        main/vendor/waveshare_psram/tlsf/tlsf.c
        "${GEA_CHIPS}/touch/ft6336/ft6336.cpp"
        "${GEA_APP_FONT_CPP}"
        "${GEA_APP_ASSET_CPP}"
        ${GEA_APP_NATIVE_SOURCES}
        ${GEA_ENGINE_SOURCES}
    )
    add_dependencies(gea_rp2350_touch_amoled_241_app gea_rp2350_touch_amoled_241_codegen)

    target_include_directories(gea_rp2350_touch_amoled_241_app PRIVATE
        main
        main/include
        main/vendor/waveshare_psram
        "${GEA_APP_BUILD_DIR}"
        ${GEA_APP_NATIVE_INCLUDE_DIRS}
        "${GEA_CHIPS}"
        "${GEA_CORE}/include"
        "${GEA_ENGINE}"
        "${GEA_ENGINE}/ui"
        "${GEA_ELEMENTS}"
        "${GEA_ELEMENTS}/ui"
        "${GEA_ENGINE}/vendor/AnimatedGIF"
        "${GEA_ENGINE}/vendor/stb"
        "${GEA_HOST}/include"
    )

    target_compile_definitions(gea_rp2350_touch_amoled_241_app PRIVATE
        GEA_CPP_VALUE_AVAILABLE=1
        GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=1
        GEA_EMBEDDED_DISPLAY_WIDTH=450
        GEA_EMBEDDED_DISPLAY_HEIGHT=600
        GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=450
        GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=600
        GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
        GEA_EMBEDDED_HAS_GENERATED_FONTS=1
        GEA_EMBEDDED_DEFAULT_FRAME_INTERVAL_US=8333
        GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH=1
        GEA_EMBEDDED_DISPLAY_FUSE_INTERLEAVED_REPLAY_FLUSH=1
        GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_CLEAR_RGB565=-1
        GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS=1
        GEA_EMBEDDED_GRADIENT_LUT_SLOTS=6
        GEA_EMBEDDED_PERF=0
        GEA_EMBEDDED_RAF_PERF=0
        GEA_EMBEDDED_SINGLE_THREAD_IMAGE_STORE=1
        GEA_PICO_SDK=1
        GEA_RP2350_PSRAM_SIZE_BYTES=8388608
        PICO_PANIC_FUNCTION=gea_panic
        GEA_RP2350_RENDER_CORE1=1
        GEA_RP2350_RENDER_CORE1_STACK_BYTES=12288
        GEA_RP2350_RENDER_CORE1_WAIT_TIMEOUT_US=100000
        GEA_RP2350_FRAME_PHASE_PERF=0
        GEA_RP2350_REFRESH_DETAIL_PERF=0
        PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1
    )

    if(GEA_EMBEDDED_TTF_RUNTIME_FONTS)
        target_compile_definitions(gea_rp2350_touch_amoled_241_app PRIVATE
            GEA_EMBEDDED_TTF_RUNTIME_FONTS=1
        )
    endif()

    if(GEA_RP2350_STARTUP_TRACE)
        target_compile_definitions(gea_rp2350_touch_amoled_241_app PRIVATE
            GEA_RP2350_STARTUP_TRACE=1
        )
    endif()

    if(GEA_EMBEDDED_TTF_RUNTIME_TRACE)
        target_compile_definitions(gea_rp2350_touch_amoled_241_app PRIVATE
            GEA_EMBEDDED_TTF_RUNTIME_TRACE=1
        )
    endif()

    target_compile_options(gea_rp2350_touch_amoled_241_app PRIVATE
        -Wno-deprecated
        -Wno-unused-value
        -Wno-parentheses-equality
        -Wno-volatile
    )
    gea_geatsc_add_generated_archive(
        TARGET gea_rp2350_touch_amoled_241_app
        NAME gea_rp2350_touch_amoled_241_geatsc_archive
        SOURCE_LIST "${GEA_APP_SOURCE_LIST}"
        OUTPUT "${GEA_APP_GEATSC_ARCHIVE}"
        DEPENDS "${GEA_APP_STAMP}"
        COMPILE_OPTIONS
            -include
            "${GEA_APP_NATIVE_HEADERS_H}"
            -Wno-unused-variable
            -Wno-return-type
            -Wno-format-truncation
            -Os
    )

    target_link_libraries(gea_rp2350_touch_amoled_241_app
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_gpio
        hardware_i2c
        hardware_pio
        hardware_spi
        hardware_sync
        hardware_vreg
        hardware_xip_cache
        pico_multicore
        pico_rand
        m
    )

    pico_enable_stdio_usb(gea_rp2350_touch_amoled_241_app 1)
    pico_enable_stdio_uart(gea_rp2350_touch_amoled_241_app 0)
    pico_add_extra_outputs(gea_rp2350_touch_amoled_241_app)
endif()
