cmake_minimum_required(VERSION 3.20)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/gea_build_config.cmake")

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

project(gea_stm32f746g_disco C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")

# The CMSIS headers and startup file are vendored (vendor/README.md); an
# STM32CubeF7 checkout given as STM32_CUBE_F7_PATH replaces them.
set(STM32_CUBE_F7_PATH "$ENV{STM32_CUBE_F7_PATH}" CACHE PATH "Optional STM32CubeF7 checkout to take the CMSIS headers and startup file from")
set(GEA_STM32_CMSIS "${CMAKE_CURRENT_LIST_DIR}/vendor/cmsis")
if(EXISTS "${STM32_CUBE_F7_PATH}/Drivers/CMSIS/Device/ST/STM32F7xx/Include/stm32f746xx.h")
    set(GEA_STM32_CMSIS "${STM32_CUBE_F7_PATH}/Drivers/CMSIS")
elseif(STM32_CUBE_F7_PATH)
    message(WARNING "STM32_CUBE_F7_PATH has no Drivers/CMSIS/Device/ST/STM32F7xx; using the vendored CMSIS headers.")
endif()
message(STATUS "CMSIS: ${GEA_STM32_CMSIS}")

if(NOT DEFINED ENV{GEA_CHIPS_DIR} OR "$ENV{GEA_CHIPS_DIR}" STREQUAL "")
    message(FATAL_ERROR "GEA_CHIPS_DIR is unset. Run this target through the gea CLI so npm packages are resolved first.")
endif()
get_filename_component(GEA_CHIPS "$ENV{GEA_CHIPS_DIR}" REALPATH)
get_filename_component(GEA_TARGETS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

set(GEA_STM32_STEM gea_stm32f746g_disco)
set(GEA_STM32_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/stm32f746g_disco.ld")

# Board support shared by the bring-up and app images. An object library, not
# an archive: the strong fault handlers and newlib syscalls must always win
# over the weak defaults in the startup file and libnosys.
add_library(${GEA_STM32_STEM}_board OBJECT
    "${GEA_STM32_CMSIS}/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s"
    main/chrono_clocks.cpp
    main/clock.cpp
    main/console.cpp
    main/fault_handlers.cpp
    main/i2c_bus.cpp
    main/lcd.cpp
    main/newlib_syscalls.cpp
    main/qspi.cpp
    main/qspi_personality.S
    main/rtc_clock.cpp
    main/system_init.cpp
    main/touch_panel.cpp
    main/unwind_tables.cpp
    main/user_button.cpp
    "${GEA_CHIPS}/touch/ft5336/ft5336.cpp"
)
target_include_directories(${GEA_STM32_STEM}_board PUBLIC
    main
    main/include
    "${GEA_CHIPS}"
    "${GEA_STM32_CMSIS}/Core/Include"
    "${GEA_STM32_CMSIS}/Device/ST/STM32F7xx/Include"
)
target_compile_definitions(${GEA_STM32_STEM}_board PUBLIC STM32F746xx)
target_compile_options(${GEA_STM32_STEM}_board PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra>")

# Links `archive` into `target` from QSPI (the linker script places every
# *_qspi.a member there) and points its unwind entries at the QSPI thunk.
function(gea_stm32_place_archive_in_qspi target archive)
    add_custom_command(TARGET ${target} PRE_LINK
        COMMAND "${CMAKE_OBJCOPY}" --redefine-sym __gxx_personality_v0=__gea_qspi_gxx_personality_v0 "${archive}"
        VERBATIM
    )
endfunction()

function(gea_stm32_link_image target)
    target_link_libraries(${target} PRIVATE ${GEA_STM32_STEM}_board)
    target_link_options(${target} PRIVATE
        "-T${GEA_STM32_LINKER_SCRIPT}"
        --specs=nosys.specs
        "-Wl,-Map=$<TARGET_FILE_DIR:${target}>/${target}.map"
        -Wl,--print-memory-usage
    )
    set_target_properties(${target} PROPERTIES LINK_DEPENDS "${GEA_STM32_LINKER_SCRIPT}")
endfunction()

add_library(${GEA_STM32_STEM}_bringup_qspi STATIC main/bringup_qspi_probe.cpp)
add_executable(${GEA_STM32_STEM}_bringup main/bringup.cpp)
gea_stm32_link_image(${GEA_STM32_STEM}_bringup)
target_link_libraries(${GEA_STM32_STEM}_bringup PRIVATE ${GEA_STM32_STEM}_bringup_qspi)
gea_stm32_place_archive_in_qspi(${GEA_STM32_STEM}_bringup "$<TARGET_FILE:${GEA_STM32_STEM}_bringup_qspi>")

# ---- app image ----------------------------------------------------------------

set(GEA_EMBEDDED_APP "" CACHE STRING "Gea app id to compile into the STM32 firmware")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.0" CACHE STRING "Logical CSS px to physical pixel scale for this board")
set(GEA_STM32_DISPLAY_WIDTH 480)
set(GEA_STM32_DISPLAY_HEIGHT 272)

if(GEA_EMBEDDED_APP)
    foreach(_GEA_REQUIRED_ENV GEA_CORE_DIR GEA_ENGINE_DIR GEA_HOST_DIR GEA_ELEMENTS_DIR)
        if(NOT DEFINED ENV{${_GEA_REQUIRED_ENV}} OR "$ENV{${_GEA_REQUIRED_ENV}}" STREQUAL "")
            message(FATAL_ERROR "${_GEA_REQUIRED_ENV} is unset. Run this target through the gea CLI so npm packages are resolved first.")
        endif()
    endforeach()
    get_filename_component(GEA_CORE "$ENV{GEA_CORE_DIR}" REALPATH)
    get_filename_component(GEA_ENGINE "$ENV{GEA_ENGINE_DIR}" REALPATH)
    get_filename_component(GEA_HOST "$ENV{GEA_HOST_DIR}" REALPATH)
    get_filename_component(GEA_ELEMENTS "$ENV{GEA_ELEMENTS_DIR}" REALPATH)
    include("${GEA_TARGETS_ROOT}/targets/cmake/geatsc_generated_archive.cmake")

    find_program(NODE_EXECUTABLE NAMES node REQUIRED)

    # App metadata (root;entry;runtime;native sources...) is resolved once by
    # the gea CLI and passed as -DGEA_EMBEDDED_APP_META (and in the environment).
    if(NOT GEA_EMBEDDED_APP_META AND DEFINED ENV{GEA_EMBEDDED_APP_META})
        set(GEA_EMBEDDED_APP_META "$ENV{GEA_EMBEDDED_APP_META}")
    endif()
    if(NOT GEA_EMBEDDED_APP_META)
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META is unset for app '${GEA_EMBEDDED_APP}'. Run this target through the gea CLI: gea build --board <alias> --app ${GEA_EMBEDDED_APP}")
    endif()
    set(GEA_APP_META_LIST "${GEA_EMBEDDED_APP_META}")
    list(GET GEA_APP_META_LIST 0 GEA_APP_DIR)
    list(GET GEA_APP_META_LIST 1 GEA_APP_ENTRY)
    if(NOT IS_ABSOLUTE "${GEA_APP_DIR}" OR NOT EXISTS "${GEA_APP_DIR}/package.json")
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META must start with the absolute directory of app '${GEA_EMBEDDED_APP}' (got '${GEA_APP_DIR}')")
    endif()

    # Elements 3+ are the app's gea.nativeSources, relative to its directory.
    set(GEA_APP_NATIVE_SOURCES "")
    set(GEA_APP_NATIVE_INCLUDE_DIRS "")
    list(LENGTH GEA_APP_META_LIST GEA_APP_META_LENGTH)
    if(GEA_APP_META_LENGTH GREATER 3)
        list(SUBLIST GEA_APP_META_LIST 3 -1 GEA_APP_NATIVE_SOURCE_RELS)
        foreach(GEA_APP_NATIVE_SOURCE_REL IN LISTS GEA_APP_NATIVE_SOURCE_RELS)
            set(GEA_APP_NATIVE_SOURCE "${GEA_APP_DIR}/${GEA_APP_NATIVE_SOURCE_REL}")
            get_filename_component(GEA_APP_NATIVE_SOURCE_DIR "${GEA_APP_NATIVE_SOURCE}" DIRECTORY)
            list(APPEND GEA_APP_NATIVE_SOURCES "${GEA_APP_NATIVE_SOURCE}")
            list(APPEND GEA_APP_NATIVE_INCLUDE_DIRS "${GEA_APP_NATIVE_SOURCE_DIR}")
        endforeach()
        list(REMOVE_DUPLICATES GEA_APP_NATIVE_INCLUDE_DIRS)
    endif()

    set(GEA_APP_BUILD_DIR "${CMAKE_BINARY_DIR}/apps/${GEA_EMBEDDED_APP}")
    set(GEA_APP_STAMP "${GEA_APP_BUILD_DIR}/.geatsc.stamp")
    set(GEA_APP_SOURCE_LIST "${GEA_APP_BUILD_DIR}/geatsc-sources.txt")
    set(GEA_APP_ARCHIVE "${GEA_APP_BUILD_DIR}/libgea_app_qspi.a")
    set(GEA_APP_FONT_CPP "${GEA_APP_BUILD_DIR}/gea_embedded_font_generated.cpp")
    set(GEA_APP_ASSET_CPP "${GEA_APP_BUILD_DIR}/gea_embedded_assets_generated.cpp")

    file(GLOB_RECURSE GEA_APP_INPUTS CONFIGURE_DEPENDS
        "${GEA_APP_DIR}/*.css"
        "${GEA_APP_DIR}/*.ts"
        "${GEA_APP_DIR}/*.tsx"
        "${GEA_APP_DIR}/*.ttf"
        "${GEA_APP_DIR}/*.otf"
        "${GEA_APP_DIR}/*.png"
        "${GEA_APP_DIR}/*.jpg"
        "${GEA_APP_DIR}/*.jpeg"
        "${GEA_APP_DIR}/*.gif"
        "${GEA_APP_DIR}/package.json"
        "${GEA_APP_DIR}/vite.config.*"
    )
    list(FILTER GEA_APP_INPUTS EXCLUDE REGEX "/node_modules/|/\\.gea/|/dist/")

    # gea.compilerPlugins (absolute paths from the gea CLI), as the repeated
    # flag the geatsc driver takes.
    set(GEA_APP_GEATSC_PLUGINS "$ENV{GEA_EMBEDDED_APP_GEATSC_PLUGINS}")
    set(GEA_APP_GEATSC_PLUGIN_ARGS "")
    foreach(GEA_APP_GEATSC_PLUGIN IN LISTS GEA_APP_GEATSC_PLUGINS)
        if(NOT EXISTS "${GEA_APP_GEATSC_PLUGIN}")
            message(FATAL_ERROR "gea.compilerPlugins entry does not exist: ${GEA_APP_GEATSC_PLUGIN}")
        endif()
        list(APPEND GEA_APP_GEATSC_PLUGIN_ARGS --extra-geatsc-plugin "${GEA_APP_GEATSC_PLUGIN}")
    endforeach()

    # The LTDC takes little-endian RGB565 as the CPU stores it, so canvas
    # colour literals stay unswapped (--pixel-panel-endian 0).
    add_custom_command(
        OUTPUT "${GEA_APP_STAMP}"
        BYPRODUCTS
            "${GEA_APP_SOURCE_LIST}"
            "${GEA_APP_FONT_CPP}"
            "${GEA_APP_ASSET_CPP}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${GEA_APP_BUILD_DIR}"
        COMMAND "${NODE_EXECUTABLE}"
            "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
            --build-config "${GEA_BUILD_CONFIG_JSON}"
            --app-dir "${GEA_APP_DIR}"
            --entry "${GEA_APP_ENTRY}"
            --out-dir "${GEA_APP_BUILD_DIR}"
            --gea-ir-backend
            --cpp-prelude-symbol gea_plugin_cpp_register_prelude
            --pixel-panel-endian 0
            --font-viewport-width ${GEA_STM32_DISPLAY_WIDTH}
            --font-viewport-height ${GEA_STM32_DISPLAY_HEIGHT}
            --font-device-pixel-ratio ${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
            --allow-any
            ${GEA_APP_GEATSC_PLUGIN_ARGS}
        COMMAND "${CMAKE_COMMAND}" -E touch "${GEA_APP_STAMP}"
        DEPENDS
            "${GEA_BUILD_CONFIG_JSON}"
            "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
            ${GEA_APP_INPUTS}
            ${GEA_APP_GEATSC_PLUGINS}
        VERBATIM
    )
    add_custom_target(${GEA_STM32_STEM}_codegen DEPENDS "${GEA_APP_STAMP}")
    set_source_files_properties("${GEA_APP_FONT_CPP}" "${GEA_APP_ASSET_CPP}" PROPERTIES GENERATED TRUE)

    # Framework code that runs once per change rather than once per pixel
    # (CSS parsing and cascade, DOM events, image decoding, form controls)
    # executes from QSPI; the per-frame render, layout and canvas paths stay
    # in internal flash, which the ART accelerator serves without wait states.
    set(GEA_FRAMEWORK_QSPI_SOURCES
        "${GEA_ENGINE}/vendor/AnimatedGIF/AnimatedGIF.c"
        "${GEA_ENGINE}/image_store.cpp"
        "${GEA_ENGINE}/ui/css_atom.cpp"
        "${GEA_ENGINE}/ui/document.cpp"
        "${GEA_ENGINE}/ui/input.cpp"
        "${GEA_ENGINE}/ui/style.cpp"
        "${GEA_ENGINE}/ui/tree_events.cpp"
        "${GEA_ENGINE}/ui/tree_nodes.cpp"
        "${GEA_ENGINE}/ui/virtual_keyboard.cpp"
        "${GEA_ELEMENTS}/ui/camera_element.cpp"
        "${GEA_ELEMENTS}/ui/virtual_list.cpp"
        "${GEA_HOST}/host/gpio.cpp"
        "${GEA_HOST}/host/image.cpp"
        "${GEA_HOST}/host/memory.cpp"
    )
    file(GLOB GEA_ENGINE_UI_SOURCES CONFIGURE_DEPENDS "${GEA_ENGINE}/ui/*.cpp")
    set(GEA_FRAMEWORK_SOURCES
        "${GEA_ENGINE}/bitmap_font.cpp"
        "${GEA_ENGINE}/canvas.cpp"
        "${GEA_ENGINE}/rasterized_font.cpp"
        "${GEA_ENGINE}/touch_runtime.cpp"
        ${GEA_ENGINE_UI_SOURCES}
        "${GEA_CORE}/app_frame_perf.cpp"
        "${GEA_CORE}/events.cpp"
        "${GEA_HOST}/input.cpp"
        "${GEA_HOST}/host/display.cpp"
        "${GEA_HOST}/host/input.cpp"
        "${GEA_HOST}/host/touch.cpp"
    )
    list(REMOVE_ITEM GEA_FRAMEWORK_SOURCES ${GEA_FRAMEWORK_QSPI_SOURCES})

    add_executable(${GEA_STM32_STEM}_app
        main/absent_audio.cpp
        main/app.cpp
        main/atomic64.c
        main/dev_control.cpp
        main/frame_rate.cpp
        main/platform_display.cpp
        main/platform_gpio.cpp
        main/platform_sensors.cpp
        main/platform_services.cpp
        main/platform_timers.cpp
        main/platform_touch.cpp
        main/thread_local_storage.cpp
        "${GEA_APP_FONT_CPP}"
        "${GEA_APP_ASSET_CPP}"
        ${GEA_APP_NATIVE_SOURCES}
        ${GEA_FRAMEWORK_SOURCES}
    )
    add_dependencies(${GEA_STM32_STEM}_app ${GEA_STM32_STEM}_codegen)

    target_include_directories(${GEA_STM32_STEM}_app PRIVATE
        "${GEA_APP_BUILD_DIR}"
        ${GEA_APP_NATIVE_INCLUDE_DIRS}
        "${GEA_CORE}/include"
        "${GEA_ENGINE}"
        "${GEA_ENGINE}/ui"
        "${GEA_ELEMENTS}"
        "${GEA_ELEMENTS}/ui"
        "${GEA_ENGINE}/vendor/AnimatedGIF"
        "${GEA_ENGINE}/vendor/stb"
        "${GEA_HOST}/include"
    )

    # GEA_EMBEDDED_CPP_BOARD sizes the runtime's allocation pools for a board.
    # The engine anchors partial flushes at x=0 for a panel quirk the LTDC
    # front buffer does not have, so flushes cover only the dirty width.
    target_compile_definitions(${GEA_STM32_STEM}_app PRIVATE
        ${GEA_EMBEDDED_APP_DEFINES}
        GEA_EMBEDDED_CPP_BOARD
        GEA_RUNTIME_SINGLE_THREADED=1
        GEA_EMBEDDED_GIF_C_API
        GEA_STM32_APP_ID="${GEA_EMBEDDED_APP}"
        GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=0
        GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS=0
        # One distance per AA edge pixel, not a 4x4 grid of point samples.
        GEA_EMBEDDED_ANALYTIC_AA=1
        GEA_EMBEDDED_DISPLAY_WIDTH=${GEA_STM32_DISPLAY_WIDTH}
        GEA_EMBEDDED_DISPLAY_HEIGHT=${GEA_STM32_DISPLAY_HEIGHT}
        GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
        GEA_EMBEDDED_HAS_GENERATED_FONTS=1
        GEA_EMBEDDED_PERF=0
        GEA_EMBEDDED_RAF_PERF=0
        GEA_EMBEDDED_SINGLE_THREAD_IMAGE_STORE=1
    )
    # The bare-metal libstdc++ has no std::mutex; the host sources lock with it.
    target_compile_options(${GEA_STM32_STEM}_app PRIVATE
        "$<$<COMPILE_LANGUAGE:CXX>:-include;${CMAKE_CURRENT_LIST_DIR}/main/single_thread_sync.h>"
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated -Wno-unused-value -Wno-volatile -Wno-unused-parameter>
    )

    gea_geatsc_add_generated_archive(
        TARGET ${GEA_STM32_STEM}_app
        NAME ${GEA_STM32_STEM}_geatsc_archive
        SOURCE_LIST "${GEA_APP_SOURCE_LIST}"
        OUTPUT "${GEA_APP_ARCHIVE}"
        DEPENDS "${GEA_APP_STAMP}"
        COMPILE_OPTIONS
            -Wno-unused-variable
            -Wno-return-type
            -Wno-format-truncation
            -O2
    )
    # Both targets list the codegen stamp; ordering them keeps make -j from
    # running the generator twice at once.
    add_dependencies(${GEA_STM32_STEM}_geatsc_archive ${GEA_STM32_STEM}_codegen)
    # Built exactly like the framework sources in internal flash, and linked
    # whole so its place on the link line beside the app archive never matters.
    add_library(${GEA_STM32_STEM}_framework_qspi STATIC ${GEA_FRAMEWORK_QSPI_SOURCES})
    target_include_directories(${GEA_STM32_STEM}_framework_qspi PRIVATE $<TARGET_PROPERTY:${GEA_STM32_STEM}_app,INCLUDE_DIRECTORIES>)
    target_compile_definitions(${GEA_STM32_STEM}_framework_qspi PRIVATE $<TARGET_PROPERTY:${GEA_STM32_STEM}_app,COMPILE_DEFINITIONS>)
    target_compile_options(${GEA_STM32_STEM}_framework_qspi PRIVATE $<TARGET_PROPERTY:${GEA_STM32_STEM}_app,COMPILE_OPTIONS>)
    target_link_libraries(${GEA_STM32_STEM}_framework_qspi PRIVATE $<COMPILE_ONLY:${GEA_STM32_STEM}_board>)
    target_link_libraries(${GEA_STM32_STEM}_app PRIVATE $<LINK_LIBRARY:WHOLE_ARCHIVE,${GEA_STM32_STEM}_framework_qspi>)

    gea_stm32_link_image(${GEA_STM32_STEM}_app)
    gea_stm32_place_archive_in_qspi(${GEA_STM32_STEM}_app "${GEA_APP_ARCHIVE}")
    gea_stm32_place_archive_in_qspi(${GEA_STM32_STEM}_app "$<TARGET_FILE:${GEA_STM32_STEM}_framework_qspi>")
endif()
