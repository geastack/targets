# This component definition is shared by the two ESP32 e-paper targets. A
# board that includes this file sets GEA_EMBEDDED_TARGET_MAIN_DIR and the
# target-specific knobs below before including it.
if(NOT DEFINED GEA_EMBEDDED_TARGET_MAIN_DIR)
    set(GEA_EMBEDDED_TARGET_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}")
endif()

if(NOT DEFINED GEA_EMBEDDED_TARGET_CPP_BOARD)
    set(GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-s3-epaper-1.54")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_DISPLAY_WIDTH)
    set(GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 200)
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT)
    set(GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 200)
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH)
    set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH ${GEA_EMBEDDED_TARGET_DISPLAY_WIDTH})
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT)
    set(GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT ${GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT})
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO)
    set(GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO "1.0")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_AUDIO_SOURCE)
    set(GEA_EMBEDDED_TARGET_AUDIO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../esp32/chip_bindings/audio/es8311.cpp")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_EXTRA_SOURCES)
    set(GEA_EMBEDDED_TARGET_EXTRA_SOURCES "")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_DISPLAY_MONOCHROME)
    set(GEA_EMBEDDED_TARGET_DISPLAY_MONOCHROME 1)
endif()
# Accelerometer/gyroscope implementation. Boards with no IMU keep the default
# no-op file; a board that actually has one (PaperS3's BMI270) points this at
# its real driver instead of shipping a file named "stub" that is not one.
if(NOT DEFINED GEA_EMBEDDED_TARGET_IMU_SOURCE)
    set(GEA_EMBEDDED_TARGET_IMU_SOURCE "imu_stub.cpp")
endif()
# 1 = framebuffer stores panel-endian (byteswapped) RGB565 like the amoled
# targets, so SPI flushes skip the per-pixel swap. Epaper family packs to
# 1-bit itself and keeps native little-endian.
if(NOT DEFINED GEA_EMBEDDED_TARGET_PIXEL_PANEL_ENDIAN)
    set(GEA_EMBEDDED_TARGET_PIXEL_PANEL_ENDIAN 0)
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_EXTRA_REQUIRES)
    set(GEA_EMBEDDED_TARGET_EXTRA_REQUIRES "")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS)
    set(GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS "")
endif()
if(NOT DEFINED GEA_EMBEDDED_TARGET_LDFRAGMENTS)
    set(GEA_EMBEDDED_TARGET_LDFRAGMENTS "")
endif()

set(GEA_EMBEDDED_ROOT "${GEA_EMBEDDED_TARGET_MAIN_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)

# geastack split: resolve the gea framework via npm + the shared manifest.
# Exports GEA_CORE/GEA_COMPILER/GEA_PLUGIN/GEA_CHIPS/GEA_HOST,
# GEA_APP_ENTRY_SRC, and GEA_FW_CXX_SOURCES/GEA_FW_C_SOURCES/GEA_FW_INCLUDE_DIRS.
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")
include("${GEA_EMBEDDED_ROOT}/targets/cmake/geatsc_generated_archive.cmake")
if(NOT DEFINED GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE)
    set(GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE "${GEA_HOST}/audio_runtime.cpp")
endif()

# GEA_EMBEDDED_APP must be set explicitly in the real configure pass — no
# default. The build script `scripts/board` passes
# `-DGEA_EMBEDDED_APP=<id>` for every command, so this is invisible in
# normal use. Without an explicit value the previous default `tic-tac-toe`
# silently picked an arbitrary app whose features didn't match the user's
# actual selection, producing a REQUIRES list mismatched against the real
# pass.
#
# Script-mode (`cmake -P`, used by IDF's component_get_requirements.cmake)
# is a special case: it runs before the user's -D args reach the cache, so
# a hard error there would break component-requirement extraction
# entirely. We fall through to `tilt-breakout` in that pass because it is a
# gea app with a broad native binding surface.
set(GEA_EMBEDDED_APP "" CACHE STRING "Example app to build (must match a package.json gea.id) — required, no default")
if(CMAKE_SCRIPT_MODE_FILE AND "${GEA_EMBEDDED_APP}" STREQUAL "")
    set(GEA_EMBEDDED_APP "tilt-breakout")
elseif(NOT CMAKE_SCRIPT_MODE_FILE AND "${GEA_EMBEDDED_APP}" STREQUAL "")
    message(FATAL_ERROR "GEA_EMBEDDED_APP is required. Pass -DGEA_EMBEDDED_APP=<id> or use scripts/board which sets it for you (see package.json gea.id values for valid ids).")
endif()
set(GEA_EMBEDDED_HTTPS_SUPPORT "auto" CACHE STRING "Enable HTTPS fetch support: auto, on, or off")
set(GEA_EMBEDDED_CPP_BOARD "${GEA_EMBEDDED_TARGET_CPP_BOARD}" CACHE STRING "Board profile passed to geatsc for C++ target selection" FORCE)
# DPR 1.0 (logical px == physical 200x200): pixel fonts (Cozette) only render
# crisp, uniform stems when baked at an integer multiple of their native size,
# which requires baked-size == CSS-size. A non-integer DPR (1.5) would force
# fractional baked sizes and reintroduce uneven 1px/2px stems on this 1-bit panel.
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "${GEA_EMBEDDED_TARGET_CSS_DEVICE_PIXEL_RATIO}" CACHE STRING "Logical CSS px to physical pixel scale for this board" FORCE)

# Write a file only when its content would change. Preserves mtime when content matches,
# so downstream make/ninja rules don't see spurious changes on every reconfigure.
function(_gea_write_if_different FILE_PATH CONTENT)
    set(_should_write 1)
    if(EXISTS "${FILE_PATH}")
        file(READ "${FILE_PATH}" _existing)
        if("${_existing}" STREQUAL "${CONTENT}")
            set(_should_write 0)
        endif()
    endif()
    if(_should_write)
        file(WRITE "${FILE_PATH}" "${CONTENT}")
    endif()
endfunction()

# Enumerate files under DIR matching one of the source extensions / config filenames, pruning
# node_modules / dist / .vite at directory boundaries. CMake's file(GLOB_RECURSE) walks every file
# in the tree before filtering — for example dirs that's ~1000 stat calls per node_modules. find(1)
# prunes at the directory level, so configure stays under a second instead of half a minute.
function(_gea_find_sources OUT_VAR DIR)
    get_filename_component(_gea_find_dir "${DIR}" REALPATH)
    set(_gea_optional_prunes "")
    if(ARGC GREATER 2 AND "${ARGV2}" STREQUAL "PRUNE_VENDOR")
        list(APPEND _gea_optional_prunes -o -name vendor)
    endif()
    execute_process(
        COMMAND find -L "${_gea_find_dir}"
            -type d \( -name node_modules -o -name dist -o -name .vite -o -name build -o -name .build -o -name .scratch -o -name .test-tmp -o -name generated-output ${_gea_optional_prunes} \) -prune
            -o -type f \(
                -name "*.tsx" -o -name "*.ts" -o -name "*.jsx" -o -name "*.js"
                -o -name "*.css" -o -name "*.html"
                -o -name "package.json" -o -name "tsconfig.json" -o -name "vite.config.ts"
            \) -print
        OUTPUT_VARIABLE _output
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_output STREQUAL "")
        set(${OUT_VAR} "" PARENT_SCOPE)
    else()
        string(REPLACE "\n" ";" _list "${_output}")
        set(${OUT_VAR} "${_list}" PARENT_SCOPE)
    endif()
endfunction()

function(_gea_collect_app_sources OUT_VAR APP_DIR)
    _gea_find_sources(_src "${APP_DIR}")
    set(${OUT_VAR} "${_src}" PARENT_SCOPE)
endfunction()

# App image assets (PNG/JPG/GIF). generate-gea-embedded-assets.mjs embeds these as
# .rodata referenced by `<img src>`, so they MUST be on the geatsc_build DEPENDS
# list: otherwise resizing/replacing an image never re-runs codegen and the stale
# bytes ship (and an oversized PNG silently fails to decode on-device). Scoped to
# the app dir (not the framework) so configure stays fast.
function(_gea_collect_app_assets OUT_VAR APP_DIR)
    execute_process(
        COMMAND find -L "${APP_DIR}"
            -type d \( -name node_modules -o -name dist -o -name .vite -o -name build \) -prune
            -o -type f \(
                -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" -o -name "*.gif"
            \) -print
        OUTPUT_VARIABLE _output
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_output STREQUAL "")
        set(${OUT_VAR} "" PARENT_SCOPE)
    else()
        string(REPLACE "\n" ";" _list "${_output}")
        set(${OUT_VAR} "${_list}" PARENT_SCOPE)
    endif()
endfunction()

# Source files that affect Gea compiler -> geatsc output. Keep this scoped to
# the embedded framework, upstream Gea compiler, and @geatsc/plugin-gea.
_gea_find_sources(_GEA_COMPILER_DEP_SRCS "${GEA_CORE}/..")
_gea_find_sources(_GEA_UPSTREAM_GEA_SRCS "${GEA_COMPILER}" PRUNE_VENDOR)
_gea_find_sources(_GEATSC_GEA_PLUGIN_SRCS "${GEA_PLUGIN}")
if(CMAKE_SCRIPT_MODE_FILE)
    file(GLOB_RECURSE _GEATSC_GEA_PLUGIN_DIST_SRCS
        "${GEA_PLUGIN}/dist/*.js"
        "${GEA_PLUGIN}/dist/*.json"
    )
else()
    file(GLOB_RECURSE _GEATSC_GEA_PLUGIN_DIST_SRCS CONFIGURE_DEPENDS
        "${GEA_PLUGIN}/dist/*.js"
        "${GEA_PLUGIN}/dist/*.json"
    )
endif()
list(APPEND _GEA_COMPILER_DEP_SRCS
    "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs"
    "${GEA_CORE}/scripts/gea-embedded-compat-transform.mjs"
    "${GEA_CORE}/scripts/generate-gea-embedded-fonts.mjs"
    ${_GEA_UPSTREAM_GEA_SRCS}
    ${_GEATSC_GEA_PLUGIN_SRCS}
    ${_GEATSC_GEA_PLUGIN_DIST_SRCS}
)
list(REMOVE_DUPLICATES _GEA_COMPILER_DEP_SRCS)
get_property(_GEA_CONFIGURE_DEPS_REGISTERED GLOBAL PROPERTY GEA_CONFIGURE_DEPS_REGISTERED)
if(NOT CMAKE_SCRIPT_MODE_FILE AND _GEA_COMPILER_DEP_SRCS AND NOT _GEA_CONFIGURE_DEPS_REGISTERED)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_GEA_COMPILER_DEP_SRCS})
    set_property(GLOBAL PROPERTY GEA_CONFIGURE_DEPS_REGISTERED TRUE)
endif()

# Platform-universal ESP32 host facades (single source of truth).
include("${GEA_EMBEDDED_ROOT}/targets/esp32/host_sources.cmake")

file(GLOB _GEA_FONT_ASSETS
    "${GEA_EMBEDDED_ROOT}/assets/fonts/*.ttf"
    "${GEA_EMBEDDED_ROOT}/assets/fonts/*.otf"
    "${GEA_EMBEDDED_ROOT}/assets/fonts/*.woff"
    "${GEA_EMBEDDED_ROOT}/assets/fonts/*.woff2"
)

# App metadata (root;entry;runtime;native sources...) is resolved once by the
# gea CLI and handed to gea_framework.cmake as GEA_EMBEDDED_APP_META; the
# GEA_EMBEDDED_APP_DIR/ENTRY/RUNTIME/NATIVE_SOURCE_RELS variables come from there.

set(GEA_EMBEDDED_APP_NATIVE_SOURCES "")
# Expose each app-owned native source's dir (plus the app's `native/` dir) on the
# include path so the generated program's host-binding includes — e.g.
# "epub_archive.h" emitted for `gea::host::epub_archive` — resolve. Mirrors the
# amoled target (esp32-s3-touch-amoled-2.06/main/CMakeLists.txt).
set(GEA_EMBEDDED_APP_NATIVE_INCLUDE_DIRS "")
foreach(_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL IN LISTS GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS)
    list(APPEND GEA_EMBEDDED_APP_NATIVE_SOURCES "${GEA_EMBEDDED_APP_DIR}/${_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL}")
    get_filename_component(_GEA_EMBEDDED_APP_NATIVE_SOURCE_DIR "${GEA_EMBEDDED_APP_DIR}/${_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL}" DIRECTORY)
    list(APPEND GEA_EMBEDDED_APP_NATIVE_INCLUDE_DIRS "${_GEA_EMBEDDED_APP_NATIVE_SOURCE_DIR}")
endforeach()
if(GEA_EMBEDDED_APP_NATIVE_SOURCES)
    # Apps may keep native sources anywhere (e.g. vendored under src/); only
    # add the conventional native/ dir when it actually exists — ESP-IDF
    # rejects nonexistent include dirs.
    if(IS_DIRECTORY "${GEA_EMBEDDED_APP_DIR}/native")
        list(APPEND GEA_EMBEDDED_APP_NATIVE_INCLUDE_DIRS "${GEA_EMBEDDED_APP_DIR}/native")
    endif()
    list(REMOVE_DUPLICATES GEA_EMBEDDED_APP_NATIVE_INCLUDE_DIRS)
endif()

set(JS_APP_SRC "${GEA_EMBEDDED_APP_DIR}/${GEA_EMBEDDED_APP_ENTRY}")

# Each application's generated output gets its own build directory.
set(GENERATED_APP_DIR "${CMAKE_BINARY_DIR}/apps/${GEA_EMBEDDED_APP}")
file(MAKE_DIRECTORY "${GENERATED_APP_DIR}")
set(JS_APP "${GENERATED_APP_DIR}/dist/index.js")
# geatsc emits one C++ translation unit per source module and records the
# compile set in geatsc-sources.txt. A generated archive consumes that list at
# build time so CMake configure does not need to know module filenames up front.
set(GEATSC_SOURCE_LIST "${GENERATED_APP_DIR}/geatsc-sources.txt")
set(GEATSC_ARCHIVE "${GENERATED_APP_DIR}/libgea_active_geatsc.a")
set(GENERATED_FONT_CPP "${GENERATED_APP_DIR}/gea_embedded_font_generated.cpp")
set(GENERATED_FONT_H "${GENERATED_APP_DIR}/gea_embedded_font_generated.h")
set(GENERATED_ASSETS_CPP "${GENERATED_APP_DIR}/gea_embedded_assets_generated.cpp")
set(GENERATED_CONFIG_H "${GENERATED_APP_DIR}/gea_embedded_app_config.h")
set(GENERATED_INSTALLED_APPS_H "${CMAKE_BINARY_DIR}/gea_embedded_installed_apps_generated.h")
execute_process(
    COMMAND node
        "${GEA_CORE}/scripts/generate-installed-app-plan.mjs"
        "${GEA_EMBEDDED_TARGET_MAIN_DIR}/../partitions.csv"
        "${GENERATED_INSTALLED_APPS_H}"
    RESULT_VARIABLE GEA_EMBEDDED_INSTALLED_APPS_RESULT
)
if(NOT GEA_EMBEDDED_INSTALLED_APPS_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to generate installed app plan from partitions.csv")
endif()

# The app source owns app-level defaults. On a fresh configure, create only a minimal
# placeholder; the unified geatsc path can overwrite it with app-derived config
# later if a native define surface lands here.
if(NOT EXISTS "${GENERATED_CONFIG_H}")
    # Stub config header kept around because app_main.cpp still includes it.
    _gea_write_if_different("${GENERATED_CONFIG_H}" "#pragma once\n")
endif()
if(NOT EXISTS "${GENERATED_FONT_H}")
    _gea_write_if_different("${GENERATED_FONT_H}" [[
#pragma once
#define GEA_EMBEDDED_HAS_GENERATED_FONTS 1
#include "graphics/font.h"
#define GEA_EMBEDDED_FONT_COUNT 0
#define GEA_EMBEDDED_FONT_FAMILY_COUNT 0
]])
endif()
if(NOT EXISTS "${GENERATED_FONT_CPP}")
    _gea_write_if_different("${GENERATED_FONT_CPP}" [[
#include "gea_embedded_font_generated.h"
namespace gea::framework::graphics::generated {
void ensureLinked() {}
const RasterizedFontData *lookupFont(int fontId)
{
    (void)fontId;
    return nullptr;
}
const RasterizedFontData *lookupFontForFamily(int familyId, int sizePx)
{
    (void)familyId;
    (void)sizePx;
    return nullptr;
}
int lookupFontFamily(const char *family)
{
    (void)family;
    return -1;
}
}  // namespace gea::framework::graphics::generated
]])
endif()
# Stub asset TU on the SRCS list before geatsc_build overwrites it with the
# real (or empty) generated registry. Mirrors the font stub above.
if(NOT EXISTS "${GENERATED_ASSETS_CPP}")
    _gea_write_if_different("${GENERATED_ASSETS_CPP}" [[
// No embedded assets (stub overwritten by generate-gea-embedded-assets.mjs).
]])
endif()

# geatsc mode: generated app/user code is compiled from geatsc-sources.txt
# into a separate archive, then linked with the firmware-side Gea adapter and host bindings.
# Without app metadata (a configure driven by hand, or IDF's script-mode
# requirements pass before the CLI's -D values land) JS_APP_SRC is a bare
# directory; analyzing it would crash node with EISDIR and abort the
# configure with a misleading "has geatsc been built?" error. The normal
# pass refuses with the real reason; script mode carries on with an empty
# binding set, since that pass only extracts component requirements.
if(NOT EXISTS "${JS_APP_SRC}" OR IS_DIRECTORY "${JS_APP_SRC}")
    if(NOT CMAKE_SCRIPT_MODE_FILE)
        message(FATAL_ERROR "No app entry to build for '${GEA_EMBEDDED_APP}' (JS_APP_SRC='${JS_APP_SRC}'). Run this target through the gea CLI with an app: gea build --board <alias> --app <id>")
    endif()
    set(_GEA_EMBEDDED_ANALYZE_RAW "bindings=\nfeatures=")
    set(_GEA_EMBEDDED_ANALYZE_STATUS 0)
else()
execute_process(
    COMMAND node
        "${GEA_COMPILER}/dist/cli.js"
        analyze
        "${JS_APP_SRC}"
        --plugin
        "${GEA_PLUGIN}/dist/index.js"
    OUTPUT_VARIABLE _GEA_EMBEDDED_ANALYZE_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _GEA_EMBEDDED_ANALYZE_STATUS
)
endif()
if(NOT _GEA_EMBEDDED_ANALYZE_STATUS EQUAL 0)
    message(FATAL_ERROR "geatsc analyze failed for app '${GEA_EMBEDDED_APP}' — has packages/geatsc been built? (run `npm --prefix packages/geatsc run build`)")
endif()
# `geatsc analyze` outputs two lines: `bindings=<ids>` and `features=<ids>`.
string(REGEX MATCH "bindings=([^\n]*)" _ "${_GEA_EMBEDDED_ANALYZE_RAW}")
set(_GEA_EMBEDDED_HOST_BINDINGS "${CMAKE_MATCH_1}")
list(REMOVE_DUPLICATES _GEA_EMBEDDED_HOST_BINDINGS)
if("${_GEA_EMBEDDED_HOST_BINDINGS}" MATCHES "(^|;)screen($|;)")
    message(FATAL_ERROR "The gea screen host binding was removed; generated apps must target the embedded UI/canvas framework")
endif()
string(REGEX MATCH "features=([^\n]*)" _ "${_GEA_EMBEDDED_ANALYZE_RAW}")
set(_GEA_EMBEDDED_HOST_FEATURES "${CMAKE_MATCH_1}")

set(APP_GENERATED_SOURCES
    ${GEA_EMBEDDED_APP_NATIVE_SOURCES}
    "${GEA_APP_ENTRY_SRC}"
    ${GENERATED_FONT_CPP}
    ${GENERATED_ASSETS_CPP}
)
set(GEATSC_COMPILE_OPTIONS
    -Wno-unused-variable
    -Wno-return-type
    -Os
    ${GEA_XTENSA_LITERALS}  # -mtext-section-literals on xtensa, empty on riscv (gea_framework.cmake)
)
set(GEATSC_COMPILE_DEFINITIONS "")
if(NOT CMAKE_SCRIPT_MODE_FILE)
    set_source_files_properties(
        "${GEA_HOST}/host/rtc_esp.cpp"
        PROPERTIES COMPILE_OPTIONS "-Wno-unused-variable;-Os"
    )
endif()
set(GEA_EMBEDDED_HTTPS_REQUIRES "")
set(GEA_EMBEDDED_HTTPS_COMPILE_DEFINITIONS "")
set(GEA_EMBEDDED_BLE_COMPILE_DEFINITIONS "")
set(GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS "")

# The board command passes these after analyzing the active app plus every
# application. Direct idf.py builds fall back to the same analyzer data
# collected above, so linkage is capability-driven on both entry paths.
if(DEFINED ENV{GEA_EMBEDDED_CAPABILITY_NETWORK}
        AND NOT "$ENV{GEA_EMBEDDED_CAPABILITY_NETWORK}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_NETWORK "$ENV{GEA_EMBEDDED_CAPABILITY_NETWORK}")
elseif(NOT DEFINED GEA_EMBEDDED_CAPABILITY_NETWORK OR "${GEA_EMBEDDED_CAPABILITY_NETWORK}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_NETWORK 0)
    if("${_GEA_EMBEDDED_HOST_BINDINGS}" MATCHES "(^|;)(wifi|fetch|http|websocket|rtc)($|;)"
            OR "${_GEA_EMBEDDED_HOST_FEATURES}" MATCHES "(^|;)https($|;)")
        set(GEA_EMBEDDED_CAPABILITY_NETWORK 1)
    endif()
endif()
if(DEFINED ENV{GEA_EMBEDDED_CAPABILITY_BLE}
        AND NOT "$ENV{GEA_EMBEDDED_CAPABILITY_BLE}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_BLE "$ENV{GEA_EMBEDDED_CAPABILITY_BLE}")
elseif(NOT DEFINED GEA_EMBEDDED_CAPABILITY_BLE OR "${GEA_EMBEDDED_CAPABILITY_BLE}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_BLE 0)
    if("${_GEA_EMBEDDED_HOST_BINDINGS}" MATCHES "(^|;)ble($|;)")
        set(GEA_EMBEDDED_CAPABILITY_BLE 1)
    endif()
endif()
if(DEFINED ENV{GEA_EMBEDDED_CAPABILITY_AUDIO}
        AND NOT "$ENV{GEA_EMBEDDED_CAPABILITY_AUDIO}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_AUDIO "$ENV{GEA_EMBEDDED_CAPABILITY_AUDIO}")
elseif(NOT DEFINED GEA_EMBEDDED_CAPABILITY_AUDIO OR "${GEA_EMBEDDED_CAPABILITY_AUDIO}" STREQUAL "")
    set(GEA_EMBEDDED_CAPABILITY_AUDIO 0)
    if("${_GEA_EMBEDDED_HOST_BINDINGS}" MATCHES "(^|;)audio($|;)")
        set(GEA_EMBEDDED_CAPABILITY_AUDIO 1)
    endif()
endif()

set(GEA_EMBEDDED_NETWORK_SOURCES
    "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/diagnostics_transport_offline.cpp")
set(GEA_EMBEDDED_NETWORK_REQUIRES "")
set(GEA_EMBEDDED_RTC_SOURCES "")
if(GEA_EMBEDDED_CAPABILITY_NETWORK)
    set(GEA_EMBEDDED_NETWORK_SOURCES
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/diagnostics_transport.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/wifi.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/ota.cpp")
    list(APPEND GEA_EMBEDDED_RTC_SOURCES "${GEA_HOST}/host/rtc_esp.cpp")
    list(APPEND GEA_EMBEDDED_NETWORK_REQUIRES
        lwip esp_wifi esp_http_server esp_http_client esp-tls app_update esp_netif esp_event)
else()
    list(APPEND GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS
        GEA_EMBEDDED_WIFI_DISABLED=1
        GEA_EMBEDDED_NETWORK_SERVICES_DISABLED=1
        GEA_EMBEDDED_OTA_DISABLED=1)
endif()

set(GEA_EMBEDDED_BLE_SOURCES "")
set(GEA_EMBEDDED_BLE_REQUIRES "")
if(GEA_EMBEDDED_CAPABILITY_BLE)
    list(APPEND GEA_EMBEDDED_BLE_SOURCES
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/ble_hid.cpp")
    list(APPEND GEA_EMBEDDED_BLE_REQUIRES bt)
    list(APPEND GEA_EMBEDDED_BLE_COMPILE_DEFINITIONS GEA_EMBEDDED_APP_USES_BLE=1)
else()
    list(APPEND GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS GEA_EMBEDDED_BLE_DISABLED=1)
endif()

set(GEA_EMBEDDED_AUDIO_SOURCES "")
set(GEA_EMBEDDED_AUDIO_REQUIRES "")
if(GEA_EMBEDDED_CAPABILITY_AUDIO)
    list(APPEND GEA_EMBEDDED_AUDIO_SOURCES
        "${GEA_EMBEDDED_TARGET_AUDIO_SOURCE}"
        "${GEA_EMBEDDED_TARGET_AUDIO_RUNTIME_SOURCE}")
    list(APPEND GEA_EMBEDDED_AUDIO_REQUIRES esp_driver_i2s esp_codec_dev)
    list(APPEND GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS GEA_EMBEDDED_APP_USES_AUDIO=1)
else()
    # Framework surfaces (HTMLAudioElement, sound attributes, device control)
    # reference the AudioSystem backend unconditionally; satisfy those symbols
    # with no-op stubs when the app carries no audio capability.
    list(APPEND GEA_EMBEDDED_AUDIO_SOURCES
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/audio_runtime_stub.cpp")
    list(APPEND GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS GEA_EMBEDDED_AUDIO_UNUSED=1)
endif()

message(STATUS "Gea app capabilities: network=${GEA_EMBEDDED_CAPABILITY_NETWORK} ble=${GEA_EMBEDDED_CAPABILITY_BLE} audio=${GEA_EMBEDDED_CAPABILITY_AUDIO}")

# `_GEA_EMBEDDED_HOST_FEATURES` is populated by `geatsc analyze` for the
# application.
#
# The user-facing `GEA_EMBEDDED_HTTPS_SUPPORT` cache option lets the build
# override auto-detection: `on` forces HTTPS linkage even when no
# `https://` URL appears in source, `off` skips it even when one does.
# `auto` (default) trusts analyze's verdict.
string(TOLOWER "${GEA_EMBEDDED_HTTPS_SUPPORT}" _GEA_EMBEDDED_HTTPS_SUPPORT_MODE)
set(_GEA_EMBEDDED_ENABLE_HTTPS 0)
if(_GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "auto")
    if("${_GEA_EMBEDDED_HOST_FEATURES}" MATCHES "(^|;)https($|;)")
        set(_GEA_EMBEDDED_ENABLE_HTTPS 1)
    endif()
elseif(_GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "on"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "true"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "yes"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "1")
    set(_GEA_EMBEDDED_ENABLE_HTTPS 1)
elseif(_GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "off"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "false"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "no"
        OR _GEA_EMBEDDED_HTTPS_SUPPORT_MODE STREQUAL "0")
    set(_GEA_EMBEDDED_ENABLE_HTTPS 0)
else()
    message(FATAL_ERROR "GEA_EMBEDDED_HTTPS_SUPPORT must be auto, on, or off")
endif()

if(_GEA_EMBEDDED_ENABLE_HTTPS)
    list(APPEND GEA_EMBEDDED_HTTPS_COMPILE_DEFINITIONS GEA_EMBEDDED_ENABLE_HTTPS=1)
    # `mbedtls` lands in REQUIRES iff the active app actually needs HTTPS.
    # The historical reason for unconditionally listing it (script-mode
    # discrepancy with `GEA_EMBEDDED_APP=tic-tac-toe`) is gone now that
    # script-mode falls through to a feature-rich gea placeholder. The
    # condition here is the actual signal: an app with no https:// in source
    # genuinely doesn't need mbedtls, and `--gc-sections` would strip it
    # anyway.
    list(APPEND GEA_EMBEDDED_HTTPS_REQUIRES mbedtls)
endif()

idf_component_register(
    SRCS
        "app_main.cpp"
        "buttons.cpp"
        "display.cpp"
        "i2c.cpp"
        "${GEA_EMBEDDED_TARGET_IMU_SOURCE}"
        "launcher_button.cpp"
        "power.cpp"
        "sdcard_mount.cpp"
        "touch.cpp"
        ${GEA_EMBEDDED_AUDIO_SOURCES}
        ${GEA_EMBEDDED_TARGET_EXTRA_SOURCES}
        "${GEA_EMBEDDED_ROOT}/targets/esp32/apps/app_manager.cpp"
        ${GEA_EMBEDDED_NETWORK_SOURCES}
        # @geastack framework C/C++ sources — single source of truth: @geastack/core/gea_sources.sh
        # (resolved via node_modules, shared with the web build).
        ${GEA_FW_APP_SENSITIVE_SOURCES}
        # esp32-specific host shims (generic host + services come from the manifest):
        ${GEA_EMBEDDED_RTC_SOURCES}
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/app_runner.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/app_state.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/battery_service.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/device_control.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/frame_scheduler.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/memory.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/storage_service.cpp"
        ${GEA_EMBEDDED_BLE_SOURCES}
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/heap.cpp"
        ${APP_GENERATED_SOURCES}
    INCLUDE_DIRS
        "${GEA_EMBEDDED_TARGET_MAIN_DIR}/include"
        "${GEA_EMBEDDED_ROOT}/targets/esp32"
        ${GEA_FW_INCLUDE_DIRS}
        "${GEA_CHIPS}"
        ${GEA_EMBEDDED_APP_NATIVE_INCLUDE_DIRS}
        ${GENERATED_APP_DIR}
        ${CMAKE_BINARY_DIR}
    REQUIRES
        gea_framework
        esp_system
        # app_manager reads the running image descriptor independently of OTA.
        esp_app_format
        esp_timer
        freertos
        ${GEA_EMBEDDED_NETWORK_REQUIRES}
        ${GEA_EMBEDDED_HTTPS_REQUIRES}
        esp_mm
        nvs_flash
        esp_lcd
        driver
        esp_driver_i2c
        esp_driver_usb_serial_jtag
        esp_driver_sdmmc
        sdmmc
        fatfs
        ${GEA_EMBEDDED_AUDIO_REQUIRES}
        ${GEA_EMBEDDED_TARGET_EXTRA_REQUIRES}
        ${GEA_EMBEDDED_BLE_REQUIRES}
    LDFRAGMENTS
        ${GEA_EMBEDDED_TARGET_LDFRAGMENTS}
)

if(TARGET idf::fatfs)
    idf_component_get_property(_GEA_FATFS_LIB fatfs COMPONENT_LIB)
    target_compile_options(${_GEA_FATFS_LIB} PRIVATE
        "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/fatfs_ffconf.h")
endif()

set(GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE 0)
if(DEFINED ENV{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG})
    set(GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE "$ENV{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG}")
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    GEA_EMBEDDED_DISPLAY_WIDTH=${GEA_EMBEDDED_TARGET_DISPLAY_WIDTH}
    GEA_EMBEDDED_DISPLAY_HEIGHT=${GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT}
    GEA_EMBEDDED_DISPLAY_NATIVE_WIDTH=${GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_WIDTH}
    GEA_EMBEDDED_DISPLAY_NATIVE_HEIGHT=${GEA_EMBEDDED_TARGET_DISPLAY_NATIVE_HEIGHT}
    GEA_EMBEDDED_FLUSH_ANCHOR_LEFT_PARTIAL_RECTS=0
    GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=${GEA_EMBEDDED_TARGET_PIXEL_PANEL_ENDIAN}
    # 1-bit panels (epaper family default): stylesheets can theme via
    # `@media (monochrome)`. Color boards reusing this definition set 0.
    GEA_EMBEDDED_DISPLAY_MONOCHROME=${GEA_EMBEDDED_TARGET_DISPLAY_MONOCHROME}
    # Board id as a C string for app native sources (e.g. the diagnostics
    # deviceInfo facade keys its capability catalog off deviceId()). Without this
    # the facade's #ifndef fallback returns "unknown" and every subsystem shows
    # up absent/N/A.
    GEA_EMBEDDED_CPP_BOARD=\"${GEA_EMBEDDED_CPP_BOARD}\"
    GEA_CPP_ENABLE_REGEX=0
    GEA_CPP_USE_TO_CHARS_DOUBLE=0
    GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG=${GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE}
    GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
    GEA_EMBEDDED_HAS_GENERATED_FONTS=1
    GEA_EMBEDDED_APP_FRAME_TASK_CORE=0
    GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES=0
    GEA_EMBEDDED_APP_FRAME_TASK_PRIORITY=23
    ${GEA_EMBEDDED_TARGET_COMPILE_DEFINITIONS}
    ${GEA_EMBEDDED_CAPABILITY_COMPILE_DEFINITIONS}
    ${GEA_EMBEDDED_BLE_COMPILE_DEFINITIONS}
    ${GEA_EMBEDDED_HTTPS_COMPILE_DEFINITIONS}
)

# Production lockdown: gated by CONFIG_GEA_EMBEDDED_PRODUCTION_LOCKDOWN (sdkconfig.defaults).
# When enabled, adds defines that compile out perf/heap logs and disable diagnostics server.
if(CONFIG_GEA_EMBEDDED_PRODUCTION_LOCKDOWN)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LOG=0
        GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG=0
        GEA_FRAME_PERF_LOG=0
        GEA_EMBEDDED_DIAGNOSTICS_DISABLED=1
    )
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE
    ${GEA_XTENSA_LITERALS}  # -mtext-section-literals on xtensa, empty on riscv (gea_framework.cmake)
    -Wno-unused-parameter
    -Wno-sign-compare
    -Wno-missing-field-initializers
    -Wno-implicit-fallthrough
    $<$<COMPILE_LANGUAGE:C>:-Wno-old-style-declaration>
    -Wno-misleading-indentation
    -Wno-type-limits
    -Wno-maybe-uninitialized
    -Wno-calloc-transposed-args
    -Wno-unused-function
    -Wno-clobbered
)

# --- App compilation: vite-plugin-gea -> geatsc ---

_gea_collect_app_sources(_ACTIVE_APP_SRCS "${GEA_EMBEDDED_APP_DIR}")
_gea_collect_app_assets(_ACTIVE_APP_ASSETS "${GEA_EMBEDDED_APP_DIR}")
# The Gea compiler emits JS and IR, then geatsc emits C++. We use a stamp file as
# the canonical OUTPUT because geatsc now writes a source manifest plus multiple
# translation units. The generated archive consumes geatsc-sources.txt after this
# command has completed.
set(_GEATSC_BIN "${GEA_COMPILER}/dist/cli.js")
set(_GEATSC_BUILD_STAMP "${GENERATED_APP_DIR}/geatsc_build.stamp")
set(_GEA_VITE_GEATSC_SCRIPT "${GEA_CORE}/scripts/build-gea-vite-geatsc.mjs")
file(GLOB_RECURSE _GEATSC_DIST_SRCS CONFIGURE_DEPENDS
    "${GEA_COMPILER}/dist/*.js"
    "${GEA_COMPILER}/dist/targets/cpp/runtime/*.cpp"
    "${GEA_PLUGIN}/dist/*.js"
)
add_custom_command(
    OUTPUT ${_GEATSC_BUILD_STAMP}
    COMMAND node ${_GEA_VITE_GEATSC_SCRIPT}
            --app-dir ${GEA_EMBEDDED_APP_DIR}
            --entry ${GEA_EMBEDDED_APP_ENTRY}
            --cpp-board "${GEA_EMBEDDED_CPP_BOARD}"
            --out-dir ${GENERATED_APP_DIR}
            --geatsc-bin ${_GEATSC_BIN}
            --geatsc-gea-plugin "${GEA_PLUGIN}/dist/index.js"
            --font-device-pixel-ratio ${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
            --pixel-panel-endian ${GEA_EMBEDDED_TARGET_PIXEL_PANEL_ENDIAN}
            --gea-ir-backend
            # gea.compilerPlugins: an app's own geatsc plugins, e.g. the one that
            # declares its native host-function boundary. Empty for most apps.
            ${GEA_EMBEDDED_APP_GEATSC_PLUGIN_ARGS}
    COMMAND ${CMAKE_COMMAND} -E touch ${_GEATSC_BUILD_STAMP}
    DEPENDS
        ${GEA_EMBEDDED_APP_GEATSC_PLUGINS}
        ${_ACTIVE_APP_SRCS}
        ${_ACTIVE_APP_ASSETS}
        ${_GEATSC_BIN}
        ${_GEATSC_DIST_SRCS}
        ${_GEA_COMPILER_DEP_SRCS}
        ${_GEA_FONT_ASSETS}
    BYPRODUCTS
        "${GEATSC_SOURCE_LIST}"
        "${GENERATED_APP_DIR}/entry.cpp"
        "${GENERATED_APP_DIR}/generated_support.hpp"
        "${GENERATED_APP_DIR}/runtime.cpp"
        "${GENERATED_APP_DIR}/runtime_pch.h"
        ${GENERATED_CONFIG_H}
        ${GENERATED_FONT_CPP}
        ${GENERATED_FONT_H}
        ${GENERATED_ASSETS_CPP}
        "${GENERATED_APP_DIR}/gea-vite-bundle.js"
    COMMENT "Compiling ${GEA_EMBEDDED_APP}: TSX -> JS via vite-plugin-gea -> C++ via geatsc"
    VERBATIM
)
# The generated archive already depends on the stamp. Keeping this helper
# target out of ALL prevents Make from launching two copies of Vite/geatsc
# against the same output directory when both branches are scheduled in
# parallel.
add_custom_target(geatsc_build DEPENDS ${_GEATSC_BUILD_STAMP})
gea_geatsc_add_generated_archive(
    TARGET ${COMPONENT_LIB}
    NAME geatsc_active_archive
    SOURCE_LIST "${GEATSC_SOURCE_LIST}"
    OUTPUT "${GEATSC_ARCHIVE}"
    DEPENDS ${_GEATSC_BUILD_STAMP}
    COMPILE_OPTIONS ${GEATSC_COMPILE_OPTIONS}
    COMPILE_DEFINITIONS ${GEATSC_COMPILE_DEFINITIONS}
)

gea_framework_inherit_build_settings()
