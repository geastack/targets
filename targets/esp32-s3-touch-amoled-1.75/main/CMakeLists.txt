set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)

# geastack split: resolve the gea framework via npm + the shared manifest.
# Exports GEA_CORE/GEA_COMPILER/GEA_PLUGIN/GEA_CHIPS/GEA_HOST,
# GEA_APP_ENTRY_SRC, and GEA_FW_CXX_SOURCES/GEA_FW_C_SOURCES/GEA_FW_INCLUDE_DIRS.
include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")
include("${GEA_EMBEDDED_ROOT}/targets/cmake/geatsc_generated_archive.cmake")

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
set(GEA_EMBEDDED_CPP_BOARD "esp32-s3-touch-amoled-1.75" CACHE STRING "Board profile passed to geatsc for C++ target selection")
set(GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO "1.5" CACHE STRING "Logical CSS px to physical pixel scale for this board")

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

include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_find_sources.cmake")

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
foreach(_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL IN LISTS GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS)
    list(APPEND GEA_EMBEDDED_APP_NATIVE_SOURCES "${GEA_EMBEDDED_APP_DIR}/${_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL}")
endforeach()

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
        "${CMAKE_CURRENT_LIST_DIR}/../partitions.csv"
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
    -mtext-section-literals
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
if("${_GEA_EMBEDDED_HOST_BINDINGS}" MATCHES "(^|;)ble($|;)")
    list(APPEND GEA_EMBEDDED_BLE_COMPILE_DEFINITIONS GEA_EMBEDDED_APP_USES_BLE=1)
endif()
# Note: BLE driver registration + BLE controller preinit are forced from
# `app_main.cpp` and `bluetooth_service.cpp` regardless of the macro above,
# because `geatsc analyze` misses BLE usage routed through the internal
# `runtime-surface` re-export (as `gea-embedded`'s Settings panel does).
# The macro is kept narrow because `app_runner.cpp` uses it to decide
# whether `gea_init` is allocated in SPIRAM (when undefined) or internal
# DRAM (when defined). With 64 KB gea_init stack, forcing internal DRAM
# fails — only 32 KB largest contiguous block remains after wifi/diag init.
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
        "i2c.cpp"
        "launcher_button.cpp"
        "touch.cpp"
        "${GEA_CHIPS}/displays/co5300/co5300.cpp"
        "${GEA_CHIPS}/power/axp2101/axp2101.cpp"
        "${GEA_CHIPS}/imu/qmi8658/qmi8658.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/display.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/displays/co5300.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/power/axp2101.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/touch/cst9217.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/gps/lc76g.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/audio/es8311.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/chip_bindings/imu/qmi8658.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/apps/app_manager.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/diagnostics_transport.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/wifi.cpp"
        # @geastack framework C/C++ sources — single source of truth: @geastack/core/gea_sources.sh
        # (resolved via node_modules, shared with the web build).
        ${GEA_FW_APP_SENSITIVE_SOURCES}
        # esp32-specific host shims (generic host + services come from the manifest):
        "${GEA_HOST}/host/rtc_esp.cpp"
        "${GEA_HOST}/audio_runtime.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/app_runner.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/app_state.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/battery_service.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/device_control.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/frame_scheduler.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/memory.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/ota.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/storage_service.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/connectivity/ble_hid.cpp"
        "${GEA_EMBEDDED_ROOT}/targets/esp32/services/heap.cpp"
        ${APP_GENERATED_SOURCES}
    INCLUDE_DIRS
        "include"
        "${GEA_EMBEDDED_ROOT}/targets/esp32"
        ${GEA_FW_INCLUDE_DIRS}
        "${GEA_CHIPS}"
        ${GENERATED_APP_DIR}
        ${CMAKE_BINARY_DIR}
    REQUIRES
        gea_framework
        esp_system
        esp_timer
        freertos
        lwip
        esp_wifi
        esp_http_server
        esp_http_client
        ${GEA_EMBEDDED_HTTPS_REQUIRES}
        app_update
        esp_mm
        esp_netif
        esp_event
        nvs_flash
        esp_lcd
        driver
        esp_driver_i2c
        esp_driver_usb_serial_jtag
        esp_driver_uart
        esp_driver_i2s
        esp_codec_dev
        bt
    LDFRAGMENTS
)

set(GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE 0)
if(DEFINED ENV{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG})
    set(GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE "$ENV{GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG}")
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=1
    GEA_CPP_ENABLE_REGEX=0
    GEA_CPP_USE_TO_CHARS_DOUBLE=0
    GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG=${GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG_VALUE}
    GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO=${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
    GEA_EMBEDDED_DISPLAY_WIDTH=466
    GEA_EMBEDDED_DISPLAY_HEIGHT=466
    GEA_EMBEDDED_HAS_GENERATED_FONTS=1
    GEA_EMBEDDED_WIFI_EARLY_CONNECT=1
    GEA_EMBEDDED_CO5300_PANEL_X_GAP=6
    GEA_EMBEDDED_CO5300_PANEL_Y_GAP=0
    GEA_EMBEDDED_CO5300_WAVESHARE_175_INIT=1
    # Pin the frame ticker to core 0, where app_main/main_task also runs the
    # runtime/render event loop. That keeps render submission on the same core as
    # the display mutex + SPI/DMA-completion path and avoids the old per-boot
    # cross-core flush wait lottery.
    GEA_EMBEDDED_APP_FRAME_TASK_CORE=0
    GEA_EMBEDDED_DISPLAY_CO5300_FRAMEBUFFER_STREAM_FLUSH=1
    # Prototype: fuse replay into the flush (rasterize dirty chunks straight into the
    # DMA buffer, no PSRAM framebuffer copy, replay overlapped with DMA).
    GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH=1
    # Golden (e6614f6a, 62fps) used no partial-flush edge pad and showed no smudge.
    # The 2px all-sides pad inflates every per-rect ball flush window (~24/frame:
    # 15k -> 24k px). Drop it; if the CO5300 left-edge smudge reappears it's a
    # separate regression to fix at its source, not by padding every flush.
    GEA_EMBEDDED_DISPLAY_PARTIAL_FLUSH_EDGE_PAD_PX=0
    # QSPI display SPI clock. 80 MHz is the panel's proven rate and the source
    # default (display.cpp). Pinned explicitly here because the build had drifted
    # to an effective 60 MHz (no CMake/cache/source basis — a stale configure
    # artifact). A full-screen flush is 466x466x2B = 434 KB, bandwidth-bound over
    # the 4 QSPI lanes, so the clock sets flush time linearly: 80 MHz (~40 MB/s,
    # ~10.3 ms) vs 60 MHz (~30 MB/s, ~13.7 ms). Dominates full-screen canvas apps.
    GEA_EMBEDDED_DISPLAY_QSPI_PCLK_HZ=80000000
    # Direct PSRAM->SPI framebuffer flush: DISABLED — we use the
    # PSRAM->internal-staging->SPI path instead. Exhaustively tested (PSRAM at
    # 80/60/40/20 MHz, GDMA burst 32/64B, and PSRAM overclocked to 120 MHz): the
    # GDMA can't keep the 64-byte SPI TX FIFO fed straight from octal PSRAM for a
    # full-frame stream, so it underflows, and the underflow recovery hangs the
    # frame task. Staging is the correct, stable 60 fps path on this board.
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_FLUSH=0
    GEA_EMBEDDED_DISPLAY_CO5300_STREAM_PACE_CHUNKS=4
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STREAM_PACE_CHUNKS=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_RAMWRC_CONTINUE=1
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_CS_HELD_STREAM=1
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_MAX_TRANSFER_BYTES=32256
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STATIC_TX_PROBE_IDLE_FLUSH=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_EDGE_GUARD_BYTES=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_STAGED_PREFIX_ROWS=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_ALIGN_TRANSFERS=1
    # Real direct-PSRAM: packed stride (466 px, 932 B/row) keeps the framebuffer
    # contiguous so flushFramebufferRectDirect takes the directBytes branch and
    # GDMA reads PSRAM with no internal-RAM staging. CS_HELD_STREAM=1 above keeps
    # the SPI tx open across the whole frame so the touch task can't yield
    # between segments and starve the FIFO. REQUIRE_LENGTH_ALIGNMENT=0 lets the
    # final 6-row tail also flow direct from PSRAM (vs being staged through an
    # internal-RAM buffer). The GDMA hardware accepts non-64-aligned descriptor
    # tails; the matching esp_cache_msync uses ESP_CACHE_MSYNC_FLAG_UNALIGNED.
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_REQUIRE_LENGTH_ALIGNMENT=0
    GEA_EMBEDDED_DISPLAY_SOFTWARE_SCROLL_PADDED_STRIDE=0
    GEA_EMBEDDED_DISPLAY_DIRECT_PSRAM_PADDED_ROW_STREAM=0
    # Only relevant when DIRECT_PSRAM_FLUSH=1 (disabled above). Left at 0.
    GEA_EMBEDDED_CO5300_SPI_MANUAL_DMA_ALIGN_FLAG=0
    # Was =1 from an old display-pacing experiment (f06ad9724) that disabled the
    # then-broken queueFrameEvent catch-up. =0 re-enables catch-up, now driven by
    # the work-based, event-loop graceful-overrun path (frame_scheduler/runtime).
    GEA_EMBEDDED_FRAME_SCHEDULER_DROP_CATCHUP_FRAMES=0
    GEA_EMBEDDED_APP_FRAME_TASK_PRIORITY=23
    # Compact scheduler perf line (gea_esp32_frame perf-lite) for cross-checking
    # the app-level gea.perf window without the full phase dump's serial cost.
    GEA_EMBEDDED_FRAME_SCHEDULER_PERF_LITE=1
    # 2-core replay split: defaults (96 rows / 64k px) only engage at the largest
    # dirty phases of an animated-transform scene; lower so mid-size regions split
    # too. Measured on css-3d-cube (glassy): median 52.3 -> 59.2 fps, min 44 -> 49.
    GEA_EMBEDDED_RENDER_PARALLEL_MIN_ROWS=48
    GEA_EMBEDDED_RENDER_PARALLEL_MIN_PIXELS=24000
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
            --font-device-pixel-ratio ${GEA_EMBEDDED_CSS_DEVICE_PIXEL_RATIO}
            --pixel-panel-endian 1
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
add_custom_target(geatsc_build ALL DEPENDS ${_GEATSC_BUILD_STAMP})
gea_geatsc_add_generated_archive(
    TARGET ${COMPONENT_LIB}
    NAME geatsc_active_archive
    SOURCE_LIST "${GEATSC_SOURCE_LIST}"
    OUTPUT "${GEATSC_ARCHIVE}"
    DEPENDS ${_GEATSC_BUILD_STAMP}
    COMPILE_OPTIONS ${GEATSC_COMPILE_OPTIONS}
    COMPILE_DEFINITIONS ${GEATSC_COMPILE_DEFINITIONS}
)
add_dependencies(geatsc_active_archive geatsc_build)

gea_framework_inherit_build_settings()
