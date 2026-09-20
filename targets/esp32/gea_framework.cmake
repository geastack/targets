# Shared geastack framework resolution for every esp32 board target.
#
# Requires GEA_EMBEDDED_ROOT to be set by the includer (the board's main
# CMakeLists, = the targets repo root). Resolves the gea framework through
# npm: `targets` is a native npm package whose deps (@geastack/core,
# @geastack/compiler, @geastack/geatsc-plugin-gea) are installed there. The always-linked framework C/C++ source set + include
# roots come from the shared manifest in `@geastack/core` (single source of
# truth, shared with the web build), resolved relative to @geastack/core.
#
# Exports for the board CMakeLists:
#   GEA_CORE / GEA_COMPILER / GEA_PLUGIN / GEA_CHIPS / GEA_ENGINE / GEA_HOST
#                            — package dirs
#   GEA_APP_ENTRY_SRC        — gea_app_entry.cpp (glob-resolved; survives folds)
#   GEA_FW_CXX_SOURCES / GEA_FW_C_SOURCES / GEA_FW_INCLUDE_DIRS — framework set

# ESP-IDF's component-requirements pass does not reliably preserve command-line
# cache values from an app-specific build directory. The CLI exports the same
# values so every configure phase selects the current project app.
if(DEFINED ENV{GEA_EMBEDDED_APP} AND NOT "$ENV{GEA_EMBEDDED_APP}" STREQUAL "")
    set(GEA_EMBEDDED_APP "$ENV{GEA_EMBEDDED_APP}" CACHE STRING "Gea app id" FORCE)
endif()
if(DEFINED ENV{GEA_EMBEDDED_APP_META} AND NOT "$ENV{GEA_EMBEDDED_APP_META}" STREQUAL "")
    set(GEA_EMBEDDED_APP_META "$ENV{GEA_EMBEDDED_APP_META}" CACHE STRING "Gea app metadata: root;entry;runtime;nativeSources..." FORCE)
endif()
# An app may also contribute preprocessor defines and ESP-IDF linker fragments.
# They travel in their own variables because the meta line above is positional
# and already ends in a variable-length native-source list.
if(DEFINED ENV{GEA_EMBEDDED_APP_DEFINES} AND NOT "$ENV{GEA_EMBEDDED_APP_DEFINES}" STREQUAL "")
    set(GEA_EMBEDDED_APP_DEFINES "$ENV{GEA_EMBEDDED_APP_DEFINES}" CACHE STRING "Gea app compile definitions" FORCE)
endif()
if(DEFINED ENV{GEA_EMBEDDED_APP_LDFRAGMENTS} AND NOT "$ENV{GEA_EMBEDDED_APP_LDFRAGMENTS}" STREQUAL "")
    set(GEA_EMBEDDED_APP_LDFRAGMENTS "$ENV{GEA_EMBEDDED_APP_LDFRAGMENTS}" CACHE STRING "Gea app linker fragment files" FORCE)
endif()
foreach(_GEA_APP_VAR GEA_EMBEDDED_APP_LINK_OPTIONS GEA_EMBEDDED_APP_EMBED_FILES GEA_EMBEDDED_APP_PARTITION_DATA
                     GEA_EMBEDDED_APP_COMPONENT_REQUIRES GEA_EMBEDDED_APP_GEATSC_PLUGINS)
    if(DEFINED ENV{${_GEA_APP_VAR}} AND NOT "$ENV{${_GEA_APP_VAR}}" STREQUAL "")
        set(${_GEA_APP_VAR} "$ENV{${_GEA_APP_VAR}}" CACHE STRING "Gea app native build input" FORCE)
    endif()
endforeach()
# gea.compilerPlugins, as the repeated flag the geatsc driver takes.
set(GEA_EMBEDDED_APP_GEATSC_PLUGIN_ARGS "")
foreach(_GEA_APP_GEATSC_PLUGIN IN LISTS GEA_EMBEDDED_APP_GEATSC_PLUGINS)
    if(NOT EXISTS "${_GEA_APP_GEATSC_PLUGIN}")
        message(FATAL_ERROR "gea.compilerPlugins entry does not exist: ${_GEA_APP_GEATSC_PLUGIN}")
    endif()
    list(APPEND GEA_EMBEDDED_APP_GEATSC_PLUGIN_ARGS --extra-geatsc-plugin "${_GEA_APP_GEATSC_PLUGIN}")
endforeach()
# The CLI inspects the app's package.json once (gea apps inspect --format cmake)
# and passes the result here; CMake never spawns node to read manifests.
if(GEA_EMBEDDED_APP)
    if(NOT GEA_EMBEDDED_APP_META)
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META is unset for app '${GEA_EMBEDDED_APP}'. Run this target through the gea CLI: gea build --board <alias> --app ${GEA_EMBEDDED_APP}")
    endif()
    set(GEA_EMBEDDED_APP_META_LIST "${GEA_EMBEDDED_APP_META}")
    list(LENGTH GEA_EMBEDDED_APP_META_LIST GEA_EMBEDDED_APP_META_LENGTH)
    if(GEA_EMBEDDED_APP_META_LENGTH LESS 3)
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META must be root;entry;runtime[;native sources...] (got '${GEA_EMBEDDED_APP_META}')")
    endif()
    # Element 0 is the app's absolute directory.
    list(GET GEA_EMBEDDED_APP_META_LIST 0 GEA_EMBEDDED_APP_DIR)
    list(GET GEA_EMBEDDED_APP_META_LIST 1 GEA_EMBEDDED_APP_ENTRY)
    list(GET GEA_EMBEDDED_APP_META_LIST 2 GEA_EMBEDDED_APP_RUNTIME)
    set(GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS "")
    if(GEA_EMBEDDED_APP_META_LENGTH GREATER 3)
        list(SUBLIST GEA_EMBEDDED_APP_META_LIST 3 -1 GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS)
    endif()
    if(NOT IS_ABSOLUTE "${GEA_EMBEDDED_APP_DIR}")
        message(FATAL_ERROR "GEA_EMBEDDED_APP_META must start with the app's absolute directory (got '${GEA_EMBEDDED_APP_DIR}')")
    endif()
endif()

foreach(_GEA_REQUIRED_ENV GEA_CORE_DIR GEA_COMPILER_DIR GEA_PLUGIN_DIR GEA_ENGINE_DIR GEA_HOST_DIR GEA_CHIPS_DIR GEA_ELEMENTS_DIR GEA_GEAOS_PACKAGE_DIR)
    if(NOT DEFINED ENV{${_GEA_REQUIRED_ENV}} OR "$ENV{${_GEA_REQUIRED_ENV}}" STREQUAL "")
        message(FATAL_ERROR "${_GEA_REQUIRED_ENV} is unset. Run this target through the gea CLI so npm packages are resolved first.")
    endif()
endforeach()
get_filename_component(GEA_CORE "$ENV{GEA_CORE_DIR}" REALPATH)
get_filename_component(GEA_COMPILER "$ENV{GEA_COMPILER_DIR}" REALPATH)
get_filename_component(GEA_PLUGIN "$ENV{GEA_PLUGIN_DIR}" REALPATH)
get_filename_component(GEA_ENGINE "$ENV{GEA_ENGINE_DIR}" REALPATH)
get_filename_component(GEA_HOST "$ENV{GEA_HOST_DIR}" REALPATH)
get_filename_component(GEA_CHIPS "$ENV{GEA_CHIPS_DIR}" REALPATH)
get_filename_component(GEA_ELEMENTS "$ENV{GEA_ELEMENTS_DIR}" REALPATH)
get_filename_component(GEA_GEAOS_PACKAGE "$ENV{GEA_GEAOS_PACKAGE_DIR}" REALPATH)
# gea_app_entry.cpp is an app-mode source exported by @geastack/core.
set(GEA_APP_ENTRY_SRC "${GEA_CORE}/gea_app_entry.cpp")
if(NOT EXISTS "${GEA_APP_ENTRY_SRC}")
    message(FATAL_ERROR "@geastack/core does not contain gea_app_entry.cpp")
endif()
# Pull the framework source/include lists from the shared manifest once at configure.
# The manifest is handed the REAL package paths, the same spelling
# `GEA_FW_APP_SENSITIVE_SOURCES` below is written in. A project that links
# `@geastack/host` into node_modules (a workspace symlink) otherwise names
# every framework source under `node_modules/@geastack/host/...` while
# `${GEA_HOST}` says `core/packages/host/...`; `list(REMOVE_ITEM)` compares
# strings, misses, and fetch.cpp/websocket.cpp are compiled a second time in
# the framework component, where the network requirements main carries for
# them do not exist (`esp_http_client.h: No such file`).
# The manifest is read with node rather than bash: Windows has no bash, so
# `bash -c "source gea_sources.sh; ..."` failed outright there and no ESP32
# target could configure. Node is already required by everything that reaches
# this file, and gea_sources.sh is now a thin wrapper over the same manifest.
set(_GEA_FW_ENV
    "GEA_CORE=${GEA_CORE}" "GEA_ENGINE_DIR=${GEA_ENGINE}" "GEA_HOST_DIR=${GEA_HOST}"
    "GEA_ELEMENTS_DIR=${GEA_ELEMENTS}" "GEA_GEAOS_PACKAGE_DIR=${GEA_GEAOS_PACKAGE}")
set(_GEA_FW_MANIFEST "${GEA_CORE}/gea_sources.mjs")
execute_process(COMMAND ${CMAKE_COMMAND} -E env ${_GEA_FW_ENV} node "${_GEA_FW_MANIFEST}" cxx-sources
    OUTPUT_VARIABLE _GEA_FW_CXX OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _GEA_FW_RC1)
execute_process(COMMAND ${CMAKE_COMMAND} -E env ${_GEA_FW_ENV} node "${_GEA_FW_MANIFEST}" c-sources
    OUTPUT_VARIABLE _GEA_FW_C OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _GEA_FW_RC2)
execute_process(COMMAND ${CMAKE_COMMAND} -E env ${_GEA_FW_ENV} node "${_GEA_FW_MANIFEST}" include-flags
    OUTPUT_VARIABLE _GEA_FW_INC OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _GEA_FW_RC3)
if(_GEA_FW_RC1 OR _GEA_FW_RC2 OR _GEA_FW_RC3)
    message(FATAL_ERROR "gea_sources.mjs framework manifest failed (rc ${_GEA_FW_RC1}/${_GEA_FW_RC2}/${_GEA_FW_RC3})")
endif()
# -mtext-section-literals is XTENSA-ONLY (esp32, esp32-s3). The decomposed
# framework TUs each instantiate large gea_cpp_value COMDATs whose literal pools
# overflow the xtensa l32r range at link without it. RISC-V targets (esp32-p4,
# riscv32) reject the flag and don't have the limit — so it's empty there.
# Boards splice ${GEA_XTENSA_LITERALS} into their component compile options.
# App-declared defines go on the WHOLE native build, not only the app's own
# sources. These macros size framework types -- a cache-slot count changes a
# struct's layout -- so the app and the framework must be compiled with the same
# value or they disagree about that struct at link. Setting them as a build
# property reaches every component without each board target repeating them.
# IDF's component-requirements pass runs this file through `cmake -P`, where the
# build-property commands do not exist yet. That pass only collects REQUIRES, so
# skipping the defines there costs nothing.
if(COMMAND idf_build_set_property)
    foreach(_GEA_APP_DEFINE IN LISTS GEA_EMBEDDED_APP_DEFINES)
        if(_GEA_APP_DEFINE)
            idf_build_set_property(COMPILE_DEFINITIONS "${_GEA_APP_DEFINE}" APPEND)
        endif()
    endforeach()
endif()

# Linker fragments are absolute paths from the CLI. ldgen merges every
# fragment in the build into one script and each mapping names the archive it
# applies to, so which component registers them does not matter -- the shared
# gea_framework component carries them for every board.
set(GEA_EMBEDDED_APP_LDFRAGMENT_FILES "")
foreach(_GEA_APP_LDFRAGMENT IN LISTS GEA_EMBEDDED_APP_LDFRAGMENTS)
    if(NOT _GEA_APP_LDFRAGMENT)
        continue()
    endif()
    if(NOT EXISTS "${_GEA_APP_LDFRAGMENT}")
        message(FATAL_ERROR "gea.ldFragments file does not exist: ${_GEA_APP_LDFRAGMENT}")
    endif()
    list(APPEND GEA_EMBEDDED_APP_LDFRAGMENT_FILES "${_GEA_APP_LDFRAGMENT}")
endforeach()

# GEA_EMBEDDED_UI_STATE_EXTERNAL=1 in gea.defines moves the engine's task-only
# state into external RAM. That is two changes which only work together, so one
# define drives both: the engine initializes that state at runtime so it is
# .bss rather than .data (GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT), and the shipped
# linker fragment sweeps that .bss to external RAM.
set(_GEA_UI_STATE_EXTERNAL OFF)
foreach(_GEA_APP_DEFINE IN LISTS GEA_EMBEDDED_APP_DEFINES)
    if(_GEA_APP_DEFINE MATCHES "^GEA_EMBEDDED_UI_STATE_EXTERNAL(=[1-9][0-9]*)?$")
        set(_GEA_UI_STATE_EXTERNAL ON)
    endif()
endforeach()
if(_GEA_UI_STATE_EXTERNAL)
    if(DEFINED CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY AND NOT CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY)
        message(FATAL_ERROR "GEA_EMBEDDED_UI_STATE_EXTERNAL needs CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y in the app's sdkconfig")
    endif()
    if(COMMAND idf_build_set_property)
        idf_build_set_property(COMPILE_DEFINITIONS "GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT=1" APPEND)
    endif()
    list(APPEND GEA_EMBEDDED_APP_LDFRAGMENT_FILES "${GEA_EMBEDDED_ROOT}/targets/esp32/ldfragments/gea_ui_state_external.lf")
endif()

set(GEA_XTENSA_LITERALS "")
if(CMAKE_CXX_COMPILER MATCHES "xtensa")
    set(GEA_XTENSA_LITERALS "-mtext-section-literals")
endif()

string(REPLACE "\n" ";" GEA_FW_CXX_SOURCES "${_GEA_FW_CXX}")
string(REPLACE "\n" ";" GEA_FW_C_SOURCES "${_GEA_FW_C}")
string(REPLACE "\n" ";" _GEA_FW_INC_FLAGS "${_GEA_FW_INC}")
set(GEA_FW_INCLUDE_DIRS "")
foreach(_f ${_GEA_FW_INC_FLAGS})
    string(REGEX REPLACE "^-I" "" _d "${_f}")
    if(_d)
        list(APPEND GEA_FW_INCLUDE_DIRS "${_d}")
    endif()
endforeach()

# Keep sources whose dependencies or compile-time behavior vary with the active
# app in main. The remaining framework sources form a stable IDF component:
# changing generated app code, network capability, or render-tuning flags no
# longer recompiles the whole framework.
set(GEA_FW_APP_SENSITIVE_SOURCES
    "${GEA_ENGINE}/ui/tree_render.cpp"
    "${GEA_ENGINE}/ui/tree_style.cpp"
    "${GEA_HOST}/host/fetch.cpp"
    "${GEA_HOST}/host/websocket.cpp"
    "${GEA_HOST}/host/http.cpp"
    "${GEA_HOST}/host/image.cpp"
    "${GEA_HOST}/host/rtc.cpp"
    "${GEA_HOST}/host/tile_loader.cpp"
    "${GEA_HOST}/services/bluetooth_service.cpp"
    "${GEA_HOST}/services/network_services.cpp"
    "${GEA_CORE}/runtime.cpp"
)
list(REMOVE_ITEM GEA_FW_CXX_SOURCES ${GEA_FW_APP_SENSITIVE_SOURCES})

# The direct-canvas profiling apps intentionally use a small framework slice.
# Reproduce that slice in the stable component instead of compiling the normal
# framework and relying on the linker to discard it.
if(GEA_EMBEDDED_APP STREQUAL "canvas-3d" OR GEA_EMBEDDED_APP STREQUAL "gea3d-cube")
    set(GEA_FW_CXX_SOURCES
        "${GEA_ENGINE}/ui/canvas_element.cpp"
        "${GEA_ENGINE}/canvas.cpp"
        "${GEA_ENGINE}/bitmap_font.cpp"
        "${GEA_ENGINE}/rasterized_font.cpp"
        "${GEA_CORE}/app_frame_perf.cpp"
        "${GEA_CORE}/runtime.cpp"
        "${GEA_CORE}/events.cpp"
        "${GEA_HOST}/host/timers.cpp"
        "${GEA_HOST}/host/display.cpp"
    )
    set(GEA_FW_C_SOURCES "")
    set(GEA_FW_APP_SENSITIVE_SOURCES "")
endif()

# Digital pins and the WS2812 strand most modules carry as their "onboard LED",
# appended AFTER the direct-canvas slice above so that profile keeps them too.
# Every ESP32 board gets them, and deliberately so: on a board with no display
# these pins are the only output an app can reach, and a capability gate would
# mean a headless board's one way to say anything depends on a flag nobody sets.
list(APPEND GEA_FW_CXX_SOURCES
    "${GEA_HOST}/host/gpio.cpp"
    "${GEA_EMBEDDED_ROOT}/targets/esp32/gpio.cpp"
)

function(gea_framework_compile_definitions)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${ARGN})
    if(TARGET idf::gea_framework)
        idf_component_get_property(_gea_framework_lib gea_framework COMPONENT_LIB)
        target_compile_definitions(${_gea_framework_lib} PRIVATE ${ARGN})
    endif()
endfunction()

function(gea_framework_compile_options)
    target_compile_options(${COMPONENT_LIB} PRIVATE ${ARGN})
    if(TARGET idf::gea_framework)
        idf_component_get_property(_gea_framework_lib gea_framework COMPONENT_LIB)
        target_compile_options(${_gea_framework_lib} PRIVATE ${ARGN})
    endif()
endfunction()

function(gea_framework_inherit_build_settings)
    if(NOT TARGET idf::gea_framework)
        return()
    endif()

    idf_component_get_property(_gea_framework_lib gea_framework COMPONENT_LIB)
    get_target_property(_gea_framework_definitions ${COMPONENT_LIB} COMPILE_DEFINITIONS)
    if(_gea_framework_definitions)
        # These definitions only affect sources deliberately retained in main.
        # Filtering them keeps active-app capability/tuning changes from
        # invalidating the stable framework component.
        list(FILTER _gea_framework_definitions EXCLUDE REGEX
            "^(GEA_EMBEDDED_APP_USES_BLE|GEA_EMBEDDED_APP_USES_AUDIO|GEA_EMBEDDED_BLE_DISABLED|GEA_EMBEDDED_AUDIO_UNUSED|GEA_EMBEDDED_ENABLE_HTTPS|GEA_EMBEDDED_WIFI_DISABLED|GEA_EMBEDDED_DISPLAY_FUSE_REPLAY_FLUSH|GEA_EMBEDDED_SUBTREE_REVEAL_CHECK|GEA_EMBEDDED_SKIP_POSITION_INLINE_RECORD)(=.*)?$")
        target_compile_definitions(${_gea_framework_lib} PRIVATE ${_gea_framework_definitions})
    endif()

    get_target_property(_gea_framework_options ${COMPONENT_LIB} COMPILE_OPTIONS)
    if(_gea_framework_options)
        set(_gea_framework_stable_options "")
        set(_gea_skip_iquote_path FALSE)
        foreach(_gea_option IN LISTS _gea_framework_options)
            if(_gea_skip_iquote_path)
                set(_gea_skip_iquote_path FALSE)
            elseif(_gea_option STREQUAL "-iquote")
                # Generated app headers are private to main. Inheriting their
                # app-specific path changes every framework compile command and
                # defeats component/cache reuse even though framework sources
                # do not include those headers.
                set(_gea_skip_iquote_path TRUE)
            else()
                list(APPEND _gea_framework_stable_options "${_gea_option}")
            endif()
        endforeach()
        target_compile_options(${_gea_framework_lib} PRIVATE ${_gea_framework_stable_options})
    endif()

    # Framework code calls target implementations retained in main, while main
    # also calls into the framework. Keep the pair in one explicit rescan group;
    # IDF's normal one-pass component ordering cannot resolve both directions.
    # main is a static archive, so PRIVATE link options stop at the archive and
    # never reach the final executable link. Publish the rescan group through
    # its link interface so the ELF linker actually sees it.
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl$<COMMA>--start-group$<COMMA>${CMAKE_BINARY_DIR}/esp-idf/main/libmain.a$<COMMA>${CMAKE_BINARY_DIR}/esp-idf/gea_framework/libgea_framework.a$<COMMA>--end-group")

endfunction()

# Applied from the shared gea_framework component, which every board requires.
# Link options published through its INTERFACE reach the final executable link,
# and binary data compiled into libgea_framework.a resolves for main the same
# way it would have there -- so an app gets both without any board target
# repeating them.
function(gea_apply_app_native_build)
    # Runtime::run() calls gea_app_native_boot() if an app defines one, through a
    # weak declaration. A weak undefined reference does NOT pull an object out of
    # a static archive, and every app native source is archived into libmain.a --
    # so without this the hook would silently link as null. `-u` enters the name
    # as a real undefined symbol, which does pull the member.
    #
    # Only when the app actually spells the hook. `-u` makes the reference
    # STRONG, so ld must resolve it -- name it for an app that never defines one
    # and the link fails outright with "undefined reference to
    # gea_app_native_boot". The flag is what pulls a hook that exists; where none
    # exists the weak reference is meant to stay null, and naming the symbol is
    # what stops it from doing so.
    #
    # "Has native sources" is NOT the same question: an app may bring native code
    # that is pure library surface reached from TS (e-reader's epub_archive.cpp)
    # and define no boot hook at all -- that spelling failed the link for every
    # such app. Ask the real question instead, by reading the app's own native
    # sources at configure time. CMake re-runs when any of them changes, so a hook
    # added or removed later re-decides this on the next build.
    set(_GEA_APP_DEFINES_NATIVE_BOOT NO)
    foreach(_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL IN LISTS GEA_EMBEDDED_APP_NATIVE_SOURCE_RELS)
        set(_GEA_APP_NATIVE_SOURCE "${GEA_EMBEDDED_APP_DIR}/${_GEA_EMBEDDED_APP_NATIVE_SOURCE_REL}")
        if(EXISTS "${_GEA_APP_NATIVE_SOURCE}")
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_GEA_APP_NATIVE_SOURCE}")
            file(STRINGS "${_GEA_APP_NATIVE_SOURCE}" _GEA_APP_BOOT_HIT REGEX "gea_app_native_boot[ \t]*\\(")
            if(_GEA_APP_BOOT_HIT)
                set(_GEA_APP_DEFINES_NATIVE_BOOT YES)
            endif()
        endif()
    endforeach()
    if(_GEA_APP_DEFINES_NATIVE_BOOT)
        target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,-u,gea_app_native_boot")
    endif()

    foreach(_GEA_APP_LINK_OPTION IN LISTS GEA_EMBEDDED_APP_LINK_OPTIONS)
        if(_GEA_APP_LINK_OPTION)
            target_link_options(${COMPONENT_LIB} INTERFACE "${_GEA_APP_LINK_OPTION}")
        endif()
    endforeach()

    # `symbol=file`: the name the firmware refers to the bytes by, and where
    # they come from. Split on the FIRST '=' so a path may contain one.
    foreach(_GEA_APP_EMBED IN LISTS GEA_EMBEDDED_APP_EMBED_FILES)
        if(NOT _GEA_APP_EMBED)
            continue()
        endif()
        string(FIND "${_GEA_APP_EMBED}" "=" _GEA_APP_EMBED_SPLIT)
        if(_GEA_APP_EMBED_SPLIT LESS 1)
            message(FATAL_ERROR "gea.targets.esp32.embedFiles entry is not symbol=file: ${_GEA_APP_EMBED}")
        endif()
        string(SUBSTRING "${_GEA_APP_EMBED}" 0 ${_GEA_APP_EMBED_SPLIT} _GEA_APP_EMBED_SYMBOL)
        math(EXPR _GEA_APP_EMBED_REST "${_GEA_APP_EMBED_SPLIT} + 1")
        string(SUBSTRING "${_GEA_APP_EMBED}" ${_GEA_APP_EMBED_REST} -1 _GEA_APP_EMBED_FILE)
        if(NOT EXISTS "${_GEA_APP_EMBED_FILE}")
            message(FATAL_ERROR "gea.targets.esp32.embedFiles file does not exist: ${_GEA_APP_EMBED_FILE}")
        endif()
        target_add_binary_data(${COMPONENT_LIB} "${_GEA_APP_EMBED_FILE}" BINARY
                               RENAME_TO ${_GEA_APP_EMBED_SYMBOL} DEPENDS "${_GEA_APP_EMBED_FILE}")
    endforeach()

    # `partition=file`: written to that named data partition at flash time,
    # which is a different thing from the embedded copy above. An app may want
    # both -- the pedal repairs a stale factory partition from the embedded one.
    foreach(_GEA_APP_PAYLOAD IN LISTS GEA_EMBEDDED_APP_PARTITION_DATA)
        if(NOT _GEA_APP_PAYLOAD)
            continue()
        endif()
        string(FIND "${_GEA_APP_PAYLOAD}" "=" _GEA_APP_PAYLOAD_SPLIT)
        if(_GEA_APP_PAYLOAD_SPLIT LESS 1)
            message(FATAL_ERROR "partition payload is not partition=file: ${_GEA_APP_PAYLOAD}")
        endif()
        string(SUBSTRING "${_GEA_APP_PAYLOAD}" 0 ${_GEA_APP_PAYLOAD_SPLIT} _GEA_APP_PAYLOAD_NAME)
        math(EXPR _GEA_APP_PAYLOAD_REST "${_GEA_APP_PAYLOAD_SPLIT} + 1")
        string(SUBSTRING "${_GEA_APP_PAYLOAD}" ${_GEA_APP_PAYLOAD_REST} -1 _GEA_APP_PAYLOAD_FILE)
        esptool_py_flash_to_partition(flash "${_GEA_APP_PAYLOAD_NAME}" "${_GEA_APP_PAYLOAD_FILE}")
    endforeach()
endfunction()
