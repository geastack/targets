# Included by every esp32 board's top-level CMakeLists BEFORE project(), which
# is the only phase where EXTRA_COMPONENT_DIRS still has an effect. An app
# declares its own IDF components -- a vendored driver, a DSP library with its
# own flags -- through gea.targets.esp32.componentDirs; they cannot be folded
# into nativeSources because a component carries its own compile options,
# REQUIRES and conditions, not just a list of files.
if(DEFINED ENV{GEA_EMBEDDED_APP_COMPONENT_DIRS} AND NOT "$ENV{GEA_EMBEDDED_APP_COMPONENT_DIRS}" STREQUAL "")
    set(GEA_EMBEDDED_APP_COMPONENT_DIRS "$ENV{GEA_EMBEDDED_APP_COMPONENT_DIRS}"
        CACHE STRING "Gea app IDF component directories" FORCE)
endif()
foreach(_GEA_APP_COMPONENT_DIR IN LISTS GEA_EMBEDDED_APP_COMPONENT_DIRS)
    if(NOT _GEA_APP_COMPONENT_DIR)
        continue()
    endif()
    if(NOT IS_DIRECTORY "${_GEA_APP_COMPONENT_DIR}")
        message(FATAL_ERROR "gea.targets.esp32.componentDirs is not a directory: ${_GEA_APP_COMPONENT_DIR}")
    endif()
    list(APPEND EXTRA_COMPONENT_DIRS "${_GEA_APP_COMPONENT_DIR}")
endforeach()

# ESP-SR is needed only by apps opting into the shared audio echo canceller.
# Derive this before IDF resolves managed dependencies, including its script pass.
set(_GEA_AUDIO_DEFINES "${GEA_EMBEDDED_APP_DEFINES};$ENV{GEA_EMBEDDED_APP_DEFINES}")
set(ENV{GEA_EMBEDDED_CAPABILITY_AEC} "0")
if(";${_GEA_AUDIO_DEFINES};" MATCHES ";GEA_AUDIO_ECHO_CANCELLATION=1;")
    set(ENV{GEA_EMBEDDED_CAPABILITY_AEC} "1")
endif()

# The managed-dependency manifests (main/idf_component.yml) gate the registry
# components that belong to one capability -- the codec, the WebSocket client,
# the WebRTC peer -- on these variables, and the component manager treats an
# unset environment variable in an `if:` rule as a hard error rather than a
# false. The gea CLI always exports these values; a direct `idf.py` build has no
# app analysis to go on, so it defaults to keeping everything, which is what it
# built before the rules existed.
foreach(_GEA_CAPABILITY NETWORK BLE AUDIO RTC H264)
    if(NOT DEFINED ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}
            OR "$ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}" STREQUAL "")
        # Incremental Ninja/CMake reconfiguration does not inherit the CLI's
        # original environment. Keep its cached analysis instead of silently
        # enabling BLE (and consuming the audio DMA heap) on the next build.
        if(DEFINED CACHE{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}
                AND NOT "$CACHE{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}" STREQUAL "")
            set(ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}
                "$CACHE{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}")
        else()
            set(ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}} "1")
        endif()
    endif()
endforeach()

# The component manager writes the dependencies it solved into the lock file,
# and which components are dependencies depends on the app being built (the
# GEA_EMBEDDED_CAPABILITY_* rules in main/idf_component.yml). Solving into the
# target's tracked dependencies.lock therefore rewrote it for every app with a
# different capability set. A target calls this after including project.cmake:
# the build solves into its own build directory, starting from the tracked lock
# (copied again whenever the tracked one is newer) so the versions it pins still
# hold, and the source tree stays as it was checked out.
macro(gea_solve_dependencies_in_build_dir)
    set(_GEA_TRACKED_LOCK "${CMAKE_CURRENT_LIST_DIR}/dependencies.lock")
    set(_GEA_BUILD_LOCK "${CMAKE_BINARY_DIR}/dependencies.lock")
    if(EXISTS "${_GEA_TRACKED_LOCK}"
            AND (NOT EXISTS "${_GEA_BUILD_LOCK}" OR "${_GEA_TRACKED_LOCK}" IS_NEWER_THAN "${_GEA_BUILD_LOCK}"))
        configure_file("${_GEA_TRACKED_LOCK}" "${_GEA_BUILD_LOCK}" COPYONLY)
    endif()
    idf_build_set_property(DEPENDENCIES_LOCK "${_GEA_BUILD_LOCK}")
endmacro()
