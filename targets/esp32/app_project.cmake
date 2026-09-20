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

# The managed-dependency manifests (main/idf_component.yml) gate the registry
# components that belong to one capability -- the codec, the WebSocket client,
# the WebRTC peer -- on these variables, and the component manager treats an
# unset environment variable in an `if:` rule as a hard error rather than a
# false. The gea CLI always exports all three; a direct `idf.py` build has no
# app analysis to go on, so it defaults to keeping everything, which is what it
# built before the rules existed.
foreach(_GEA_CAPABILITY NETWORK BLE AUDIO)
    if(NOT DEFINED ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}
            OR "$ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}}" STREQUAL "")
        set(ENV{GEA_EMBEDDED_CAPABILITY_${_GEA_CAPABILITY}} "1")
    endif()
endforeach()
