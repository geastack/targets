include("${CMAKE_CURRENT_LIST_DIR}/../cmake/gea_build_config.cmake")
if(NOT GEA_APP_CAPABILITY_ANALYSIS_VERSION EQUAL 1)
    message(FATAL_ERROR "Capability analysis is missing or unsupported. Update @geastack/cli together with @geastack/targets.")
endif()
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

if(GEA_APP_CAPABILITY_DEPENDENCY_DIR)
    list(APPEND EXTRA_COMPONENT_DIRS "${GEA_APP_CAPABILITY_DEPENDENCY_DIR}")
endif()

# The component manager writes the dependencies it solved into the lock file,
# and which components are dependencies depends on the app being built (the
# capability-selected SDKs the CLI places in GEA_APP_CAPABILITY_DEPENDENCY_DIR).
# Solving into the target's tracked dependencies.lock therefore rewrote it for
# every app with a different capability set. A target calls this after
# including project.cmake: the build solves into its own build directory,
# starting from the tracked lock (copied again whenever the tracked one is
# newer) so the versions it pins still hold, and the source tree stays as it
# was checked out.
macro(gea_solve_dependencies_in_build_dir)
    set(_GEA_TRACKED_LOCK "${CMAKE_CURRENT_LIST_DIR}/dependencies.lock")
    set(_GEA_BUILD_LOCK "${CMAKE_BINARY_DIR}/dependencies.lock")
    if(EXISTS "${_GEA_TRACKED_LOCK}"
            AND (NOT EXISTS "${_GEA_BUILD_LOCK}" OR "${_GEA_TRACKED_LOCK}" IS_NEWER_THAN "${_GEA_BUILD_LOCK}"))
        configure_file("${_GEA_TRACKED_LOCK}" "${_GEA_BUILD_LOCK}" COPYONLY)
    endif()
    idf_build_set_property(DEPENDENCIES_LOCK "${_GEA_BUILD_LOCK}")
endmacro()
