foreach(_required
        GEA_GEATSC_SOURCE_LIST
        GEA_GEATSC_ARCHIVE
        GEA_GEATSC_OBJECT_DIR
        GEA_GEATSC_DEPFILE
        GEA_GEATSC_FLAGS_FILE
        GEA_GEATSC_CXX_COMPILER
        GEA_GEATSC_AR)
    if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
        message(FATAL_ERROR "${_required} is required")
    endif()
endforeach()

get_filename_component(
    _GEA_GEATSC_ARCHIVE_BUILDER
    "${CMAKE_CURRENT_LIST_DIR}/geatsc-generated-archive-build.mjs"
    ABSOLUTE
)
if(NOT EXISTS "${_GEA_GEATSC_ARCHIVE_BUILDER}")
    message(FATAL_ERROR "geatsc archive builder not found: ${_GEA_GEATSC_ARCHIVE_BUILDER}")
endif()
find_program(_GEA_GEATSC_NODE_EXECUTABLE NAMES node REQUIRED)

set(_GEA_GEATSC_COMPILER_LAUNCHER_ARGS "")
if(DEFINED GEA_GEATSC_COMPILER_LAUNCHER AND NOT "${GEA_GEATSC_COMPILER_LAUNCHER}" STREQUAL "")
    list(APPEND _GEA_GEATSC_COMPILER_LAUNCHER_ARGS
        --compiler-launcher "${GEA_GEATSC_COMPILER_LAUNCHER}")
endif()

# Keep this CMake entry point stable for every ESP32/RP2350 caller, but let the
# Node runner own per-object depfile checks and the bounded worker pool. The
# compiler inputs, flags, object names, archive order, and link interface remain
# unchanged.
execute_process(
    COMMAND "${_GEA_GEATSC_NODE_EXECUTABLE}"
        "${_GEA_GEATSC_ARCHIVE_BUILDER}"
        --source-list "${GEA_GEATSC_SOURCE_LIST}"
        --archive "${GEA_GEATSC_ARCHIVE}"
        --object-dir "${GEA_GEATSC_OBJECT_DIR}"
        --depfile "${GEA_GEATSC_DEPFILE}"
        --flags-file "${GEA_GEATSC_FLAGS_FILE}"
        --compiler "${GEA_GEATSC_CXX_COMPILER}"
        ${_GEA_GEATSC_COMPILER_LAUNCHER_ARGS}
        --ar "${GEA_GEATSC_AR}"
        --ranlib "${GEA_GEATSC_RANLIB}"
    RESULT_VARIABLE _build_result
)
if(NOT _build_result EQUAL 0)
    message(FATAL_ERROR "Failed to build geatsc generated archive: ${GEA_GEATSC_ARCHIVE}")
endif()
