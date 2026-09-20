function(gea_geatsc_add_generated_archive)
    get_filename_component(
        _GEA_GEATSC_ARCHIVE_BUILDER
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/geatsc-generated-archive-build.mjs"
        ABSOLUTE
    )
    set(_options)
    set(_one_value_args
        TARGET
        NAME
        SOURCE_LIST
        OUTPUT
        OBJECT_DIR
        DEPFILE
        CXX_STANDARD
    )
    set(_multi_value_args
        DEPENDS
        COMPILE_DEFINITIONS
        COMPILE_OPTIONS
    )
    cmake_parse_arguments(GEA_GEATSC "${_options}" "${_one_value_args}" "${_multi_value_args}" ${ARGN})

    if(NOT GEA_GEATSC_TARGET)
        message(FATAL_ERROR "gea_geatsc_add_generated_archive requires TARGET")
    endif()
    if(NOT GEA_GEATSC_SOURCE_LIST)
        message(FATAL_ERROR "gea_geatsc_add_generated_archive requires SOURCE_LIST")
    endif()
    if(NOT GEA_GEATSC_OUTPUT)
        message(FATAL_ERROR "gea_geatsc_add_generated_archive requires OUTPUT")
    endif()

    if(NOT GEA_GEATSC_NAME)
        set(GEA_GEATSC_NAME "${GEA_GEATSC_TARGET}_geatsc")
    endif()
    if(NOT GEA_GEATSC_OBJECT_DIR)
        set(GEA_GEATSC_OBJECT_DIR "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${GEA_GEATSC_NAME}.dir")
    endif()
    if(NOT GEA_GEATSC_DEPFILE)
        set(GEA_GEATSC_DEPFILE "${GEA_GEATSC_OBJECT_DIR}/${GEA_GEATSC_NAME}.d")
    endif()
    if(NOT GEA_GEATSC_CXX_STANDARD)
        if(CMAKE_CXX_STANDARD)
            set(GEA_GEATSC_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
        else()
            set(GEA_GEATSC_CXX_STANDARD 20)
        endif()
    endif()

    # The target's COMPILE_DEFINITIONS carry duplicates (ESP-IDF propagates
    # mbedtls's `__STDC_WANT_LIB_EXT1__=0` through more than one component
    # interface), and xtensa gcc 15 treats a repeated `-D__STDC_...` as a
    # redefinition even when both spellings agree -- an error under -Werror. The
    # normal compile rules never see this because CMake de-duplicates
    # definitions when it builds a target's own command line; this response
    # file is assembled by hand, so it de-duplicates here.
    # The generated content embeds the target's COMPILE_OPTIONS, which may carry
    # per-language genexes (e.g. $<$<COMPILE_LANGUAGE:C>:...>). That forces
    # file(GENERATE) into per-language evaluation, so the OUTPUT path must vary by
    # $<COMPILE_LANGUAGE> or CMake aborts with "written multiple times with
    # different content". Emit one .rsp per language and consume the CXX variant,
    # since the geatsc-generated archive is compiled as C++.
    set(_flags_file_template "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${GEA_GEATSC_NAME}.$<COMPILE_LANGUAGE>.rsp")
    set(_flags_file "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${GEA_GEATSC_NAME}.CXX.rsp")
    set(_extra_flags "")
    foreach(_definition IN LISTS GEA_GEATSC_COMPILE_DEFINITIONS)
        list(APPEND _extra_flags "-D${_definition}")
    endforeach()
    foreach(_option IN LISTS GEA_GEATSC_COMPILE_OPTIONS)
        list(APPEND _extra_flags "${_option}")
    endforeach()
    string(REPLACE ";" "\n" _extra_flags_content "${_extra_flags}")

    set(_config_flags "")
    if(CMAKE_BUILD_TYPE)
        string(TOUPPER "${CMAKE_BUILD_TYPE}" _build_type_upper)
        set(_config_flags "${CMAKE_CXX_FLAGS_${_build_type_upper}}")
    endif()

    file(GENERATE
        OUTPUT "${_flags_file_template}"
        CONTENT "${CMAKE_CXX_FLAGS}\n${_config_flags}\n-std=gnu++${GEA_GEATSC_CXX_STANDARD}\n$<$<BOOL:$<TARGET_PROPERTY:${GEA_GEATSC_TARGET},INCLUDE_DIRECTORIES>>:-I$<JOIN:$<TARGET_PROPERTY:${GEA_GEATSC_TARGET},INCLUDE_DIRECTORIES>,\n-I>>\n$<$<BOOL:$<TARGET_PROPERTY:${GEA_GEATSC_TARGET},COMPILE_DEFINITIONS>>:-D$<JOIN:$<REMOVE_DUPLICATES:$<TARGET_PROPERTY:${GEA_GEATSC_TARGET},COMPILE_DEFINITIONS>>,\n-D>>\n$<JOIN:$<TARGET_PROPERTY:${GEA_GEATSC_TARGET},COMPILE_OPTIONS>,\n>\n${_extra_flags_content}\n"
    )

    # ESP-IDF exposes ccache through RULE_LAUNCH_COMPILE rather than changing
    # CMAKE_CXX_COMPILER. This custom archive bypasses normal compile rules, so
    # forward the launcher explicitly to retain clean-build cache hits.
    get_property(_compiler_launcher GLOBAL PROPERTY RULE_LAUNCH_COMPILE)

    add_custom_command(
        OUTPUT "${GEA_GEATSC_OUTPUT}"
        COMMAND "${CMAKE_COMMAND}"
            "-DGEA_GEATSC_SOURCE_LIST=${GEA_GEATSC_SOURCE_LIST}"
            "-DGEA_GEATSC_ARCHIVE=${GEA_GEATSC_OUTPUT}"
            "-DGEA_GEATSC_OBJECT_DIR=${GEA_GEATSC_OBJECT_DIR}"
            "-DGEA_GEATSC_DEPFILE=${GEA_GEATSC_DEPFILE}"
            "-DGEA_GEATSC_FLAGS_FILE=${_flags_file}"
            "-DGEA_GEATSC_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
            "-DGEA_GEATSC_COMPILER_LAUNCHER=${_compiler_launcher}"
            "-DGEA_GEATSC_AR=${CMAKE_AR}"
            "-DGEA_GEATSC_RANLIB=${CMAKE_RANLIB}"
            -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/geatsc_generated_archive_build.cmake"
        DEPENDS
            ${GEA_GEATSC_DEPENDS}
            "${_flags_file}"
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/geatsc_generated_archive_build.cmake"
            "${_GEA_GEATSC_ARCHIVE_BUILDER}"
        DEPFILE "${GEA_GEATSC_DEPFILE}"
        COMMENT "Archiving geatsc generated sources for ${GEA_GEATSC_TARGET}"
        VERBATIM
    )
    add_custom_target(${GEA_GEATSC_NAME} DEPENDS "${GEA_GEATSC_OUTPUT}")
    add_dependencies(${GEA_GEATSC_TARGET} ${GEA_GEATSC_NAME})
    target_link_libraries(${GEA_GEATSC_TARGET} PRIVATE "${GEA_GEATSC_OUTPUT}")
endfunction()
