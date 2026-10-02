# Decoder-only libvpx, shared by native Gea WebRTC clients. No encoder, VP9,
# postprocessing, worker pool, or desktop assembly is built for the S3.
cmake_minimum_required(VERSION 3.24)

if(COMMAND idf_component_register)
    idf_component_register(SRCS "vpx_memory.c" REQUIRES heap)
    if(CMAKE_SCRIPT_MODE_FILE)
        return()
    endif()
    set(_gea_vpx_target ${COMPONENT_LIB})
else()
    project(gea_vpx C)
    add_library(gea_vpx STATIC)
    set(_gea_vpx_target gea_vpx)
endif()

include(FetchContent)
FetchContent_Declare(gea_vpx_source
    URL https://github.com/webmproject/libvpx/archive/refs/tags/v1.16.0.tar.gz
    URL_HASH SHA256=7a479a3c66b9f5d5542a4c6a1b7d3768a983b1e5c14c60a9396edc9b649e015c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# Upstream is a make project; CMake owns compilation with the actual IDF flags.
FetchContent_MakeAvailable(gea_vpx_source)
find_program(GEA_VPX_MAKE NAMES gmake make REQUIRED)
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env "TMPDIR=${gea_vpx_source_BINARY_DIR}"
        sh "${gea_vpx_source_SOURCE_DIR}/configure"
        --target=generic-gnu --enable-external-build
        --disable-vp9 --disable-vp8-encoder --enable-vp8-decoder
        --disable-multithread --disable-runtime-cpu-detect --disable-postproc
        --disable-examples --disable-tools --disable-docs --disable-unit-tests
        --disable-webm-io --disable-libyuv
    WORKING_DIRECTORY "${gea_vpx_source_BINARY_DIR}"
    RESULT_VARIABLE _gea_vpx_configure_result
    OUTPUT_VARIABLE _gea_vpx_configure_output
    ERROR_VARIABLE _gea_vpx_configure_error
)
if(NOT _gea_vpx_configure_result EQUAL 0)
    message(FATAL_ERROR "libvpx configure failed: ${_gea_vpx_configure_output}\n${_gea_vpx_configure_error}")
endif()
execute_process(
    COMMAND "${GEA_VPX_MAKE}" libvpx_srcs.txt vp8_rtcd.h vpx_scale_rtcd.h vpx_dsp_rtcd.h
    WORKING_DIRECTORY "${gea_vpx_source_BINARY_DIR}"
    RESULT_VARIABLE _gea_vpx_generate_result
    OUTPUT_VARIABLE _gea_vpx_generate_output
    ERROR_VARIABLE _gea_vpx_generate_error
)
if(NOT _gea_vpx_generate_result EQUAL 0)
    message(FATAL_ERROR "libvpx source generation failed: ${_gea_vpx_generate_output}\n${_gea_vpx_generate_error}")
endif()

file(STRINGS "${gea_vpx_source_BINARY_DIR}/libvpx_srcs.txt" _gea_vpx_sources)
foreach(_gea_vpx_source IN LISTS _gea_vpx_sources)
    if(_gea_vpx_source MATCHES "\\.c$" AND NOT _gea_vpx_source STREQUAL "vpx_mem/vpx_mem.c")
        if(EXISTS "${gea_vpx_source_SOURCE_DIR}/${_gea_vpx_source}")
            target_sources(${_gea_vpx_target} PRIVATE "${gea_vpx_source_SOURCE_DIR}/${_gea_vpx_source}")
        elseif(EXISTS "${gea_vpx_source_BINARY_DIR}/${_gea_vpx_source}")
            target_sources(${_gea_vpx_target} PRIVATE "${gea_vpx_source_BINARY_DIR}/${_gea_vpx_source}")
        else()
            message(FATAL_ERROR "Missing libvpx source: ${_gea_vpx_source}")
        endif()
    endif()
endforeach()
if(NOT COMMAND idf_component_register)
    target_sources(${_gea_vpx_target} PRIVATE vpx_memory.c)
endif()
target_include_directories(${_gea_vpx_target} PUBLIC "${gea_vpx_source_SOURCE_DIR}"
    PRIVATE "${gea_vpx_source_BINARY_DIR}")
target_compile_options(${_gea_vpx_target} PRIVATE -O2 -ffunction-sections -fdata-sections)
if(IDF_TARGET STREQUAL "esp32s3")
    # Boolean arithmetic decoding is serial, not a SIMD lane operation. The
    # generic -O2 build leaves one out-of-line call per entropy bit. Inline it
    # explicitly and use Xtensa NSAU instead of a normalization-table load.
    # Patches are checked against the pinned upstream spelling and idempotent.
    set(_gea_bool_header "${gea_vpx_source_SOURCE_DIR}/vp8/decoder/dboolhuff.h")
    file(READ "${_gea_bool_header}" _gea_bool_source)
    set(_gea_bool_before "${_gea_bool_source}")
    string(REPLACE "static VPX_NO_UNSIGNED_SHIFT_CHECK int vp8dx_decode_bool"
        "static inline __attribute__((always_inline)) VPX_NO_UNSIGNED_SHIFT_CHECK int vp8dx_decode_bool"
        _gea_bool_source "${_gea_bool_source}")
    string(REPLACE "const unsigned char shift = vp8_norm[(unsigned char)range];"
        "const unsigned char shift = (unsigned char)(__builtin_clz(range) - 24);"
        _gea_bool_source "${_gea_bool_source}")
    if(NOT _gea_bool_source MATCHES "always_inline.*VPX_NO_UNSIGNED_SHIFT_CHECK int vp8dx_decode_bool"
        OR NOT _gea_bool_source MATCHES "__builtin_clz\\(range\\) - 24")
        message(FATAL_ERROR "libvpx boolean decoder patch did not match pinned source")
    endif()
    if(NOT _gea_bool_before STREQUAL _gea_bool_source)
        file(WRITE "${_gea_bool_header}" "${_gea_bool_source}")
    endif()
    set(_gea_tokens_source "${gea_vpx_source_SOURCE_DIR}/vp8/decoder/detokenize.c")
    file(READ "${_gea_tokens_source}" _gea_tokens)
    set(_gea_tokens_before "${_gea_tokens}")
    string(REPLACE "BOOL_DECODER local_bc = *x->current_bc;"
        "BOOL_DECODER *state_bc = x->current_bc;\n  BOOL_DECODER local_bc = *state_bc;"
        _gea_tokens "${_gea_tokens}")
    string(REPLACE "BOOL_DECODER *bc = x->current_bc;"
        "BOOL_DECODER *state_bc = x->current_bc;\n  BOOL_DECODER local_bc = *state_bc;\n  BOOL_DECODER *bc = &local_bc;"
        _gea_tokens "${_gea_tokens}")
    string(REPLACE "*x->current_bc = local_bc;" "*state_bc = local_bc;"
        _gea_tokens "${_gea_tokens}")
    if(NOT _gea_tokens MATCHES "\\*state_bc = local_bc;")
        string(REPLACE "  return eobtotal;"
            "  *state_bc = local_bc;\n  return eobtotal;"
            _gea_tokens "${_gea_tokens}")
    endif()
    if(NOT _gea_tokens MATCHES "BOOL_DECODER local_bc = \\*state_bc;"
        OR NOT _gea_tokens MATCHES "\\*state_bc = local_bc;")
        message(FATAL_ERROR "libvpx token decoder state patch did not match pinned source")
    endif()
    # Bounded stage clocks run only when a native diagnostic explicitly binds
    # counters. No profiling task, interrupt, frame copy or codec setting change.
    if(NOT _gea_tokens MATCHES "gea_vp8_profile_clock_s3")
        string(REPLACE "#include \"detokenize.h\""
            "#include \"detokenize.h\"\n#include \"vp8_profile_s3.h\"" _gea_tokens "${_gea_tokens}")
        string(REPLACE "int vp8_decode_mb_tokens(VP8D_COMP *dx, MACROBLOCKD *x) {"
            "int vp8_decode_mb_tokens(VP8D_COMP *dx, MACROBLOCKD *x) {\n  const uint64_t gea_tokens_started = gea_vp8_profile_clock_s3();"
            _gea_tokens "${_gea_tokens}")
        string(REPLACE "  return eobtotal;"
            "  gea_vp8_profile_add_s3(GEA_VP8_PROFILE_TOKENS, gea_tokens_started);\n  return eobtotal;"
            _gea_tokens "${_gea_tokens}")
    endif()
    if(NOT _gea_tokens MATCHES "gea_tokens_started = gea_vp8_profile_clock_s3"
        OR NOT _gea_tokens MATCHES "GEA_VP8_PROFILE_TOKENS, gea_tokens_started")
        message(FATAL_ERROR "libvpx token profile patch did not match pinned source")
    endif()
    if(NOT _gea_tokens_before STREQUAL _gea_tokens)
        file(WRITE "${_gea_tokens_source}" "${_gea_tokens}")
    endif()
    set(_gea_frame_source "${gea_vpx_source_SOURCE_DIR}/vp8/decoder/decodeframe.c")
    file(READ "${_gea_frame_source}" _gea_frame)
    set(_gea_frame_before "${_gea_frame}")
    if(NOT _gea_frame MATCHES "gea_vp8_profile_clock_s3")
        string(REPLACE "#include \"vpx_config.h\""
            "#include \"vpx_config.h\"\n#include \"vp8_profile_s3.h\"" _gea_frame "${_gea_frame}")
        string(REPLACE "      decode_macroblock(pbi, xd, mb_idx);"
            "      {\n        const uint64_t gea_mb_started = gea_vp8_profile_clock_s3();\n        decode_macroblock(pbi, xd, mb_idx);\n        gea_vp8_profile_add_s3(GEA_VP8_PROFILE_MACROBLOCK, gea_mb_started);\n      }"
            _gea_frame "${_gea_frame}")
        string(REGEX MATCHALL "vp8_loop_filter_row_(normal|simple)\\([^;]+\\)" _gea_filter_calls "${_gea_frame}")
        list(LENGTH _gea_filter_calls _gea_filter_call_count)
        if(NOT _gea_filter_call_count EQUAL 4)
            message(FATAL_ERROR "libvpx filter profile expected four pinned call sites")
        endif()
        string(REGEX REPLACE "(vp8_loop_filter_row_(normal|simple)\\([^;]+\\);)"
            "{ const uint64_t gea_filter_started = gea_vp8_profile_clock_s3();\n\\1\ngea_vp8_profile_add_s3(GEA_VP8_PROFILE_LOOPFILTER, gea_filter_started); }"
            _gea_frame "${_gea_frame}")
    endif()
    if(NOT _gea_frame MATCHES "GEA_VP8_PROFILE_MACROBLOCK, gea_mb_started"
        OR NOT _gea_frame MATCHES "GEA_VP8_PROFILE_LOOPFILTER, gea_filter_started")
        message(FATAL_ERROR "libvpx frame profile patch did not match pinned source")
    endif()
    if(NOT _gea_frame_before STREQUAL _gea_frame)
        file(WRITE "${_gea_frame_source}" "${_gea_frame}")
    endif()
    target_sources(${_gea_vpx_target} PRIVATE i420_rgb565_s3.S vp8_filter_s3.S vp8_idct_s3.S
        vp8_loopfilter_s3.S vp8_loopfilter_s3.c vp8_s3.c vp8_intrapred_s3.S vp8_entropy_s3.c
        vp8_workspace_s3.c vp8_profile_s3.c vp8_copy_s3.S vp8_blocks_s3.c)
    target_compile_definitions(${_gea_vpx_target} PUBLIC GEA_VIDEO_SIMD_ESP32S3=1)
    target_include_directories(${_gea_vpx_target} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
    # Keep upstream scalar symbols for on-device differential tests. Select our
    # kernels at libvpx's normal dispatch boundary, not by patching call sites.
    set(_gea_vp8_rtcd "${gea_vpx_source_BINARY_DIR}/vp8_rtcd.h")
    file(READ "${_gea_vp8_rtcd}" _gea_vp8_dispatch)
    set(_gea_vp8_dispatch_before "${_gea_vp8_dispatch}")
    foreach(_size 16x16 8x8 8x4)
        set(_name "vp8_copy_mem${_size}")
        string(REPLACE "#define ${_name} ${_name}_c" "#define ${_name} ${_name}_s3"
            _gea_vp8_dispatch "${_gea_vp8_dispatch}")
        if(NOT _gea_vp8_dispatch MATCHES "#define ${_name} ${_name}_s3")
            message(FATAL_ERROR "libvpx copy dispatch missing: ${_name}")
        endif()
    endforeach()
    foreach(_kind sixtap bilinear)
        foreach(_size 4x4 8x4 8x8 16x16)
            set(_name "vp8_${_kind}_predict${_size}")
            string(REPLACE "#define ${_name} ${_name}_c" "#define ${_name} ${_name}_s3"
                _gea_vp8_dispatch "${_gea_vp8_dispatch}")
            if(NOT _gea_vp8_dispatch MATCHES "#define ${_name} ${_name}_s3")
                message(FATAL_ERROR "libvpx dispatch entry missing: ${_name}")
            endif()
        endforeach()
    endforeach()
    foreach(_edge bh bv mbh mbv)
        set(_name "vp8_loop_filter_${_edge}")
        string(REPLACE "#define ${_name} ${_name}_c" "#define ${_name} ${_name}_s3"
            _gea_vp8_dispatch "${_gea_vp8_dispatch}")
        if(NOT _gea_vp8_dispatch MATCHES "#define ${_name} ${_name}_s3")
            message(FATAL_ERROR "libvpx dispatch entry missing: ${_name}")
        endif()
    endforeach()
    set(_simple_c_names bhs bvs simple_horizontal_edge simple_vertical_edge)
    set(_simple_s3_names bh bv mbh mbv)
    foreach(_index RANGE 0 3)
        list(GET _simple_c_names ${_index} _c_name)
        list(GET _simple_s3_names ${_index} _s3_name)
        string(REPLACE "#define vp8_loop_filter_simple_${_s3_name} vp8_loop_filter_${_c_name}_c"
            "#define vp8_loop_filter_simple_${_s3_name} vp8_loop_filter_simple_${_s3_name}_s3"
            _gea_vp8_dispatch "${_gea_vp8_dispatch}")
        if(NOT _gea_vp8_dispatch MATCHES "#define vp8_loop_filter_simple_${_s3_name} vp8_loop_filter_simple_${_s3_name}_s3")
            message(FATAL_ERROR "libvpx simple filter dispatch entry missing: ${_s3_name}")
        endif()
    endforeach()
    foreach(_name vp8_short_idct4x4llm vp8_dequant_idct_add vp8_dc_only_idct_add vp8_short_inv_walsh4x4
        vp8_dequant_idct_add_y_block vp8_dequant_idct_add_uv_block)
        string(REPLACE "#define ${_name} ${_name}_c" "#define ${_name} ${_name}_s3"
            _gea_vp8_dispatch "${_gea_vp8_dispatch}")
        if(NOT _gea_vp8_dispatch MATCHES "#define ${_name} ${_name}_s3")
            message(FATAL_ERROR "libvpx transform dispatch entry missing: ${_name}")
        endif()
    endforeach()
    if(NOT _gea_vp8_dispatch MATCHES "#include \"vp8_s3.h\"")
        string(APPEND _gea_vp8_dispatch "\n#include \"vp8_s3.h\"\n")
    endif()
    if(NOT _gea_vp8_dispatch STREQUAL _gea_vp8_dispatch_before)
        file(WRITE "${_gea_vp8_rtcd}" "${_gea_vp8_dispatch}")
    endif()
    set(_gea_dsp_rtcd "${gea_vpx_source_BINARY_DIR}/vpx_dsp_rtcd.h")
    file(READ "${_gea_dsp_rtcd}" _gea_dsp_dispatch)
    set(_gea_dsp_before "${_gea_dsp_dispatch}")
    foreach(_size 4 8 16)
        set(_name "vpx_tm_predictor_${_size}x${_size}")
        string(REPLACE "#define ${_name} ${_name}_c" "#define ${_name} ${_name}_s3"
            _gea_dsp_dispatch "${_gea_dsp_dispatch}")
        if(NOT _gea_dsp_dispatch MATCHES "#define ${_name} ${_name}_s3")
            message(FATAL_ERROR "libvpx intra predictor dispatch missing: ${_name}")
        endif()
    endforeach()
    if(NOT _gea_dsp_dispatch MATCHES "#include \"vp8_s3.h\"")
        string(APPEND _gea_dsp_dispatch "\n#include \"vp8_s3.h\"\n")
    endif()
    if(NOT _gea_dsp_dispatch STREQUAL _gea_dsp_before)
        file(WRITE "${_gea_dsp_rtcd}" "${_gea_dsp_dispatch}")
    endif()
endif()
