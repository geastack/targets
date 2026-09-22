# Shared enumeration of the files that feed the geatsc codegen DEPENDS lists.
#
# These used to be eight copies of an `execute_process(COMMAND find ...)` block, one per
# target. find(1) prunes node_modules / dist / .vite at the directory boundary, which is
# what keeps configure under a second (file(GLOB_RECURSE) stats every file first, half a
# minute next to a node_modules tree). Windows has no find(1): find.exe is a text filter,
# so the walk silently returned nothing and TSX edits never re-triggered codegen. The
# walk now lives in gea-find-files.mjs next to this file; node is already a build
# requirement.
#
# Knobs a target may set before including this file:
#   GEA_FIND_SOURCES_FOLLOW_SYMLINKS  ON (default) follows symlinks like `find -L`; a
#                                     pnpm-style tree may want OFF.
#   GEA_FIND_SOURCES_EXTRA_PRUNES     directory names pruned in addition to the defaults.
if(NOT DEFINED GEA_FIND_SOURCES_FOLLOW_SYMLINKS)
    set(GEA_FIND_SOURCES_FOLLOW_SYMLINKS ON)
endif()
set(_GEA_FIND_FILES_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/gea-find-files.mjs")
set(_GEA_FIND_DEFAULT_PRUNES
    node_modules dist .vite build .build .scratch .test-tmp generated-output
    ${GEA_FIND_SOURCES_EXTRA_PRUNES})

function(_gea_run_find_files OUT_VAR DIR)
    get_filename_component(_dir "${DIR}" REALPATH)
    set(_args "${_dir}")
    if(GEA_FIND_SOURCES_FOLLOW_SYMLINKS)
        list(APPEND _args --follow-symlinks)
    endif()
    execute_process(
        COMMAND node "${_GEA_FIND_FILES_SCRIPT}" ${_args} ${ARGN}
        OUTPUT_VARIABLE _output
        ERROR_VARIABLE _error
        RESULT_VARIABLE _result
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "gea-find-files failed for ${_dir}: ${_error}")
    endif()
    if(_output STREQUAL "")
        set(${OUT_VAR} "" PARENT_SCOPE)
    else()
        string(REPLACE "\n" ";" _list "${_output}")
        set(${OUT_VAR} "${_list}" PARENT_SCOPE)
    endif()
endfunction()

# Source files and configs under DIR that affect Gea compiler -> geatsc output.
# Pass PRUNE_VENDOR to also skip `vendor` directories (the upstream compiler tree).
function(_gea_find_sources OUT_VAR DIR)
    set(_prunes ${_GEA_FIND_DEFAULT_PRUNES})
    if(ARGC GREATER 2 AND "${ARGV2}" STREQUAL "PRUNE_VENDOR")
        list(APPEND _prunes vendor)
    endif()
    list(JOIN _prunes "," _prune_arg)
    _gea_run_find_files(_src "${DIR}"
        --prune "${_prune_arg}"
        --ext ".tsx,.ts,.jsx,.js,.css,.html"
        --name "package.json,tsconfig.json,vite.config.ts")
    set(${OUT_VAR} "${_src}" PARENT_SCOPE)
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
    list(JOIN _GEA_FIND_DEFAULT_PRUNES "," _prune_arg)
    _gea_run_find_files(_assets "${APP_DIR}"
        --prune "${_prune_arg}"
        --ext ".png,.jpg,.jpeg,.gif")
    set(${OUT_VAR} "${_assets}" PARENT_SCOPE)
endfunction()
