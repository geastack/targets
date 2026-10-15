# Windows hosts: picolibc's specs reach its libstdc++ multilib headers through
# `<gcc>/bin/../<triple>/../picolibc/include/../<triple>/include/c++/...`.
# GCC opens that spelling unnormalised, and under a typical user profile the
# deepest header (bits/error_constants.h, pulled in by <cmath> on GCC 16)
# crosses Windows' 260-character MAX_PATH, so the include fails as "No such
# file". --picolibc-prefix makes the specs start from the toolchain root
# instead, which drops the bin/../<triple>/.. detour. Linking reads the same
# specs, and COMPILE_OPTIONS also reach the link line through CMAKE_CXX_FLAGS.
if(CMAKE_HOST_WIN32 AND NOT IDF_CUSTOM_TOOLCHAIN AND CONFIG_IDF_TOOLCHAIN_GCC AND CONFIG_LIBC_PICOLIBC)
    get_filename_component(_gea_toolchain_bin "${CMAKE_C_COMPILER}" DIRECTORY)
    get_filename_component(_gea_toolchain_root "${_gea_toolchain_bin}/.." ABSOLUTE)
    if(EXISTS "${_gea_toolchain_root}/picolibc/include")
        idf_toolchain_add_flags(COMPILE_OPTIONS "--picolibc-prefix=${_gea_toolchain_root}")
    endif()
endif()
