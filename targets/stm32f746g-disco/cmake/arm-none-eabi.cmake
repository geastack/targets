# Bare-metal Cortex-M7 (STM32F746, single-precision FPU) with the Arm GNU
# toolchain. ARM_TOOLCHAIN_PATH pins a toolchain root; PATH is searched otherwise.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Absolute tool paths: helper scripts (the generated-archive builder) run
# the archiver outside CMake's own PATH lookup.
find_program(GEA_ARM_GCC arm-none-eabi-gcc HINTS "$ENV{ARM_TOOLCHAIN_PATH}/bin" REQUIRED)
get_filename_component(_GEA_ARM_BIN "${GEA_ARM_GCC}" DIRECTORY)
set(_GEA_ARM_PREFIX "${_GEA_ARM_BIN}/arm-none-eabi-")

set(CMAKE_C_COMPILER "${_GEA_ARM_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${_GEA_ARM_PREFIX}g++")
set(CMAKE_ASM_COMPILER "${_GEA_ARM_PREFIX}gcc")
set(CMAKE_AR "${_GEA_ARM_PREFIX}ar")
set(CMAKE_RANLIB "${_GEA_ARM_PREFIX}ranlib")
set(CMAKE_OBJCOPY "${_GEA_ARM_PREFIX}objcopy")
set(CMAKE_SIZE "${_GEA_ARM_PREFIX}size")

set(GEA_STM32_CPU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${GEA_STM32_CPU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${GEA_STM32_CPU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS_INIT "${GEA_STM32_CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${GEA_STM32_CPU_FLAGS} -Wl,--gc-sections")

# CMake defines $<LINK_LIBRARY:WHOLE_ARCHIVE,...> only for known platforms.
set(CMAKE_LINK_LIBRARY_USING_WHOLE_ARCHIVE "LINKER:--whole-archive" "<LINK_ITEM>" "LINKER:--no-whole-archive")
set(CMAKE_LINK_LIBRARY_USING_WHOLE_ARCHIVE_SUPPORTED TRUE)

set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
