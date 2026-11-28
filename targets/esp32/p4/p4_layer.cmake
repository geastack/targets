# ESP32-P4 chip-family layer, included by every P4 board's main/CMakeLists.txt
# after idf_component_register(). Holds only what is true of the P4 boards as a
# family; board hardware stays in each board's own directory.

# Primary I2C bus on controller 1 (esp32/board/i2c.cpp reads this).
target_compile_definitions(${COMPONENT_LIB} PRIVATE GEA_BOARD_PRIMARY_I2C_PORT=I2C_NUM_1)

# exFAT microSD: IDF has no Kconfig for FF_FS_EXFAT, so force-include the
# family's ffconf.h (FF_FS_EXFAT=1) into the fatfs component. Its
# `#ifndef _FFCONF_DEFINED` guard makes IDF's own ffconf.h a no-op.
if(TARGET idf::fatfs)
    idf_component_get_property(_GEA_FATFS_LIB fatfs COMPONENT_LIB)
    target_compile_options(${_GEA_FATFS_LIB} PRIVATE
        "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/fatfs_ffconf.h")
endif()
