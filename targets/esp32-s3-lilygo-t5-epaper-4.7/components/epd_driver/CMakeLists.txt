# ED047TC1 driver for the LilyGo T5 4.7" E-Paper S3. Our own esp_lcd i80
# implementation (gea_epd.c) of the epd_driver.h API, replacing the vendored
# LilyGo legacy I2S-in-LCD path (epd_driver.c / i2s_data_bus.c / ed047tc1.c /
# rmt_pulse.c), which does not port to the S3 / IDF 6. Those files are kept in
# the tree for reference but are NOT compiled; only their headers (epd_driver.h
# for the API + types, ed047tc1.h for the S3 pin map) are used.
idf_component_register(
    SRCS "gea_epd.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_driver_gpio esp_driver_i2c esp_timer
)

# epd_driver.h declares the draw entrypoints IRAM_ATTR; with -ffunction-sections
# GCC reports the declaration's .iram1.N section as "conflicting" with the
# definition's and, benignly, ignores the second (the function still lands in
# IRAM). -Werror turns that into a build stop, so relax just this warning.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=attributes)
