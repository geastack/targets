cmake_minimum_required(VERSION 3.16)

# Resolve exactly what main/idf_component.yml pins. The component manager's
# "is there a newer version" pass reads registry metadata for releases this
# board never uses, and newer esp_hosted releases gate their USB-host transports
# on `$CONFIG{ESP_HOSTED_HOST_USB_*} == 1` rules that no Kconfig in the build
# defines -- which stops every fresh dependency solve at configure time.
set(ENV{IDF_COMPONENT_CHECK_NEW_VERSION} 0)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../esp32/components")
include("${CMAKE_CURRENT_LIST_DIR}/../esp32/app_project.cmake")
set(GEA_EMBEDDED_APP "tic-tac-toe" CACHE STRING "Example app to build (must match a package.json gea.id)")
set(PROJECT_VER "${GEA_EMBEDDED_APP}" CACHE STRING "Firmware app id" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
gea_solve_dependencies_in_build_dir()
project(gea_embedded)
