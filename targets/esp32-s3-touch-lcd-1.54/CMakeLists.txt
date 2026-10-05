cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../esp32/components")
include("${CMAKE_CURRENT_LIST_DIR}/../esp32/app_project.cmake")
set(GEA_EMBEDDED_APP "tic-tac-toe" CACHE STRING "Example app to build (must match a package.json gea.id)")
set(PROJECT_VER "${GEA_EMBEDDED_APP}" CACHE STRING "Firmware app id" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(gea_embedded)
