cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../esp32/components")
include("${CMAKE_CURRENT_LIST_DIR}/../esp32/app_project.cmake")
set(GEA_EMBEDDED_APP "tic-tac-toe" CACHE STRING "Example app to build (must match a package.json gea.id)")
set(PROJECT_VER "${GEA_EMBEDDED_APP}" CACHE STRING "Firmware app id" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
# Camera component patches use git apply --no-index against PROJECT_DIR.
# Inside a consumer Git checkout, discovery of its enclosing repository makes
# Git silently skip those project-relative paths and report success. Stop
# discovery at this target's parent while the SDK components configure.
set(GEA_CAMERA_PREVIOUS_GIT_CEILING "$ENV{GIT_CEILING_DIRECTORIES}")
get_filename_component(GEA_CAMERA_GIT_CEILING "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
set(ENV{GIT_CEILING_DIRECTORIES} "${GEA_CAMERA_GIT_CEILING}")
project(gea_embedded)
set(ENV{GIT_CEILING_DIRECTORIES} "${GEA_CAMERA_PREVIOUS_GIT_CEILING}")
