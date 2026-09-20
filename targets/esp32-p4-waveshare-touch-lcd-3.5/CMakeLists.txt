cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../esp32/components")
include("${CMAKE_CURRENT_LIST_DIR}/../esp32/app_project.cmake")
set(GEA_EMBEDDED_APP "tic-tac-toe" CACHE STRING "Example app to build (must match a package.json gea.id)")
set(PROJECT_VER "${GEA_EMBEDDED_APP}" CACHE STRING "Firmware app id" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(gea_embedded)

# Re-apply this board's tracked managed-component patches. The IDF component
# manager fetches dependencies into managed_components/ (gitignored, regenerated)
# during the project() call above — which silently wipes any hand-edits. This
# re-applies them from component-patches/*.patch after the fetch and before the
# components compile. Idempotent + non-fatal; see component-patches/apply.sh.
execute_process(
    COMMAND bash "${CMAKE_CURRENT_LIST_DIR}/component-patches/apply.sh" "${CMAKE_CURRENT_LIST_DIR}"
)
