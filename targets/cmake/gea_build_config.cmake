# Resolved application choices come from the CLI, never from an app name.
# IDF's early requirements pass only receives the path through its child env.
foreach(_gea_input GEA_BUILD_CONFIG_FILE GEA_BUILD_CONFIG_JSON)
    if(NOT ${_gea_input} AND DEFINED ENV{${_gea_input}})
        set(${_gea_input} "$ENV{${_gea_input}}")
    endif()
endforeach()
if(GEA_BUILD_CONFIG_FILE)
    include("${GEA_BUILD_CONFIG_FILE}")
elseif(NOT CMAKE_SCRIPT_MODE_FILE)
    message(FATAL_ERROR "Resolved package.json build settings are missing. Build with a current gea CLI.")
endif()
