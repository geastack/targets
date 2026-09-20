set(GEA_EMBEDDED_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")
cmake_path(ABSOLUTE_PATH GEA_EMBEDDED_ROOT NORMALIZE)

include("${GEA_EMBEDDED_ROOT}/targets/esp32/gea_framework.cmake")

# None of the framework's own stable sources include an HTTP or OTA header:
# host/http.cpp, fetch.cpp, websocket.cpp and app_manager.cpp are app-sensitive
# and compile into `main`. These entries are here only so REQUIRES' transitivity
# reaches them without every board target repeating the list -- which means they
# must follow the app's network capability too, or an app that asks for no
# network still drags the HTTP server and the OTA writer into its image. The
# capability comes from the gea CLI (env, then the -D cache entry); a direct
# `idf.py` build leaves it unset and keeps the components, since the board is
# then the only thing that knows what the app needs.
set(GEA_EMBEDDED_FRAMEWORK_NETWORK_REQUIRES app_update esp_http_server)
if(DEFINED ENV{GEA_EMBEDDED_CAPABILITY_NETWORK}
        AND "$ENV{GEA_EMBEDDED_CAPABILITY_NETWORK}" STREQUAL "0")
    set(GEA_EMBEDDED_FRAMEWORK_NETWORK_REQUIRES "")
elseif(DEFINED CACHE{GEA_EMBEDDED_CAPABILITY_NETWORK}
        AND "$CACHE{GEA_EMBEDDED_CAPABILITY_NETWORK}" STREQUAL "0")
    set(GEA_EMBEDDED_FRAMEWORK_NETWORK_REQUIRES "")
endif()

idf_component_register(
    SRCS
        ${GEA_FW_CXX_SOURCES}
        ${GEA_FW_C_SOURCES}
    INCLUDE_DIRS
        ${GEA_FW_INCLUDE_DIRS}
    LDFRAGMENTS
        ${GEA_EMBEDDED_APP_LDFRAGMENT_FILES}
    REQUIRES
        esp_system
        esp_timer
        freertos
        ${GEA_EMBEDDED_FRAMEWORK_NETWORK_REQUIRES}
        esp_partition
        esp_app_format
        nvs_flash
        # targets/esp32/gpio.cpp: plain pins through esp_driver_gpio, and WS2812
        # bit timing clocked by an RMT TX channel rather than by the managed
        # `led_strip` component, which would land in the dependency lock of
        # every board that builds this file.
        esp_driver_gpio
        esp_driver_rmt
        # gea.targets.esp32.componentRequires: the components an app's own native
        # sources need headers or PUBLIC definitions from. REQUIRES is transitive,
        # so listing them here reaches the app sources compiled into `main` without
        # every board target having to know about them.
        ${GEA_EMBEDDED_APP_COMPONENT_REQUIRES}
)

# gpio.cpp has no caller inside the framework or `main` — only the app's own
# geatsc archive, which the linker reaches after every ESP-IDF component
# archive. Naming its anchor undefined from the start makes the linker take the
# object while esp_driver_rmt (and the gdma/soc archives behind it) are still
# ahead of the cursor; without it every rmt_* call is undefined at link. The
# flag travels through the link INTERFACE because this component is a static
# archive, so a PRIVATE option would stop at the archive.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl$<COMMA>-u$<COMMA>gea_gpio_link_bindings")

gea_apply_app_native_build()
