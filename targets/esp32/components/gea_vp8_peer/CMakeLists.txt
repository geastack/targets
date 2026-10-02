idf_component_register(SRCS vp8_peer.cpp INCLUDE_DIRS .
    REQUIRES esp_peer
    PRIV_REQUIRES gea_juice esp_libsrtp mbedtls esp_timer heap freertos)
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

if(NOT DEFINED ENV{GEA_HOST_DIR})
    message(FATAL_ERROR "The VP8 peer requires GEA_HOST_DIR from the Gea build")
endif()
target_include_directories(${COMPONENT_LIB} PRIVATE "$ENV{GEA_HOST_DIR}/include")
# esp_peer 1.5.5 exposes the DTLS/SRTP helper in its source component. Reuse
# its IDF-version-specific implementation and certificate/key lifecycle.
idf_component_get_property(peer_dir espressif__esp_peer COMPONENT_DIR)
target_include_directories(${COMPONENT_LIB} PRIVATE "${peer_dir}/src")
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
