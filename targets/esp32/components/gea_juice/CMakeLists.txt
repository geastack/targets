cmake_minimum_required(VERSION 3.24)

idf_component_register(SRCS juice_memory.c INCLUDE_DIRS . PRIV_REQUIRES heap esp_netif mbedtls)
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

include(FetchContent)
FetchContent_Declare(gea_libjuice
    URL https://codeload.github.com/0x1abin/libjuice/tar.gz/34bae17c9c361639f64347da689503d3b06c0803
    URL_HASH SHA256=285a2461dc55804da7fe47ad48d58f2304c29d1f9fd644dee190a160416507b0
    SOURCE_SUBDIR gea-no-upstream-cmake
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(gea_libjuice)

# The fork's FreeRTOS adapter allocates task stacks in internal RAM and
# self-deletes. External-stack tasks require an explicit join/delete instead.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/patch_threads.py"
    "${gea_libjuice_SOURCE_DIR}/src/thread.h" COMMAND_ERROR_IS_FATAL ANY)

set(juice_sources)
foreach(name addr agent base64 conn conn_poll conn_thread conn_mux const_time crc32
        hash hmac ice juice log random server stun timestamp tcp turn udp compat)
    list(APPEND juice_sources "${gea_libjuice_SOURCE_DIR}/src/${name}.c")
endforeach()
target_sources(${COMPONENT_LIB} PRIVATE ${juice_sources})
target_include_directories(${COMPONENT_LIB} PUBLIC "${gea_libjuice_SOURCE_DIR}/include")
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${gea_libjuice_SOURCE_DIR}/src" "${gea_libjuice_SOURCE_DIR}/include/juice")
set_source_files_properties(${juice_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_CURRENT_LIST_DIR}/juice_memory.h")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    USE_NETTLE=0 USE_MBEDTLS=1 JUICE_STATIC JUICE_EXPORTS NO_SERVER
    JUICE_TASK_STACK_SIZE=16384
    udp_sendto=juice_udp_sendto udp_sendto_self=juice_udp_sendto_self
    udp_recvfrom=juice_udp_recvfrom udp_create_socket=juice_udp_create_socket
    udp_get_port=juice_udp_get_port udp_get_bound_addr=juice_udp_get_bound_addr
    udp_get_local_addr=juice_udp_get_local_addr udp_get_addrs=juice_udp_get_addrs
    udp_set_diffserv=juice_udp_set_diffserv
    agent_get_local_description=juice_agent_get_local_description
    agent_set_remote_description=juice_agent_set_remote_description
    agent_send=juice_agent_send
    hmac_sha1=juice_hmac_sha1 hmac_sha256=juice_hmac_sha256
    hash_md5=juice_hash_md5 hash_sha1=juice_hash_sha1 hash_sha256=juice_hash_sha256)
