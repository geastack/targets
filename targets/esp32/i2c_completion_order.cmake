# ESP-IDF 6.0.x wakes a synchronous I2C caller before copying the final RX
# bytes. On dual-core chips that caller can replace i2c_trans.ops while the
# ISR still uses it (observed with GT911 reads followed by an address probe).
# Compile a corrected build-local source, leaving the installed SDK untouched.
function(gea_fix_i2c_completion_order)
    if(NOT TARGET __idf_esp_driver_i2c)
        return()
    endif()

    get_target_property(component_dir __idf_esp_driver_i2c SOURCE_DIR)
    set(source "${component_dir}/i2c_master.c")
    file(READ "${source}" contents)
    set(notify [=[    if (i2c_master->event != I2C_EVENT_ALIVE) {
        xQueueSendFromISR(i2c_master->event_queue, (void *)&i2c_master->event, &HPTaskAwoken);
    }
]=])
    set(receive [=[    if (i2c_master->contains_read == true) {
        if (int_mask & I2C_LL_INTR_MST_COMPLETE || int_mask & I2C_LL_INTR_END_DETECT) {
            i2c_isr_receive_handler(i2c_master);
        }
    }
]=])
    string(FIND "${contents}" "${notify}${receive}" unsafe_order)
    if(unsafe_order EQUAL -1)
        # A newer SDK may already do the copy first. Do not rewrite a driver
        # whose implementation differs from the precisely identified defect.
        string(FIND "${contents}" "${receive}${notify}" fixed_order)
        if(fixed_order EQUAL -1)
            message(FATAL_ERROR "I2C completion ordering is unknown for this SDK; review i2c_master.c before using concurrent reads/probes")
        endif()
        return()
    endif()

    string(REPLACE "${notify}${receive}" "${receive}${notify}" corrected "${contents}")
    set(output "${CMAKE_BINARY_DIR}/gea_i2c_master.c")
    if(EXISTS "${output}")
        file(READ "${output}" previous)
    endif()
    if(NOT "${previous}" STREQUAL "${corrected}")
        file(WRITE "${output}" "${corrected}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source}")
    get_target_property(sources __idf_esp_driver_i2c SOURCES)
    set(replaced_sources)
    foreach(entry IN LISTS sources)
        if(entry STREQUAL "i2c_master.c" OR entry STREQUAL "${source}")
            list(APPEND replaced_sources "${output}")
        elseif(IS_ABSOLUTE "${entry}" OR entry MATCHES "^\\$<")
            list(APPEND replaced_sources "${entry}")
        else()
            list(APPEND replaced_sources "${component_dir}/${entry}")
        endif()
    endforeach()
    set_property(TARGET __idf_esp_driver_i2c PROPERTY SOURCES "${replaced_sources}")
    target_include_directories(__idf_esp_driver_i2c PRIVATE "${component_dir}")
    message(STATUS "I2C: finish RX copy before notifying synchronous callers")
endfunction()
