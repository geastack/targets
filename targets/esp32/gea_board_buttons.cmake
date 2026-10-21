# The board's physical buttons come from JSON (the board's "buttons" plus the
# app's changes); the CLI generates gea_board_buttons.h and passes its
# directory. With none, the button service compiles to nothing.
function(gea_board_buttons target)
    set(_dir "${GEA_BOARD_BUTTONS_DIR}")
    if(NOT _dir AND DEFINED ENV{GEA_BOARD_BUTTONS_DIR})
        set(_dir "$ENV{GEA_BOARD_BUTTONS_DIR}")
    endif()
    if(_dir)
        target_include_directories(${target} PRIVATE "${_dir}")
        target_compile_definitions(${target} PRIVATE GEA_BOARD_HAS_BUTTONS=1)
    endif()
endfunction()
