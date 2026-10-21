// Waveshare panel bring-up, from the vendor's ESP-IDF examples
// (Examples/ESP-IDF/09_LVGL_V8_Test/main/main.cpp). The esp_lcd_axs15231b
// component itself sends SLPOUT, MADCTL and COLMOD before this list; the
// factory glass needs nothing beyond a second sleep-out and display-on.
#pragma once
static const axs15231b_lcd_init_cmd_t kBoardAxsInitCommands[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x29, (uint8_t[]){0x00}, 0, 100},
};
