// SPDX-License-Identifier: Apache-2.0
// The one wrapper around ESP-IDF's panic handler (-Wl,--wrap=esp_panic_handler,
// gea_framework.cmake). Whoever wants a crash's registers before the reset
// defines one of the weak hooks below instead of wrapping the handler again: a
// second --wrap of the same symbol does not chain.
//
//   gea_board_panic_record  the board (the Mosaico keeps them in LP registers)
//   gea_geapp_panic_record  Home (names the program and where it crashed)
//
// Both run inside the panic handler: no allocation, no locks, no logging.
#include "esp_private/panic_internal.h"

extern "C" {
void gea_board_panic_record(const panic_info_t *info) __attribute__((weak));
void gea_geapp_panic_record(const panic_info_t *info) __attribute__((weak));
void __real_esp_panic_handler(panic_info_t *info);

void __wrap_esp_panic_handler(panic_info_t *info)
{
	if (gea_board_panic_record) gea_board_panic_record(info);
	if (gea_geapp_panic_record) gea_geapp_panic_record(info);
	__real_esp_panic_handler(info);
}
}
