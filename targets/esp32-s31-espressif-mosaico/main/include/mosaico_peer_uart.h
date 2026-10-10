#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Target-owned GPIO/UART reservation. Side 0 is left, side 1 is right. This API
// carries digital bytes only; packet framing and DSP are application concerns.
// Explicit opt-in; unconfigured boards do not drive expansion pins.
bool gea_mosaico_peer_uart_open(uint32_t baud);
void gea_mosaico_peer_uart_close(void);
int gea_mosaico_peer_uart_read(unsigned side, void* bytes, size_t capacity);
int gea_mosaico_peer_uart_write(unsigned side, const void* bytes, size_t length);
#ifdef __cplusplus
}
#endif
