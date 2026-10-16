#pragma once
#include <cstddef>
using uart_port_t = int;
constexpr int ESP_OK = 0, UART_DATA_8_BITS = 8, UART_PARITY_DISABLE = 0;
constexpr int UART_STOP_BITS_1 = 1, UART_HW_FLOWCTRL_DISABLE = 0;
constexpr int UART_SCLK_DEFAULT = 1, UART_PIN_NO_CHANGE = -1;
struct uart_config_t {
  int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk;
};
bool uart_is_driver_installed(uart_port_t);
int uart_driver_install(uart_port_t, int, int, int, void*, int);
int uart_driver_delete(uart_port_t);
int uart_param_config(uart_port_t, const uart_config_t*);
int uart_set_pin(uart_port_t, int, int, int, int);
int uart_set_rx_full_threshold(uart_port_t, int);
int uart_set_rx_timeout(uart_port_t, int);
int uart_get_tx_buffer_free_size(uart_port_t, std::size_t*);
int uart_get_buffered_data_len(uart_port_t, std::size_t*);
int uart_read_bytes(uart_port_t, void*, std::size_t, int);
int uart_write_bytes(uart_port_t, const void*, std::size_t);
