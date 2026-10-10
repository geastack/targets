#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "mosaico_peer_uart.h"
#include <array>
namespace {
// Digital side-header link profile. H1 pin 12 -> H2 pin 12 is right TX
// -> left RX; H1 pin 11 <- H2 pin 11 is right RX <- left TX. Connect only
// these signal contacts plus GND; this does not join output power rails.
struct Pins {
  uart_port_t uart;
  int tx, rx;
};
constexpr std::array<Pins, 2> pins{{{UART_NUM_1, 15, 4}, {UART_NUM_2, 5, 38}}};
bool opened = false;
} // namespace
extern "C" bool gea_mosaico_peer_uart_open(uint32_t baud) {
#if GEA_MOSAICO_CAMERA_ENABLED
  // Camera expansion owns these side GPIOs when that capability is enabled.
  return false;
#endif
  if (opened)
    return true;
  if (baud < 115200 || baud > 5000000)
    return false;
  // Never steal a UART already claimed by another board capability.
  for (const auto &p : pins)
    if (uart_is_driver_installed(p.uart))
      return false;
  unsigned installed = 0;
  for (const auto &p : pins) {
    uart_config_t config{};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    // RX driver storage and TX ring are reserved once, off the audio path.
    if (uart_param_config(p.uart, &config) != ESP_OK ||
        uart_set_pin(p.uart, p.tx, p.rx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK ||
        uart_driver_install(p.uart, 4096, 4096, 0, nullptr, 0) != ESP_OK) {
      for (unsigned i = 0; i < installed; ++i)
        uart_driver_delete(pins[i].uart);
      for (const auto &reset : pins) {
        gpio_reset_pin(static_cast<gpio_num_t>(reset.tx));
        gpio_reset_pin(static_cast<gpio_num_t>(reset.rx));
      }
      return false;
    }
    ++installed;
    uart_set_rx_full_threshold(p.uart, 96);
    uart_set_rx_timeout(p.uart, 2);
  }
  opened = true;
  return true;
}
extern "C" void gea_mosaico_peer_uart_close(void) {
  if (!opened)
    return;
  for (const auto &p : pins) {
    uart_driver_delete(p.uart);
    gpio_reset_pin(static_cast<gpio_num_t>(p.tx));
    gpio_reset_pin(static_cast<gpio_num_t>(p.rx));
  }
  opened = false;
}
extern "C" int gea_mosaico_peer_uart_read(unsigned side, void *bytes,
                                          size_t capacity) {
  if (!opened || side >= pins.size() || !bytes || !capacity)
    return -1;
  return uart_read_bytes(pins[side].uart, bytes, capacity, 0);
}
extern "C" int gea_mosaico_peer_uart_write(unsigned side, const void *bytes,
                                           size_t length) {
  if (!opened || side >= pins.size() || !bytes || !length)
    return -1;
  // The transport task owns this call. DSP only pushes its preallocated queue.
  size_t free = 0;
  if (uart_get_tx_buffer_free_size(pins[side].uart, &free) != ESP_OK ||
      free < length + 32)
    return 0;
  return uart_write_bytes(pins[side].uart, bytes, length);
}
