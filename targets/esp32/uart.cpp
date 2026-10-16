#include "platform/uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/soc_caps.h"
#include <array>
#include <limits>
#include <mutex>

// Board resource policy is separate from the chip's GPIO matrix capability.
extern "C" bool gea_board_uart_pin_allowed(int pin) __attribute__((weak));
extern "C" void gea_uart_link_bindings() {}

namespace {
struct Port {
  bool open = false;
  gea::platform::uart::Config config{-1, -1, 0};
};
std::array<Port, SOC_UART_HP_NUM> ports{};
std::mutex mutex;
bool validPort(int port) {
  return port >= 0 && port < static_cast<int>(ports.size());
}
bool usablePin(int pin, bool output) {
  if (pin < 0 || pin >= GPIO_NUM_MAX)
    return false;
  if (output ? !GPIO_IS_VALID_OUTPUT_GPIO(pin) : !GPIO_IS_VALID_GPIO(pin))
    return false;
  return !gea_board_uart_pin_allowed || gea_board_uart_pin_allowed(pin);
}
bool configured(int port) { return validPort(port) && ports[port].open; }
int writable(int port) {
  std::size_t bytes = 0;
  if (uart_get_tx_buffer_free_size(static_cast<uart_port_t>(port), &bytes) !=
      ESP_OK)
    return -1;
  // IDF's ring buffer also needs an item header and alignment for each write.
  return bytes > 32 ? static_cast<int>(bytes - 32) : 0;
}
} // namespace

extern "C" bool gea_uart_open(int port, int tx, int rx, int baud, int rxBytes,
                              int txBytes) {
  std::lock_guard lock(mutex);
  if (!validPort(port) || !usablePin(tx, true) || !usablePin(rx, false) ||
      tx == rx || baud < 1 || baud > 5000000 || rxBytes < 256 ||
      rxBytes > 65536 || txBytes < 256 || txBytes > 65536)
    return false;
#if defined(CONFIG_ESP_CONSOLE_UART_NUM)
  if (port == CONFIG_ESP_CONSOLE_UART_NUM)
    return false;
#endif
  const auto uart = static_cast<uart_port_t>(port);
  if (ports[port].open) {
    const auto &old = ports[port].config;
    return old.tx == tx && old.rx == rx && old.baudRate == baud &&
           old.rxBufferSize == rxBytes && old.txBufferSize == txBytes;
  }
  if (uart_is_driver_installed(uart))
    return false;
  for (const auto &other : ports) {
    if (other.open && (other.config.tx == tx || other.config.rx == tx ||
                       other.config.tx == rx || other.config.rx == rx))
      return false;
  }
  uart_config_t config{};
  config.baud_rate = baud;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  // Allocate before routing pins: a failed allocation cannot disturb the
  // pins.
  if (uart_driver_install(uart, rxBytes, txBytes, 0, nullptr, 0) != ESP_OK)
    return false;
  // IDF's matrix RX route enables input but deliberately preserves output
  // (it also supports shared TX/RX pads). Gea requires distinct pins, so clear
  // stale routes first, including a previous firmware's TX on our new RX pad.
  // Software restart can leave the pad configured after driver state is lost.
  if (uart_param_config(uart, &config) != ESP_OK ||
      gpio_reset_pin(static_cast<gpio_num_t>(tx)) != ESP_OK ||
      gpio_reset_pin(static_cast<gpio_num_t>(rx)) != ESP_OK ||
      uart_set_pin(uart, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
          ESP_OK ||
      uart_set_rx_full_threshold(uart, 96) != ESP_OK ||
      uart_set_rx_timeout(uart, 2) != ESP_OK) {
    uart_driver_delete(uart);
    gpio_reset_pin(static_cast<gpio_num_t>(tx));
    gpio_reset_pin(static_cast<gpio_num_t>(rx));
    return false;
  }
  ports[port] = {true, {tx, rx, baud, rxBytes, txBytes}};
  return true;
}
extern "C" bool gea_uart_is_open(int port) {
  std::lock_guard lock(mutex);
  return configured(port);
}
extern "C" void gea_uart_close(int port) {
  std::lock_guard lock(mutex);
  if (!configured(port))
    return;
  uart_driver_delete(static_cast<uart_port_t>(port));
  gpio_reset_pin(static_cast<gpio_num_t>(ports[port].config.tx));
  gpio_reset_pin(static_cast<gpio_num_t>(ports[port].config.rx));
  ports[port] = {};
}
extern "C" int gea_uart_read(int port, void *bytes, std::size_t capacity) {
  std::lock_guard lock(mutex);
  if (!configured(port) || (!bytes && capacity) ||
      capacity > std::numeric_limits<int>::max())
    return -1;
  return capacity ? uart_read_bytes(static_cast<uart_port_t>(port), bytes,
                                    capacity, 0)
                  : 0;
}
extern "C" int gea_uart_write(int port, const void *bytes, std::size_t length) {
  std::lock_guard lock(mutex);
  if (!configured(port) || (!bytes && length) ||
      length > std::numeric_limits<int>::max())
    return -1;
  if (!length)
    return 0;
  const int free = writable(port);
  if (free < 0)
    return -1;
  if (length > static_cast<std::size_t>(free))
    return 0;
  return uart_write_bytes(static_cast<uart_port_t>(port), bytes, length);
}
extern "C" int gea_uart_available(int port) {
  std::lock_guard lock(mutex);
  if (!configured(port))
    return -1;
  std::size_t bytes = 0;
  return uart_get_buffered_data_len(static_cast<uart_port_t>(port), &bytes) ==
                 ESP_OK
             ? static_cast<int>(bytes)
             : -1;
}
extern "C" int gea_uart_writable(int port) {
  std::lock_guard lock(mutex);
  return configured(port) ? writable(port) : -1;
}
