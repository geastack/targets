#include "driver/uart.h"
#if defined(GEA_UART_TEST_TYPED_ARRAY)
#include "gea_runtime.h"
#endif
#include "host/uart.h"
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace {
struct Driver {
  bool installed = false;
  int tx = -1, rx = -1, baud = 0;
  std::size_t free = 4096;
  std::vector<std::uint8_t> received, sent;
};
std::array<Driver, 4> drivers;
struct Pin {
  bool output = false;
  bool input = false;
};
std::array<Pin, 64> pins;
bool failInstall = false, failPins = false;
unsigned writes = 0, pinResets = 0;
} // namespace
extern "C" bool gea_board_uart_pin_allowed(int pin) { return pin < 50; }
int gpio_reset_pin(int pin) {
  pins[pin] = {};
  ++pinResets;
  return 0;
}
bool uart_is_driver_installed(int port) { return drivers[port].installed; }
int uart_driver_install(int port, int, int txBytes, int, void *, int) {
  if (failInstall)
    return -1;
  drivers[port].installed = true;
  drivers[port].free = txBytes;
  return 0;
}
int uart_driver_delete(int port) {
  drivers[port] = {};
  return 0;
}
int uart_param_config(int port, const uart_config_t *config) {
  assert(config->data_bits == 8 && config->parity == 0 &&
         config->stop_bits == 1);
  drivers[port].baud = config->baud_rate;
  return 0;
}
int uart_set_pin(int port, int tx, int rx, int, int) {
  if (failPins)
    return -1;
  drivers[port].tx = tx;
  drivers[port].rx = rx;
  pins[tx].output = true;
  // IDF's GPIO-matrix RX route enables input without clearing an old output
  // route. Pads can retain that route across a software restart.
  pins[rx].input = true;
  return 0;
}
int uart_set_rx_full_threshold(int, int n) {
  assert(n == 96);
  return 0;
}
int uart_set_rx_timeout(int, int n) {
  assert(n == 2);
  return 0;
}
int uart_get_tx_buffer_free_size(int port, std::size_t *n) {
  *n = drivers[port].free;
  return 0;
}
int uart_get_buffered_data_len(int port, std::size_t *n) {
  *n = drivers[port].received.size();
  return 0;
}
int uart_read_bytes(int port, void *out, std::size_t capacity, int wait) {
  assert(wait == 0);
  auto &bytes = drivers[port].received;
  const auto n = std::min(capacity, bytes.size());
  if (n)
    std::memcpy(out, bytes.data(), n);
  bytes.erase(bytes.begin(), bytes.begin() + n);
  return static_cast<int>(n);
}
int uart_write_bytes(int port, const void *data, std::size_t n) {
  ++writes;
  const auto *bytes = static_cast<const std::uint8_t *>(data);
  drivers[port].sent.insert(drivers[port].sent.end(), bytes, bytes + n);
  drivers[port].free -= n + 32;
  return static_cast<int>(n);
}
int main() {
  const auto &uart = gea::host::Uart;
  assert(!uart.open(0, 1, 0, 5000000, 4096, 4096)); // console reserved
  for (double invalid : {-1., 4., 1.5, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()})
    assert(!uart.open(invalid, 1, 0, 5000000, 4096, 4096));
  assert(!uart.open(1, 50, 0, 5000000, 4096, 4096)); // board reservation
  assert(!uart.open(1, 1, 1, 5000000, 4096, 4096));
  assert(!uart.open(1, 1, 0, 5000001, 4096, 4096));
  assert(!uart.open(1, 1, 0, 5000000, 128, 4096));
  failInstall = true;
  const auto resetsBeforeInstall = pinResets;
  assert(!uart.open(1, 1, 0, 5000000, 4096, 4096));
  assert(pinResets == resetsBeforeInstall);
  failInstall = false;
  failPins = true;
  assert(!uart.open(1, 1, 0, 5000000, 4096, 4096));
  assert(!drivers[1].installed && !uart.isOpen(1));
  failPins = false;
  assert(uart.open(1, 1, 0, 5000000, 4096, 4096));
  const auto resetsBeforeIdempotentOpen = pinResets;
  assert(uart.open(1, 1, 0, 5000000, 4096, 4096)); // idempotent
  assert(pinResets == resetsBeforeIdempotentOpen);
  assert(
      !uart.open(1, 1, 0, 115200, 4096, 4096)); // cannot reconfigure live port
  assert(!uart.open(2, 46, 1, 5000000, 4096, 4096)); // overlapping route
  assert(uart.open(2, 46, 47, 5000000, 4096, 4096));
  assert(drivers[1].tx == 1 && drivers[1].rx == 0 && drivers[2].tx == 46 &&
         drivers[2].rx == 47);
  assert(drivers[1].baud == 5000000);
  std::array<std::uint8_t, 3> bytes{5, 6, 7};
  assert(uart.write(1, bytes) == 3 && uart.available(1) == 0);
  drivers[1].free = 34;
  assert(uart.writable(1) == 2 && uart.write(1, bytes) == 0 && writes == 1);
  drivers[2].received = {1, 2, 3, 4};
  assert(uart.available(2) == 4 && uart.read(2, bytes) == 3);
  assert((bytes == std::array<std::uint8_t, 3>{1, 2, 3}));
  assert(uart.available(2) == 1 && uart.read(2, bytes) == 1 && bytes[0] == 4);
  assert(uart.read(2, bytes) == 0);
#if defined(GEA_UART_TEST_TYPED_ARRAY)
  auto storage = gea::makeRef<gea::TypedArray<std::uint8_t>>(8);
  auto view = storage->subarray(2, 5);
  drivers[2].received = {21, 22, 23, 24};
  const auto &hostView = gea::detail::hostTypedArrayArgument(view);
  assert(uart.read(2, hostView) == 3);
  assert(storage->data()[1] == 0 && storage->data()[2] == 21 &&
         storage->data()[4] == 23 && storage->data()[5] == 0);
  assert(uart.available(2) == 1);
  assert(uart.write(2, hostView) == 3);
  assert((drivers[2].sent == std::vector<std::uint8_t>{21, 22, 23}));
  auto empty = storage->subarray(3, 3);
  assert(uart.read(2, gea::detail::hostTypedArrayArgument(empty)) == 0);
  assert(uart.available(2) == 1);
#endif
  uart.close(1);
  assert(!uart.isOpen(1) && uart.read(1, bytes) == -1 &&
         uart.write(1, bytes) == -1);
  assert(uart.isOpen(2));
  assert(uart.open(1, 1, 0, 115200, 4096, 4096));
  uart.close(1);
  uart.close(2);
  pins[1].output = true; // TX from a previous firmware, now requested as RX
  assert(uart.open(1, 4, 1, 5000000, 4096, 4096));
  assert(pins[4].output && pins[1].input && !pins[1].output);
  uart.close(1);
  drivers[3].installed = true; // owned by another native capability
  assert(!uart.open(3, 1, 0, 115200, 4096, 4096));
  uart.close(3);
  assert(drivers[3].installed); // cannot close somebody else's driver
  std::puts(
      "UART routing, ownership, rollback, byte I/O and backpressure passed");
}
