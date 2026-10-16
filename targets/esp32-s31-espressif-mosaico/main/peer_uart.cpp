#include "mosaico_peer_uart.h"
#include "mosaico_peer_uart_pins.h"
#include "platform/uart.h"

namespace peer = gea::platform::board::peer;
using gea::platform::uart::Uart;

extern "C" bool gea_board_uart_pin_allowed(int pin) {
#if GEA_MOSAICO_CAMERA_ENABLED
  return false;
#else
  return peer::isExpansionGpio(pin);
#endif
}

// Compatibility for native apps using the original side-link entry points.
// New applications configure controllers through the public Uart API.
extern "C" bool gea_mosaico_peer_uart_open(uint32_t baud) {
  if (baud < 115200 || baud > 5000000)
    return false;
  const bool leftWasOpen = Uart::isOpen(1);
  if (!Uart::open(1, {peer::left.tx, peer::left.rx, static_cast<int>(baud)}))
    return false;
  if (Uart::open(2, {peer::right.tx, peer::right.rx, static_cast<int>(baud)}))
    return true;
  if (!leftWasOpen)
    Uart::close(1);
  return false;
}
extern "C" void gea_mosaico_peer_uart_close() {
  Uart::close(1);
  Uart::close(2);
}
extern "C" int gea_mosaico_peer_uart_read(unsigned side, void *bytes,
                                          size_t capacity) {
  return side < 2 ? Uart::read(static_cast<int>(side) + 1, bytes, capacity)
                  : -1;
}
extern "C" int gea_mosaico_peer_uart_write(unsigned side, const void *bytes,
                                           size_t length) {
  return side < 2 ? Uart::write(static_cast<int>(side) + 1, bytes, length) : -1;
}
