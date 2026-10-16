#pragma once

// Override through the Mosaico board's package.json defines. Defaults match
// the H1 12 -> H2 12 and H1 11 <- H2 11 side-header link.
#ifndef GEA_MOSAICO_PEER_LEFT_TX_GPIO
#define GEA_MOSAICO_PEER_LEFT_TX_GPIO 15
#endif
#ifndef GEA_MOSAICO_PEER_LEFT_RX_GPIO
#define GEA_MOSAICO_PEER_LEFT_RX_GPIO 4
#endif
#ifndef GEA_MOSAICO_PEER_RIGHT_TX_GPIO
#define GEA_MOSAICO_PEER_RIGHT_TX_GPIO 5
#endif
#ifndef GEA_MOSAICO_PEER_RIGHT_RX_GPIO
#define GEA_MOSAICO_PEER_RIGHT_RX_GPIO 38
#endif

namespace gea::platform::board::peer {
struct PinPair {
  int tx;
  int rx;
};

inline constexpr PinPair left{GEA_MOSAICO_PEER_LEFT_TX_GPIO,
                              GEA_MOSAICO_PEER_LEFT_RX_GPIO};
inline constexpr PinPair right{GEA_MOSAICO_PEER_RIGHT_TX_GPIO,
                               GEA_MOSAICO_PEER_RIGHT_RX_GPIO};

// Only unclaimed V1.2 expansion GPIOs. In this target GPIO0/1 are the unused
// expansion I2C bus; the onboard touch/sensors use GPIO56/3. Exclude the
// codec, display, NAND, power controls, USB/JTAG and UART0 console pins.
constexpr bool isExpansionGpio(int pin) {
  switch (pin) {
  case 0:
  case 1:
  case 4:
  case 5:
  case 10:
  case 11:
  case 12:
  case 13:
  case 14:
  case 15:
  case 16:
  case 17:
  case 18:
  case 19:
  case 38:
  case 39:
  case 46:
  case 47:
  case 48:
  case 53:
  case 55:
    return true;
  default:
    return false;
  }
}

static_assert(isExpansionGpio(left.tx) && isExpansionGpio(left.rx) &&
                  isExpansionGpio(right.tx) && isExpansionGpio(right.rx),
              "Mosaico peer UART pins must be unclaimed expansion GPIOs");
static_assert(left.tx != left.rx && left.tx != right.tx &&
                  left.tx != right.rx && left.rx != right.tx &&
                  left.rx != right.rx && right.tx != right.rx,
              "Mosaico peer UART requires four distinct GPIOs");
} // namespace gea::platform::board::peer
