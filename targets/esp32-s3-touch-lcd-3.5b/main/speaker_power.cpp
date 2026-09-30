#include <cstdint>
#include "board.h"
#include "i2c.h"
#include "driver/i2c_master.h"

namespace gea::platform::board {
// The amplifier's enable is EXIO7 on the TCA9554 at 0x20. Preserve every
// other output, including the panel reset and SD card signals.
bool setSpeakerPower(bool enabled) {
  static i2c_master_dev_handle_t device{};
  if (!device) {
    const auto bus = gea::platform::i2c::Bus::primary();
    if (!bus.available()) return false;
    i2c_device_config_t config{};
    config.device_address = 0x20;
    config.scl_speed_hz = 100000;
    if (i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device) != ESP_OK) return false;
  }
  const auto update = [](std::uint8_t reg, bool high) {
    std::uint8_t value{};
    if (i2c_master_transmit_receive(device, &reg, 1, &value, 1, 100) != ESP_OK) return false;
    const std::uint8_t bytes[]{reg, static_cast<std::uint8_t>(high ? value | 0x80 : value & ~0x80)};
    return i2c_master_transmit(device, bytes, sizeof(bytes), 100) == ESP_OK;
  };
  return update(1, enabled) && update(3, false);
}
}
