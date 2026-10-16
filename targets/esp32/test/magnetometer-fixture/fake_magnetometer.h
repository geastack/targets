#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
using esp_err_t = int;
using i2c_master_bus_handle_t = void *;
inline constexpr int ESP_OK = 0;
inline constexpr int BMM150_OK = 0;
inline constexpr unsigned BMM150_REG_DATA_READY_STATUS = 0x48;
inline constexpr unsigned BMM150_DRDY_STATUS_MSK = 1;
inline constexpr unsigned BMM150_POWERMODE_NORMAL = 0;
inline constexpr unsigned BMM150_PRESETMODE_REGULAR = 2;
struct bmm150_dev {
  void *intf_ptr = nullptr;
};
struct bmm150_settings {
  unsigned pwr_mode = 0, preset_mode = 0;
};
#ifdef BMM150_USE_FLOATING_POINT
struct bmm150_mag_data {
  float x = 0, y = 0, z = 0;
};
#else
struct bmm150_mag_data {
  std::int16_t x = 0, y = 0, z = 0;
};
#endif
int bmm150_interface_init_from_master_bus(bmm150_dev *, void *, unsigned,
                                          unsigned);
int bmm150_interface_deinit_device(bmm150_dev *);
int bmm150_init(bmm150_dev *);
int bmm150_set_op_mode(bmm150_settings *, bmm150_dev *);
int bmm150_set_presetmode(bmm150_settings *, bmm150_dev *);
int bmm150_get_regs(unsigned, std::uint8_t *, unsigned, bmm150_dev *);
int bmm150_read_mag_data(bmm150_mag_data *, bmm150_dev *);
std::int64_t esp_timer_get_time();
void vTaskDelay(unsigned);
#define pdMS_TO_TICKS(ms) (ms)
namespace gea::platform::board {
inline constexpr std::array<std::uint8_t, 2> magnetometerAddresses{0x11, 0x12};
void powerOnRails();
} // namespace gea::platform::board
namespace gea::platform::i2c {
struct Bus {
  static Bus primary();
  bool available() const;
  void *nativeHandle() const;
};
} // namespace gea::platform::i2c
