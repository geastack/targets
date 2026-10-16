// SPDX-License-Identifier: Apache-2.0
// Uses Espressif's BSD-3-Clause BMM150 adapter on the existing Gea I2C bus.
// Integer compensation matches Espressif's Mosaico magnetic calibration;
// overflow sentinels are meaningful edge evidence and must not become zero.
#include "bmm150_common.h"
#include "board.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform/magnetometer.h"
#include <array>
#include <mutex>
#include <type_traits>

static_assert(std::is_same_v<decltype(bmm150_mag_data::x), std::int16_t>,
              "BMM150 and its consumers must use the same integer ABI");

namespace {
using gea::platform::sensors::MagnetometerReading;
constexpr auto kAddresses = gea::platform::board::magnetometerAddresses;
struct Sensor {
  bmm150_dev device{};
  bool initialized = false;
};
std::array<Sensor, kAddresses.size()> sensors;
std::mutex sensorMutex;

bool detach(Sensor &sensor) {
  sensor.initialized = false;
  // A failed removal retains the adapter context for a later close/open
  // retry. Never discard that pointer or delete the shared I2C bus.
  return !sensor.device.intf_ptr ||
         bmm150_interface_deinit_device(&sensor.device) == ESP_OK;
}

bool firstConversion(bmm150_dev &device) {
  for (unsigned elapsed = 0; elapsed <= 200; elapsed += 5) {
    std::uint8_t status = 0;
    if (bmm150_get_regs(BMM150_REG_DATA_READY_STATUS, &status, 1, &device) !=
        BMM150_OK)
      return false;
    if (status & BMM150_DRDY_STATUS_MSK)
      return true;
    if (elapsed < 200)
      vTaskDelay(pdMS_TO_TICKS(5));
  }
  return false;
}

bool readSensor(unsigned index, MagnetometerReading &out) {
  out = {};
  if (index >= sensors.size() || !sensors[index].initialized)
    return false;
  bmm150_mag_data reading{};
  if (bmm150_read_mag_data(&reading, &sensors[index].device) != BMM150_OK)
    return false;
  out.x = reading.x;
  out.y = reading.y;
  out.z = reading.z;
  const auto saturated = [](std::int16_t value) {
    return value <= -30000 || value >= 30000;
  };
  out.saturatedAxes = (saturated(out.x) ? 1U : 0U) |
                      (saturated(out.y) ? 2U : 0U) |
                      (saturated(out.z) ? 4U : 0U);
  out.valid = true;
  return true;
}
} // namespace

extern "C" unsigned gea_magnetometer_count() { return sensors.size(); }

extern "C" bool gea_magnetometer_open() {
  std::lock_guard lock(sensorMutex);
  bool complete = true;
  for (const auto &sensor : sensors)
    complete = complete && sensor.initialized;
  if (complete)
    return true;
  gea::platform::board::powerOnRails();
  const auto bus = gea::platform::i2c::Bus::primary();
  if (!bus.available())
    return false;
  for (unsigned index = 0; index < sensors.size(); ++index) {
    auto &sensor = sensors[index];
    if (sensor.initialized)
      continue;
    if (!detach(sensor) ||
        bmm150_interface_init_from_master_bus(
            &sensor.device,
            static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()),
            kAddresses[index], 400000) != ESP_OK)
      return false;
    bmm150_settings settings{};
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    if (bmm150_init(&sensor.device) != BMM150_OK ||
        bmm150_set_op_mode(&settings, &sensor.device) != BMM150_OK ||
        bmm150_set_presetmode(&settings, &sensor.device) != BMM150_OK ||
        !firstConversion(sensor.device)) {
      (void)detach(sensor);
      return false;
    }
    sensor.initialized = true;
  }
  return true;
}

extern "C" void gea_magnetometer_close() {
  std::lock_guard lock(sensorMutex);
  for (auto &sensor : sensors)
    (void)detach(sensor);
}

extern "C" bool gea_magnetometer_read(unsigned index,
                                      MagnetometerReading *out) {
  if (!out)
    return false;
  std::lock_guard lock(sensorMutex);
  const bool valid = readSensor(index, *out);
  out->timestampUs = static_cast<std::uint64_t>(esp_timer_get_time());
  return valid;
}

extern "C" bool gea_magnetometer_read_all(MagnetometerReading *out,
                                          std::size_t capacity) {
  if (!out || capacity < sensors.size())
    return false;
  std::lock_guard lock(sensorMutex);
  bool valid = true;
  for (unsigned index = 0; index < sensors.size(); ++index) {
    const bool current = readSensor(index, out[index]);
    valid = valid && current;
  }
  const auto timestamp = static_cast<std::uint64_t>(esp_timer_get_time());
  for (unsigned index = 0; index < sensors.size(); ++index)
    out[index].timestampUs = timestamp;
  return valid;
}
