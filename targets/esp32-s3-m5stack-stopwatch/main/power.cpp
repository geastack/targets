// SPDX-License-Identifier: Apache-2.0
#include "power.h"
#include "board.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/clock.h"
#include "i2c.h"
#include <algorithm>
#include <atomic>
#include <ctime>
#include <mutex>
#include <sys/time.h>

namespace {
i2c_master_dev_handle_t ioe = nullptr, pm = nullptr, rtc = nullptr;
std::mutex busMutex;
std::atomic<int64_t> motorUntil{0};
int filteredMv = 0;
int64_t nextBattery = 0;
int batteryLevel = -1;
bool externalPower = false;
bool ready = false;

bool attach(i2c_master_dev_handle_t& dev, int address) {
  auto bus =
      static_cast<i2c_master_bus_handle_t>(gea::platform::i2c::Bus::primary().nativeHandle());
  if (!bus) {
    return false;
  }
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = 100000;
  return i2c_master_bus_add_device(bus, &config, &dev) == ESP_OK;
}

bool read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, size_t count) {
  if (!dev) {
    return false;
  }
  bool ok = i2c_master_transmit_receive(dev, &reg, 1, data, count, 50) == ESP_OK;
  // PM1 / IOE firmware needs transaction processing time.
  if (dev == ioe || dev == pm) {
    vTaskDelay(1);
  }
  return ok;
}

bool write(i2c_master_dev_handle_t dev, uint8_t reg, const uint8_t* data, size_t count) {
  if (!dev || count > 8) {
    return false;
  }
  uint8_t bytes[9]{reg};
  std::copy(data, data + count, bytes + 1);
  bool ok = i2c_master_transmit(dev, bytes, count + 1, 50) == ESP_OK;
  if (dev == ioe || dev == pm) {
    vTaskDelay(1);
  }
  return ok;
}

uint16_t read16(i2c_master_dev_handle_t dev, uint8_t reg) {
  uint8_t data[2]{};
  read(dev, reg, data, 2);
  return data[0] | (data[1] << 8);
}

bool write16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t value) {
  uint8_t data[]{uint8_t(value), uint8_t(value >> 8)};
  return write(dev, reg, data, 2);
}

void pin(int bit, bool level) {
  uint16_t mask = 1 << bit;
  write16(ioe, 0x09, read16(ioe, 0x09) & ~mask);
  write16(ioe, 0x0B, read16(ioe, 0x0B) & ~mask);
  write16(ioe, 0x13, read16(ioe, 0x13) & ~mask);
  write16(ioe, 0x03, read16(ioe, 0x03) | mask);
  write16(ioe, 0x05, (read16(ioe, 0x05) & ~mask) | (level ? mask : 0));
}

int bcd(uint8_t n) {
  return (n >> 4) * 10 + (n & 15);
}

uint8_t packed(int n) {
  return ((n / 10) << 4) | (n % 10);
}

void readClock() {
  uint8_t d[7]{};
  if (!read(rtc, 0x10, d, 7)) {
    return;
  }
  tm time{};
  time.tm_sec = bcd(d[0] & 127);
  time.tm_min = bcd(d[1] & 127);
  time.tm_hour = bcd(d[2] & 63);
  time.tm_mday = bcd(d[4] & 63);
  time.tm_mon = bcd(d[5] & 31) - 1;
  time.tm_year = bcd(d[6]) + 100;
  if (time.tm_mday < 1 || time.tm_mon < 0 || time.tm_mon > 11 || time.tm_hour > 23) {
    return;
  }
  timeval value{mktime(&time), 0};
  settimeofday(&value, nullptr);
}

void updateBattery() {
  const auto now = esp_timer_get_time();
  if (now < nextBattery) {
    return;
  }
  nextBattery = now + 1000000;
  uint8_t bytes[2]{};
  if (read(pm, 0x22, bytes, 2)) {
    const int mv = bytes[0] | (bytes[1] << 8);
    filteredMv = filteredMv ? (filteredMv * 7 + mv + 4) / 8 : mv;
    batteryLevel = std::clamp((filteredMv - 3300) * 100 / 900, 0, 100);
  }
  if (read(pm, 0x24, bytes, 2)) {
    externalPower = (bytes[0] | (bytes[1] << 8)) > 4000;
  }
}

void motorTask(void*) {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(busMutex);
      if (motorUntil > 0 && esp_timer_get_time() >= motorUntil) {
        write16(ioe, 0x1B, 0x8000);
        motorUntil = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
} // namespace

namespace gea::platform::power {
bool Power::init() {
  std::lock_guard<std::mutex> lock(busMutex);
  if (ready) {
    return true;
  }
  gea::platform::board::prepareDisplayPanel();
  if (!attach(ioe, 0x4F) || !attach(pm, 0x6E) || !attach(rtc, 0x32)) {
    return false;
  }
  uint8_t identity[3]{};
  if (!read(ioe, 0, identity, 3)) {
    i2c_master_bus_rm_device(ioe);
    ioe = nullptr;
    if (!attach(ioe, 0x6F) || !read(ioe, 0, identity, 3)) {
      return false;
    }
  }
  pin(0, false);
  pin(2, true);
  pin(8, false);
  pin(9, false);
  gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_14, 0);
  write16(ioe, 0x25, 5000);
  write16(ioe, 0x1B, 0x8000);
  setenv("TZ", "UTC0", 1);
  tzset();
  readClock();
  updateBattery();
  if (xTaskCreate(motorTask, "stopwatch_motor", 3072, nullptr, 5, nullptr) != pdPASS) {
    return false;
  }
  ready = true;
  return true;
}

int Power::batteryPercent() {
  std::lock_guard<std::mutex> lock(busMutex);
  updateBattery();
  return batteryLevel;
}

bool Power::charging() {
  std::lock_guard<std::mutex> lock(busMutex);
  updateBattery();
  return externalPower;
}

bool Power::vibrate(double duration, int strength) {
  std::lock_guard<std::mutex> lock(busMutex);
  if (!ready) {
    return false;
  }
#if defined(GEA_EMBEDDED_COMPARISON_BENCHMARK) && GEA_EMBEDDED_COMPARISON_BENCHMARK
  const int duty = 0;
  (void)strength;
#else
  const int duty = duration > 0 && strength > 0 ? 25 + std::clamp(strength, 0, 100) * 75 / 100 : 0;
#endif
  const bool ok = write16(ioe, 0x1B, uint16_t(duty * 4095 / 100) | 0x8000);
  motorUntil = duty ? esp_timer_get_time() + int64_t(duration * 1000) : 0;
  return ok;
}
} // namespace gea::platform::power

namespace gea::platform::board {
bool setSpeakerPower(bool enabled) {
#if defined(GEA_EMBEDDED_COMPARISON_BENCHMARK) && GEA_EMBEDDED_COMPARISON_BENCHMARK
  enabled = false;
#endif
  std::lock_guard<std::mutex> lock(busMutex);
  if (!ready) {
    return false;
  }
  pin(9, enabled);
  gpio_set_level(GPIO_NUM_14, enabled ? 1 : 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  return true;
}
} // namespace gea::platform::board

namespace gea::platform::clock {
bool setEpochMs(double timestamp) {
  const time_t value = time_t(timestamp / 1000);
  tm time{};
  gmtime_r(&value, &time);
  if (time.tm_year < 100 || time.tm_year > 199) {
    return false;
  }
  std::lock_guard<std::mutex> lock(busMutex);
  uint8_t control = 0;
  if (!read(rtc, 0x1E, &control, 1)) {
    return false;
  }
  const uint8_t stopped = control | 0x40;
  if (!write(rtc, 0x1E, &stopped, 1)) {
    return false;
  }
  const uint8_t d[]{packed(time.tm_sec),       packed(time.tm_min),  packed(time.tm_hour),
                    packed(time.tm_wday),      packed(time.tm_mday), packed(time.tm_mon + 1),
                    packed(time.tm_year - 100)};
  bool ok = write(rtc, 0x10, d, 7);
  control &= ~0x40;
  ok = write(rtc, 0x1E, &control, 1) && ok;
  if (ok) {
    timeval now{value, 0};
    settimeofday(&now, nullptr);
  }
  return ok;
}
} // namespace gea::platform::clock
