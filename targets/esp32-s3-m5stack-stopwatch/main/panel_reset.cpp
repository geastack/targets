// StopWatch routes AMOLED power/reset and touch reset through an M5IOE1.
// Pin names in the schematic are one-based; IOE1 register bits are zero-based.
// https://docs.m5stack.com/en/core/StopWatch
// https://github.com/m5stack/M5StopWatch-UserDemo/blob/main/main/hal/hal_ioe.cpp
// https://github.com/m5stack/M5IOE1/blob/main/src/M5IOE1.h
// https://github.com/m5stack/M5PM1/blob/main/src/M5PM1.h
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "panel_power.h"
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace {
constexpr char kTag[] = "stopwatch_power";
constexpr std::uint8_t kPm1Address = 0x6E;
constexpr std::uint8_t kPm1I2cConfig = 0x09;
constexpr std::uint8_t kPm1Watchdog = 0x0A;
constexpr std::uint8_t kIoeMode = 0x03;
constexpr std::uint8_t kIoeOutput = 0x05;
constexpr std::uint8_t kIoeInput = 0x07;
constexpr std::uint8_t kIoePullUp = 0x09;
constexpr std::uint8_t kIoePullDown = 0x0B;
constexpr std::uint8_t kIoeDrive = 0x13;
constexpr std::uint8_t kIoePowerPwm = 0x1D;
constexpr std::uint8_t kIoeI2cConfig = 0x23;
constexpr std::uint16_t kTouchReset = 1U << 3; // PIN_4
constexpr std::uint16_t kOledReset = 1U << 4;  // PIN_5
constexpr std::uint16_t kOledPower = 1U << 7;  // PIN_8, L3B_EN
constexpr std::uint16_t kResets = kTouchReset | kOledReset;
constexpr std::uint16_t kPanelPins = kResets | kOledPower;

std::mutex powerMutex;
i2c_master_dev_handle_t pm1 = nullptr;
i2c_master_dev_handle_t ioe = nullptr;
bool initialized = false;

void delayMs(std::uint32_t milliseconds) {
  vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

esp_err_t readRegisters(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t* bytes,
                        std::size_t size) {
  esp_err_t result = ESP_FAIL;
  for (int attempt = 0; attempt < 3; ++attempt) {
    result = i2c_master_transmit_receive(device, &reg, 1, bytes, size, 100);
    // IOE1/PM1 firmware needs processing time between transactions.
    esp_rom_delay_us(500);
    if (result == ESP_OK) {
      return result;
    }
    if (attempt < 2) {
      delayMs(50);
    }
  }
  return result;
}

esp_err_t writeRegisters(i2c_master_dev_handle_t device, const std::uint8_t* bytes,
                         std::size_t size) {
  esp_err_t result = ESP_FAIL;
  for (int attempt = 0; attempt < 3; ++attempt) {
    result = i2c_master_transmit(device, bytes, size, 100);
    esp_rom_delay_us(500);
    if (result == ESP_OK) {
      return result;
    }
    if (attempt < 2) {
      delayMs(50);
    }
  }
  return result;
}

std::uint8_t read8(i2c_master_dev_handle_t device, std::uint8_t reg) {
  std::uint8_t value = 0;
  ESP_ERROR_CHECK(readRegisters(device, reg, &value, 1));
  return value;
}

std::uint16_t read16(i2c_master_dev_handle_t device, std::uint8_t reg) {
  std::uint8_t bytes[2]{};
  ESP_ERROR_CHECK(readRegisters(device, reg, bytes, sizeof(bytes)));
  return static_cast<std::uint16_t>(bytes[0]) | (static_cast<std::uint16_t>(bytes[1]) << 8);
}

void write8(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint8_t value) {
  const std::uint8_t bytes[]{reg, value};
  ESP_ERROR_CHECK(writeRegisters(device, bytes, sizeof(bytes)));
  if (read8(device, reg) != value) {
    ESP_ERROR_CHECK(ESP_ERR_INVALID_RESPONSE);
  }
}

void write16(i2c_master_dev_handle_t device, std::uint8_t reg, std::uint16_t value) {
  const std::uint8_t bytes[]{reg, static_cast<std::uint8_t>(value),
                             static_cast<std::uint8_t>(value >> 8)};
  ESP_ERROR_CHECK(writeRegisters(device, bytes, sizeof(bytes)));
  if (read16(device, reg) != value) {
    ESP_ERROR_CHECK(ESP_ERR_INVALID_RESPONSE);
  }
}

void updateOutput(std::uint16_t mask, std::uint16_t value) {
  const std::uint16_t previous = read16(ioe, kIoeOutput);
  write16(ioe, kIoeOutput, (previous & ~mask) | (value & mask));
}

i2c_master_dev_handle_t attachDevice(i2c_master_bus_handle_t bus, std::uint8_t address) {
  // The first START wakes a sleeping PM1/IOE1 and may legitimately NACK.
  (void)i2c_master_probe(bus, address, 20);
  delayMs(10);
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = 100000;
  i2c_master_dev_handle_t device = nullptr;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &config, &device));
  return device;
}

void initializeLocked() {
  if (initialized) {
    return;
  }
  const auto sharedBus = gea::platform::i2c::Bus::primary();
  if (!sharedBus.available()) {
    ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
  }
  const auto bus = static_cast<i2c_master_bus_handle_t>(sharedBus.nativeHandle());

  pm1 = attachDevice(bus, kPm1Address);
  std::uint8_t pm1Identity[4]{};
  ESP_ERROR_CHECK(readRegisters(pm1, 0x00, pm1Identity, sizeof(pm1Identity)));
  // Factory firmware disables both I2C sleep and the PM1 watchdog. The
  // watchdog is independent of ESP-IDF's watchdog and otherwise resets us.
  for (int attempt = 0; attempt < 2; ++attempt) {
    write8(pm1, kPm1I2cConfig, read8(pm1, kPm1I2cConfig) & ~0x0FU);
  }
  write8(pm1, kPm1Watchdog, 0);

  std::uint8_t ioeIdentity[3]{};
  std::uint8_t ioeAddress = 0x4F;
  ioe = attachDevice(bus, ioeAddress);
  if (readRegisters(ioe, 0x00, ioeIdentity, sizeof(ioeIdentity)) != ESP_OK) {
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(ioe));
    ioeAddress = 0x6F;
    ioe = attachDevice(bus, ioeAddress);
    ESP_ERROR_CHECK(readRegisters(ioe, 0x00, ioeIdentity, sizeof(ioeIdentity)));
  }
  for (int attempt = 0; attempt < 2; ++attempt) {
    write8(ioe, kIoeI2cConfig, read8(ioe, kIoeI2cConfig) & ~0x0FU);
  }

  // PIN_8 can be PWM channel 2. A warm factory state must not override the
  // display's power latch with PWM, pull resistors or an open-drain output.
  write16(ioe, kIoePowerPwm, read16(ioe, kIoePowerPwm) & ~0x8000U);
  updateOutput(kPanelPins, kOledPower); // Hold resets low before driving pins.
  write16(ioe, kIoePullUp, read16(ioe, kIoePullUp) & ~kPanelPins);
  write16(ioe, kIoePullDown, read16(ioe, kIoePullDown) & ~kPanelPins);
  write16(ioe, kIoeDrive, read16(ioe, kIoeDrive) & ~kPanelPins);
  write16(ioe, kIoeMode, read16(ioe, kIoeMode) | kPanelPins);

  bool powered = false;
  for (int attempt = 0; attempt < 4; ++attempt) {
    delayMs(80);
    if ((read16(ioe, kIoeInput) & kOledPower) != 0) {
      powered = true;
      break;
    }
    updateOutput(kOledPower, kOledPower);
  }
  if (!powered) {
    ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
  }
  updateOutput(kResets, kResets);
  delayMs(50);
  initialized = true;
  ESP_LOGI(kTag, "PM1 watchdog disabled; IOE1 0x%02X AMOLED power and resets ready", ioeAddress);
}
} // namespace

namespace gea::platform::board {
void prepareDisplayPanel() {
  const std::lock_guard<std::mutex> guard(powerMutex);
  initializeLocked();
}

void resetTouchPanel() {
  const std::lock_guard<std::mutex> guard(powerMutex);
  initializeLocked();
  updateOutput(kTouchReset, 0);
  delayMs(10);
  updateOutput(kTouchReset, kTouchReset);
  delayMs(50);
}
} // namespace gea::platform::board
