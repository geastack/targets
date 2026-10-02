#include "chip_bindings/expanders/io_expander.h"
#include "expanders/tca9554/tca9554.h"
#include "board.h"
#include "i2c.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace gea::platform::esp32::chip_bindings::expanders {
namespace {
class Tca9554 final : public IoExpander, private gea::chips::tca9554::Bus {
public:
  bool init() override {
    Guard guard(lock_);
    if (ready_) return true;
    auto bus = gea::platform::i2c::Bus::primary();
    if (!bus.available()) return false;
    if (!device_) {
      i2c_device_config_t config{};
      config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      config.device_address = gea::platform::board::expander.address;
      config.scl_speed_hz = 400000;
      if (i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &config, &device_) != ESP_OK) return false;
    }
    ready_ = driver_.configure(*this, gea::platform::board::expander.initialOutputs,
                              gea::platform::board::expander.initialDirections);
    return ready_;
  }

  bool writePin(int pin, bool high) override {
    if (!init()) return false;
    Guard guard(lock_);
    return driver_.setPin(*this, pin, high);
  }

  bool readPin(int pin, bool &high) override {
    if (!init()) return false;
    Guard guard(lock_);
    return driver_.readPin(*this, pin, high);
  }

  bool setInput(int pin, bool input) override {
    if (!init()) return false;
    Guard guard(lock_);
    return driver_.setInput(*this, pin, input);
  }

private:
  struct Guard {
    explicit Guard(SemaphoreHandle_t mutex) : mutex_(mutex) { xSemaphoreTake(mutex_, portMAX_DELAY); }
    ~Guard() { xSemaphoreGive(mutex_); }
    SemaphoreHandle_t mutex_;
  };

  bool write(std::uint8_t reg, std::uint8_t value) override {
    const std::uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(device_, bytes, 2, 100) == ESP_OK;
  }

  bool read(std::uint8_t reg, std::uint8_t &value) override {
    return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, 100) == ESP_OK;
  }

  StaticSemaphore_t mutexStorage_{};
  SemaphoreHandle_t lock_ = xSemaphoreCreateMutexStatic(&mutexStorage_);
  i2c_master_dev_handle_t device_ = nullptr;
  gea::chips::tca9554::Driver driver_;
  bool ready_ = false;
};
}  // namespace

IoExpander &ioExpander() { static Tca9554 driver; return driver; }
const char *ioExpanderDriverName() { return "tca9554"; }
}  // namespace gea::platform::esp32::chip_bindings::expanders
