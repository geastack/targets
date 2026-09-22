// ESP-IDF binding for the CH422G I/O expander: adapts the new i2c_master API
// to gea::chips::ch422g::Bus. The chip answers at four fixed addresses (one per
// function), so the binding holds one device handle per address on the primary
// bus. Boot-time levels and the pins' roles come from board.h.

#include "chip_bindings/expanders/io_expander.h"

#include "board.h"
#include "expanders/ch422g/ch422g.h"
#include "i2c.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

constexpr char kTag[] = "ch422g";
constexpr TickType_t kI2cTimeoutTicks = pdMS_TO_TICKS(100);

#if GEA_BOARD_HAS_EXPANDER
constexpr std::uint8_t kInitialOutputs = static_cast<std::uint8_t>(gea::platform::board::expander.initialOutputs);
#else
constexpr std::uint8_t kInitialOutputs = gea::chips::ch422g::kDefaultIoLevels;
#endif

class EspI2cCommandBus final : public gea::chips::ch422g::Bus {
public:
	bool attach(i2c_master_bus_handle_t bus) {
		return attachAddress(bus, gea::chips::ch422g::kAddressWriteSystem, system_) &&
		       attachAddress(bus, gea::chips::ch422g::kAddressWriteOpenDrain, openDrain_) &&
		       attachAddress(bus, gea::chips::ch422g::kAddressWriteIo, io_) &&
		       attachAddress(bus, gea::chips::ch422g::kAddressReadIo, read_);
	}

	bool write(std::uint8_t address, std::uint8_t value) override {
		i2c_master_dev_handle_t device = handleFor(address);
		return device && i2c_master_transmit(device, &value, 1, kI2cTimeoutTicks) == ESP_OK;
	}

	bool read(std::uint8_t address, std::uint8_t &value) override {
		i2c_master_dev_handle_t device = handleFor(address);
		return device && i2c_master_receive(device, &value, 1, kI2cTimeoutTicks) == ESP_OK;
	}

private:
	static bool attachAddress(i2c_master_bus_handle_t bus, std::uint8_t address, i2c_master_dev_handle_t &out) {
		if (out) return true;
		i2c_device_config_t config = {};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = address;
		config.scl_speed_hz = gea::chips::ch422g::kI2cFrequencyHz;
		return i2c_master_bus_add_device(bus, &config, &out) == ESP_OK;
	}

	i2c_master_dev_handle_t handleFor(std::uint8_t address) const {
		if (address == gea::chips::ch422g::kAddressWriteSystem) return system_;
		if (address == gea::chips::ch422g::kAddressWriteOpenDrain) return openDrain_;
		if (address == gea::chips::ch422g::kAddressWriteIo) return io_;
		if (address == gea::chips::ch422g::kAddressReadIo) return read_;
		return nullptr;
	}

	i2c_master_dev_handle_t system_ = nullptr;
	i2c_master_dev_handle_t openDrain_ = nullptr;
	i2c_master_dev_handle_t io_ = nullptr;
	i2c_master_dev_handle_t read_ = nullptr;
};

class Ch422gExpander final : public gea::platform::esp32::chip_bindings::expanders::IoExpander {
public:
	bool init() override {
		if (attempted_) return ready_;
		attempted_ = true;
		lock_ = xSemaphoreCreateMutex();

		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) {
			ESP_LOGE(kTag, "I2C bus not ready");
			return false;
		}
		if (!bus_.attach(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()))) {
			ESP_LOGE(kTag, "failed to add CH422G devices");
			return false;
		}
		if (!driver_.configure(bus_, kInitialOutputs)) {
			ESP_LOGE(kTag, "CH422G did not acknowledge");
			return false;
		}
		ready_ = true;
		ESP_LOGI(kTag, "CH422G ready (IO0-7 outputs=0x%02x)", static_cast<unsigned>(kInitialOutputs));
		return true;
	}

	bool writePin(int pin, bool high) override {
		if (!init()) return false;
		Guard guard(lock_);
		return driver_.setPin(bus_, pin, high);
	}

	bool readPin(int pin, bool &high) override {
		if (!init()) return false;
		if (pin < 0 || pin >= gea::chips::ch422g::kIoPinCount) return false;
		Guard guard(lock_);
		std::uint8_t levels = 0;
		if (!driver_.readInputs(bus_, levels)) return false;
		high = (levels >> pin) & 1u;
		return true;
	}

private:
	struct Guard {
		explicit Guard(SemaphoreHandle_t lock) : lock_(lock) {
			if (lock_) xSemaphoreTake(lock_, portMAX_DELAY);
		}
		~Guard() {
			if (lock_) xSemaphoreGive(lock_);
		}
		SemaphoreHandle_t lock_;
	};

	EspI2cCommandBus bus_;
	gea::chips::ch422g::Driver driver_;
	SemaphoreHandle_t lock_ = nullptr;
	bool attempted_ = false;
	bool ready_ = false;
};

}  // namespace

namespace gea::platform::esp32::chip_bindings::expanders {

IoExpander &ioExpander() {
	static Ch422gExpander expander;
	return expander;
}

const char *ioExpanderDriverName() { return "ch422g"; }

}  // namespace gea::platform::esp32::chip_bindings::expanders
