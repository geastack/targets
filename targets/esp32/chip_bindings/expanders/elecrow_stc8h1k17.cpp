// Elecrow CrowPanel Advanced ESP32-P4 board control: an STC8H1K17 MCU at 0x2F
// owns the slow lines the P4 has no pins for. Unlike the S3 CrowPanels' STC
// (0x30, single-byte commands), it takes ordinary register writes:
//
//   0x00+i  read   battery struct, one byte at a time:
//                  { u32 adc mV, u32 battery mV, u8 level %, u8 charge state, u8 led state }
//   0x10+n  read   input n (0: the SPI/UART function DIP switch)
//   0x18+n  write  output n: 0 TP_RST (not fitted), 1 CSI_RST (camera reset),
//                  2 AUDIO_SD (amplifier shutdown, high = off), 3 LCD_BL_POWER
//                  (its switch is not fitted; the backlight is always powered)
//   0x20+n  write  PWM n, 0-100: 0 = backlight (MT9201 EN)
//
// Lines 0-3 are the outputs; line 4 is the backlight PWM, so a writePin on it
// switches the backlight and setLevel dims it.

#include "chip_bindings/expanders/elecrow_stc8h1k17.h"
#include "chip_bindings/expanders/io_expander.h"
#include "board.h"
#include "i2c.h"

#include <algorithm>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace gea::platform::esp32::chip_bindings::expanders {
namespace {

constexpr char kTag[] = "board_ctrl";
constexpr std::uint8_t kAddress = 0x2f;
constexpr std::uint8_t kRegBattery = 0x00;
constexpr std::uint8_t kRegInput = 0x10;
constexpr std::uint8_t kRegOutput = 0x18;
constexpr std::uint8_t kRegPwm = 0x20;
constexpr int kOutputCount = 4;
constexpr int kLineBacklight = 4;
constexpr int kTimeoutMs = 50;

class Stc8h1k17 final : public IoExpander {
public:
	bool init() override {
		Guard guard(lock_);
		if (ready_) return true;
		if (failed_) return false;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;
		auto handle = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
		// The STC comes out of reset with the P4 (its NRST is CHIP_PU); give
		// its firmware a moment to start answering.
		vTaskDelay(pdMS_TO_TICKS(50));
		esp_err_t err = ESP_FAIL;
		for (int attempt = 0; attempt < 5 && err != ESP_OK; ++attempt) {
			err = i2c_master_probe(handle, kAddress, kTimeoutMs);
			if (err != ESP_OK) vTaskDelay(pdMS_TO_TICKS(20));
		}
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "STC8H1K17 board-control MCU did not answer at 0x%02x", kAddress);
			failed_ = true;
			return false;
		}
		i2c_device_config_t config{};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = kAddress;
		config.scl_speed_hz = 100000;
		if (i2c_master_bus_add_device(handle, &config, &device_) != ESP_OK) {
			failed_ = true;
			return false;
		}
		ready_ = true;
		ESP_LOGI(kTag, "STC8H1K17 board-control MCU at 0x%02x", kAddress);
		return true;
	}

	bool writePin(int pin, bool high) override {
		if (!init()) return false;
		Guard guard(lock_);
		if (pin == kLineBacklight) return writeRegister(kRegPwm, high ? static_cast<std::uint8_t>(backlightPercent_) : 0);
		if (pin < 0 || pin >= kOutputCount) return false;
		return writeRegister(static_cast<std::uint8_t>(kRegOutput + pin), high ? 1 : 0);
	}

	bool readPin(int pin, bool &high) override {
		if (!init()) return false;
		Guard guard(lock_);
		std::uint8_t value = 0;
		if (pin != 0 || !readRegister(static_cast<std::uint8_t>(kRegInput + pin), value)) return false;
		high = value != 0;
		return true;
	}

	bool setLevel(int pin, int percent) override {
		if (pin != kLineBacklight || !init()) return false;
		Guard guard(lock_);
		backlightPercent_ = std::clamp(percent, 0, 100);
		return writeRegister(kRegPwm, static_cast<std::uint8_t>(backlightPercent_));
	}

	bool battery(Stc8h1k17Battery &out) {
		if (!init()) return false;
		Guard guard(lock_);
		std::uint8_t raw[11]{};
		for (std::uint8_t i = 0; i < sizeof(raw); ++i) {
			if (!readRegister(static_cast<std::uint8_t>(kRegBattery + i), raw[i])) return false;
		}
		auto u32 = [&](int at) {
			return static_cast<std::uint32_t>(raw[at]) | (static_cast<std::uint32_t>(raw[at + 1]) << 8) |
			       (static_cast<std::uint32_t>(raw[at + 2]) << 16) | (static_cast<std::uint32_t>(raw[at + 3]) << 24);
		};
		out.adcMillivolts = u32(0);
		out.batteryMillivolts = u32(4);
		out.levelPercent = raw[8];
		out.chargeState = raw[9];
		out.ledState = raw[10];
		return true;
	}

private:
	struct Guard {
		explicit Guard(SemaphoreHandle_t mutex) : mutex_(mutex) { xSemaphoreTakeRecursive(mutex_, portMAX_DELAY); }
		~Guard() { xSemaphoreGiveRecursive(mutex_); }
		SemaphoreHandle_t mutex_;
	};

	bool writeRegister(std::uint8_t reg, std::uint8_t value) {
		const std::uint8_t bytes[] = {reg, value};
		return i2c_master_transmit(device_, bytes, 2, kTimeoutMs) == ESP_OK;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) {
		return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, kTimeoutMs) == ESP_OK;
	}

	StaticSemaphore_t mutexStorage_{};
	SemaphoreHandle_t lock_ = xSemaphoreCreateRecursiveMutexStatic(&mutexStorage_);
	i2c_master_dev_handle_t device_ = nullptr;
	bool ready_ = false;
	bool failed_ = false;
	int backlightPercent_ = 100;
};

Stc8h1k17 &instance() {
	static Stc8h1k17 driver;
	return driver;
}

}  // namespace

IoExpander &ioExpander() { return instance(); }
const char *ioExpanderDriverName() { return "elecrow-stc8h1k17"; }
bool stc8h1k17Battery(Stc8h1k17Battery &out) { return instance().battery(out); }

}  // namespace gea::platform::esp32::chip_bindings::expanders
