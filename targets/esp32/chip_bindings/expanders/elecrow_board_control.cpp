// Elecrow CrowPanel Advance board control: the slow control lines (backlight,
// touch reset, amplifier mute and shutdown) sit behind one of two parts,
// depending on the board revision, and this binding probes which at init.
//
//   V1.0-V1.2: a PCA9557 I/O expander at 0x18. IO1 backlight enable, IO2 touch
//              reset, IO3 amplifier mute, IO4 amplifier shutdown (high = off).
//   V1.3+:     an STC8H1K28 board-control MCU at 0x30 that takes single-byte
//              writes with no register: 0..245 backlight (0 brightest, 245
//              off), 248 enable the speaker amplifier, 250 recover touch.
//              Elecrow documents no other command, so none is sent.
//
// The board definition names lines by the PCA9557's pin numbers; on an STC
// board each line maps onto the command that does the same job. Lines with no
// STC equivalent (mute, amplifier off) succeed without sending anything.

#include "chip_bindings/expanders/io_expander.h"
#include "expanders/pca9557/pca9557.h"
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
constexpr std::uint8_t kStcAddress = 0x30;
constexpr std::uint8_t kPcaAddress = 0x18;
constexpr int kLineBacklight = 1;
constexpr int kLineTouchReset = 2;
constexpr int kLineShutdown = 4;
constexpr std::uint8_t kStcBacklightOff = 245;
constexpr std::uint8_t kStcSpeakerOn = 248;
constexpr std::uint8_t kStcTouchRecover = 250;
constexpr int kTimeoutMs = 50;

enum class Controller { None, Stc, Pca9557 };

class ElecrowBoardControl final : public IoExpander, private gea::chips::pca9557::Bus {
public:
	bool init() override {
		Guard guard(lock_);
		if (controller_ != Controller::None) return true;
		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;
		auto handle = static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
		// Give the board-control MCU time to come up after power-on (Elecrow
		// waits 50 ms after starting the bus).
		vTaskDelay(pdMS_TO_TICKS(50));
		if (i2c_master_probe(handle, kStcAddress, kTimeoutMs) == ESP_OK && attach(handle, kStcAddress)) {
			controller_ = Controller::Stc;
			ESP_LOGI(kTag, "STC8H board-control MCU at 0x%02x (board V1.3+)", kStcAddress);
			return true;
		}
		if (i2c_master_probe(handle, kPcaAddress, kTimeoutMs) == ESP_OK && attach(handle, kPcaAddress) &&
		    pca_.configure(*this, static_cast<std::uint8_t>(gea::platform::board::expander.initialOutputs),
		                   static_cast<std::uint8_t>(gea::platform::board::expander.initialDirections))) {
			controller_ = Controller::Pca9557;
			ESP_LOGI(kTag, "PCA9557 expander at 0x%02x (board V1.0-V1.2)", kPcaAddress);
			return true;
		}
		ESP_LOGE(kTag, "neither the STC board MCU (0x30) nor the PCA9557 (0x18) answered");
		return false;
	}

	bool writePin(int pin, bool high) override {
		if (!init()) return false;
		Guard guard(lock_);
		if (controller_ == Controller::Pca9557) return pca_.setPin(*this, pin, high);
		switch (pin) {
			case kLineBacklight: return send(high ? levelCommand(backlightPercent_) : kStcBacklightOff);
			case kLineTouchReset: {
				if (high) return true;
				// Elecrow's recovery holds INT (GPIO1) low for 120 ms after the
				// command; gt911.cpp holds it across this call and ~110 ms more.
				const bool sent = send(kStcTouchRecover);
				vTaskDelay(pdMS_TO_TICKS(20));
				return sent;
			}
			case kLineShutdown: return high ? true : send(kStcSpeakerOn);
			default: return true;
		}
	}

	bool readPin(int pin, bool &high) override {
		if (!init()) return false;
		Guard guard(lock_);
		if (controller_ == Controller::Pca9557) return pca_.readPin(*this, pin, high);
		high = false;
		return false;
	}

	bool setInput(int pin, bool input) override {
		if (!init()) return false;
		Guard guard(lock_);
		if (controller_ == Controller::Pca9557) return pca_.setInput(*this, pin, input);
		return false;
	}

	bool setLevel(int pin, int percent) override {
		if (pin != kLineBacklight || !init()) return false;
		Guard guard(lock_);
		if (controller_ != Controller::Stc) return false;
		backlightPercent_ = std::clamp(percent, 0, 100);
		return send(backlightPercent_ > 0 ? levelCommand(backlightPercent_) : kStcBacklightOff);
	}

	const char *name() const {
		return controller_ == Controller::Stc ? "elecrow-stc8h" : controller_ == Controller::Pca9557 ? "pca9557" : "none";
	}

private:
	struct Guard {
		explicit Guard(SemaphoreHandle_t mutex) : mutex_(mutex) { xSemaphoreTakeRecursive(mutex_, portMAX_DELAY); }
		~Guard() { xSemaphoreGiveRecursive(mutex_); }
		SemaphoreHandle_t mutex_;
	};

	// 100% -> 0 (brightest), 1% -> 244 (dimmest that is still on).
	static std::uint8_t levelCommand(int percent) {
		percent = std::clamp(percent, 1, 100);
		return static_cast<std::uint8_t>(((100 - percent) * (kStcBacklightOff - 1)) / 99);
	}

	bool attach(i2c_master_bus_handle_t handle, std::uint8_t address) {
		i2c_device_config_t config{};
		config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		config.device_address = address;
		config.scl_speed_hz = 400000;
		return i2c_master_bus_add_device(handle, &config, &device_) == ESP_OK;
	}

	bool send(std::uint8_t command) {
		return i2c_master_transmit(device_, &command, 1, kTimeoutMs) == ESP_OK;
	}

	bool write(std::uint8_t reg, std::uint8_t value) override {
		const std::uint8_t bytes[] = {reg, value};
		return i2c_master_transmit(device_, bytes, 2, kTimeoutMs) == ESP_OK;
	}

	bool read(std::uint8_t reg, std::uint8_t &value) override {
		return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, kTimeoutMs) == ESP_OK;
	}

	StaticSemaphore_t mutexStorage_{};
	SemaphoreHandle_t lock_ = xSemaphoreCreateRecursiveMutexStatic(&mutexStorage_);
	i2c_master_dev_handle_t device_ = nullptr;
	gea::chips::pca9557::Driver pca_;
	Controller controller_ = Controller::None;
	int backlightPercent_ = 100;
};

ElecrowBoardControl &instance() {
	static ElecrowBoardControl driver;
	return driver;
}

}  // namespace

IoExpander &ioExpander() { return instance(); }
const char *ioExpanderDriverName() { return instance().name(); }
}  // namespace gea::platform::esp32::chip_bindings::expanders
