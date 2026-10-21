#include "power.h"

#include "i2c.h"
#include "memory_config.h"
#include "power/axp2101/axp2101.h"

#include <cstddef>
#include <cstdint>

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
#include "esp_attr.h"
#include "esp_heap_trace.h"
#include "esp_memory_utils.h"
#include "esp_rom_sys.h"
#endif
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if GEA_BOARD_HAS_BUTTONS
#include "input.h"
#include "services/buttons.h"
#endif

namespace {

constexpr const char *kTag = "axp2101";
constexpr int kI2cTimeoutMs = 100;

#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
constexpr std::size_t kPowerInitHeapTraceRecords = 256;
constexpr std::size_t kPowerInitHeapTracePrintMinBytes = 64;
constexpr std::uintptr_t kPowerInitHeapTraceFocusBegin = 0x3fcc0000;
constexpr std::uintptr_t kPowerInitHeapTraceFocusEnd = 0x3fce0000;

EXT_RAM_BSS_ATTR heap_trace_record_t powerInitHeapTraceRecords[kPowerInitHeapTraceRecords];
bool powerInitHeapTraceReady = false;

void printPowerInitHeapTracePcList(const char *prefix, const void *const *pcs)
{
	for (int i = 0; i < CONFIG_HEAP_TRACING_STACK_DEPTH; ++i) {
		if (!pcs[i]) break;
		esp_rom_printf(" %s%d=0x%08x",
		               prefix,
		               i,
		               static_cast<unsigned>(reinterpret_cast<std::uintptr_t>(pcs[i])));
	}
}

class PowerInitHeapTraceScope {
public:
	PowerInitHeapTraceScope()
	{
		if (!powerInitHeapTraceReady) {
			const esp_err_t init = heap_trace_init_standalone(powerInitHeapTraceRecords, kPowerInitHeapTraceRecords);
			powerInitHeapTraceReady = (init == ESP_OK);
			esp_rom_printf("[heap-trace] power_init init=%d records=%u\n",
			               static_cast<int>(init),
			               static_cast<unsigned>(kPowerInitHeapTraceRecords));
		}
		if (!powerInitHeapTraceReady) return;
		const esp_err_t start = heap_trace_start(HEAP_TRACE_ALL);
		active_ = (start == ESP_OK);
		esp_rom_printf("[heap-trace] power_init start=%d\n", static_cast<int>(start));
	}

	~PowerInitHeapTraceScope()
	{
		if (!active_) return;
		const esp_err_t stop = heap_trace_stop();
		heap_trace_summary_t summary{};
		const esp_err_t summaryErr = heap_trace_summary(&summary);
		esp_rom_printf(
			"[heap-trace] power_init stop=%d summary=%d mode=%d allocs=%u frees=%u count=%u capacity=%u high=%u overflow=%u\n",
			static_cast<int>(stop),
			static_cast<int>(summaryErr),
			static_cast<int>(summary.mode),
			static_cast<unsigned>(summary.total_allocations),
			static_cast<unsigned>(summary.total_frees),
			static_cast<unsigned>(summary.count),
			static_cast<unsigned>(summary.capacity),
			static_cast<unsigned>(summary.high_water_mark),
			static_cast<unsigned>(summary.has_overflowed));

		std::uint32_t internalEvents = 0;
		std::uint32_t printedEvents = 0;
		const size_t count = heap_trace_get_count();
		for (size_t i = 0; i < count; ++i) {
			heap_trace_record_t record{};
			if (heap_trace_get(i, &record) != ESP_OK || !record.address) continue;
			if (!esp_ptr_internal(record.address)) continue;
			internalEvents += 1;

			const auto address = reinterpret_cast<std::uintptr_t>(record.address);
			const bool inFocusRange = address >= kPowerInitHeapTraceFocusBegin && address < kPowerInitHeapTraceFocusEnd;
			if (record.size < kPowerInitHeapTracePrintMinBytes && !inFocusRange) continue;

			printedEvents += 1;
			esp_rom_printf("[heap-event] scope=power_init ptr=0x%08x size=%u freed=%u",
			               static_cast<unsigned>(address),
			               static_cast<unsigned>(record.size),
			               record.freed ? 1U : 0U);
			printPowerInitHeapTracePcList("pc", record.alloced_by);
			if (record.freed) printPowerInitHeapTracePcList("free", record.freed_by);
			esp_rom_printf("\n");
		}
		esp_rom_printf("[heap-trace] power_init internal_events=%u printed=%u min_print=%u focus=0x%08x-0x%08x\n",
		               static_cast<unsigned>(internalEvents),
		               static_cast<unsigned>(printedEvents),
		               static_cast<unsigned>(kPowerInitHeapTracePrintMinBytes),
		               static_cast<unsigned>(kPowerInitHeapTraceFocusBegin),
		               static_cast<unsigned>(kPowerInitHeapTraceFocusEnd));
	}

private:
	bool active_ = false;
};
#endif

class EspI2cRegisterBus final : public gea::chips::axp2101::RegisterBus {
public:
	bool attach() {
		if (device_) return true;

		auto bus = gea::platform::i2c::Bus::primary();
		if (!bus.available()) return false;

		i2c_device_config_t cfg = {};
		cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		cfg.device_address = gea::chips::axp2101::kI2cAddress;
		cfg.scl_speed_hz = gea::chips::axp2101::kI2cFrequencyHz;
		return i2c_master_bus_add_device(static_cast<i2c_master_bus_handle_t>(bus.nativeHandle()), &cfg, &device_) == ESP_OK;
	}

	bool writeRegister(std::uint8_t reg, std::uint8_t value) override {
		if (!attach()) return false;
		std::uint8_t payload[2] = {reg, value};
		return i2c_master_transmit(device_, payload, sizeof(payload), kI2cTimeoutMs) == ESP_OK;
	}

	bool readRegister(std::uint8_t reg, std::uint8_t &value) override {
		if (!attach()) return false;
		return i2c_master_transmit_receive(device_, &reg, 1, &value, 1, kI2cTimeoutMs) == ESP_OK;
	}

private:
	i2c_master_dev_handle_t device_ = nullptr;
};

class Axp2101Binding {
public:
	static Axp2101Binding &instance() {
		static Axp2101Binding binding;
		return binding;
	}

	bool init() {
		if (initialized_) return true;
		if (!bus_.attach()) return false;
		initialized_ = pmu_.begin();
		if (initialized_) ESP_LOGI(kTag, "measurement channels enabled");
		else ESP_LOGE(kTag, "measurement channel enable failed");
#if GEA_BOARD_HAS_BUTTONS
		if (initialized_) startPowerKey();
#endif
		return initialized_;
	}

	int batteryPercent() {
		if (!init()) return -1;
		return pmu_.batteryPercent();
	}

private:
	Axp2101Binding() : pmu_(bus_) {}

#if GEA_BOARD_HAS_BUTTONS
	// The power key is the board's Source::Pmu button, when it declares one.
	// The PMU's interrupt line is not wired to the ESP32 on these boards (the
	// vendor's own example polls it too), so its latched edge bits are read on
	// an interval -- the one button with no interrupt to wait on.
	static constexpr int kPowerKeyPollMs = 30;

	void startPowerKey() {
		if (powerKeyTask_) return;
		powerKeyButton_ = gea::platform::esp32::buttons::pmuButton();
		if (powerKeyButton_ < 0 || !pmu_.enablePowerKeyEdges()) return;
		if (xTaskCreate(powerKeyTask, "pmu_key", 2560, this, 4, &powerKeyTask_) != pdPASS) {
			powerKeyTask_ = nullptr;
			ESP_LOGE(kTag, "power key task unavailable");
			return;
		}
		ESP_LOGI(kTag, "PWR key is button %s (polled)", gea::framework::input::buttonCode(powerKeyButton_));
	}

	static void powerKeyTask(void *arg) {
		auto *self = static_cast<Axp2101Binding *>(arg);
		bool pressed = false;
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(kPowerKeyPollMs));
			bool fell = false;
			bool rose = false;
			if (!self->pmu_.takePowerKeyEdges(fell, rose) || (!fell && !rose)) continue;
			// Both latched means a whole edge pair fit between two reads: replay
			// them in the order the current state allows.
			for (int step = 0; step < 2; ++step) {
				if (!pressed && fell) {
					fell = false;
					pressed = true;
				} else if (pressed && rose) {
					rose = false;
					pressed = false;
				} else {
					continue;
				}
				gea::framework::input::queueButton(self->powerKeyButton_, pressed);
				ESP_LOGI(kTag, "PWR %s", pressed ? "down" : "up");
			}
		}
	}

	int powerKeyButton_ = -1;
	TaskHandle_t powerKeyTask_ = nullptr;
#endif

	EspI2cRegisterBus bus_;
	gea::chips::axp2101::PowerManagementUnit pmu_;
	bool initialized_ = false;
};

}  // namespace

bool gea::platform::power::Power::init() {
#if GEA_EMBEDDED_HEAP_DIAGNOSTICS_LOG && CONFIG_HEAP_TRACING_STANDALONE
	PowerInitHeapTraceScope heapTrace;
#endif
	return Axp2101Binding::instance().init();
}

int gea::platform::power::Power::batteryPercent() {
	return Axp2101Binding::instance().batteryPercent();
}
