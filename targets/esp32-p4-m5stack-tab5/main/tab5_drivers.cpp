#include "tab5_drivers.h"

#include "board.h"
#include "i2c.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace gea::platform::tab5 {

namespace {

constexpr const char *kTag = "tab5_drivers";
constexpr int kNativeWidth = 720;
constexpr int kNativeHeight = 1280;
constexpr int kDsiLaneCount = 2;
constexpr int kI2cTimeoutMs = 80;
constexpr int kTouchTimeoutMs = 20;
constexpr std::uint32_t kI2cSpeedHz = 100000;

constexpr std::uint8_t kPi4RegChipIdCtrl = 0x01;
constexpr std::uint8_t kPi4RegDirection = 0x03;
constexpr std::uint8_t kPi4RegOutput = 0x05;
constexpr std::uint8_t kPi4RegOutputHighIm = 0x07;
constexpr std::uint8_t kPi4RegInputDefault = 0x09;
constexpr std::uint8_t kPi4RegPullEnable = 0x0B;
constexpr std::uint8_t kPi4RegPullSelect = 0x0D;
constexpr std::uint8_t kPi4RegInput = 0x0F;
constexpr std::uint8_t kPi4RegIntMask = 0x11;

constexpr std::uint8_t kIo43 = 0x43;
constexpr std::uint8_t kIo44 = 0x44;
constexpr std::uint8_t kIo43SpkEn = 1;
constexpr std::uint8_t kIo43LcdRst = 4;
constexpr std::uint8_t kIo43TouchRst = 5;
constexpr std::uint8_t kIo43CamEn = 6;
constexpr std::uint8_t kIo44ChargeEn = 7;
constexpr std::uint8_t kIo44ChargeStat = 6;

constexpr std::uint8_t kIo43HpDet = 1u << 7;
constexpr std::uint8_t kIo43CamEnMask = 1u << 6;
constexpr std::uint8_t kIo43TpRst = 1u << 5;
constexpr std::uint8_t kIo43LcdRstMask = 1u << 4;
constexpr std::uint8_t kIo43Ext5vEn = 1u << 2;
constexpr std::uint8_t kIo43SpkEnMask = 1u << 1;
constexpr std::uint8_t kIo43Outputs = kIo43CamEnMask | kIo43TpRst | kIo43LcdRstMask | kIo43Ext5vEn | kIo43SpkEnMask;
constexpr std::uint8_t kIo43DefaultOutputs = kIo43Outputs & ~kIo43SpkEnMask;
constexpr std::uint8_t kIo43PullUps = kIo43Outputs;

constexpr std::uint8_t kIo44ChargeEnMask = 1u << 7;
constexpr std::uint8_t kIo44ChargeStatMask = 1u << 6;
constexpr std::uint8_t kIo44NoChargeQcEn = 1u << 5;
constexpr std::uint8_t kIo44PowerOffPulse = 1u << 4;
constexpr std::uint8_t kIo44Usb5vEn = 1u << 3;
constexpr std::uint8_t kIo44WlanPowerEn = 1u << 0;
constexpr std::uint8_t kIo44Outputs =
	kIo44ChargeEnMask | kIo44NoChargeQcEn | kIo44PowerOffPulse | kIo44Usb5vEn | kIo44WlanPowerEn;
constexpr std::uint8_t kIo44HighZ = (1u << 2) | (1u << 1);
constexpr std::uint8_t kIo44DefaultOutputs = kIo44WlanPowerEn | kIo44Usb5vEn;
constexpr std::uint8_t kIo44PullUps =
	kIo44Usb5vEn | kIo44WlanPowerEn | kIo44PowerOffPulse | kIo44NoChargeQcEn | kIo44ChargeEnMask;
constexpr std::uint8_t kIo44PullDowns = kIo44ChargeStatMask;

constexpr std::uint8_t kTouchGt911 = 0x14;
constexpr std::uint8_t kTouchSt7123 = 0x55;
constexpr std::uint8_t kBmi270 = 0x68;
constexpr std::uint8_t kIna226 = 0x41;

constexpr std::uint8_t kBmi270RegChipId = 0x00;
constexpr std::uint8_t kBmi270RegInternalStatus = 0x21;
constexpr std::uint8_t kBmi270RegAccX = 0x0c;
constexpr std::uint8_t kBmi270RegGyrX = 0x12;
constexpr std::uint8_t kBmi270RegAccConf = 0x40;
constexpr std::uint8_t kBmi270RegAccRange = 0x41;
constexpr std::uint8_t kBmi270RegGyrConf = 0x42;
constexpr std::uint8_t kBmi270RegGyrRange = 0x43;
constexpr std::uint8_t kBmi270RegInitCtrl = 0x59;
constexpr std::uint8_t kBmi270RegInitAddr0 = 0x5b;
constexpr std::uint8_t kBmi270RegInitData = 0x5e;
constexpr std::uint8_t kBmi270RegPwrConf = 0x7c;
constexpr std::uint8_t kBmi270RegPwrCtrl = 0x7d;
constexpr std::uint8_t kBmi270RegCmd = 0x7e;
constexpr std::uint8_t kBmi270ChipId = 0x24;
constexpr std::uint8_t kBmi270SoftReset = 0xb6;
constexpr std::uint8_t kBmi270InternalStatusInitOk = 0x01;
constexpr std::uint8_t kBmi270AccConf100HzNormal = 0xa8;
constexpr std::uint8_t kBmi270AccRange4g = 0x01;
constexpr std::uint8_t kBmi270GyrConf100HzNormal = 0xe8;
constexpr std::uint8_t kBmi270GyrRange2000Dps = 0x00;
constexpr std::uint8_t kBmi270PowerAccelGyroTemp = 0x0e;
constexpr std::size_t kBmi270ConfigChunkSize = 32;
constexpr int kBmi270ConfigTimeoutMs = 250;

// Bosch Sensortec BMI270 SensorAPI v2.86.1 maximum-FIFO configuration file.
// The BMI270 requires one published configuration payload after reset before
// raw accelerometer/gyroscope data becomes valid.
constexpr std::uint8_t kBmi270MaxFifoConfigFile[] = {
	0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
	0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
	0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5, 0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00,
	0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22, 0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f,
	0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50,
	0x00, 0x30, 0x12, 0x24, 0xeb, 0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f,
	0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5, 0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80,
	0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41, 0x33, 0x98, 0x2e, 0xc2, 0xc4,
	0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24, 0x00, 0xfc,
	0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe,
	0x94, 0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2,
	0xfe, 0x82, 0x05, 0x2f, 0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e,
	0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20, 0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a,
	0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08, 0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42,
	0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0, 0x6f, 0x00, 0x2e,
	0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
	0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30,
	0x23, 0x2e, 0x61, 0xf5, 0xeb, 0x2c, 0xe1, 0x6f,
};
static_assert(sizeof(kBmi270MaxFifoConfigFile) == 328);

struct DeviceSlot {
	i2c_master_dev_handle_t handle = nullptr;
};

std::mutex gI2cDeviceMutex;
std::mutex gI2cTransactionMutex;
DeviceSlot gDevices[128];
bool gIoExpandersReady = false;
std::uint8_t gIo43Output = kIo43DefaultOutputs;
std::uint8_t gIo44Output = kIo44DefaultOutputs;
DisplayController gDisplayController = DisplayController::Unknown;
LcdPanel gPanel;
bool gTouchReady = false;
bool gPowerReady = false;
bool gImuReady = false;
SemaphoreHandle_t gTouchReportSemaphore = nullptr;
bool gTouchInterruptInstalled = false;

i2c_master_bus_handle_t i2cBus()
{
	auto bus = gea::platform::i2c::Bus::primary();
	return static_cast<i2c_master_bus_handle_t>(bus.nativeHandle());
}

i2c_master_dev_handle_t i2cDevice(std::uint8_t address)
{
	if (address >= std::size(gDevices)) return nullptr;
	std::lock_guard<std::mutex> lock(gI2cDeviceMutex);
	if (gDevices[address].handle) return gDevices[address].handle;
	i2c_master_bus_handle_t bus = i2cBus();
	if (!bus) return nullptr;

	i2c_device_config_t config{};
	config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	config.device_address = address;
	config.scl_speed_hz = kI2cSpeedHz;
	config.scl_wait_us = 20000;
	if (i2c_master_bus_add_device(bus, &config, &gDevices[address].handle) != ESP_OK) {
		gDevices[address].handle = nullptr;
	}
	return gDevices[address].handle;
}

bool i2cProbe(std::uint8_t address, int timeoutMs = kI2cTimeoutMs)
{
	i2c_master_bus_handle_t bus = i2cBus();
	if (!bus) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	return i2c_master_probe(bus, address, timeoutMs) == ESP_OK;
}

bool i2cWrite(std::uint8_t address, const std::uint8_t *data, std::size_t size, int timeoutMs = kI2cTimeoutMs)
{
	i2c_master_dev_handle_t dev = i2cDevice(address);
	if (!dev) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	return i2c_master_transmit(dev, data, size, timeoutMs) == ESP_OK;
}

bool i2cRead(std::uint8_t address, std::uint8_t *data, std::size_t size, int timeoutMs = kI2cTimeoutMs)
{
	i2c_master_dev_handle_t dev = i2cDevice(address);
	if (!dev) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	return i2c_master_receive(dev, data, size, timeoutMs) == ESP_OK;
}

bool i2cReadReg8(std::uint8_t address, std::uint8_t reg, std::uint8_t *data, std::size_t size, int timeoutMs = kI2cTimeoutMs)
{
	i2c_master_dev_handle_t dev = i2cDevice(address);
	if (!dev) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	return i2c_master_transmit_receive(dev, &reg, 1, data, size, timeoutMs) == ESP_OK;
}

bool i2cWriteReg8(std::uint8_t address, std::uint8_t reg, std::uint8_t value, int timeoutMs = kI2cTimeoutMs)
{
	const std::uint8_t data[] = {reg, value};
	return i2cWrite(address, data, sizeof(data), timeoutMs);
}

bool i2cReadReg16(std::uint8_t address, std::uint16_t reg, std::uint8_t *data, std::size_t size, int timeoutMs = kI2cTimeoutMs)
{
	const std::uint8_t regBytes[] = {static_cast<std::uint8_t>(reg >> 8), static_cast<std::uint8_t>(reg & 0xff)};
	i2c_master_dev_handle_t dev = i2cDevice(address);
	if (!dev) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	return i2c_master_transmit_receive(dev, regBytes, sizeof(regBytes), data, size, timeoutMs) == ESP_OK;
}

bool i2cReadReg16Split(std::uint8_t address, std::uint16_t reg, std::uint8_t *data, std::size_t size, int timeoutMs = kI2cTimeoutMs)
{
	const std::uint8_t regBytes[] = {static_cast<std::uint8_t>(reg >> 8), static_cast<std::uint8_t>(reg & 0xff)};
	i2c_master_dev_handle_t dev = i2cDevice(address);
	if (!dev) return false;
	std::lock_guard<std::mutex> lock(gI2cTransactionMutex);
	if (i2c_master_transmit(dev, regBytes, sizeof(regBytes), timeoutMs) != ESP_OK) return false;
	return i2c_master_receive(dev, data, size, timeoutMs) == ESP_OK;
}

bool i2cWriteReg16(std::uint8_t address, std::uint16_t reg, std::uint8_t value, int timeoutMs = kI2cTimeoutMs)
{
	const std::uint8_t data[] = {
		static_cast<std::uint8_t>(reg >> 8),
		static_cast<std::uint8_t>(reg & 0xff),
		value,
	};
	return i2cWrite(address, data, sizeof(data), timeoutMs);
}

std::uint16_t be16(const std::uint8_t *data)
{
	return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::int16_t le16s(const std::uint8_t *data)
{
	return static_cast<std::int16_t>(static_cast<std::uint16_t>(data[0] | (static_cast<std::uint16_t>(data[1]) << 8)));
}

bool initExpander(std::uint8_t address,
                  std::uint8_t direction,
                  std::uint8_t output,
                  std::uint8_t highZ,
                  std::uint8_t pullUps,
                  std::uint8_t pullDowns)
{
	i2cWriteReg8(address, kPi4RegChipIdCtrl, 0x01);
	vTaskDelay(pdMS_TO_TICKS(2));
	if (!i2cWriteReg8(address, kPi4RegOutput, output)) return false;
	if (!i2cWriteReg8(address, kPi4RegOutputHighIm, highZ)) return false;
	if (!i2cWriteReg8(address, kPi4RegPullEnable, pullUps | pullDowns)) return false;
	if (!i2cWriteReg8(address, kPi4RegPullSelect, pullUps)) return false;
	if (!i2cWriteReg8(address, kPi4RegDirection, direction)) return false;
	if (!i2cWriteReg8(address, kPi4RegInputDefault, 0x00)) return false;
	if (!i2cWriteReg8(address, kPi4RegIntMask, 0xff)) return false;
	return true;
}

bool setIoPin(std::uint8_t address, std::uint8_t bit, bool level)
{
	if (!initIoExpanders()) return false;
	std::uint8_t *output = address == kIo43 ? &gIo43Output : address == kIo44 ? &gIo44Output : nullptr;
	if (!output) return false;
	if (level) *output |= (1u << bit);
	else *output &= static_cast<std::uint8_t>(~(1u << bit));
	return i2cWriteReg8(address, kPi4RegOutput, *output);
}

bool getIoPin(std::uint8_t address, std::uint8_t bit, bool *level)
{
	std::uint8_t value = 0;
	if (!level || !initIoExpanders()) return false;
	if (!i2cReadReg8(address, kPi4RegInput, &value, 1)) return false;
	*level = ((value >> bit) & 1u) != 0;
	return true;
}

bool lcdReset(bool assertReset)
{
	return setIoPin(kIo43, kIo43LcdRst, !assertReset);
}

bool touchReset(bool assertReset)
{
	return setIoPin(kIo43, kIo43TouchRst, !assertReset);
}

void touchReportIsr(void *)
{
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	if (gTouchReportSemaphore) {
		xSemaphoreGiveFromISR(gTouchReportSemaphore, &higherPriorityTaskWoken);
	}
	if (higherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

bool configureTouchInterrupt()
{
	if (!gTouchReportSemaphore) {
		gTouchReportSemaphore = xSemaphoreCreateBinary();
		if (!gTouchReportSemaphore) return false;
	}
	xSemaphoreTake(gTouchReportSemaphore, 0);

	gpio_config_t config{};
	config.pin_bit_mask = 1ULL << gea::platform::board::touch.interrupt;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_NEGEDGE;
	esp_err_t err = gpio_config(&config);
	if (err != ESP_OK) {
		ESP_LOGW(kTag, "TP_INT GPIO config failed: %s", esp_err_to_name(err));
		return false;
	}

	err = gpio_install_isr_service(0);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGW(kTag, "GPIO ISR service install failed: %s", esp_err_to_name(err));
		return false;
	}
	if (!gTouchInterruptInstalled) {
		err = gpio_isr_handler_add(gea::platform::board::touch.interrupt, touchReportIsr, nullptr);
		if (err != ESP_OK) {
			ESP_LOGW(kTag, "TP_INT ISR install failed: %s", esp_err_to_name(err));
			return false;
		}
		gTouchInterruptInstalled = true;
	}
	gpio_intr_enable(gea::platform::board::touch.interrupt);
	return true;
}

bool sendDsiCommand(esp_lcd_panel_io_handle_t io, std::uint8_t command, std::initializer_list<std::uint8_t> data, int delayMs = 0)
{
	const void *payload = data.size() ? data.begin() : nullptr;
	const esp_err_t err = esp_lcd_panel_io_tx_param(io, command, payload, data.size());
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "DSI command 0x%02x failed: %s", command, esp_err_to_name(err));
		return false;
	}
	if (delayMs > 0) vTaskDelay(pdMS_TO_TICKS(delayMs));
	return true;
}

bool initSt7123(esp_lcd_panel_io_handle_t io)
{
	auto cmd = [&](std::uint8_t c, std::initializer_list<std::uint8_t> data, int delay = 0) {
		return sendDsiCommand(io, c, data, delay);
	};

	return cmd(0x60, {0x71, 0x23, 0xa2}) &&
	       cmd(0x60, {0x71, 0x23, 0xa3}) &&
	       cmd(0x60, {0x71, 0x23, 0xa4}) &&
	       cmd(0xA4, {0x31}) &&
	       cmd(0xD7, {0x10, 0x0A, 0x10, 0x2A, 0x80, 0x80}) &&
	       cmd(0x90, {0x71, 0x23, 0x5A, 0x20, 0x24, 0x09, 0x09}) &&
	       cmd(0xA3, {0x80, 0x01, 0x88, 0x30, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00,
	                  0x1E, 0x5C, 0x1E, 0x80, 0x00, 0x4F, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46,
	                  0x00, 0x00, 0x1E, 0x5C, 0x1E, 0x80, 0x00, 0x6F, 0x58, 0x00, 0x00, 0x00, 0xFF}) &&
	       cmd(0xA6, {0x03, 0x00, 0x24, 0x55, 0x36, 0x00, 0x39, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0x24,
	                  0x55, 0x38, 0x00, 0x37, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0x24, 0x11, 0x00, 0x00,
	                  0x00, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x00, 0xEC, 0x11, 0x00, 0x03, 0x00, 0x03, 0x6E,
	                  0x6E, 0xFF, 0xFF, 0x00, 0x08, 0x80, 0x08, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00}) &&
	       cmd(0xA7, {0x19, 0x19, 0x80, 0x64, 0x40, 0x07, 0x16, 0x40, 0x00, 0x44, 0x03, 0x6E, 0x6E, 0x91, 0xFF,
	                  0x08, 0x80, 0x64, 0x40, 0x25, 0x34, 0x40, 0x00, 0x02, 0x01, 0x6E, 0x6E, 0x91, 0xFF, 0x08,
	                  0x80, 0x64, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x6E, 0x6E, 0x91, 0xFF, 0x08, 0x80,
	                  0x64, 0x40, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x6E, 0x6E, 0x84, 0xFF, 0x08, 0x80, 0x44}) &&
	       cmd(0xAC, {0x03, 0x19, 0x19, 0x18, 0x18, 0x06, 0x13, 0x13, 0x11, 0x11, 0x08, 0x08, 0x0A, 0x0A, 0x1C,
	                  0x1C, 0x07, 0x07, 0x00, 0x00, 0x02, 0x02, 0x01, 0x19, 0x19, 0x18, 0x18, 0x06, 0x12, 0x12,
	                  0x10, 0x10, 0x09, 0x09, 0x0B, 0x0B, 0x1C, 0x1C, 0x07, 0x07, 0x03, 0x03, 0x01, 0x01}) &&
	       cmd(0xAD, {0xF0, 0x00, 0x46, 0x00, 0x03, 0x50, 0x50, 0xFF, 0xFF, 0xF0, 0x40, 0x06, 0x01,
	                  0x07, 0x42, 0x42, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}) &&
	       cmd(0xAE, {0xFE, 0x3F, 0x3F, 0xFE, 0x3F, 0x3F, 0x00}) &&
	       cmd(0xB2, {0x15, 0x19, 0x05, 0x23, 0x49, 0xAF, 0x03, 0x2E, 0x5C, 0xD2, 0xFF, 0x10, 0x20, 0xFD, 0x20,
	                  0xC0, 0x00}) &&
	       cmd(0xE8, {0x20, 0x6F, 0x04, 0x97, 0x97, 0x3E, 0x04, 0xDC, 0xDC, 0x3E, 0x06, 0xFA, 0x26, 0x3E}) &&
	       cmd(0x75, {0x03, 0x04}) &&
	       cmd(0xE7, {0x3B, 0x00, 0x00, 0x7C, 0xA1, 0x8C, 0x20, 0x1A, 0xF0, 0xB1, 0x50, 0x00,
	                  0x50, 0xB1, 0x50, 0xB1, 0x50, 0xD8, 0x00, 0x55, 0x00, 0xB1, 0x00, 0x45,
	                  0xC9, 0x6A, 0xFF, 0x5A, 0xD8, 0x18, 0x88, 0x15, 0xB1, 0x01, 0x01, 0x77}) &&
	       cmd(0xEA, {0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x2C}) &&
	       cmd(0xB0, {0x22, 0x43, 0x11, 0x61, 0x25, 0x43, 0x43}) &&
	       cmd(0xB7, {0x00, 0x00, 0x73, 0x73}) &&
	       cmd(0xBF, {0xA6, 0xAA}) &&
	       cmd(0xA9, {0x00, 0x00, 0x73, 0xFF, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03}) &&
	       cmd(0xC8, {0x00, 0x00, 0x10, 0x1F, 0x36, 0x00, 0x5D, 0x04, 0x9D, 0x05, 0x10, 0xF2, 0x06,
	                  0x60, 0x03, 0x11, 0xAD, 0x00, 0xEF, 0x01, 0x22, 0x2E, 0x0E, 0x74, 0x08, 0x32,
	                  0xDC, 0x09, 0x33, 0x0F, 0xF3, 0x77, 0x0D, 0xB0, 0xDC, 0x03, 0xFF}) &&
	       cmd(0xC9, {0x00, 0x00, 0x10, 0x1F, 0x36, 0x00, 0x5D, 0x04, 0x9D, 0x05, 0x10, 0xF2, 0x06,
	                  0x60, 0x03, 0x11, 0xAD, 0x00, 0xEF, 0x01, 0x22, 0x2E, 0x0E, 0x74, 0x08, 0x32,
	                  0xDC, 0x09, 0x33, 0x0F, 0xF3, 0x77, 0x0D, 0xB0, 0xDC, 0x03, 0xFF}) &&
	       cmd(0x11, {}, 100) &&
	       cmd(0x29, {}) &&
	       cmd(0x35, {0x00}, 100) &&
	       cmd(0x36, {0x00}) &&
	       cmd(0x3A, {0x55});
}

bool initIli9881c(esp_lcd_panel_io_handle_t io)
{
	auto cmd = [&](std::uint8_t c, std::initializer_list<std::uint8_t> data, int delay = 0) {
		return sendDsiCommand(io, c, data, delay);
	};

	return cmd(0xff, {0x98, 0x81, 0x01}) &&
	       cmd(0xb7, {0x03}) &&
	       cmd(0xff, {0x98, 0x81, 0x00}) &&
	       cmd(0x11, {}, 120) &&
	       cmd(0x36, {0x00}) &&
	       cmd(0x3a, {0x55}) &&
	       cmd(0x29, {}, 20);
}

esp_lcd_video_timing_t videoTiming(DisplayController controller)
{
	esp_lcd_video_timing_t timing{};
	timing.h_size = kNativeWidth;
	timing.v_size = kNativeHeight;
	if (controller == DisplayController::Ili9881c) {
		timing.hsync_back_porch = 140;
		timing.hsync_pulse_width = 40;
		timing.hsync_front_porch = 40;
		timing.vsync_back_porch = 20;
		timing.vsync_pulse_width = 4;
		timing.vsync_front_porch = 20;
	} else {
		timing.hsync_back_porch = 40;
		timing.hsync_pulse_width = 2;
		timing.hsync_front_porch = 40;
		timing.vsync_back_porch = 8;
		timing.vsync_pulse_width = 2;
		timing.vsync_front_porch = 220;
	}
	return timing;
}

bool readSt7123Touch(TouchSample *sample)
{
	std::uint8_t adv = 0;
	if (!i2cReadReg16Split(kTouchSt7123, 0x0010, &adv, 1, kTouchTimeoutMs)) return false;
	if ((adv & (1u << 3)) == 0) {
		*sample = {};
		return true;
	}

	std::uint8_t maxTouches = 0;
	if (!i2cReadReg16Split(kTouchSt7123, 0x0009, &maxTouches, 1, kTouchTimeoutMs)) return false;
	maxTouches = std::min<std::uint8_t>(maxTouches, 10);
	if (maxTouches == 0) {
		*sample = {};
		return true;
	}

	std::uint8_t data[10 * 7] = {};
	if (!i2cReadReg16Split(kTouchSt7123, 0x0014, data, maxTouches * 7, kTouchTimeoutMs)) return false;
	for (std::uint8_t i = 0; i < maxTouches; ++i) {
		const std::uint8_t *p = data + i * 7;
		if ((p[0] & 0x80) == 0) continue;
		sample->touching = true;
		sample->points = 1;
		sample->x = static_cast<std::uint16_t>(((p[0] & 0x3f) << 8) | p[1]);
		sample->y = static_cast<std::uint16_t>((p[2] << 8) | p[3]);
		return true;
	}
	*sample = {};
	return true;
}

bool readGt911Touch(TouchSample *sample)
{
	std::uint8_t info = 0;
	if (!i2cReadReg16(kTouchGt911, 0x814e, &info, 1, kTouchTimeoutMs)) return false;
	if ((info & 0x80) == 0) return true;
	const std::uint8_t points = std::min<std::uint8_t>(info & 0x0f, 5);
	if (points == 0) {
		*sample = {};
		(void)i2cWriteReg16(kTouchGt911, 0x814e, 0x00, kTouchTimeoutMs);
		return true;
	}
	std::uint8_t data[8] = {};
	if (!i2cReadReg16(kTouchGt911, 0x814f, data, sizeof(data), kTouchTimeoutMs)) return false;
	sample->touching = true;
	sample->points = points;
	sample->x = static_cast<std::uint16_t>(data[1] | (static_cast<std::uint16_t>(data[2]) << 8));
	sample->y = static_cast<std::uint16_t>(data[3] | (static_cast<std::uint16_t>(data[4]) << 8));
	(void)i2cWriteReg16(kTouchGt911, 0x814e, 0x00, kTouchTimeoutMs);
	return true;
}

bool writeIna226Reg(std::uint8_t reg, std::uint16_t value)
{
	const std::uint8_t data[] = {reg, static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value & 0xff)};
	return i2cWrite(kIna226, data, sizeof(data));
}

bool readIna226Reg(std::uint8_t reg, std::uint16_t *value)
{
	std::uint8_t data[2] = {};
	if (!value || !i2cReadReg8(kIna226, reg, data, sizeof(data))) return false;
	*value = be16(data);
	return true;
}

bool writeBmi270ConfigChunk(std::size_t offset, const std::uint8_t *data, std::size_t size)
{
	if (!data || size == 0 || size > kBmi270ConfigChunkSize || (offset & 1u) != 0) return false;

	const std::uint16_t wordOffset = static_cast<std::uint16_t>(offset / 2);
	const std::uint8_t addressBytes[] = {
		kBmi270RegInitAddr0,
		static_cast<std::uint8_t>(wordOffset & 0x0f),
		static_cast<std::uint8_t>(wordOffset >> 4),
	};
	if (!i2cWrite(kBmi270, addressBytes, sizeof(addressBytes), kBmi270ConfigTimeoutMs)) return false;

	std::uint8_t payload[kBmi270ConfigChunkSize + 1] = {kBmi270RegInitData};
	std::memcpy(payload + 1, data, size);
	return i2cWrite(kBmi270, payload, size + 1, kBmi270ConfigTimeoutMs);
}

bool uploadBmi270Config()
{
	if (!i2cWriteReg8(kBmi270, kBmi270RegInitCtrl, 0x00, kBmi270ConfigTimeoutMs)) return false;

	for (std::size_t offset = 0; offset < sizeof(kBmi270MaxFifoConfigFile); offset += kBmi270ConfigChunkSize) {
		const std::size_t remaining = sizeof(kBmi270MaxFifoConfigFile) - offset;
		const std::size_t chunk = std::min<std::size_t>(remaining, kBmi270ConfigChunkSize);
		if (!writeBmi270ConfigChunk(offset, kBmi270MaxFifoConfigFile + offset, chunk)) return false;
	}

	if (!i2cWriteReg8(kBmi270, kBmi270RegInitCtrl, 0x01, kBmi270ConfigTimeoutMs)) return false;

	for (int i = 0; i < 20; ++i) {
		vTaskDelay(pdMS_TO_TICKS(10));
		std::uint8_t status = 0;
		if (!i2cReadReg8(kBmi270, kBmi270RegInternalStatus, &status, 1, kBmi270ConfigTimeoutMs)) return false;
		if ((status & 0x0f) == kBmi270InternalStatusInitOk) return true;
	}
	return false;
}

}  // namespace

const char *displayControllerName(DisplayController controller)
{
	switch (controller) {
	case DisplayController::St7123: return "ST7123/ST7121";
	case DisplayController::Ili9881c: return "ILI9881C";
	case DisplayController::Unknown:
	default: return "unknown";
	}
}

DisplayController detectDisplayController()
{
	if (gDisplayController != DisplayController::Unknown) return gDisplayController;
	if (i2cProbe(kTouchGt911, 20)) {
		gDisplayController = DisplayController::Ili9881c;
		return gDisplayController;
	}
	if (i2cProbe(kTouchSt7123, 20)) {
		gDisplayController = DisplayController::St7123;
		return gDisplayController;
	}
	return DisplayController::Unknown;
}

bool initIoExpanders()
{
	if (gIoExpandersReady) return true;
	if (!initExpander(kIo43, kIo43Outputs, kIo43DefaultOutputs, 0x00, kIo43PullUps, 0x00)) {
		ESP_LOGE(kTag, "PI4IOE5V6408 0x43 init failed");
		return false;
	}
	if (!initExpander(kIo44, kIo44Outputs, kIo44DefaultOutputs, kIo44HighZ, kIo44PullUps, kIo44PullDowns)) {
		ESP_LOGE(kTag, "PI4IOE5V6408 0x44 init failed");
		return false;
	}
	gIo43Output = kIo43DefaultOutputs;
	gIo44Output = kIo44DefaultOutputs;
	gIoExpandersReady = true;
	ESP_LOGI(kTag, "PI4IOE5V6408 expanders ready");
	return true;
}

bool setSpeakerEnabled(bool enabled)
{
	return setIoPin(kIo43, kIo43SpkEn, enabled);
}

bool setCameraEnabled(bool enabled)
{
	return setIoPin(kIo43, kIo43CamEn, enabled);
}

bool setChargingEnabled(bool enabled)
{
	return setIoPin(kIo44, kIo44ChargeEn, enabled);
}

bool initDisplayPanel(int framebufferCount, LcdPanel *outPanel)
{
	if (gPanel.panel) {
		if (outPanel) *outPanel = gPanel;
		return true;
	}
	if (!initIoExpanders()) return false;

	lcdReset(true);
	vTaskDelay(pdMS_TO_TICKS(10));
	lcdReset(false);
	vTaskDelay(pdMS_TO_TICKS(120));

	DisplayController controller = detectDisplayController();
	if (controller == DisplayController::Unknown) {
		ESP_LOGE(kTag, "display controller probe failed");
		return false;
	}

	esp_lcd_dsi_bus_config_t busConfig{};
	busConfig.bus_id = 0;
	busConfig.num_data_lanes = kDsiLaneCount;
	busConfig.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
	busConfig.lane_bit_rate_mbps = controller == DisplayController::Ili9881c ? 730 : 965;
	esp_err_t err = esp_lcd_new_dsi_bus(&busConfig, &gPanel.dsiBus);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "DSI bus init failed: %s", esp_err_to_name(err));
		return false;
	}

	esp_lcd_dbi_io_config_t dbiConfig{};
	dbiConfig.virtual_channel = 0;
	dbiConfig.lcd_cmd_bits = 8;
	dbiConfig.lcd_param_bits = 8;
	err = esp_lcd_new_panel_io_dbi(gPanel.dsiBus, &dbiConfig, &gPanel.io);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "DSI DBI panel I/O init failed: %s", esp_err_to_name(err));
		return false;
	}

	esp_lcd_dpi_panel_config_t dpiConfig{};
	dpiConfig.virtual_channel = 0;
	dpiConfig.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
	// M5's ST712x BSP requests 70MHz, which scans at ~57.8Hz. Request 82MHz to
	// stay on the 80MHz DPI divider while raising native portrait refresh further
	// without jumping to the unstable 120MHz divider path.
	dpiConfig.dpi_clock_freq_mhz = controller == DisplayController::Ili9881c ? 60 : 82;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
	dpiConfig.in_color_format = LCD_COLOR_FMT_RGB565;
	dpiConfig.out_color_format = LCD_COLOR_FMT_RGB565;
#else
	dpiConfig.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
	dpiConfig.flags.use_dma2d = true;
#endif
	dpiConfig.num_fbs = framebufferCount;
	dpiConfig.video_timing = videoTiming(controller);
	err = esp_lcd_new_panel_dpi(gPanel.dsiBus, &dpiConfig, &gPanel.panel);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "DPI panel init failed: %s", esp_err_to_name(err));
		return false;
	}

	const bool chipReady = controller == DisplayController::St7123 ? initSt7123(gPanel.io) : initIli9881c(gPanel.io);
	if (!chipReady) return false;

	err = esp_lcd_panel_init(gPanel.panel);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "DPI panel low-level init failed: %s", esp_err_to_name(err));
		return false;
	}

	gPanel.controller = controller;
	if (outPanel) *outPanel = gPanel;
	ESP_LOGI(kTag, "display panel ready: %s", displayControllerName(controller));
	return true;
}

bool initTouch()
{
	if (gTouchReady) return true;
	if (!initIoExpanders()) return false;
	DisplayController controller = detectDisplayController();
	if (controller == DisplayController::Unknown) return false;

	touchReset(true);
	vTaskDelay(pdMS_TO_TICKS(controller == DisplayController::St7123 ? 20 : 10));
	touchReset(false);
	vTaskDelay(pdMS_TO_TICKS(50));

	gpio_config_t config{};
	config.pin_bit_mask = 1ULL << gea::platform::board::touch.interrupt;
	config.mode = GPIO_MODE_INPUT;
	config.pull_up_en = GPIO_PULLUP_ENABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_NEGEDGE;
	gpio_config(&config);
	configureTouchInterrupt();

	gTouchReady = true;
	return true;
}

bool waitForTouchReport(int timeoutMs)
{
	if (!gTouchReportSemaphore) {
		if (timeoutMs > 0) vTaskDelay(pdMS_TO_TICKS(timeoutMs));
		return false;
	}
	const TickType_t ticks = timeoutMs < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
	return xSemaphoreTake(gTouchReportSemaphore, ticks) == pdTRUE;
}

bool readTouch(TouchSample *sample)
{
	if (!sample || !initTouch()) return false;
	TouchSample next = *sample;
	const DisplayController controller = detectDisplayController();
	bool ok = false;
	if (controller == DisplayController::St7123) ok = readSt7123Touch(&next);
	else if (controller == DisplayController::Ili9881c) ok = readGt911Touch(&next);
	if (!ok) return false;
	*sample = next;
	return true;
}

bool initPowerMonitor()
{
	if (gPowerReady) return true;
	if (!initIoExpanders()) return false;
	setChargingEnabled(true);

	std::uint16_t manufacturer = 0;
	std::uint16_t die = 0;
	if (!readIna226Reg(0xfe, &manufacturer) || !readIna226Reg(0xff, &die)) return false;
	if (manufacturer != 0x5449 || die != 0x2260) {
		ESP_LOGW(kTag, "INA226 ID mismatch mfg=0x%04x die=0x%04x", manufacturer, die);
		return false;
	}
	writeIna226Reg(0x00, 0x8000);
	vTaskDelay(pdMS_TO_TICKS(2));
	writeIna226Reg(0x00, 0x0527);
	constexpr float currentLsb = 0.0005f;
	constexpr float shuntOhms = 0.005f;
	const std::uint16_t cal = static_cast<std::uint16_t>((0.00512f / (currentLsb * shuntOhms)) + 0.5f);
	writeIna226Reg(0x05, cal == 0 ? 1 : cal);
	gPowerReady = true;
	return true;
}

BatteryStatus readBatteryStatus()
{
	BatteryStatus status{};
	if (!initPowerMonitor()) return status;

	std::uint16_t busRaw = 0;
	std::uint16_t currentRaw = 0;
	std::uint16_t powerRaw = 0;
	if (!readIna226Reg(0x02, &busRaw) || !readIna226Reg(0x04, &currentRaw) || !readIna226Reg(0x03, &powerRaw)) return status;

	constexpr float currentLsb = 0.0005f;
	status.voltageV = static_cast<float>(busRaw) * 1.25e-3f;
	status.currentMa = static_cast<float>(static_cast<std::int16_t>(currentRaw)) * currentLsb * 1000.0f;
	status.powerMw = static_cast<float>(powerRaw) * 25.0f * currentLsb * 1000.0f;
	constexpr float packMin = 3.2f * 2.0f;
	constexpr float packMax = 4.2f * 2.0f;
	status.chargePercent = std::clamp((status.voltageV - packMin) / (packMax - packMin), 0.0f, 1.0f) * 100.0f;
	bool chargeStat = false;
	status.isCharging = getIoPin(kIo44, kIo44ChargeStat, &chargeStat) && chargeStat && status.currentMa < -1.0f;
	status.isPresent = status.voltageV > 5.5f;
	return status;
}

bool initImu()
{
	if (gImuReady) return true;
	std::uint8_t chipId = 0;
	if (!i2cReadReg8(kBmi270, kBmi270RegChipId, &chipId, 1)) return false;
	if (chipId != kBmi270ChipId) {
		ESP_LOGW(kTag, "BMI270 ID mismatch: 0x%02x", chipId);
		return false;
	}

	if (!i2cWriteReg8(kBmi270, kBmi270RegCmd, kBmi270SoftReset)) return false;
	vTaskDelay(pdMS_TO_TICKS(3));

	if (!i2cWriteReg8(kBmi270, kBmi270RegPwrConf, 0x00)) return false;
	vTaskDelay(pdMS_TO_TICKS(1));
	if (!uploadBmi270Config()) {
		std::uint8_t status = 0;
		(void)i2cReadReg8(kBmi270, kBmi270RegInternalStatus, &status, 1, kBmi270ConfigTimeoutMs);
		ESP_LOGW(kTag, "BMI270 config load failed: internal_status=0x%02x", status);
		return false;
	}

	if (!i2cWriteReg8(kBmi270, kBmi270RegPwrCtrl, kBmi270PowerAccelGyroTemp)) return false;
	vTaskDelay(pdMS_TO_TICKS(50));
	if (!i2cWriteReg8(kBmi270, kBmi270RegAccConf, kBmi270AccConf100HzNormal)) return false;
	if (!i2cWriteReg8(kBmi270, kBmi270RegAccRange, kBmi270AccRange4g)) return false;
	if (!i2cWriteReg8(kBmi270, kBmi270RegGyrConf, kBmi270GyrConf100HzNormal)) return false;
	if (!i2cWriteReg8(kBmi270, kBmi270RegGyrRange, kBmi270GyrRange2000Dps)) return false;

	gImuReady = true;
	ESP_LOGI(kTag, "BMI270 IMU ready");
	return true;
}

Vector3 readAccelerationG()
{
	Vector3 out{};
	if (!initImu()) return out;
	std::uint8_t data[6] = {};
	if (!i2cReadReg8(kBmi270, kBmi270RegAccX, data, sizeof(data))) return out;
	out.x = static_cast<double>(le16s(&data[0])) / 8192.0;
	out.y = static_cast<double>(le16s(&data[2])) / 8192.0;
	out.z = static_cast<double>(le16s(&data[4])) / 8192.0;
	return out;
}

Vector3 readGyroscopeDps()
{
	Vector3 out{};
	if (!initImu()) return out;
	std::uint8_t data[6] = {};
	if (!i2cReadReg8(kBmi270, kBmi270RegGyrX, data, sizeof(data))) return out;
	out.x = static_cast<double>(le16s(&data[0])) / 16.4;
	out.y = static_cast<double>(le16s(&data[2])) / 16.4;
	out.z = static_cast<double>(le16s(&data[4])) / 16.4;
	return out;
}

bool readImuMotion(Vector3 *accelerationG, Vector3 *gyroscopeDps)
{
	if (!initImu()) return false;
	std::uint8_t data[12] = {};
	if (!i2cReadReg8(kBmi270, kBmi270RegAccX, data, sizeof(data))) return false;
	if (accelerationG) {
		accelerationG->x = static_cast<double>(le16s(&data[0])) / 8192.0;
		accelerationG->y = static_cast<double>(le16s(&data[2])) / 8192.0;
		accelerationG->z = static_cast<double>(le16s(&data[4])) / 8192.0;
	}
	if (gyroscopeDps) {
		gyroscopeDps->x = static_cast<double>(le16s(&data[6])) / 16.4;
		gyroscopeDps->y = static_cast<double>(le16s(&data[8])) / 16.4;
		gyroscopeDps->z = static_cast<double>(le16s(&data[10])) / 16.4;
	}
	return true;
}

}  // namespace gea::platform::tab5
