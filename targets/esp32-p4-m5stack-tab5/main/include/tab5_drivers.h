#pragma once

#include <cstdint>

#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

namespace gea::platform::tab5 {

enum class DisplayController {
	Unknown,
	St7123,
	Ili9881c,
};

struct LcdPanel {
	esp_lcd_dsi_bus_handle_t dsiBus = nullptr;
	esp_lcd_panel_io_handle_t io = nullptr;
	esp_lcd_panel_handle_t panel = nullptr;
	DisplayController controller = DisplayController::Unknown;
};

struct TouchSample {
	bool touching = false;
	std::uint8_t points = 0;
	std::uint16_t x = 0;
	std::uint16_t y = 0;
};

struct BatteryStatus {
	float voltageV = 0.0f;
	float currentMa = 0.0f;
	float powerMw = 0.0f;
	float chargePercent = 0.0f;
	bool isCharging = false;
	bool isPresent = false;
};

struct Vector3 {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

const char *displayControllerName(DisplayController controller);
DisplayController detectDisplayController();

bool initIoExpanders();
bool setCameraEnabled(bool enabled);
bool setSpeakerEnabled(bool enabled);
bool setChargingEnabled(bool enabled);

bool initDisplayPanel(int framebufferCount, LcdPanel *outPanel);

bool initTouch();
bool waitForTouchReport(int timeoutMs);
bool readTouch(TouchSample *sample);

bool initPowerMonitor();
BatteryStatus readBatteryStatus();

bool initImu();
Vector3 readAccelerationG();
Vector3 readGyroscopeDps();
bool readImuMotion(Vector3 *accelerationG, Vector3 *gyroscopeDps);

}  // namespace gea::platform::tab5
