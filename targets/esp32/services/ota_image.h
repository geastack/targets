#pragma once

#include <atomic>

// Diagnostics the BLE OTA notification reports. They describe the HTTP OTA
// server, but ble_hid.cpp reads them in every OTA-capable build -- including a
// BLE-only one, where no HTTP server is compiled at all. The storage therefore
// lives with the always-compiled image writer (ota_image.cpp) and ota.cpp
// updates it when the HTTP server is part of the build. Without a server the
// values keep their "never started" defaults, which is the truth.
namespace gea::targets::esp32::ota_debug {

extern std::atomic<int> httpStartError;
extern std::atomic<bool> httpStarted;

}  // namespace gea::targets::esp32::ota_debug
