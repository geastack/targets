#pragma once

#include "pixel.h"

#include <cstddef>
#include <cstdint>

namespace gea::platform::esp32::services {

void startDeviceControlTask();

// Capture the current frame as RGB565, the same way the serial SCREENSHOTBIN
// command does. Exposed so a transport that is NOT the USB console -- the WiFi
// OTA server's /screenshot endpoint -- can capture without a cable. Returns
// false when the display is unavailable; *width/*height are only valid on true.
bool captureScreenshotRgb565(std::uint16_t *snapshot, int pixelCapacity, int *width, int *height,
                             char *appIdBuffer, std::size_t appIdBufferSize);

#if GEA_PIXEL_STORAGE_PACKED
// Same capture, but in the board's native sub-byte-PACKED storage (GRAY4/
// GRAY2), for a caller that wants to expand to RGB565 itself while it streams
// (as the SCREENSHOTBIN/OTA-screenshot paths do) instead of asking the device
// for a full RGB565-sized allocation it may not have. See display.h's
// copySnapshotPacked for the exact packing (flat row-major stream, no per-row
// padding; byteCapacity need only be pixel::packed::rowBytes(width*height)).
// Only declared/defined for GEA_PIXEL_STORAGE_PACKED boards.
bool captureScreenshotPacked(std::uint8_t *snapshot, int byteCapacity, int *width, int *height,
                             char *appIdBuffer, std::size_t appIdBufferSize);
#endif

}  // namespace gea::platform::esp32::services
