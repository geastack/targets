// The Wi-Fi diagnostics a build without the network capability still has to
// answer.
//
// ble_hid.cpp reports Wi-Fi state alongside OTA state in its status
// notification, and that notification is compiled whenever BLE OTA is enabled.
// connectivity/wifi.cpp, which defines these, only builds with the network
// capability, so a BLE-only app referenced them and failed to link. This stub
// reports the same values that a build with Wi-Fi compiled in but never started
// would report -- state 0 is WifiStation's own initial "never started" -- so the
// notification stays truthful without inventing a protocol value for it.
#include <cstdint>

#include "esp_heap_caps.h"

namespace gea::targets::esp32::wifi::wifi_debug {

int state()
{
	return 0;
}

int lastError()
{
	return 0;
}

std::uint32_t dmaFree()
{
	return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

std::uint32_t dmaLargest()
{
	return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

}  // namespace gea::targets::esp32::wifi::wifi_debug
