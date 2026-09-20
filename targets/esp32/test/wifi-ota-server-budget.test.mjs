import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const ota = readFileSync(`${repoRoot}/targets/esp32/services/ota.cpp`, 'utf8')
// The server was split: the HTTP surface and the receive loop stayed in
// ota.cpp, while partition writing and the reboot timer moved to ota_image.cpp.
const otaImage = readFileSync(`${repoRoot}/targets/esp32/services/ota_image.cpp`, 'utf8')
const deviceControl = readFileSync(`${repoRoot}/targets/esp32/services/device_control.cpp`, 'utf8')

// 8 KiB, not 4: /screenshot runs on this task and replays the retained UI tree,
// the same call the serial device-control task sizes itself at 8 KiB for.
assert.match(ota, /constexpr int kServerStackSize = 8192;/)
// Screenshots must not require the USB cable.
assert.match(ota, /screenshotUri\.uri = "\/screenshot";/)
assert.match(ota, /received == HTTPD_SOCK_ERR_TIMEOUT/)
assert.match(ota, /while \(total < req->content_len\)/)
assert.doesNotMatch(deviceControl, /"gea_devctl",[\s\S]*?MALLOC_CAP_SPIRAM \| MALLOC_CAP_8BIT/)
assert.match(deviceControl, /#if CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG[\s\S]*?"gea_devctl_usb",[\s\S]*?MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT[\s\S]*?#else[\s\S]*?"gea_devctl",[\s\S]*?MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT/)
// The image is erased once, up front, in whole blocks — never lazily per 4 KiB
// sector inside the receive loop, which cost ~42 s of the old 123 s transfer.
assert.doesNotMatch(ota, /esp_ota_begin\([^)]*OTA_WITH_SEQUENTIAL_WRITES[^)]*\)/)
assert.match(ota, /esp_ota_begin\(updatePartition, eraseLength, &otaHandle\)/)
// Staging must be DMA-capable internal DRAM: esp_flash_write only takes its
// direct path for a DRAM source, and anything else -- PSRAM, or RTC FAST memory,
// which a bare MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT request also matches once
// DRAM runs short -- is copied 32 bytes at a time. Measured on the amoled-2.06:
// 517 KB/s from DRAM vs 212 KB/s from RTC fast.
assert.match(ota, /heap_caps_malloc\(candidate, MALLOC_CAP_INTERNAL \| MALLOC_CAP_DMA\)/)
assert.doesNotMatch(
  ota,
  /heap_caps_malloc\(candidate, MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT\)/,
  'an 8-bit-capable request also matches RTC fast memory, which is not DRAM'
)
// Scoped to the OTA receive staging struct, not the whole file: /screenshot
// stages a 402 KiB frame and PSRAM is the right place for that. The constraint
// is only that bytes handed to esp_ota_write come from DRAM.
const staging = ota.match(/struct ReceiveStaging \{[\s\S]*?\n\t\};/)?.[0] ?? ''
assert.ok(staging, 'ReceiveStaging struct not found')
assert.doesNotMatch(staging, /MALLOC_CAP_SPIRAM/)
// The 1 KiB stack buffer remains the fallback when that allocation fails.
assert.match(ota, /char stackBuffer\[1024\];/)
assert.match(otaImage, /StaticTimer_t g_rebootTimerStorage;/)
assert.match(otaImage, /xTimerCreateStatic\(/)
assert.doesNotMatch(
  otaImage.match(/void OtaServer::rebootSoon\(\)[\s\S]*?\n\}/)?.[0] ?? '',
  /^\s*xTaskCreate\(/m,
  'activating a validated OTA image must not depend on a late heap allocation'
)
