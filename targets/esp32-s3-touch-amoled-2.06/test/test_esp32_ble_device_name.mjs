import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'

const source = readFileSync(
  fileURLToPath(new URL('../../esp32/connectivity/ble_hid.cpp', import.meta.url)),
  'utf8'
)

assert.doesNotMatch(
  source,
  /void HidServer::init[\s\S]*?\(void\)device_name;[\s\S]*?"Gea AMOLED"/,
  'the shared ESP32 BLE driver must not discard the app-supplied device name'
)
assert.match(
  source,
  /device_name && device_name\[0\] \? device_name : hid::kDefaultDeviceName/,
  'BLE advertising should use the app device name with the generic HID fallback'
)

console.log('esp32 BLE device-name tests passed')
