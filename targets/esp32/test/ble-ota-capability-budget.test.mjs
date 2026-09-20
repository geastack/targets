import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const server = readFileSync(`${repoRoot}/targets/esp32/connectivity/ble_hid.cpp`, 'utf8')


const otaDataCharacteristic = server.match(/otaCharacteristics_\[1\] = \{[\s\S]*?\n\t\};/)?.[0] ?? ''
assert.match(
  otaDataCharacteristic,
  /\.flags = BLE_GATT_CHR_F_WRITE_NO_RSP \| BLE_GATT_CHR_F_WRITE_ENC,/,
  'the OTA data characteristic must not advertise acknowledged writes'
)
// The NimBLE role/connection budget for BLE-OTA-only builds and the swift
// client's write-without-response flow control are asserted in @geastack/cli
// (test/esp32.test.mjs, test/ble-ota.test.mjs).
