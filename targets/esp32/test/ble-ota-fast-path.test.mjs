import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const server = readFileSync(`${repoRoot}/targets/esp32/connectivity/ble_hid.cpp`, 'utf8')

const otaDataDefinition = server.match(/otaCharacteristics_\[1\] = \{[\s\S]*?\n\t\};/)
assert.ok(otaDataDefinition, 'expected the OTA data characteristic definition')
assert.match(
  otaDataDefinition[0],
  /\.flags = BLE_GATT_CHR_F_WRITE_NO_RSP \| BLE_GATT_CHR_F_WRITE_ENC/,
  'OTA data should be an encrypted write-without-response stream'
)
assert.doesNotMatch(
  otaDataDefinition[0],
  /BLE_GATT_CHR_F_WRITE \|/,
  'OTA data must not advertise the acknowledged write mode'
)
assert.match(server, /\.itvl_min = 6,[\s\S]*\.itvl_max = 12,/, 'OTA should request a 7.5–15 ms connection interval')
assert.match(server, /BLE_GAP_LE_PHY_2M_MASK/, 'OTA should request the 2M PHY')

