import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const wifi = readFileSync(`${repoRoot}/targets/esp32/connectivity/wifi.cpp`, 'utf8')

assert.doesNotMatch(
  wifi.match(/bool connected\(\) const override[\s\S]*?\n\t\}/)?.[0] ?? '',
  /pumpDeferredBringUp|retryDeferredBringUp/,
  'reading connected state must not be required to make configuration progress'
)
assert.match(wifi, /scheduleDeferredBringUp\(\);/, 'configuration should schedule autonomous bring-up')
assert.match(wifi, /esp_timer_start_periodic\(bringUpRetryTimer_, 50 \* 1000\)/)
assert.match(wifi, /deferredBringUpTimer[\s\S]*retryDeferredBringUp\(\)/)
assert.doesNotMatch(
  wifi.match(/void retryDeferredBringUp\(\)[\s\S]*?\n\t\}/)?.[0] ?? '',
  /scheduleDeferredBringUp\(\)/,
  'the timer callback must not depend on re-arming a one-shot timer while its callback is active'
)
const ensureBringUp = wifi.match(/void ensureBringUp\(\)[\s\S]*?\n\t\}/)?.[0] ?? ''
assert.ok(
  ensureBringUp.indexOf('reserveInternal(56 * 1024)') < ensureBringUp.indexOf('scheduleDeferredBringUp()'),
  'display staging RAM must be reserved before Wi-Fi worker scheduling'
)
assert.ok(
  ensureBringUp.indexOf('reserveInternal(56 * 1024)') < ensureBringUp.indexOf('applyPendingInternalReserve()') &&
    ensureBringUp.indexOf('applyPendingInternalReserve()') < ensureBringUp.indexOf('scheduleDeferredBringUp()'),
  'the app/frame task must apply the display reservation before scheduling Wi-Fi'
)
assert.match(ensureBringUp, /applyPendingInternalReserve\(\);[\s\S]*if \(startBringUpWorker\(\)\)[\s\S]*taskYIELD\(\)/)
assert.match(wifi, /bringUpDeferFrames_ >= 3 && startBringUpWorker\(\)/)
assert.match(wifi, /"wifi_up", 10240, this,[\s\S]*configMAX_PRIORITIES - 1/)
assert.doesNotMatch(
  wifi.match(/static void bringUpTrampoline\(void \*arg\)[\s\S]*?vTaskDeleteWithCaps\(nullptr\);/)?.[0] ?? '',
  /vTaskDelay/,
  'the high-priority worker must claim Wi-Fi memory before app mounting resumes'
)
const gotIp = wifi.match(/eventId == IP_EVENT_STA_GOT_IP[\s\S]*?WIFI_EVENT_SCAN_DONE/)?.[0] ?? ''
assert.match(
  gotIp,
  /NetworkServices::startDeferredServers\(\)/,
  'a late DHCP result must start deferred OTA services without application polling'
)
