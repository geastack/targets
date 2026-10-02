import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

const repoRoot = new URL('../../..', import.meta.url).pathname.replace(/\/$/, '')
const wifi = readFileSync(`${repoRoot}/targets/esp32/connectivity/wifi.cpp`, 'utf8')
// The block ends at applyTaskStackOverride, which replaced a direct
// `cfg.osi_funcs = &g_bumped_osi_funcs;` assignment; anchoring on the old line
// silently emptied this slice and every assertion below it passed vacuously.
const init = wifi.match(/wifi_init_config_t cfg[\s\S]*?applyTaskStackOverride\(&cfg\);/)?.[0] ?? ''
assert.ok(init, 'the wifi_init_config_t block should be found')

assert.match(init, /cfg\.static_rx_buf_num = 4;/)
// Receive depth is bought with DYNAMIC frames, which land in PSRAM under
// CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP, not with static frames, which are
// permanent internal DMA allocations competing with the display staging.
assert.match(init, /cfg\.dynamic_rx_buf_num = 16;/)
assert.match(init, /cfg\.rx_ba_win = 8;/)
// TX pools and aggregation belong to each board's sdkconfig (and, for duplex
// streaming apps, the GEA_EMBEDDED_WIFI_*_BUFFERS defines): a one-frame TX pool
// plus one cached packet cannot carry microphone uploads alongside the ACKs for
// incoming audio. The shared driver must therefore not hardcode them.
assert.doesNotMatch(init, /cfg\.static_tx_buf_num = \d+;/, 'the shared driver must not pin the TX DMA pool')
assert.doesNotMatch(init, /cfg\.cache_tx_buf_num = \d+;/, 'the shared driver must not pin the TX cache pool')
assert.doesNotMatch(init, /cfg\.ampdu_tx_enable/, 'TX aggregation is left to the board sdkconfig')
assert.match(init, /cfg\.static_tx_buf_num = GEA_EMBEDDED_WIFI_STATIC_TX_BUFFERS;/)
assert.match(init, /cfg\.cache_tx_buf_num = GEA_EMBEDDED_WIFI_CACHE_TX_BUFFERS;/)
assert.match(init, /cfg\.static_rx_buf_num = GEA_EMBEDDED_WIFI_STATIC_RX_BUFFERS;/)
assert.match(init, /cfg\.dynamic_rx_buf_num = GEA_EMBEDDED_WIFI_DYNAMIC_RX_BUFFERS;/)
assert.match(init, /cfg\.mgmt_sbuf_num = 6;/)
// The stack override moved out of wifi.cpp into its own header, where the osi
// function table is patched once. 8192 because the stock 3584-byte stack
// overflowed during WPA2 auth and 6144 was still not enough.
const taskStack = readFileSync(`${repoRoot}/targets/esp32/connectivity/wifi_task_stack.h`, 'utf8')
assert.match(taskStack, /constexpr uint32_t kOverrideBytes = 8192;/)
assert.match(taskStack, /adapter\.funcs\._task_create_pinned_to_core = task_stack_detail::createPinned;/)
