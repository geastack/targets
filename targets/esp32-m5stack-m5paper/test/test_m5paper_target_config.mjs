import assert from "node:assert/strict"
import fs from "node:fs"
import path from "node:path"

const repoRoot = path.resolve(import.meta.dirname, "../../..")
const targetDir = path.join(repoRoot, "targets/esp32-m5stack-m5paper")

function read(relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), "utf8")
}

assert.equal(fs.existsSync(targetDir), true, "M5Paper target directory should exist")

const metadata = JSON.parse(read("targets.json"))["esp32-m5stack-m5paper"]
assert.deepEqual(metadata, {
  adapter: "esp32-idf",
  targetPath: "targets/esp32-m5stack-m5paper",
  flashSize: "16MB",
  appPlatform: "esp32",
  idfTarget: "esp32",
  esptoolChip: "esp32",
  mainTaskStackSize: "4096"
})

const board = read("targets/esp32-m5stack-m5paper/main/include/board.h")
for (const pin of [
  "GPIO_NUM_2", "GPIO_NUM_4", "GPIO_NUM_5", "GPIO_NUM_12",
  "GPIO_NUM_13", "GPIO_NUM_14", "GPIO_NUM_15", "GPIO_NUM_21",
  "GPIO_NUM_22", "GPIO_NUM_23", "GPIO_NUM_27", "GPIO_NUM_35",
  "GPIO_NUM_36", "GPIO_NUM_37", "GPIO_NUM_38", "GPIO_NUM_39"
]) {
  assert.match(board, new RegExp(pin), `board.h should contain ${pin}`)
}
assert.match(board, /spiHost = SPI3_HOST/)

const cmake = read("targets/esp32-m5stack-m5paper/main/CMakeLists.txt")
assert.match(cmake, /GEA_EMBEDDED_TARGET_CPP_BOARD "esp32-m5stack-m5paper"/)
assert.match(cmake, /GEA_EMBEDDED_TARGET_DISPLAY_WIDTH 540/)
assert.match(cmake, /GEA_EMBEDDED_TARGET_DISPLAY_HEIGHT 960/)
assert.match(cmake, /GEA_EMBEDDED_AUDIO_DISABLED=1/)
assert.match(cmake, /GEA_EMBEDDED_RUNTIME_TASK_STACK_BYTES=32768/)
assert.match(cmake, /audio_runtime_stub\.cpp/)

const appMain = read("targets/esp32-m5stack-m5paper/main/app_main.cpp")
assert.match(appMain, /xTaskCreateWithCaps/)
assert.match(appMain, /"gea_runtime"/)
assert.match(appMain, /MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT/)

const partitions = read("targets/esp32-m5stack-m5paper/partitions.csv")
assert.match(partitions, /ota_0,\s+app,\s+ota_0,\s+0x20000,\s+0x700000/)
assert.match(partitions, /ota_1,\s+app,\s+ota_1,\s+0x720000,\s+0x700000/)

const display = read("targets/esp32-m5stack-m5paper/main/display.cpp")
assert.match(display, /IT8951 info=%ux%u/)
assert.match(display, /setQualityGrayscale/)
assert.match(display, /xTaskCreatePinnedToCore\(refreshTaskEntry/)
assert.match(display, /gea_epaper_full_refresh/)
assert.match(display, /kSpiClockHz = 12 \* 1000 \* 1000/)
assert.match(display, /waitForWaveform/)
// Fast-waveform binarisation. The framebuffer is packed 4bpp, so the
// threshold runs per nibble against the 16-level midpoint -- there is no
// unpacked 8-bit luma row to compare against.
assert.match(display, /\(byte >> 4\) < 8 \? 0 : 15/)
assert.match(display, /\(byte & 0x0F\) < 8 \? 0 : 15/)
assert.match(display, /kMaxBatchRegions = 1/)
assert.match(display, /const UpdateMode mode = region\.full \? UpdateGc16/)
assert.match(display, /UpdateA2 = 7/)
// Waveform selection. A rapid-update streak forces the fast 1-bit A2 waveform
// even in quality mode: 16-level partials arriving faster than the ~450 ms
// GL16 cycle read as flicker.
assert.match(display, /region\.qualityGrayscale && !region\.fastStreak\) \? UpdateGl16 : UpdateA2/)
assert.doesNotMatch(display, /partial batch regions=/)
assert.match(display, /\(left & ~3\) == \(right & ~3\)/)
assert.match(display, /coversScreen/)
assert.match(display, /y0 &= ~3/)
assert.match(display, /y1 = std::min\(height - 1/)
// Bulk region transfer must be double-buffered and interrupt-driven: fill one
// chunk while the other DMAs, then block in get_trans_result. A single-buffer
// polling transmit busy-spins this priority-6 task for the whole ~200 ms load
// and starves the frame task on the same core.
assert.match(display, /std::uint8_t \*bufs\[2\] = \{dmaChunk_, dmaChunkB_\}/)
assert.match(display, /spi_device_get_trans_result\(spi_, &done, portMAX_DELAY\)/)
assert.match(display, /const std::uint8_t prefix\[2\] = \{0x00, 0x00\}/)
assert.match(display, /power\.epdPower, 0/)
assert.doesNotMatch(display, /const std::uint8_t packet\[4\]/)

const sdkconfig = read("targets/esp32-m5stack-m5paper/sdkconfig.defaults")
assert.match(sdkconfig, /CONFIG_ESP_MAIN_TASK_STACK_SIZE=4096/)
assert.match(sdkconfig, /CONFIG_ESP_IPC_TASK_STACK_SIZE=1024/)

const linker = read("targets/esp32-m5stack-m5paper/main/m5paper_psram.lf")
// A locally-defined scheme, not IDF's `extram_bss`: IDF only ships that one
// inside esp_wifi's fragments, and in a no-network build ldgen SILENTLY drops
// entries naming an unknown scheme. The distinct name also keeps a
// wifi-enabled build from colliding on the definition.
assert.match(linker, /\[scheme:m5paper_extram_bss\]/)
assert.match(linker, /render \(m5paper_extram_bss\)/)
assert.match(linker, /canvas \(m5paper_extram_bss\)/)

const touch = read("targets/esp32-m5stack-m5paper/main/touch.cpp")
assert.match(touch, /esp_lcd_touch_new_i2c_gt911/)
assert.match(touch, /kMaxFingers = 2/)
assert.match(touch, /setPointerObserver/)
assert.match(touch, /Touchscreen::poll\(int nowMs\)/)
// GT911 sampling runs on its OWN task now, not off the frame loop: pinned to
// PRO_CPU above both the runtime task (5) and the e-paper refresh task (6),
// so touch keeps being read while the frame task is blocked mid-page-draw.
// The frame-loop poll stays as the degraded path if the task never started.
assert.match(touch, /xTaskCreatePinnedToCore\(touchTaskEntry, "m5paper_touch", 3072, this, 7, &touchTask_, 0\)/)
assert.match(touch, /if \(touchTask_ != nullptr\) return;/)
assert.doesNotMatch(touch, /StackType_t/)

const buttons = read("targets/esp32-m5stack-m5paper/main/buttons.cpp")
assert.match(buttons, /queueKeyDown\(button\.keyCode\)/)
assert.match(buttons, /kPowerOffHoldMs = 2500/)
assert.match(buttons, /esp_sleep_enable_ext0_wakeup/)
assert.match(buttons, /void poll\(int nowMs\)/)
assert.doesNotMatch(buttons, /xTaskCreate/)

const batteryService = read("targets/esp32/services/battery_service.cpp")
assert.match(batteryService, /void BatteryService::poll\(int nowMs\)/)
assert.doesNotMatch(batteryService, /xTaskCreate/)
assert.doesNotMatch(batteryService, /StackType_t/)

const power = read("targets/esp32-m5stack-m5paper/main/power.cpp")
assert.match(power, /ADC_CHANNEL_7/)
assert.match(power, /pinMillivolts \* 2/)

const storage = read("targets/esp32-m5stack-m5paper/main/sdcard_mount.cpp")
assert.match(storage, /esp_vfs_fat_sdspi_mount/)
assert.match(storage, /gea::platform::board::storage\.cs/)

console.log("M5Paper target configuration tests passed")
