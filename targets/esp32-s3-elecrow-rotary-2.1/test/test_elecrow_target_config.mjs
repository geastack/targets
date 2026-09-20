import assert from "node:assert/strict"
import fs from "node:fs"
import path from "node:path"

const repoRoot = path.resolve(import.meta.dirname, "../../..")
const targetDir = path.join(repoRoot, "targets/esp32-s3-elecrow-rotary-2.1")

function read(relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), "utf8")
}

assert.equal(fs.existsSync(targetDir), true, "Elecrow target directory should exist")
assert.equal(fs.existsSync(path.join(targetDir, "main/CMakeLists.txt")), true, "target CMakeLists.txt should exist")
assert.equal(fs.existsSync(path.join(targetDir, "main/include/board.h")), true, "board profile should exist")
assert.equal(fs.existsSync(path.join(repoRoot, "targets.json")), true, "target metadata should exist")
assert.equal(fs.existsSync(path.join(repoRoot, "scripts/esp32s3-elecrow-rotary-2.1.sh")), false, "target wrapper script should be removed")

const board = read("targets/esp32-s3-elecrow-rotary-2.1/main/include/board.h")
for (const expected of [
  "GPIO_NUM_38",
  "GPIO_NUM_39",
  "GPIO_NUM_16",
  "GPIO_NUM_40",
  "GPIO_NUM_7",
  "GPIO_NUM_15",
  "GPIO_NUM_41",
  "GPIO_NUM_6",
  "GPIO_NUM_42",
  "GPIO_NUM_4",
  "0x21",
  "0x15",
  "12 * 1000 * 1000",
]) {
  assert.match(board, new RegExp(expected.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")), `board.h should contain ${expected}`)
}

const rgbDisplay = read("targets/esp32-s3-elecrow-rotary-2.1/main/display_rgb.cpp")
const sdkconfigDefaults = read("targets/esp32-s3-elecrow-rotary-2.1/sdkconfig.defaults")
assert.match(rgbDisplay, /LCD_CLK_SRC_PLL160M/)
assert.doesNotMatch(sdkconfigDefaults, /CONFIG_LCD_RGB_RESTART_IN_VSYNC=y/)
assert.match(rgbDisplay, /constexpr int kRgbBounceRows = 20/)
assert.match(rgbDisplay, /config\.bounce_buffer_size_px = kRgbBounceBufferPixels/)
assert.match(rgbDisplay, /std::uint16_t \*g_drawFramebuffer = nullptr;/)
assert.match(rgbDisplay, /g_canvas\.bindPixels\(g_drawFramebuffer, kWidth, kHeight\)/)
assert.doesNotMatch(rgbDisplay, /g_canvas\.bindPixels\(g_framebuffer, kWidth, kHeight\)/)
assert.match(board, /\.pclkActiveNeg = false/)
assert.match(rgbDisplay, /\{0xc0, \{0x3b, 0x00\}, 2, 0\}/)
assert.match(rgbDisplay, /\{0xc1, \{0x0b, 0x02\}, 2, 0\}/)
assert.match(rgbDisplay, /\{0xc2, \{0x00, 0x02\}, 2, 0\}/)
assert.match(rgbDisplay, /configureGpioOutput\(display\.spiSck, 0\)/)
assert.match(rgbDisplay, /gpio_set_level\(display\.spiSck, 1\);\n\tesp_rom_delay_us\(1\);\n\tgpio_set_level\(display\.spiSck, 0\)/)
assert.match(rgbDisplay, /void sendInitCommandsBatched\(\)/)
assert.match(rgbDisplay, /if \(cmd\.delayMs > 0\)/)
assert.match(rgbDisplay, /writePin\(gea::platform::board::expander\.lcdPowerBit, true\)/)
assert.match(rgbDisplay, /writePin\(gea::platform::board::expander\.touchInterruptBit, true\)/)

const cmake = read("targets/esp32-s3-elecrow-rotary-2.1/main/CMakeLists.txt")
assert.match(cmake, /GEA_EMBEDDED_CPP_BOARD "esp32-s3-elecrow-rotary-2\.1"/)
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_WIDTH=480/)
assert.match(cmake, /GEA_EMBEDDED_DISPLAY_HEIGHT=480/)
assert.match(cmake, /rotary_encoder\.cpp/)
assert.doesNotMatch(cmake, /co5300\.cpp/)
assert.doesNotMatch(cmake, /ft3168\.cpp/)

const rotaryEncoder = read("targets/esp32-s3-elecrow-rotary-2.1/main/rotary_encoder.cpp")
assert.match(rotaryEncoder, /queueRotaryDelta\(-detentDirection\)/, "Elecrow rotary physical direction should be inverted before dispatching UI deltas")
assert.match(rotaryEncoder, /kTransitionsPerDetent = 2/, "Elecrow rotary should dispatch one UI delta per physical click")

const appMain = read("targets/esp32-s3-elecrow-rotary-2.1/main/app_main.cpp")
assert.match(appMain, /startRotaryEncoderTask\(\)/)

const metadata = JSON.parse(read("targets.json"))["esp32-s3-elecrow-rotary-2.1"]
assert.equal(metadata.targetPath, "targets/esp32-s3-elecrow-rotary-2.1")
assert.equal(metadata.flashSize, "16MB", "the rotary board has a 16MB flash; the CLI passes --flash_size from this field")
assert.equal(metadata.adapter, "esp32-idf")

// The board's half of the geometry contract is the CMake definition asserted
// above. The other half -- that the shared display.h honours those macros
// instead of hardcoding a size -- is @geastack/core's header, asserted there.

const wifi = read("targets/esp32/connectivity/wifi.cpp")
assert.match(wifi, /__has_include\("wifi_config\.h"\)/)
assert.match(wifi, /#define GEA_EMBEDDED_WIFI_SSID ""/)
