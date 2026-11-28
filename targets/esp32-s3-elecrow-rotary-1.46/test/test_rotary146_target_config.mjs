import assert from "node:assert/strict"
import fs from "node:fs"
import path from "node:path"

const repoRoot = path.resolve(import.meta.dirname, "../../..")

function read(relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), "utf8").replace(/\r\n/g, "\n")
}

const cmake = read("targets/esp32-s3-elecrow-rotary-1.46/main/CMakeLists.txt")
const sharedSpiCmake = read("targets/esp32/gea_spi_panel_component.cmake")

assert.match(cmake, /include\("[^"]*esp32\/gea_spi_panel_component\.cmake"\)/)
// A board never builds another board's files; shared code lives in targets/esp32.
assert.doesNotMatch(cmake, /\.\.\/\.\.\/esp32-[^/"]+\//)

for (const [axis, option] of [["WIDTH", "width"], ["HEIGHT", "height"]]) {
  const boardDimension = "GEA_EMBEDDED_TARGET_DISPLAY_" + axis
  assert.ok(cmake.includes("set(" + boardDimension + " 360)"))
  const variable = "${" + boardDimension + "}"
  assert.ok(sharedSpiCmake.includes("GEA_EMBEDDED_DISPLAY_" + axis + "=" + variable))
  assert.ok(sharedSpiCmake.includes("--font-viewport-" + option + " " + variable))
}

assert.match(cmake,
  /esp32\/chip_bindings\/input\/rotary_encoder\.cpp/,
  "The 1.46-inch board uses the shared detent decoder")
assert.match(cmake, /esp32\/chip_bindings\/displays\/st7789_spi_lcd\.cpp/)

const buttons = read("targets/esp32-s3-elecrow-rotary-1.46/main/buttons.cpp")
assert.match(buttons, /#include "chip_bindings\/input\/rotary_encoder\.h"/)
assert.match(buttons, /gea::platform::esp32::input::startRotaryEncoderTask\(\)/)
