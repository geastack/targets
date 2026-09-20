import assert from "node:assert/strict"
import fs from "node:fs"
import path from "node:path"

const repoRoot = path.resolve(import.meta.dirname, "../../..")

function read(relativePath) {
  return fs.readFileSync(path.join(repoRoot, relativePath), "utf8")
}

const deviceControl = read("targets/esp32/services/device_control.cpp")
assert.match(deviceControl, /#include "i2c\.h"/)
assert.match(deviceControl, /#include "driver\/i2c_master\.h"/)
assert.match(deviceControl, /GEADEV:I2CSCAN BEGIN/)
assert.match(deviceControl, /i2c_master_probe/)
assert.match(deviceControl, /GEADEV:I2CSCAN ADDR 0x%02x/)
assert.match(deviceControl, /GEADEV:I2CSCAN END count=%d/)
assert.match(deviceControl, /tokenEquals\(command, "I2CSCAN"\)/)

// The HOST half of this protocol is the gea CLI (`gea board i2cscan` ->
// geadev.i2cScan in @geastack/cli), a different repository, so the assertions
// about it live there -- a test here could only read a file this repo does not
// have. This half asserts the DEVICE side, which is ours.
