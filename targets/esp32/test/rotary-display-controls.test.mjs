import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'

// The rotary board drives its backlight with an LEDC PWM channel, not a binary
// GPIO, so Display.setBrightness is a real duty cycle rather than on/off.
//
// That an app actually asks for a partial brightness is guarded in
// geastack/examples, next to the app that asks.
const displaySource = readFileSync(
  new URL('../../../targets/esp32-s3-elecrow-rotary-2.1/main/display_rgb.cpp', import.meta.url),
  'utf8'
)
const boardCmake = readFileSync(
  new URL('../../../targets/esp32-s3-elecrow-rotary-2.1/main/CMakeLists.txt', import.meta.url),
  'utf8'
)

assert.match(
  boardCmake,
  /esp_driver_ledc/,
  'rotary display target should link the ESP-IDF LEDC driver for PWM backlight control'
)

assert.match(
  displaySource,
  /#include\s+"driver\/ledc\.h"/,
  'rotary display backend should use LEDC rather than binary GPIO for backlight brightness'
)

assert.match(
  displaySource,
  /ledc_timer_config_t[\s\S]*ledc_channel_config_t/,
  'rotary backlight init should configure an LEDC timer and channel'
)

assert.match(
  displaySource,
  /const\s+std::uint32_t\s+duty\s*=[\s\S]*g_brightness[\s\S]*;/,
  'rotary backlight duty should be derived from the stored brightness percentage'
)

assert.match(
  displaySource,
  /ledc_set_duty\([\s\S]*ledc_update_duty/,
  'rotary Display.setBrightness should apply brightness as a PWM duty cycle'
)

assert.doesNotMatch(
  displaySource,
  /gpio_set_level\(\s*gea::platform::board::display\.backlight\s*,\s*brightnessPercent\s*>\s*0\s*\?\s*1\s*:\s*0\s*\)/,
  'rotary Display.setBrightness must not collapse every nonzero value to full brightness'
)
