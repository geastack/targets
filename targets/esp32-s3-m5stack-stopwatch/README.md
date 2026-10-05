# M5Stack StopWatch

Built-in target: `esp32-s3-m5stack-stopwatch` for the M5Stack StopWatch C152.
The board uses an ESP32-S3R8 with 16 MB QIO flash and 8 MB octal PSRAM.

## Supported hardware

- 1.75 inch, 466 × 466 round AMOLED with a CO5300 controller on an 80 MHz
  QSPI bus. The application renders a rectangular framebuffer; the physical
  circular panel hides its corners. The default CSS device pixel ratio is 1.5.
- CST820B capacitive touch on the shared I2C bus, with its interrupt on GPIO13.
- The blue KEYB button on GPIO1 follows the shared launcher/back/settings
  button policy. The yellow KEYA button on GPIO2 is not mapped yet.
- Native USB Serial/JTAG for firmware flashing, logs and device control.
- M5PM1 and M5IOE1 control needed to power and reset the display and touch.

Audio, battery telemetry, BMI270 IMU, RX8130CE RTC and vibration are not
integrated. Their presence on the board does not imply working Gea APIs;
the target currently uses unsupported audio and sensor facades.

## Register and run

Run the CLI from an application project with `@geastack/cli` and an
`@geastack/targets` installation containing this target. For the workspace
example, use `examples/apps/bouncing-balls-jsx`.

For a new machine, connect the board by USB and run guided setup. Choose
**M5Stack StopWatch (C152)**, keep the alias **stopwatch**, and select its
detected USB device:

```sh
npx gea setup
npx gea boards list
```

Setup stores the USB serial identity, which survives changes to the serial
device path. If the `stopwatch` alias is already registered, reuse it.

Build, flash and start the JSX bouncing balls application:

```sh
npx gea run --board stopwatch --app bouncing-balls-jsx
npx gea monitor --board stopwatch
```

The target can also be built without a connected board:

```sh
npx gea build --target esp32-s3-m5stack-stopwatch --app bouncing-balls-jsx
```

If the board needs manual download mode, connect USB, hold its power button
for about two seconds until the green LED lights, then release and retry.

## Device validation

On 2026-10-05, `bouncing-balls-jsx` built with ESP-IDF 6.0.2, flashed with
hash verification, and restarted successfully after a software reboot.
The device initialized the AMOLED rail/reset, negotiated 80 MHz QSPI, and
detected CST820 at 0x15. Ten one-second production frame windows averaged
60.26 FPS (59.2–61.7 FPS). Two CRC-verified 466 × 466 framebuffer captures
differed at 25,188 pixels, confirming advancing animation. Capture transfers
pause rendering, so their on-screen FPS labels are not benchmark samples.

The firmware used the workspace's shared `compiler/dist`. Rebuilding compiler
source was blocked by existing architecture violations in `segment-scopes.ts`
and its test; the compiler source rebuild was not verified. Physical touch and
button interactions were not exercised during this bring-up.

## Hardware references

- [Official StopWatch specifications, pin map and schematics](https://docs.m5stack.com/en/core/StopWatch)
- [M5Stack factory firmware](https://github.com/m5stack/M5StopWatch-UserDemo)
- [Factory IO expander setup](https://github.com/m5stack/M5StopWatch-UserDemo/blob/main/main/hal/hal_ioe.cpp)
- [M5IOE1 driver](https://github.com/m5stack/M5IOE1)
- [M5PM1 driver](https://github.com/m5stack/M5PM1)
