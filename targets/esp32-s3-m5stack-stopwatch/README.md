# M5Stack StopWatch

Built-in target: `esp32-s3-m5stack-stopwatch` for the M5Stack StopWatch C152.
The board uses an ESP32-S3R8 with 16 MB QIO flash and 8 MB octal PSRAM.

## Supported hardware

- 1.75 inch, 466 × 466 round AMOLED with a CO5300 controller on an 80 MHz
  QSPI bus. The application renders a rectangular framebuffer; the physical
  circular panel hides its corners. The default CSS device pixel ratio is 1.5.
- CST820B capacitive touch on the shared I2C bus, with its interrupt on GPIO13.
- Yellow KEYA (GPIO2) and blue KEYB (GPIO1) send standard Gea key events:
  ArrowLeft and ArrowRight. Holding both for 500 ms sends Escape.
- BMI270 acceleration and gyroscope through the shared sensor binding.
- ES8311 speaker and microphone through Gea's audio engine and media capture.
- M5PM1 battery telemetry through `Battery.level()` and `Battery.charging()`.
- RX8130CE clock through `Clock.epochMs()` and `Clock.setEpochMs()`.
- M5IOE1 vibration through `Haptics.vibrate()`.
- Native USB Serial/JTAG for firmware flashing, logs and device control.
- M5PM1 and M5IOE1 control needed to power and reset the display and touch.

The factory-style `examples/apps/m5-stopwatch` application is written in
TypeScript/JSX. Its badge portal uses the shared HTTP and Wi-Fi APIs, settings
use `localStorage`, and its FFT runs in a TypeScript audio worklet. It has no
application-owned native sources or compiler plugin.

## Register and run

Run the CLI from an application project with `@geastack/cli` and an
`@geastack/targets` installation containing this target, for example
`apps/bouncing-balls-jsx` in the `examples` repository.

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

## Status

`bouncing-balls-jsx` builds with ESP-IDF 6.0.2, flashes, and renders at about
60 FPS on this board; touch and button input have not yet been verified on
hardware.

## Hardware references

- [Official StopWatch specifications, pin map and schematics](https://docs.m5stack.com/en/core/StopWatch)
- [M5Stack factory firmware](https://github.com/m5stack/M5StopWatch-UserDemo)
- [Factory IO expander setup](https://github.com/m5stack/M5StopWatch-UserDemo/blob/main/main/hal/hal_ioe.cpp)
- [M5IOE1 driver](https://github.com/m5stack/M5IOE1)
- [M5PM1 driver](https://github.com/m5stack/M5PM1)
