# M5Stack StopWatch

GeaStack target `esp32-s3-m5stack-stopwatch` for the
[M5Stack StopWatch (C152)](https://docs.m5stack.com/en/core/StopWatch): an
ESP32-S3R8 with 16 MB QIO flash, 8 MB octal PSRAM and a 1.75 inch round AMOLED.

GeaStack apps are TypeScript and JSX with real CSS. The compiler turns them into
native C++ firmware, so the same app that runs in a browser runs on the watch.

## Getting started

You need Node.js 22 or newer, Python 3, and a USB-C data cable. ESP-IDF is
installed for you by `gea setup`.

1. Create a project from the StopWatch's own app, a Gea port of the factory
   firmware written in TypeScript and JSX:

   ```sh
   npm create geastack@latest stopwatch-app -- --example m5-stopwatch
   cd stopwatch-app
   ```

   For a smaller first program, use `--example counter-jsx` instead.

2. Plug in the StopWatch and register it. Pick **M5Stack StopWatch (C152)**,
   keep the alias `stopwatch`, and choose the detected USB device. The first
   run also installs ESP-IDF v6.0.2.

   ```sh
   npx gea setup
   ```

3. Build, flash and open the serial monitor:

   ```sh
   npx gea run --board stopwatch
   ```

Edit the app and run the same command again; `npx gea flash --board stopwatch`
flashes without the monitor, and `npx gea screenshot shot.png --board stopwatch`
saves what the screen shows.

The target also builds without a connected board:

```sh
npx gea build --target esp32-s3-m5stack-stopwatch
```

If flashing cannot connect, put the board in download mode: with USB
connected, hold the power button until the green LED blinks, then release and
run the command again.

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

## Hardware references

- [Official StopWatch specifications, pin map and schematics](https://docs.m5stack.com/en/core/StopWatch)
- [M5Stack factory firmware](https://github.com/m5stack/M5StopWatch-UserDemo)
- [Factory IO expander setup](https://github.com/m5stack/M5StopWatch-UserDemo/blob/main/main/hal/hal_ioe.cpp)
- [M5IOE1 driver](https://github.com/m5stack/M5IOE1)
- [M5PM1 driver](https://github.com/m5stack/M5PM1)
