# M5Stack Tab5

GeaStack target `esp32-p4-m5stack-tab5` for the
[M5Stack Tab5](https://docs.m5stack.com/en/core/Tab5): an ESP32-P4 (16 MB
flash, 32 MB PSRAM) with a 5 inch 1280 × 720 MIPI-DSI touch screen, an
ESP32-C6 for Wi-Fi, a camera and stereo audio.

GeaStack apps are TypeScript and JSX with real CSS. The compiler turns them into
native C++ firmware, so the same app that runs in a browser runs on the Tab5.

## Getting started

You need Node.js 22 or newer, Python 3, and a USB-C data cable (use the Tab5's
USB-C port, not the USB-A host port). ESP-IDF is installed for you by
`gea setup`.

1. Create a project from the Counter example:

   ```sh
   npm create geastack@latest tab5-counter -- --example counter-jsx
   cd tab5-counter
   ```

2. Plug in the Tab5 and register it. Pick **M5Stack Tab5**, keep the alias
   `m5tab`, and choose the detected USB device. The first run also installs
   ESP-IDF v6.0.2.

   ```sh
   npx gea setup
   ```

3. Build, flash and open the serial monitor:

   ```sh
   npx gea run --board m5tab
   ```

Tap **+**, **−** and **Reset** on the screen to change the count.

### Next steps

- Edit the app, then run `npx gea run --board m5tab` again.
  `npx gea flash --board m5tab` flashes without the monitor.
- `npx gea screenshot shot.png --board m5tab` saves what the screen shows.
- `npx gea simulate` runs the same app in a desktop simulator.
- The Camera Studio example uses the Tab5's camera:
  `npm create geastack@latest my-camera -- --example camera-studio`.
- Other examples: `npm create geastack@latest my-app` and choose
  **Example application**, or browse
  [geastack/examples](https://github.com/geastack/examples).

## What the target supports

| Hardware | Support |
| --- | --- |
| 5 inch 1280 × 720 IPS, MIPI-DSI (ST7123/ST7121, or ILI9881C on early units) | Landscape. CSS uses a device pixel ratio of 2, so apps lay out on a 640 × 360 CSS px viewport. |
| Touch (integrated ST7123/ST7121, or GT911 on early units) | Pointer, touch and click events. |
| ESP32-C6 (esp_hosted over SDIO) | Wi-Fi through the standard network APIs, built only when the app uses them. |
| SC2356 camera (MIPI-CSI) | Camera capture for the camera APIs. |
| ES8388 codec, ES7210 microphones, speaker | Audio playback and capture. |
| BMI270 IMU | Accelerometer and gyroscope through the sensor API. |
| microSD | Mounted for file storage, including exFAT cards. |

The Tab5 has no user GPIO button. Its power and reset/boot buttons are
handled in hardware: click the power button to power on, double-click it to
power off.

## Flashing troubleshooting

The ESP32-P4 flashes over its native USB. If flashing cannot connect, put the
board in download mode: with USB connected, hold the reset/boot button until
the green LED blinks, then release and run the command again.

Charging works only after the Tab5 has booted.

## Hardware references

- [M5Stack Tab5 documentation, pin map and schematics](https://docs.m5stack.com/en/core/Tab5)
- [M5Stack factory firmware](https://github.com/m5stack/M5Tab5-UserDemo)
