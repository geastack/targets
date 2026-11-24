# M5Stack PaperS3

GeaStack target `esp32-s3-m5stack-papers3` for the
[M5Stack PaperS3 (C139)](https://docs.m5stack.com/en/core/PaperS3): a 4.7 inch
540 × 960 e-paper tablet with an ESP32-S3R8, 16 MB flash and 8 MB octal PSRAM.

GeaStack apps are TypeScript and JSX with real CSS. The compiler turns them into
native C++ firmware, so the same counter that runs in a browser runs here,
drawn straight onto the e-paper panel.

## Getting started

You need Node.js 22 or newer, Python 3, and a USB-C data cable. ESP-IDF is
installed for you by `gea setup`.

1. Create a project from the Counter example:

   ```sh
   npm create geastack@latest papers3-counter -- --example counter-jsx
   cd papers3-counter
   ```

2. Plug in the PaperS3 and register it. Pick **M5Stack PaperS3 (C139)**, keep
   the alias `papers3`, and choose the detected USB device. The first run
   also installs ESP-IDF v6.0.2.

   ```sh
   npx gea setup
   ```

3. Build, flash and open the serial monitor:

   ```sh
   npx gea run --board papers3
   ```

The screen flashes white once, then shows the counter in black on white. Tap
**+**, **−** and **Reset** to change the count.

The counter's dark theme turns light here on its own: the panel reports
`@media (monochrome)`, and the app's `Monochrome.css` switches to black on
white for it. Any CSS you write can do the same.

### Next steps

- Edit `components/App.tsx` or the CSS, then run `npx gea run --board papers3`
  again. `npx gea flash --board papers3` flashes without the monitor.
- `npx gea screenshot shot.png --board papers3` saves what the panel shows.
- `npx gea simulate` runs the same app in a desktop simulator.
- Other examples: `npm create geastack@latest my-app` and choose
  **Example application**, or browse
  [geastack/examples](https://github.com/geastack/examples).

## What the target supports

| Hardware | Support |
| --- | --- |
| ED047TC1 4.7 inch e-paper, 540 × 960 | Portrait, 16-level grayscale canvas drawn with four calibrated panel levels. Partial updates redraw only the pixels that changed. CSS `px` maps 1:1 to panel pixels. |
| GT911 capacitive touch | Two-point touch through pointer, touch and click events. |
| BMI270 IMU | Accelerometer through the sensor API. |
| microSD | Mounted for file storage. |
| Wi-Fi and BLE | Built only when the app uses them. |
| Passive buzzer | Not driven yet; the target links the no-audio backend. |

The PaperS3 has no user GPIO button: its single button belongs to the power
chip. Touch is the only input, so the launcher and back gestures are touch
gestures.

## Flashing troubleshooting

The ESP32-S3 flashes over its native USB, so no buttons are needed normally. If
flashing cannot connect, put the board in download mode: with USB connected,
hold the power button until the red LED blinks, then release and run the
command again.

E-paper keeps its last image without power, so a board that is off or stuck
can still show an old picture.

## Hardware references

- [M5Stack PaperS3 documentation, pin map and schematics](https://docs.m5stack.com/en/core/PaperS3)
- [M5Stack factory firmware](https://github.com/m5stack/M5PaperS3-UserDemo)
- The panel scan is derived from Wenting Zhang's PaperBoy and the calibrated
  waveforms from EPD_Painter's PaperS3 preset; see
  `components/papers3_epd/papers3_epd.c`.
