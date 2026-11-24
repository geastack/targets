# M5Stack StickS3

GeaStack target `esp32-s3-m5stack-sticks3` for the
[M5Stack StickS3 (K150)](https://docs.m5stack.com/en/core/StickS3): an
ESP32-S3-PICO-1-N8R8 (8 MB flash, 8 MB octal PSRAM) with a 1.14 inch
135 × 240 color LCD and two face buttons.

GeaStack apps are TypeScript and JSX with real CSS. The compiler turns them into
native C++ firmware, so the same app that runs in a browser runs on the stick.

## Getting started

You need Node.js 22 or newer, Python 3, and a USB-C data cable. ESP-IDF is
installed for you by `gea setup`.

1. Create a project from the Diagnostics example, which is driven entirely by
   the stick's two buttons:

   ```sh
   npm create geastack@latest sticks3-diagnostics -- --starter example --example diagnostics
   cd sticks3-diagnostics
   ```

2. Plug in the StickS3 and register it. Pick **M5Stack StickS3 (K150)**, keep
   the alias `sticks3`, and choose the detected USB device. The first run
   also installs ESP-IDF v6.0.2.

   ```sh
   npx gea setup
   ```

3. Build, flash and open the serial monitor:

   ```sh
   npx gea run --board sticks3
   ```

Diagnostics opens on a menu of the stick's hardware. **KEY2** (the lower face
button) moves the highlight and **KEY1** opens the highlighted item; the bar
at the bottom always says what each button does. The **Buttons** screen shows
each key as you press it, and the same pins read through the `Gpio` and
`Digital` classes.

### Using the buttons in your own app

A button press arrives as a `keydown` and its release as a `keyup`. The
event's `code` is the button's silkscreen name, `Key1` or `Key2`. Its `key`
is a default alias (`ArrowUp` for KEY1, `ArrowDown` for KEY2), so a keyboard
drives the same code in the browser:

```tsx
<div onKeyDown={e => {
  if (e.code == 'Key1' || e.key == 'ArrowUp') this.increment()
  if (e.code == 'Key2' || e.key == 'ArrowDown') this.decrement()
}}>
```

`Buttons` from `@geastack/core` lists the board's buttons (`count()`,
`code(i)`, `key(i)`, `pin(i)`) and says whether one is held now
(`pressed('Key1')`). `Gpio` and `Digital` can open a button's pin as an input
alongside the key events.

The Counter example (`--example counter-jsx`) also runs here: KEY1 counts up
and KEY2 counts down.

### Next steps

- Edit the app, then run `npx gea run --board sticks3` again.
  `npx gea flash --board sticks3` flashes without the monitor.
- `npx gea screenshot shot.png --board sticks3` saves what the screen shows.
- `npx gea simulate` runs the same app in a desktop simulator.
- Other examples: `npm create geastack@latest my-app` and choose
  **Example application**, or browse
  [geastack/examples](https://github.com/geastack/examples).

## What the target supports

| Hardware | Support |
| --- | --- |
| ST7789 1.14 inch IPS LCD, 135 × 240 | Portrait, RGB565. CSS uses a device pixel ratio of 1.5, so apps lay out on a 90 × 160 CSS px viewport; `@media (max-width: 140px)` targets it. |
| KEY1 (GPIO11), KEY2 (GPIO12) | Named key events and the `Buttons` API, interrupt-driven and debounced. |
| BMI270 IMU | Accelerometer through the sensor API. |
| ES8311 speaker and microphone | Audio playback and capture. |
| Wi-Fi and BLE | Built only when the app uses them. |

The side power button belongs to the M5PM1 power chip, which no GPIO sees.
Click it once to power on or reset; double-click it to power off.

## Flashing troubleshooting

The ESP32-S3 flashes over its native USB, so no buttons are needed normally. If
flashing cannot connect, put the stick in download mode: with USB connected,
hold the power button until the green LED blinks, then release and run the
command again.

## Hardware references

- [M5Stack StickS3 documentation, pin map and schematics](https://docs.m5stack.com/en/core/StickS3)
