# M5Stack M5Paper

GeaStack target `esp32-m5stack-m5paper` for the
[M5Stack M5Paper v1.1](https://docs.m5stack.com/en/core/m5paper_v1.1): a 4.7
inch 540 × 960 e-paper tablet with an ESP32-D0WDQ6-V3, 16 MB flash and 8 MB
PSRAM.

GeaStack apps are TypeScript and JSX with real CSS. The compiler turns them into
native C++ firmware, so the same app that runs in a browser runs here, drawn
onto the e-paper panel.

## Getting started

You need Node.js 22 or newer, Python 3, and a USB-C data cable. ESP-IDF is
installed for you by `gea setup`.

1. Create a project from the Tetris example:

   ```sh
   npm create geastack@latest m5paper-tetris -- --example button-tetris
   cd m5paper-tetris
   npm install
   ```

2. Plug in the M5Paper and register it. Pick **M5Stack M5Paper (v1.1)**, keep
   the alias `m5paper`, and choose the detected USB device. The first run also
   installs ESP-IDF v6.0.2.

   ```sh
   npx gea setup
   ```

3. Build, flash and open the serial monitor:

   ```sh
   npx gea run --board m5paper
   ```

Tap the on-screen buttons to move, rotate and drop the piece. The rocker
sends key events your own app can use; see [Controls](#controls).

### Next steps

- Edit the app, then run `npx gea run --board m5paper` again.
  `npx gea flash --board m5paper` flashes without the monitor.
- `npx gea screenshot shot.png --board m5paper` saves what the panel shows.
- `npx gea simulate` runs the same app in a desktop simulator.
- Other examples: `npm create geastack@latest my-app` and choose
  **Example application**, or browse
  [geastack/examples](https://github.com/geastack/examples).

## What the target supports

| Hardware | Support |
| --- | --- |
| ED047TC1 4.7 inch e-paper, 540 × 960, through the IT8951 controller | Portrait, 16-level grayscale. Asynchronous partial refreshes, fast 4-level updates, quality 16-level updates and periodic full refreshes. |
| GT911 capacitive touch | Two-point touch through pointer, touch and click events. |
| Left, center/power and right rocker | Key events; see [Controls](#controls). |
| Battery | Voltage and percentage reporting. |
| microSD | Mounted for file storage, on the display's shared SPI bus. |
| Wi-Fi and BLE | Built only when the app uses them. |

The M5Paper has no speaker or microphone, so the target links the no-audio
backend. The checked-in partition layout has two 7 MB OTA app slots and a local
storage partition.

## Controls

- Left/right rocker: `ArrowLeft` and `ArrowRight`, including key repeat
- Center rocker: app launcher/back action
- Hold center for 2.5 seconds: power down; press it again to wake
- Touch: normal pointer input, with two simultaneous contacts supported

## Flashing troubleshooting

The M5Paper flashes through its USB-to-serial chip, which resets the ESP32 into
the bootloader on its own. If flashing cannot connect, try another USB-C cable
(some are power-only) and make sure the board is on: hold the center rocker
until it wakes.

E-paper keeps its last image without power, so a board that is off or stuck
can still show an old picture.

## Hardware references

- [M5Stack M5Paper v1.1 documentation and schematics](https://docs.m5stack.com/en/core/m5paper_v1.1)
- [M5Stack factory firmware](https://github.com/m5stack/M5Paper_FactoryTest)

## Pin map

| Function | GPIO |
| --- | ---: |
| Power latch | 2 |
| microSD CS | 4 |
| External power enable | 5 |
| IT8951 MOSI/MISO/SCLK | 12 / 13 / 14 |
| IT8951 CS / BUSY / power | 15 / 27 / 23 |
| I2C SDA / SCL | 21 / 22 |
| Battery ADC | 35 |
| GT911 interrupt | 36 |
| Left / center / right rocker | 37 / 38 / 39 |
