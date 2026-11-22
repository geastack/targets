# M5Stack M5Paper

Gea target for the M5Paper v1.1 e-paper tablet. The default app is
`button-tetris`.

## Hardware support

- ESP32-D0WDQ6-V3 with 16 MB flash and 8 MB PSRAM
- 4.7-inch 540x960 ED047TC1 e-paper panel through the IT8951 controller
- Asynchronous partial refreshes, fast 4-level updates, quality 16-level
  grayscale updates, and periodic full refreshes
- GT911 two-point capacitive touch
- Left, center/power, and right rocker buttons
- Battery voltage and percentage reporting
- Power latch, e-paper power, and external-accessory power control
- microSD storage on the display's shared SPI bus
- ESP32 Wi-Fi and Bluetooth support

M5Paper has no onboard speaker or microphone, so the target intentionally uses
the no-audio backend.

## Controls

- Left/right rocker: `ArrowLeft` and `ArrowRight`, including key repeat
- Center rocker: app launcher/back action
- Hold center for 2.5 seconds: power down; press it again to wake
- Touch: normal pointer input, with two simultaneous contacts supported

## Build and flash

`gea setup` does not list the M5Paper, so register it by adding an entry to
`~/.geastack/boards.json` (or the project's `.gea/boards.json`) with
`"target": "esp32-m5stack-m5paper"`, `"adapter": "esp32-idf"` and its USB
serial under `transports.usbSerial.serial`; `npx gea boards discover` reads the
serial of a plugged-in board, and the entry format is in the `@geastack/cli`
[setup guide](https://github.com/geastack/cli/blob/main/docs/SETUP.md#entry-shapes).
Then, using the alias you chose (`m5paper` below), from a checkout of
[geastack/examples](https://github.com/geastack/examples) (run `npm install` at its root first; `button-tetris` is one of its
apps):

```sh
npx gea flash --board m5paper --app button-tetris
```

`gea flash` builds before flashing. Without an alias, name the target and the
serial device instead:
`npx gea flash --target esp32-m5stack-m5paper --port <serial-device> --app button-tetris`.

The checked-in partition layout provides two 7 MB OTA app slots and a local
storage partition in the board's 16 MB flash.

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
