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

Register the connected unit (`npx gea boards add`, or `npx gea boards
discover` to read its USB serial; the entry shape is in
`cli/docs/SETUP.md`), then run:

```sh
npx gea build --board m5paper --app button-tetris
npx gea flash --board m5paper --app button-tetris
```

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
