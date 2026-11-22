# Pimoroni Tufty 2350

Gea target for the Pimoroni Tufty 2350 badge.

## Hardware

- MCU: RP2350
- Display: 2.8 inch 320x240 IPS LCD, ST7789-compatible 8-bit parallel bus
- RTC: PCF85063 at I2C address `0x51`
- Inputs: A, B, C, Up, Down, Home buttons
- Wireless: CYW43 Wi-Fi/BLE module pins are declared for future networking support
- Power: GPIO power enable, VBUS/charge status pins, ADC battery sense
- Memory: 16 MB flash, 8 MB PSRAM

## Pin Map

| Function | Pin |
| --- | ---: |
| Rear lights CL0-CL3 | GP0-GP3 |
| I2C0 SDA | GP4 |
| I2C0 SCL | GP5 |
| Button Down | GP6 |
| Button A | GP7 |
| PSRAM CS | GP8 |
| Button B | GP9 |
| Button C | GP10 |
| Button Up | GP11 |
| VBUS detect | GP12 |
| RTC alarm | GP13 |
| Reset button sense | GP14 |
| Button interrupt | GP15 |
| LCD TE/vsync | GP21 |
| Home button | GP22 |
| CYW43 power/data/cs/clock | GP23, GP24, GP25, GP29 |
| LCD backlight | GP26 |
| LCD CS | GP27 |
| LCD D/C | GP28 |
| LCD WR/RD | GP30, GP31 |
| LCD DB0-DB7 | GP32-GP39 |
| Battery ADC | GP40 / ADC0 |
| Power enable | GP41 |
| 1V1 sense | GP42 |
| Light sense | GP43 / ADC3 |

## Status

This target builds a Pico SDK bring-up app and a Gea app firmware. The bring-up
path enables board power, probes the PCF85063 RTC, reports button states, and
draws color bars through the ST7789 parallel LCD driver.

The Gea app path reuses the RP2350 runtime with Tufty-specific display, PSRAM,
button, and no-touch configuration. Wi-Fi/BLE pins are present in the board
definition, but networking is not wired into the RP2350 runtime yet.

## Build and flash

Prerequisites: the Raspberry Pi Pico SDK with `PICO_SDK_PATH` exported (the
target's `CMakeLists.txt` stops without it), an Arm GNU toolchain
(`arm-none-eabi-gcc`, or `PICO_TOOLCHAIN_PATH` pointing at one), and `picotool`
or the board mounted in BOOTSEL mode for flashing.

From an application project, register the board with `npx gea setup` (choose
**Pimoroni Tufty 2350**; the suggested alias is `tufty-2350`), then:

```sh
npx gea flash --board tufty-2350
```

`gea flash` builds the Gea app firmware and loads its UF2 with
`picotool load`, or copies it to the mounted BOOTSEL volume. `npx gea build
--target rp2350-tufty-2350` compiles without a registered board. Apps written
for the `esp32` platform also build for this target (`compatibleAppPlatforms`
in `targets.json`).
