# Espressif ESP-Mosaico

Target ID: `esp32-s31-espressif-mosaico`. Select **Espressif ESP-Mosaico
(V1.2)** in `gea setup`; the suggested board alias is `mosaico`.

This target supports hardware **V1.2** (V1.1 shares the pin mapping). The board
is an ESP32-S31 (dual-core RISC-V at 320 MHz) with 16 MB in-package octal
PSRAM, 16 MB quad NOR flash, a 480x480 CO5300 QSPI AMOLED with CST9220 touch,
an ES8311 codec with NS4150B amplifier, a BMI270 IMU, two BMM150
magnetometers, a BQ27220 fuel gauge and 128 MB SPI NAND. V1.0 swaps the LCD
clock and reset lines and moves the onboard I2C bus, so it is not supported.

Pins come from Espressif's board support package
([esp-mosaico-bsp](https://github.com/esp-mosaico/esp-mosaico-bsp)) and the
V1.2 user guide; see `main/include/board.h`.

## Toolchain

The ESP32-S31 exists only in ESP-IDF master (6.2-dev, a preview target).
`targets.json` pins this target to `idfVersion: "6.2"`, so the CLI selects an
installed ESP-IDF 6.2 checkout for it, and keeps the newest stable install for
every other board. Install one next to your other IDF versions, for example
`~/esp32/esp-idf-v6.2-dev`, with `./install.sh esp32s31`.

## Flashing

From an application directory:

```sh
gea build --board mosaico
gea flash --board mosaico
```

The USB-C port is the S31's high-speed USB OTG controller, not a ROM console:
it has no DTR/RTS reset circuit, V1.2 ships with automatic download mode
disabled, and a software reset into the downloader leaves the ROM silent on
that port until a power cycle. So `gea flash` installs the app through the app
already running (`usbAppUpdate: "geadev-ota"`): the image goes over the USB
console into the next OTA slot and the board restarts into it. No buttons.

The ROM downloader is still needed for the first flash, or when no gea app is
running: **hold BOOT while powering the board on** (POWER until dark, hold
BOOT, press POWER, release BOOT), then `gea flash --board mosaico --manual-boot`.
`gea flash` falls back to that by itself when no app answers.

## Console

There is no ROM console on the USB-C port. The firmware brings up a TinyUSB
CDC-ACM interface and routes the console through it. Its USB serial number is
the chip MAC in the `AA:BB:CC:DD:EE:FF` form the S3 boards report, so register
the board by that serial and `--board mosaico` resolves the port the usual way.
UART0 keeps early boot output.

## Board bring-up

Everything on the panel side is behind VCC_3V3, a high-side switch enabled by
GPIO60 (active low). `main/board_power.cpp` soft-starts it through an LEDC
fade, as the BSP does, and releases the PWR_SW power-off request on GPIO57
before display init touches the panel.

The panel's TE line is wired on GPIO43, so apps that ask for vsync
(`Display.setVSync(true)`) render locked to the panel's ~60 Hz refresh.

Not yet wired: the BQ27220 fuel gauge (the board reports no battery reading),
the magnetometers and the SPI NAND.
