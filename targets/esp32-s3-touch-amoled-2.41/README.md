# Waveshare ESP32-S3 Touch AMOLED 2.41

Target ID: `esp32-s3-touch-amoled-2.41`. Select **Waveshare ESP32-S3 Touch
AMOLED 2.41 (V1)** in `gea setup`; the suggested board alias is `amoled-241`.

This target supports hardware **V1**, with an RM69080 AMOLED panel, FT6336
touch controller, 16 MB flash, and 8 MB PSRAM. V2 uses different reset and
interrupt wiring and is not supported by this target's current pin mapping.

From an application directory, after `npx gea setup` has registered the
board (substitute your alias if you chose another):

```sh
npx gea flash --board amoled-241 --monitor
```

`gea flash` builds first; `npx gea build --board amoled-241` compiles without
flashing. To build without registering an alias, name the target instead; a
flash then also needs the board's serial device:

```sh
npx gea build --target esp32-s3-touch-amoled-2.41
npx gea flash --target esp32-s3-touch-amoled-2.41 --port <serial-device>
```

Status: the CSS 3D Cube example builds with ESP-IDF 6.0.2, flashes, and runs at
about 60 FPS on V1 hardware.
