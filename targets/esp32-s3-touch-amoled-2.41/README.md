# Waveshare ESP32-S3 Touch AMOLED 2.41

Target ID: `esp32-s3-touch-amoled-2.41`. Select **Waveshare ESP32-S3 Touch
AMOLED 2.41 (V1)** in `gea setup`; the suggested board alias is `amoled-241`.

This target supports hardware **V1**, with an RM69080 AMOLED panel, FT6336
touch controller, 16 MB flash, and 8 MB PSRAM. V2 uses different reset and
interrupt wiring and is not supported by this target's current pin mapping.

From an application directory:

```sh
gea build --board amoled-241
gea flash --board amoled-241 --monitor
```

Without a registered alias, pass the target and USB port explicitly:

```sh
gea flash --target esp32-s3-touch-amoled-2.41 --port /dev/cu.usbmodemXXXX
```

The CSS 3D Cube example was built and USB-flashed on this board with ESP-IDF
6.0.2. Flash hashes verified, and the device reported `app=css-3d-cube` and
approximately 60 FPS over serial.
