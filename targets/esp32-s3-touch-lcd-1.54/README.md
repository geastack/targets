# Waveshare ESP32-S3-Touch-LCD-1.54

Target: `esp32-s3-touch-lcd-1.54`; setup catalog alias: `waveshare-154`.
This is the color touch LCD board, not the similarly named e-paper board.

Hardware assignments follow the [Waveshare factory ESP-IDF BSP](https://github.com/waveshareteam/ESP32-S3-Touch-LCD-1.54/tree/main/examples/ESP32-S3-Touch-LCD-1.54-demo/ESP-IDF-5.5.1/01_factory/components/esp_bsp).

| Peripheral | Implementation / connections |
| --- | --- |
| LCD | ST7789 240×240, SPI2 mode 3 at 40 MHz; MOSI 39, SCLK 38, CS 21, DC 45, reset 40 |
| Backlight | PWM on GPIO46, brightness 0–100% |
| Touch | CST816 at I2C 0x15; reset 47, interrupt 48 |
| Shared I2C | SDA 42, SCL 41 |
| Speaker | ES8311, NS4150B amplifier enable GPIO7 |
| Microphone | ES7210 at 0x40; I2S input GPIO11 |
| I2S | MCLK 8, BCLK 9, WS 10, output 12, input 11 |
| Motion | QMI8658 accelerometer and gyroscope, shared I2C |
| SD card | FAT32 at `/sdcard`; SDMMC 1-bit CLK 16, CMD 15, D0 17 |
| Battery | GPIO2 soft latch; active-low charging GPIO3; calibrated ADC1 channel 0 / GPIO1, 3:1 divider |
| Buttons | BOOT GPIO0 → ArrowUp, PLUS GPIO4 → ArrowDown, PWR GPIO5 → Enter; hold PWR 2 seconds to release battery latch |
| Wireless | Native 2.4 GHz Wi-Fi and NimBLE BLE, shared ESP32 drivers |
| Internal storage | 16 MB flash, 8 MB octal PSRAM; NVS and SPIFFS, two 6 MB OTA slots |

Wi-Fi, BLE and audio use the existing app capability/build policies. Apps must
request the corresponding APIs to link their implementation. The diagnostics app
requests all three and exposes the hardware checks. SD mount failures never format
the card. Battery percentage is a voltage estimate, not a coulomb counter; without
ADC calibration it returns unknown rather than inventing a reading.

Build with the local Gea CLI (or installed `gea`):

```sh
gea build --target esp32-s3-touch-lcd-1.54 --app diagnostics
```

The shared ST7789 pipeline retains StickS3 defaults; this board overrides only
SPI timing, glass offsets, and the absence of its M5PM1 LCD power rail. CST816
uses Espressif's controller driver through the shared touch event pipeline.

Validation: diagnostics firmware builds with network, BLE and audio enabled.
Bouncing Balls JSX was flashed over USB with hash verification and boots with
changing 240×240 rendered pixels. Speaker output, microphone quality, physical
touch alignment, battery accuracy, SD throughput and radio reliability still
require hardware checks. The offline app currently reports an empty app ID
in the serial discovery response.
