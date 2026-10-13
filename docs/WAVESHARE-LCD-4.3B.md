# Waveshare ESP32-S3-Touch-LCD-4.3B

The composed target `esp32-s3-waveshare-touch-lcd-4.3b` drives the board's 800×480
parallel RGB panel (ST7262), GT911 touch and CH422G I/O expander. The module is an
ESP32-S3-WROOM-1-N16R8: 16MB quad flash and 8MB octal PSRAM, where the framebuffers
live. The panel electronics are the 7-inch board's
(`esp32-s3-waveshare-touch-lcd-7`), so the definition is that one with the geometry
checked and the notes rewritten for what the B carries off the panel.

The USB-C port is the S3's native USB-Serial/JTAG, the base target's default
console. Unlike the 7-inch board, UART0 (GPIO43/44) feeds the RS485 transceiver,
so there is no second console on a header.

## Wiring and initialization

- RGB: DE5, HSYNC46, VSYNC3, PCLK7; data0–4 = B3–B7 (14, 38, 18, 17, 10),
  data5–10 = G2–G7 (39, 0, 45, 48, 47, 21), data11–15 = R3–R7 (1, 2, 42, 41, 40).
  Pixel clock 16MHz, the family's demo timings (4/8/8 on both axes), pclk on the
  falling edge.
- I2C: SDA8, SCL9; touch, expander and the PCF85063 RTC share the bus. The RTC has
  no role in a composed definition.
- GT911: INT on GPIO4, which the binding drives low across reset to fix the address at
  0x5d; reset is EXIO1 on the expander.
- CH422G: EXIO1 touch reset, EXIO2 backlight (DISP), EXIO3 panel reset, EXIO4 microSD
  chip-select. `initialOutputs` 0x1e as on the 7-inch: resets released, backlight on,
  card deselected. EXIO0 and EXIO5 are the isolated inputs DI0/DI1 and OD0/OD1 the
  isolated outputs; a composed board leaves them alone.
- GPIO0 carries G3, so the BOOT button is not a launcher button.
- The microSD slot is on SPI (MOSI11, SCK12, MISO13, CS on EXIO4), which the composed
  base does not drive; no storage is declared.
- Off the panel, and not represented in the definition: RS485 on UART0, CAN on
  GPIO15/16, a 7–36V DC input terminal that powers the board, and a CS8501 single-cell
  lithium charger with its own battery header.

Source: the vendor wiki page "ESP32-S3-Touch-LCD-4.3B" (pinout and expander map) and
the family's ESP-IDF demo (`waveshare_rgb_lcd_port.h`) for the timings.

## Verification

Run on the board on 2026-10-15 with the counter starter (`gea create --starter counter`)
built on the published stack (cli 0.1.102, compiler 1.0.33, core 0.1.42, engine 0.1.17) and
this branch as the targets root. The USB port identified as an ESP32-S3 (revision v0.2) with
8MB embedded octal PSRAM and 16MB quad flash in USB-Serial/JTAG mode. Build and USB flash
succeeded (2.8MB image, 55% of the slot free). The boot log shows the PSRAM memory test
passing, `CH422G ready (IO0-7 outputs=0x1e)`, `RGB panel ready 800x480 pclk=16000000`,
`GT911 found at 0x5d` running interrupt-driven, and the app at 62.5 fps. The panel shows the
counter; a USB screenshot matches it; `devctl tap` injections and fingers on the glass both
count. With the `build-config.json` entry the UI lays out at device-pixel ratio 1 (without
it the `esp32-s3` base's 1.5 applies and the starter's buttons touch the bottom edge).
Wi-Fi, OTA, the microSD slot and the B's off-panel peripherals are not exercised.
