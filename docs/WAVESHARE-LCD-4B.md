# Waveshare ESP32-S3-Touch-LCD-4B

The composed target `esp32-s3-waveshare-touch-lcd-4b` uses a 480×480 ST7701 RGB
panel, GT911 touch, a TCA9554 I/O expander, ES8311 output codec and ES7210
microphone ADC. The board has 16MB flash and 8MB octal PSRAM. A speaker must be
connected to its 8Ω / 2W speaker connector to hear playback.

The USB-UART bridge carries the console at 115200 baud. The native USB pins are
not the console for this target.

## Wiring and initialization

- I2C: SDA47, SCL48; codec control, touch and expander share the bus.
- Audio: MCLK5, BCLK16, WS7, output6, input15. ES7210 is at 7-bit address 0x40.
- TCA9554 at 0x20: panel CS0, data1, clock2, amplifier3, shared reset5 and
  touch address-select6. Pins4/7 remain inputs. After the reset sequence, pin6
  returns to input. Amplifier changes preserve all other output latches.
- Backlight GPIO4 is active-low. Zero brightness drives it high.
- The controller receives the vendor's 9-bit SPI initialization through the
  expander; pixel output uses the shared RGB DMA/rendering backend.
- Pixel clock is 16MHz, with the vendor's ST7701 480×480 timing. The physical
  panel uses its vendor 18-bit controller setting (0x3A=0x66); the S3 RGB bus
  carries 16 RGB565 lanes, matching Waveshare's BSP.

Source: [Waveshare BSP](https://github.com/waveshareteam/Waveshare-ESP32-components/tree/master/bsp/esp32_s3_touch_lcd_4b).

## Verification

The connected USB bridge was identified as an ESP32-S3 with 8MB embedded PSRAM.
Board composition and expander register/failure recovery tests pass. Firmware
build and USB flash succeeded. Boot logs confirm RGB controller initialization,
GT911 at 0x5d and Wi-Fi DHCP. End-to-end audio/video is not verified. Internal
RAM remains constrained after Wi-Fi (approximately 9KB free), and an I2C scan
triggered an ESP-IDF receive-ISR panic. This is not yet a working conversation.
