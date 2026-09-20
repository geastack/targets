# LILYGO T-Display-S3 Long

Gea target for the ESP32-S3R8 T-Display-S3 Long with 16 MB flash, 8 MB
octal PSRAM, and the native 180 x 640 AXS15231B QSPI AMOLED panel. The
framework-facing display is 640 x 180 landscape by default; the rotated Canvas
maps logical RGB565 writes directly into native portrait panel order, so flush
can copy contiguous scanline bands without a transpose buffer. This is needed
because the controller has no usable hardware XY-swap mode.

The display uses LILYGO's documented pinout (CS 12, SCK 17, D0 13, D1 18,
D2 21, D3 14, reset 16, brightness/PWM 1) in SPI mode 0. The production QSPI
clock is 20 MHz: higher clocks produced intermittent horizontal corruption on
the connected board. Rotated flushes always transmit complete native 180-pixel
scanlines and serialize panel transactions, matching the stable shape of the
vendor polling transport. The panel binding explicitly sends both CASET and
RASET for every dirty rectangle; the generic component's QSPI draw helper only
supports its sequential full-frame convention.

The integrated touch hardware uses I2C GPIO15/GPIO10, reset GPIO2, and interrupt
GPIO11. The target probes both known board revisions: CST3530 at `0x58` and the
AXS15231 touch interface at `0x3b`. An interrupt-driven task reads panel-native
coordinates and feeds the standard Gea pointer-event pipeline. Debug-injected
touch remains available through the same seam.

Build with:

```sh
npx gea build --board t-display-long --app bouncing-balls-jsx
```
