# STM32F746G-Discovery

ST's MB1191 board: Cortex-M7 STM32F746NG, 4.3" 480x272 RGB LCD, FT5336
capacitive touch, 8 MB SDRAM, 16 MB QSPI NOR flash, ST-LINK/V2-1.

## Setup

- Arm GNU toolchain (`arm-none-eabi-gcc` on `PATH`, or `ARM_TOOLCHAIN_PATH`).
- The CMSIS headers and startup file come with the target
  (`vendor/README.md`). To build against an STM32CubeF7 checkout instead, set
  `STM32_CUBE_F7_PATH` to it; only its `Drivers/CMSIS` is used.
- openocd 0.12+ (`brew install open-ocd`). One `program` writes both internal
  flash and the QSPI image.

```sh
npx gea boards add                         # finds the ST-LINK, registers its serial, checks the tools
npx gea flash --board <alias>              # bring-up image (no app)
npx gea run --board <alias> --app <id>     # build, flash, open the console
```

The console is USART1 on the ST-LINK virtual COM port, 115200 baud.

## Memory

| Region | Address | Holds |
|---|---|---|
| Internal flash | `0x08000000`, 1 MB | boot, QSPI init, runtime, the engine's per-frame render/layout/canvas/text code, libc |
| QSPI (memory-mapped) | `0x90000000`, 16 MB | the generated app and the engine's cold code (CSS cascade, DOM events, image decoding; both `*_qspi.a`), fonts and assets |
| DTCM | `0x20000000`, 64 KB | stack, console DMA ring |
| SRAM1+2 | `0x20010000`, 256 KB | `.data`, `.bss`, thread-local block |
| SDRAM | `0xC0000000`, 8 MB | front + back framebuffers (512 KB), then the malloc heap |

`SystemInit` (`main/system_init.cpp`) sets the 200 MHz clock, the MPU and
caches, then SDRAM and QSPI memory-mapped mode, all before `.data`/`.bss` and
static constructors, so C++ may use every region from its first line. It reads
the QSPI JEDEC ID and supports both the N25Q128A and the W25Q128J fitted from
board revision C03.

The QSPI and flash code sit ~2 GB apart, beyond an ARM unwind entry's reach,
so each region keeps its own exception table (`unwind_tables.cpp`) and QSPI
archives are relinked to a QSPI-resident personality thunk
(`qspi_personality.S`).

## Display

The canvas draws into the SDRAM back buffer; a flush has DMA2D copy the dirty
rectangles into the front buffer the LTDC scans. `Display.setVSync(true)`
starts each flush in vertical blanking for tear-free updates. The backlight is
on/off only; brightness dims the layer toward black.

## Board I/O

| Feature | App API | Notes |
|---|---|---|
| Touch | pointer events with `pointerId` | FT5336, all 5 fingers; finger 0 drives scroll and click |
| USER button | `keydown` with keyCode 27, `Input.consumeBackButton()` | PI11, debounced |
| Green LED | `Gpio.configureOutput(129)`, `Gpio.write(129, on)` | pin number = port * 16 + pin, so PI1 = 129 |
| GPIO | `Gpio.*` on any free pin | pins the board drives (SDRAM, QSPI, LTDC, I2C, UART, SWD) refuse writes |
| Wall clock | `Date` | RTC on the 32.768 kHz crystal, kept across resets; set it with `gea devctl set-time` |

The console also speaks GEADEV, so `gea devctl` (`ping`, `app`, `state` with
the frame-loop `fps=`, `mem`, `set-time`, `key`, `back`, `tap`, `drag`,
`brightness`, `reboot`) and `gea screenshot` work over the ST-LINK port.

## Not yet supported

Persistent storage (`localStorage` lives in RAM), networking, audio, BLE and
the microSD card.
