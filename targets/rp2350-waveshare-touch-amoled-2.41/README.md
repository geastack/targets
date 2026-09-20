# Waveshare RP2350 Touch AMOLED 2.41

Target scaffold for the Waveshare RP2350-Touch-AMOLED-2.41 board.

## Hardware

- MCU: RP2350
- Display: 450x600 AMOLED, RM690B0-compatible QSPI command set
- Touch: FT6336 at I2C address `0x38`
- IMU: QMI8658 at I2C address `0x6B`
- RTC: PCF85063 at I2C address `0x51`
- Power: ETA6098 power path, GPIO power hold/key, ADC battery sense
- Memory: 16 MB flash, 8 MB PSRAM (APS6408, KGD `0x5d`, EID `0x53`)

Note: the PSRAM pool is clamped by `GEA_RP2350_PSRAM_SIZE_BYTES` in
`CMakeLists.txt`. This was wrongly set to 2 MB until 2026-07-02 (this README
claimed 2 MB too), which capped the heap at 2 MB and made memory-heavy apps
(e.g. typography) panic with `Out of memory` at boot — a black screen. The
chip's own EID density decode reports 8 MB and the allocator now uses all of
it.

## Pin Map

| Function | Pin |
| --- | ---: |
| I2C1 SDA | GP6 |
| I2C1 SCL | GP7 |
| Touch IRQ | GP4 |
| Touch reset | GP5 |
| QMI8658 IRQ | GP8 |
| Display CS | GP9 |
| Display SCLK | GP10 |
| Display D0-D3 | GP11-GP14 |
| Display reset | GP15 |
| Power key | GP24 |
| Power hold/control | GP25 |
| Battery ADC | GP40 / ADC0 |

## Status

This target builds either the Pico SDK bring-up app (power hold, I2C probes,
QMI8658/FT6336/PCF85063 cores, battery ADC, RM690B0 link check) or, with
`--app=<id>`, a full Gea app compiled by geatsc into the firmware
(`npx gea run --board waveshare-rp2350-amoled-2.41 --app typography`).

## Fonts: baked atlases vs runtime TTF

`GEA_EMBEDDED_TTF_RUNTIME_FONTS` (CMake option, **default OFF**) picks how app
fonts are shipped:

- **OFF (default, recommended):** geatsc bakes rasterized glyph atlases into
  the firmware at build time.
- **ON (experimental):** the firmware embeds the deflate-compressed TTFs and
  rasterizes glyphs on demand via stb_truetype into a runtime slot cache
  (`core/packages/engine/rasterized_font.cpp`).

Measured on typography (2026-07-02, dpr 1.5, 450x600):

| | atlases (OFF) | runtime TTF (ON) |
| --- | ---: | ---: |
| flash image | 3.48 MB | 2.00 MB |
| PSRAM peak | 1.89 MB | 2.82 MB |
| idle fps | 59 | 59 |
| scroll | smooth | visibly janky |

Runtime TTF is kept OFF for now because it currently scrolls much worse on
text-heavy screens: `presentDirectToPanel()` in `rp2350_gea_platform.cpp`
refuses frames containing `FillText` when runtime fonts are enabled (runtime
slot atlases can be reallocated mid-replay, so glyph pointers would dangle),
which drops every scroll frame to the full-framebuffer path, and glyphs are
additionally rasterized lazily inside scroll frames on first use. Fixing that
means pinning/pre-reserving slot atlases so the direct path stays safe, and
pre-rasterizing registered families at startup. Until then the +1.5 MB flash
for atlases is the better trade; the runtime path is the option when flash is
tight or font/size combinations are too numerous to bake.

## Boot memory constraints (read before porting a new app)

Until `main()` runs `memoryInit()`, the ONLY heap is the newlib SRAM heap
(~200-260 KB after .data/.bss) — and pico_malloc PANICS on a failed alloc, so
anything allocation-hungry before/around boot kills the board with a dark USB.
Weather hit all three of these (2026-07-02):

1. **CSS/prelude registration** used to run as a file-scope static constructor
   (before main, SRAM-only). The codegen now emits it as a function via
   `--cpp-prelude-symbol gea_plugin_cpp_register_prelude` (see the
   `add_custom_command`), which `main()` calls right after `memoryInit()` so
   the stylesheet allocates from PSRAM. A big enough stylesheet OOM'd pre-main
   (typography's ~185 KB squeaked by; weather's didn't).
2. **geatsc's `gea_cpp_shared_vector`** used to allocate in its default
   constructor, so emitted `inline static` members (the dynamic-prop sidecars)
   ran allocating constructors pre-main. Fixed upstream in the geatsc runtime:
   the constructor is now constexpr/lazy and such statics constant-initialize
   into .bss.
3. **stb_image** allocated decode buffers with plain `malloc` (SRAM). Fixed in
   `core/packages/engine/image_store.cpp`: `STBI_MALLOC` routes through
   `Allocator::allocatePreferSpiram`.

Boot diagnostics live in `main/app.cpp` and stay compiled in: a HardFault
handler and a pre-main hang detector report via watchdog-scratch on the next
boot, a 1 ms PC sampler records where a locked-up boot was stuck, and
`PICO_PANIC_FUNCTION=gea_panic` broadcasts panic messages over USB for 8 s and
then reboots (instead of wedging with dead USB) — which also keeps a live
USB window every cycle so `picotool load -f` can reflash a crashing firmware
without touching BOOTSEL.

Networked apps: this board has no radio. `main/rp2350_host_fetch_stub.cpp`
resolves every `gea::host::fetch` as a failed "offline" response, so apps run
in their placeholder/offline states; `host/image.cpp` is compiled in for
embedded-asset image loading (weather's icons).

## Scroll performance

The RM690B0 on this board does NOT implement DCS vertical scrolling
(`0x33`/`0x37` are ignored — verified 2026-07-02), so the platform fakes a
scroll register in software (`GEA_RP2350_SOFTWARE_SCROLL_REGISTER`: circular
framebuffer row remap) and every scroll frame re-streams the full region to
GRAM (~540 KB). The QSPI link (PIO at clkdiv 1, 4-bit @ 200 MHz ≈ 50 MB/s)
makes that ~10.8 ms/frame of TX at minimum.

Two optimizations carry sustained scroll from 46.8 to ~56 fps on typography
(measured via `scripts/rp2350-scroll-bench.py`, which drives the firmware's
`GEADEV DRAG` synthetic touch):

1. **Zero-copy span flush.** The scroll-region flush streams as ≤2 contiguous
   full-width framebuffer spans via 32-bit byte-swapped DMA from the uncached
   PSRAM alias (`panelFlushFullWidthSpans`), with the PIO autopull threshold
   switched 8→32 around the burst (the SM's shift state must be restarted at
   the switch or 24 stale bits misalign the stream — uniform hue corruption).
   No CPU copies; ~45 MB/s effective. 46.8 → 51.3 fps.
2. **Translate-only display-list maintenance** (engine,
   `RootScrollOnlyRefresh`): instead of a full clear()+recordNode() every
   scroll frame (~2.1 ms), the ENTIRE scroll content is pre-recorded once
   (record clip expanded to the content bounds), then each scroll frame just
   translates the scroll subtree's commands in place, un-translates the
   scroll node's own box, and patches the scrollbar thumb
   (`DisplayList::patchScrollbarThumb`). Scrolling never re-records; only a
   content change (dirty nodes → the plain rebuild path) or another record
   (`recordSerial()`) invalidates coverage. Content taller than ~16k px
   (int16 command coords) or a command-buffer overflow
   (`DisplayList::commandOverflow()`) falls back to a ±512 px window around
   the viewport. Verified pixel-identical to the rebuild path at both scroll
   clamps (`scripts/rp2350-scroll-parity.py`);
   `GEA_EMBEDDED_ROOT_SCROLL_TRANSLATE=0` restores the old behavior for A/B.
   51.3 → ~56 fps, and no periodic coverage-re-record hitch mid-fling.

Remaining scroll frame costs: ~12 ms TX-bound flush (the QSPI link is the
wall) + ~1.5 ms strip replay + ~0.6 ms scrollbar damage replay. Known dead
ends: DMA via the cached window thrashes the 16 KB XIP cache (net loss);
chained/fully-async DMA completion showed intermittent 100 ms+ stalls
(unexplained — parked; the machinery is still in rp2350_panel.cpp); uncached
byte-wide DMA runs ~15 MB/s. The last ~3 fps to 60 needs the flush TX
overlapped with engine work (fix the async stall) or fewer streamed bytes.
