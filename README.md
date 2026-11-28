# @geastack/targets

Native embedded build targets for GeaStack. The package ships the ESP32 and
RP2350 board sources, board catalog, BLE/WiFi OTA clients, and build/flash
adapter used by the `gea` CLI.

The CLI analyzes application APIs to select camera, networking, and audio
capabilities. An app that both captures from the microphone and plays audio
gets acoustic echo cancellation (AEC) and full duplex automatically: the CLI
adds `GEA_AUDIO_ECHO_CANCELLATION=1` and `GEA_AUDIO_FULL_DUPLEX=1` to the
build. `"GEA_AUDIO_ECHO_CANCELLATION=0"` in the app's `gea.defines` opts out.
Playback alone enables neither (see [ES8311 audio duplex mode](#es8311-audio-duplex-mode)).
Resolved capabilities live in the generated build configuration, rather than
per-capability environment variables. Each ESP32 target keeps optional SDK
dependencies in `main/gea-dependencies.json`; the CLI generates an IDF
dependency component containing only the selected SDKs.

Applications normally install `@geastack/cli`, which brings this package with
it. This package in turn depends on `@geastack/compiler`,
`@geastack/geatsc-plugin-gea`, `@geastack/core`, `@geastack/host`,
`@geastack/engine`, `@geastack/elements`, `@geastack/chips`, and
`@geastack/geaos`.

## Quick start

```sh
npm create geastack@latest my-app   # pick an ESP32 or RP2350 target, or append: -- --targets web,esp32
cd my-app
npm install
npx gea doctor                      # checks the npm packages and toolchains (ESP-IDF, cmake, ninja; arm-none-eabi-gcc and picotool for RP2350)
npx gea setup                       # pick your board, name its alias, install or verify ESP-IDF
npx gea flash --board <alias> --monitor
```

RP2350 targets also need the Raspberry Pi Pico SDK with `PICO_SDK_PATH`
exported; `gea doctor` does not check for it.

`gea setup` suggests an alias for each known board (for example `amoled` for
the Waveshare ESP32-S3-Touch-AMOLED-2.06) and prints the exact flash command
when it finishes. With a single registered board, `--board` can be left out.
`npx gea build --board <alias>` compiles the firmware without flashing it.

The CLI resolves every native package from npm. Generated C++, CMake state,
sdkconfig, and firmware images are written to the application’s
`.gea/build/<target>/` directory.

Every board workflow is a `gea` command; this package has no build or flash
scripts of its own (a few targets carry small helper scripts, such as a
component patcher or a camera capture tool, which their READMEs describe).
`targets.json` at the package root is the target catalog the CLI reads
(`npx gea targets list`, run from inside an application project), and each
`targets/<id>/` directory is the matching ESP-IDF or Pico SDK project.
[docs/BOARD-WORKFLOW.md](docs/BOARD-WORKFLOW.md) describes the board workflow
in detail.

Board aliases (which physical unit answers to `--board <alias>`, its USB
serial, its IP) are machine-local configuration and are not part of this
package: the CLI reads `~/.geastack/boards.json` for every board on the
machine and the application's `.gea/boards.json` for project overrides.
`npx gea boards add` registers a board, `npx gea boards discover` identifies
the ones plugged in, and `npx gea boards list` shows what resolves.

## Supported boards

Generated from `targets.json`. Composed targets reuse another target's project
with a board definition from `targets/<id>.json`; the panel column is filled
where that definition names the display.

| Target id | MCU | Project | Panel | Docs |
| --- | --- | --- | --- | --- |
| `esp32-s3` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` |  | composable base |
| `esp32-s3-devkit-n16r8` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` | headless |  |
| `esp32-s3-waveshare-touch-lcd-7` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` | rgb_panel RGB 800×480 |  |
| `esp32-s3-elecrow-crowpanel-advance-7.0` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` | rgb_panel RGB 800×480 |  |
| `esp32-s3-touch-amoled-2.06` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` |  |  |
| `esp32-s3-touch-amoled-1.8` | ESP32-S3 | `esp32-s3-touch-amoled-1.8` |  |  |
| `esp32-s3-lilygo-t-display-s3-long` | ESP32-S3 | `esp32-s3-lilygo-t-display-s3-long` |  | [README](targets/esp32-s3-lilygo-t-display-s3-long/README.md) |
| `esp32-s3-touch-amoled-1.75` | ESP32-S3 | `esp32-s3-touch-amoled-1.75` |  |  |
| `esp32-s3-touch-amoled-1.75c` | ESP32-S3 | `esp32-s3-touch-amoled-1.75c` |  |  |
| `esp32-s3-epaper-1.54` | ESP32-S3 | `esp32-s3-epaper-1.54` |  |  |
| `esp32-s3-lilygo-t5-epaper-4.7` | ESP32-S3 | `esp32-s3-lilygo-t5-epaper-4.7` |  |  |
| `esp32-m5stack-m5paper` | ESP32 | `esp32-m5stack-m5paper` |  | [README](targets/esp32-m5stack-m5paper/README.md) |
| `esp32-s3-m5stack-sticks3` | ESP32-S3 | `esp32-s3-m5stack-sticks3` |  | [README](targets/esp32-s3-m5stack-sticks3/README.md) |
| `esp32-s3-elecrow-rotary-2.1` | ESP32-S3 | `esp32-s3-elecrow-rotary-2.1` |  |  |
| `esp32-p4-waveshare-touch-lcd-7` | ESP32-P4 | `esp32-p4-waveshare-touch-lcd-7` |  |  |
| `esp32-p4-m5stack-tab5` | ESP32-P4 | `esp32-p4-m5stack-tab5` |  | [README](targets/esp32-p4-m5stack-tab5/README.md) |
| `esp32-p4-waveshare-touch-lcd-3.5` | ESP32-P4 | `esp32-p4-waveshare-touch-lcd-3.5` |  |  |
| `esp32-p4` | ESP32-P4 | `esp32-p4-waveshare-touch-lcd-7` |  | composable base |
| `esp32-p4-elecrow-crowpanel-advanced-5.0` | ESP32-P4 | `esp32-p4-waveshare-touch-lcd-7` | rgb_panel RGB 800×480 |  |
| `rp2350-waveshare-touch-amoled-2.41` | RP2350 | `rp2350-waveshare-touch-amoled-2.41` |  | [README](targets/rp2350-waveshare-touch-amoled-2.41/README.md) |
| `rp2350-tufty-2350` | RP2350 | `rp2350-tufty-2350` |  | [README](targets/rp2350-tufty-2350/README.md) |
| `esp32-s3-m5stack-papers3` | ESP32-S3 | `esp32-s3-m5stack-papers3` |  | [README](targets/esp32-s3-m5stack-papers3/README.md) |
| `esp32-s3-touch-amoled-2.41` | ESP32-S3 | `esp32-s3-touch-amoled-2.41` |  | [README](targets/esp32-s3-touch-amoled-2.41/README.md) |
| `esp32-s3-waveshare-touch-lcd-4b` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` | rgb_panel RGB 480×480 | [docs/WAVESHARE-LCD-4B.md](docs/WAVESHARE-LCD-4B.md) |
| `esp32-s3-touch-lcd-3.5b` | ESP32-S3 | `esp32-s3-touch-lcd-3.5b` |  |  |
| `esp32-s3-touch-lcd-3.49` | ESP32-S3 | `esp32-s3-touch-lcd-3.49` |  |  |
| `esp32-s31-espressif-mosaico` | ESP32-S31 | `esp32-s31-espressif-mosaico` |  | [README](targets/esp32-s31-espressif-mosaico/README.md) |
| `esp32-s3-m5stack-stopwatch` | ESP32-S3 | `esp32-s3-m5stack-stopwatch` |  | [README](targets/esp32-s3-m5stack-stopwatch/README.md) |
| `esp32-s3-touch-lcd-1.54` | ESP32-S3 | `esp32-s3-touch-lcd-1.54` |  | [README](targets/esp32-s3-touch-lcd-1.54/README.md) |
| `esp32-s3-touch-amoled-2.06-sh8601` | ESP32-S3 | `esp32-s3-touch-amoled-2.06` | sh8601 QSPI 410×502 | [panel variant](#amoled-206-panel-variant) |

## App-local custom targets

An application can compose a custom target from a supported native base. The
board alias points to a target definition relative to `.gea/boards.json`:

```json
{
  "my-board": {
    "target": "my-board",
    "targetDefinition": "targets/my-board.json",
    "appPlatform": "esp32"
  }
}
```

The definition names its `extends` base, reusable chip drivers, buses, and GPIO
pins. `gea chips list`, `gea chips info`, `gea chips add`, and
`gea chips remove` edit those selections from the installed
`@geastack/chips/catalog.json`. During CMake configuration this package checks
the definition against that same catalog, generates `board.h`, rejects GPIO
collisions, and includes only the selected portable sources and ESP-IDF
bindings. The composable platform bases are `esp32-s3` and `esp32-p4`
(`"composable": true` in `targets.json`). `gea chips list` shows each chip and
whether it has an ESP-IDF binding (`esp32-idf`) or none yet (`unbound`); a
controller without a binding fails before compilation.

Every chip role is optional, so a bare module composes too. Omit `chips.display`
and the board is **headless**: `display_headless.cpp` gives it the same
canvas-backed `Display` every target has, rasterizing into PSRAM and flushing to
memory rather than a panel. The native test host runs the framework the same
way. Omit `chips.touch` and it takes the no-op `Touchscreen`.
A definition also carries its own `flashSize` and `partitions`, because those
are the module's geometry and not the base's.

See the `tutorials` repository's
`embedded-tutorials/02-custom-board-composition` course for a complete target
definition and firmware build.

## ES8311 audio duplex mode

The ESP32 ES8311 binding defaults to half duplex. An app that both captures
and plays audio gets full duplex with echo cancellation from the CLI, which
adds `GEA_AUDIO_ECHO_CANCELLATION=1` and `GEA_AUDIO_FULL_DUPLEX=1`; adding
`"GEA_AUDIO_ECHO_CANCELLATION=0"` to the app's `gea.defines` opts out and
restores half duplex. Any app can opt into full duplex without echo
cancellation with `"GEA_AUDIO_FULL_DUPLEX=1"` in `gea.defines`. These are
per-app settings, not board-wide ones. On the AMOLED 2.06, capture uses its
separate ES7210 ADC.

In full duplex, opening either direction preserves the other. RX uses 20 ms
blocks and does not hold the TX write lock while waiting for samples. Both
directions must use the same sample rate because they share I2S clocks; a
conflicting open fails without closing the existing stream. Capture is 16 kHz,
so simultaneous playback must also be 16 kHz. Closing playback leaves capture
running, and audio resources are released once neither direction uses them.

`GEA_AUDIO_FULL_DUPLEX=1` on its own exposes raw microphone audio and adds no
echo processing. Echo cancellation (`GEA_AUDIO_ECHO_CANCELLATION=1`) requires
full duplex and a 16 kHz device clock; the build fails if it is combined with
`GEA_AUDIO_FULL_DUPLEX=0`, and playback must open at 16 kHz in one or two
channels. Default half-duplex capture keeps its existing 2048-sample blocks
and playback arbitration.

`node targets/esp32/test/audio-full-duplex.test.mjs` compiles the production
open/capture methods in both modes, checks clock conflicts, and verifies that
a blocked RX read permits TX progress only in the opted-in mode. Its executable
uses the board's existing ignored `build/` directory.

With echo cancellation enabled, applications can scale the board's calibrated
post-AEC gain with `GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE`. Its default is `1.0f`;
`0.501187234f` reduces gain by 6 dB. Scaling happens before int16 clipping,
without changing ADC gain or the echo reference. Valid scales are 0 through 64.
`node targets/esp32/test/audio-aec-output-gain.test.mjs` verifies both AMOLED
calibrations and rejects invalid scales without creating build artifacts.

## License

This repo is the GeaStack **embedded board support** (ESP32, RP2350) and is
licensed **GPL-3.0-only** (see `LICENSE`). The GeaStack framework, compiler
and desktop/mobile targets are Apache-2.0; the embedded targets are where the
GPL applies. In practice:

- **Open-source firmware:** free.
- **Evaluation, prototypes, internal devices:** free. The GPL's conditions
  apply when you distribute, not when you use.
- **Closed-source firmware shipped through this target:** needs a commercial
  license. Contact [contact@geastack.com](mailto:contact@geastack.com) for commercial terms.
- **Applications that GeaOS firmware loads at runtime:** any license you choose,
  including closed source. The [GeaOS Application Exception](GEAOS-APP-EXCEPTION.md)
  exempts them from the GPL. GeaOS labels applications from developers outside
  the GeaStack developer program as coming from an unverified developer.

## AMOLED 2.06 panel variant

`esp32-s3-touch-amoled-2.06-sh8601` shares the existing `esp32-s3-touch-amoled-2.06` build project and pinout. Its composed definition selects the SH8601 software driver with [Waveshare’s startup sequence](https://github.com/waveshareteam/Waveshare-ESP32-components/blob/master/bsp/esp32_s3_touch_amoled_2_06/esp32_s3_touch_amoled_2_06.c), 410×502 resolution, a 22-column gap, 40 MHz QSPI, bitmap transfers, and two-row minimum flush chunks. The software profile was verified on hardware; the driver name does not establish the physical controller.

Use this variant when the original AMOLED profile leaves the display black or retains the startup image. The original target retains its defaults. Both use the same peripherals and wiring; no separate native board project is needed. The two-row floor preserves address-window alignment when radio startup reduces staging-buffer RAM.
