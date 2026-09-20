# @geastack/targets

Native embedded build targets for GeaStack. The package ships the ESP32 and
RP2350 board sources, board catalog, BLE/WiFi OTA clients, and build/flash
adapter used by the `gea` CLI.

Applications normally install `@geastack/cli`, which brings this package and
the required compiler, runtime, host, engine, elements, and chip packages with
it:

```sh
npm install @geastack/cli
npx gea setup
npx gea build --board amoled
npx gea flash --board amoled --monitor
```

The CLI resolves every native package from npm. Generated C++, CMake state,
sdkconfig, and firmware images are written to the application’s
`.gea/build/<target>/` directory.

Every board workflow is a `gea` command; this package contains no scripts of
its own. `targets.json` at the package root is the target catalog the CLI
reads (`npx gea targets list`), and each `targets/<id>/` directory is the
matching ESP-IDF or Pico SDK project.

Board aliases (which physical unit answers to `--board amoled`, its USB
serial, its IP) are machine-local configuration and are not part of this
package: the CLI reads `~/.geastack/boards.json` for every board on the
machine and the application's `.gea/boards.json` for project overrides.
`npx gea boards add` registers a board, `npx gea boards discover` identifies
the ones plugged in, and `npx gea boards list` shows what resolves.

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
bindings. The first composable platform base is `esp32-s3`; its complete
display and touch stack is CO5300, FT3168, AXP2101, QMI8658, and ES8311.
Controllers without an ESP32-S3 binding fail before compilation.

Every chip role is optional, so a bare module composes too. Omit `chips.display`
and the board is **headless**: `display_headless.cpp` gives it the same
canvas-backed `Display` every target has, rasterizing into PSRAM and flushing to
memory rather than a panel — the arrangement the native test host already runs
the whole framework on. Omit `chips.touch` and it takes the no-op `Touchscreen`.
A definition also carries its own `flashSize` and `partitions`, because those
are the module's geometry and not the base's.

See the `tutorials` repository's
`embedded-tutorials/02-custom-board-composition` course for a complete target
definition and firmware build.


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
