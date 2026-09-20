# Embedded Board Workflow

This guide documents how embedded board work is organized. Every command is a
`gea` CLI command from `@geastack/cli`; this package ships sources, metadata,
and CMake only.

## Board Configuration

Before building or flashing, install the shared GeaStack toolchains from
[the `@geastack/cli` setup guide](https://github.com/geastack/cli/blob/main/docs/SETUP.md), especially Node.js, Python 3, and ESP-IDF v6.0.

Board aliases are machine-local configuration and never ship in a package.
The CLI merges two files, project over home:

- `~/.geastack/boards.json` -- every board on this machine (`GEA_HOME`
  relocates the directory);
- `<project>/.gea/boards.json` -- aliases specific to one application.

`gea boards add` registers a board interactively (`--global` / `--local` pick
the file), `gea boards discover` identifies the boards plugged in over USB
and reports their app and IP, `gea boards set <alias> host <ip>` edits one
field, and `gea boards list` shows what resolves and from which file. The
entry shapes for every adapter are documented in [the `@geastack/cli` setup guide](https://github.com/geastack/cli/blob/main/docs/SETUP.md).

Built-in target metadata lives in `targets.json` at the package root: adapter,
project directory, flash size, IDF target, and app platform per target id. The
CLI resolves a board alias to that entry (`cli/src/boards/resolve.mjs`).

## Everyday commands

```sh
npx gea doctor
npx gea build --board amoled --app watch
npx gea flash --board amoled --app watch
npx gea run --board amoled --app watch        # flash, then monitor
npx gea monitor --board amoled
npx gea ota --board amoled --app watch        # WiFi OTA (transports.ota.host)
npx gea ota --board amoled --app watch --logs # then follow the log over WiFi
npx gea logs --board amoled                   # WiFi when the board has a host, else USB
npx gea screenshot shot.png --board amoled
npx gea devctl state --board amoled           # GEADEV protocol verbs
npx gea devctl brightness 60 --board amoled   # display knobs: brightness, hbm, vsync
npx gea devctl hbm on --board amoled          # over USB or WiFi, whichever reaches the board
```

Every `devctl` verb, both wire protocols (`GEADEV` over USB serial and the
display routes on HTTP port 8080), and the transport rules are documented in
the CLI's [Device Control reference](https://github.com/geastack/cli/blob/main/docs/DEVICE-CONTROL.md).

Generated C++, CMake state, the build-local `sdkconfig`, and firmware images go
to the application's `.gea/build/<target>/app-builds/<app>/`.

## Target Folders

Concrete board targets live under `targets`.

Common files:

- `CMakeLists.txt`
- `dependencies.lock`
- `sdkconfig.defaults`
- `partitions.csv`
- target-specific `main` sources or component patches
- target-specific tests

Shared ESP32 sources live under `targets/esp32`. Board-specific target folders
should reuse shared sources and add only board composition, config, and
hardware-specific glue. The active app reaches CMake as
`GEA_EMBEDDED_APP` / `GEA_EMBEDDED_APP_META` (root;entry;runtime;native
sources), set by the CLI; targets never spawn node.

## Internal RAM Switches

Some boards need their internal SRAM for something other than the UI: a DMA
audio graph, a network stack, a large model. The framework keeps everything in
internal memory by default and lets an app move the UI's task-only state and
stacks out through `gea.defines`. Every switch is off unless the app sets it, so
a board that never asks sees no change.

| Define | What moves | Requires |
| --- | --- | --- |
| `GEA_EMBEDDED_UI_STATE_EXTERNAL=1` | The engine's caches and tables (`libgea_framework.a` `.bss`, plus the `display`, `runtime` and `tree_render` objects of the app's main component) to external RAM, through the shipped `targets/esp32/ldfragments/gea_ui_state_external.lf`. It also sets `GEA_EMBEDDED_UI_STATE_DYNAMIC_INIT=1` for the engine, so state with nonzero sentinels is `.bss` instead of `.data` and the sweep can see it (see `packages/engine/ui/state_init.h` in core). | `CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y`; configuration fails otherwise. |
| `GEA_EMBEDDED_RENDER_WORKER_STACK_EXTERNAL=1` | The render worker's stack (`targets/esp32/display.cpp`). | `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` |
| `GEA_EMBEDDED_APP_FRAME_TASK_STACK_EXTERNAL=1` | The frame task's stack, allocated in `start()` (`targets/esp32/services/frame_scheduler.cpp`). The control block and event queue stay internal. | `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` |
| `GEA_EMBEDDED_TOUCH_TASK_STACK_EXTERNAL=1` | The FT3168 touch controller object and its task stack (`targets/esp32/chip_bindings/touch/ft3168.cpp`). | `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` |
| `GEA_EMBEDDED_RUNTIME_TASK_STACK_EXTERNAL=1` | The runtime task's stack (board `app_main.cpp`). | `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` |
| `GEA_EMBEDDED_LAZY_GRADIENT_LUTS=1` | The gradient dither tables, allocated on first use instead of as static arrays (core, `packages/engine/ui/render.cpp`). | |
| `GEA_RUNTIME_COMPACT_ALLOCATION` | The compiler runtime's allocation pool: no per-size free lists or chunks, every cell is a plain heap block returned on free (compiler, `src/targets/cpp/runtime/gea_runtime.h`). | |

A stack in external RAM is only safe for a task that never runs while the flash
cache is disabled; the tasks above wait on queues, talk to peripherals and
render, and none of them writes flash. Nothing that is DMA'd or read from an
ISR with the cache off is covered by these switches: framebuffers, the frame
scheduler's control block and the display's transfer buffers stay internal.

An app that already ships its own linker fragment for the framework archive
should drop that mapping when it turns on `GEA_EMBEDDED_UI_STATE_EXTERNAL`;
ldgen rejects two mappings for one object.

## Chip Bindings

Generic chip code belongs in the core repo under `chips`. ESP-IDF adapters live
under `targets/esp32/chip_bindings`.

If a future non-ESP target uses the same chip, it should reuse generic chip
logic and implement its own target binding rather than importing ESP-IDF code.

## Device Helpers

Device access (serial GEADEV protocol, WiFi diagnostics stream, HTTP screenshot
and OTA server, BLE OTA) is implemented in `@geastack/cli` under `src/device`
and `src/esp32`. Board-specific bench tools stay next to their target
(`targets/<id>/tools/`).

## Tests

Shared ESP32 tests live under `targets/esp32/test`; target folders contain
focused tests for component manifests, logging defaults, runtime task
affinity, framebuffer behavior, and hardware-specific diagnostics. Board
resolution, sdkconfig policy, flashing, OTA and device transports are tested in
`@geastack/cli` (`cli/test`).

Run the smallest relevant test after a metadata change, then a representative
board command when hardware behavior changed.

## Verification Checklist

Before landing embedded target changes:

1. Run the target metadata tests.
2. Run `npx gea doctor`.
3. Build a representative app for the touched target.
4. Flash or monitor on hardware when the change affects deployment/runtime.
5. Update target README or notes for hardware behavior, SDK patches, or
   calibration changes.
