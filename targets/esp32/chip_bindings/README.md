# ESP32 Chip Bindings

This folder contains ESP-IDF bindings for standalone chips.

The chips themselves are not board-specific and not owned by ESP32. Generic
chip declarations and shared, target-independent logic belong under
`chips/<category>/<chip>`.

Files here are the ESP32 handoff for those chips:

- They may include ESP-IDF headers.
- They may use ESP32 services such as I2C, I2S, GPIO, tasks, or interrupts.
- They read board composition from the active board target's `board.h`.
- They implement framework platform surfaces such as display, power, touch,
  audio, and sensors by wiring the selected chip to ESP-IDF.

Another ESP32 board that uses the same chip should reuse these bindings and
provide a different board composition. A non-ESP target should keep the same
generic chip declarations under `chips/` and add its own target binding under
its target folder.

The long-term shape is:

```text
chips/audio/es8311/
  es8311.h / es8311.cpp          target-independent codec protocol or formats

targets/esp32/chip_bindings/audio/es8311.cpp
  ESP-IDF I2C/I2S/GPIO adapter using the generic ES8311 driver

targets/<other-platform>/chip_bindings/audio/es8311.cpp
  that platform's I2C/I2S/GPIO adapter using the same generic ES8311 driver
```

So a non-ESP board does not reuse the ESP-IDF binding. It reuses the generic
chip driver and supplies a platform binding for its bus/runtime APIs.

Any register maps, chip initialization sequences, sample conversion, packet
formatting, and chip-level state machines that are not inherently ESP-IDF
specific should be pulled down into `chips/`. Code in this folder should become
thin transport glue.

Concrete examples in the current tree:

- `displays/qspi_panel.h` is the shared interface `display.cpp` depends on.
  Each QSPI LCD binding (`co5300.cpp`, `sh8601.cpp`, …) implements `QspiPanel`
  and exports `displays::qspiPanel()` / `qspiPanelDriverName()`. Board targets
  pick the driver by linking one binding source file — no `#if` chains in
  `display.cpp`.
- `power/axp2101.cpp` adapts ESP-IDF I2C to `gea::chips::axp2101::RegisterBus`.
- `imu/qmi8658.cpp` adapts ESP-IDF I2C and FreeRTOS delay to
  `gea::chips::qmi8658::Driver`.
- `touch/ft3168.cpp` adapts ESP-IDF I2C, GPIO, interrupts, and task delivery to
  `gea::chips::ft3168::ControllerCore`.

If a future non-ESP board uses one of those chips, it should implement the same
small bus/runtime interfaces in its own target folder and reuse the code under
`chips/`.
