# ESP-IDF SDK patches

Patches applied to ESP-IDF to unlock features the stock SDK gates off.
Each patch is a standalone `git format-patch` output — applies cleanly with
`git am` on top of the SDK commit it targets.

## NOT needed on ESP-IDF 6.0+ (the supported version)

As of ESP-IDF **6.0**, both patches below are **obsolete — Espressif
upstreamed PSRAM SPI DMA**, verified against v6.0.1:

- `components/esp_driver_spi/src/gpspi/spi_common.c` now ships
  `.access_ext_mem = true` in the GDMA transfer config by default (this was
  the entire point of patch 0002).
- `components/esp_driver_spi/src/gpspi/spi_master.c` no longer gates TX
  buffers on `esp_ptr_dma_capable()` / no internal bounce-buffer path, so
  patch 0001 (widening `esp_ptr_dma_capable`) has no effect — on 6.0 that
  function is only read for a diagnostic log in the gea display backend.

So on 6.0 the PSRAM framebuffer DMAs to the QSPI panel **out of the box, no
SDK patching**. The gea backend's `esp_cache_msync(..., C2M)` before each
flush is still required and is present (targets/esp32/display.cpp).

One residual tuning difference: 6.0 defaults the SPI GDMA
`max_data_burst_size` to **32**; the 5.5 patch below used **64** for extra
headroom against SPI-TX-FIFO underrun at 80 MHz QSPI (underrun shows as
horizontal solid-colour bands). If you ever see banding under heavy flush on
6.0, bump that one line in `spi_common.c` to 64 — but 32 is the tested 6.0
default and renders cleanly here.

The patches remain below for building on **ESP-IDF ≤ 5.5.x** only.

## How to apply (ESP-IDF ≤ 5.5.x only)

```sh
cd "$IDF_PATH"
git checkout -b gea-psram-spi-dma v5.5.1     # or whatever you were on
git am /path/to/targets/targets/esp32/sdk-patches/*.patch
```

## How to roll back

```sh
cd "$IDF_PATH"
git checkout v5.5.1                          # back to the unpatched commit
git branch -D gea-psram-spi-dma              # optional: delete the patch branch
```

The patched branch (`gea-psram-spi-dma`) keeps your edits available — checkout
the unpatched ref to disable, checkout the patched branch to re-enable.

## Current patches

Both patches must be applied together — they are companion changes. The
first lets `spi_master`'s argument validation accept PSRAM; the second
configures the GDMA channel `spi_master` uses to actually fetch from PSRAM.
Either one alone leaves the LCD pipeline broken.

### `0001-esp_ptr_dma_capable-accept-PSRAM.patch`

**Target:** ESP-IDF v5.5.1 (commit `fcae3288`).
**File:** `components/esp_hw_support/include/esp_memory_utils.h`.

**What it does:** widens `esp_ptr_dma_capable()` so it returns true for PSRAM
buffers on SoCs that support PSRAM DMA (ESP32-S3 octal PSRAM). The base
inline check only recognised internal SRAM (`SOC_DMA_LOW..SOC_DMA_HIGH`).

**Why we need it:** `esp_lcd_panel_io_spi` → `spi_master.c` calls this check
on every transmit buffer. When the LCD framebuffer lives in PSRAM (which it
must — it's 411 KB, doesn't fit in internal RAM), the check fails and
`spi_master` allocates an internal bounce buffer the same size as the chunk
(64+ KB) and `memcpy`s into it. Two problems:

1. The internal heap is fragmented after WiFi/BT/staging allocations — the
   largest contiguous block is ~32 KB, so the bounce alloc fails outright
   with `ESP_ERR_NO_MEM` and the LCD draw is dropped.
2. Even when it succeeds, the bounce-copy negates the perf reason for
   wanting direct DMA in the first place.

With the patch the SPI master accepts the PSRAM pointer as-is. The caller
must call `esp_cache_msync(addr, size, ESP_CACHE_MSYNC_FLAG_DIR_C2M)` before
queuing the transaction to flush CPU writes to PSRAM (otherwise DMA reads
stale cache contents). The Gea display backend does this in
`flushFramebufferRect()`.

### `0002-gdma-access_ext_mem-spi_master.patch`

**Target:** ESP-IDF v5.5.1 (commit `fcae3288`).
**File:** `components/esp_driver_spi/src/gpspi/spi_common.c`.

**What it does:** two related changes to the GDMA channel `spi_master`
allocates:

1. Flips `gdma_transfer_config_t.access_ext_mem` from `false` to `true`.
2. Bumps `max_data_burst_size` from `16` to `64` bytes — matching what
   `esp_lcd_panel_rgb` uses by default for its PSRAM framebuffer reads.

Companion to patch 0001.

**Why we need it:** even after patch 0001 lets PSRAM pointers through
`spi_master`'s validation, the underlying GDMA channel `spi_master` allocates
for itself is explicitly configured to **not access external memory**:

```c
// TODO: add support to allow SPI transfer PSRAM buffer
gdma_transfer_config_t trans_cfg = {
    .max_data_burst_size = 16,
    .access_ext_mem = false,
};
```

ESP-IDF themselves left a `TODO` to add PSRAM support. Without the flag,
GDMA happily queues descriptors that reference PSRAM addresses but reads
garbage from the bus when it actually tries to fetch — you see horizontal
bands of solid colour on the LCD instead of the framebuffer contents.
Setting the flag to `true` enables the GDMA hardware path that resolves
PSRAM virtual addresses through the cache controller.

**Risk:** marginal performance cost for SPI transfers from internal RAM,
since the GDMA channel is now configured to support both. The cost is in
descriptor setup, not transfer rate. No correctness impact for non-PSRAM
sources.

**Upstream:** the TODO suggests Espressif will eventually do this themselves;
when they do, this patch becomes redundant. Until then, it's a local
override. Keep applied as long as you are building with ESP-IDF ≤ 5.5.x
and using PSRAM framebuffers.
