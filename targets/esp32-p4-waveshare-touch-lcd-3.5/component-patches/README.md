# Managed-component patches

Patches applied to **managed** (component-manager-fetched) components for this
board. Unlike `targets/esp32/sdk-patches/` (which patch the ESP-IDF tree by hand),
these are applied **automatically at build time**: the target's top-level
`CMakeLists.txt` runs `apply.sh` right after `project()` (i.e. after the component
manager fetches into the gitignored, regenerated `managed_components/`, and before
the components compile).

`apply.sh` is idempotent (a patch already present is skipped) and non-fatal (a
patch that no longer matches only warns — it never breaks the build), so it is safe
to re-run on every configure and survives component re-fetches.

## Patches

### `0001-esp_video-csi-raw-passthrough-p4-rev-lt-v3.patch`

**Component:** `espressif/esp_video` (0.8.0~3) — `src/device/esp_video_csi_device.c`.

**Why:** this board is an **ESP32-P4 rev v1.3**. On rev < v3.0 the MIPI-CSI *bridge*
cannot do RAW→RGB565 color conversion (hard-gated in IDF `esp_driver_cam`), but
esp_video's default pipeline asks it to → `VIDIOC_STREAMON failed: failed to
configure format conversion`, so the camera never streams.

**What it does:** sets `csi_config.data_type.bits_per_pixel = 16` **plus the bridge
pixel-filter range** (`data_type_min = 0x12`, `data_type_max = 0x2f`) in
`csi_video_start()`. The 16 bpp forces the CSI bridge into **custom-data-depth
pass-through** (no color conversion → the rev gate is never reached; the inline ISP
demosaics RAW→RGB565 instead), and sizes the CSI frame at the sensor's **unpacked
16-bit** RAW width. Without the 16-bit sizing the frame is sized at the packed RAW10
width (10 bpp) and the ISP fills only `10/16 = 62.5%` of the height, leaving a stale
band across the bottom of the preview.

**The filter range is mandatory on the custom path.** `esp_cam_new_csi_ctlr()` only
auto-fills the bridge's MIPI data-type pixel filter to `[0x12, 0x2f]` on the
`bits_per_pixel == 0` branch; on the custom-data-depth branch it copies
`data_type_min`/`data_type_max` verbatim. Leaving them 0 sets the filter to `[0, 0]`,
so the bridge passes **only** MIPI data type `0x00` and drops every RAW10 packet
(DT `0x2b`). The result: the camera *streams* (no `STREAMON` error) but delivers
**zero frames** — a blank preview where the UI shows but the viewfinder never paints.
`0x12..0x2f` is the same standard range the driver uses for `bits_per_pixel == 0` and
covers RAW8/10/12, YUV, RGB and embedded data.

To regenerate after editing the component: `diff -u <pristine> <patched>` with paths
rewritten to `a/managed_components/.../file` / `b/managed_components/.../file`.
