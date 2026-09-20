# ESP32-P4 full-resolution camera capture (to SD card)

Captures a **full-resolution (1920×1080) still** and a **1080p H.264 video** from the
Waveshare ESP32-P4's OV5647 camera using the board's **hardware JPEG / H.264
encoders**, writes them to the **microSD card**, then pulls the on-card files off
over the USB-serial console with a **chunked, CRC-verified transfer** — the pulled
files are bit-perfect copies of what's on the card.

## Quick start

```bash
# Build + flash the camera-studio app (1920x1080 sensor mode + HW H.264 + SD)
npx gea build --board waveshare-p4-7 --app camera-studio
npx gea flash --board waveshare-p4-7 --app camera-studio

# Capture a still + a 10s clip into targets/.../captures/
python3 targets/esp32-p4-waveshare-touch-lcd-7/tools/capture-camera.py

# Options
python3 tools/capture-camera.py --still-only
python3 tools/capture-camera.py --clip-only --seconds 10
```

Outputs (host copies; the originals also remain on the SD card as `/sdcard/frame.jpg`
and `/sdcard/clip.h264`):
- `captures/frame_<ts>.jpg`  — 1920×1080 hardware-JPEG still (~750 KB)
- `captures/clip_<ts>.h264`  — 1080p hardware-H.264 elementary stream
- `captures/clip_<ts>.mp4`   — remuxed with ffmpeg (if installed), plays anywhere

## How it works

**Sensor / encoders / SD** (`sdkconfig` / `sdkconfig.defaults`):
- `CONFIG_CAMERA_OV5647_MIPI_RAW10_1920x1080_30FPS` — the OV5647 driver's max mode
  (no higher/5MP mode exists), so 1080p is "full resolution" here.
- `CONFIG_ESP_VIDEO_ENABLE_HW_H264_VIDEO_DEVICE` — the P4 HW H.264 encoder as a V4L2
  M2M device at `/dev/video11`.
- `CONFIG_ESP_CONSOLE_UART_CUSTOM` + `…_BAUDRATE=921600` — fast console so the pull
  isn't UART-bound (host reads the baud-agnostic USB-JTAG side via the `IOSSIOSPEED`
  ioctl).
- `CONFIG_VFS_MAX_COUNT=16` — esp_video registers several `/dev/video*` VFS entries;
  the default of 8 leaves no slot for the `/sdcard` mount.
- `CONFIG_FATFS_SECTOR_512=y` + `CONFIG_FATFS_USE_LABEL=y` — SD cards use 512-byte
  sectors (IDF defaults to 4096), and the exFAT code path needs the volume-label
  option defined.
- **exFAT** (cards ≥64GB usually ship exFAT, which IDF's FATFS disables and exposes no
  Kconfig for) is enabled **without touching the ESP-IDF tree**: `main/fatfs_ffconf.h`
  is a repo-local copy of IDF's `ffconf.h` with `FF_FS_EXFAT=1`, force-included into
  the fatfs component from `main/CMakeLists.txt` (`-include …/fatfs_ffconf.h`). Its
  `#ifndef _FFCONF_DEFINED` guard makes IDF's own ffconf.h a no-op, so the override
  wins and the config is fully reproducible from the repo. If ESP-IDF is upgraded,
  re-sync `fatfs_ffconf.h` from the new `components/fatfs/src/ffconf.h`.

**Device** (`main/camera.cpp`, `CameraDriver`):
- Mounts the microSD at `/sdcard` (SDMMC slot 1, 4-bit, CLK43/CMD44/D0–D3=39–42, P4
  on-chip LDO channel 4 for the SD I/O rail).
- `captureStillToSd()` — one full-res frame → HW JPEG → `/sdcard/frame.jpg`.
- `recordH264ClipToFile()` — switches `/dev/video0` to YUV420, drains warmup frames,
  drives the `/dev/video11` M2M encoder for the duration into a **persistent PSRAM
  buffer** (allocated before the camera reconfiguration so the contiguous alloc
  succeeds; reused across captures), then **bulk-writes** the clip to SD. Buffering in
  RAM keeps the capture at ~15 fps — a per-frame fwrite to SD inside the loop halves
  it. `min_qp=22 @ 5 Mbps` → good 1080p, ~6–7 MB / 10s.

**GEADEV commands** (weak `geaHandleExtraDevCommand` hook in
`targets/esp32/services/device_control.cpp`, overridden in `camera.cpp` for the P4):
- `GEADEV CAMSTILL`              → writes `/sdcard/frame.jpg`, returns size
- `GEADEV CAMCLIP [seconds]`     → writes `/sdcard/clip.h264`, returns size/frames/fps
- `GEADEV STAT <path>`           → file size
- `GEADEV GET  <path> <off> <len>` → CRC-tagged base64 chunk (reflected CRC-32)

**Host** (`tools/capture-camera.py`): triggers the capture, then pulls the file in
64 KB chunks; each chunk is CRC-checked and re-requested on mismatch, so dropped
serial lines never corrupt the result.

## Notes / characteristics

- Capture runs at ~15 fps at 1080p (the synchronous capture→encode loop); a 10s clip
  is ~147 frames played at ~15 fps — a faithful 10 seconds of real time, **0 decode
  errors** (the CRC-verified pull is bit-perfect).
- The serial pull is the slow part (~20–25 KB/s → a 7 MB clip ≈ 4–5 min). It's fully
  automated and reliable; the originals also sit on the SD card if you'd rather pull
  the card. (Faster transfer would need WiFi/HTTP or USB mass-storage.)
- Recording freezes the on-screen preview for its duration (the camera is exclusively
  owned during capture).
