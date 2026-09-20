#pragma once

// Target-local contract between display.cpp and camera.cpp for the camera's
// NativeOverlay fast path on this board's portrait-scan ST7796.
//
// The panel's gate driver always scans the physical portrait lines; landscape
// orientations therefore keep the panel in portrait addressing and rotate at
// present time (see display.cpp applyPanelOrientation). For the camera that
// rotation is free: the PPA composes the sensor frame directly into a
// panel-ordered (portrait, pre-byteswapped) PSRAM buffer which the SPI DMA
// pushes without any CPU pixel work — the stock Waveshare demo's structure.

#include <cstdint>

namespace gea::platform::esp32_p4_waveshare::display_native {

// Maps a logical-orientation rect to panel-native (portrait) coordinates.
// Returns false when the panel addressing matches the logical orientation
// 1:1 (portrait orientations) — callers then use the normal canvas path.
// extraRotCcw is the quarter-turn count to ADD to a renderer's logical
// rotation so it lands panel-oriented.
bool panelMapLogicalRect(int lx, int ly, int lw, int lh,
                         int *px, int *py, int *pw, int *ph, int *extraRotCcw);

// Pushes a packed little-endian RGB565 pixel block to the panel at portrait
// coordinates, byteswapping to the panel's MSB-first stream through the
// display's internal staging band (the PPA cannot pre-swap: its byte_swap
// flag scrambles colors through the downscale filter). The buffer must be
// coherent in physical memory (e.g. PPA-written). Blocks until the SPI DMA
// completes.
bool presentPanelRect(const std::uint16_t *pixels, int px, int py, int w, int h);

// Asynchronous variant: hands the block to the panel-push worker task and
// returns, letting the caller compose the next frame into a different buffer
// while this one streams out. Back-pressured to one job in flight; the buffer
// must stay untouched until the NEXT submit returns (double-buffer and
// alternate).
bool presentPanelRectAsync(const std::uint16_t *pixels, int px, int py, int w, int h);

// Measured panel scan (vsync) period from the ST7796 GSCAN register, or 0
// when unreadable (tear sync disabled). Viewfinder-class pushes are gated to
// start at the scan top; see display.cpp waitForScanTop().
std::int64_t panelScanPeriodUs();

// Blocks until no async panel push is queued or streaming. Call when the
// viewfinder geometry changes, BEFORE the renderer repaints the freed region
// — otherwise a stale-rect push lands after the repaint and wipes the new UI
// with one dead frame of old-geometry camera content.
void waitPanelPushIdle();

// Fills a logical-framebuffer rect with black. The panel-native camera
// present never writes the logical framebuffer, so its camera region holds
// whatever was there last (often an old full-screen frame); any UI flush
// that overlaps it would paint that dead frame over the live screen. The
// camera calls this on viewfinder geometry changes so repaint-overlapping
// flushes show black until the next live present covers it.
void clearLogicalRect(int x, int y, int w, int h);

}  // namespace gea::platform::esp32_p4_waveshare::display_native
