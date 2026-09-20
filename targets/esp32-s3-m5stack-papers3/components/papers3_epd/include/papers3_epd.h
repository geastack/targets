// M5PaperS3 e-paper driver — ED047TC1-class 960x540 panel wired directly to the
// ESP32-S3 (no IT8951 controller, unlike the original M5Paper).
//
// The scan/timing layer is derived from Wenting Zhang's "Modos Smooth Graphics"
// PaperBoy (MIT), which runs on this exact board; the calibrated four-state
// waveforms come from EPD_Painter's PaperS3 preset.
//
// Panel command protocol — TWO BITS per pixel, 4 pixels per byte, MSB first:
//   0b00 = NOP (leave the pixel alone)   0b01 = drive dark
//   0b10 = drive light                  0b11 = balanced/both
// A "field" is one full 540-row scan. Calibrated sequences of these commands
// produce the panel's four stable physical levels.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PAPERS3_EPD_WIDTH  960
#define PAPERS3_EPD_HEIGHT 540

// Source framebuffer format: GRAY4, 2 pixels per byte, high nibble = left pixel
// (matches gea's pixel::packed), 0 = black .. 15 = white.
#define PAPERS3_EPD_GRAY4_STRIDE (PAPERS3_EPD_WIDTH / 2) /* 480 bytes per row */
#define PAPERS3_EPD_GRAY4_BYTES  (PAPERS3_EPD_GRAY4_STRIDE * PAPERS3_EPD_HEIGHT)

// Bring up GPIO, the i80 bus and the DMA row buffers. Does not power the panel.
void papers3_epd_init(void);

// Panel rails. Every drawing entry point below powers the rails up on entry and
// back down on exit, so callers only need these for explicit power management.
void papers3_epd_power_on(void);
void papers3_epd_power_off(void);

// Flash the panel to a clean white. DC-balanced (equal dark and light drive) and
// de-ghosting; use it on boot and periodically to reset accumulated ghosting.
void papers3_epd_clear(void);
// Deghosting/recovery treatment loop: balanced deep drive + rails-off rest,
// 30-minute session cap, never returns. See the function's comment.
void papers3_epd_treatment_forever(void);

// Render a full 960x540 GRAY4 framebuffer as a PARTIAL update: each pixel is
// driven from what is currently on the glass to its new level (dark where it
// must darken, light where it must lighten, untouched where unchanged), so this
// erases as well as draws. No prior clear is needed. Faster than
// papers3_epd_refresh() but lets ghosting build up over many updates.
//
// QUALITY HIGH: quantizes the GRAY4 canvas to white/light-gray/dark-gray/black,
// then uses EPD_Painter's 13-pass PaperS3 transition sequences. A change between
// two non-white states is erased to white and repainted in a second cycle.
void papers3_epd_draw_gray4(const uint8_t *fb);

// Same four-level state machine with EPD_Painter's shorter 7-pass FAST tables.
// This preserves gray pixels, but leaves more residue than the high path and is
// therefore intended for transient motion rather than e-reader page turns.
void papers3_epd_draw_fast_gray4(const uint8_t *fb);

// Clear-then-draw: the safe general-purpose "show this image" entry point.
void papers3_epd_refresh(const uint8_t *fb);

// Same as papers3_epd_refresh() but only rows [y0, y1] are touched; every row
// outside the band is scanned with NOPs so the gate driver still advances.
// Cheaper than a full refresh for a partial page update.
void papers3_epd_refresh_region(const uint8_t *fb, int y0, int y1);

// Cumulative field counter, for telemetry/debugging drive duty.
uint32_t papers3_epd_field_count(void);

#ifdef __cplusplus
}
#endif
