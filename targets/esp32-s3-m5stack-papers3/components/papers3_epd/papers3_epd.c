// M5PaperS3 e-paper driver. See include/papers3_epd.h for the protocol summary.
//
// Scan/timing layer derived from Wenting Zhang's "Modos Smooth Graphics"
// PaperBoy (MIT) — the gate-start dance, the per-row latch sequence and the i80
// bus setup are its proven-on-this-board recipe. The calibrated four-state
// transition waveforms come from EPD_Painter's PaperS3 preset.
//
// ---------------------------------------------------------------------------
// WHY THIS WAVEFORM IS SHAPED THE WAY IT IS
//
// A previous e-paper bring-up in this repo destroyed a panel by re-driving
// every pixel toward its target on every frame: pixels already AT their target
// kept receiving one-way pulses for hours, which is sustained saturation DC and
// the cardinal E-Ink sin. The structural fix is that a pixel must stop being
// driven the moment it reaches its target.
//
// Here that is not a check bolted onto the renderer — it falls out of the
// transition selector. Unchanged pixels emit NOP for every phase. Changed
// pixels run one bounded calibrated waveform toward white or toward one of the
// three ink levels, then stop.
//
// Deliberately NOT ported from the LilyGo driver: its `swept16` current-limiting
// (splitting a solid field into 16 interleaved passes). That mitigated a damaged
// VCOM plane on that specific unit. This board's power design drives full-screen
// solid fields safely — upstream PaperBoy's own image mode does exactly that on
// this hardware — so sweeping here would cost 16x scan time for no benefit.
// ---------------------------------------------------------------------------

#include "papers3_epd.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "soc/gpio_reg.h"

static const char *TAG = "papers3_epd";

// ---- Pin map -------------------------------------------------------------
// Verified against upstream PaperBoy running on this board (main/msg/msg.h).
#define EPD_PWR_EN_PIN 45
#define EPD_BST_EN_PIN 46
#define EPD_SDCE_PIN   13 /* source driver chip enable  (i80 CS)  */
#define EPD_SDLE_PIN   15 /* source driver latch enable           */
#define EPD_SDCK_PIN   16 /* source driver clock        (i80 WR)  */
#define EPD_GDSP_PIN   17 /* gate driver start pulse              */
#define EPD_GDCK_PIN   18 /* gate driver clock                    */

#define EPD_D0_PIN 6
#define EPD_D1_PIN 14
#define EPD_D2_PIN 7
#define EPD_D3_PIN 12
#define EPD_D4_PIN 9
#define EPD_D5_PIN 11
#define EPD_D6_PIN 8
#define EPD_D7_PIN 10

#define EPD_XCK      24000000
#define EPD_BUSW     8
#define EPD_LINE_PAD 16

// ---- Command encoding ----------------------------------------------------
#define CMD_NOP   0x0
#define CMD_DARK  0x1
#define CMD_LIGHT 0x2
#define CMD_BOTH  0x3

#define ROW_CMD_BYTES (PAPERS3_EPD_WIDTH / 4)            /* 240 */
#define ROW_TOTAL     (ROW_CMD_BYTES + EPD_LINE_PAD)     /* 256 */
#define DRIVE_BYTES   (ROW_CMD_BYTES * PAPERS3_EPD_HEIGHT)

#define ROW_ALL_DARK  0x55 /* four CMD_DARK  */
#define ROW_ALL_LIGHT 0xaa /* four CMD_LIGHT */

// ---- Calibrated PaperS3 waveforms ---------------------------------------
// Ported from the PaperS3 preset used by CrossPoint's EPD_Painter driver. The
// crucial behavior is not merely these values: a non-white -> non-white change
// is first erased toward white, then painted from white on a second cycle.
// Treating the four panel states as a linear numeric distance made our software
// shadow diverge from the physical particles on every page turn.
//
// EPD_Painter is Apache-2.0: https://github.com/tonywestonuk/EPD_Painter
// CrossPoint PaperS3 port: https://github.com/juicecultus/crosspoint-reader-papers3
#define kHighWaveformFields 13
#define kFastWaveformFields 7

// Rows are light gray, dark gray, black. Commands are NOP/DARK/LIGHT/BOTH.
static const uint8_t kHighLighter[3][kHighWaveformFields] = {
	{1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2},
	{1, 3, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
};
static const uint8_t kHighDarker[3][kHighWaveformFields] = {
	// The stock PaperS3 preset makes both intermediate states too dark on our
	// panel. Keep its bounded staged shape, but reduce the net dark drive:
	// light gray gets 4 DARK / 8 LIGHT fields and dark gray inherits the old
	// light-gray waveform (7 DARK / 5 LIGHT). Black remains unchanged below.
	// This preserves two visible AA shades without weakening glyph stems.
	{1, 3, 1, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1},
	{1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};
static const uint8_t kFastLighter[3][kFastWaveformFields] = {
	{1, 2, 2, 2, 2, 2, 3},
	{3, 2, 2, 2, 2, 2, 3},
	{2, 2, 2, 2, 2, 2, 2},
};
static const uint8_t kFastDarker[3][kFastWaveformFields] = {
	// Seven-field compression of the tuned HIGH gray states: the new ink
	// reaches a useful density in roughly half the wall time while retaining
	// two distinct AA shades. Solid black still receives all seven DARK fields.
	{1, 3, 1, 2, 2, 2, 2},
	{1, 3, 1, 1, 2, 2, 1},
	{1, 1, 1, 1, 1, 1, 1},
};

// ---- Drive-duty safety ---------------------------------------------------
// A runaway caller (a render loop that refreshes every frame) is the failure
// mode that killed the previous panel. Cap sustained drive: past the budget we
// drop the rails and rest, which makes a runaway self-limiting instead of
// silently cooking the film.
#define kMaxFieldsPerMinute 1800
#define kRestMs             2000

static volatile bool dma_done = false;
static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t panel_io = NULL;

// Ping-pong buffers for the per-row computed (grayscale) path.
static uint8_t *dma_buf[2];
static int dma_cur;
// Immutable pre-built rows for the constant-command paths. These are never
// written after init, so re-sending one while the previous transfer of the SAME
// buffer is still in flight is harmless — which a memset-per-row would not be.
static uint8_t *row_nop;
static uint8_t *row_dark;
static uint8_t *row_light;
// EPD_Painter-style packed drive planes, four 2-bit physical states per byte.
// Preparing these once per image keeps the timed row scan to two byte lookups
// instead of re-quantizing and branching on all four pixels in every pass.
static uint8_t *drive_stage0_darker;
static uint8_t *drive_stage0_lighter;
static uint8_t *drive_stage1_darker;
// Source-line crosstalk settle. A pixel is driven through its source line
// (a physical column, which is a logical-portrait ROW). While a column carries
// many DARK fields -- bold glyph stems, a solid black button -- the pixels on
// it that stay white (and so NOP) are pulled slightly gray, which shows as a
// full-width gray band exactly as tall as the inked element. Pixels that stay
// white on a column that received ink drive get a short LIGHT pulse after the
// draw. It is bounded per draw and only touches columns this draw inked, so an
// idle screen is never re-driven.
#define kSettleFields 2
static uint8_t *drive_settle;
static uint8_t column_inked[PAPERS3_EPD_WIDTH];
// What is currently ON THE GLASS, GRAY4 packed exactly like the source
// framebuffer. Every drive is a delta against this, which is what lets a
// partial update erase instead of only adding ink. Lives in PSRAM (253 KB).
static uint8_t *shadow;
// Is `shadow` actually known to match the glass? FALSE after boot: e-paper
// RETAINS its last image across power loss and reflashing, so at startup the
// panel is showing whatever the previous firmware left there while `shadow`
// says "white". Drawing a delta against that lie leaves the old image on screen
// forever — every pixel where old and new agree computes zero delta and NOPs,
// so only the differing pixels get painted, on top of stale content.
//
// The shadow only becomes trustworthy after a real flash to white has actually
// been driven onto the glass.
static bool shadow_valid;
static bool powered;

// The continuous engine (below) keeps per-pixel drive state; any path that
// writes the glass outside it marks those rows idle.
static void engine_idle_rows(int y0, int y1);

static uint32_t total_fields;
static uint32_t window_fields;
static int64_t window_start_us;
static int64_t field_start_us;
static int64_t field_us_accum;
static int field_us_count;

static bool dma_done_cb(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ev, void *ctx)
{
	(void)io;
	(void)ev;
	(void)ctx;
	dma_done = true;
	return false;
}

// ---- Low-level scan ------------------------------------------------------

// Kept as gpio_set_level() to match upstream PaperBoy exactly.
//
// These were briefly swapped for direct GPIO_OUT_W1TS/W1TC register stores on
// the theory that SDLE (the source-driver latch) needs a wide pulse and that the
// HAL call's ~1us cost was providing it. MEASURED: field time was 20.4us/field
// with raw stores and 20.9us/field with gpio_set_level — no meaningful
// difference, so the HAL call is not the pulse width and the register write was
// not a speedup either. Reverted for fidelity to the reference implementation,
// not because it was proven harmful. If you optimise the row loop, measure the
// field time before AND after; do not infer it from present totals.
static void IRAM_ATTR send_row(const uint8_t *data)
{
	while (!dma_done)
		; // Spin: the gate is open, yielding here would stretch the row.

	gpio_set_level(EPD_GDCK_PIN, 0);
	gpio_set_level(EPD_SDLE_PIN, 1);
	gpio_set_level(EPD_SDLE_PIN, 0);
	gpio_set_level(EPD_GDCK_PIN, 1);

	dma_done = false;
	esp_lcd_panel_io_tx_color(panel_io, -1, data, ROW_TOTAL);
}

// Gate-driver start-of-frame sequence. Timings are upstream's; they are not
// arbitrary — the panel's shift register needs these exact pulse widths.
static void IRAM_ATTR frame_start(void)
{
	field_start_us = esp_timer_get_time();
	gpio_set_level(EPD_GDCK_PIN, 1);
	ets_delay_us(7);
	gpio_set_level(EPD_GDSP_PIN, 0);
	ets_delay_us(10);
	gpio_set_level(EPD_GDCK_PIN, 0);
	gpio_set_level(EPD_GDCK_PIN, 1);
	ets_delay_us(8);
	gpio_set_level(EPD_GDSP_PIN, 1);
	ets_delay_us(10);
	gpio_set_level(EPD_GDCK_PIN, 0);
	for (int i = 0; i < 2; i++) {
		gpio_set_level(EPD_GDCK_PIN, 1);
		gpio_set_level(EPD_GDCK_PIN, 0);
	}
}

// Flush the scan pipeline: the last real row has only been latched once.
static void IRAM_ATTR frame_end(void)
{
	send_row(row_nop);
	while (!dma_done)
		;
}

// Charge the duty budget for one field, resting the panel if it is exhausted.
static void account_field(void)
{
	const int64_t now = esp_timer_get_time();

	// Field-time telemetry. The scan floor (a field of constant rows, no
	// per-pixel CPU) is the hard ceiling on any animation rate, so measure it
	// rather than inferring it from present totals.
	if (field_start_us) {
		field_us_accum += now - field_start_us;
		if (++field_us_count >= 60) {
			ESP_LOGI(TAG, "field time avg %lldus over %d fields",
			         (long long)(field_us_accum / field_us_count), field_us_count);
			field_us_accum = 0;
			field_us_count = 0;
		}
	}
	if (now - window_start_us >= 60 * 1000 * 1000) {
		window_start_us = now;
		window_fields = 0;
	}
	total_fields++;
	if (++window_fields > kMaxFieldsPerMinute) {
		ESP_LOGW(TAG, "drive duty budget exhausted (%u fields/min) — resting rails off", (unsigned)window_fields);
		papers3_epd_power_off();
		vTaskDelay(pdMS_TO_TICKS(kRestMs));
		papers3_epd_power_on();
		window_start_us = esp_timer_get_time();
		window_fields = 0;
	}
}

// One field driving a single constant command, restricted to rows [y0, y1].
// Rows outside the band get NOP so the gate driver still advances.
static void field_solid_band(const uint8_t *solid_row, int y0, int y1)
{
	frame_start();
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; y++)
		send_row((y >= y0 && y <= y1) ? solid_row : row_nop);
	frame_end();
	account_field();
}

// CrossPoint/EPD_Painter's calibrated full clear: 6 dark, 2 light, 4 dark,
// 8 light. It is DC-balanced (10 fields in each direction) and ends white.
static void flash_band(int y0, int y1)
{
	for (int f = 0; f < 6; f++) field_solid_band(row_dark, y0, y1);
	for (int f = 0; f < 2; f++) field_solid_band(row_light, y0, y1);
	for (int f = 0; f < 4; f++) field_solid_band(row_dark, y0, y1);
	for (int f = 0; f < 8; f++) field_solid_band(row_light, y0, y1);
	if (shadow)
		memset(shadow + (size_t)y0 * PAPERS3_EPD_GRAY4_STRIDE, 0xff,
		       (size_t)(y1 - y0 + 1) * PAPERS3_EPD_GRAY4_STRIDE);
	engine_idle_rows(y0, y1);
	// The glass has now genuinely been driven white, so the shadow is true.
	if (y0 == 0 && y1 == PAPERS3_EPD_HEIGHT - 1) shadow_valid = true;
}

// Guarantee the shadow matches the glass before any delta drive. On the first
// draw after boot this costs one full white flash; afterwards it is free.
static void ensure_shadow_valid(void)
{
	if (shadow_valid) return;
	ESP_LOGI(TAG, "shadow unknown (panel retains a previous image) — flashing to a known white first");
	flash_band(0, PAPERS3_EPD_HEIGHT - 1);
}

// The band now shows `fb`.
static void shadow_commit(const uint8_t *fb, int y0, int y1)
{
	if (!shadow) return;
	const size_t off = (size_t)y0 * PAPERS3_EPD_GRAY4_STRIDE;
	memcpy(shadow + off, fb + off, (size_t)(y1 - y0 + 1) * PAPERS3_EPD_GRAY4_STRIDE);
	engine_idle_rows(y0, y1);
}

// Quantize the 16-level canvas into EPD_Painter's four stable physical states:
// 0=white, 1=light gray, 2=dark gray, 3=black. The deliberate boundary between
// GRAY4 7 and 6 preserves the e-reader's baked AA edge pair (7=light, 5=dark).
static inline uint8_t panel_level(uint8_t gray4)
{
	gray4 &= 0x0f;
	if (gray4 >= 14) return 0;
	if (gray4 >= 7) return 1;
	if (gray4 >= 2) return 2;
	return 3;
}

// stage 0 merges two pixel-disjoint directions into one row transmission:
// new ink on white uses the darker waveform, while changed existing ink uses
// the lighter waveform and lands at white. stage 1 paints the final level for
// non-white -> different-non-white transitions. This is EPD_Painter's two-cycle
// transition model expressed directly over our immutable GRAY4 snapshot.
static inline void prepare_pixel_transition(
	uint8_t target, uint8_t current, int shift,
	uint8_t *stage0Darker, uint8_t *stage0Lighter, uint8_t *stage1Darker)
{
	target = panel_level(target);
	current = panel_level(current);
	if (target == current) return;
	if (current == 0)
		*stage0Darker |= (uint8_t)(target << shift);
	else
		*stage0Lighter |= (uint8_t)(current << shift);
	if (current != 0 && target != 0)
		*stage1Darker |= (uint8_t)(target << shift);
}

static void prepare_transition_planes(const uint8_t *fb, int y0, int y1, bool *stage0Work, bool *stage1Work)
{
	*stage0Work = false;
	*stage1Work = false;
	memset(column_inked, 0, sizeof(column_inked));
	memset(drive_stage0_darker, 0, DRIVE_BYTES);
	memset(drive_stage0_lighter, 0, DRIVE_BYTES);
	memset(drive_stage1_darker, 0, DRIVE_BYTES);
	for (int y = y0; y <= y1; ++y) {
		const size_t srcOff = (size_t)y * PAPERS3_EPD_GRAY4_STRIDE;
		const size_t driveOff = (size_t)y * ROW_CMD_BYTES;
		for (int b = 0; b < ROW_CMD_BYTES; ++b) {
			const uint8_t t0 = fb[srcOff + 2 * b], t1 = fb[srcOff + 2 * b + 1];
			const uint8_t c0 = shadow[srcOff + 2 * b], c1 = shadow[srcOff + 2 * b + 1];
			uint8_t d0 = 0, l0 = 0, d1 = 0;
			prepare_pixel_transition(t0 >> 4, c0 >> 4, 6, &d0, &l0, &d1);
			prepare_pixel_transition(t0,      c0,      4, &d0, &l0, &d1);
			prepare_pixel_transition(t1 >> 4, c1 >> 4, 2, &d0, &l0, &d1);
			prepare_pixel_transition(t1,      c1,      0, &d0, &l0, &d1);
			drive_stage0_darker[driveOff + b] = d0;
			drive_stage0_lighter[driveOff + b] = l0;
			drive_stage1_darker[driveOff + b] = d1;
			// Only ink drive (toward a darker level) pulls its column gray.
			const uint8_t inked = (uint8_t)(d0 | d1);
			if (inked) {
				for (int p = 0; p < 4; ++p)
					if ((inked >> (6 - 2 * p)) & 3) column_inked[4 * b + p] = 1;
			}
			*stage0Work |= (d0 | l0) != 0;
			*stage1Work |= d1 != 0;
		}
	}
}

// The settle plane: CMD_LIGHT for every pixel that is white before and after
// this draw and sits on a column the draw inked. Returns whether any exists.
static bool prepare_settle_plane(const uint8_t *fb, int y0, int y1)
{
	if (!drive_settle) return false;
	bool any = false;
	memset(drive_settle, 0, DRIVE_BYTES);
	for (int y = y0; y <= y1; ++y) {
		const size_t srcOff = (size_t)y * PAPERS3_EPD_GRAY4_STRIDE;
		const size_t driveOff = (size_t)y * ROW_CMD_BYTES;
		for (int b = 0; b < ROW_CMD_BYTES; ++b) {
			uint8_t out = 0;
			for (int p = 0; p < 4; ++p) {
				const int x = 4 * b + p;
				if (!column_inked[x]) continue;
				const uint8_t t = (uint8_t)((fb[srcOff + (x >> 1)] >> ((x & 1) ? 0 : 4)) & 0x0f);
				const uint8_t c = (uint8_t)((shadow[srcOff + (x >> 1)] >> ((x & 1) ? 0 : 4)) & 0x0f);
				if (panel_level(t) == 0 && panel_level(c) == 0) out |= (uint8_t)(CMD_LIGHT << (6 - 2 * p));
			}
			drive_settle[driveOff + b] = out;
			any |= out != 0;
		}
	}
	return any;
}

static void field_settle(int y0, int y1)
{
	frame_start();
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; ++y) {
		if (y < y0 || y > y1) {
			send_row(row_nop);
			continue;
		}
		// Through the ping-pong DMA rows: they live in DMA-capable RAM and keep
		// the 16 pad bytes past the panel's last pixel NOP.
		uint8_t *out = dma_buf[dma_cur];
		memcpy(out, drive_settle + (size_t)y * ROW_CMD_BYTES, ROW_CMD_BYTES);
		send_row(out);
		dma_cur = !dma_cur;
	}
	frame_end();
	account_field();
}

static void build_waveform_map(uint8_t map[256], const uint8_t *waveform, int waveformFields, int pass)
{
	for (int packed = 0; packed < 256; ++packed) {
		uint8_t out = 0;
		for (int shift = 6; shift >= 0; shift -= 2) {
			const uint8_t state = (uint8_t)((packed >> shift) & 3);
			if (state != 0)
				out |= (uint8_t)(waveform[(state - 1) * waveformFields + pass] << shift);
		}
		map[packed] = out;
	}
}

static void field_transition(
	int y0, int y1, int stage, int pass,
	const uint8_t *lighter, const uint8_t *darker, int waveformFields)
{
	uint8_t lighterMap[256], darkerMap[256];
	build_waveform_map(lighterMap, lighter, waveformFields, pass);
	build_waveform_map(darkerMap, darker, waveformFields, pass);
	frame_start();
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; ++y) {
		if (y < y0 || y > y1) {
			send_row(row_nop);
			continue;
		}
		uint8_t *out = dma_buf[dma_cur];
		const size_t rowOff = (size_t)y * ROW_CMD_BYTES;
		const uint8_t *darkPlane = (stage == 0 ? drive_stage0_darker : drive_stage1_darker) + rowOff;
		const uint8_t *lightPlane = drive_stage0_lighter + rowOff;
		for (int b = 0; b < ROW_CMD_BYTES; ++b) {
			out[b] = darkerMap[darkPlane[b]] |
			         (stage == 0 ? lighterMap[lightPlane[b]] : 0);
		}
		send_row(out);
		dma_cur = !dma_cur;
	}
	frame_end();
	account_field();
}

static void draw_calibrated(
	const uint8_t *fb, int y0, int y1,
	const uint8_t *lighter, const uint8_t *darker, int waveformFields, int holdMs)
{
	if (!shadow || !drive_stage0_darker || !drive_stage0_lighter || !drive_stage1_darker) return;
	bool stage0Work = false, stage1Work = false;
	// Wall-time breakdown for one draw. The rolling telemetry in account_field()
	// gives the average field time; this gives the OTHER half — how much of a draw
	// is CPU transition-plane prep (PSRAM reads + quantize) versus the i80 field
	// scan, and how many fields the draw actually drove. That split is what tells
	// us whether the scan-loop rewrite (RMT gate clock / RGB-mode continuous scan)
	// would move the needle, or whether the cost is already CPU-side prep.
	const int64_t prep_start_us = esp_timer_get_time();
	prepare_transition_planes(fb, y0, y1, &stage0Work, &stage1Work);
	const int64_t scan_start_us = esp_timer_get_time();
	const uint32_t fields_before = total_fields;
	for (int stage = 0; stage < 2; ++stage) {
		if ((stage == 0 && !stage0Work) || (stage == 1 && !stage1Work)) continue;
		for (int pass = 0; pass < waveformFields; ++pass) {
			field_transition(y0, y1, stage, pass, lighter, darker, waveformFields);
			if (holdMs > 0) vTaskDelay(pdMS_TO_TICKS(holdMs));
		}
	}
	if ((stage0Work || stage1Work) && prepare_settle_plane(fb, y0, y1)) {
		for (int f = 0; f < kSettleFields; ++f) field_settle(y0, y1);
	}
	const int64_t scan_end_us = esp_timer_get_time();
	ESP_LOGI(TAG, "draw %d rows: prep %lldms, scan %lldms over %u fields",
	         y1 - y0 + 1,
	         (long long)((scan_start_us - prep_start_us) / 1000),
	         (long long)((scan_end_us - scan_start_us) / 1000),
	         (unsigned)(total_fields - fields_before));
	shadow_commit(fb, y0, y1);
}

// ---- Continuous per-pixel engine ----------------------------------------
// The scheme of PaperBoy's video mode (Modos Smooth Graphics), extended from
// its 1-bit pixels to the four calibrated levels above. The panel is scanned
// field after field while anything is changing. Every pixel carries its own
// position in its own transition, so a change that lands mid-scan starts on
// the very next field instead of waiting for an in-flight draw to finish.
// Each pixel stops being driven as soon as its bounded waveform ends.
//
// A transition is one waveform for white -> ink and ink -> white, and erase
// then paint (lighter, then darker) for ink -> other ink, as in the blocking
// path. `shadow` holds the level each pixel is being driven TO, and the state
// byte holds where its waveform started and how far into it the pixel is.
//
// Only rows with a pixel in flight are computed; all others go out as the
// immutable NOP row. The field period is held at the blocking path's measured
// ~26 ms so every field delivers the same dose the calibrated tables assume,
// and the spare time is slept, not spun, so the runtime keeps the core.
#define kFieldPeriodUs   26000
#define ENG_PHASE_IDLE   31
#define ENG_IDLE_BYTE    ((uint8_t)(ENG_PHASE_IDLE << 2))

// [7:2] phase (31 = idle), [1:0] panel level the waveform started from.
static uint8_t *pix_state;
// Active command-byte span of each physical row; lo > hi means idle.
static int16_t row_lo[PAPERS3_EPD_HEIGHT];
static int16_t row_hi[PAPERS3_EPD_HEIGHT];
static int active_rows;
static int burst_y0 = PAPERS3_EPD_HEIGHT, burst_y1 = -1;
static const uint8_t *eng_lighter = &kFastLighter[0][0];
static const uint8_t *eng_darker = &kFastDarker[0][0];
static int eng_fields = kFastWaveformFields;
static bool eng_want_high;
static int64_t last_field_start_us;
static uint32_t burst_fields;
static int64_t burst_start_us;

static inline int eng_length(uint8_t from, uint8_t to)
{
	if (from == to) return 0;
	return (from == 0 || to == 0) ? eng_fields : 2 * eng_fields;
}

static inline uint8_t eng_command(uint8_t from, uint8_t to, int phase)
{
	if (from == 0) return eng_darker[(to - 1) * eng_fields + phase];
	if (to == 0) return eng_lighter[(from - 1) * eng_fields + phase];
	return phase < eng_fields ? eng_lighter[(from - 1) * eng_fields + phase]
	                          : eng_darker[(to - 1) * eng_fields + phase - eng_fields];
}

static inline uint8_t shadow_nibble(int y, int x)
{
	const uint8_t b = shadow[(size_t)y * PAPERS3_EPD_GRAY4_STRIDE + (x >> 1)];
	return (x & 1) ? (uint8_t)(b & 0x0f) : (uint8_t)(b >> 4);
}

static void engine_idle_rows(int y0, int y1)
{
	if (!pix_state) return;
	for (int y = y0; y <= y1; ++y) {
		if (row_lo[y] <= row_hi[y]) --active_rows;
		row_lo[y] = ROW_CMD_BYTES;
		row_hi[y] = -1;
	}
	memset(pix_state + (size_t)y0 * PAPERS3_EPD_WIDTH, ENG_IDLE_BYTE,
	       (size_t)(y1 - y0 + 1) * PAPERS3_EPD_WIDTH);
}

static inline void engine_mark(int y, int byteIndex)
{
	if (row_lo[y] > row_hi[y]) {
		++active_rows;
		row_lo[y] = (int16_t)byteIndex;
		row_hi[y] = (int16_t)byteIndex;
	} else {
		if (byteIndex < row_lo[y]) row_lo[y] = (int16_t)byteIndex;
		if (byteIndex > row_hi[y]) row_hi[y] = (int16_t)byteIndex;
	}
	if (y < burst_y0) burst_y0 = y;
	if (y > burst_y1) burst_y1 = y;
}

bool papers3_epd_engine_available(void)
{
	return pix_state != NULL && shadow != NULL;
}

bool papers3_epd_engine_active(void)
{
	return active_rows > 0;
}

void papers3_epd_engine_set_high_quality(bool high)
{
	eng_want_high = high;
}

// Fold a new target frame into the in-flight drive. Only rows flagged in
// `rows` (or every row when it is NULL) are compared, and only pixels whose
// panel level changed get a new waveform.
void papers3_epd_engine_submit(const uint8_t *fb, const uint8_t *rows)
{
	if (!pix_state || !shadow || !fb) return;
	ensure_shadow_valid();
	if (active_rows == 0) {
		const bool high = eng_want_high;
		eng_lighter = high ? &kHighLighter[0][0] : &kFastLighter[0][0];
		eng_darker = high ? &kHighDarker[0][0] : &kFastDarker[0][0];
		eng_fields = high ? kHighWaveformFields : kFastWaveformFields;
	}
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; ++y) {
		if (rows && !rows[y]) continue;
		const size_t off = (size_t)y * PAPERS3_EPD_GRAY4_STRIDE;
		const uint8_t *src = fb + off;
		uint8_t *dst = shadow + off;
		if (memcmp(src, dst, PAPERS3_EPD_GRAY4_STRIDE) == 0) continue;
		uint8_t *state = pix_state + (size_t)y * PAPERS3_EPD_WIDTH;
		for (int i = 0; i < PAPERS3_EPD_GRAY4_STRIDE; ++i) {
			const uint8_t f = src[i];
			const uint8_t c = dst[i];
			if (f == c) continue;
			dst[i] = f;
			for (int half = 0; half < 2; ++half) {
				const uint8_t t = panel_level(half ? f : (uint8_t)(f >> 4));
				const uint8_t prev = panel_level(half ? c : (uint8_t)(c >> 4));
				if (t == prev) continue;
				const int x = 2 * i + half;
				const uint8_t st = state[x];
				const int phase = st >> 2;
				uint8_t from = prev;
				// Retargeted mid-waveform: past half way the pixel is nearer the
				// level it was heading to, before half way nearer where it began.
				if (phase != ENG_PHASE_IDLE) {
					const uint8_t oldFrom = st & 3;
					if (phase < eng_length(oldFrom, prev) / 2) from = oldFrom;
				}
				if (from == t) {
					state[x] = ENG_IDLE_BYTE;
					continue;
				}
				state[x] = from;
				engine_mark(y, x >> 2);
			}
		}
	}
	if (active_rows > 0 && burst_fields == 0) burst_start_us = esp_timer_get_time();
}

static void engine_settle(void)
{
	if (!drive_settle || burst_y1 < burst_y0) return;
	bool any = false;
	for (int y = burst_y0; y <= burst_y1; ++y) {
		const size_t driveOff = (size_t)y * ROW_CMD_BYTES;
		for (int b = 0; b < ROW_CMD_BYTES; ++b) {
			uint8_t out = 0;
			for (int p = 0; p < 4; ++p) {
				const int x = 4 * b + p;
				if (column_inked[x] && panel_level(shadow_nibble(y, x)) == 0)
					out |= (uint8_t)(CMD_LIGHT << (6 - 2 * p));
			}
			drive_settle[driveOff + b] = out;
			any |= out != 0;
		}
	}
	if (any)
		for (int f = 0; f < kSettleFields; ++f) field_settle(burst_y0, burst_y1);
}

// Drive one field of everything in flight. When the last pixel finishes, the
// source lines that carried ink get their settle pulse and the burst ends.
void papers3_epd_engine_field(void)
{
	if (!pix_state || active_rows == 0) return;
	// Hold the calibrated field period from one field start to the next.
	if (last_field_start_us) {
		const int64_t wait = last_field_start_us + kFieldPeriodUs - esp_timer_get_time();
		if (wait > 1500) vTaskDelay(pdMS_TO_TICKS((wait - 500) / 1000));
		const int64_t rest = last_field_start_us + kFieldPeriodUs - esp_timer_get_time();
		if (rest > 0 && rest < kFieldPeriodUs) ets_delay_us((uint32_t)rest);
	}
	last_field_start_us = esp_timer_get_time();
	frame_start();
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; ++y) {
		if (row_lo[y] > row_hi[y]) {
			send_row(row_nop);
			continue;
		}
		uint8_t *out = dma_buf[dma_cur];
		memset(out, 0, ROW_CMD_BYTES);
		uint8_t *state = pix_state + (size_t)y * PAPERS3_EPD_WIDTH;
		int lo = ROW_CMD_BYTES, hi = -1;
		for (int b = row_lo[y]; b <= row_hi[y]; ++b) {
			uint8_t cmd = 0;
			for (int p = 0; p < 4; ++p) {
				const int x = 4 * b + p;
				const uint8_t st = state[x];
				const int phase = st >> 2;
				if (phase == ENG_PHASE_IDLE) continue;
				const uint8_t from = st & 3;
				const uint8_t to = panel_level(shadow_nibble(y, x));
				const int length = eng_length(from, to);
				if (phase >= length) {
					state[x] = ENG_IDLE_BYTE;
					continue;
				}
				cmd |= (uint8_t)(eng_command(from, to, phase) << (6 - 2 * p));
				if (to != 0) column_inked[x] = 1;
				if (phase + 1 >= length) {
					state[x] = ENG_IDLE_BYTE;
				} else {
					state[x] = (uint8_t)(((phase + 1) << 2) | from);
					if (b < lo) lo = b;
					hi = b;
				}
			}
			out[b] = cmd;
		}
		send_row(out);
		dma_cur = !dma_cur;
		if (hi < lo) --active_rows;
		row_lo[y] = (int16_t)lo;
		row_hi[y] = (int16_t)hi;
	}
	frame_end();
	account_field();
	++burst_fields;
	if (active_rows == 0) {
		engine_settle();
		ESP_LOGI(TAG, "engine burst %lldms over %u fields, rows %d-%d",
		         (long long)((esp_timer_get_time() - burst_start_us) / 1000),
		         (unsigned)burst_fields, burst_y0, burst_y1);
		memset(column_inked, 0, sizeof(column_inked));
		burst_y0 = PAPERS3_EPD_HEIGHT;
		burst_y1 = -1;
		burst_fields = 0;
		last_field_start_us = 0;
	}
}

// ---- Public API ----------------------------------------------------------

void papers3_epd_init(void)
{
	if (panel_io) return;

	gpio_config_t cfg = {
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	cfg.pin_bit_mask = (1ull << EPD_PWR_EN_PIN) | (1ull << EPD_BST_EN_PIN) | (1ull << EPD_SDCE_PIN) |
	                   (1ull << EPD_SDLE_PIN) | (1ull << EPD_SDCK_PIN) | (1ull << EPD_GDSP_PIN) |
	                   (1ull << EPD_GDCK_PIN);
	ESP_ERROR_CHECK(gpio_config(&cfg));

	esp_lcd_i80_bus_config_t bus_cfg = {
		// The panel has no data/command line, but esp_lcd_new_i80_bus REQUIRES a
		// DC pin: passing -1 makes it fail with "configure GPIO failed" and the
		// board boot-loops. Upstream PaperBoy's trick is an out-of-range GPIO
		// (49 > the S3's max of 48) — the bus accepts the config and no real pin
		// is ever driven. The cost is one harmless "gpio_func_sel: GPIO number
		// error" line at boot. Do not "clean this up" to -1; it was tried.
		.dc_gpio_num = (gpio_num_t)49,
		.wr_gpio_num = (gpio_num_t)EPD_SDCK_PIN,
		.clk_src = LCD_CLK_SRC_PLL160M,
		.data_gpio_nums = {
			(gpio_num_t)EPD_D0_PIN, (gpio_num_t)EPD_D1_PIN,
			(gpio_num_t)EPD_D2_PIN, (gpio_num_t)EPD_D3_PIN,
			(gpio_num_t)EPD_D4_PIN, (gpio_num_t)EPD_D5_PIN,
			(gpio_num_t)EPD_D6_PIN, (gpio_num_t)EPD_D7_PIN,
		},
		.bus_width = EPD_BUSW,
		.max_transfer_bytes = ROW_TOTAL + 16,
		.dma_burst_size = 32,
	};
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_cfg, &i80_bus));

	esp_lcd_panel_io_i80_config_t io_cfg = {
		.cs_gpio_num = (gpio_num_t)EPD_SDCE_PIN,
		.pclk_hz = EPD_XCK,
		.trans_queue_depth = 4,
		.on_color_trans_done = dma_done_cb,
		.user_ctx = NULL,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
		.dc_levels = {
			.dc_idle_level = 0,
			.dc_cmd_level = 0,
			.dc_dummy_level = 0,
			.dc_data_level = 1,
		},
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_cfg, &panel_io));

	uint8_t **rows[5] = {&dma_buf[0], &dma_buf[1], &row_nop, &row_dark, &row_light};
	for (int i = 0; i < 5; i++) {
		*rows[i] = (uint8_t *)heap_caps_aligned_alloc(16, ROW_TOTAL + 16, MALLOC_CAP_DMA);
		if (!*rows[i]) {
			ESP_LOGE(TAG, "DMA row buffer allocation failed");
			return;
		}
		// Pad bytes past the panel's 960 pixels stay NOP for every row, always.
		memset(*rows[i], 0, ROW_TOTAL + 16);
	}
	memset(row_dark, ROW_ALL_DARK, ROW_CMD_BYTES);
	memset(row_light, ROW_ALL_LIGHT, ROW_CMD_BYTES);

	// Shadow of the glass. Nothing has been drawn yet and the first thing every
	// caller does is clear to white, so start it white (GRAY4 15 = 0xff).
	shadow = (uint8_t *)heap_caps_malloc(PAPERS3_EPD_GRAY4_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!shadow) {
		ESP_LOGE(TAG, "shadow buffer allocation failed (PSRAM required)");
		return;
	}
	memset(shadow, 0xff, PAPERS3_EPD_GRAY4_BYTES);

	drive_stage0_darker = (uint8_t *)heap_caps_malloc(DRIVE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	drive_stage0_lighter = (uint8_t *)heap_caps_malloc(DRIVE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	drive_stage1_darker = (uint8_t *)heap_caps_malloc(DRIVE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	// Optional: without it the settle step is skipped, nothing else changes.
	drive_settle = (uint8_t *)heap_caps_malloc(DRIVE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!drive_stage0_darker || !drive_stage0_lighter || !drive_stage1_darker) {
		ESP_LOGE(TAG, "transition plane allocation failed (PSRAM required)");
		return;
	}
	for (int y = 0; y < PAPERS3_EPD_HEIGHT; ++y) {
		row_lo[y] = ROW_CMD_BYTES;
		row_hi[y] = -1;
	}
	active_rows = 0;
	pix_state = (uint8_t *)heap_caps_malloc((size_t)PAPERS3_EPD_WIDTH * PAPERS3_EPD_HEIGHT,
	                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (pix_state)
		memset(pix_state, ENG_IDLE_BYTE, (size_t)PAPERS3_EPD_WIDTH * PAPERS3_EPD_HEIGHT);
	else
		ESP_LOGE(TAG, "engine state allocation failed; partial updates use the blocking path");

	dma_done = true;
	dma_cur = 0;
	window_start_us = esp_timer_get_time();
	ESP_LOGI(TAG, "init ok: %dx%d, %d bytes/row, %d Hz", PAPERS3_EPD_WIDTH, PAPERS3_EPD_HEIGHT,
	         ROW_TOTAL, EPD_XCK);
}

void papers3_epd_power_on(void)
{
	if (powered) return;
	gpio_set_level(EPD_PWR_EN_PIN, 1);
	ets_delay_us(100);
	gpio_set_level(EPD_BST_EN_PIN, 1);
	ets_delay_us(100);
	gpio_set_level(EPD_GDCK_PIN, 1);
	gpio_set_level(EPD_GDSP_PIN, 1);
	powered = true;
}

void papers3_epd_power_off(void)
{
	if (!powered) return;
	gpio_set_level(EPD_GDCK_PIN, 0);
	gpio_set_level(EPD_GDSP_PIN, 0);
	gpio_set_level(EPD_BST_EN_PIN, 0);
	ets_delay_us(100);
	gpio_set_level(EPD_PWR_EN_PIN, 0);
	ets_delay_us(100);
	powered = false;
}

void papers3_epd_clear(void)
{
	if (!panel_io) return;
	const bool was_powered = powered;
	papers3_epd_power_on();

	flash_band(0, PAPERS3_EPD_HEIGHT - 1);

	if (!was_powered) papers3_epd_power_off();
}

// Deghosting/recovery treatment, ported from the LilyGo T5 panel post-mortem
// (same ED047TC1 film chemistry; see that target's safety notes). Alternates
// balanced deep drive with rails-off zero-bias rest — trapped charge drains
// on the minutes timescale under zero bias, so the rest phases matter as much
// as the drive. Hard session cap: parks with rails off after 30 minutes;
// reboot/power-cycle for another session. The driver's own duty budget
// (account_field) inserts additional micro-rests throughout. Never returns.
void papers3_epd_treatment_forever(void)
{
	if (!panel_io) return;
	const int64_t session_start = esp_timer_get_time();
	int round = 0;
	for (;;) {
		if (esp_timer_get_time() - session_start > 30LL * 60 * 1000 * 1000) {
			papers3_epd_power_off();
			ESP_LOGW(TAG, "treatment: 30-minute session cap — parked (rails off); reboot for another session");
			for (;;) vTaskDelay(pdMS_TO_TICKS(60000));
		}
		round++;
		ESP_LOGI(TAG, "treatment round %d: ~4 min drive, then 3 min rails-off rest", round);
		papers3_epd_power_on();
		for (int c = 0; c < 10; c++) {
			// Deep balanced cycle: saturating pushes both directions plus the
			// panel's own calibrated flash (6d/2l/4d/8l, DC-balanced).
			for (int f = 0; f < 10; f++) field_solid_band(row_dark, 0, PAPERS3_EPD_HEIGHT - 1);
			vTaskDelay(pdMS_TO_TICKS(100));
			for (int f = 0; f < 10; f++) field_solid_band(row_light, 0, PAPERS3_EPD_HEIGHT - 1);
			vTaskDelay(pdMS_TO_TICKS(100));
			flash_band(0, PAPERS3_EPD_HEIGHT - 1);
		}
		papers3_epd_power_off();
		ESP_LOGI(TAG, "treatment round %d: resting (zero bias, 3 min)...", round);
		vTaskDelay(pdMS_TO_TICKS(180 * 1000));
	}
}

void papers3_epd_draw_gray4(const uint8_t *fb)
{
	if (!panel_io || !fb) return;
	const bool was_powered = powered;
	papers3_epd_power_on();
	ensure_shadow_valid();

	// Our measured i80 field is ~21 ms. That already matches EPD_Painter's
	// faster field scan plus its 8 ms QUALITY_HIGH hold, so adding another
	// delay here would over-dose every pulse.
	draw_calibrated(fb, 0, PAPERS3_EPD_HEIGHT - 1,
	                &kHighLighter[0][0], &kHighDarker[0][0],
	                kHighWaveformFields, 0);

	if (!was_powered) papers3_epd_power_off();
}

void papers3_epd_draw_fast_gray4(const uint8_t *fb)
{
	if (!panel_io || !fb) return;
	const bool was_powered = powered;
	papers3_epd_power_on();
	ensure_shadow_valid();

	draw_calibrated(fb, 0, PAPERS3_EPD_HEIGHT - 1,
	                &kFastLighter[0][0], &kFastDarker[0][0],
	                kFastWaveformFields, 0);

	if (!was_powered) papers3_epd_power_off();
}

void papers3_epd_refresh(const uint8_t *fb)
{
	papers3_epd_refresh_region(fb, 0, PAPERS3_EPD_HEIGHT - 1);
}

void papers3_epd_refresh_region(const uint8_t *fb, int y0, int y1)
{
	if (!panel_io || !fb) return;
	if (y0 < 0) y0 = 0;
	if (y1 > PAPERS3_EPD_HEIGHT - 1) y1 = PAPERS3_EPD_HEIGHT - 1;
	if (y0 > y1) return;

	const bool was_powered = powered;
	papers3_epd_power_on();
	// A band flash only makes the band trustworthy. If the whole panel is still
	// unknown (first draw after boot), clear all of it first — otherwise the
	// rows outside the band keep whatever the previous firmware left there.
	ensure_shadow_valid();

	// Flash the band back to a known white, then paint it. Both phases scan the
	// whole panel; only the band receives non-NOP commands.
	flash_band(y0, y1);
	draw_calibrated(fb, y0, y1,
	                &kHighLighter[0][0], &kHighDarker[0][0],
	                kHighWaveformFields, 0);

	if (!was_powered) papers3_epd_power_off();
}

uint32_t papers3_epd_field_count(void)
{
	return total_fields;
}
