// Our own ED047TC1 driver for the LilyGo T5 4.7" E-Paper S3 (PCA9555 + TPS65185
// revision — the epdiy `lilygo_board_s3` variant).
//
// Implements the epd_driver.h subset the gea display backend uses. The panel is
// driven through the ESP32-S3 LCD peripheral in i80 mode via esp_lcd (our own
// DMA scan — the layer that will become the 60fps per-pixel engine).
//
// Pins + power from epdiy's src/board/lilygo_board_s3.c (data D0-D7 =
// {5,6,7,15,16,17,18,8}, edge clock CKH=4; direct-GPIO control CKV(gate)=48,
// STH(source start / i80 CS)=41, LEH(latch)=42, STV(gate start)=45; power/control
// over I2C SDA=39/SCL=40 via a PCA9555 @0x20 + TPS65185 @0x68).
//
// The frame scan (gate-start dance + per-row latch/clock/DMA order) mirrors
// Wenting Zhang's "Modos Smooth Graphics" (gitlab.com/zephray/paperboy msg.c,
// MIT) — the proven manual-GPIO sequence for this exact ED047TC1 panel — with
// OE/MODE driven over the PCA9555 (which the M5PaperS3 handles differently).
//
// STAGE 0: 1-bit black-on-white over EPD_IMAGE_FIELDS fields after a clear.

#include "epd_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gea_epd";

// ---- Pins (epdiy lilygo_board_s3) -------------------------------------------
#define PIN_D0 5
#define PIN_D1 6
#define PIN_D2 7
#define PIN_D3 15
#define PIN_D4 16
#define PIN_D5 17
#define PIN_D6 18
#define PIN_D7 8
#define PIN_CKH 4    // edge / pixel clock (i80 WR)
#define PIN_STH 41   // source start pulse (i80 CS)
#define PIN_CKV 48   // gate clock (direct GPIO)
#define PIN_LEH 42   // source latch (direct GPIO)
#define PIN_STV 45   // gate start (direct GPIO)
#define PIN_DC  9    // unused dummy DC for the i80 bus
#define PIN_CFG_SDA 39
#define PIN_CFG_SCL 40

// ---- PCA9555 (@0x20) --------------------------------------------------------
#define PCA_ADDR 0x20
#define PCA_REG_INPUT0  0
#define PCA_REG_OUTPUT0 2
#define PCA_REG_CONFIG0 6
#define PCA_OE       0x01
#define PCA_MODE     0x02
#define PCA_PWRUP    0x08
#define PCA_VCOM     0x10
#define PCA_WAKEUP   0x20
#define PCA_PWRGOOD  0x40
#define PCA_INT      0x80

// ---- TPS65185 (@0x68) -------------------------------------------------------
#define TPS_ADDR 0x68
#define TPS_REG_ENABLE 0x01
#define TPS_REG_VCOM1  0x03
#define TPS_REG_VCOM2  0x04
#define TPS_REG_UPSEQ0 0x09
#define TPS_REG_UPSEQ1 0x0A
#define TPS_REG_PG     0x0F
// Factory-calibrated for THIS panel: the T5 E-Paper S3 PRO factory firmware
// reads eps_vcom=1560 from NVS (seen on serial 2026-07-18). Wrong VCOM shows
// up as washed-out contrast and ghosting.
#define TPS_VCOM_MV    1560

#define EPD_LINE_BYTES (EPD_WIDTH / 4)   // 2 bits/pixel, 4 px/byte -> 240
#define EPD_LINE_PAD   16
#define EPD_XCK_HZ     12000000
#define EPD_IMAGE_FIELDS 12

// Source-command polarity: standard ED047TC1, same as PaperBoy's M5PaperS3
// (push_lut_b uses 0x1, push_lut_w uses 0x2): 0x1 darkens (toward black),
// 0x2 lightens (toward white). The earlier bring-up note claiming this board
// was swapped came from an ambiguous 1-bit scene — with the swap the e-reader
// rendered as a washed-out negative (clear ended black, text lightened, page
// darkened).
#define DRV_HOLD  0x0
#define DRV_DARK  0x1
#define DRV_LIGHT 0x2

static i2c_master_bus_handle_t s_i2c;
static i2c_master_dev_handle_t s_pca;
static i2c_master_dev_handle_t s_tps;
static uint8_t s_pca_power;   // sticky PWRUP|VCOM|WAKEUP bits held during operation
static esp_lcd_i80_bus_handle_t s_i80_bus;
static esp_lcd_panel_io_handle_t s_panel_io;
static volatile bool s_dma_done = true;
static uint8_t *s_dma[2];     // ping-pong DMA line buffers
static int s_cur;
static bool s_inited = false;

// Extra per-row gate-on time in µs. The bare scan only energizes each row for
// the ~21µs the next row's DMA takes — enough for the fast per-pixel engine's
// many-fields regime, but a WEAK push per field. Quality draws and panel
// repair extend the row-select window (epdiy's strong pushes do the same via
// its ckv_high_time), multiplying the charge transferred per field.
static int s_row_hold_us = 0;

void gea_epd_set_row_hold_us(int hold_us)
{
	s_row_hold_us = hold_us < 0 ? 0 : hold_us;
}

// ---- PANEL SAFETY LAYER ------------------------------------------------------
// The first panel was destroyed by sustained saturation DC: a renderer that
// re-drove at-target pixels every frame for hours, plus an inverted-polarity
// session, plus marathon repair loops. This layer makes those failure modes
// STRUCTURALLY impossible rather than avoidable. Everything below funnels
// through the one scan chokepoint (frame_start/send_row), so no field
// function — present or future — can bypass it.
//
//  1. Per-pixel signed DC ledger (1 byte/px, PSRAM): every DARK/LIGHT command
//     a pixel receives moves its ledger by a hold-scaled quantum; at the
//     bound the command is rewritten to NOP in the DMA row. Balanced content
//     hovers near zero forever; sustained one-way drive self-limits after
//     roughly two transitions' worth of charge.
//  2. Duty limiter: a cap on scans per minute, and a forced rails-off rest
//     after sustained continuous activity — a runaway build stops itself.
//  3. VCOM interlock: no scan drives the panel until the panel's OWN
//     calibration has been provided via gea_epd_set_vcom_mv(). Never reuse
//     another panel's value.
//  4. Experimental gating: repair/recovery/diagnostic waveforms refuse to run
//     unless explicitly enabled, and hard-stop at a 30-minute wall clock.
#define LEDGER_BOUND            96      // hold-scaled quanta ≈ ~2 transitions one-way
#define LEDGER_QUANTUM_US       50      // 1 quantum per started 50µs of row drive
#define MAX_SCANS_PER_MINUTE    900
#define FORCED_REST_AFTER_US    (120LL * 1000 * 1000)
#define FORCED_REST_MS          5000
#define EXPERIMENTAL_CAP_US     (30LL * 60 * 1000 * 1000)

static int8_t *s_ledger;                 // [EPD_HEIGHT][EPD_WIDTH], PSRAM
static bool s_scan_suppressed;           // set per frame_start; send_row NOPs the data
static int s_scan_row;
static bool s_vcom_set = false;
static unsigned s_vcom_mv;
static bool s_experimental = false;
static int64_t s_experimental_started_us;
static uint32_t s_scans_total, s_scans_this_minute, s_px_clamped;
static int64_t s_minute_epoch_us, s_activity_start_us, s_last_scan_us, s_last_telemetry_us, s_last_refuse_log_us;

static void epd_poweron_internal(void);

// Decide whether the scan that is about to start may DRIVE the panel. Runs
// the duty limiter, the forced-rest scheduler, and telemetry. A refused scan
// still clocks through mechanically but carries all-NOP data (zero drive).
static bool field_scan_permitted(void)
{
	const int64_t now = esp_timer_get_time();
	if (now - s_last_telemetry_us > 60LL * 1000 * 1000) {
		s_last_telemetry_us = now;
		ESP_LOGI(TAG, "safety: scans=%lu clamped_px=%lu rate=%lu/min",
		         (unsigned long)s_scans_total, (unsigned long)s_px_clamped, (unsigned long)s_scans_this_minute);
	}
	if (!s_vcom_set || !s_ledger) {
		if (now - s_last_refuse_log_us > 5LL * 1000 * 1000) {
			s_last_refuse_log_us = now;
			ESP_LOGE(TAG, "safety: drive locked (%s) — all scans emitted as NOP",
			         !s_vcom_set ? "panel VCOM not set" : "ledger allocation failed");
		}
		return false;
	}
	if (now - s_minute_epoch_us > 60LL * 1000 * 1000) {
		s_minute_epoch_us = now;
		s_scans_this_minute = 0;
	}
	if (++s_scans_this_minute > MAX_SCANS_PER_MINUTE) {
		if (now - s_last_refuse_log_us > 5LL * 1000 * 1000) {
			s_last_refuse_log_us = now;
			ESP_LOGW(TAG, "safety: duty limit (%d scans/min) — dropping drive until the window resets", MAX_SCANS_PER_MINUTE);
		}
		return false;
	}
	// Idle gaps reset the continuous-activity clock; sustained activity earns
	// a mandatory rails-off zero-bias rest.
	if (now - s_last_scan_us > 2LL * 1000 * 1000) s_activity_start_us = now;
	s_last_scan_us = now;
	if (now - s_activity_start_us > FORCED_REST_AFTER_US) {
		ESP_LOGW(TAG, "safety: %llds continuous drive — forced %dms rails-off rest",
		         (long long)((now - s_activity_start_us) / 1000000), FORCED_REST_MS);
		epd_poweroff();
		vTaskDelay(pdMS_TO_TICKS(FORCED_REST_MS));
		epd_poweron_internal();
		s_activity_start_us = esp_timer_get_time();
		s_last_scan_us = s_activity_start_us;
	}
	s_scans_total++;
	return true;
}

static bool experimental_ok(const char *who)
{
	if (!s_experimental) {
		ESP_LOGE(TAG, "%s refused: experimental waveforms not enabled (call gea_epd_enable_experimental)", who);
		return false;
	}
	if (esp_timer_get_time() - s_experimental_started_us > EXPERIMENTAL_CAP_US) {
		ESP_LOGE(TAG, "%s stopped: 30-minute experimental wall-clock cap reached", who);
		return false;
	}
	return true;
}

void gea_epd_set_vcom_mv(unsigned mv)
{
	if (mv < 100 || mv > 5000) {
		ESP_LOGE(TAG, "rejected VCOM %umV (sane range 100..5000)", mv);
		return;
	}
	s_vcom_mv = mv;
	s_vcom_set = true;
	ESP_LOGI(TAG, "panel VCOM set to %umV", mv);
}

void gea_epd_enable_experimental(void)
{
	s_experimental = true;
	s_experimental_started_us = esp_timer_get_time();
	ESP_LOGW(TAG, "experimental waveforms ENABLED (30-minute wall-clock cap)");
}

// ---- I2C helpers ------------------------------------------------------------

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_master_transmit(dev, buf, 2, 1000);
}

static uint8_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg)
{
	uint8_t val = 0;
	i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 1000);
	return val;
}

static inline void pca_set_config(uint8_t v) { reg_write(s_pca, PCA_REG_CONFIG0 + 1, v); }
static inline void pca_set_output(uint8_t v) { reg_write(s_pca, PCA_REG_OUTPUT0 + 1, v); }
static inline uint8_t pca_read_input(void)   { return reg_read(s_pca, PCA_REG_INPUT0 + 1); }

// Set OE / MODE while keeping the sticky power bits. Slow (I2C); only called at
// frame boundaries, never per row.
static void pca_ctrl(bool oe, bool mode)
{
	pca_set_output((uint8_t)((oe ? PCA_OE : 0) | (mode ? PCA_MODE : 0) | s_pca_power));
}

static inline void tps_set_vcom(unsigned mV)
{
	unsigned val = mV / 10;
	reg_write(s_tps, TPS_REG_VCOM2, (uint8_t)((val & 0x100) >> 8));
	reg_write(s_tps, TPS_REG_VCOM1, (uint8_t)(val & 0xFF));
}

// ---- DMA row scan (PaperBoy sequence) ---------------------------------------

static bool IRAM_ATTR dma_done_cb(esp_lcd_panel_io_handle_t io,
                                  esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
	(void)io; (void)ed; (void)ctx;
	s_dma_done = true;
	return false;
}

// Wait for the previous line's DMA, advance one gate line (CKV) latching the
// previously-shifted source data (LEH), then start shifting THIS row's data.
// Pipelined: the data given here is latched+driven on the NEXT call.
static void send_row(uint8_t *row)
{
	// SAFETY chokepoint: every drive command in every scan passes through
	// here. A suppressed scan (duty/interlock refusal) is stripped to NOP;
	// otherwise the per-pixel DC ledger is charged and out-of-budget pixels
	// are individually rewritten to NOP before the data reaches the panel.
	if (s_scan_suppressed) {
		memset(row, 0, EPD_LINE_BYTES);
	} else if (s_ledger && s_scan_row < EPD_HEIGHT) {
		const int inc = 1 + s_row_hold_us / LEDGER_QUANTUM_US;
		int8_t *lrow = s_ledger + (size_t)s_scan_row * EPD_WIDTH;
		for (int xb = 0; xb < EPD_LINE_BYTES; xb++) {
			uint8_t b = row[xb];
			if (!b) continue;
			for (int p = 0; p < 4; p++) {
				const uint8_t cmd = (uint8_t)((b >> (p * 2)) & 3u);
				if (cmd != DRV_DARK && cmd != DRV_LIGHT) continue;
				int8_t *l = &lrow[xb * 4 + p];
				const int next = (cmd == DRV_DARK) ? (int)*l + inc : (int)*l - inc;
				if (next > LEDGER_BOUND || next < -LEDGER_BOUND) {
					b &= (uint8_t)~(3u << (p * 2));
					s_px_clamped++;
				} else {
					*l = (int8_t)next;
				}
			}
			row[xb] = b;
		}
	}
	s_scan_row++;
	while (!s_dma_done) {}
	gpio_set_level(PIN_CKV, 0);
	gpio_set_level(PIN_LEH, 1);
	gpio_set_level(PIN_LEH, 0);
	gpio_set_level(PIN_CKV, 1);
	// Extend the gate-on window: the just-latched source data keeps driving
	// the selected row for the entire CKV-high period.
	if (s_row_hold_us) esp_rom_delay_us((uint32_t)s_row_hold_us);
	s_dma_done = false;
	esp_lcd_panel_io_tx_color(s_panel_io, -1, row, EPD_LINE_BYTES + EPD_LINE_PAD);
}

// Reset the gate driver to the first line (PaperBoy gate-start dance;
// STV = gate start, CKV = gate clock).
static void frame_start(void)
{
	s_scan_row = 0;
	s_scan_suppressed = !field_scan_permitted();
	gpio_set_level(PIN_CKV, 1);
	esp_rom_delay_us(7);
	gpio_set_level(PIN_STV, 0);
	esp_rom_delay_us(10);
	gpio_set_level(PIN_CKV, 0);
	gpio_set_level(PIN_CKV, 1);
	esp_rom_delay_us(8);
	gpio_set_level(PIN_STV, 1);
	esp_rom_delay_us(10);
	gpio_set_level(PIN_CKV, 0);
	for (int i = 0; i < 2; i++) {
		gpio_set_level(PIN_CKV, 1);
		gpio_set_level(PIN_CKV, 0);
	}
}

// One field: reset the gate, stream every row, then one trailing dummy line so
// the last real row gets latched+driven (pipeline flush).
// 4x4 ordered (Bayer) dither: a per-pixel threshold so 16-level grayscale
// renders as black/white patterns on this 1-bit-per-field drive. Pure black
// (g=0) is always dark, pure white (g=15) always light, mids dither.
static const uint8_t kBayer[4][4] = {
	{ 0,  8,  2, 10},
	{12,  4, 14,  6},
	{ 3, 11,  1,  9},
	{15,  7, 13,  5},
};

// One field driving each pixel toward its dithered target — or, with the
// commands swapped, toward the INVERSE of the target (the negative-image
// phase of a classic flashing full refresh).
static void field_from_rows_cmds(const uint8_t *fb, uint8_t drv_dark, uint8_t drv_light)
{
	frame_start();
	const int in_stride = EPD_WIDTH / 2;   // 4bpp source rows
	for (int y = 0; y < EPD_HEIGHT; y++) {
		uint8_t *out = s_dma[s_cur];
		const uint8_t *in = fb + (size_t)y * in_stride;
		const uint8_t *bm = kBayer[y & 3];
		for (int xb = 0; xb < EPD_LINE_BYTES; xb++) {
			const uint8_t b0 = in[xb * 2 + 0];
			const uint8_t b1 = in[xb * 2 + 1];
			// The 4 pixels of this output byte are at x = xb*4 + {0,1,2,3}.
			const uint8_t p0 = b0 & 0x0F, p1 = (b0 >> 4) & 0x0F;
			const uint8_t p2 = b1 & 0x0F, p3 = (b1 >> 4) & 0x0F;
			// Drive EVERY pixel toward its (dithered) target each field so a
			// moving scene erases its previous frame instead of ghosting.
			// Threshold rule: dark iff (p <= bayer && p != 15). With 16 gray
			// levels against a 16-cell matrix one endpoint must saturate by
			// construction — pin BOTH: pure black (p=0, <= every cell) is
			// always driven dark and pure white (p=15) always light. A plain
			// `p < bm` left 1/16 of pure-black pixels driven LIGHT every
			// field (the bayer-0 cell), visibly graying solid text.
			uint8_t v = 0;
			v |= (uint8_t)(((p0 <= bm[0] && p0 != 15) ? drv_dark : drv_light) << 0);
			v |= (uint8_t)(((p1 <= bm[1] && p1 != 15) ? drv_dark : drv_light) << 2);
			v |= (uint8_t)(((p2 <= bm[2] && p2 != 15) ? drv_dark : drv_light) << 4);
			v |= (uint8_t)(((p3 <= bm[3] && p3 != 15) ? drv_dark : drv_light) << 6);
			out[xb] = v;
		}
		memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
		send_row(out);
		s_cur ^= 1;
	}
	// Trailing dummy line flushes the last real row through the pipeline.
	memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
	send_row(s_dma[s_cur]);
	s_cur ^= 1;
	while (!s_dma_done) {}
}

static void field_from_rows(const uint8_t *fb)
{
	field_from_rows_cmds(fb, DRV_DARK, DRV_LIGHT);
}

static void field_uniform(uint8_t cmd);
static void field_uniform_masked(uint8_t cmd, int parity);
static void field_uniform_swept16(uint8_t cmd);
static void field_uniform_swept_range(uint8_t cmd, int byte_lo, int byte_hi);

// The "shake": rapid whole-panel black/white alternation, one field per
// direction — the activation phase of commercial e-paper waveforms (the
// familiar full-refresh flicker). Alternation, not sustained push, is what
// loosens stuck particles and clears latent charge.
static void shake(int alternations, int hold_us)
{
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = hold_us;
	for (int i = 0; i < alternations; i++) {
		field_uniform_swept16(DRV_DARK);
		field_uniform_swept16(DRV_LIGHT);
	}
	s_row_hold_us = saved_hold;
}

// Drive the whole panel toward one 2-bit command for `fields` fields.
static void field_uniform(uint8_t cmd)
{
	const uint8_t byte = (uint8_t)((cmd << 6) | (cmd << 4) | (cmd << 2) | cmd);
	frame_start();
	for (int y = 0; y < EPD_HEIGHT; y++) {
		uint8_t *out = s_dma[s_cur];
		memset(out, byte, EPD_LINE_BYTES);
		memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
		send_row(out);
		s_cur ^= 1;
	}
	memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
	send_row(s_dma[s_cur]);
	s_cur ^= 1;
	while (!s_dma_done) {}
}

// One field driving HALF the pixels (x parity == `parity`) toward `cmd`, the
// other half NOP. A raw solid field makes all 960 source outputs in a row
// drive the SAME polarity at once — worst-case current on one supply rail —
// and the far end of the source chain (portrait bottom) sees sagged rails and
// never reaches full drive. Ground truth on THIS panel (2026-07-20): solid
// fills leave a bottom halo that checkerboard patterns never show, because a
// checker splits the row's load between the two rails. Driving solids as two
// complementary half-masked passes delivers the same charge to every pixel at
// checker-level per-row load.
static void field_uniform_masked(uint8_t cmd, int parity)
{
	const uint8_t byte = parity
		? (uint8_t)((cmd << 6) | (DRV_HOLD << 4) | (cmd << 2) | DRV_HOLD)
		: (uint8_t)((DRV_HOLD << 6) | (cmd << 4) | (DRV_HOLD << 2) | cmd);
	frame_start();
	for (int y = 0; y < EPD_HEIGHT; y++) {
		uint8_t *out = s_dma[s_cur];
		memset(out, byte, EPD_LINE_BYTES);
		memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
		send_row(out);
		s_cur ^= 1;
	}
	memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
	send_row(s_dma[s_cur]);
	s_cur ^= 1;
	while (!s_dma_done) {}
}

// GROUND TRUTH on this panel (user, 2026-07-20): checker fields (both
// polarities per row, net VCOM return current ~0) drive the WHOLE panel
// including the halo region; single-polarity fields — full solids AND
// half-masked solids (net 480 of 960 outputs one way) — leave the halo
// region undriven. The common electrode sags under net return current and
// the far region loses its effective drive voltage. So a solid may only be
// built from passes whose net current is a small fraction of a row: drive
// pixels with (x % 16 == group) — 60 of 960 outputs (~6%) — and sweep all
// 16 groups for full coverage. ~16x the scan time of a raw solid; solids
// are clears/flashes, not the animation path, so that's fine.
static void field_uniform_swept16(uint8_t cmd)
{
	for (int group = 0; group < 16; group++) {
		// 4-byte tile covering 16 pixels; exactly one driven per tile.
		uint8_t tile[4] = { 0, 0, 0, 0 };
		tile[group >> 2] = (uint8_t)(cmd << ((group & 3) * 2));
		frame_start();
		for (int y = 0; y < EPD_HEIGHT; y++) {
			uint8_t *out = s_dma[s_cur];
			for (int xb = 0; xb < EPD_LINE_BYTES; xb++) out[xb] = tile[xb & 3];
			memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
			send_row(out);
			s_cur ^= 1;
		}
		memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
		send_row(s_dma[s_cur]);
		s_cur ^= 1;
		while (!s_dma_done) {}
	}
}

// ---- Public epd_driver.h API ------------------------------------------------

void epd_init(void)
{
	if (s_inited) return;

	gpio_config_t io = {
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = (1ULL << PIN_CKV) | (1ULL << PIN_LEH) | (1ULL << PIN_STV),
	};
	ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io));
	gpio_set_level(PIN_CKV, 0);
	gpio_set_level(PIN_LEH, 0);
	gpio_set_level(PIN_STV, 1);

	i2c_master_bus_config_t bus = {
		.i2c_port = -1,
		.sda_io_num = PIN_CFG_SDA,
		.scl_io_num = PIN_CFG_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &s_i2c));
	i2c_device_config_t pcacfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = PCA_ADDR, .scl_speed_hz = 100000 };
	ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c, &pcacfg, &s_pca));
	i2c_device_config_t tpscfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = TPS_ADDR, .scl_speed_hz = 100000 };
	ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c, &tpscfg, &s_tps));
	pca_set_config((uint8_t)(PCA_PWRGOOD | PCA_INT));  // those two are inputs
	s_pca_power = 0;
	pca_set_output(0);

	esp_lcd_i80_bus_config_t i80 = {
		.dc_gpio_num = PIN_DC,
		.wr_gpio_num = PIN_CKH,
		.clk_src = LCD_CLK_SRC_PLL160M,
		.data_gpio_nums = { PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7 },
		.bus_width = 8,
		.max_transfer_bytes = EPD_LINE_BYTES + EPD_LINE_PAD + 16,
		.dma_burst_size = 32,
	};
	ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&i80, &s_i80_bus));
	esp_lcd_panel_io_i80_config_t iocfg = {
		.cs_gpio_num = PIN_STH,
		.pclk_hz = EPD_XCK_HZ,
		.trans_queue_depth = 4,
		.on_color_trans_done = dma_done_cb,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
		.dc_levels = { .dc_idle_level = 0, .dc_cmd_level = 0, .dc_dummy_level = 0, .dc_data_level = 1 },
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(s_i80_bus, &iocfg, &s_panel_io));

	for (int i = 0; i < 2; i++)
		s_dma[i] = heap_caps_aligned_alloc(16, EPD_LINE_BYTES + EPD_LINE_PAD + 16, MALLOC_CAP_DMA);
	s_cur = 0;
	s_dma_done = true;
	// Per-pixel DC ledger (PSRAM). If this fails, field_scan_permitted locks
	// all drive rather than running unprotected.
	s_ledger = heap_caps_calloc((size_t)EPD_WIDTH * EPD_HEIGHT, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
	if (!s_ledger) ESP_LOGE(TAG, "safety: DC-ledger allocation FAILED — panel drive locked");
	s_inited = true;
	ESP_LOGI(TAG, "ED047TC1 i80/PCA9555/TPS65185 driver up: %dx%d pclk=%dMHz D0-7={%d,%d,%d,%d,%d,%d,%d,%d} CKH=%d CKV=%d STH=%d LEH=%d STV=%d I2C=%d/%d",
	         EPD_WIDTH, EPD_HEIGHT, EPD_XCK_HZ / 1000000,
	         PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7,
	         PIN_CKH, PIN_CKV, PIN_STH, PIN_LEH, PIN_STV, PIN_CFG_SDA, PIN_CFG_SCL);
}

static void epd_poweron_internal(void)
{
	if (!s_inited) return;
	s_pca_power = PCA_WAKEUP;                        pca_ctrl(true, false);
	vTaskDelay(1);
	// Rail power-up sequencing (epdiy tps65185.c values): UPSEQ0=0xE1 orders
	// the strobes, UPSEQ1=0xAA sets the inter-rail delays. We previously left
	// the chip's defaults; program the proven config before PWRUP triggers the
	// sequenced bring-up.
	reg_write(s_tps, TPS_REG_UPSEQ0, 0xE1);
	reg_write(s_tps, TPS_REG_UPSEQ1, 0xAA);
	s_pca_power = PCA_WAKEUP | PCA_PWRUP;            pca_ctrl(true, false);
	s_pca_power = PCA_WAKEUP | PCA_PWRUP | PCA_VCOM; pca_ctrl(true, false);
	vTaskDelay(1);

	int tries = 0;
	while (!(pca_read_input() & PCA_PWRGOOD)) {
		if (++tries >= 200) { ESP_LOGE(TAG, "PWRGOOD timeout"); break; }
		vTaskDelay(1);
	}
	reg_write(s_tps, TPS_REG_ENABLE, 0x3F);
	// VCOM interlock: only ever program the panel's OWN calibration (set via
	// gea_epd_set_vcom_mv). Unset -> chip default stays, and every scan is
	// NOP-suppressed by field_scan_permitted anyway.
	if (s_vcom_set) tps_set_vcom(s_vcom_mv);
	gpio_set_level(PIN_STH, 1);

	tries = 0;
	while ((reg_read(s_tps, TPS_REG_PG) & 0xFA) != 0xFA) {
		if (++tries >= 500) { ESP_LOGE(TAG, "TPS power-good failed PG=0x%02x", reg_read(s_tps, TPS_REG_PG)); break; }
		vTaskDelay(1);
	}
	// Enable source outputs + drive mode for the whole draw session.
	pca_ctrl(true, true);
}

void epd_poweron(void)
{
	epd_poweron_internal();
}

void epd_poweroff(void)
{
	if (!s_inited) return;
	s_pca_power = PCA_WAKEUP;
	pca_ctrl(false, false);
	vTaskDelay(1);
	s_pca_power = 0;
	pca_ctrl(false, false);
}

void epd_poweroff_all(void) { epd_poweroff(); }

void epd_clear(void)
{
	if (!s_inited) return;
	const int saved_hold = s_row_hold_us;
	// A clear must erase HISTORY, not merely push toward a color. Pixels
	// carrying residual charge from a high-contrast pattern land off-target
	// under a one-way push and the pattern shows through the solid. Full-range
	// alternation first — every pass flips EVERY pixel — bleeds the asymmetric
	// residue; then the sustained white push finishes on clean film (the
	// classic flashing clear of commercial readers).
	s_row_hold_us = 60;
	for (int pass = 0; pass < 3; pass++) {
		for (int i = 0; i < 3; i++) field_uniform_swept16(DRV_DARK);
		for (int i = 0; i < 3; i++) field_uniform_swept16(DRV_LIGHT);
	}
	s_row_hold_us = 40;
	for (int i = 0; i < 4; i++) field_uniform_swept16(DRV_DARK);
	for (int i = 0; i < 6; i++) field_uniform_swept16(DRV_LIGHT);
	s_row_hold_us = saved_hold;
}

// One field of a block-checkerboard at a given block size (pixels; use
// multiples of 4 so blocks land on byte boundaries). Visible to the eye,
// unlike a per-pixel checker which reads as uniform gray at this dpi, and
// coherent enough to exercise lateral charge differences across capsule
// clusters. Varying the block size between passes moves the lattice edges
// so no boundary line sits on the same pixels all session.
static void field_checker(int block_px, bool invert)
{
	const uint8_t dark_fill = (uint8_t)((DRV_DARK << 6) | (DRV_DARK << 4) | (DRV_DARK << 2) | DRV_DARK);
	const uint8_t light_fill = (uint8_t)((DRV_LIGHT << 6) | (DRV_LIGHT << 4) | (DRV_LIGHT << 2) | DRV_LIGHT);
	if (block_px < 4) block_px = 4;
	frame_start();
	for (int y = 0; y < EPD_HEIGHT; y++) {
		uint8_t *out = s_dma[s_cur];
		const int yblk = y / block_px;
		for (int xb = 0; xb < EPD_LINE_BYTES; xb++) {
			const int parity = ((((xb * 4) / block_px) + yblk) & 1) ^ (invert ? 1 : 0);
			out[xb] = parity ? dark_fill : light_fill;
		}
		memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
		send_row(out);
		s_cur ^= 1;
	}
	memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
	send_row(s_dma[s_cur]);
	s_cur ^= 1;
	while (!s_dma_done) {}
}

// Deep panel recovery: multipass full-quality refresh cycles to dissipate
// accumulated residual charge (the aftermath of DC-imbalanced driving — the
// inverted-polarity phase kept pushing the page area one way). Each cycle is
// four saturating phases: all-black, all-white, checkerboard, inverse
// checkerboard — the pattern phases exercise LATERAL charge differences that
// uniform flashing can't reach (same recipe as vendor screen-repair tools).
// Every phase drives both pixel classes equally, so each cycle is zero-mean.
// Caller must have powered the panel on.
// Strong-drive repair: every field runs with an extended row-select window
// (REPAIR_ROW_HOLD_US) so each pass transfers ~4x the charge of a bare scan,
// and each phase is followed by a settle pause — e-ink particles keep moving
// on the ~100ms timescale after the field ends, and drive+settle cycling
// shakes trapped charge loose better than continuous drive.
#define REPAIR_ROW_HOLD_US 120
#define REPAIR_SETTLE_MS   100

void gea_epd_repair(int cycles, int fields_per_phase)
{
	if (!s_inited || !experimental_ok("gea_epd_repair")) return;
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = REPAIR_ROW_HOLD_US;
	static const int kCheckerScales[3] = { 16, 32, 48 };
	for (int c = 0; c < cycles; c++) {
		// One checker scale per cycle (rotating across cycles so the lattice
		// walks), and the checker is IMMEDIATELY followed by its exact inverse
		// at the SAME scale: that pair flips every pixel and is zero-mean, so
		// the pattern phase leaves no per-pixel-class history behind. Entering
		// a solid straight from a pattern flips only HALF the pixels — the
		// half that saturates in place keeps its offset and the pattern shows
		// through the solid (seen on glass 2026-07-20) — so solids are only
		// entered after a shake equalizes both classes' recent history.
		const bool inv = (c & 1) != 0;
		const int scale = kCheckerScales[c % 3];
		shake(4, 20);
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_DARK);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		for (int i = 0; i < fields_per_phase; i++) field_checker(scale, inv);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		for (int i = 0; i < fields_per_phase; i++) field_checker(scale, !inv);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		shake(4, 20);
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		if ((c % 25) == 24) ESP_LOGI(TAG, "repair cycle %d/%d", c + 1, cycles);
	}
	// Rest white.
	for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
	s_row_hold_us = saved_hold;
}

// Solids-only deghosting wash for a PATTERN imprint (e.g. a checkerboard that
// shows through solid fills). Strong full-range black/white alternation with
// settle pauses: every transition flips EVERY pixel, so each cycle is exactly
// zero-mean per pixel and the imprint bleeds out — and unlike the checker-based
// repair there are no pattern phases that could re-imprint anything. Warming
// the panel WHILE this runs speeds recovery (particle mobility rises with
// temperature); heat without drive moves nothing.
void gea_epd_wash(int cycles, int fields_per_phase)
{
	if (!s_inited || !experimental_ok("gea_epd_wash")) return;
	const int saved_hold = s_row_hold_us;
	for (int c = 0; c < cycles; c++) {
		shake(6, 20);   // rapid activation flicker
		s_row_hold_us = REPAIR_ROW_HOLD_US;
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_DARK);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
		s_row_hold_us = saved_hold;
		if ((c % 25) == 24) ESP_LOGI(TAG, "wash cycle %d/%d", c + 1, cycles);
	}
	// Rest white.
	s_row_hold_us = REPAIR_ROW_HOLD_US;
	for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
	s_row_hold_us = saved_hold;
}

// FIRST-LIGHT ritual for a NEW panel: after gea_epd_set_vcom_mv() with THAT
// panel's own calibration, drive a gentle 4-field swept DARK band across the
// screen center, then power off and stop. The user must visually confirm the
// band DARKENED — if it lightened, the polarity is inverted and NOTHING else
// may run (the first panel took a whole session of inverted drive before the
// swap was caught). Sustained rendering stays locked until the board config
// asserts the ritual was confirmed.
void gea_epd_first_light_ritual(void)
{
	if (!s_inited) return;
	ESP_LOGW(TAG, "FIRST LIGHT: driving a gentle DARK band across the screen center...");
	epd_poweron_internal();
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = 80;
	for (int i = 0; i < 4; i++) field_uniform_swept_range(DRV_DARK, 80, 160);
	s_row_hold_us = saved_hold;
	epd_poweroff();
	ESP_LOGW(TAG, "FIRST LIGHT done. The center band must be DARKER than before.");
	ESP_LOGW(TAG, "If it got LIGHTER: polarity is inverted — STOP, fix DRV_DARK/DRV_LIGHT.");
	ESP_LOGW(TAG, "If darker: set kFirstLightConfirmed=true in display.cpp and reflash.");
}

// RECOVERY-ONLY program: runs forever, never returns. Alternating treatment
// and REST blocks:
//   - treatment: power on, activation shake + strong solid black/white
//     alternation (every transition flips every pixel — exactly zero-mean,
//     nothing patterned), end white;
//   - rest: power fully OFF (TPS rails + VCOM down, no bias across the film)
//     for minutes — trapped charge dissipates on the minutes timescale, and
//     resting under zero bias recovers more than continuous drive.
// Use by flashing a build whose display init calls this INSTEAD of starting
// the app: the panel gets treatment with zero app interference until a new
// firmware is flashed. Warming the panel during treatment blocks helps.
void gea_epd_recovery_forever(void)
{
	if (!s_inited || !experimental_ok("gea_epd_recovery_forever")) return;
	// Swept saturation costs MORE fields than raw drive: in a swept pass a
	// pixel is driven for one gate window, then its row is rewritten with NOP
	// on the next pass — unlike a raw solid where the latched drive holds
	// between scans. 6 swept fields at hold 80 read visibly-not-black on
	// glass (2026-07-20); 16 fields per direction at hold 150 triples the
	// integrated per-pixel drive. The swept form is the ONLY solid drive that
	// reaches the halo region (net-VCOM-current collapse under solid rows).
	const int fields_per_phase = 16;
	// The halo region (portrait bottom = physical row bytes [120,240) — the
	// far, high-droop end) reads brighter than the rest at equal drive, so it
	// gets extra balanced fields per cycle on top of the global fill.
	const int halo_extra_fields = 12;
	const int halo_byte_lo = 120, halo_byte_hi = 240;
	int round = 0;
	for (;;) {
		if (!experimental_ok("gea_epd_recovery_forever")) { epd_poweroff(); for (;;) vTaskDelay(pdMS_TO_TICKS(60000)); }
		round++;
		ESP_LOGI(TAG, "recovery round %d: treatment (~8 min, deep swept + halo top-up)...", round);
		epd_poweron();
		const int saved_hold = s_row_hold_us;
		for (int c = 0; c < 4; c++) {
			shake(2, 20);
			s_row_hold_us = 200;
			for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_DARK);
			for (int i = 0; i < halo_extra_fields; i++) field_uniform_swept_range(DRV_DARK, halo_byte_lo, halo_byte_hi);
			vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
			for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
			for (int i = 0; i < halo_extra_fields; i++) field_uniform_swept_range(DRV_LIGHT, halo_byte_lo, halo_byte_hi);
			vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
			s_row_hold_us = saved_hold;
		}
		// Rest white with rails fully off.
		s_row_hold_us = 80;
		for (int i = 0; i < fields_per_phase; i++) field_uniform_swept16(DRV_LIGHT);
		s_row_hold_us = saved_hold;
		epd_poweroff();
		ESP_LOGI(TAG, "recovery round %d: rest 3 min (power off)...", round);
		vTaskDelay(pdMS_TO_TICKS(180 * 1000));
	}
}

// Swept solid restricted to row bytes [byte_lo, byte_hi) — same 16-group
// current-limited sweep, but only within the region; everything outside gets
// NOP. Used to give the halo region (the far, high-droop end of the panel —
// it needs MORE integrated drive than the rest for the same optical result)
// extra balanced fields per treatment cycle. Callers MUST apply equal dark
// and light top-ups so the region stays zero-mean.
static void field_uniform_swept_range(uint8_t cmd, int byte_lo, int byte_hi)
{
	if (byte_lo < 0) byte_lo = 0;
	if (byte_hi > EPD_LINE_BYTES) byte_hi = EPD_LINE_BYTES;
	if (byte_lo >= byte_hi) return;
	for (int group = 0; group < 16; group++) {
		uint8_t tile[4] = { 0, 0, 0, 0 };
		tile[group >> 2] = (uint8_t)(cmd << ((group & 3) * 2));
		frame_start();
		for (int y = 0; y < EPD_HEIGHT; y++) {
			uint8_t *out = s_dma[s_cur];
			memset(out, 0, EPD_LINE_BYTES);
			for (int xb = byte_lo; xb < byte_hi; xb++) out[xb] = tile[xb & 3];
			memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
			send_row(out);
			s_cur ^= 1;
		}
		memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
		send_row(s_dma[s_cur]);
		s_cur ^= 1;
		while (!s_dma_done) {}
	}
}

// DIAGNOSTIC loop discriminating the two remaining bottom-halo theories
// (2026-07-20). Cycles three visually distinct phases, ~20s each, separated by
// ~3s of stillness, forever:
//   A: true checker alternation — every row carries BOTH polarities (net VCOM
//      return current ~0). If the halo FLICKERS here, the region receives
//      data and the failure is single-polarity drive (VCOM iR-droop): fix =
//      current-balanced waveforms.
//   B: split-masked solids (single polarity + NOP) — current recovery drive.
//   C: raw full solids — the original failing mode.
// If the halo stays inert in ALL phases including A, the region never gets
// source data at all (geometric/scan defect — e.g. trailing pad bytes
// clocking data through the source shift chain) and "checkers work" was the
// periodicity illusion (stale checker looks like checker).
void gea_epd_diagnostic_forever(void)
{
	if (!s_inited || !experimental_ok("gea_epd_diagnostic_forever")) return;
	int round = 0;
	for (;;) {
		if (!experimental_ok("gea_epd_diagnostic_forever")) { epd_poweroff(); for (;;) vTaskDelay(pdMS_TO_TICKS(60000)); }
		round++;
		epd_poweron();
		const int saved_hold = s_row_hold_us;
		s_row_hold_us = 80;
		ESP_LOGI(TAG, "diag round %d phase A: checker alternation 20s", round);
		for (int i = 0; i < 90; i++) {
			field_checker(8, 0);
			field_checker(8, 1);
		}
		s_row_hold_us = saved_hold;
		epd_poweroff();
		vTaskDelay(pdMS_TO_TICKS(3000));
		epd_poweron();
		s_row_hold_us = 80;
		ESP_LOGI(TAG, "diag round %d phase B: SWEPT-16 solids 30s", round);
		for (int i = 0; i < 16; i++) {
			field_uniform_swept16(DRV_DARK);
			field_uniform_swept16(DRV_LIGHT);
		}
		s_row_hold_us = saved_hold;
		epd_poweroff();
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}

// Region-targeted overdrive: extra black/white cycles applied only to row
// bytes [byte_lo, byte_hi); everything else gets NOP (hold). Concentrates
// treatment on a burned-in area without wearing the healthy half. Portrait
// coordinates map y -> physical x, so the portrait BOTTOM half is row bytes
// [120, 240).
void gea_epd_repair_region(int cycles, int fields_per_phase, int byte_lo, int byte_hi)
{
	if (!s_inited || !experimental_ok("gea_epd_repair_region")) return;
	if (byte_lo < 0) byte_lo = 0;
	if (byte_hi > EPD_LINE_BYTES) byte_hi = EPD_LINE_BYTES;
	if (byte_lo >= byte_hi) return;
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = REPAIR_ROW_HOLD_US;
	for (int c = 0; c < cycles; c++) {
		for (int phase = 0; phase < 2; phase++) {
			const uint8_t cmd = phase ? DRV_LIGHT : DRV_DARK;
			const uint8_t fill = (uint8_t)((cmd << 6) | (cmd << 4) | (cmd << 2) | cmd);
			for (int i = 0; i < fields_per_phase; i++) {
				frame_start();
				for (int y = 0; y < EPD_HEIGHT; y++) {
					uint8_t *out = s_dma[s_cur];
					memset(out, DRV_HOLD, EPD_LINE_BYTES);
					memset(out + byte_lo, fill, (size_t)(byte_hi - byte_lo));
					memset(out + EPD_LINE_BYTES, 0, EPD_LINE_PAD);
					send_row(out);
					s_cur ^= 1;
				}
				memset(s_dma[s_cur], 0, EPD_LINE_BYTES + EPD_LINE_PAD);
				send_row(s_dma[s_cur]);
				s_cur ^= 1;
				while (!s_dma_done) {}
			}
		}
		if ((c % 25) == 24) ESP_LOGI(TAG, "region repair cycle %d/%d", c + 1, cycles);
		vTaskDelay(pdMS_TO_TICKS(REPAIR_SETTLE_MS));
	}
	s_row_hold_us = saved_hold;
}

Rect_t epd_full_screen(void)
{
	Rect_t r = { .x = 0, .y = 0, .width = EPD_WIDTH, .height = EPD_HEIGHT };
	return r;
}

// STAGE 0: 1-bit black-on-white. `data` = panel-native 4bpp grayscale
// framebuffer (EPD_WIDTH/2 bytes/row, even x = low nibble). Darker-than-mid
// pixels are driven dark over EPD_IMAGE_FIELDS fields; the rest hold white.
void IRAM_ATTR epd_draw_grayscale_image(Rect_t area, uint8_t *data)
{
	if (!s_inited || !data) return;
	(void)area;
	// Quality draw: extend the row-select window so each field is a real
	// push (the bare ~21µs scan leaves text gray). ~40µs x 540 rows x 12
	// fields ≈ 0.4s per refresh — fine for page turns; the 60fps per-pixel
	// engine will run with hold=0 and many weak fields instead.
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = 40;
	for (int f = 0; f < EPD_IMAGE_FIELDS; f++)
		field_from_rows(data);
	s_row_hold_us = saved_hold;
}

void IRAM_ATTR epd_draw_image(Rect_t area, uint8_t *data, DrawMode_t mode)
{
	(void)mode;
	epd_draw_grayscale_image(area, data);
}

// Classic flashing FULL refresh, the "usual e-paper refresh" sequence:
// activation shake (rapid B/W flicker), NEGATIVE of the image, white reset,
// then the image. Clears ghosting far better than repeated target-driving
// because every pixel traverses its full range with alternation.
void gea_epd_draw_full_flash(uint8_t *data)
{
	if (!s_inited || !data) return;
	const int saved_hold = s_row_hold_us;
	s_row_hold_us = 40;
	shake(3, 20);
	for (int f = 0; f < 4; f++) field_from_rows_cmds(data, DRV_LIGHT, DRV_DARK);   // negative image
	for (int f = 0; f < 3; f++) field_uniform_swept16(DRV_LIGHT);                          // white reset
	for (int f = 0; f < EPD_IMAGE_FIELDS; f++) field_from_rows(data);              // final image
	s_row_hold_us = saved_hold;
}
