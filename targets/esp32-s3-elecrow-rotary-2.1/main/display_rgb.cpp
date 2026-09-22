#include "display.h"

#include "board.h"
#include "canvas.h"
#include "display_present.h"
#include "pcf8574.h"

#ifndef GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
// Strip-raster fused present: compose each horizontal strip in INTERNAL SRAM
// (fast, cached, no contention with the RGB panel's continuous PSRAM scanout),
// then bulk-copy the finished strip straight to the live scanout framebuffer.
// Replaces the slow "raster 1000 scattered circles directly into the PSRAM back
// buffer (75ms under scanout contention) + memcpy back->scanout (36ms)" path for
// full-screen-redraw frames. Still composes each strip fully off-screen, so it is
// tear-equivalent to the double buffer (never exposes clear-then-draw intermediates).
#define GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER 1
#endif

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/clk_tree_defs.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

// Second-core worker (defined later in this file). Used by the strip-raster path to
// copy a finished strip to the PSRAM scanout buffer on the other core while this
// core rasters the next strip into SRAM.
extern "C" bool gea_render_parallel_rows_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1);
extern "C" void gea_render_parallel_wait();

namespace {

constexpr const char *kTag = "elecrow_rgb";
constexpr int kWidth = gea::platform::display::kWidth;
constexpr int kHeight = gea::platform::display::kHeight;
constexpr std::size_t kPixelCount = static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight);
constexpr std::size_t kFramebufferBytes = kPixelCount * sizeof(std::uint16_t);
	constexpr int kRgbBounceRows = 20;
	constexpr std::size_t kRgbBounceBufferPixels = static_cast<std::size_t>(kWidth) * kRgbBounceRows;
	static_assert(kPixelCount % kRgbBounceBufferPixels == 0, "RGB bounce buffer must divide the framebuffer evenly");
constexpr ledc_mode_t kBacklightLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_channel_t kBacklightLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_t kBacklightLedcTimer = LEDC_TIMER_0;
constexpr ledc_timer_bit_t kBacklightLedcDutyResolution = LEDC_TIMER_10_BIT;
constexpr std::uint32_t kBacklightLedcMaxDuty = (1u << 10) - 1u;

namespace present = gea::framework::display_present;

// Internal-SRAM scratch strips that the fused present path composes into before
// copying to the PSRAM scanout buffer. Double-buffered: while core0 rasters strip
// N+1 into one half (SRAM), the worker core copies strip N from the other half to
// scanout (PSRAM) — different buffers, different buses, so they overlap with no
// contention. Two 24-row strips fit the same 46KB block one 48-row strip used.
constexpr int kStripRows = 24;
constexpr int kStripBufferRows = kStripRows * 2;
constexpr std::size_t kStripBufferPixels = static_cast<std::size_t>(kWidth) * kStripBufferRows;

gea::framework::graphics::Canvas g_canvas;
// Band canvas for the renderer's coarse 2-core replay split: bound to the SAME
// back buffer as g_canvas but with a private clip stack + dirty accumulator, so
// the worker core's band replay never races the main core on clip/dirty state.
// g_renderCoreId is the core the renderer submits from (recorded per
// gea_render_parallel_submit); Display draw calls on any OTHER core are the
// worker's band replay and route here via replayCanvas().
gea::framework::graphics::Canvas g_workerCanvas;
int g_renderCoreId = -1;

bool onWorkerCore()
{
	return g_renderCoreId >= 0 && static_cast<int>(xPortGetCoreID()) != g_renderCoreId;
}

gea::framework::graphics::Canvas &replayCanvas()
{
	return onWorkerCore() ? g_workerCanvas : g_canvas;
}
esp_lcd_panel_handle_t g_panel = nullptr;
std::uint16_t *g_framebuffer = nullptr;
std::uint16_t *g_drawFramebuffer = nullptr;
std::uint16_t *g_stripBuffer = nullptr;
	// The strip path publishes straight to scanout and leaves g_drawFramebuffer
	// untouched. If a later frame falls back to the per-command path (partial
	// present with no full-screen base), re-sync the back buffer once first.
	bool g_drawBufferStale = false;
	bool g_initialized = false;
	bool g_backlightPwmInitialized = false;
	int g_brightness = 100;
	int g_flushRows = kHeight;
	int g_flushDepth = 1;
	gea::platform::display::DisplayFlushPerfStats g_stats = {};

	bool clipRect(int *x0, int *y0, int *x1, int *y1);

struct InitCommand {
	std::uint8_t command;
	std::uint8_t data[16];
	std::uint8_t length;
	std::uint16_t delayMs;
};

constexpr InitCommand kSt7701Init[] = {
	{0x01, {}, 0, 120},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
	{0xc0, {0x3b, 0x00}, 2, 0},
	{0xc1, {0x0b, 0x02}, 2, 0},
	{0xc2, {0x00, 0x02}, 2, 0},
	{0xcc, {0x10}, 1, 0},
	{0xcd, {0x08}, 1, 0},
	{0xb0, {0x02, 0x13, 0x1b, 0x0d, 0x10, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0e, 0x2c, 0x33, 0x1d}, 16, 0},
	{0xb1, {0x05, 0x13, 0x1b, 0x0d, 0x11, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0e, 0x2c, 0x33, 0x1d}, 16, 0},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
	{0xb0, {0x5d}, 1, 0},
	{0xb1, {0x43}, 1, 0},
	{0xb2, {0x81}, 1, 0},
	{0xb3, {0x80}, 1, 0},
	{0xb5, {0x43}, 1, 0},
	{0xb7, {0x85}, 1, 0},
	{0xb8, {0x20}, 1, 0},
	{0xc1, {0x78}, 1, 0},
	{0xc2, {0x78}, 1, 0},
	{0xd0, {0x88}, 1, 0},
	{0xe0, {0x00, 0x00, 0x02}, 3, 0},
	{0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 11, 0},
	{0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 13, 0},
	{0xe3, {0x00, 0x00, 0x11, 0x00}, 4, 0},
	{0xe4, {0x22, 0x00}, 2, 0},
	{0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
	{0xe6, {0x00, 0x00, 0x11, 0x00}, 4, 0},
	{0xe7, {0x22, 0x00}, 2, 0},
	{0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
	{0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 7, 0},
	{0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 16, 0},
	{0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 6, 0},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
	{0xef, {0x08}, 1, 0},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
	{0x36, {0x00}, 1, 0},
	{0x3a, {0x60}, 1, 0},
	{0x11, {}, 0, 120},
	{0x29, {}, 0, 50},
	{0x20, {}, 0, 0},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
	{0xc7, {0x00}, 1, 0},
	{0xff, {0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
	{0x36, {0x00}, 1, 0},
};

	void cacheWritebackAll()
	{
		if (!g_framebuffer) return;
		esp_cache_msync(g_framebuffer,
		                kFramebufferBytes,
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
	}

	void cacheWritebackRect(int x0, int y0, int x1, int y1)
	{
		if (!g_framebuffer || !clipRect(&x0, &y0, &x1, &y1)) return;
		const std::size_t begin = static_cast<std::size_t>(y0) * kWidth + static_cast<std::size_t>(x0);
		const std::size_t end = static_cast<std::size_t>(y1) * kWidth + static_cast<std::size_t>(x1) + 1;
		if (end <= begin || end > kPixelCount) return;
		esp_cache_msync(&g_framebuffer[begin],
		                (end - begin) * sizeof(std::uint16_t),
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
	}

void addFlushStats(int64_t totalUs, int pixelCount)
{
	g_stats.totalUs += totalUs;
	g_stats.callCount += 1;
	g_stats.pixelCount += pixelCount;
}

void addFlushStageStats(int64_t copyUs, int64_t cacheUs)
{
	g_stats.copyUs += copyUs;
	g_stats.completeWaitUs += cacheUs;
}

void configureGpioOutput(gpio_num_t pin, int level)
{
	gpio_reset_pin(pin);
	gpio_config_t config = {};
	config.pin_bit_mask = 1ULL << pin;
	config.mode = GPIO_MODE_OUTPUT;
	config.pull_up_en = GPIO_PULLUP_DISABLE;
	config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	config.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&config);
	gpio_set_level(pin, level);
}

void configureControlPins()
{
	const auto &display = gea::platform::board::display;
	configureGpioOutput(display.spiCs, 1);
	configureGpioOutput(display.spiSck, 0);
	configureGpioOutput(display.spiSda, 0);
}

void writeControlBit(bool value)
{
	const auto &display = gea::platform::board::display;
	gpio_set_level(display.spiSda, value ? 1 : 0);
	gpio_set_level(display.spiSck, 1);
	esp_rom_delay_us(1);
	gpio_set_level(display.spiSck, 0);
	esp_rom_delay_us(1);
}

void writeControlWord(bool dataWord, std::uint8_t value)
{
	writeControlBit(dataWord);
	for (int bit = 7; bit >= 0; --bit) {
		writeControlBit((value >> bit) & 1);
	}
}

void sendInitCommandsBatched()
{
	const auto &display = gea::platform::board::display;
	gpio_set_level(display.spiCs, 0);
	bool selected = true;
	for (const auto &cmd : kSt7701Init) {
		if (!selected) {
			gpio_set_level(display.spiCs, 0);
			selected = true;
		}
		writeControlWord(false, cmd.command);
		for (int i = 0; i < cmd.length; ++i) {
			writeControlWord(true, cmd.data[i]);
		}
		if (cmd.delayMs > 0) {
			gpio_set_level(display.spiCs, 1);
			selected = false;
			vTaskDelay(pdMS_TO_TICKS(cmd.delayMs));
		}
	}
	if (selected) gpio_set_level(display.spiCs, 1);
	esp_rom_delay_us(5);
}

esp_err_t powerAndResetPanel()
{
	esp_err_t err = gea::platform::elecrow::Pcf8574::init();
	if (err != ESP_OK) return err;

	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.lcdPowerBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(100));

	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.lcdResetBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(100));
	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.lcdResetBit, false);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));
	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.lcdResetBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));

	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.touchResetBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(100));
	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.touchResetBit, false);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));
	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.touchResetBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));
	err = gea::platform::elecrow::Pcf8574::writePin(gea::platform::board::expander.touchInterruptBit, true);
	if (err != ESP_OK) return err;
	vTaskDelay(pdMS_TO_TICKS(120));
	return ESP_OK;
}

esp_err_t initSt7701()
{
	configureControlPins();
	esp_err_t err = powerAndResetPanel();
	if (err != ESP_OK) return err;

	sendInitCommandsBatched();
	return ESP_OK;
}

esp_err_t updateBacklightDuty()
{
	if (!g_backlightPwmInitialized) return ESP_OK;
	const std::uint32_t duty = (kBacklightLedcMaxDuty * static_cast<std::uint32_t>(g_brightness)) / 100u;
	esp_err_t err = ledc_set_duty(kBacklightLedcMode, kBacklightLedcChannel, duty);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to set backlight duty: %s", esp_err_to_name(err));
		return err;
	}
	err = ledc_update_duty(kBacklightLedcMode, kBacklightLedcChannel);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to update backlight duty: %s", esp_err_to_name(err));
		return err;
	}
	return ESP_OK;
}

esp_err_t initBacklight()
{
	ledc_timer_config_t timerConfig = {};
	timerConfig.speed_mode = kBacklightLedcMode;
	timerConfig.timer_num = kBacklightLedcTimer;
	timerConfig.duty_resolution = kBacklightLedcDutyResolution;
	timerConfig.freq_hz = 5000;
	timerConfig.clk_cfg = LEDC_AUTO_CLK;
	esp_err_t err = ledc_timer_config(&timerConfig);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to configure backlight LEDC timer: %s", esp_err_to_name(err));
		return err;
	}

	ledc_channel_config_t channelConfig = {};
	channelConfig.gpio_num = gea::platform::board::display.backlight;
	channelConfig.speed_mode = kBacklightLedcMode;
	channelConfig.channel = kBacklightLedcChannel;
	channelConfig.timer_sel = kBacklightLedcTimer;
	channelConfig.duty = kBacklightLedcMaxDuty;
	channelConfig.hpoint = 0;
	err = ledc_channel_config(&channelConfig);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to configure backlight LEDC channel: %s", esp_err_to_name(err));
		return err;
	}

	g_backlightPwmInitialized = true;
	return updateBacklightDuty();
}

esp_err_t initRgbPanel()
{
	const auto &display = gea::platform::board::display;
	esp_lcd_rgb_panel_config_t config = {};
	config.clk_src = LCD_CLK_SRC_PLL160M;
	config.timings.pclk_hz = display.pclkHz;
	config.timings.h_res = kWidth;
	config.timings.v_res = kHeight;
	config.timings.hsync_pulse_width = display.hsyncPulseWidth;
	config.timings.hsync_back_porch = display.hsyncBackPorch;
	config.timings.hsync_front_porch = display.hsyncFrontPorch;
	config.timings.vsync_pulse_width = display.vsyncPulseWidth;
	config.timings.vsync_back_porch = display.vsyncBackPorch;
	config.timings.vsync_front_porch = display.vsyncFrontPorch;
	config.timings.flags.pclk_active_neg = display.pclkActiveNeg ? 1 : 0;
	config.data_width = 16;
	config.in_color_format = LCD_COLOR_FMT_RGB565;
	config.out_color_format = LCD_COLOR_FMT_RGB565;
	config.num_fbs = 1;
	// Bounce buffers vs direct PSRAM scanout: with bounce, a core-0 ISR memcpys
	// every scanline batch from PSRAM into internal SRAM (~27.6MB/s at 12MHz pclk)
	// — a standing ~30% CPU tax on the render core. With 0 the GDMA reads the
	// PSRAM framebuffer directly (no CPU in the scanout path); risk is scanout
	// underrun (visible drift) when CPU PSRAM traffic starves the DMA.
#ifndef GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
#define GEA_EMBEDDED_DISPLAY_RGB_BOUNCE 1
#endif
#if GEA_EMBEDDED_DISPLAY_RGB_BOUNCE
	config.bounce_buffer_size_px = kRgbBounceBufferPixels;
#else
	config.bounce_buffer_size_px = 0;
#endif
	config.dma_burst_size = 64;
	config.hsync_gpio_num = display.hsync;
	config.vsync_gpio_num = display.vsync;
	config.de_gpio_num = display.de;
	config.pclk_gpio_num = display.pclk;
	config.disp_gpio_num = GPIO_NUM_NC;
	config.data_gpio_nums[0] = display.b0;
	config.data_gpio_nums[1] = display.b1;
	config.data_gpio_nums[2] = display.b2;
	config.data_gpio_nums[3] = display.b3;
	config.data_gpio_nums[4] = display.b4;
	config.data_gpio_nums[5] = display.g0;
	config.data_gpio_nums[6] = display.g1;
	config.data_gpio_nums[7] = display.g2;
	config.data_gpio_nums[8] = display.g3;
	config.data_gpio_nums[9] = display.g4;
	config.data_gpio_nums[10] = display.g5;
	config.data_gpio_nums[11] = display.r0;
	config.data_gpio_nums[12] = display.r1;
	config.data_gpio_nums[13] = display.r2;
	config.data_gpio_nums[14] = display.r3;
	config.data_gpio_nums[15] = display.r4;
	config.flags.fb_in_psram = 1;

	esp_err_t err = esp_lcd_new_rgb_panel(&config, &g_panel);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to create RGB panel: %s", esp_err_to_name(err));
		return err;
	}
	err = esp_lcd_panel_reset(g_panel);
	if (err != ESP_OK) return err;
	err = esp_lcd_panel_init(g_panel);
	if (err != ESP_OK) return err;

	void *fb = nullptr;
	err = esp_lcd_rgb_panel_get_frame_buffer(g_panel, 1, &fb);
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "failed to get RGB framebuffer: %s", esp_err_to_name(err));
		return err;
	}
	g_framebuffer = static_cast<std::uint16_t *>(fb);
	if (!g_framebuffer) return ESP_ERR_NO_MEM;

	// Double buffer: the panel framebuffer (g_framebuffer) is continuously scanned out,
	// so drawing directly into it exposes every intermediate step (e.g. a tick's
	// background clears, THEN the tick paints — a visible flash). Render into a separate
	// PSRAM back buffer instead; publishDrawRect() copies the fully-composited dirty
	// region into the scanout framebuffer in one shot, so the panel only ever sees
	// finished pixels. Falls back to single-buffered direct draw if PSRAM is exhausted.
	g_drawFramebuffer = static_cast<std::uint16_t *>(
	    heap_caps_malloc(kFramebufferBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!g_drawFramebuffer) {
		ESP_LOGW(kTag, "no PSRAM for back buffer; falling back to single-buffered draw");
		g_drawFramebuffer = g_framebuffer;
	}

	std::memset(g_framebuffer, 0, kFramebufferBytes);
	if (g_drawFramebuffer != g_framebuffer) std::memset(g_drawFramebuffer, 0, kFramebufferBytes);
		cacheWritebackAll();
	g_canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	g_workerCanvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);

#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
	// Internal SRAM (not PSRAM): the whole point is to raster off the scanout bus.
	// 16-byte aligned: rows are 960B (16-multiple), so an aligned base keeps every
	// staged-replay row 16-aligned — maximal PIE-SIMD span coverage per scanline.
	g_stripBuffer = static_cast<std::uint16_t *>(
	    heap_caps_aligned_alloc(16, kStripBufferPixels * sizeof(std::uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
	ESP_LOGI(kTag, "strip-raster %s strip_buf=%p rows=%d",
	         g_stripBuffer ? "ON" : "OFF (no internal SRAM)", g_stripBuffer, kStripRows);
#endif
	ESP_LOGI(kTag, "RGB panel ready %dx%d pclk=%d bounce_rows=%d fb=%p draw_fb=%p single_fb=%d",
	         kWidth,
	         kHeight,
	         display.pclkHz,
	         kRgbBounceRows,
	         g_framebuffer,
	         g_drawFramebuffer,
	         g_framebuffer == g_drawFramebuffer ? 1 : 0);
	return ESP_OK;
}

void replayPresentCommand(const gea::platform::display::DisplayPresentCommand &command)
{
	using Type = gea::platform::display::DisplayPresentCommandType;
	switch (command.type) {
		case Type::Clear:
			g_canvas.clear(command.clear.color);
			break;
		case Type::FillRectRgb565:
			g_canvas.setGlobalAlpha(command.fillRectRgb565.alpha);
			g_canvas.fillRect(command.fillRectRgb565.x,
			                  command.fillRectRgb565.y,
			                  command.fillRectRgb565.w,
			                  command.fillRectRgb565.h,
			                  command.fillRectRgb565.color);
			break;
		case Type::StrokeRectRgb565:
			g_canvas.setGlobalAlpha(command.strokeRectRgb565.alpha);
			g_canvas.strokeRect(command.strokeRectRgb565.x,
			                    command.strokeRectRgb565.y,
			                    command.strokeRectRgb565.w,
			                    command.strokeRectRgb565.h,
			                    command.strokeRectRgb565.color);
			break;
		case Type::FillTriangleRgb565:
			g_canvas.setGlobalAlpha(command.fillTriangleRgb565.alpha);
			g_canvas.fillTriangle(command.fillTriangleRgb565.x0,
			                      command.fillTriangleRgb565.y0,
			                      command.fillTriangleRgb565.x1,
			                      command.fillTriangleRgb565.y1,
			                      command.fillTriangleRgb565.x2,
			                      command.fillTriangleRgb565.y2,
			                      command.fillTriangleRgb565.color);
			break;
		case Type::FillTrianglesRgb565: {
			// Depth-ordered batch (farthest-first). Opaque batches take the span-occlusion
			// path so each pixel is written once; translucent ones must stay painter's order.
			const auto &batch = command.fillTrianglesRgb565;
			if (!batch.entries) break;
			g_canvas.setGlobalAlpha(batch.alpha);
			if (batch.alpha == 255) {
				g_canvas.fillTrianglesOpaqueOccluded(batch.entries, batch.count, 0, 0);
			} else {
				for (int i = 0; i < batch.count; i++) {
					const auto &t = batch.entries[i];
					g_canvas.fillTriangle(t.x0, t.y0, t.x1, t.y1, t.x2, t.y2, t.color);
				}
			}
			break;
		}
		case Type::FillCircleRgb565:
			g_canvas.setGlobalAlpha(command.fillCircleRgb565.alpha);
			g_canvas.fillCircle(command.fillCircleRgb565.x,
			                    command.fillCircleRgb565.y,
			                    command.fillCircleRgb565.radius,
			                    command.fillCircleRgb565.color);
			break;
		case Type::StrokeCircleRgb565:
			g_canvas.setGlobalAlpha(command.strokeCircleRgb565.alpha);
			g_canvas.strokeCircle(command.strokeCircleRgb565.x,
			                      command.strokeCircleRgb565.y,
			                      command.strokeCircleRgb565.radius,
			                      command.strokeCircleRgb565.color);
			break;
			case Type::FillCirclesRgb565:
				g_canvas.setGlobalAlpha(command.fillCirclesRgb565.alpha);
				g_canvas.fillCirclesRgb565(command.fillCirclesRgb565.xs,
				                           command.fillCirclesRgb565.ys,
				                           command.fillCirclesRgb565.count,
				                           command.fillCirclesRgb565.radius,
				                           command.fillCirclesRgb565.colors);
				break;
			case Type::DrawImage:
				g_canvas.setGlobalAlpha(command.drawImage.alpha);
			g_canvas.drawImage(command.drawImage.pixels,
			                   command.drawImage.alphaPixels,
			                   command.drawImage.srcWidth,
			                   command.drawImage.srcHeight,
			                   command.drawImage.x,
			                   command.drawImage.y);
			break;
		case Type::DrawImageScaled:
			g_canvas.setGlobalAlpha(command.drawImageScaled.alpha);
			g_canvas.drawImage(command.drawImageScaled.pixels,
			                   command.drawImageScaled.alphaPixels,
			                   command.drawImageScaled.srcWidth,
			                   command.drawImageScaled.srcHeight,
			                   command.drawImageScaled.x,
			                   command.drawImageScaled.y,
			                   command.drawImageScaled.w,
			                   command.drawImageScaled.h);
			break;
		case Type::DrawImageRotated90CW:
			g_canvas.setGlobalAlpha(command.drawImageRotated90CW.alpha);
			g_canvas.drawImageRotated90CW(command.drawImageRotated90CW.pixels,
			                              command.drawImageRotated90CW.alphaPixels,
			                              command.drawImageRotated90CW.srcWidth,
			                              command.drawImageRotated90CW.srcHeight,
			                              command.drawImageRotated90CW.x,
			                              command.drawImageRotated90CW.y,
			                              command.drawImageRotated90CW.w,
			                              command.drawImageRotated90CW.h);
			break;
		case Type::DrawImageTiledX:
			g_canvas.setGlobalAlpha(command.drawImageTiledX.alpha);
			g_canvas.drawImageTiledX(command.drawImageTiledX.pixels,
			                         command.drawImageTiledX.alphaPixels,
			                         command.drawImageTiledX.srcWidth,
			                         command.drawImageTiledX.srcHeight,
			                         command.drawImageTiledX.x,
			                         command.drawImageTiledX.y,
			                         command.drawImageTiledX.w);
			break;
		case Type::FillText:
			g_canvas.setGlobalAlpha(command.fillText.alpha);
			g_canvas.drawText(command.fillText.text,
			                  command.fillText.x,
			                  command.fillText.y,
			                  command.fillText.color,
			                  command.fillText.scale);
			break;
	}
}

bool clipRect(int *x0, int *y0, int *x1, int *y1)
{
	if (!x0 || !y0 || !x1 || !y1) return false;
	if (*x0 < 0) *x0 = 0;
	if (*y0 < 0) *y0 = 0;
	if (*x1 >= kWidth) *x1 = kWidth - 1;
	if (*y1 >= kHeight) *y1 = kHeight - 1;
	return *x0 <= *x1 && *y0 <= *y1;
}

	int publishDrawRect(int x0, int y0, int x1, int y1, int *outX0 = nullptr, int *outY0 = nullptr, int *outX1 = nullptr, int *outY1 = nullptr)
	{
		if (!g_framebuffer || !g_drawFramebuffer || !clipRect(&x0, &y0, &x1, &y1)) return 0;
		if (outX0) *outX0 = x0;
		if (outY0) *outY0 = y0;
		if (outX1) *outX1 = x1;
		if (outY1) *outY1 = y1;
		const int width = x1 - x0 + 1;
		if (g_framebuffer == g_drawFramebuffer) return width * (y1 - y0 + 1);
		for (int y = y0; y <= y1; ++y) {
		const std::size_t offset = static_cast<std::size_t>(y) * kWidth + static_cast<std::size_t>(x0);
		std::memcpy(&g_framebuffer[offset], &g_drawFramebuffer[offset], static_cast<std::size_t>(width) * sizeof(std::uint16_t));
	}
	return width * (y1 - y0 + 1);
}

struct RenderRowWorker {
	TaskHandle_t task = nullptr;
	void (*volatile fn)(void *, int, int) = nullptr;
	void *volatile ctx = nullptr;
	volatile int y0 = 0;
	volatile int y1 = 0;
	std::atomic<std::uint32_t> jobSeq{0};
	std::atomic<bool> done{true};
	// DIAG: cumulative on-worker band execution time (read by the wait-side print;
	// only ever written between done=false and done=true, so the read is stable).
	std::int64_t bandUs = 0;
	bool attempted = false;
};

RenderRowWorker g_renderRowWorker;
// 12KB (was 8KB for the memcpy-only strip copies): the worker now also runs the
// renderer's full band replay (gradient projection math + rounded-rect batch +
// command handlers), which is much deeper than a strip copy.
constexpr std::uint32_t kRenderRowWorkerStackBytes = 12288;

void renderRowWorkerTask(void *)
{
	// MUST start at 0 (jobSeq's initial value), NOT jobSeq.load(): submit() creates
	// this task and increments jobSeq immediately after, so by the time this task
	// gets its first instructions in, jobSeq is usually already 1 — loading it here
	// would swallow job 1 (the caller times out and the first parallel band is
	// silently dropped; for the in-place recolor that band then stays stale forever,
	// since later recolors only match the CURRENT oldColor).
	std::uint32_t seen = 0;
	std::int64_t idleSince = esp_timer_get_time();
	for (;;) {
		while (g_renderRowWorker.jobSeq.load(std::memory_order_acquire) == seen) {
			if (esp_timer_get_time() - idleSince > 4000) {
				ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
				idleSince = esp_timer_get_time();
			}
		}
		seen = g_renderRowWorker.jobSeq.load(std::memory_order_acquire);
		void (*fn)(void *, int, int) = g_renderRowWorker.fn;
		const std::int64_t b0 = esp_timer_get_time();
		if (fn) fn(g_renderRowWorker.ctx, g_renderRowWorker.y0, g_renderRowWorker.y1);
		g_renderRowWorker.bandUs += esp_timer_get_time() - b0;
		g_renderRowWorker.done.store(true, std::memory_order_release);
		idleSince = esp_timer_get_time();
	}
}

#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
// Runs on the worker core: copy a finished SRAM strip (ctx) to PSRAM scanout rows
// [y0,y1] and write it back so the panel's scanout DMA sees it.
void copyStripToScanoutThunk(void *ctx, int y0, int y1)
{
	const std::uint16_t *src = static_cast<const std::uint16_t *>(ctx);
	const int rows = y1 - y0 + 1;
	std::memcpy(&g_framebuffer[static_cast<std::size_t>(y0) * kWidth],
	            src,
	            static_cast<std::size_t>(rows) * kWidth * sizeof(std::uint16_t));
	cacheWritebackRect(0, y0, kWidth - 1, y1);
}

// Compose the whole frame strip-by-strip in internal SRAM, copying each finished
// strip to the live scanout buffer. Used only for full-screen-redraw frames
// (frameHasOpaqueBase), so each strip is fully composited before it is published —
// tear-equivalent to the PSRAM double buffer, but the 1000-circle raster runs in
// SRAM instead of fighting the panel's continuous scanout for PSRAM bandwidth.
//
// Pipelined across cores: this core rasters strip N+1 into one SRAM half while the
// worker core copies strip N out of the other half to scanout. Raster (SRAM-only)
// and copy (PSRAM) hit different buses, so the ~6.5ms raster hides under the ~20ms
// copy instead of adding to it.
bool presentStripFused(const present::Frame &frame)
{
	if (!g_framebuffer || !g_stripBuffer) return false;
	int stripRows = g_flushRows > 0 ? g_flushRows : kStripRows;
	if (stripRows > kStripRows) stripRows = kStripRows;
	const std::int64_t start = esp_timer_get_time();
	bool inFlight = false;
	int idx = 0;
	for (int row = 0; row < kHeight; row += stripRows, ++idx) {
		int rows = stripRows;
		if (row + rows > kHeight) rows = kHeight - row;
		std::uint16_t *buf = g_stripBuffer + static_cast<std::size_t>(idx & 1) * kStripRows * kWidth;
		// Raster strip `idx` into its SRAM half — overlaps the worker copying the
		// previous strip out of the OTHER half (different buffer + bus, no contention).
		present::rasterFrameRowsStrided(buf, kWidth, 0, kWidth, row, rows, kHeight, frame);
		// The single worker handles one copy at a time. Waiting here also guarantees
		// the half we'll reuse two strips from now is free before we raster into it.
		if (inFlight) { gea_render_parallel_wait(); inFlight = false; }
		if (gea_render_parallel_rows_submit(copyStripToScanoutThunk, buf, row, row + rows - 1)) {
			inFlight = true;
		} else {
			copyStripToScanoutThunk(buf, row, row + rows - 1);  // no worker: inline copy
		}
	}
	if (inFlight) gea_render_parallel_wait();
	g_drawBufferStale = (g_drawFramebuffer != g_framebuffer);
	addFlushStats(esp_timer_get_time() - start, kWidth * kHeight);
	return true;
}
#endif

	}  // namespace

namespace {

// DIAG: band-replay split engagement — submit count + the main core's join-wait
// time, printed every 300 draw-band jobs (mirrors the esp32 backend's [worker]
// line). Tells "split never engages" apart from "split engages but doesn't help".
int g_drawBandJobs = 0;
std::int64_t g_drawBandWaitUs = 0;
bool g_lastSubmitWasDrawBand = false;

// Shared single-job submit behind both worker entry points: the strip-copy path
// (gea_render_parallel_rows_submit) and the renderer's coarse band replay
// (gea_render_parallel_submit). One job in flight at a time — both producers run
// on the frame task and join via gea_render_parallel_wait before resubmitting.
bool submitRenderWorkerJob(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	if (!g_renderRowWorker.attempted) {
		g_renderRowWorker.attempted = true;
		const BaseType_t renderCore = xPortGetCoreID();
		const BaseType_t workerCore = renderCore == 0 ? 1 : 0;
		const UBaseType_t priority = uxTaskPriorityGet(nullptr);
		const BaseType_t created = xTaskCreatePinnedToCore(renderRowWorkerTask,
		                                                   "gea_rowwrk",
		                                                   kRenderRowWorkerStackBytes,
		                                                   nullptr,
		                                                   priority,
		                                                   &g_renderRowWorker.task,
		                                                   workerCore);
		if (created != pdPASS || !g_renderRowWorker.task) {
			ESP_LOGW(kTag, "row worker create failed: create=%d stack=%u",
			         static_cast<int>(created),
			         static_cast<unsigned>(kRenderRowWorkerStackBytes));
		} else {
			std::printf("[render-worker] task=%p renderCore=%d workerCore=%d prio=%u stack=%u\n",
			            static_cast<void *>(g_renderRowWorker.task), static_cast<int>(renderCore),
			            static_cast<int>(workerCore), static_cast<unsigned>(priority),
			            static_cast<unsigned>(kRenderRowWorkerStackBytes));
		}
	}
	if (!g_renderRowWorker.task) return false;
	g_renderRowWorker.fn = fn;
	g_renderRowWorker.ctx = ctx;
	g_renderRowWorker.y0 = y0;
	g_renderRowWorker.y1 = y1;
	g_renderRowWorker.done.store(false, std::memory_order_release);
	g_renderRowWorker.jobSeq.fetch_add(1, std::memory_order_release);
	xTaskNotifyGive(g_renderRowWorker.task);
	return true;
}

}  // namespace

extern "C" bool gea_render_parallel_rows_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	g_lastSubmitWasDrawBand = false;
	return submitRenderWorkerJob(fn, ctx, y0, y1);
}

// NOTE: a 410-permille biased split (worker takes more rows; both bands finish
// together) was measured and REVERTED: with the cores fully overlapped the pair
// is bus-limited, so rebalancing rows doesn't shorten the wall — it only deepens
// the PSRAM saturation window, and the bounce-fill ISR (which keeps the panel
// synced) starts missing its deadline → dropped/restarted frames. Even split
// (the weak default 500) leaves the main core a solo tail that the ISR rides.

extern "C" bool gea_render_parallel_submit(void (*fn)(void *, int, int), void *ctx, int y0, int y1)
{
	// Record the render core so Display draw calls on the OTHER core (the worker's
	// band replay) route to the band canvas. Re-set each submit in case the frame
	// task ever migrates cores.
	g_renderCoreId = static_cast<int>(xPortGetCoreID());
	const bool ok = submitRenderWorkerJob(fn, ctx, y0, y1);
	g_lastSubmitWasDrawBand = ok;
	return ok;
}

// After a coarse band-replay joins, fold the worker canvas's dirty bbox into the
// primary so the publish copy (which reads g_canvas.dirty()) transmits both bands.
extern "C" void gea_render_parallel_merge_dirty()
{
	int x0, y0, x1, y1;
	if (g_workerCanvas.dirty(&x0, &y0, &x1, &y1)) g_canvas.markDirty(x0, y0, x1, y1);
	g_workerCanvas.resetDirty();
}

namespace {

// SRAM-staged dirty-region replay (gea_replay_stage_* hooks): the region replay
// rasters into a per-core half of g_stripBuffer instead of read-modify-writing the
// PSRAM back buffer under the RGB panel's continuous scanout. Measured before this:
// a HALF-band replay cost the same ~20ms as the full region — the PSRAM bus, not
// the CPU, was the limit, so the 2-core split alone bought nothing. Per-core halves:
// with the band split each core stages its own (disjoint) band rows, so the halves
// never overlap; presentStripFused also uses both halves but only on canvas-present
// frames, never concurrently with a region replay.
struct ReplayStageState {
	bool active = false;
	int x0 = 0, y0 = 0, x1 = -1, y1 = -1;  // staged window (screen coords)
	bool hadDirty = false;                 // canvas dirty union accumulated before the
	int dx0 = 0, dy0 = 0, dx1 = -1, dy1 = -1;  // rebind (bindPixels resets it)
	std::uint8_t alpha = 255;
	// DIAG: per-strip cost decomposition (seed copy-in / replay body / copy-back),
	// printed every kStageDiagStrips strips per core.
	std::int64_t beginDoneUs = 0;
	std::int64_t seedUs = 0, bodyUs = 0, cbUs = 0;
	int strips = 0, px = 0;
};
ReplayStageState g_replayStage[2];
constexpr int kStageDiagStrips = 600;
// DIAG: PIE SIMD blend engagement (calls + 8-px groups), printed with [stage].
volatile int g_pieCalls = 0;
volatile int g_pieCount8 = 0;

}  // namespace

// Staged-replay gate. Measured on css-3d-cube: staging the replay through SRAM
// strips LOST ~6fps — the per-strip re-replay multiplies commands whose raster
// cost is NOT row-proportional (the transformed face-label text re-rasters its
// full quad in every overlapping 24-row strip; rUs text went to ~10ms/frame),
// and the per-pixel blend turned out compute-bound, not PSRAM-bound, so the
// SRAM dst saved nothing. Keep the machinery (hooks + per-core windows) gated
// off for A/B; flip to 1 to re-measure.
#ifndef GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE
#define GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE 0
#endif

extern "C" int gea_replay_stage_rows()
{
#if GEA_EMBEDDED_DISPLAY_REPLAY_SRAM_STAGE
	return g_stripBuffer ? kStripRows : 0;
#else
	return 0;
#endif
}

extern "C" bool gea_replay_stage_begin(int x0, int y0, int x1, int y1)
{
	if (!g_stripBuffer || !g_drawFramebuffer) return false;
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= kWidth) x1 = kWidth - 1;
	if (y1 >= kHeight) y1 = kHeight - 1;
	const int rows = y1 - y0 + 1;
	if (x0 > x1 || rows <= 0 || rows > kStripRows) return false;
	const int core = xPortGetCoreID() & 1;
	ReplayStageState &st = g_replayStage[core];
	if (st.active) return false;  // staging never nests; fall back to direct if it ever would
	std::uint16_t *strip = g_stripBuffer + static_cast<std::size_t>(core) * kStripRows * kWidth;
	// Seed the window from the back buffer so alpha blends and partial paints meet
	// the same dst bytes the direct path would have.
	const std::int64_t s0 = esp_timer_get_time();
	const int width = x1 - x0 + 1;
	for (int y = y0; y <= y1; ++y) {
		std::memcpy(&strip[static_cast<std::size_t>(y - y0) * kWidth + x0],
		            &g_drawFramebuffer[static_cast<std::size_t>(y) * kWidth + x0],
		            static_cast<std::size_t>(width) * sizeof(std::uint16_t));
	}
	st.seedUs += esp_timer_get_time() - s0;
	st.px += width * rows;
	auto &canvas = replayCanvas();
	st.hadDirty = canvas.dirty(&st.dx0, &st.dy0, &st.dx1, &st.dy1);
	st.alpha = canvas.globalAlpha();
	// Window binding: claim rows [0..y1] with the pixel base rewound by y0 rows, so
	// screen row y lands at strip row (y - y0) — the replay keeps drawing in screen
	// coordinates. Rows above the window are unbacked memory; the guard clip below
	// keeps every drawer inside the window (all drawers clamp to the clip, and the
	// replay never reads outside it).
	canvas.bindPixels(strip - static_cast<std::size_t>(y0) * kWidth, kWidth, y1 + 1, kWidth);
	canvas.setGlobalAlpha(st.alpha);
	canvas.pushClip(x0, y0, width, rows);
	st.active = true;
	st.x0 = x0;
	st.y0 = y0;
	st.x1 = x1;
	st.y1 = y1;
	st.beginDoneUs = esp_timer_get_time();
	return true;
}

extern "C" void gea_replay_stage_end()
{
	const int core = xPortGetCoreID() & 1;
	ReplayStageState &st = g_replayStage[core];
	if (!st.active) return;
	st.active = false;
	const std::int64_t e0 = esp_timer_get_time();
	st.bodyUs += e0 - st.beginDoneUs;
	auto &canvas = replayCanvas();
	int dx0, dy0, dx1, dy1;
	bool dirty = canvas.dirty(&dx0, &dy0, &dx1, &dy1);
	if (dirty) {
		// Copy back only the painted bbox (clamped to the window) — unpainted strip
		// pixels are seed-identical, so skipping them saves PSRAM write traffic.
		if (dx0 < st.x0) dx0 = st.x0;
		if (dy0 < st.y0) dy0 = st.y0;
		if (dx1 > st.x1) dx1 = st.x1;
		if (dy1 > st.y1) dy1 = st.y1;
		dirty = dx0 <= dx1 && dy0 <= dy1;
	}
	if (dirty) {
		const std::uint16_t *strip = g_stripBuffer + static_cast<std::size_t>(core) * kStripRows * kWidth;
		const int width = dx1 - dx0 + 1;
		for (int y = dy0; y <= dy1; ++y) {
			std::memcpy(&g_drawFramebuffer[static_cast<std::size_t>(y) * kWidth + dx0],
			            &strip[static_cast<std::size_t>(y - st.y0) * kWidth + dx0],
			            static_cast<std::size_t>(width) * sizeof(std::uint16_t));
		}
	}
	canvas.bindPixels(g_drawFramebuffer, kWidth, kHeight);
	canvas.setGlobalAlpha(st.alpha);
	// bindPixels reset the dirty tracker: restore the frame's accumulated union,
	// then add this strip's painted bbox so the publish copy transmits it.
	if (st.hadDirty) canvas.markDirty(st.dx0, st.dy0, st.dx1, st.dy1);
	if (dirty) canvas.markDirty(dx0, dy0, dx1, dy1);
	st.cbUs += esp_timer_get_time() - e0;
	if (++st.strips >= kStageDiagStrips) {
		std::printf("[stage] core%d strips=%d px=%d seed=%dus body=%dus cb=%dus (avg/strip) pie=%d/%dpx\n",
		            core, st.strips, st.px / st.strips,
		            static_cast<int>(st.seedUs / st.strips),
		            static_cast<int>(st.bodyUs / st.strips),
		            static_cast<int>(st.cbUs / st.strips),
		            g_pieCalls, g_pieCount8 * 8);
		g_pieCalls = 0;
		g_pieCount8 = 0;
		st.seedUs = st.bodyUs = st.cbUs = 0;
		st.strips = 0;
		st.px = 0;
	}
}

namespace {

// ── PIE SIMD over-blend (gea_pie_blend hooks) ────────────────────────────────
// Same S3 vector kernel as targets/esp32/display.cpp's pieBlendSpan8, minus the
// panel<->normal byte-swap steps: this board stores native-order RGB565
// (GEA_EMBEDDED_PIXEL_PANEL_ENDIAN=0), so the framebuffer bytes already ARE the
// "normal" layout the channel masks need. Contract unchanged: dst = bg in /
// result out (storage order), fgN = normal-RGB565 fg, a5 = per-pixel 0..32;
// lerp oC = bgC + ((fgC-bgC)*a5)>>5 per channel, +16 round-to-nearest.
// cb2 rows (16B apart): [0]=1, [1]=0x1F, [2]=0x3F, [5]=2048(<<11), [6]=32(<<5),
// [7]=16(round); rows 3/4 (0xFF mask, <<8) are the swap constants — unused here
// but kept so the table layout matches the esp32 kernel's documented block.
// Held: q0=bg, q1=ones, q2=fgN, q3=a5, q6=accum. Scratch: q4, q5(const/mask), q7.
void pieBlendSpan8Native(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8,
                         const std::int16_t *cb2)
{
	int t, ad;
	asm volatile(
	    "ee.vld.128.ip q1, %[cb], 0\n" // q1 = ones (cb2[0])
	    "1:\n"
	    "ee.vld.128.ip q0, %[d], 0\n"  // q0 = bg (native order), no increment (stored back)
	    "ee.vld.128.ip q2, %[f], 16\n" // q2 = fgN
	    "ee.vld.128.ip q3, %[a], 16\n" // q3 = a5
	    // B channel (bits 0-4): oB = bgB + ((fgB-bgB)*a5 + 16)>>5
	    "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n"                            // q5 = mask1F
	    "ee.andq q4, q0, q5\n"                                                            // bgB
	    "ee.andq q7, q2, q5\n"                                                            // fgB
	    "ee.vsubs.s16 q7, q7, q4\n"                                                       // fgB-bgB
	    "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                          // diff*a5 (raw)
	    "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n" // +16 (round)
	    "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                          // >>5
	    "ee.vadds.s16 q6, q7, q4\n"                                                       // oB (accum)
	    // G channel (bits 5-10): mask 0x3F, repack <<5
	    "addi %[ad], %[cb], 32\n ee.vld.128.ip q5, %[ad], 0\n"                                                        // q5 = mask3F
	    "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n"                                 // bgG
	    "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n"                                                               // fgG (SAR still 5)
	    "ee.vsubs.s16 q7, q7, q4\n"                                                                                   // diff
	    "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                                                      // diff*a5 (raw)
	    "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"                             // +16
	    "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                                                      // >>5
	    "ee.vadds.s16 q7, q7, q4\n"                                                                                   // oG
	    "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 96\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n" // oG<<5 (c32)
	    "ee.orq q6, q6, q7\n"
	    // R channel (bits 11-15): mask 0x1F, repack <<11
	    "addi %[ad], %[cb], 16\n ee.vld.128.ip q5, %[ad], 0\n"                         // q5 = mask1F
	    "movi %[t], 11\n wsr.sar %[t]\n ee.vmul.s16 q4, q0, q1\n ee.andq q4, q4, q5\n" // bgR
	    "ee.vmul.s16 q7, q2, q1\n ee.andq q7, q7, q5\n"                                // fgR (SAR still 11)
	    "ee.vsubs.s16 q7, q7, q4\n"
	    "movi %[t], 0\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q3\n"                                                      // diff*a5 (raw)
	    "addi %[ad], %[cb], 112\n ee.vld.128.ip q5, %[ad], 0\n ee.vadds.s16 q7, q7, q5\n"                             // +16
	    "movi %[t], 5\n wsr.sar %[t]\n ee.vmul.s16 q7, q7, q1\n"                                                      // >>5
	    "ee.vadds.s16 q7, q7, q4\n"                                                                                   // oR
	    "movi %[t], 0\n wsr.sar %[t]\n addi %[ad], %[cb], 80\n ee.vld.128.ip q5, %[ad], 0\n ee.vmul.s16 q7, q7, q5\n" // oR<<11 (c2048)
	    "ee.orq q6, q6, q7\n"                                                                                         // q6 = result (native order)
	    "ee.vst.128.ip q6, %[d], 16\n" // store result + advance dst by 8 px
	    "addi %[n], %[n], -1\n bnez %[n], 1b\n"
	    : [d] "+r"(dst), [f] "+r"(fgN), [a] "+r"(a5), [n] "+r"(count8), [t] "=&r"(t), [ad] "=&r"(ad)
	    : [cb] "r"(cb2)
	    : "memory");
}

}  // namespace

// Like-for-like on css-3d-cube (same app build, even split, no staging): scalar
// gradient replay = 28-41ms/frame, SIMD = ~13.5ms — the kernel halves the
// dominant replay cost. (An earlier "no difference" reading compared across two
// different app revisions — the example was being reworked in a parallel session
// mid-measurement.)
extern "C" bool gea_pie_blend_available() { return true; }
extern "C" void gea_pie_blend_span8(std::uint16_t *dst, const std::uint16_t *fgN, const std::int16_t *a5, int count8)
{
	// Same constant block layout as the esp32 kernel (rows 3/4 unused here).
	alignas(16) static const std::int16_t cb2[8][8] = {
	    {1, 1, 1, 1, 1, 1, 1, 1},
	    {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F},
	    {0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F},
	    {0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF},
	    {256, 256, 256, 256, 256, 256, 256, 256},
	    {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048},
	    {32, 32, 32, 32, 32, 32, 32, 32},
	    {16, 16, 16, 16, 16, 16, 16, 16},
	};
	// PER-CORE critical section — the PIE q-registers are not saved across FreeRTOS
	// context switches, so a mid-kernel preemption (WiFi/system tasks on core 0,
	// the render worker's own preemptors on core 1) corrupts the blend. Per-core
	// mux so the two render cores never serialize on it (proven pattern on the
	// esp32 target, where corruption stripes appeared without the guard).
	static portMUX_TYPE pieMux[2] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};
	portMUX_TYPE *mux = &pieMux[xPortGetCoreID() & 1];
	g_pieCalls = g_pieCalls + 1;
	g_pieCount8 = g_pieCount8 + count8;
	taskENTER_CRITICAL(mux);
	pieBlendSpan8Native(dst, fgN, a5, count8, &cb2[0][0]);
	taskEXIT_CRITICAL(mux);
}

extern "C" int gea_current_render_core()
{
	return static_cast<int>(xPortGetCoreID());
}

extern "C" void gea_render_parallel_wait()
{
	if (!g_renderRowWorker.task) return;
	const std::int64_t w0 = esp_timer_get_time();
	const std::int64_t deadline = w0 + 100000;
	while (!g_renderRowWorker.done.load(std::memory_order_acquire)) {
		if (esp_timer_get_time() > deadline) {
			ESP_LOGW(kTag, "row worker wait timeout");
			break;
		}
	}
	if (g_lastSubmitWasDrawBand) {
		g_drawBandWaitUs += esp_timer_get_time() - w0;
		if (++g_drawBandJobs >= 300) {
			std::printf("[worker] draw-band jobs=300 wait=%dus band=%dus avg\n",
			            static_cast<int>(g_drawBandWaitUs / 300),
			            static_cast<int>(g_renderRowWorker.bandUs / 300));
			g_drawBandWaitUs = 0;
			g_drawBandJobs = 0;
			g_renderRowWorker.bandUs = 0;
		}
	}
}

namespace gea::platform::display {

bool Display::init()
{
	if (g_initialized) return true;
	esp_err_t err = initBacklight();
	if (err != ESP_OK) return false;
	err = initSt7701();
	if (err != ESP_OK) {
		ESP_LOGE(kTag, "ST7701 init failed: %s", esp_err_to_name(err));
		return false;
	}
	err = initRgbPanel();
	if (err != ESP_OK) return false;
	g_initialized = true;
	return true;
}

bool Display::start()
{
	if (!g_initialized && !init()) return false;
	setBrightness(g_brightness);
	flush();
	return true;
}

// Routed per-core: the worker core's band replay (and render.cpp helpers that
// draw raw pixels through canvas()) must hit the band canvas — its own clip +
// dirty accumulator over the same framebuffer. On the main core this is g_canvas.
gea::framework::graphics::Canvas *Display::canvas() { return &replayCanvas(); }

// The render layer binds canvas() onto a scratch buffer (retained snapshots for
// screenshots/OTA, fused rasterized flushes) and calls this to point it back at
// the draw framebuffer. bindPixels resets the dirty tracker, which is right
// here: the union it held describes the scratch buffer, not the framebuffer.
void Display::rebindCanvasToFramebuffer()
{
	replayCanvas().bindPixels(g_drawFramebuffer, kWidth, kHeight);
}

bool Display::copySnapshotRgb565(std::uint16_t *dst, int pixelCapacity, int *width, int *height, bool)
{
	if (width) *width = kWidth;
	if (height) *height = kHeight;
	if (!dst || pixelCapacity < static_cast<int>(kPixelCount)) return false;
	const std::uint16_t *source = g_framebuffer ? g_framebuffer : g_drawFramebuffer;
	if (!source) return false;
	std::copy_n(source, kPixelCount, dst);
	return true;
}

int Display::countNonBlackPixels(bool)
{
	const std::uint16_t *source = g_framebuffer ? g_framebuffer : g_drawFramebuffer;
	if (!source) return 0;
	int count = 0;
	for (std::size_t i = 0; i < kPixelCount; ++i) {
		if (source[i] != 0) ++count;
	}
	return count;
}

void Display::clear()
{
	g_canvas.clear(0x0000);
	flush();
}

void Display::clearNoFlush() { g_canvas.clear(0x0000); }
void Display::print(const char *) {}

void Display::flush()
{
	if (!g_framebuffer || !g_drawFramebuffer) return;
	const int64_t start = esp_timer_get_time();
	int64_t copyUs = 0;
	int64_t cacheUs = 0;
	int x0 = 0;
	int y0 = 0;
	int x1 = -1;
	int y1 = -1;
	int pixels = 0;
		if (g_canvas.dirty(&x0, &y0, &x1, &y1)) {
			const int64_t copyStart = esp_timer_get_time();
			pixels = publishDrawRect(x0, y0, x1, y1, &x0, &y0, &x1, &y1);
			copyUs += esp_timer_get_time() - copyStart;
		}
		if (pixels > 0) {
			const int64_t cacheStart = esp_timer_get_time();
			cacheWritebackRect(x0, y0, x1, y1);
			cacheUs += esp_timer_get_time() - cacheStart;
		}
	g_canvas.resetDirty();
	addFlushStats(esp_timer_get_time() - start, pixels);
	addFlushStageStats(copyUs, cacheUs);
}

void Display::flushRects(const DisplayFlushRect *rects, int count, bool)
{
	if (!g_framebuffer || !g_drawFramebuffer || !rects || count <= 0) return;
	const int64_t start = esp_timer_get_time();
	int64_t copyUs = 0;
	int64_t cacheUs = 0;
		int pixels = 0;
		for (int i = 0; i < count; ++i) {
			int x0 = 0;
			int y0 = 0;
			int x1 = -1;
			int y1 = -1;
			const int64_t copyStart = esp_timer_get_time();
			const int rectPixels = publishDrawRect(rects[i].x0, rects[i].y0, rects[i].x1, rects[i].y1, &x0, &y0, &x1, &y1);
			copyUs += esp_timer_get_time() - copyStart;
			if (rectPixels > 0) {
				pixels += rectPixels;
				const int64_t cacheStart = esp_timer_get_time();
				cacheWritebackRect(x0, y0, x1, y1);
				cacheUs += esp_timer_get_time() - cacheStart;
			}
		}
		g_canvas.resetDirty();
	addFlushStats(esp_timer_get_time() - start, pixels);
	addFlushStageStats(copyUs, cacheUs);
}

bool Display::streamRect(int x, int y, int w, int h, DisplayStreamRasterFn raster, void *user)
{
	if (!g_framebuffer || !g_drawFramebuffer || !raster || w <= 0 || h <= 0) return false;
	int x0 = x;
	int y0 = y;
	int x1 = x + w - 1;
	int y1 = y + h - 1;
	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= kWidth) x1 = kWidth - 1;
	if (y1 >= kHeight) y1 = kHeight - 1;
	if (x0 > x1 || y0 > y1) return true;

	const int width = x1 - x0 + 1;
	const int height = y1 - y0 + 1;
	const std::size_t bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * sizeof(std::uint16_t);
	auto *scratch = static_cast<std::uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!scratch) scratch = static_cast<std::uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
	if (!scratch) return false;

	const int64_t start = esp_timer_get_time();
	raster(scratch, width, height, x0, y0, user);
	for (int row = 0; row < height; ++row) {
		std::copy_n(&scratch[static_cast<std::size_t>(row) * width],
		            width,
		            &g_drawFramebuffer[static_cast<std::size_t>(y0 + row) * kWidth + x0]);
	}
	heap_caps_free(scratch);
		int publishedX0 = 0;
		int publishedY0 = 0;
		int publishedX1 = -1;
		int publishedY1 = -1;
		const int pixels = publishDrawRect(x0, y0, x1, y1, &publishedX0, &publishedY0, &publishedX1, &publishedY1);
		if (pixels > 0) cacheWritebackRect(publishedX0, publishedY0, publishedX1, publishedY1);
		g_canvas.resetDirty();
		addFlushStats(esp_timer_get_time() - start, pixels);
	return true;
}

bool Display::present(const DisplayPresentCommand *commands, int commandCount)
{
	if (!commands || commandCount <= 0) return false;
#if GEA_EMBEDDED_DISPLAY_RGB_STRIP_RASTER
	// Fast path: full-screen-redraw frames compose in SRAM strips and publish
	// straight to scanout. Anything without a full-screen opaque base needs the
	// persistent back buffer, so it falls through.
	if (g_stripBuffer) {
		present::Frame frame;
		if (present::extractFrame(commands, commandCount, frame) &&
		    present::frameHasOpaqueBase(frame, kWidth, kHeight)) {
			return presentStripFused(frame);
		}
	}
#endif
	// Fallback: per-command replay into the PSRAM back buffer, then publish. If
	// the last frame went through the strip path, the back buffer is stale —
	// re-seed it from the live scanout before drawing the partial update on top.
	if (g_drawBufferStale && g_drawFramebuffer != g_framebuffer) {
		std::memcpy(g_drawFramebuffer, g_framebuffer, kFramebufferBytes);
		g_drawBufferStale = false;
	}
	for (int i = 0; i < commandCount; ++i) {
		replayPresentCommand(commands[i]);
	}
	g_canvas.setGlobalAlpha(255);
	flush();
	return true;
}

void Display::setFlushConfig(int chunkRows, int queueDepth)
{
	g_flushRows = chunkRows;
	g_flushDepth = queueDepth;
}

void Display::reserveInternal(std::size_t) {}
bool Display::setHighBrightnessMode(bool) { return false; }
bool Display::highBrightnessMode() { return false; }
// The RGB-panel backend draws straight into the framebuffer with no separate
// internal-RAM flush staging to reserve, so the deferred-reserve apply is a no-op
// here (matches reserveInternal above). Required since frame_scheduler now calls it.
void Display::applyPendingInternalReserve() {}
int Display::flushChunkRows() { return g_flushRows; }
int Display::flushQueueDepth() { return g_flushDepth; }
int Display::flushBufferBytes() { return 0; }
// All clip/alpha/draw entry points below route through replayCanvas() so the
// 2nd-core band replay lands on the worker canvas (its own clip + dirty over
// the same framebuffer). On the main core replayCanvas() == g_canvas, so the
// single-core paths are unchanged.
void Display::pushClip(int x, int y, int w, int h) { replayCanvas().pushClip(x, y, w, h); }
void Display::popClip() { replayCanvas().popClip(); }
void Display::resetClip() { replayCanvas().resetClip(); }
void Display::setAlpha(std::uint8_t alpha) { replayCanvas().setGlobalAlpha(alpha); }
std::uint8_t Display::alpha() { return replayCanvas().globalAlpha(); }
int Display::brightness() { return g_brightness; }

void Display::setBrightness(int brightnessPercent)
{
	if (brightnessPercent < 0) brightnessPercent = 0;
	if (brightnessPercent > 100) brightnessPercent = 100;
	g_brightness = brightnessPercent;
	if (g_backlightPwmInitialized) updateBacklightDuty();
}

// Tearing sync (TE/VBlank): not implemented on this target.
void Display::setVSync(bool) {}
void Display::invalidate() {}
bool Display::vsyncEnabled() { return false; }
void Display::vsyncWaitForFrame() {}

void Display::clip(int *x0, int *y0, int *x1, int *y1) { replayCanvas().currentClip(x0, y0, x1, y1); }
void Display::fillRect(int x, int y, int w, int h, std::uint16_t color) { replayCanvas().fillRect(x, y, w, h, color); }
void Display::scrollRect(int x, int y, int w, int h, int dx, int dy) { replayCanvas().scrollRect(x, y, w, h, dx, dy); }
void Display::resetScrollRegion() { replayCanvas().setScrollRegion(0, 0, 0); }
void Display::strokeRect(int x, int y, int w, int h, std::uint16_t color) { replayCanvas().strokeRect(x, y, w, h, color); }
void Display::fillCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().fillCircle(cx, cy, r, color); }
void Display::strokeCircle(int cx, int cy, int r, std::uint16_t color) { replayCanvas().strokeCircle(cx, cy, r, color); }
void Display::drawLine(int x0, int y0, int x1, int y1, std::uint16_t color) { replayCanvas().drawLine(x0, y0, x1, y1, color); }
void Display::drawArc(int cx, int cy, int r, int startDeg, int endDeg, std::uint16_t color) { replayCanvas().drawArc(cx, cy, r, startDeg, endDeg, color); }
void Display::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, std::uint16_t color) { replayCanvas().fillTriangle(x0, y0, x1, y1, x2, y2, color); }
void Display::drawText(const char *text, int x, int y, std::uint16_t color, float scale) { replayCanvas().drawText(text, x, y, color, scale); }
void Display::drawTextFont(const char *text, int x, int y, std::uint16_t color, int fontId) { replayCanvas().drawTextFont(text, x, y, color, fontId); }
void Display::drawTextFontFamily(const char *text, int x, int y, std::uint16_t color, int familyId, int sizePx) { replayCanvas().drawTextFontFamily(text, x, y, color, familyId, sizePx); }
void Display::setPixel(int x, int y, std::uint16_t color) { replayCanvas().fillRect(x, y, 1, 1, color); }
void Display::fillRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, std::uint16_t color) { replayCanvas().fillRoundedRect(x, y, w, h, tl, tr, br, bl, color); }
void Display::fillRoundedRectBoxesRgb565(const std::int16_t *xs, const std::int16_t *ys, int count,
                                         int w, int h, int tl, int tr, int br, int bl,
                                         const std::uint16_t *colors)
{
	replayCanvas().fillRoundedRectBoxesRgb565(xs, ys, count, w, h, tl, tr, br, bl, colors);
}
void Display::strokeRoundedRect(int x, int y, int w, int h, int tl, int tr, int br, int bl, int lineWidth, std::uint16_t color) { replayCanvas().strokeRoundedRect(x, y, w, h, tl, tr, br, bl, lineWidth, color); }
void Display::blitImage(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy) { replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy); }
void Display::blitImageScaled(const gea::framework::graphics::pixel::native_t *src, const std::uint8_t *alpha, int srcWidth, int srcHeight, int dx, int dy, int dstWidth, int dstHeight) { replayCanvas().drawImage(src, alpha, srcWidth, srcHeight, dx, dy, dstWidth, dstHeight); }
void Display::setWorldOverlay(const std::uint16_t *, int, int, int, int) {}
void Display::setWorldScroll(int) {}

void Display::flushStatsRead(int64_t *totalUs, int *callCount, int *pixelCount)
{
	if (totalUs) *totalUs = g_stats.totalUs;
	if (callCount) *callCount = g_stats.callCount;
	if (pixelCount) *pixelCount = g_stats.pixelCount;
}

DisplayFlushPerfStats Display::flushPerfStatsRead() { return g_stats; }
void Display::flushStatsReset() { g_stats = {}; }
const char *Display::flushStageName() { return "idle"; }
int Display::flushStageChunk() { return 0; }
DisplayFlushStageDetail Display::flushStageDetail() { return {}; }

}  // namespace gea::platform::display

// Full-screen PSRAM scratch for the static-backdrop cache. Without this hook the
// weak default in ui/render.cpp returns nullptr and the bake SILENTLY never runs —
// an animated-transform scene (css-3d-cube) then full-rebuilds + re-rasterizes the
// whole static stage every frame (~17fps at 480x480) instead of reprojecting over
// a baked backdrop. Same shape as the targets/esp32/display.cpp implementation.
extern "C" std::uint16_t *gea_backdrop_cache(int *cap_px)
{
	static std::uint16_t *buffer = nullptr;
	static bool attempted = false;
	constexpr int kCapPx = gea::platform::display::kWidth * gea::platform::display::kHeight;
	if (!attempted)
	{
		attempted = true;
		buffer = static_cast<std::uint16_t *>(heap_caps_malloc(
		    static_cast<std::size_t>(kCapPx) * sizeof(std::uint16_t), MALLOC_CAP_SPIRAM));
	}
	if (cap_px)
		*cap_px = buffer ? kCapPx : 0;
	return buffer;
}
