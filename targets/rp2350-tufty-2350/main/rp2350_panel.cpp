#include "rp2350_panel.h"

#include "board.h"
#include "st7789_parallel.pio.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gea::rp2350 {
namespace {

namespace board = gea::platform::board;

enum class Reg : std::uint8_t {
	SwReset = 0x01,
	MadCtl = 0x36,
	ColMod = 0x3a,
	PorCtrl = 0xb2,
	RamCtrl = 0xb0,
	GCtrl = 0xb7,
	VComS = 0xbb,
	LcmCtrl = 0xc0,
	VdvVrhEn = 0xc2,
	Vrhs = 0xc3,
	Vdvs = 0xc4,
	FrameCtrl2 = 0xc6,
	PowerCtrl1 = 0xd0,
	GammaPositive = 0xe0,
	GammaNegative = 0xe1,
	InvOn = 0x21,
	SleepOut = 0x11,
	DisplayOn = 0x29,
	RamWrite = 0x2c,
	ColumnAddress = 0x2a,
	RowAddress = 0x2b,
};

struct PanelState {
	PIO pio = pio1;
	uint sm = 0;
	uint offset = 0;
	uint dma = 0;
	dma_channel_config dmaConfig{};
	bool ready = false;
	int flushRows = 64;
	int flushDepth = 1;
};

PanelState gPanel;
std::uint16_t gLineA[kNativePanelWidth];
std::uint16_t gLineB[kNativePanelWidth];
std::uint16_t gRasterLine[kPanelWidth];
PanelFlushStats gFlushStats;

std::uint16_t be16(std::uint16_t value)
{
	return static_cast<std::uint16_t>((value << 8) | (value >> 8));
}

void waitForPioIdle()
{
	const std::uint32_t stallMask = 1u << (PIO_FDEBUG_TXSTALL_LSB + gPanel.sm);
	gPanel.pio->fdebug = stallMask;
	while (!(gPanel.pio->fdebug & stallMask)) {
		tight_loop_contents();
	}
}

void waitDma()
{
	const std::uint64_t startUs = time_us_64();
	dma_channel_wait_for_finish_blocking(gPanel.dma);
	waitForPioIdle();
	gFlushStats.txWaitUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

void startDma(const std::uint8_t *data, std::size_t bytes)
{
	if (!data || bytes == 0) return;
	gFlushStats.chunkCount++;
	dma_channel_set_trans_count(gPanel.dma, static_cast<std::uint32_t>(bytes), false);
	dma_channel_set_read_addr(gPanel.dma, data, true);
}

void writeBlocking(const std::uint8_t *data, std::size_t bytes)
{
	startDma(data, bytes);
	waitDma();
}

void command(std::uint8_t reg, const std::uint8_t *data = nullptr, std::size_t bytes = 0)
{
	waitDma();
	gpio_put(board::display.dc, 0);
	gpio_put(board::display.cs, 0);
	writeBlocking(&reg, 1);
	if (data && bytes > 0) {
		gpio_put(board::display.dc, 1);
		writeBlocking(data, bytes);
	}
	gpio_put(board::display.cs, 1);
}

void command(Reg reg, const std::uint8_t *data = nullptr, std::size_t bytes = 0)
{
	command(static_cast<std::uint8_t>(reg), data, bytes);
}

void commandBytes(Reg reg, const char *data, std::size_t bytes)
{
	command(reg, reinterpret_cast<const std::uint8_t *>(data), bytes);
}

void setWindow(int x0, int y0, int x1, int y1)
{
	x0 = std::clamp(x0, 0, kNativePanelWidth - 1);
	y0 = std::clamp(y0, 0, kNativePanelHeight - 1);
	x1 = std::clamp(x1, x0, kNativePanelWidth - 1);
	y1 = std::clamp(y1, y0, kNativePanelHeight - 1);

	const std::uint16_t columns[] = {
	    be16(static_cast<std::uint16_t>(x0)),
	    be16(static_cast<std::uint16_t>(x1)),
	};
	const std::uint16_t rows[] = {
	    be16(static_cast<std::uint16_t>(y0)),
	    be16(static_cast<std::uint16_t>(y1)),
	};
	command(Reg::ColumnAddress, reinterpret_cast<const std::uint8_t *>(columns), sizeof(columns));
	command(Reg::RowAddress, reinterpret_cast<const std::uint8_t *>(rows), sizeof(rows));
}

void beginPixels(int x0, int y0, int x1, int y1)
{
	const std::uint64_t startUs = time_us_64();
	setWindow(x0, y0, x1, y1);
	gFlushStats.setWindowUs += static_cast<std::int64_t>(time_us_64() - startUs);
	waitDma();
	const auto ramWrite = static_cast<std::uint8_t>(Reg::RamWrite);
	gpio_put(board::display.dc, 0);
	gpio_put(board::display.cs, 0);
	writeBlocking(&ramWrite, 1);
	gpio_put(board::display.dc, 1);
}

void endPixels()
{
	waitDma();
	gpio_put(board::display.cs, 1);
}

void configureDma(bool readIncrement)
{
	gPanel.dmaConfig = dma_channel_get_default_config(gPanel.dma);
	channel_config_set_read_increment(&gPanel.dmaConfig, readIncrement);
	channel_config_set_transfer_data_size(&gPanel.dmaConfig, DMA_SIZE_8);
	channel_config_set_dreq(&gPanel.dmaConfig, pio_get_dreq(gPanel.pio, gPanel.sm, true));
	dma_channel_configure(gPanel.dma, &gPanel.dmaConfig, &gPanel.pio->txf[gPanel.sm], nullptr, 0, false);
}

void initParallelBus()
{
	gPanel.pio = board::display.pio == 1 ? pio1 : pio0;
	gPanel.sm = static_cast<uint>(board::display.stateMachine);
	pio_set_gpio_base(gPanel.pio, board::display.data0 + 8 >= 32 ? 16 : 0);

	pio_gpio_init(gPanel.pio, board::display.wr);
	for (int pin = 0; pin < 8; ++pin) {
		pio_gpio_init(gPanel.pio, board::display.data0 + pin);
	}

	pio_sm_claim(gPanel.pio, gPanel.sm);
	gPanel.offset = pio_add_program(gPanel.pio, &st7789_parallel_program);

	pio_sm_config config = st7789_parallel_program_get_default_config(gPanel.offset);
	sm_config_set_out_pins(&config, board::display.data0, 8);
	sm_config_set_sideset_pins(&config, board::display.wr);
	sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
	sm_config_set_out_shift(&config, false, true, 8);
	const std::uint32_t sysHz = clock_get_hz(clk_sys);
	constexpr std::uint32_t maxPioHz = 44u * 1000u * 1000u;
	float div = static_cast<float>(sysHz) / static_cast<float>(maxPioHz);
	if (div < 1.0f) div = 1.0f;
	sm_config_set_clkdiv(&config, div);

	pio_sm_set_consecutive_pindirs(gPanel.pio, gPanel.sm, board::display.data0, 8, true);
	pio_sm_set_consecutive_pindirs(gPanel.pio, gPanel.sm, board::display.wr, 1, true);
	pio_sm_init(gPanel.pio, gPanel.sm, gPanel.offset, &config);
	pio_sm_set_enabled(gPanel.pio, gPanel.sm, true);

	gPanel.dma = dma_claim_unused_channel(true);
	configureDma(true);
}

void initPins()
{
	gpio_init(board::display.cs);
	gpio_set_dir(board::display.cs, GPIO_OUT);
	gpio_put(board::display.cs, 1);

	gpio_init(board::display.dc);
	gpio_set_dir(board::display.dc, GPIO_OUT);
	gpio_put(board::display.dc, 1);

	gpio_init(board::display.rd);
	gpio_set_dir(board::display.rd, GPIO_OUT);
	gpio_put(board::display.rd, 1);

	if (board::display.vsync >= 0) {
		gpio_init(board::display.vsync);
		gpio_set_dir(board::display.vsync, GPIO_IN);
	}

	pwm_config config = pwm_get_default_config();
	const uint slice = pwm_gpio_to_slice_num(board::display.backlight);
	pwm_set_wrap(slice, 65535);
	pwm_init(slice, &config, true);
	gpio_set_function(board::display.backlight, GPIO_FUNC_PWM);
	panelSetBrightness(0);
}

void initRegisters()
{
	command(Reg::SwReset);
	sleep_ms(150);

	commandBytes(Reg::ColMod, "\x05", 1);
	commandBytes(Reg::PorCtrl, "\x0c\x0c\x00\x33\x33", 5);
	commandBytes(Reg::LcmCtrl, "\x2c", 1);
	commandBytes(Reg::VdvVrhEn, "\x01", 1);
	commandBytes(Reg::Vrhs, "\x0f", 1);
	commandBytes(Reg::Vdvs, "\x20", 1);
	commandBytes(Reg::PowerCtrl1, "\xa4\xa1", 2);
	commandBytes(Reg::FrameCtrl2, "\x0f", 1);
	commandBytes(Reg::RamCtrl, "\x00\xc0", 2);
	commandBytes(Reg::GCtrl, "\x35", 1);
	commandBytes(Reg::VComS, "\x1b", 1);
	commandBytes(Reg::GammaPositive, "\xf0\x00\x06\x04\x05\x05\x31\x44\x48\x36\x12\x12\x2b\x34", 14);
	commandBytes(Reg::GammaNegative, "\xf0\x0b\x0f\x0f\x0d\x26\x31\x43\x47\x38\x14\x14\x2c\x32", 14);
	command(Reg::InvOn);
	command(Reg::SleepOut);
	sleep_ms(100);

	constexpr std::uint8_t kMadctlNativeLandscape180 = 0x80 | 0x20 | 0x10;
	command(Reg::MadCtl, &kMadctlNativeLandscape180, 1);

	panelClear(0x0000);
	command(Reg::DisplayOn);
	panelSetBrightness(100);
}

int clipped(int value, int min, int max)
{
	return std::max(min, std::min(max, value));
}

void flushPreparedLine(std::uint16_t *line, int count)
{
	startDma(reinterpret_cast<const std::uint8_t *>(line), static_cast<std::size_t>(count) * sizeof(std::uint16_t));
}

int nativeStart(int logical)
{
	return logical * kPanelScale;
}

int nativeEndInclusive(int logical)
{
	return (logical + 1) * kPanelScale - 1;
}

int expandRowFromFramebuffer(std::uint16_t *out,
                             const std::uint16_t *pixels,
                             int stridePixels,
                             int y,
                             int x0,
                             int x1)
{
	int count = 0;
	for (int x = x0; x <= x1; ++x) {
		const std::uint16_t color = pixels[static_cast<std::size_t>(y) * stridePixels + x];
		for (int repeat = 0; repeat < kPanelScale; ++repeat) {
			out[count++] = color;
		}
	}
	return count;
}

int expandRowFromRaster(std::uint16_t *out, const std::uint16_t *source, int width)
{
	int count = 0;
	for (int x = 0; x < width; ++x) {
		const std::uint16_t color = source[x];
		for (int repeat = 0; repeat < kPanelScale; ++repeat) {
			out[count++] = color;
		}
	}
	return count;
}

}  // namespace

bool panelInit()
{
	if (gPanel.ready) return true;
	initPins();
	initParallelBus();
	gPanel.ready = true;
	initRegisters();
	return true;
}

void panelSetBrightness(int percent)
{
	percent = std::clamp(percent, 0, 100);
	const std::uint32_t p = static_cast<std::uint32_t>(percent);
	const std::uint16_t level = static_cast<std::uint16_t>((p * p * 65535u) / 10000u);
	pwm_set_gpio_level(board::display.backlight, level);
}

void panelSetFlushConfig(int rows, int depth)
{
	if (rows > 0) gPanel.flushRows = std::min(rows, kPanelHeight);
	if (depth > 0) gPanel.flushDepth = std::max(1, depth);
}

void panelFlushRect(const std::uint16_t *pixels, int stridePixels, int x0, int y0, int x1, int y1)
{
	if (!panelInit() || !pixels || stridePixels <= 0) return;
	x0 = clipped(x0, 0, kPanelWidth - 1);
	y0 = clipped(y0, 0, kPanelHeight - 1);
	x1 = clipped(x1, x0, kPanelWidth - 1);
	y1 = clipped(y1, y0, kPanelHeight - 1);

	const std::uint64_t startUs = time_us_64();
	gFlushStats.callCount++;
	const int nativeX0 = nativeStart(x0);
	const int nativeY0 = nativeStart(y0);
	const int nativeX1 = nativeEndInclusive(x1);
	const int nativeY1 = nativeEndInclusive(y1);
	gFlushStats.pixelCount += (nativeX1 - nativeX0 + 1) * (nativeY1 - nativeY0 + 1);
	beginPixels(nativeX0, nativeY0, nativeX1, nativeY1);
	std::uint16_t *current = gLineA;
	std::uint16_t *next = gLineB;
	for (int y = y0; y <= y1; ++y) {
		const std::uint64_t copyStartUs = time_us_64();
		const int preparedWidth = expandRowFromFramebuffer(current, pixels, stridePixels, y, x0, x1);
		gFlushStats.copyUs += static_cast<std::int64_t>(time_us_64() - copyStartUs);
		for (int repeat = 0; repeat < kPanelScale; ++repeat) {
			waitDma();
			flushPreparedLine(current, preparedWidth);
		}
		std::swap(current, next);
	}
	endPixels();
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

void panelStreamRect(int x0, int y0, int x1, int y1, PanelStreamRasterFn raster, void *user)
{
	if (!panelInit() || !raster) return;
	x0 = clipped(x0, 0, kPanelWidth - 1);
	y0 = clipped(y0, 0, kPanelHeight - 1);
	x1 = clipped(x1, x0, kPanelWidth - 1);
	y1 = clipped(y1, y0, kPanelHeight - 1);

	const std::uint64_t startUs = time_us_64();
	gFlushStats.callCount++;
	const int nativeX0 = nativeStart(x0);
	const int nativeY0 = nativeStart(y0);
	const int nativeX1 = nativeEndInclusive(x1);
	const int nativeY1 = nativeEndInclusive(y1);
	gFlushStats.pixelCount += (nativeX1 - nativeX0 + 1) * (nativeY1 - nativeY0 + 1);
	beginPixels(nativeX0, nativeY0, nativeX1, nativeY1);
	std::uint16_t *current = gLineA;
	std::uint16_t *next = gLineB;
	const int width = x1 - x0 + 1;
	for (int y = y0; y <= y1; ++y) {
		const std::uint64_t copyStartUs = time_us_64();
		raster(gRasterLine, width, 1, x0, y, user);
		const int preparedWidth = expandRowFromRaster(current, gRasterLine, width);
		gFlushStats.copyUs += static_cast<std::int64_t>(time_us_64() - copyStartUs);
		for (int repeat = 0; repeat < kPanelScale; ++repeat) {
			waitDma();
			flushPreparedLine(current, preparedWidth);
		}
		std::swap(current, next);
	}
	endPixels();
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

void panelClear(std::uint16_t color)
{
	if (!gPanel.ready) return;
	for (std::uint16_t &pixel : gLineA) pixel = color;
	const std::uint64_t startUs = time_us_64();
	gFlushStats.callCount++;
	gFlushStats.pixelCount += kNativePanelWidth * kNativePanelHeight;
	beginPixels(0, 0, kNativePanelWidth - 1, kNativePanelHeight - 1);
	for (int y = 0; y < kNativePanelHeight; ++y) {
		waitDma();
		flushPreparedLine(gLineA, kNativePanelWidth);
	}
	endPixels();
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

PanelFlushStats panelFlushStatsRead()
{
	return gFlushStats;
}

void panelFlushStatsReset()
{
	gFlushStats = {};
}

}  // namespace gea::rp2350
