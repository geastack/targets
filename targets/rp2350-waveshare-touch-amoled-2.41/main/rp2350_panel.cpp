#include "rp2350_panel.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/xip_cache.h"
#include "pico/stdlib.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace gea::rp2350 {
namespace {

constexpr int kPinCs = 9;
constexpr int kPinSclk = 10;
constexpr int kPinDio0 = 11;
constexpr int kPinReset = 15;
constexpr int kXGap = 16;
constexpr int kDrawRounding = 2;
constexpr float kQspiClockDiv = 1.0f;
// PIO clock the panel bus was tuned at (sys_clk=200MHz, div=1.0 => 200MHz PIO,
// ~100MHz SCLK). The divider is recomputed at init from the live clk_sys so the
// wire speed stays pinned here even when the system is overclocked past 200MHz.
constexpr float kQspiPioTargetHz = 200000000.0f;
constexpr int kMaxDmaChunkRows = 64;
constexpr int kMaxDmaDepth = 2;

struct QspiState {
	PIO pio = pio0;
	uint sm = 0;
	int dma = -1;
	dma_channel_config dmaConfig{};
	bool ready = false;
	int brightness = 100;
	int flushRows = 64;
	int flushDepth = 2;
};

// Async span-flush state. While a flush is in flight the DMA (re-armed for
// span B from its completion IRQ) feeds the PIO, the SM runs with a 32-bit
// autopull threshold, and CS stays asserted; panelSpanFlushComplete() settles
// all of it before any other panel traffic. `spanBWords` is consumed by the
// IRQ handler; `spanActive` covers the whole kick-to-settle window.
volatile bool gSpanActive = false;
volatile std::uint32_t gSpanBWords = 0;
const std::uint8_t *volatile gSpanBSrc = nullptr;
dma_channel_config gSpanWordConfig{};
int gSpanStallCount = 0;

QspiState gQspi;
std::uint16_t gDmaChunk[kMaxDmaDepth][kPanelWidth * kMaxDmaChunkRows];
PanelFlushStats gFlushStats;

#define qspi_4wire_data_wrap_target 0
#define qspi_4wire_data_wrap 1
static const std::uint16_t qspi_4wire_data_program_instructions[] = {
	0x7004,  // out pins, 4 side 0
	0xb842,  // nop side 1
};

static const pio_program qspi_4wire_data_program = {
	.instructions = qspi_4wire_data_program_instructions,
	.length = 2,
	.origin = -1,
};

pio_sm_config qspiProgramDefaultConfig(uint offset)
{
	pio_sm_config config = pio_get_default_sm_config();
	sm_config_set_wrap(&config, offset + qspi_4wire_data_wrap_target, offset + qspi_4wire_data_wrap);
	sm_config_set_sideset(&config, 2, true, false);
	return config;
}

void qspiProgramInit(PIO pio, uint sm, uint offset)
{
	pio_sm_config config = qspiProgramDefaultConfig(offset);
	pio_gpio_init(pio, kPinSclk);
	pio_sm_set_consecutive_pindirs(pio, sm, kPinSclk, 1, true);
	sm_config_set_sideset_pins(&config, kPinSclk);
	sm_config_set_out_pins(&config, kPinDio0, 4);
	sm_config_set_out_shift(&config, false, true, 8);
	for (uint pin = 0; pin < 4; ++pin) {
		pio_gpio_init(pio, kPinDio0 + pin);
	}
	pio_sm_set_consecutive_pindirs(pio, sm, kPinDio0, 4, true);
	// Keep the panel bus at its tuned speed independent of the system overclock.
	const float clkDiv = static_cast<float>(clock_get_hz(clk_sys)) / kQspiPioTargetHz;
	sm_config_set_clkdiv(&config, clkDiv < 1.0f ? 1.0f : clkDiv);
	pio_sm_init(pio, sm, offset, &config);
	pio_sm_clear_fifos(pio, sm);
	pio_sm_set_enabled(pio, sm, true);
}

void select()
{
	gpio_put(kPinCs, 0);
}

void waitQspiIdle()
{
	const std::uint32_t txStallMask = 1u << (PIO_FDEBUG_TXSTALL_LSB + gQspi.sm);
	gQspi.pio->fdebug = txStallMask;
	while (!(gQspi.pio->fdebug & txStallMask)) {
		tight_loop_contents();
	}
	sleep_us(2);
}

void deselect()
{
	waitQspiIdle();
	gpio_put(kPinCs, 1);
}

void writePacked(std::uint32_t value)
{
	pio_sm_put_blocking(gQspi.pio, gQspi.sm, value << 24);
}

void writeQspiByte(std::uint8_t value)
{
	std::uint8_t packed[4]{};
	for (int i = 0; i < 4; ++i) {
		const std::uint8_t bit0 = (value & (1u << (2 * i))) ? 1u : 0u;
		const std::uint8_t bit1 = (value & (1u << (2 * i + 1))) ? 1u : 0u;
		packed[3 - i] = static_cast<std::uint8_t>(bit0 | (bit1 << 4));
	}
	for (std::uint8_t byte : packed) writePacked(byte);
}

void writeRegister(std::uint8_t address)
{
	writeQspiByte(0x02);
	writeQspiByte(0x00);
	writeQspiByte(address);
	writeQspiByte(0x00);
}

void writePixelRegister(std::uint8_t address)
{
	writeQspiByte(0x32);
	writeQspiByte(0x00);
	writeQspiByte(address);
	writeQspiByte(0x00);
}

void writeCommand(std::uint8_t command)
{
	select();
	writeRegister(command);
	deselect();
}

void writeCommandData(std::uint8_t command, std::uint8_t data)
{
	select();
	writeRegister(command);
	writeQspiByte(data);
	deselect();
}

void setWindow(int x0, int y0, int x1Exclusive, int y1Exclusive)
{
	x0 += kXGap;
	x1Exclusive += kXGap;

	select();
	writeRegister(0x2a);
	writeQspiByte(static_cast<std::uint8_t>(x0 >> 8));
	writeQspiByte(static_cast<std::uint8_t>(x0 & 0xff));
	writeQspiByte(static_cast<std::uint8_t>((x1Exclusive - 1) >> 8));
	writeQspiByte(static_cast<std::uint8_t>((x1Exclusive - 1) & 0xff));
	deselect();

	select();
	writeRegister(0x2b);
	writeQspiByte(static_cast<std::uint8_t>(y0 >> 8));
	writeQspiByte(static_cast<std::uint8_t>(y0 & 0xff));
	writeQspiByte(static_cast<std::uint8_t>((y1Exclusive - 1) >> 8));
	writeQspiByte(static_cast<std::uint8_t>((y1Exclusive - 1) & 0xff));
	deselect();
}

int roundDownWindowCoord(int value)
{
	return value & ~(kDrawRounding - 1);
}

int roundUpWindowCoord(int value)
{
	return (value + kDrawRounding - 1) & ~(kDrawRounding - 1);
}

void roundMemoryWindow(int *x0, int *y0, int *x1, int *y1)
{
	static_assert((kDrawRounding & (kDrawRounding - 1)) == 0, "kDrawRounding must be a power of two");
	if (!x0 || !y0 || !x1 || !y1) return;
	*x0 = std::max(0, roundDownWindowCoord(*x0));
	*y0 = std::max(0, roundDownWindowCoord(*y0));
	const int xEnd = std::min(kPanelWidth, roundUpWindowCoord(*x1 + 1));
	const int yEnd = std::min(kPanelHeight, roundUpWindowCoord(*y1 + 1));
	*x1 = xEnd - 1;
	*y1 = yEnd - 1;
}

void resetPanel()
{
	gpio_put(kPinReset, 1);
	sleep_ms(50);
	gpio_put(kPinReset, 0);
	sleep_ms(50);
	gpio_put(kPinReset, 1);
	sleep_ms(300);
}

void initRegisters()
{
	writeCommand(0x11);
	sleep_ms(120);

	select();
	writeRegister(0x44);
	writeQspiByte(0x01);
	writeQspiByte(0xd1);
	deselect();

	writeCommandData(0xfe, 0x20);
	writeCommandData(0x63, 0xff);
	writeCommandData(0x26, 0x0a);
	writeCommandData(0x24, 0x80);
	writeCommandData(0xfe, 0x00);
	writeCommandData(0x3a, 0x55);
	writeCommandData(0xc4, 0x80);
	writeCommandData(0xc2, 0x00);
	sleep_ms(10);
	writeCommandData(0x35, 0x00);
	writeCommandData(0x51, 0x00);
	writeCommand(0x29);
	sleep_ms(10);
	writeCommandData(0x51, 0xff);
	sleep_ms(10);
}

void startDma(const std::uint16_t *buffer, int pixelCount)
{
	gFlushStats.chunkCount++;
	gFlushStats.pixelCount += pixelCount;
	dma_channel_configure(gQspi.dma,
	                      &gQspi.dmaConfig,
	                      &gQspi.pio->txf[gQspi.sm],
	                      reinterpret_cast<const std::uint8_t *>(buffer),
	                      static_cast<std::uint32_t>(pixelCount * 2),
	                      true);
}

void waitDma()
{
	const std::uint64_t startUs = time_us_64();
	while (dma_channel_is_busy(gQspi.dma)) {
		tight_loop_contents();
	}
	gFlushStats.txWaitUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

int prepareChunk(std::uint16_t *dst, const std::uint16_t *pixels, int stridePixels, int x0, int y, int y1, int width)
{
	const std::uint64_t startUs = time_us_64();
	const int rows = std::min(gQspi.flushRows, y1 - y + 1);
	for (int row = 0; row < rows; ++row) {
		std::copy_n(pixels + static_cast<std::size_t>(y + row) * stridePixels + x0,
		            width,
		            dst + static_cast<std::size_t>(row) * width);
	}
	gFlushStats.copyUs += static_cast<std::int64_t>(time_us_64() - startUs);
	return rows;
}

void rasterFramebufferChunk(std::uint16_t *dst, int width, int rows, int x0, int y, void *user)
{
	const auto *source = static_cast<const std::uint16_t *>(user);
	if (!source) return;
	const std::uint64_t startUs = time_us_64();
	for (int row = 0; row < rows; ++row) {
		std::copy_n(source + static_cast<std::size_t>(y + row) * kPanelWidth + x0,
		            width,
		            dst + static_cast<std::size_t>(row) * width);
	}
	gFlushStats.copyUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

void streamPreparedRect(int x0, int y0, int x1, int y1, PanelStreamRasterFn raster, void *user)
{
	if (!raster) return;
	gFlushStats.callCount++;
	const int width = x1 - x0 + 1;
	int y = y0;
	int active = 0;
	int rows = std::min(gQspi.flushRows, y1 - y + 1);
	raster(gDmaChunk[active], width, rows, x0, y, user);
	y += rows;
	const std::uint64_t windowStartUs = time_us_64();
	setWindow(x0, y0, x1 + 1, y1 + 1);
	gFlushStats.setWindowUs += static_cast<std::int64_t>(time_us_64() - windowStartUs);
	select();
	writePixelRegister(0x2c);
	channel_config_set_dreq(&gQspi.dmaConfig, pio_get_dreq(gQspi.pio, gQspi.sm, true));
	startDma(gDmaChunk[active], width * rows);
	if (gQspi.flushDepth <= 1) {
		waitDma();
		while (y <= y1) {
			rows = std::min(gQspi.flushRows, y1 - y + 1);
			raster(gDmaChunk[0], width, rows, x0, y, user);
			startDma(gDmaChunk[0], width * rows);
			waitDma();
			y += rows;
		}
	} else {
		while (y <= y1) {
			const int next = 1 - active;
			const int nextRows = std::min(gQspi.flushRows, y1 - y + 1);
			raster(gDmaChunk[next], width, nextRows, x0, y, user);
			waitDma();
			startDma(gDmaChunk[next], width * nextRows);
			active = next;
			y += nextRows;
		}
		waitDma();
	}
	deselect();
}

}  // namespace

void panelSpanFlushComplete();
void spanDmaIrqHandler();

bool panelInit()
{
	if (gQspi.ready) return true;

	gpio_init(kPinCs);
	gpio_set_dir(kPinCs, GPIO_OUT);
	gpio_put(kPinCs, 1);

	gpio_init(kPinReset);
	gpio_set_dir(kPinReset, GPIO_OUT);

	const uint offset = pio_add_program(gQspi.pio, &qspi_4wire_data_program);
	qspiProgramInit(gQspi.pio, gQspi.sm, offset);

	gQspi.dma = dma_claim_unused_channel(true);
	gQspi.dmaConfig = dma_channel_get_default_config(gQspi.dma);
	channel_config_set_transfer_data_size(&gQspi.dmaConfig, DMA_SIZE_8);
	channel_config_set_read_increment(&gQspi.dmaConfig, true);
	channel_config_set_write_increment(&gQspi.dmaConfig, false);
	channel_config_set_dreq(&gQspi.dmaConfig, pio_get_dreq(gQspi.pio, gQspi.sm, true));

	// Async span flush: the channel's completion IRQ re-arms it for span B
	// (single channel throughout — two channels chained on ONE shared DREQ can
	// lose pacing credits at the handover, which matched the intermittent
	// 100ms+ stalls of the earlier chain_to design).
	irq_add_shared_handler(DMA_IRQ_1, spanDmaIrqHandler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	dma_irqn_set_channel_enabled(1, static_cast<uint>(gQspi.dma), true);
	irq_set_enabled(DMA_IRQ_1, true);

	resetPanel();
	initRegisters();
	gQspi.ready = true;
	panelClear(0x0000);
	return true;
}

void panelSetBrightness(int percent)
{
	panelSpanFlushComplete();
	if (percent < 0) percent = 0;
	if (percent > 100) percent = 100;
	gQspi.brightness = percent;
	if (!gQspi.ready) return;
	const auto value = static_cast<std::uint8_t>((percent * 255) / 100);
	writeCommandData(0x51, value);
}

void panelSetFlushConfig(int rows, int depth)
{
	if (rows > 0) gQspi.flushRows = std::clamp(rows, 1, kMaxDmaChunkRows);
	if (depth > 0) gQspi.flushDepth = std::clamp(depth, 1, kMaxDmaDepth);
}

void panelSetVerticalScroll(int startRow)
{
	panelSpanFlushComplete();
	if (!gQspi.ready) return;
	startRow %= kPanelHeight;
	if (startRow < 0) startRow += kPanelHeight;

	static bool regionDefined = false;
	if (!regionDefined) {
		// VSCRDEF: the whole panel is the scrolling area (TFA=0, VSA=600, BFA=0).
		select();
		writeRegister(0x33);
		writeQspiByte(0x00);
		writeQspiByte(0x00);
		writeQspiByte(static_cast<std::uint8_t>(kPanelHeight >> 8));
		writeQspiByte(static_cast<std::uint8_t>(kPanelHeight & 0xff));
		writeQspiByte(0x00);
		writeQspiByte(0x00);
		deselect();
		regionDefined = true;
	}

	select();
	writeRegister(0x37);
	writeQspiByte(static_cast<std::uint8_t>(startRow >> 8));
	writeQspiByte(static_cast<std::uint8_t>(startRow & 0xff));
	deselect();
}

void panelFlushRect(const std::uint16_t *pixels, int stridePixels, int x0, int y0, int x1, int y1)
{
	panelSpanFlushComplete();
	const std::uint64_t totalStartUs = time_us_64();
	if (!gQspi.ready || !pixels || stridePixels <= 0) return;
	x0 = std::max(0, x0);
	y0 = std::max(0, y0);
	x1 = std::min(kPanelWidth - 1, x1);
	y1 = std::min(kPanelHeight - 1, y1);
	if (x0 > x1 || y0 > y1) return;
	roundMemoryWindow(&x0, &y0, &x1, &y1);

	if (stridePixels == kPanelWidth) {
		streamPreparedRect(x0, y0, x1, y1, rasterFramebufferChunk, const_cast<std::uint16_t *>(pixels));
	} else {
		const int width = x1 - x0 + 1;
		gFlushStats.callCount++;
		const std::uint64_t windowStartUs = time_us_64();
		setWindow(x0, y0, x1 + 1, y1 + 1);
		gFlushStats.setWindowUs += static_cast<std::int64_t>(time_us_64() - windowStartUs);
		select();
		writePixelRegister(0x2c);
		channel_config_set_dreq(&gQspi.dmaConfig, pio_get_dreq(gQspi.pio, gQspi.sm, true));
		if (gQspi.flushDepth <= 1) {
			for (int y = y0; y <= y1;) {
				const int rows = prepareChunk(gDmaChunk[0], pixels, stridePixels, x0, y, y1, width);
				startDma(gDmaChunk[0], width * rows);
				waitDma();
				y += rows;
			}
		} else {
			int y = y0;
			int active = 0;
			int rows = prepareChunk(gDmaChunk[active], pixels, stridePixels, x0, y, y1, width);
			startDma(gDmaChunk[active], width * rows);
			y += rows;
			while (y <= y1) {
				const int next = 1 - active;
				const int nextRows = prepareChunk(gDmaChunk[next], pixels, stridePixels, x0, y, y1, width);
				waitDma();
				startDma(gDmaChunk[next], width * nextRows);
				active = next;
				y += nextRows;
			}
			waitDma();
		}
		deselect();
	}
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - totalStartUs);
}

void setPullThreshold(uint threshold)
{
	pio_sm_set_enabled(gQspi.pio, gQspi.sm, false);
	hw_write_masked(&gQspi.pio->sm[gQspi.sm].shiftctrl,
	                (threshold & 0x1fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB,
	                PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);
	// The OSR shift counter carries across a threshold change: after the byte
	// phase it reads "8 consumed", which under a new threshold of 32 makes the
	// SM clock out 24 stale bits before the first autopull — every pixel then
	// lands 3 bytes late (uniform hue corruption). Restart clears the shift
	// state so the first OUT pulls fresh data. FIFO is drained by the caller.
	pio_sm_restart(gQspi.pio, gQspi.sm);
	pio_sm_set_enabled(gQspi.pio, gQspi.sm, true);
}

// DMA completion IRQ: span A finished — re-arm the SAME channel for span B.
// One channel throughout keeps the PIO DREQ pacing credits coherent (two
// channels handing over on one DREQ can drop a credit at the boundary).
void spanDmaIrqHandler()
{
	if (!dma_irqn_get_channel_status(1, static_cast<uint>(gQspi.dma))) return;
	dma_irqn_acknowledge_channel(1, static_cast<uint>(gQspi.dma));
	if (gSpanBWords > 0) {
		const std::uint8_t *src = gSpanBSrc;
		const std::uint32_t words = gSpanBWords;
		gSpanBWords = 0;
		gSpanBSrc = nullptr;
		dma_channel_configure(gQspi.dma,
		                      &gSpanWordConfig,
		                      &gQspi.pio->txf[gQspi.sm],
		                      src,
		                      words,
		                      true);
	}
}

// Settles a pending asynchronous span flush: waits out the (possibly IRQ
// re-armed) DMA and the PIO FIFO, restores the byte-oriented autopull
// threshold, and raises CS. Must run before any other panel traffic (every
// public panel entry point calls it). A stall past 30ms dumps the DMA/PIO
// state once; past 60ms it aborts the transfer and resyncs so a hardware
// hiccup costs one glitched frame instead of a wedge.
void panelSpanFlushComplete()
{
	if (!gSpanActive) return;
	const std::uint64_t startUs = time_us_64();
	bool dumped = false;
	while (gSpanBWords > 0 || dma_channel_is_busy(gQspi.dma)) {
		const std::uint64_t waitedUs = time_us_64() - startUs;
		if (!dumped && waitedUs > 30000) {
			dumped = true;
			gSpanStallCount++;
			std::printf("gea.rp2350.span_stall n=%d bpend=%u ctrl=%08x count=%u fdebug=%08x flevel=%08x\n",
			            gSpanStallCount,
			            static_cast<unsigned>(gSpanBWords),
			            static_cast<unsigned>(dma_hw->ch[gQspi.dma].ctrl_trig),
			            static_cast<unsigned>(dma_hw->ch[gQspi.dma].transfer_count),
			            static_cast<unsigned>(gQspi.pio->fdebug),
			            static_cast<unsigned>(gQspi.pio->flevel));
			std::fflush(stdout);
		}
		if (waitedUs > 60000) {
			dma_channel_abort(gQspi.dma);
			gSpanBWords = 0;
			gSpanBSrc = nullptr;
			pio_sm_clear_fifos(gQspi.pio, gQspi.sm);
			break;
		}
		tight_loop_contents();
	}
	gFlushStats.txWaitUs += static_cast<std::int64_t>(time_us_64() - startUs);
	// Drain the word burst, then restore the byte threshold for commands.
	waitQspiIdle();
	setPullThreshold(8);
	gpio_put(kPinCs, 1);
	gSpanActive = false;
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - startUs);
}

void panelFlushFullWidthSpans(const std::uint16_t *span0, int rows0,
                              const std::uint16_t *span1, int rows1,
                              int gramY0)
{
	const std::uint64_t totalStartUs = time_us_64();
	if (!gQspi.ready) return;
	if (rows0 <= 0 && rows1 <= 0) return;
	if (rows0 <= 0) {
		span0 = span1;
		rows0 = rows1;
		span1 = nullptr;
		rows1 = 0;
	}
	if (!span0) return;

	const int totalRows = rows0 + (span1 ? rows1 : 0);
	if (gramY0 < 0 || gramY0 + totalRows > kPanelHeight) return;

	panelSpanFlushComplete();

	// Commit pending framebuffer writes, then DMA 32-bit words from the
	// uncached PSRAM alias: word transactions run ~4x fewer QMI round-trips
	// than bytes (uncached bytes measured ~15MB/s; cached bytes ~35MB/s but
	// thrash the 16KB XIP cache that code executes through). The DMA
	// byte-swaps each word so the wire order matches the byte stream, and the
	// PIO autopull threshold is widened to 32 for the burst. The kick returns
	// without waiting: span B is re-armed from the completion IRQ and the
	// settle happens lazily at the next panel op, so the CPU builds the next
	// frame while the QSPI drains. Note the cache-clean here also snapshots
	// the framebuffer for the in-flight TX: CPU writes during the overlap sit
	// in the (write-back) XIP cache and only reach raw PSRAM on eviction, so
	// the visible tear window is limited to evicted lines.
	xip_cache_clean_all();
	auto uncachedAlias = [](const std::uint16_t *p) -> const std::uint16_t * {
		const auto addr = reinterpret_cast<uintptr_t>(p);
		if (addr >= 0x11000000u && addr < 0x12000000u) {
			return reinterpret_cast<const std::uint16_t *>(addr + 0x04000000u);
		}
		return p;
	};

	gFlushStats.callCount++;
	const std::uint64_t windowStartUs = time_us_64();
	setWindow(0, gramY0, kPanelWidth, gramY0 + totalRows);
	gFlushStats.setWindowUs += static_cast<std::int64_t>(time_us_64() - windowStartUs);
	select();
	writePixelRegister(0x2c);
	// Drain the command bytes, then widen autopull to a full word (threshold
	// 32 encodes as 0) so 32-bit FIFO pushes shift out completely.
	waitQspiIdle();
	setPullThreshold(0);

	gSpanWordConfig = gQspi.dmaConfig;
	channel_config_set_transfer_data_size(&gSpanWordConfig, DMA_SIZE_32);
	channel_config_set_bswap(&gSpanWordConfig, true);
	channel_config_set_dreq(&gSpanWordConfig, pio_get_dreq(gQspi.pio, gQspi.sm, true));

	gFlushStats.chunkCount++;
	gFlushStats.pixelCount += kPanelWidth * totalRows;
	// Arm span B for the completion IRQ BEFORE triggering span A.
	if (span1 && rows1 > 0) {
		gSpanBSrc = reinterpret_cast<const std::uint8_t *>(uncachedAlias(span1));
		gSpanBWords = static_cast<std::uint32_t>((kPanelWidth * rows1) / 2);
	} else {
		gSpanBSrc = nullptr;
		gSpanBWords = 0;
	}
	gSpanActive = true;
	dma_channel_configure(gQspi.dma,
	                      &gSpanWordConfig,
	                      &gQspi.pio->txf[gQspi.sm],
	                      reinterpret_cast<const std::uint8_t *>(uncachedAlias(span0)),
	                      static_cast<std::uint32_t>((kPanelWidth * rows0) / 2),
	                      true);
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - totalStartUs);
}

void panelStreamRect(int x0, int y0, int x1, int y1, PanelStreamRasterFn raster, void *user)
{
	panelSpanFlushComplete();
	const std::uint64_t totalStartUs = time_us_64();
	if (!gQspi.ready || !raster) return;
	x0 = std::max(0, x0);
	y0 = std::max(0, y0);
	x1 = std::min(kPanelWidth - 1, x1);
	y1 = std::min(kPanelHeight - 1, y1);
	if (x0 > x1 || y0 > y1) return;
	roundMemoryWindow(&x0, &y0, &x1, &y1);
	streamPreparedRect(x0, y0, x1, y1, raster, user);
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - totalStartUs);
}

void panelClear(std::uint16_t color)
{
	panelSpanFlushComplete();
	const std::uint64_t totalStartUs = time_us_64();
	static std::uint16_t row[kPanelWidth];
	std::fill_n(row, kPanelWidth, color);
	gFlushStats.callCount++;
	const std::uint64_t windowStartUs = time_us_64();
	setWindow(0, 0, kPanelWidth, kPanelHeight);
	gFlushStats.setWindowUs += static_cast<std::int64_t>(time_us_64() - windowStartUs);
	select();
	writePixelRegister(0x2c);
	channel_config_set_dreq(&gQspi.dmaConfig, pio_get_dreq(gQspi.pio, gQspi.sm, true));
	for (int y = 0; y < kPanelHeight; ++y) {
		startDma(row, kPanelWidth);
		waitDma();
	}
	deselect();
	gFlushStats.totalUs += static_cast<std::int64_t>(time_us_64() - totalStartUs);
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
