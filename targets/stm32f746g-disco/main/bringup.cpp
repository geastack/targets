// Hardware bring-up image: no app, just every board subsystem exercised once
// and a touch-paint loop. Flash it with `gea flash --board <alias>` (no --app).

#include "board.h"
#include "bringup_qspi_probe.h"
#include "clock.h"
#include "console.h"
#include "gpio_pins.h"
#include "lcd.h"
#include "rtc_clock.h"
#include "stm32f7xx.h"
#include "system.h"
#include "touch_panel.h"
#include "user_button.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

using namespace gea::stm32;

namespace {

constexpr std::uint16_t kBarColors[] = {0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000};

__attribute__((section(".qspi.rodata"))) const char kQspiProbe[] = "read from QSPI";

const char *verdict(bool ok) {
	return ok ? "ok" : "FAILED";
}

void reportBoot() {
	const BootInfo &boot = bootInfo();
	std::printf("\ngea.stm32 bring-up: STM32F746G-Discovery\n");
	std::printf("  clock 200 MHz: %s\n", verdict(boot.clockReady));
	std::printf("  sdram: %s\n", verdict(boot.sdramReady));
	std::printf("  qspi: %s (jedec %06lx)\n", verdict(boot.qspiReady), static_cast<unsigned long>(boot.qspiJedecId));
}

// Walks a pattern through 1 MB of the heap region through malloc, so the
// SDRAM, the MPU region and _sbrk are all on the path.
bool testSdram() {
	constexpr std::size_t kWords = (1U << 20) / sizeof(std::uint32_t);
	auto *words = static_cast<std::uint32_t *>(std::malloc(kWords * sizeof(std::uint32_t)));
	if (!words) return false;

	for (std::size_t i = 0; i < kWords; ++i) words[i] = static_cast<std::uint32_t>(i * 2654435761U);

	SCB_CleanInvalidateDCache();
	bool ok = true;
	for (std::size_t i = 0; i < kWords && ok; ++i) ok = words[i] == static_cast<std::uint32_t>(i * 2654435761U);

	std::free(words);
	return ok;
}

bool testExceptionsAcrossRegions() {
	try {
		throwFromQspi(1);
	} catch (const std::runtime_error &) {
		return true;
	}
	return false;
}

void drawColorBars() {
	constexpr int kBars = sizeof(kBarColors) / sizeof(kBarColors[0]);
	for (int i = 0; i < kBars; ++i) {
		const int x = i * lcd::kWidth / kBars;
		const int w = (i + 1) * lcd::kWidth / kBars - x;
		lcd::fill(lcd::backBuffer(), x, 0, w, lcd::kHeight, kBarColors[i]);
	}
	lcd::waitForBlanking();
	lcd::present(0, 0, lcd::kWidth, lcd::kHeight);
}

void paint(int cx, int cy) {
	constexpr int kBrush = 6;
	const int x = std::max(0, cx - kBrush);
	const int y = std::max(0, cy - kBrush);
	const int w = std::min(lcd::kWidth, cx + kBrush) - x;
	const int h = std::min(lcd::kHeight, cy + kBrush) - y;

	std::uint16_t *back = lcd::backBuffer();
	for (int row = y; row < y + h; ++row) {
		for (int col = x; col < x + w; ++col) back[row * lcd::kWidth + col] = 0xFFFF;
	}
	lcd::waitForBlanking();
	lcd::present(x, y, w, h);
}

// The USER button toggles the green LED (PI1).
void pollButton(bool &ledOn) {
	if (!user_button::pollPressed(clock::micros())) return;

	ledOn = !ledOn;
	gpio::write(GPIOI, gpio::pins({1}), ledOn);
	std::printf("button: led %s\n", ledOn ? "on" : "off");
}

void echoConsole() {
	char buffer[64];
	const std::size_t count = console::read(buffer, sizeof(buffer));
	if (count) console::write(buffer, count);
}

}  // namespace

int main() {
	clock::init();
	console::init(board::kConsoleBaud);
	reportBoot();

	std::printf("  qspi xip: %s\n", kQspiProbe);
	std::printf("  sdram malloc test: %s\n", verdict(testSdram()));
	std::printf("  c++ exceptions qspi->flash: %s\n", verdict(testExceptionsAcrossRegions()));
	std::printf("  rtc (32.768 kHz crystal): %s, time %s\n", verdict(rtc_clock::init()), rtc_clock::isSet() ? "set" : "not set");

	user_button::init();
	gpio::configure(GPIOI, gpio::pins({1}), {gpio::Mode::Output});
	bool ledOn = false;

	lcd::init();
	drawColorBars();
	std::printf("  lcd: color bars on\n");

	const bool touchReady = touch_panel::init();
	std::printf("  touch ft5336: %s\n", verdict(touchReady));
	std::printf("touch the panel to paint (all fingers), press USER to toggle the LED; typed characters echo back\n");

	for (;;) {
		touch_panel::Sample sample;
		if (touchReady && touch_panel::read(sample)) {
			for (int i = 0; i < sample.count; ++i) paint(sample.x[i], sample.y[i]);
		}

		pollButton(ledOn);
		echoConsole();
	}
}
