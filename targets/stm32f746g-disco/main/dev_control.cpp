#include "dev_control.h"

#include "console.h"
#include "display.h"
#include "frame_rate.h"
#include "input.h"
#include "lcd.h"
#include "memory.h"
#include "rtc_clock.h"
#include "stm32f7xx.h"
#include "touch.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifndef GEA_STM32_APP_ID
#define GEA_STM32_APP_ID "bringup"
#endif

namespace gea::stm32::dev_control {

namespace {

using gea::platform::touch::Phase;
using gea::platform::touch::Touchscreen;

constexpr std::size_t kLineCapacity = 128;

char gLine[kLineCapacity];
std::size_t gLineLength = 0;
bool gLineOverflow = false;

// A synthetic finger for TAP and DRAG: it walks from (x1, y1) to (x2, y2) in
// `steps` moves, `delayUs` apart, then lifts after `holdUs`.
struct Gesture {
	bool active = false;
	const char *verb = "";
	int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	int steps = 0;
	int step = 0;
	std::uint64_t delayUs = 0;
	std::uint64_t holdUs = 0;
	std::uint64_t nextUs = 0;
};

Gesture gGesture;

void reply(const char *format, ...) __attribute__((format(printf, 1, 2)));

void reply(const char *format, ...) {
	char text[160];
	va_list args;
	va_start(args, format);
	const int length = std::vsnprintf(text, sizeof(text), format, args);
	va_end(args);
	console::write(text, static_cast<std::size_t>(length > 0 ? std::min<int>(length, sizeof(text) - 1) : 0));
	console::write("\n", 1);
}

void startGesture(const char *verb, int x1, int y1, int x2, int y2, int steps, int delayMs, int holdMs, std::uint64_t nowUs) {
	const std::uint64_t delayUs = static_cast<std::uint64_t>(delayMs) * 1000;
	const std::uint64_t holdUs = static_cast<std::uint64_t>(holdMs) * 1000;
	steps = std::max(steps, 0);
	gGesture = {true, verb, x1, y1, x2, y2, steps, 0, delayUs, holdUs, nowUs + (steps > 0 ? delayUs : holdUs)};
	Touchscreen::injectEvent(Phase::Down, true, x1, y1);
}

void advanceGesture(std::uint64_t nowUs) {
	Gesture &g = gGesture;
	if (!g.active || nowUs < g.nextUs) return;

	if (g.step < g.steps) {
		++g.step;
		Touchscreen::injectEvent(Phase::Move, true, g.x1 + (g.x2 - g.x1) * g.step / g.steps, g.y1 + (g.y2 - g.y1) * g.step / g.steps);
		g.nextUs = nowUs + (g.step < g.steps ? g.delayUs : g.holdUs);
		return;
	}

	Touchscreen::injectEvent(Phase::Up, false, g.x2, g.y2);
	g.active = false;
	reply("GEADEV:OK %s x1=%d y1=%d x2=%d y2=%d", g.verb, g.x1, g.y1, g.x2, g.y2);
}

// rgb565-rle-v1: {u16 count, u16 pixel} runs, little-endian, base64 in
// 76-character GEADEV:DATA lines.
class Base64Lines {
public:
	void push(std::uint8_t byte) {
		pending_[pendingLength_++] = byte;
		if (pendingLength_ == sizeof(pending_)) flush();
	}

	void flush() {
		if (!pendingLength_) return;

		static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		char line[12 + 76 + 1] = "GEADEV:DATA ";
		std::size_t out = 12;
		for (std::size_t i = 0; i < pendingLength_; i += 3) {
			const std::uint32_t remaining = static_cast<std::uint32_t>(pendingLength_ - i);
			const std::uint32_t triple = (pending_[i] << 16) | (remaining > 1 ? pending_[i + 1] << 8 : 0) | (remaining > 2 ? pending_[i + 2] : 0);
			line[out++] = kAlphabet[(triple >> 18) & 0x3F];
			line[out++] = kAlphabet[(triple >> 12) & 0x3F];
			line[out++] = remaining > 1 ? kAlphabet[(triple >> 6) & 0x3F] : '=';
			line[out++] = remaining > 2 ? kAlphabet[triple & 0x3F] : '=';
		}
		line[out++] = '\n';
		console::write(line, out);
		pendingLength_ = 0;
	}

private:
	std::uint8_t pending_[57] = {};
	std::size_t pendingLength_ = 0;
};

void pushRun(Base64Lines &lines, std::uint16_t count, std::uint16_t pixel) {
	for (std::uint16_t value : {count, pixel}) {
		lines.push(static_cast<std::uint8_t>(value & 0xFF));
		lines.push(static_cast<std::uint8_t>(value >> 8));
	}
}

// The back buffer holds the last complete frame after every flush.
void sendScreenshot() {
	reply("GEADEV:SCREENSHOT BEGIN width=%d height=%d encoding=rgb565-rle-v1 app=%s", lcd::kWidth, lcd::kHeight, GEA_STM32_APP_ID);

	const std::uint16_t *pixels = lcd::backBuffer();
	const std::size_t count = static_cast<std::size_t>(lcd::kWidth) * lcd::kHeight;
	Base64Lines lines;
	std::uint16_t run = 1;
	for (std::size_t i = 1; i <= count; ++i) {
		if (i < count && pixels[i] == pixels[i - 1] && run < 0xFFFF) {
			++run;
			continue;
		}

		pushRun(lines, run, pixels[i - 1]);
		run = 1;
	}
	lines.flush();
	reply("GEADEV:SCREENSHOT END");
}

void setTime(const char *args) {
	char *end = nullptr;
	const long long seconds = std::strtoll(args, &end, 10);
	if (end == args) {
		reply("GEADEV:ERR SETTIME usage=SETTIME <epochSeconds>");
		return;
	}

	if (!rtc_clock::setEpochSeconds(seconds)) {
		reply("GEADEV:ERR SETTIME rtc-unavailable");
		return;
	}

	reply("GEADEV:OK SETTIME epoch=%lld readback=%lld", seconds, static_cast<long long>(rtc_clock::epochMicros() / 1'000'000));
}

void brightness(const char *args) {
	if (*args) gea::platform::display::Display::setBrightness(std::atoi(args));

	reply("GEADEV:OK BRIGHTNESS value=%d", gea::platform::display::Display::brightness());
}

// Reads up to `capacity` space-separated integers; returns how many it found.
int parseInts(const char *text, int *values, int capacity) {
	int count = 0;
	for (char *end = nullptr; count < capacity; text = end) {
		const long value = std::strtol(text, &end, 10);
		if (end == text) break;

		values[count++] = static_cast<int>(value);
	}
	return count;
}

void handle(const char *command, std::uint64_t nowUs) {
	const char *space = std::strchr(command, ' ');
	const std::size_t verbLength = space ? static_cast<std::size_t>(space - command) : std::strlen(command);
	const char *args = space ? space + 1 : "";
	auto is = [&](const char *verb) { return std::strlen(verb) == verbLength && std::strncmp(command, verb, verbLength) == 0; };

	int v[6] = {};
	const int n = parseInts(args, v, 6);
	if (is("PING")) {
		reply("GEADEV:PONG app=%s board=stm32f746g-disco", GEA_STM32_APP_ID);
	} else if (is("APP")) {
		reply("GEADEV:APP id=%s", GEA_STM32_APP_ID);
	} else if (is("STATE")) {
		reply("GEADEV:STATE width=%d height=%d brightness=%d fps=%d", lcd::kWidth, lcd::kHeight, gea::platform::display::Display::brightness(),
		      frame_rate::fps());
	} else if (is("MEM")) {
		reply("GEADEV:MEM internal_free=0 psram_free=%lu", static_cast<unsigned long>(gea::platform::memory::Memory::psramFree()));
	} else if (is("SETTIME")) {
		setTime(args);
	} else if (is("KEY") && n == 1) {
		gea::framework::input::queueKeyDown(v[0]);
		reply("GEADEV:OK KEY keycode=%d", v[0]);
	} else if (is("BACK")) {
		gea::framework::input::pressBackButton();
		reply("GEADEV:OK BACK returned=0");
	} else if (is("BRIGHTNESS")) {
		brightness(args);
	} else if (is("TAP") && n >= 2) {
		startGesture("TAP", v[0], v[1], v[0], v[1], 0, 0, v[2] > 0 ? v[2] : 80, nowUs);
	} else if (is("DRAG") && n >= 4) {
		startGesture("DRAG", v[0], v[1], v[2], v[3], v[4] > 0 ? v[4] : 6, v[5] > 0 ? v[5] : 24, 0, nowUs);
	} else if (is("SCREENSHOT")) {
		sendScreenshot();
	} else if (is("REBOOT")) {
		reply("GEADEV:OK REBOOT");
		NVIC_SystemReset();
	} else {
		reply("GEADEV:ERR unknown-command");
	}
}

void handleLine(std::uint64_t nowUs) {
	gLine[gLineLength] = '\0';
	const bool overflowed = gLineOverflow;
	gLineLength = 0;
	gLineOverflow = false;

	constexpr char kPrefix[] = "GEADEV ";
	if (overflowed || std::strncmp(gLine, kPrefix, sizeof(kPrefix) - 1) != 0) return;

	handle(gLine + sizeof(kPrefix) - 1, nowUs);
}

}  // namespace

void poll(std::uint64_t nowUs) {
	char bytes[64];
	for (std::size_t count = console::read(bytes, sizeof(bytes)); count; count = console::read(bytes, sizeof(bytes))) {
		for (std::size_t i = 0; i < count; ++i) {
			const char byte = bytes[i];
			if (byte == '\r') continue;

			if (byte == '\n') {
				handleLine(nowUs);
			} else if (gLineLength < kLineCapacity - 1) {
				gLine[gLineLength++] = byte;
			} else {
				gLineOverflow = true;
			}
		}
	}

	advanceGesture(nowUs);
}

}  // namespace gea::stm32::dev_control
