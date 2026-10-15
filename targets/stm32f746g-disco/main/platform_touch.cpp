// gea's Touchscreen on the FT5336: polled once per frame from the main loop.
// Each finger holds a pointer slot, matched by the controller's touch ID, from
// down to up, and every slot reports Down/Move/Up through the pointer
// observer, so apps get true multi-touch; slot 0 is the engine's primary
// pointer.

#include "touch.h"

#include "touch_panel.h"

#include <array>

namespace {

namespace touch_panel = gea::stm32::touch_panel;
using gea::platform::touch::Phase;
using gea::platform::touch::Touchscreen;

constexpr int kSlots = gea::chips::ft5336::kMaxTouchPoints;
// The id of a finger GEADEV injects: the panel never reports it, so the panel
// poll must not lift it.
constexpr int kInjectedId = -1;

struct Finger {
	bool down = false;
	int id = -1;
	int x = 0;
	int y = 0;
};

Touchscreen::Observer gObserver = nullptr;
Touchscreen::PointerObserver gPointerObserver = nullptr;
std::array<Finger, kSlots> gFingers{};
bool gReady = false;

void notify(int slot, Phase phase, int x, int y) {
	Finger &finger = gFingers[static_cast<std::size_t>(slot)];
	finger.down = phase != Phase::Up;
	finger.x = x;
	finger.y = y;

	if (gPointerObserver) {
		gPointerObserver(phase, finger.down, x, y, slot);
	} else if (gObserver && slot == 0) {
		gObserver(phase, finger.down, x, y);
	}
}

// The finger's slot if it is already down, else the lowest free slot.
int slotFor(int id) {
	int free = -1;
	for (int slot = kSlots - 1; slot >= 0; --slot) {
		const Finger &finger = gFingers[static_cast<std::size_t>(slot)];
		if (finger.down && finger.id == id) return slot;
		if (!finger.down) free = slot;
	}
	return free;
}

void track(const touch_panel::Sample &sample) {
	std::array<bool, kSlots> seen{};
	for (int i = 0; i < sample.count; ++i) {
		const int slot = slotFor(sample.id[i]);
		if (slot < 0) continue;

		seen[static_cast<std::size_t>(slot)] = true;
		Finger &finger = gFingers[static_cast<std::size_t>(slot)];
		if (!finger.down) {
			finger.id = sample.id[i];
			notify(slot, Phase::Down, sample.x[i], sample.y[i]);
		} else if (finger.x != sample.x[i] || finger.y != sample.y[i]) {
			notify(slot, Phase::Move, sample.x[i], sample.y[i]);
		}
	}

	for (int slot = 0; slot < kSlots; ++slot) {
		const Finger &finger = gFingers[static_cast<std::size_t>(slot)];
		if (finger.down && finger.id != kInjectedId && !seen[static_cast<std::size_t>(slot)]) notify(slot, Phase::Up, finger.x, finger.y);
	}
}

}  // namespace

namespace gea::platform::touch {

void Touchscreen::setObserver(Observer observer) { gObserver = observer; }

void Touchscreen::setPointerObserver(PointerObserver observer) { gPointerObserver = observer; }

bool Touchscreen::init() {
	gReady = touch_panel::init();
	return gReady;
}

void Touchscreen::poll(int) {
	if (!gReady) return;

	touch_panel::Sample sample;
	touch_panel::read(sample);
	track(sample);
}

int Touchscreen::read(int *x, int *y) {
	touch_panel::Sample sample;
	const bool touching = gReady && touch_panel::read(sample);
	if (touching) {
		gFingers[0].x = sample.x[0];
		gFingers[0].y = sample.y[0];
	}

	if (x) *x = gFingers[0].x;
	if (y) *y = gFingers[0].y;
	return touching ? 1 : 0;
}

int Touchscreen::readCached(int *x, int *y) {
	if (x) *x = gFingers[0].x;
	if (y) *y = gFingers[0].y;
	return gFingers[0].down ? 1 : 0;
}

void Touchscreen::consumeLatestMove(int *x, int *y) {
	if (x) *x = gFingers[0].x;
	if (y) *y = gFingers[0].y;
}

void Touchscreen::injectEvent(Phase phase, bool, int x, int y) {
	gFingers[0].id = kInjectedId;
	notify(0, phase, x, y);
}

}  // namespace gea::platform::touch
