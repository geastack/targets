// The StickS3 has no touchscreen. This satisfies the platform Touchscreen
// seam with a hardware-free implementation that still honors injectEvent, so
// GEADEV debug taps/drags (services/device_control) drive apps exactly like a
// real finger would on a touch board.
#include "touch.h"

#include <atomic>

namespace {

struct TouchSample {
	bool touching = false;
	int x = 0;
	int y = 0;
};

gea::platform::touch::Touchscreen::Observer g_observer = nullptr;
TouchSample g_current{};
TouchSample g_latestMove{};
std::atomic<bool> g_latestMoveQueued{false};

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer)
{
	g_observer = observer;
}

bool gea::platform::touch::Touchscreen::init()
{
	return true;
}

int gea::platform::touch::Touchscreen::read(int *x, int *y)
{
	if (x) *x = g_current.x;
	if (y) *y = g_current.y;
	return g_current.touching ? 1 : 0;
}

int gea::platform::touch::Touchscreen::readCached(int *x, int *y)
{
	return read(x, y);
}

void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y)
{
	if (g_latestMoveQueued.exchange(false, std::memory_order_acq_rel)) {
		if (x) *x = g_latestMove.x;
		if (y) *y = g_latestMove.y;
		return;
	}
	if (x) *x = -1;
	if (y) *y = -1;
}

void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y)
{
	g_current = {touching, x, y};
	if (phase == Phase::Move) {
		g_latestMove = g_current;
		g_latestMoveQueued.store(true, std::memory_order_release);
	}
	if (g_observer) g_observer(phase, touching, x, y);
}
