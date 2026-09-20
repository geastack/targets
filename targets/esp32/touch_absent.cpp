// Touchscreen for a board that has no touch panel.
//
// The same shape as connectivity/wifi_debug_offline.cpp: the framework and the
// host facades reference Touchscreen unconditionally, so a board without the
// hardware still has to provide the symbols. Reporting "nothing is touching"
// forever is the accurate answer, and it keeps every caller — the runtime's
// event loop, the host touch facade, the debug FIFO — on its normal path
// instead of needing a display-less special case.
//
// Compiled in place of a controller binding when a composed board selects no
// `chips.touch`. Input on such a board arrives through buttons (GPIO) or over
// the wire, both of which go through their own facades.

#include "touch.h"

namespace gea::platform::touch {

void Touchscreen::setObserver(Observer) {}
void Touchscreen::setPointerObserver(PointerObserver) {}

// Reporting success keeps boot on the normal path: there is no hardware to
// fail, and an init() that returns false reads as a panel that did not answer.
bool Touchscreen::init() { return true; }

int Touchscreen::read(int *x, int *y)
{
	if (x) *x = 0;
	if (y) *y = 0;
	return 0;
}

int Touchscreen::readCached(int *x, int *y) { return read(x, y); }

void Touchscreen::consumeLatestMove(int *x, int *y)
{
	if (x) *x = 0;
	if (y) *y = 0;
}

// Injection is how the debug FIFO drives synthetic drags. With no observer and
// no cache to update there is nothing to deliver it to.
void Touchscreen::injectEvent(Phase, bool, int, int) {}

}  // namespace gea::platform::touch
