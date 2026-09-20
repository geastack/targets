#include "touch.h"
#include "chip_bindings/touch/ft3168_controller.h"

namespace {

gea::platform::esp32::chip_bindings::ft3168::TouchController &touchController() {
  return gea::platform::esp32::chip_bindings::ft3168::TouchController::instance();
}

}  // namespace

void gea::platform::touch::Touchscreen::setObserver(Observer observer) {
  touchController().setObserver(observer);
}

// Strong override of the runtime's weak no-op default: routes the multi-finger
// observer to the FT3168 controller so a second contact (pointerId 1) reaches
// the app (maps pinch-zoom). Without this the runtime keeps the single-pointer
// path and pinch never sees the second finger.
void gea::platform::touch::Touchscreen::setPointerObserver(PointerObserver observer) {
  touchController().setPointerObserver(observer);
}

bool gea::platform::touch::Touchscreen::init() {
  return touchController().begin() == ESP_OK;
}

int gea::platform::touch::Touchscreen::read(int *x, int *y) {
  gea::chips::ft3168::TouchSample sample;
  const bool touching = touchController().read(sample);
  if (x) *x = sample.x;
  if (y) *y = sample.y;
  return touching ? 1 : 0;
}

int gea::platform::touch::Touchscreen::readCached(int *x, int *y) {
  const auto sample = touchController().cached();
  if (x) *x = sample.x;
  if (y) *y = sample.y;
  return sample.touching ? 1 : 0;
}

void gea::platform::touch::Touchscreen::consumeLatestMove(int *x, int *y) {
  touchController().consumeLatestMove(x, y);
}

void gea::platform::touch::Touchscreen::injectEvent(Phase phase, bool touching, int x, int y) {
  touchController().injectEvent(phase, touching, x, y);
}
