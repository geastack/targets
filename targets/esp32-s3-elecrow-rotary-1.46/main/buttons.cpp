#include "chip_bindings/input/rotary_encoder.h"
#include "services/buttons.h"

namespace gea::platform::elecrow::buttons {
void startButtonsTask()
{
  gea::platform::esp32::buttons::start();
  gea::platform::esp32::input::startRotaryEncoderTask();
}
}
