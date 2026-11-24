#include "../../esp32-s3-elecrow-rotary-2.1/main/include/rotary_encoder.h"
#include "services/buttons.h"

namespace gea::platform::elecrow::buttons {
void startButtonsTask()
{
  gea::platform::esp32::buttons::start();
  startRotaryEncoderTask();
}
}
