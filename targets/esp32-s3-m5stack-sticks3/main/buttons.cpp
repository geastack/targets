// The StickS3's face buttons, KEY1 (GPIO11) and KEY2 (GPIO12), are declared in
// targets.json and watched by the shared interrupt-driven button service
// (targets/esp32/services/buttons.cpp). Apps get keydown/keyup events whose
// `code` is 'Key1'/'Key2' and whose key aliases are ArrowUp/ArrowDown, and can
// list or read them through the Buttons API. The side power key belongs to the
// M5PM1 PMIC and no GPIO sees it.
#include "buttons.h"

#include "services/buttons.h"

namespace gea::platform::esp32_s3_sticks3::buttons {

void startButtonsTask() { gea::platform::esp32::buttons::start(); }

}  // namespace gea::platform::esp32_s3_sticks3::buttons
