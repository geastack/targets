#pragma once

namespace gea::platform::esp32_s3_epaper::buttons {

// Polls the BOOT/PWR buttons and queues ArrowUp/ArrowDown keydown events for
// the runtime frame loop. Call once from app_main.
void startButtonsTask();

}  // namespace gea::platform::esp32_s3_epaper::buttons
