#pragma once

namespace gea::platform::esp32_s3_sticks3::buttons {

// Polls the KEY1/KEY2 face buttons and queues ArrowUp/ArrowDown keydown events
// for the runtime frame loop. Call once from app_main.
void startButtonsTask();

}  // namespace gea::platform::esp32_s3_sticks3::buttons
