#pragma once

namespace gea::platform::esp32_s3_sticks3::buttons {

// Starts the shared board-button service for KEY1/KEY2 (declared in
// targets.json). Call once from app_main.
void startButtonsTask();

}  // namespace gea::platform::esp32_s3_sticks3::buttons
