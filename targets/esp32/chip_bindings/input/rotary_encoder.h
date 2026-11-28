#pragma once

// Quadrature rotary encoder on board::rotary.phaseA / phaseB; turns post
// ArrowLeft/ArrowRight key events through the input facade.
namespace gea::platform::esp32::input {

void startRotaryEncoderTask();

}  // namespace gea::platform::esp32::input
