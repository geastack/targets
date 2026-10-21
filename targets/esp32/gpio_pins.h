#pragma once

// Pins whose interrupt another ESP32 driver owns: the board's buttons
// (services/buttons.cpp). The Gpio and Digital classes can still open such a
// pin as an input. Gpio::configure() then leaves the owner's setup alone, and
// Gpio::watch() rides on the owner's interrupt, which reports each edge
// through ownedPinEdge(). Without this, opening the pin would reset it and
// replace the owner's interrupt handler.
namespace gea::platform::esp32::gpio {

// `interrupt`: the owner takes edge interrupts on the pin (false for a polled
// button, which Gpio then polls too).
void claimPin(int pin, bool interrupt);
// Interrupt context; IRAM-safe.
void ownedPinEdge(int pin);

}  // namespace gea::platform::esp32::gpio
