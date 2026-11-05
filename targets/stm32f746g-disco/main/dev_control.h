#pragma once

#include <cstdint>

// The GEADEV line protocol on the serial console, as the gea CLI speaks it
// (`gea devctl`, `gea screenshot`): one command per line, replies prefixed
// "GEADEV:". Polled once per frame.
namespace gea::stm32::dev_control {

void poll(std::uint64_t nowUs);

}  // namespace gea::stm32::dev_control
