#pragma once

#include <cstdint>

// Frame-loop passes per second, for GEADEV STATE. A pass that overruns the
// frame interval delays the next, so the count drops exactly when rendering
// cannot keep up.
namespace gea::stm32::frame_rate {

// Once per frame-loop pass.
void tick(std::uint64_t nowUs);
// The passes completed in the last full second.
int fps();

}  // namespace gea::stm32::frame_rate
