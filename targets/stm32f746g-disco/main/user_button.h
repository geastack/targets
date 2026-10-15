#pragma once

#include <cstdint>

// The blue USER button (PI11, reads high while pressed).
namespace gea::stm32::user_button {

void init();
// True once per press, when the level has held for the debounce time.
bool pollPressed(std::uint64_t nowUs);

}  // namespace gea::stm32::user_button
