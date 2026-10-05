#pragma once

namespace gea::platform::board {

// Initialize the PM1/IOE1, enable the AMOLED rail and reset display and touch.
// Hardware failures abort initialization instead of leaving a dark display.
void prepareDisplayPanel();

// Reset CST820 through IOE1 while preserving every other expander output.
void resetTouchPanel();

} // namespace gea::platform::board
