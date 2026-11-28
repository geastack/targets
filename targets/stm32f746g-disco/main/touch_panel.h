#pragma once

#include "touch/ft5336/ft5336.h"

// FT5336 capacitive touch, reported in panel pixels.
namespace gea::stm32::touch_panel {

// id follows a finger for as long as it stays down.
struct Sample {
	int count = 0;
	int id[chips::ft5336::kMaxTouchPoints] = {};
	int x[chips::ft5336::kMaxTouchPoints] = {};
	int y[chips::ft5336::kMaxTouchPoints] = {};
};

bool init();
// False when nothing touches the panel or the controller did not answer.
bool read(Sample &sample);

}  // namespace gea::stm32::touch_panel
