#include "frame_rate.h"

namespace gea::stm32::frame_rate {

namespace {

constexpr std::uint64_t kWindowUs = 1'000'000;

std::uint64_t gWindowStartUs = 0;
int gFrames = 0;
int gFps = 0;

}  // namespace

void tick(std::uint64_t nowUs) {
	++gFrames;
	if (nowUs - gWindowStartUs < kWindowUs) return;

	gFps = gFrames;
	gFrames = 0;
	gWindowStartUs = nowUs;
}

int fps() { return gFps; }

}  // namespace gea::stm32::frame_rate
