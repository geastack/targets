#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "host/media.h"

namespace {

int outputVolume = 50;

}  // namespace

bool gea::platform::audio::OutputDriver::open(int, int, int) { return false; }
bool gea::platform::audio::OutputDriver::write(const std::int16_t *, std::size_t, int) { return false; }
void gea::platform::audio::OutputDriver::close() {}
int gea::platform::audio::OutputDriver::volume() { return outputVolume; }
void gea::platform::audio::OutputDriver::setVolume(int value)
{
	if (value < 0) value = 0;
	if (value > 100) value = 100;
	outputVolume = value;
}

namespace gea::host::media {

void platform_attach_track(NativeMediaTrackHandle) {}
void platform_detach_track(NativeMediaTrackHandle) {}

}  // namespace gea::host::media
