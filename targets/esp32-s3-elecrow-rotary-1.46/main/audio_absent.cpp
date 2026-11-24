#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "host/media.h"

namespace gea::platform::audio {
bool OutputDriver::open(int, int, int) { return false; }
bool OutputDriver::write(const std::int16_t *, std::size_t, int) { return false; }
void OutputDriver::close() {}
int OutputDriver::volume() { return 0; }
void OutputDriver::setVolume(int) {}
}

namespace gea::host::media {
void platform_attach_track(NativeMediaTrackHandle) {}
void platform_detach_track(NativeMediaTrackHandle) {}
}
