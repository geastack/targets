// Audio output for a board with no speaker path.
//
// Same shape as power_absent.cpp / imu_absent.cpp: the host audio and media
// facades reference these symbols unconditionally, so a board without an
// output stage still has to provide them. open() failing is the accurate
// answer and keeps callers on their normal "no audio device" path.
#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "host/media.h"

namespace gea::platform::audio {

bool OutputDriver::open(int, int, int) { return false; }
bool OutputDriver::write(const std::int16_t *, std::size_t, int) { return false; }
void OutputDriver::close() {}
int OutputDriver::volume() { return 0; }
void OutputDriver::setVolume(int) {}

}  // namespace gea::platform::audio

namespace gea::host::media {

void platform_attach_track(NativeMediaTrackHandle) {}
void platform_detach_track(NativeMediaTrackHandle) {}

}  // namespace gea::host::media
