#include "imu.h"
#include "power.h"

#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "host/media.h"

namespace gea::platform::power {

bool Power::init() { return true; }
int Power::batteryPercent() { return -1; }

}  // namespace gea::platform::power

namespace gea::platform::sensors {

void Accelerometer::init() {}
void Accelerometer::close() {}
void Accelerometer::calibrateBias() {}
int Accelerometer::tiltX() { return 0; }
int Accelerometer::tiltY() { return 0; }
double Accelerometer::accelerationX() { return 0.0; }
double Accelerometer::accelerationY() { return 0.0; }
double Accelerometer::accelerationZ() { return 1.0; }
double Accelerometer::gyroscopeX() { return 0.0; }
double Accelerometer::gyroscopeY() { return 0.0; }
double Accelerometer::gyroscopeZ() { return 0.0; }
void Accelerometer::setWebTilt(int, int) {}

}  // namespace gea::platform::sensors

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
