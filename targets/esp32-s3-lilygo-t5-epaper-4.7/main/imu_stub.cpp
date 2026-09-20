// M5Paper v1.1 has no IMU. Keep the accelerometer surface deterministic so
// apps can feature-detect through zero readings without a missing platform TU.
#include "imu.h"

namespace gea::platform::sensors {

void Accelerometer::init() {}
void Accelerometer::close() {}
void Accelerometer::calibrateBias() {}
int Accelerometer::tiltX() { return 0; }
int Accelerometer::tiltY() { return 0; }
double Accelerometer::accelerationX() { return 0.0; }
double Accelerometer::accelerationY() { return 0.0; }
double Accelerometer::accelerationZ() { return 0.0; }
double Accelerometer::gyroscopeX() { return 0.0; }
double Accelerometer::gyroscopeY() { return 0.0; }
double Accelerometer::gyroscopeZ() { return 0.0; }
void Accelerometer::setWebTilt(int, int) {}

}  // namespace gea::platform::sensors
