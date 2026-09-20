// Accelerometer/gyroscope for a board with no IMU.
//
// Same shape as touch_absent.cpp: host/imu.cpp is part of every board's host
// facade set, so the symbols have to exist even where the hardware does not.
// A board at rest on a desk is what zero acceleration and zero tilt describe,
// so tilt-driven apps behave as if the board is simply being held still rather
// than reading uninitialized values.
//
// Compiled in place of an IMU binding when a composed board selects no
// `chips.imu`.

#include "imu.h"

namespace gea::platform::sensors {

void Accelerometer::init() {}
void Accelerometer::close() {}
void Accelerometer::calibrateBias() {}

int Accelerometer::tiltX() { return 0; }
int Accelerometer::tiltY() { return 0; }

double Accelerometer::accelerationX() { return 0.0; }
double Accelerometer::accelerationY() { return 0.0; }
// Gravity on a board lying flat. Zero on all three axes is free fall, which is
// a reading no stationary board ever produces.
double Accelerometer::accelerationZ() { return 1.0; }

double Accelerometer::gyroscopeX() { return 0.0; }
double Accelerometer::gyroscopeY() { return 0.0; }
double Accelerometer::gyroscopeZ() { return 0.0; }

// The web simulator's tilt injection. There is no cached sample to write into.
void Accelerometer::setWebTilt(int, int) {}

}  // namespace gea::platform::sensors
