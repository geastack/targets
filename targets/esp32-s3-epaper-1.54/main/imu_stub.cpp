// The ESP32-S3-ePaper-1.54 has no IMU (its I2C peripherals are the PCF85063
// RTC, SHTC3 temp/humidity sensor, FT6336 touch, and ES8311 codec). Apps that
// read the accelerometer get zeros.
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
