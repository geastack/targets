// The StickS3's accelerometer is a real BMI270, driven by
// targets/esp32/chip_bindings/imu/bmi270.cpp (wired in via
// GEA_EMBEDDED_TARGET_EXTRA_SOURCES in this board's main/CMakeLists.txt). That
// file provides gea::platform::sensors::Accelerometer.
//
// This board-local file exists only because the shared ePaper component
// definition always compiles a board-local imu_stub.cpp; on the StickS3 it must
// stay empty — defining the Accelerometer symbols here would shadow the MPU6886
// binding, since the stub sorts before it in the static archive.
