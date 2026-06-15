// =============================================================================
//  imu.hpp  --  IIM-42652 6-axis IMU (optional / currently disabled)
// =============================================================================
//
//  initImu() brings up the IIM-42652 over I2C and calibrates the gyro bias.
//  imuTask() runs a complementary filter at 100 Hz to fuse the accelerometer
//  and gyroscope into a drift-resistant roll/pitch/yaw estimate, publishing it
//  to imuMailbox for any consumer. See imu.cpp for the full explanation.
//
//  Enabled via ENABLE_IMU in config.h (requires the RAK12033-IIM42652 library).
//  When ENABLE_IMU is undefined both functions become harmless no-op stubs, so
//  the firmware still builds without the library or the hardware present.
// =============================================================================
#ifndef IMU_HPP
#define IMU_HPP

// Bring up the IMU and calibrate gyro bias (glove must be held still). setup().
// Returns true if the IMU responded, false if it wasn't found. The caller uses
// this to decide whether to start imuTask -- so a missing/unwired IMU can NEVER
// hang boot or stop the rest of the firmware from running.
bool initImu(void);

// 100 Hz complementary-filter sensor-fusion task. Publishes Orientation.
void imuTask(void* pvParameters);

// One-shot read of accel (g) + gyro (deg/s, bias-removed) for the self-test.
// Returns false if the IMU is compiled out (ENABLE_IMU). Thread-safe (I2C mutex).
bool imu_sample(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);

#endif // IMU_HPP
