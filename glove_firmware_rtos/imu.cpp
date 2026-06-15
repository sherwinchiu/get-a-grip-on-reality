// =============================================================================
//  imu.cpp  --  IIM-42652 6-axis IMU with complementary-filter sensor fusion
// =============================================================================
//
//  WHAT "SENSOR FUSION" MEANS HERE
//  -------------------------------
//  An accelerometer and a gyroscope each measure orientation badly on their own:
//
//    * Accelerometer  -> senses the gravity vector, so it gives an ABSOLUTE tilt
//                        (roll/pitch) with NO long-term drift. BUT it also picks
//                        up every hand movement as acceleration, so it's noisy
//                        and useless during motion. It cannot sense yaw.
//
//    * Gyroscope      -> measures angular RATE. Integrate it and you get a very
//                        SMOOTH angle that tracks fast motion perfectly. BUT the
//                        tiny zero-rate error integrates into unbounded DRIFT.
//
//  A COMPLEMENTARY FILTER fuses them: trust the gyro over the short term and let
//  the accelerometer slowly pull the estimate back toward "true" gravity. One
//  line does it:
//
//      angle = ALPHA*(angle + gyro_rate*dt) + (1-ALPHA)*accel_angle
//              \__ high-pass the gyro __/    \__ low-pass the accel __/
//
//  This runs in its own FreeRTOS task at a fixed 100 Hz (see IMU_PERIOD_MS), so
//  it is genuinely "real-time IMU sensor fusion" running alongside the BLE and
//  servo tasks without any of them blocking each other.
// =============================================================================
#include "imu.hpp"
#include "config.h"

#ifdef ENABLE_IMU
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "RAK12033-IIM42652.h"
#include "rtos_common.hpp"   // imuMailbox
#include "logger.hpp"
#include "protocol_logic.h"  // pure, unit-tested tilt + complementary-filter math

static IIM42652 IMU;

// Measured at boot while the glove is still; subtracted from every gyro reading.
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// Sensor full-scale factors (datasheet). These match the original firmware.
static const float ACC_LSB_PER_G   = 2048.0f;  // +/-16 g  range
static const float GYR_LSB_PER_DPS  = 16.4f;    // +/-2000 deg/s range
static const float RAD2DEG          = 57.2957795f;

bool initImu(void) {
    pinMode(41, OUTPUT);
    digitalWrite(41, LOW);
    Wire.begin(40, 39);                 // SDA=40, SCL=39

    // Try a BOUNDED number of times instead of forever. If the IMU isn't wired
    // up or the library can't talk to it, we give up and return false so boot
    // continues -- the glove still streams finger data and drives the servos.
    const int MAX_TRIES = 5;
    bool found = false;
    for (int i = 0; i < MAX_TRIES; ++i) {
        if (IMU.begin(Wire, 0x68)) { found = true; break; }
        logf("[IMU] IIM-42652 not found (try %d/%d)...\n", i + 1, MAX_TRIES);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!found) {
        logf("[IMU] giving up -- continuing WITHOUT IMU\n");
        return false;
    }
    IMU.ex_idle();
    IMU.accelerometer_enable();
    IMU.gyroscope_enable();
    IMU.temperature_enable();
    logf("[IMU] connected, calibrating gyro bias -- HOLD STILL...\n");

    // --- Gyro bias calibration ---------------------------------------------
    // Average many readings while stationary. Whatever the gyro reports now is
    // its zero-rate offset; we subtract it later so a still hand reads 0 deg/s.
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < IMU_BIAS_SAMPLES; ++i) {
        IIM42652_axis_t g;
        IMU.get_gyro_data(&g);
        sx += g.x; sy += g.y; sz += g.z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_bias_x = (float)(sx / IMU_BIAS_SAMPLES);
    gyro_bias_y = (float)(sy / IMU_BIAS_SAMPLES);
    gyro_bias_z = (float)(sz / IMU_BIAS_SAMPLES);
    logf("[IMU] bias done (x=%.1f y=%.1f z=%.1f LSB)\n",
         gyro_bias_x, gyro_bias_y, gyro_bias_z);
    return true;
}

bool imu_sample(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
    IIM42652_axis_t a, g;
    if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);   // share the bus with the charger
    IMU.get_accel_data(&a);
    IMU.get_gyro_data(&g);
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    *ax = a.x / ACC_LSB_PER_G;  *ay = a.y / ACC_LSB_PER_G;  *az = a.z / ACC_LSB_PER_G;
    *gx = (g.x - gyro_bias_x) / GYR_LSB_PER_DPS;
    *gy = (g.y - gyro_bias_y) / GYR_LSB_PER_DPS;
    *gz = (g.z - gyro_bias_z) / GYR_LSB_PER_DPS;
    return true;
}

void imuTask(void* pvParameters) {
    Orientation att = {0.0f, 0.0f, 0.0f};   // running attitude estimate (deg)

    // Fixed timestep: because we pace the loop with vTaskDelayUntil, every
    // iteration is IMU_PERIOD_MS apart, so dt is constant and known.
    const float dt = IMU_PERIOD_MS / 1000.0f;

    // Seed roll/pitch from the accelerometer once so we don't start at 0,0 and
    // slowly converge -- we begin already aligned with gravity.
    {
        IIM42652_axis_t a;
        // Take the shared I2C bus mutex around the transaction (charger shares it).
        if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
        IMU.get_accel_data(&a);
        if (i2cMutex) xSemaphoreGive(i2cMutex);
        float ax = a.x / ACC_LSB_PER_G, ay = a.y / ACC_LSB_PER_G, az = a.z / ACC_LSB_PER_G;
        att.roll  = glove::accel_roll_deg(ax, ay, az);
        att.pitch = glove::accel_pitch_deg(ax, ay, az);
    }

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_PERIOD_MS);
    uint32_t tick = 0;

    for (;;) {
        IIM42652_axis_t a, g;
        // One lock for both reads -- keeps the accel/gyro sample pair close in
        // time and prevents the charger task from interleaving on the bus.
        if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
        IMU.get_accel_data(&a);
        IMU.get_gyro_data(&g);
        if (i2cMutex) xSemaphoreGive(i2cMutex);

        // --- Scale to physical units ---------------------------------------
        float ax = a.x / ACC_LSB_PER_G;          // g
        float ay = a.y / ACC_LSB_PER_G;
        float az = a.z / ACC_LSB_PER_G;
        float gx = (g.x - gyro_bias_x) / GYR_LSB_PER_DPS;  // deg/s (bias removed)
        float gy = (g.y - gyro_bias_y) / GYR_LSB_PER_DPS;
        float gz = (g.z - gyro_bias_z) / GYR_LSB_PER_DPS;

        // --- Absolute tilt from gravity (accelerometer) --------------------
        float accel_roll  = glove::accel_roll_deg(ax, ay, az);
        float accel_pitch = glove::accel_pitch_deg(ax, ay, az);

        // --- Complementary filter: fuse gyro integration with accel --------
        att.roll  = glove::comp_filter(att.roll,  gx, dt, accel_roll,  IMU_COMP_ALPHA);
        att.pitch = glove::comp_filter(att.pitch, gy, dt, accel_pitch, IMU_COMP_ALPHA);

        // Yaw has no absolute reference (no magnetometer), so it's pure gyro
        // integration -- accurate short-term, but will slowly drift.
        att.yaw += gz * dt;

        // --- Publish the fused orientation for other tasks -----------------
        xQueueOverwrite(imuMailbox, &att);

        // Log ~5x/sec (every 20th iteration at 100 Hz) so we don't flood Serial.
        if (++tick % 20 == 0) {
            logf("[IMU] roll=%6.1f  pitch=%6.1f  yaw=%6.1f [deg]\n",
                 att.roll, att.pitch, att.yaw);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

#else  // ENABLE_IMU not defined: provide empty stubs so links still resolve.

bool initImu(void) { return false; }   // no IMU compiled in
void imuTask(void* pvParameters) { vTaskDelete(NULL); }
bool imu_sample(float*, float*, float*, float*, float*, float*) { return false; }

#endif // ENABLE_IMU
