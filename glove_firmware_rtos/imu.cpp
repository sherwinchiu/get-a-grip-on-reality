// =============================================================================
//  imu.cpp  --  MPU-6050 6-axis IMU (I2C) with complementary-filter fusion
// =============================================================================
//
//  WHAT "SENSOR FUSION" MEANS HERE
//  -------------------------------
//    * Accelerometer -> senses gravity, so it gives ABSOLUTE tilt (roll/pitch)
//      with no long-term drift, but it's noisy under motion and can't see yaw.
//    * Gyroscope     -> measures angular RATE; integrate it for a smooth angle
//      that tracks fast motion, but the zero-rate error integrates into DRIFT.
//
//  A COMPLEMENTARY FILTER fuses them (trust the gyro short-term, let the
//  accelerometer slowly correct drift):
//      angle = ALPHA*(angle + gyro*dt) + (1-ALPHA)*accel_angle
//  The filter math lives in protocol_logic.h (pure + unit-tested), so it's
//  sensor-agnostic -- swapping the IIM-42652 for an MPU-6050 only changes the
//  transport (now raw I2C) and the scale factors below.
//
//  Driven from its own FreeRTOS task at IMU_PERIOD_MS, publishing to imuMailbox.
// =============================================================================
#include "imu.hpp"
#include "config.h"

#ifdef ENABLE_IMU
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "rtos_common.hpp"   // imuMailbox
#include "logger.hpp"
#include "protocol_logic.h"  // pure tilt + complementary-filter math

// ---- MPU-6050 register map --------------------------------------------------
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_WHO_AM_I     = 0x75;   // reads 0x68 on an MPU-6050
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;   // DLPF
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;   // 14 bytes: accel(6) temp(2) gyro(6)

// Accept the whole register-compatible MPU-6xxx/9xxx family by WHO_AM_I, since many
// "MPU-6050" modules actually carry an MPU-6500 (0x70) or similar. They share the
// PWR_MGMT/CONFIG/data registers and ±g/±dps encodings we use here.
//   0x68 MPU-6050   0x70 MPU-6500/6555   0x71 MPU-9250   0x73 MPU-9255
static inline bool is_mpu_whoami(uint8_t id) {
    return id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73;
}

// Full scale we configure below -> the LSB/unit conversions:
//   accel ±4 g  -> 8192 LSB/g     gyro ±500 dps -> 65.5 LSB/dps
//   (headroom for hand motion; ±2 g/±250 dps would clip on a fast flick)
static const float ACC_LSB_PER_G   = 8192.0f;
static const float GYR_LSB_PER_DPS = 65.5f;

// Measured at boot while still; subtracted from every gyro reading.
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

// The IMU shares the I2C bus with the BQ25887 charger, so every transaction must
// hold i2cMutex (else a 100 Hz IMU read can interleave with a charger read and both
// get garbage). The `?` guard makes these no-op the lock before rtos_init() exists
// (boot-time calibration runs before the charger task, so there's no contention yet).
static inline bool i2c_take(void) { return i2cMutex ? (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) : true; }
static inline void i2c_give(void) { if (i2cMutex) xSemaphoreGive(i2cMutex); }

static void wreg(uint8_t reg, uint8_t val) {
    if (!i2c_take()) return;
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
    i2c_give();
}

static uint8_t rreg(uint8_t reg) {
    if (!i2c_take()) return 0;
    uint8_t v = 0;
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)IMU_I2C_ADDR, 1) == 1) v = Wire.read();
    i2c_give();
    return v;
}

// Burst-read accel + gyro as raw signed 16-bit (big-endian on the MPU-6050).
// Reads are done in explicit, sequenced statements (operand evaluation order
// inside one expression is unspecified in C++ -- a classic byte-swap bug).
static bool read_raw(int16_t* ax, int16_t* ay, int16_t* az,
                     int16_t* gx, int16_t* gy, int16_t* gz) {
    if (!i2c_take()) return false;
    bool ok = false;
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)IMU_I2C_ADDR, 14) == 14) {
        uint8_t b[14];
        for (int i = 0; i < 14; ++i) b[i] = Wire.read();
        *ax = (int16_t)((b[0]  << 8) | b[1]);
        *ay = (int16_t)((b[2]  << 8) | b[3]);
        *az = (int16_t)((b[4]  << 8) | b[5]);
        // b[6],b[7] = temperature (skipped)
        *gx = (int16_t)((b[8]  << 8) | b[9]);
        *gy = (int16_t)((b[10] << 8) | b[11]);
        *gz = (int16_t)((b[12] << 8) | b[13]);
        ok = true;
    }
    i2c_give();
    return ok;
}

bool initImu(void) {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);   // 400 kHz fast-mode I2C

    // Probe WHO_AM_I a few times; if absent we return false and the glove runs
    // without orientation (boot is NOT blocked -- tasks already exist).
    const int MAX_TRIES = 5;
    bool found = false;
    for (int i = 0; i < MAX_TRIES; ++i) {
        uint8_t who = rreg(REG_WHO_AM_I);
        if (is_mpu_whoami(who)) { logf("[IMU] found MPU (WHO_AM_I=0x%02X)\n", who); found = true; break; }
        logf("[IMU] MPU not found (try %d/%d) WHO_AM_I=0x%02X on SDA=%d SCL=%d\n",
             i + 1, MAX_TRIES, who, IMU_SDA_PIN, IMU_SCL_PIN);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (!found) { logf("[IMU] giving up -- continuing WITHOUT IMU\n"); return false; }

    // Wake + configure (the MPU-6050 boots ASLEEP).
    wreg(REG_PWR_MGMT_1,  0x01);   // wake; clock source = gyro X PLL (more stable)
    wreg(REG_CONFIG,      0x03);   // DLPF ~44 Hz -> smooths accel/gyro
    wreg(REG_SMPLRT_DIV,  0x04);   // sample rate = 1 kHz / (1+4) = 200 Hz
    wreg(REG_GYRO_CONFIG, 0x08);   // ±500 dps
    wreg(REG_ACCEL_CONFIG,0x08);   // ±4 g
    vTaskDelay(pdMS_TO_TICKS(50));
    logf("[IMU] MPU-6050 connected, calibrating gyro bias -- HOLD STILL...\n");

    // Gyro zero-rate bias: average many still readings, subtract later.
    double sx = 0, sy = 0, sz = 0; int n = 0;
    for (int i = 0; i < IMU_BIAS_SAMPLES; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        if (read_raw(&ax, &ay, &az, &gx, &gy, &gz)) { sx += gx; sy += gy; sz += gz; ++n; }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (n) { gyro_bias_x = sx / n; gyro_bias_y = sy / n; gyro_bias_z = sz / n; }
    logf("[IMU] bias done (x=%.1f y=%.1f z=%.1f LSB)\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
    return true;
}

bool imu_sample(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    if (!read_raw(&rax, &ray, &raz, &rgx, &rgy, &rgz)) return false;
    *ax = rax / ACC_LSB_PER_G;  *ay = ray / ACC_LSB_PER_G;  *az = raz / ACC_LSB_PER_G;
    *gx = (rgx - gyro_bias_x) / GYR_LSB_PER_DPS;
    *gy = (rgy - gyro_bias_y) / GYR_LSB_PER_DPS;
    *gz = (rgz - gyro_bias_z) / GYR_LSB_PER_DPS;
    return true;
}

// One sensor-fusion step: update the attitude in place from a fresh sample.
//   accelerometer -> absolute roll/pitch (drift-free, noisy);
//   gyroscope     -> integrated rate (smooth, drifts);
// the complementary filter (protocol_logic.h) blends them. Yaw is gyro-only (no
// magnetometer, so it drifts). Raw accel is carried along for the host motion effect.
static void fuse_attitude(Orientation* att, float ax, float ay, float az,
                          float gx, float gy, float gz, float dt) {
    float accel_roll  = glove::accel_roll_deg(ax, ay, az);
    float accel_pitch = glove::accel_pitch_deg(ax, ay, az);
    att->roll  = glove::comp_filter(att->roll,  gx, dt, accel_roll,  IMU_COMP_ALPHA);
    att->pitch = glove::comp_filter(att->pitch, gy, dt, accel_pitch, IMU_COMP_ALPHA);
    att->yaw  += gz * dt;                 // no magnetometer -> gyro-only (drifts)
    att->ax = ax; att->ay = ay; att->az = az;   // raw accel (g) -> host motion effect
}

// Seed the initial attitude from one gravity sample so the filter starts ALIGNED
// instead of visibly converging up from zero. Yaw starts at 0 (no absolute reference).
static void seed_attitude(Orientation* att) {
    float ax, ay, az, gx, gy, gz;
    if (imu_sample(&ax, &ay, &az, &gx, &gy, &gz)) {
        att->roll  = glove::accel_roll_deg(ax, ay, az);
        att->pitch = glove::accel_pitch_deg(ax, ay, az);
    }
}

void imuTask(void* pvParameters) {
    Orientation att = {0.0f, 0.0f, 0.0f};
    const float dt = IMU_PERIOD_MS / 1000.0f;
    seed_attitude(&att); // align with gravity sample
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        float ax, ay, az, gx, gy, gz;
        if (imu_sample(&ax, &ay, &az, &gx, &gy, &gz)) {
            fuse_attitude(&att, ax, ay, az, gx, gy, gz, dt);
            xQueueOverwrite(imuMailbox, &att); 
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

#else  // ENABLE_IMU not defined: empty stubs so links still resolve.

#include <Arduino.h>   // FreeRTOS (vTaskDelete) + NULL

bool initImu(void) { return false; }
void imuTask(void* pvParameters) { (void)pvParameters; vTaskDelete(NULL); }
bool imu_sample(float*, float*, float*, float*, float*, float*) { return false; }

#endif // ENABLE_IMU
