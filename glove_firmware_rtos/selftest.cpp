// =============================================================================
//  selftest.cpp  --  Power-On Self-Test implementation
// =============================================================================
#include "selftest.hpp"

#include <Arduino.h>
#include <stdarg.h>
#include "config.h"
#include "logger.hpp"
#include "bluetooth.hpp"
#include "hall.hpp"
#include "servo.hpp"
#include "imu.hpp"
#include "bq25887.hpp"
#include "rtos_common.hpp"
#include "protocol_logic.h"   // pure pass/fail thresholds (unit-tested)

// Report one line to BOTH Serial (mutex-guarded logf) and the connected BLE
// client (as a text frame). A small delay paces the BLE notifications so we
// don't congest the stack during the burst of report lines.
static void report(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    logf("%s", buf);
    bleNotifyText(buf);
    vTaskDelay(pdMS_TO_TICKS(12));   // BLE pacing (safe: runs in setup context)
}

SelfTestSummary run_selftest(bool imuOk, bool chgOk) {
    SelfTestSummary s = { 0, 0, 0 };
    const char* fname[NUM_HALL_ROWS] = { "thumb", "index", "middle", "ring", "pinky" };

    logf("\n[POST] ===== POWER-ON SELF-TEST =====\n");
    logf("[POST] waiting up to %lus for a BLE client...\n",
         (unsigned long)(SELFTEST_BLE_WAIT_MS / 1000));

    // ---- 1. BLE link --------------------------------------------------------
    bool ble = bleWaitConnected(SELFTEST_BLE_WAIT_MS);
    if (ble) {
        report("[POST] BLE   : client connected             OK\n");
        s.passed++;
    } else {
        // No client isn't a firmware fault (maybe a USB-only bench run) -> SKIP.
        report("[POST] BLE   : no client (Serial-only)      SKIP\n");
        s.skipped++;
    }

    // ---- 2. Hall sensors (ADC + driver) ------------------------------------
    uint16_t raw[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];
    hall_sample_raw(raw);
    int hallBad = 0;
    for (int f = 0; f < NUM_HALL_ROWS; ++f) {
        report("[POST] HALL  : %-6s base=%4u knuckle=%4u splay=%4u\n",
               fname[f], raw[f][0], raw[f][1], raw[f][2]);
        for (int j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
            if (f == IGNORE_SPLAY_FINGER && j == 2) continue;       // index splay not wired
            if (!glove::hall_channel_healthy(raw[f][j])) hallBad++;
        }
    }
    if (hallBad == 0) { report("[POST] HALL  : all channels in range        OK\n");   s.passed++; }
    else              { report("[POST] HALL  : %d channel(s) railed         FAIL\n", hallBad); s.failed++; }

    // ---- 3. IMU (I2C + driver + data sanity) -------------------------------
    if (imuOk) {
        float ax, ay, az, gx, gy, gz;
        if (imu_sample(&ax, &ay, &az, &gx, &gy, &gz)) {
            double mag = glove::imu_accel_magnitude_g(ax, ay, az);
            report("[POST] IMU   : |g|=%.2f  gyro=(%.0f,%.0f,%.0f)dps\n", mag, gx, gy, gz);
            if (glove::imu_accel_ok(ax, ay, az)) { report("[POST] IMU   : gravity vector sane          OK\n"); s.passed++; }
            else                                 { report("[POST] IMU   : |g| off 1.0 (moving?)        FAIL\n"); s.failed++; }
        } else { report("[POST] IMU   : sample failed                FAIL\n"); s.failed++; }
    } else { report("[POST] IMU   : not detected                 SKIP\n"); s.skipped++; }

    // ---- 4. Charger / BMS (I2C + driver) -----------------------------------
    if (chgOk) {
        ChargerStatus st;
        if (bq_read_status(&st)) {
            report("[POST] BQ    : pack=%umV (top=%u bot=%u) batt=%u%%%s\n",
                   st.vbat_mv, st.vcell_top_mv, st.vcell_bot_mv, st.battery_pct,
                   st.fault ? " FAULT" : "");
            if (!st.fault && glove::pack_voltage_plausible(st.vbat_mv)) {
                report("[POST] BQ    : status nominal              OK\n");   s.passed++;
            } else {
                report("[POST] BQ    : fault / implausible V       FAIL\n"); s.failed++;
            }
        } else { report("[POST] BQ    : read failed                  FAIL\n"); s.failed++; }
    } else { report("[POST] BQ    : not detected                 SKIP\n"); s.skipped++; }

    // ---- 5. Servos (PWM + driver) ------------------------------------------
    //  Servos have no position feedback, so this is a COMMANDED sweep the user
    //  confirms visually -- each finger flexes then relaxes, one at a time.
    report("[POST] SERVO : sweeping each finger -- WATCH THE GLOVE\n");
    for (int i = 0; i < NUM_SERVO_ROWS; ++i) {
        report("[POST] SERVO : finger %d (%s)...\n", i, fname[i]);
        servos[i].write(0);    vTaskDelay(pdMS_TO_TICKS(250));
        servos[i].write(60);   vTaskDelay(pdMS_TO_TICKS(400));   // flex
        servos[i].write(0);    vTaskDelay(pdMS_TO_TICKS(250));   // relax
    }
    report("[POST] SERVO : sweep done (confirm all 5 moved)  OK*\n");
    s.passed++;   // counted as pass if it ran; the * means "human-confirmed"

    // ---- summary ------------------------------------------------------------
    report("[POST] ------------------------------------\n");
    report("[POST] RESULT: %d passed, %d failed, %d skipped\n", s.passed, s.failed, s.skipped);
    report(s.failed == 0 ? "[POST] >>> ALL CHECKS PASSED <<<\n"
                         : "[POST] >>> SEE FAILURES ABOVE <<<\n");
    report("[POST] ================================\n\n");
    return s;
}
