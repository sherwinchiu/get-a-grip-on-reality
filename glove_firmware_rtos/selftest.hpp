// =============================================================================
//  selftest.hpp  --  Power-On Self-Test (POST) / "first firmware go"
// =============================================================================
//
//  A one-shot hardware bring-up check, SEPARATE from the main loop. After a BLE
//  client connects, it exercises every subsystem THROUGH ITS REAL DRIVER and
//  reports PASS/FAIL over both Serial and BLE, then hands off to normal
//  operation. Use it the first time you flash a built glove to confirm the whole
//  stack -- ADC + Hall, I2C + IMU, I2C + charger, PWM + servos, and the BLE link
//  itself -- is wired and working before you trust the live data.
//
//  It runs in setup() while still single-threaded (before the tasks start), so
//  there's no contention on the shared buses and the results are deterministic.
//  Enabled by RUN_SELFTEST_ON_BOOT in config.h.
// =============================================================================
#ifndef SELFTEST_HPP
#define SELFTEST_HPP

struct SelfTestSummary {
    int passed;
    int failed;
    int skipped;
};

// Run the POST. `imuOk`/`chgOk` are the results of initImu()/bq_init() so absent
// optional hardware is reported as SKIPPED, not FAILED. Call AFTER
// initBluetooth/initImu/bq_init and BEFORE creating the streaming tasks.
SelfTestSummary run_selftest(bool imuOk, bool chgOk);

#endif // SELFTEST_HPP
