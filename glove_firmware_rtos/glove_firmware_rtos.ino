// =============================================================================
//  glove_firmware_rtos.ino  --  ESP32-S3 haptic glove, FreeRTOS edition
// =============================================================================
//
//  WHAT CHANGED vs THE ORIGINAL
//  ----------------------------
//  Old design: a single loop() called bluetoothTask() and print_hall() in turn,
//  then busy-waited ~30 ms. Everything ran sequentially on one core; a slow BLE
//  operation would stall sensor reads and vice-versa.
//
//  New design: independent FreeRTOS TASKS, each doing one job, communicating
//  through QUEUES, coordinated by an EVENT GROUP, sharing the UART through a
//  MUTEX. The scheduler interleaves them across the ESP32-S3's two cores.
//
//        +-----------+   InputData    +-----------+   BLE notify
//        | hallTask  |==============>>| bleTask   |=============> phone/PC
//        | (core 1)  | sensorMailbox  | (core 0)  |
//        +-----------+   (mailbox)    +-----------+
//                                          ||  BLE write (haptics)
//                                          \/
//                                     +-----------+   ServoCommand
//                                     | RX cb     |================> servoQueue
//                                     +-----------+                     ||
//                                                                       \/
//                                                                  +-----------+
//                                                                  | servoTask |
//                                                                  | (core 1)  |
//                                                                  +-----------+
//
//  See RTOS_GUIDE.md for the full explanation of every primitive.
// =============================================================================

#include "config.h"
#include "rtos_common.hpp"
#include "logger.hpp"
#include "hall.hpp"
#include "servo.hpp"
#include "bluetooth.hpp"
#include "imu.hpp"
#include "bq25887.hpp"
#include "selftest.hpp"

// We keep the task handles so loop() can inspect them (stack usage, etc.).
// A TaskHandle_t is just an opaque pointer the scheduler gives us at creation.
static TaskHandle_t hHall  = nullptr;
static TaskHandle_t hBle   = nullptr;
static TaskHandle_t hServo = nullptr;
static TaskHandle_t hImu   = nullptr;
static TaskHandle_t hChg   = nullptr;

void setup() {
    // ---- 1. Bring up logging (Serial) FIRST so early messages are visible. --
    logInit(115200);
    logf("\n[BOOT] Glove firmware (FreeRTOS) starting\n");

    // ---- 2. Create the shared RTOS objects BEFORE anything can use them. -----
    //  Order matters: BLE callbacks reference servoQueue/bleEvents, and every
    //  module logs through serialMutex, so these must exist first.
    rtos_init();

    // ---- 3. Initialise hardware peripherals. --------------------------------
    pinMode(21, OUTPUT);   // status LED (was toggled on BLE writes originally)
    init_servos();
    init_hall();

    // Optional open/close min-max calibration (see config.h). Runs here, while
    // we're still single-threaded in setup(), so it finishes before hallTask
    // starts publishing. No flag defined -> the hard-coded ranges are used.
#ifdef HALL_CALIBRATE_ON_BOOT
    hall_calibrate(HALL_CALIBRATE_MS);   // run open/close calibration + save to flash
#else
    hall_load_calibration();             // otherwise restore the saved calibration (if any)
#endif

    initBluetooth();
    // initImu() returns false (after a few retries) if the IMU isn't found, so
    // a missing/unwired IMU never hangs boot -- the rest of the glove still runs.
    bool imuOk = initImu();
    g_imuPresent = imuOk;  // tells bleTask whether to append orientation (44B vs 32B)

    // Configure the BQ25887 charger over I2C (shares the IMU bus). Returns false
    // if absent, so a board without it still boots and runs everything else.
    bool chgOk = bq_init();

    // ---- 3b. POWER-ON SELF-TEST (optional) ----------------------------------
    //  Runs here while still single-threaded (no task contention on the buses):
    //  waits for a BLE client, sweeps every subsystem through its driver, and
    //  reports PASS/FAIL over Serial + BLE before normal operation begins.
#ifdef RUN_SELFTEST_ON_BOOT
    run_selftest(imuOk, chgOk);
#endif

    // ---- 4. Create the tasks. -----------------------------------------------
    //  xTaskCreatePinnedToCore(
    //      taskFunction,        // the void(void*) entry point
    //      "name",              // human-readable name (shows in debug tools)
    //      stackBytes,          // private stack size (BYTES on ESP32)
    //      pvParameters,        // arg passed to the task (we don't need one)
    //      priority,            // higher number = higher priority
    //      &handle,             // out: handle to the created task
    //      coreID);             // which physical core to pin it to
    //
    //  The task starts running as soon as it's created and the scheduler picks
    //  it -- there is no "start()" step. (The FreeRTOS scheduler is already
    //  running by the time Arduino calls setup().)

    xTaskCreatePinnedToCore(hallTask,  "hall",  STACK_HALL,  nullptr, PRIO_HALL,  &hHall,  CORE_APP);
    xTaskCreatePinnedToCore(servoTask, "servo", STACK_SERVO, nullptr, PRIO_SERVO, &hServo, CORE_APP);
    xTaskCreatePinnedToCore(bleTask,   "ble",   STACK_BLE,   nullptr, PRIO_BLE,   &hBle,   CORE_PROTOCOL);

    // Only spin up the IMU fusion task if the sensor actually responded.
    if (imuOk) {
        xTaskCreatePinnedToCore(imuTask, "imu", STACK_IMU, nullptr, PRIO_IMU, &hImu, CORE_APP);
    } else {
        logf("[BOOT] IMU absent -> skipping fusion task (orientation stays 0)\n");
    }

    // Only spin up the charger monitor if the BQ25887 responded.
    if (chgOk) {
        xTaskCreatePinnedToCore(chargerTask, "chg", STACK_CHARGER, nullptr, PRIO_CHARGER, &hChg, CORE_APP);
    } else {
        logf("[BOOT] charger absent -> skipping charger task (battery_lvl stays 0)\n");
    }

    logf("[BOOT] tasks created. Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());

    // setup() returns and Arduino's loopTask keeps calling loop() below. We
    // repurpose loop() as a low-priority health monitor.
}

// -----------------------------------------------------------------------------
//  loop() == the Arduino "loopTask" (priority 1, core 1). We don't run any
//  control logic here anymore -- instead it's a handy diagnostics heartbeat.
//
//  uxTaskGetStackHighWaterMark(handle) returns the SMALLEST number of stack
//  WORDS that task has ever had free. If it trends toward 0, that task is about
//  to overflow and you must grow its STACK_* value in config.h. This is the
//  single most useful tool for right-sizing stacks.
// -----------------------------------------------------------------------------
void loop() {
    logf("[diag] heap=%u  stack free (words) hall=%u ble=%u servo=%u imu=%u chg=%u\n",
         (unsigned)ESP.getFreeHeap(),
         (unsigned)uxTaskGetStackHighWaterMark(hHall),
         (unsigned)uxTaskGetStackHighWaterMark(hBle),
         (unsigned)uxTaskGetStackHighWaterMark(hServo),
         hImu ? (unsigned)uxTaskGetStackHighWaterMark(hImu) : 0,
         hChg ? (unsigned)uxTaskGetStackHighWaterMark(hChg) : 0);

    // Sleep this task for 5 s. delay() on Arduino-ESP32 is RTOS-aware: it calls
    // vTaskDelay under the hood, so the CPU is free for real work meanwhile.
    delay(5000);
}
