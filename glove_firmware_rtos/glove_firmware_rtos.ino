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
static TaskHandle_t hBlink = nullptr;

// -----------------------------------------------------------------------------
//  blinkTask : a 1 Hz heartbeat on the GPIO21 LED — a visible "firmware is alive"
//  sign. It runs at the LOWEST priority (0 = idle level) and spends ~all its time
//  blocked in vTaskDelay, so it costs essentially nothing and never competes with
//  the real-time sensor/BLE work. A clean little example of a background task.
// -----------------------------------------------------------------------------
static void blinkTask(void* pvParameters) {
    pinMode(21, OUTPUT);
    bool on = false;
    for (;;) {
        // Status-encoded heartbeat you can read WITHOUT a serial monitor:
        //   slow 1 Hz blink  -> firmware alive, NOT connected (advertising)
        //   fast ~5 Hz blink -> a BLE client is connected (tasks are running!)
        // So: after you connect from the phone, if this LED SPEEDS UP, the
        // firmware sees the link and bleTask is running. If it stays slow, the
        // connection isn't reaching our tasks. If it never blinks at all, the
        // new firmware didn't flash / tasks aren't being created.
        bool conn = bleConnected();
        on = !on;
        digitalWrite(21, on ? HIGH : LOW);
#ifdef RGB_BUILTIN
        // If the dev board has an addressable onboard LED, mirror the state there
        // too (green = connected, amber = idle) for a guaranteed-visible signal.
        rgbLedWrite(RGB_BUILTIN, on ? (conn ? 0 : 18) : 0,
                                 on ? (conn ? 18 : 7) : 0, 0);
#endif
        vTaskDelay(pdMS_TO_TICKS(conn ? 100 : (on ? 120 : 880)));
    }
}

void setup() {
    // ---- 1. Bring up logging (Serial) FIRST so early messages are visible. --
    logInit(115200);
    // Unique build marker: if you DON'T see this exact line after reflashing, the
    // new firmware did not take (old image still running) -- reflash and watch here.
    logf("\n############################################################\n");
    logf("# Glove firmware (FreeRTOS)  build: %s %s\n", __DATE__, __TIME__);
    logf("# tx-diagnostics + MTU247 + 15s-calibration\n");
    logf("############################################################\n");

    // ---- 2. Create the shared RTOS objects BEFORE anything can use them. -----
    //  Order matters: BLE callbacks reference servoQueue/bleEvents, and every
    //  module logs through serialMutex, so these must exist first.
    rtos_init();

    // ---- 3. Initialise hardware peripherals. --------------------------------
    pinMode(21, OUTPUT);   // status LED (was toggled on BLE writes originally)
    init_servos();
    init_hall();

    // Optional servo bring-up sweep (config SERVO_SWEEP_TEST): moves each servo
    // alone so you can see which physically respond, isolating a dead servo/pin
    // from a shared-power brownout. Runs here while single-threaded.
#ifdef SERVO_SWEEP_TEST
    servo_sweep_test();
#endif

    // Optional open/close min-max calibration (see config.h). Runs here, while
    // we're still single-threaded in setup(), so it finishes before hallTask
    // starts publishing. No flag defined -> the hard-coded ranges are used.
#ifdef HALL_CALIBRATE_ON_BOOT
    hall_calibrate(HALL_CALIBRATE_MS);   // run open/close calibration + save to flash
#else
    hall_load_calibration();             // otherwise restore the saved calibration (if any)
#endif

    initBluetooth();

    // ---- 4. Create the CORE streaming tasks FIRST. --------------------------
    //  CRITICAL ORDERING (this was a real bug): the optional IMU/charger init
    //  below touches I2C, and an absent/wedged I2C device can make a sensor
    //  library's begin() BLOCK. If that init runs before we create the tasks,
    //  a hung begin() means hallTask/bleTask/blinkTask are NEVER created and the
    //  glove never streams -- even though BLE still connects (the BLE controller
    //  runs in its own stack task). So we create the real-time tasks NOW; then
    //  nothing the optional peripherals do can stop sensor data from flowing.
    //
    //  xTaskCreatePinnedToCore(fn, "name", stackBytes, arg, priority, &handle, core)
    //  The task runs as soon as the scheduler picks it -- there is no start() step.
    // Per-step logging: whichever "created" line is the LAST one you see pins the
    // exact task whose creation (or immediate first run) is stalling/crashing.
    logf("[BOOT] initBluetooth done; creating core tasks...\n");
    xTaskCreatePinnedToCore(hallTask,  "hall",  STACK_HALL,  nullptr, PRIO_HALL,  &hHall,  CORE_APP);
    logf("[BOOT]  + hall created\n");
    xTaskCreatePinnedToCore(servoTask, "servo", STACK_SERVO, nullptr, PRIO_SERVO, &hServo, CORE_APP);
    logf("[BOOT]  + servo created\n");
    xTaskCreatePinnedToCore(bleTask,   "ble",   STACK_BLE,   nullptr, PRIO_BLE,   &hBle,   CORE_PROTOCOL);
    logf("[BOOT]  + ble created\n");
    // Heartbeat LED on GPIO21 — priority 1 (just above idle) so it's guaranteed
    // to get scheduled; tiny stack. rgbLedWrite (if used) needs a little room.
    xTaskCreatePinnedToCore(blinkTask, "blink", 2048, nullptr, 1, &hBlink, CORE_APP);
    logf("[BOOT]  + blink created\n");
    logf("[BOOT] core tasks created (hall/servo/ble/blink). Free heap: %u bytes\n",
         (unsigned)ESP.getFreeHeap());

    // ---- 5. OPTIONAL peripherals (safe now -- core glove already running). --
    // initImu() returns false if the IMU isn't found; bq_init() likewise. Even
    // if one of these blocks, BLE data is already streaming.
    bool imuOk = initImu();
    g_imuPresent = imuOk;  // tells bleTask whether to append orientation (44B vs 32B)
    bool chgOk = bq_init();

    // POWER-ON SELF-TEST (optional): sweeps every subsystem, reports over BLE.
#ifdef RUN_SELFTEST_ON_BOOT
    run_selftest(imuOk, chgOk);
#endif

    // Spin up the IMU fusion task only if the sensor responded.
    if (imuOk) {
        xTaskCreatePinnedToCore(imuTask, "imu", STACK_IMU, nullptr, PRIO_IMU, &hImu, CORE_APP);
    } else {
        logf("[BOOT] IMU absent -> skipping fusion task (orientation stays 0)\n");
    }

    // Spin up the charger monitor only if the BQ25887 responded.
    if (chgOk) {
        xTaskCreatePinnedToCore(chargerTask, "chg", STACK_CHARGER, nullptr, PRIO_CHARGER, &hChg, CORE_APP);
    } else {
        logf("[BOOT] charger absent -> skipping charger task (battery_lvl stays 0)\n");
    }

    logf("[BOOT] setup() complete -- entering loop()\n");

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
