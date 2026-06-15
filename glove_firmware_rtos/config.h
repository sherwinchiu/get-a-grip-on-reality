// =============================================================================
//  config.h  --  Central tuning knobs for the FreeRTOS glove firmware
// =============================================================================
//
//  WHY THIS FILE EXISTS
//  --------------------
//  In the old Arduino version, magic numbers (delays, pin counts, etc.) were
//  scattered across every .cpp file. In an RTOS design you constantly tune four
//  things per task: which CORE it runs on, its PRIORITY, its STACK size, and
//  how OFTEN it runs. Keeping all of those in one place makes the whole system
//  easy to reason about and is exactly the kind of "show me your system design"
//  artifact an interviewer likes to see.
//
// =============================================================================
#ifndef CONFIG_H
#define CONFIG_H

// -----------------------------------------------------------------------------
//  Which hand are we building for? (unchanged from the original firmware)
// -----------------------------------------------------------------------------
//#define LEFT_HAND
#define RIGHT_HAND

// -----------------------------------------------------------------------------
//  CORE ASSIGNMENT
// -----------------------------------------------------------------------------
//  The ESP32-S3 has TWO CPU cores (0 and 1). The Bluetooth controller + host
//  stack is very timing sensitive, so the standard pattern is:
//
//      Core 0  ->  "protocol" core : radio / BLE work
//      Core 1  ->  "application" core : your sensor + control logic
//
//  Pinning a task to a core (xTaskCreatePinnedToCore) guarantees it always runs
//  on that core, which keeps the BLE stack from being starved by our ADC reads
//  and keeps cache behaviour predictable.
// -----------------------------------------------------------------------------
#define CORE_PROTOCOL   0   // BLE notify task lives here
#define CORE_APP        1   // hall sensor + servo + imu tasks live here

// -----------------------------------------------------------------------------
//  TASK PRIORITIES
// -----------------------------------------------------------------------------
//  In FreeRTOS, HIGHER number == HIGHER priority. The scheduler always runs the
//  highest-priority task that is "ready". A task only gives up the CPU when it
//  BLOCKS (waits on a queue/delay/semaphore) or a higher priority task wakes up.
//
//  Reference points:
//      0  = the idle task (runs when nothing else wants the CPU)
//      1  = the Arduino loop() task ("loopTask") default priority
//
//  We make the servo task the most urgent because it delivers HAPTIC feedback:
//  when the VR app says "you touched something", the user should feel it now.
// -----------------------------------------------------------------------------
#define PRIO_SERVO      3   // event-driven, must react fast to BLE commands
#define PRIO_HALL       2   // periodic sensor sampling
#define PRIO_BLE        2   // periodic notify of sensor data
#define PRIO_IMU        1   // optional, lowest urgency
#define PRIO_CHARGER    1   // slow (1 Hz) battery monitor + watchdog service

// -----------------------------------------------------------------------------
//  STACK SIZES (in BYTES)
// -----------------------------------------------------------------------------
//  Every task gets its OWN stack carved out of the heap when it is created.
//  Too small -> stack overflow -> crash/reboot. Too big -> wasted RAM.
//  NOTE: On ESP-IDF / Arduino-ESP32 the stack argument is in BYTES (plain
//  FreeRTOS uses words -- this is a classic gotcha). We print the high-water
//  mark in loop() so you can see how much headroom each task actually has and
//  shrink these later.
// -----------------------------------------------------------------------------
#define STACK_SERVO     3072
#define STACK_HALL      4096   // read_hall() puts a 32x5x3 sample buffer on its stack
#define STACK_BLE       4096   // BLE notify + packet building
#define STACK_IMU       4096   // sensor fusion uses floats / atan2 / logf
#define STACK_CHARGER   3072   // I2C reads + logging

// -----------------------------------------------------------------------------
//  TIMING (periods, in milliseconds)
// -----------------------------------------------------------------------------
//  These replace the old `while(lastSend + 30 > millis()) delay(1);` busy-wait.
//  Instead we use vTaskDelayUntil(), which sleeps the task precisely so the CPU
//  is free for other work in the meantime.
// -----------------------------------------------------------------------------
#define HALL_PERIOD_MS  20    // sample finger sensors at 50 Hz
#define BLE_PERIOD_MS   30    // push a BLE notification ~33 times/second
#define IMU_PERIOD_MS   10    // IMU sensor fusion at 100 Hz (fast = less drift)

// -----------------------------------------------------------------------------
//  IMU (IIM-42652) -- sensor fusion
// -----------------------------------------------------------------------------
//  ENABLE_IMU compiles in the real driver + fusion task. Comment it out to build
//  without the RAK12033-IIM42652 library (the task then becomes a no-op stub).
#define ENABLE_IMU

//  Complementary-filter weight: how much we TRUST the gyro (smooth but drifts)
//  vs. the accelerometer (absolute but noisy under motion). 0.98 => 98% gyro,
//  2% accel each step. Higher = smoother but slower to correct drift.
#define IMU_COMP_ALPHA  0.98f

//  How many samples to average at startup to measure the gyro's zero-rate bias
//  (the glove must be held still during this). Subtracting bias stops yaw from
//  spinning while stationary.
#define IMU_BIAS_SAMPLES 300

// -----------------------------------------------------------------------------
//  HALL CALIBRATION (open/close to capture each sensor's min & max)
// -----------------------------------------------------------------------------
//  Uncomment HALL_CALIBRATE_ON_BOOT to run the interactive calibration at
//  startup, exactly like the original firmware: for HALL_CALIBRATE_MS the user
//  fully OPENS then CLOSES their hand, and we record the lowest/highest raw ADC
//  value seen on every sensor. Those become the min/max used to scale readings.
//  Left OFF by default so a normal boot uses the hard-coded ranges in hall.cpp
//  (matching how your original .ino actually shipped -- the call was commented).
//#define HALL_CALIBRATE_ON_BOOT
#define HALL_CALIBRATE_MS 10000   // 10 second open/close window

// -----------------------------------------------------------------------------
//  SERIAL PLOTTING (for Tools -> Serial Plotter)
// -----------------------------------------------------------------------------
//  Uncomment PLOT_HALL to stream the live Hall values in Arduino Serial Plotter
//  format ("label:value" pairs) every hall cycle. While this is on, normal
//  logf() text output is SILENCED so the plotter sees clean numeric data only.
//
//  PLOT_HALL_FINGER selects what to plot:
//     -1  = all 5 fingers x 3 sensors (15 traces -- busy but complete)
//    0..4 = just that one finger's 3 sensors (thumb=0 ... pinky=4)
//#define PLOT_HALL
#define PLOT_HALL_FINGER  1   // index finger (matches the old print_hall default)

// -----------------------------------------------------------------------------
//  POWER-ON SELF-TEST (bring-up diagnostic -- "first firmware go")
// -----------------------------------------------------------------------------
//  When defined, after a BLE client connects the firmware runs a one-shot sweep
//  of every subsystem (Hall, IMU, charger, servos) through the real drivers and
//  reports PASS/FAIL over Serial + BLE, THEN starts normal operation. Leave it
//  on for bench bring-up; comment it out for everyday/production use so boots are
//  fast and the servos don't sweep every time.
#define RUN_SELFTEST_ON_BOOT
#define SELFTEST_BLE_WAIT_MS  30000   // wait this long for a BLE client; else Serial-only

// -----------------------------------------------------------------------------
//  BQ25887 BATTERY CHARGER (2-cell Li-ion, I2C, automatic cell balancing)
// -----------------------------------------------------------------------------
//  Comment out ENABLE_CHARGER to build without the charger driver (the task then
//  becomes a no-op stub and battery_lvl stays 0).
#define ENABLE_CHARGER

#define CHARGER_I2C_ADDR   0x6A   // BQ25887 7-bit address (datasheet rev B; rev A misprinted 0x6B)
#define CHARGER_SDA        40     // shares the IMU's I2C bus -- VERIFY against YOUR schematic
#define CHARGER_SCL        39
#define CHARGER_PERIOD_MS  1000   // status poll + watchdog service period (1 Hz)

//  >>> SET THESE FOR YOUR ACTUAL BATTERY <<<  (values are PER CELL; 2 cells in series)
//  Wrong values can over-charge/over-current a pack. ICHG should be <= the cell's
//  rated charge current (often ~0.5C). Defaults below are deliberately conservative.
#define CHARGER_VCELL_MV   4200   // per-cell charge voltage  (3400..4600, 5 mV step)
#define CHARGER_ICHG_MA    500    // fast-charge current      (100..2200, 50 mA step)
#define CHARGER_ITERM_MA   150    // termination current      (50..800,  50 mA step)
#define CHARGER_IPRECHG_MA 150    // pre-charge current       (50..800,  50 mA step)

//  Battery % is a rough linear estimate between these PACK (2-cell) voltages.
#define BATT_EMPTY_MV      6000   // ~3.00 V/cell
#define BATT_FULL_MV       8400   // ~4.20 V/cell

//  Keep the BQ25887's I2C watchdog enabled (40 s) and let chargerTask kick it.
//  Fail-safe: if firmware hangs >40 s the charger reverts to its own safe
//  defaults. Comment this out to DISABLE the watchdog instead, in which case our
//  custom settings persist even with no MCU servicing (less fail-safe).
#define CHARGER_SERVICE_WATCHDOG

#endif // CONFIG_H
