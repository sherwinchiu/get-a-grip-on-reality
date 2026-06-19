// =============================================================================
//  rtos_common.hpp  --  The FreeRTOS primitives shared across all tasks
// =============================================================================
//
//  This is the "nervous system" of the firmware. Everything that lets the tasks
//  talk to each other safely lives here:
//
//    1. QUEUES        -> pass data between tasks without sharing raw globals
//    2. A MUTEX       -> protect a shared resource (the Serial port) from being
//                        scribbled on by two tasks at once
//    3. AN EVENT GROUP-> broadcast a state change (BLE connected / disconnected)
//                        that several tasks can wait on
//
//  These objects are created ONCE in rtos_init() (called from setup()) and then
//  referenced everywhere via these `extern` handles.
// =============================================================================
#ifndef RTOS_COMMON_HPP
#define RTOS_COMMON_HPP

#include <Arduino.h>
#include "shared.h"

// FreeRTOS headers. On Arduino-ESP32 these are bundled with the core, so no
// extra library install is needed. We include them explicitly so the handle
// types below are defined.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// -----------------------------------------------------------------------------
//  QUEUE 1 : servoQueue  (BLE callback  ->  servoTask)
// -----------------------------------------------------------------------------
//  WHY A QUEUE HERE?
//  The BLE "onWrite" callback runs inside the BLE host task. You must NOT do
//  slow work there (moving 5 servos, delays) or you stall the whole BLE stack.
//  So the callback just drops a ServoCommand into this queue and returns. The
//  servoTask, blocked waiting on the queue, wakes up and does the actual work.
//  This is the textbook "defer work out of a callback/ISR" pattern.
// -----------------------------------------------------------------------------
extern QueueHandle_t servoQueue;

// -----------------------------------------------------------------------------
//  QUEUE 2 : sensorMailbox  (hallTask  ->  bleTask)
// -----------------------------------------------------------------------------
//  This queue has LENGTH 1 and is written with xQueueOverwrite(). That makes it
//  a "mailbox": it always holds the LATEST sensor snapshot, overwriting any
//  stale one. The hall task produces ~50 packets/sec; the BLE task consumes the
//  freshest one whenever it is ready to transmit. Old samples are useless for
//  real-time tracking, so dropping them is exactly what we want.
// -----------------------------------------------------------------------------
extern QueueHandle_t sensorMailbox;

// -----------------------------------------------------------------------------
//  QUEUE 3 : imuMailbox  (imuTask  ->  any consumer)
// -----------------------------------------------------------------------------
//  Same length-1 "mailbox" pattern as sensorMailbox, but carrying the fused
//  Orientation from the IMU task. Always holds the latest tilt; readers grab it
//  with a 0-timeout xQueueReceive (or xQueuePeek if they don't want to drain it).
// -----------------------------------------------------------------------------
extern QueueHandle_t imuMailbox;

// -----------------------------------------------------------------------------
//  MUTEX : serialMutex  (protects the shared Serial UART)
// -----------------------------------------------------------------------------
//  Serial is a single shared peripheral. If two tasks call Serial.print() at the
//  same time, their characters interleave into garbage. A MUTEX gives a task
//  EXCLUSIVE access: a task must "take" it before printing and "give" it after.
//  Any other task that tries to take it blocks until it is free. (See logger.)
// -----------------------------------------------------------------------------
extern SemaphoreHandle_t serialMutex;

// (No i2cMutex: there is no I2C bus in the current build -- the IMU is wired for
//  SPI and the charger is not present. Re-add one here only if an I2C peripheral
//  shared between two tasks is introduced.)

// Latest battery charge estimate (0..100), written by chargerTask and read by
// the hall packet builder so it rides along in InputData.battery_lvl. A single
// byte write/read is atomic on a 32-bit core, so no lock is needed for it.
extern volatile uint8_t g_batteryPercent;

// -----------------------------------------------------------------------------
//  EVENT GROUP : bleEvents  (broadcasts BLE connection state)
// -----------------------------------------------------------------------------
//  An event group is a set of bits that tasks can set, clear, and wait on. We
//  use a single bit to mean "a phone is connected". The BLE server callbacks
//  set/clear it; the bleTask waits on it so it only burns CPU notifying when
//  there is actually someone listening.
// -----------------------------------------------------------------------------
extern EventGroupHandle_t bleEvents;
#define BLE_CONNECTED_BIT  (1 << 0)   // bit 0 == "client connected"

// -----------------------------------------------------------------------------
//  Create all of the above. MUST be called (once) before any task starts or any
//  BLE callback can fire, because those code paths use these handles.
// -----------------------------------------------------------------------------
void rtos_init(void);

#endif // RTOS_COMMON_HPP
