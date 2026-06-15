// =============================================================================
//  servo.hpp  --  Haptic feedback servos (5, one per finger)
// =============================================================================
#ifndef SERVO_HPP
#define SERVO_HPP

#include <ESP32Servo.h>
#include "shared.h"   // NUM_SERVO_ROWS + ServoCommand

// GPIOs the servos are wired to (unchanged from original firmware).
const static int servo_pins[NUM_SERVO_ROWS] = {38, 37, 36, 35, 45};

extern Servo servos[NUM_SERVO_ROWS];

// Hardware setup. Call once from setup() before the task starts.
void init_servos(void);

// Drive all servos to the given angles. Called by the servo TASK, not directly
// from the BLE callback (see servoTask).
void move_servos(const int* pos);

// -----------------------------------------------------------------------------
//  servoTask : the FreeRTOS task that owns the servos.
//
//  It BLOCKS on servoQueue waiting for a ServoCommand. While the queue is empty
//  the task consumes ZERO CPU -- the scheduler simply doesn't run it. The moment
//  the BLE callback posts a command, the task wakes, moves the motors, and goes
//  back to sleep. This is "event-driven" task design.
// -----------------------------------------------------------------------------
void servoTask(void* pvParameters);

#endif // SERVO_HPP
