// =============================================================================
//  servo.hpp  --  Haptic feedback servos (5, one per finger)
// =============================================================================
#ifndef SERVO_HPP
#define SERVO_HPP

#include "shared.h"   // NUM_SERVO_ROWS + ServoCommand

// We drive the servos with the ESP32's NATIVE LEDC PWM API (esp32-hal-ledc),
// giving each servo its OWN explicit channel. We deliberately do NOT use the
// ESP32Servo library: its 3.2.1 PWM allocator double-books channels on the
// ESP32-S3 (two fingers land on the same channel -> they toggle each other and
// buzz). Native LEDC, one channel per pin, is simple and deterministic.

// GPIOs the servos are wired to (unchanged from original firmware).
//   [0]thumb=38  [1]index=37  [2]middle=36  [3]ring=35  [4]pinky=45
//
//  ⚠ ESP32-S3 PIN CAVEATS (NOT all of these are clean, deterministic GPIOs):
//   - GPIO35/36/37 are the OCTAL flash/PSRAM SPI pins. On "R8" modules
//     (e.g. WROOM-1-N16R8) they are consumed by PSRAM and CANNOT be used as
//     GPIO -> ring (35) would never drive. (If 36/37 work, you're likely NOT R8.)
//   - GPIO45 (pinky) is a STRAPPING pin (VDD_SPI). It's sampled/driven at boot,
//     so it's not a clean GPIO until attach() runs -> a common cause of a flaky
//     servo. Prefer moving pinky to a plain output GPIO (e.g. 1, 2, 42, 47, 48)
//     and rewiring, if you keep seeing problems there.
const static int servo_pins[NUM_SERVO_ROWS] = {38, 37, 36, 35, 45};

// Hardware setup. Call once from setup() before the task starts.
void init_servos(void);

// Drive ONE servo to an angle (0..180). Used by the self-test.
void servo_write(int index, int angle);

// Drive all servos to the given angles. Called by the servo TASK, not directly
// from the BLE callback (see servoTask).
void move_servos(const int* pos);

// Bring-up diagnostic: sweep each servo one at a time (see config SERVO_SWEEP_TEST).
void servo_sweep_test(void);

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
