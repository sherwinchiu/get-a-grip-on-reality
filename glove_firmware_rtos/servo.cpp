// =============================================================================
//  servo.cpp
// =============================================================================
#include "servo.hpp"

#include "rtos_common.hpp"   // servoQueue
#include "logger.hpp"

Servo servos[NUM_SERVO_ROWS];

void init_servos(void) {
    for (int i = 0; i < NUM_SERVO_ROWS; i++) {
        servos[i].attach(servo_pins[i]);  // bind the Servo object to its GPIO
        servos[i].write(0);               // start relaxed (no haptic force)
    }
}

void move_servos(const int* pos) {
    for (int i = 0; i < NUM_SERVO_ROWS; i++) {
        servos[i].write(pos[i]);          // PWM angle 0..180
    }
}

// -----------------------------------------------------------------------------
//  THE TASK
// -----------------------------------------------------------------------------
void servoTask(void* pvParameters) {
    ServoCommand cmd;   // scratch space the queue copies each command into

    // A FreeRTOS task is an infinite loop. It must NEVER "return" or fall off
    // the end -- if it does, the scheduler aborts. If you ever need to end a
    // task, call vTaskDelete(NULL) instead.
    for (;;) {
        // xQueueReceive blocks this task until an item is available.
        //   - &cmd       : where to copy the received item
        //   - portMAX_DELAY : wait FOREVER (no timeout). Because we block here,
        //                     the servo task uses no CPU at all while idle.
        if (xQueueReceive(servoQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            // We're now in our own task context with all the time we need --
            // safe to drive the motors (which would have been rude to do inside
            // the BLE stack's callback).
            move_servos(cmd.pos);
        }
    }
}
