// =============================================================================
//  servo.cpp  --  5 haptic servos, driven with the ESP32 NATIVE LEDC PWM API.
//
//  WHY NOT THE ESP32Servo LIBRARY:
//  ESP32Servo 3.2.1's PWM allocator is buggy on the ESP32-S3 -- its MCPWM/LEDC
//  channel assignment double-books channels, so two fingers (e.g. index + ring)
//  land on the SAME channel and drive each other (one moves, the other un-moves,
//  and a servo fed a shared signal buzzes/ticks). We sidestep all of that: each
//  servo gets its OWN explicit LEDC channel (servo i -> channel i). Simple,
//  deterministic, and it's exactly what the library was supposed to do for us.
// =============================================================================
#include "servo.hpp"

#include <Arduino.h>         // ledcAttachChannel / ledcWrite
#include "rtos_common.hpp"   // servoQueue
#include "logger.hpp"

// --- Servo PWM parameters ----------------------------------------------------
//  Standard hobby servo (MG90s): 50 Hz frame (20 ms), pulse ~500..2500 us = 0..180.
//  14-bit duty resolution -> 16384 ticks per 20 ms frame (~1.2 us granularity).
//  NOTE: the ESP32-S3 LEDC caps resolution at 14 bits (LEDC_MAX_BIT_WIDTH) -- using
//  16 here made ledcAttachChannel() reject EVERY servo ("ATTACH FAILED").
static const uint32_t SERVO_FREQ_HZ  = 50;
static const uint8_t  SERVO_RES_BITS = 14;
static const uint32_t SERVO_MAX_DUTY = (1UL << SERVO_RES_BITS);   // 16384
static const uint32_t SERVO_FRAME_US = 1000000UL / SERVO_FREQ_HZ; // 20000 us
// MG90s-safe pulse band. Driving all the way to 500/2500us can push an MG90s past
// its mechanical end stop, where it stalls and BUZZES/ticks. 600..2400us gives
// near-full travel without hammering the limits (and still plenty for stall haptics).
static const uint32_t SERVO_MIN_US   = 550;                       // 0 deg
static const uint32_t SERVO_MAX_US   = 2450;                      // 180 deg

// Per-servo direction. ALL servos run through the IDENTICAL code path. But a servo
// can be MOUNTED or WIRED reversed (it pulls/engages at 0 deg instead of 180). If
// one finger sits engaged at rest while the others are slack, flip its flag here
// and its angle is mirrored (180 - a). Hardware-compensation knob, not logic.
//                                      thumb  index  middle  ring   pinky
static const bool servo_invert[NUM_SERVO_ROWS] = { false, false, false, false, false };

// Convert a 0..180 degree angle to an LEDC duty count for a 500..2500 us pulse.
static uint32_t angle_to_duty(int angle) {
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    uint32_t pulse_us = SERVO_MIN_US + (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180;
    return (uint32_t)((uint64_t)pulse_us * SERVO_MAX_DUTY / SERVO_FRAME_US);
}

void servo_write(int index, int angle) {
    if (index < 0 || index >= NUM_SERVO_ROWS) return;
    int a = servo_invert[index] ? (180 - angle) : angle;
    ledcWrite(servo_pins[index], angle_to_duty(a));   // 3.x LEDC API addresses by PIN
}

void init_servos(void) {
    for (int i = 0; i < NUM_SERVO_ROWS; i++) {
        // Explicit channel = i -> every servo is guaranteed its OWN channel, so no
        // two fingers can ever collide. They all share 50 Hz timers automatically.
        bool ok = ledcAttachChannel(servo_pins[i], SERVO_FREQ_HZ, SERVO_RES_BITS, i);
        ledcWrite(servo_pins[i], angle_to_duty(0));   // start relaxed
        logf("[servo] finger %d -> GPIO %2d  channel %d : %s\n",
             i, servo_pins[i], i, ok ? "attached OK" : "ATTACH FAILED");
    }
}

void move_servos(const int* pos) {
    for (int i = 0; i < NUM_SERVO_ROWS; i++) servo_write(i, pos[i]);
}

// -----------------------------------------------------------------------------
//  servo_sweep_test : drive each servo through its full range ONE AT A TIME.
//  Only a single servo moves at any moment (tiny current draw), so this isolates
//  a DEAD servo/pin (never moves even solo) from a POWER problem (moves fine solo
//  but browns out when several engage together). Logs index + GPIO as it goes.
// -----------------------------------------------------------------------------
void servo_sweep_test(void) {
    logf("\n[servo-test] sweeping each servo individually (no BLE / no contact)\n");
    for (int i = 0; i < NUM_SERVO_ROWS; i++) {
        logf("[servo-test] servo %d (GPIO %d): 0 -> 180 -> 0\n", i + 1, servo_pins[i]);
        for (int a = 0;   a <= 180; a += 10) { servo_write(i, a); delay(25); }
        delay(250);
        for (int a = 180; a >= 0;   a -= 10) { servo_write(i, a); delay(25); }
        delay(250);
    }
    logf("[servo-test] done -- all 5 swept\n\n");
}

// -----------------------------------------------------------------------------
//  THE TASK
// -----------------------------------------------------------------------------
void servoTask(void* pvParameters) {
    ServoCommand cmd;   // scratch space the queue copies each command into

    // A FreeRTOS task is an infinite loop. It must NEVER "return" or fall off the
    // end -- if it does, the scheduler aborts. To end a task call vTaskDelete(NULL).
    int prev[NUM_SERVO_ROWS] = { -1, -1, -1, -1, -1 };
    for (;;) {
        // xQueueReceive blocks until an item arrives (portMAX_DELAY = wait forever),
        // so the task uses ZERO CPU while idle.
        if (xQueueReceive(servoQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            move_servos(cmd.pos);
            // Log the commanded angle PER FINGER, only when it changes -- proves all
            // five go through identical logic. If they all get the same angle but one
            // servo misbehaves, that one is hardware/wiring (see servo_invert[]).
            bool changed = false;
            for (int i = 0; i < NUM_SERVO_ROWS; i++) { if (cmd.pos[i] != prev[i]) changed = true; prev[i] = cmd.pos[i]; }
            if (changed)
                logf("[servo] T=%d I=%d M=%d R=%d P=%d\n",
                     cmd.pos[0], cmd.pos[1], cmd.pos[2], cmd.pos[3], cmd.pos[4]);
        }
    }
}
