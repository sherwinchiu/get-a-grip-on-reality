// =============================================================================
//  shared.h  --  Data types that travel BETWEEN tasks
// =============================================================================
//
//  This header only contains plain data structures (no logic). They are the
//  "messages" that flow through our FreeRTOS queues:
//
//      hallTask  --(InputData)-->     sensorMailbox  --> bleTask  --> phone
//      BLE RX    --(ServoCommand)-->  servoQueue     --> servoTask --> motors
//
//  Keeping the message types in one small header avoids include cycles between
//  the producer module and the consumer module.
// =============================================================================
#ifndef SHARED_H
#define SHARED_H

#include "config.h"   // pulls in LEFT_HAND / RIGHT_HAND selection

// -----------------------------------------------------------------------------
//  InputData : the packet we notify to the phone/PC over BLE.
//  (Unchanged byte layout from the original firmware so the receiver app still
//   parses it correctly.) It is sent little-endian, 2 bytes per sensor value.
// -----------------------------------------------------------------------------
struct InputData {
    unsigned char battery_lvl[1];   // 1 byte : battery level 0 -> 99
    unsigned char bend_angle[20];   // 20 bytes: 5 fingers x 2 segments x 2 bytes
    unsigned char splay[8];         // 8 bytes : 4 splay gaps x 2 bytes (thumb->pinky)
    unsigned char button;           // 1 byte  : button state
    unsigned char joystick[2];      // 2 bytes : joystick X/Y
    // ---- IMU orientation appended at the tail (bytes 32..43) ----------------
    // The original fields above sum to 32 bytes (already 4-byte aligned), so
    // these three little-endian floats append with NO padding -> sizeof==44.
    // Existing fields keep their byte offsets, so a host parser that only reads
    // the first 32 bytes still works; new hosts read 12 more for tilt.
    float roll;                     // bytes 32..35 : degrees about X
    float pitch;                    // bytes 36..39 : degrees about Y
    float yaw;                      // bytes 40..43 : degrees about Z (drifts)
};

// -----------------------------------------------------------------------------
//  ServoCommand : one haptic feedback command, produced by the BLE RX callback
//  and consumed by the servo task. We copy the parsed positions into this POD
//  struct and hand it to a queue so the BLE stack callback returns instantly.
// -----------------------------------------------------------------------------
#define NUM_SERVO_ROWS 5   // 5 haptic servos (one per finger)

struct ServoCommand {
    int pos[NUM_SERVO_ROWS];   // target angle 0..180 for each servo
};

// -----------------------------------------------------------------------------
//  Orientation : the FUSED output of the IMU task (degrees).
//  Produced by imuTask, published to imuMailbox, consumed by whoever needs the
//  hand's tilt (the BLE task can attach it, or it can drive gesture logic).
//
//      roll  -> rotation about X (hand tilting left/right)
//      pitch -> rotation about Y (hand tilting up/down)
//      yaw   -> rotation about Z (integrated gyro; drifts without a magnetometer)
// -----------------------------------------------------------------------------
struct Orientation {
    float roll;
    float pitch;
    float yaw;
};

#endif // SHARED_H
