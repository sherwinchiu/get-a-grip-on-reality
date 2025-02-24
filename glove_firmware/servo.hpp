#ifndef SERVO_HPP
#define SERVO_HPP
#include <ESP32Servo.h>

#define NUM_SERVO_ROWS 5   // 5 Servo motors

const static int servo_pins[NUM_SERVO_ROWS] = {38, 37, 36, 35, 45};

extern Servo servos[NUM_SERVO_ROWS];

void init_servos(void);
void move_servos(int* pos); 

#endif
