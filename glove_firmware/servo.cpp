#include "servo.hpp"

Servo servos[NUM_SERVO_ROWS];

void init_servos(void){
  for (int i = 0; i < NUM_SERVO_ROWS; i++) {
    servos[i].attach(servo_pins[i]); // attach 
    servos[i].write(0);
  }
  

}
void move_servos(int* pos) {
  for(int i = 0; i < NUM_SERVO_ROWS; i++) {
    servos[i].write(pos[i]);
  }
}