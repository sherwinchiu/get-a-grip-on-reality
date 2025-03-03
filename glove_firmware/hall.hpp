#ifndef HALL_HPP
#define HALL_HPP

#define NUM_HALL_ROWS 5   // 5 Fingers
#define HALL_SENSORS_PER_FINGER 3   // 3 Sensors per finger, 1 splay 2 close
#define NUM_SAMPLES_AVG 32
#define IGNORE_SPLAY_FINGER 1 //Index finger has no splay hall sensor connected

void init_hall(void);
int** read_hall(void);
void print_hall(void);

void read_hall_calibration();
void hall_callibration();

void construct_package(void);

#endif
