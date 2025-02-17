#ifndef HALL_HPP
#define HALL_HPP

#define NUM_HALL_ROWS 5   // 5 Fingers
#define HALL_SENSORS_PER_FINGER 3   // 3 Sensors per finger, 1 splay 2 close

void init_hall(void);
int** read_hall(void);
void print_hall(void);

int** read_hall_calibration();
void hall_callibration();
int individual_hall_callibration(int hall_reading);

void construct_package(void);

#endif
