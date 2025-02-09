struct InputData {
    unsigned char battery_lvl[1];        // 1 byte for battery level 0 -> 99
    unsigned char bend_angle[20];         // 20 bytes for finger bend, 4 bytes/finger, 2 bytes/first two segments
    // thumb, -> pinky, lower finger, upper finger
    // 
    unsigned char splay[8];              // 2 bytes per in between finger, thumb -> pinky
    unsigned char button;				// 1 byte for anything we do w/ button
    unsigned char joystick[2];				// 2 byte for anything we do w/ joystick
};

InputData package_data;

#ifndef HALL_HPP
#define HALL_HPP

#define NUM_HALL_ROWS 5   // 5 Fingers
#define HALL_SENSORS_PER_FINGER 3   // 3 Sensors per finger, 1 splay 2 close

void init_hall(void);
int** read_hall(void);
void print_hall(void);

int** read_hall_calibration(bool min_or_max);
void hall_callibration();
int individual_hall_callibration(int hall_reading);

void construct_package(void);


#endif
