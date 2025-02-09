#ifndef SHARED_H
#define SHARED_H
struct InputData {
    unsigned char battery_lvl[1];        // 1 byte for battery level 0 -> 99
    unsigned char bend_angle[20];         // 20 bytes for finger bend, 4 bytes/finger, 2 bytes/first two segments
    // thumb, -> pinky, lower finger, upper finger
    // 
    unsigned char splay[8];              // 2 bytes per in between finger, thumb -> pinky
    unsigned char button;				// 1 byte for anything we do w/ button
    unsigned char joystick[2];				// 2 byte for anything we do w/ joystick
};

extern InputData package_data;
#endif