#include "hall.hpp"

#include <sys/types.h>
#include <Arduino.h>
#include "shared.h"
#include "logger.hpp"

#ifdef RIGHT_HAND                                           // segment 0, segment 1, splay
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { {  4,  5,  6 }, // thumb
                                                                       {  7, 15, 16 }, // index
                                                                       { 17, 18,  8 },
                                                                       { 11, 12, 13 },  // swapped these 2 for
                                                                       {  3,  9, 10 }}; // better cable management
#elif defined(LEFT_HAND)
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { {  3,  9, 10 },
                                                                       { 11, 12, 13 },
                                                                       { 17, 18,  8 },
                                                                       {  7, 15, 16 },
                                                                       {  4,  5,  6 } };
#else
#error "Left or right hand not selected in board.h"
#endif

                                                               //Base, Knuckle, Splay
uint8_t invert_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 0, 1, 0 },   // Thumb
                                                                      { 0, 0, 0 },   // Index
                                                                      { 0, 1, 0 },   // Middle
                                                                      { 0, 1, 0 },   // Ring
                                                                      { 1, 1, 0 } }; // Pinky

uint16_t min_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 1080, 2450, 1050 },
                                                                    { 1500, 1550, 4095 },
                                                                    { 1610, 1970, 1578 },
                                                                    { 1420, 2050, 1480 },
                                                                    { 2250, 2030, 2315 } };

uint16_t max_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 1150, 2660, 1650 },
                                                                    { 1620, 1960, 4096 },
                                                                    { 1770, 2550, 1597 },
                                                                    { 1780, 2570, 1530 },
                                                                    { 2680, 2450, 2340 } };

#define HALL_TEST 1

static unsigned short hall[5][3];

void writeUnsignedShortToCharArrayLE(unsigned short value, unsigned char* charArray, uint8_t index) {
    // Store the most significant byte in the second element
    charArray[index + 1] = static_cast<char>((value >> 8) & 0xFF);
    // Store the least significant byte in the first element
    charArray[index] = static_cast<char>(value & 0xFF);
}

void init_hall() {
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
      pinMode(hall_pins[i][j], INPUT);
    }
  }
}

int** read_hall() {
  // Read raw values, software average filter. Burst read then process values
  unsigned short raw_hall[NUM_SAMPLES_AVG][5][3];
  for (uint8_t s = 0; s < NUM_SAMPLES_AVG; ++s){
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
      for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
        raw_hall[s][i][j] = analogRead(hall_pins[i][j]);
      }
    }
  }
  // Clamp filtered values and map from [0,4096). Invert magnet direction if needed
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
      // Average across NUM_SAMPLES_AVG samples
      uint32_t avg_sample = 0;
      for (uint8_t s = 0; s < NUM_SAMPLES_AVG; ++s){
        avg_sample += raw_hall[s][i][j];
      }

      // Clamp to min-max, map to [0,4096), then invert if needed
      uint32_t temp_hall_value = avg_sample / NUM_SAMPLES_AVG;
      temp_hall_value = constrain(temp_hall_value, min_hall_value[i][j], max_hall_value[i][j]);
      temp_hall_value = map(temp_hall_value, min_hall_value[i][j], max_hall_value[i][j], 0, 4096);
      if (invert_hall_value[i][j]) temp_hall_value = 4095 - temp_hall_value;

      // Transform from linear to sinesodial (less drop-off at beginning, more after bending half.)
      //    If the initial drop-off is too slow, change to ^0.5 or some other fraction
      // hall[i][j] = temp_hall_value;
      hall[i][j] = static_cast<uint32_t>((1.0 - cos(temp_hall_value * HALF_PI / 4095.0 )) * 4095.0);
    }
  }
  return (int**)hall;
}

void read_hall_calibration() { 
  read_hall();
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
        int temp_hall_value = hall[i][j];
        min_hall_value[i][j] = (min_hall_value[i][j] > temp_hall_value) ? temp_hall_value : min_hall_value[i][j];
        max_hall_value[i][j] = (max_hall_value[i][j] < temp_hall_value) ? temp_hall_value : max_hall_value[i][j];
    }
  }
}

void hall_callibration() {
  Serial.println("Please open, then close your hand so that we can calibrate!");
  int start_time = millis();
  while (millis() - start_time < 10000) {  
    read_hall_calibration();
  }
  // store in EEPROM?
  Serial.println("Values calibrated!");
}


void print_hall(void) {
  char buffer[8];
  read_hall();
  for (uint8_t i = HALL_TEST; i < HALL_TEST+1; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
      snprintf((char*)buffer, 8, "%*d ", 4, hall[i][j]);
      Serial.print(buffer);
    }
    Serial.println();
  }
}

void construct_package() {
  int bend_count = 0;
  int splay_count = 0;
  int splay_values_added = 0;
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER - 1; ++j) { // don't include splay
      writeUnsignedShortToCharArrayLE(hall[i][j], package_data.bend_angle, bend_count); // should call this 20 times
      bend_count += sizeof(unsigned short);
    }
    // 1 of the fingers does not have the splay sensor connected, ignore packaging that one
    if(i == IGNORE_SPLAY_FINGER) continue;
    // writeUnsignedShortToCharArrayLE(hall[i][2], package_data.splay, splay_count);
    if(i > 0){
      writeUnsignedShortToCharArrayLE(2048, package_data.splay, splay_count);
    }else{
      writeUnsignedShortToCharArrayLE(hall[i][2], package_data.splay, splay_count);
    }
    splay_count += sizeof(unsigned short);
  }
  package_data.battery_lvl[0] = 0;
  package_data.button = 0;
  package_data.joystick[0] = 0;
  package_data.joystick[1] = 0;
}