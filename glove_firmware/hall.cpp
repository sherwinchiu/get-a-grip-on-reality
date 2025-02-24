#include "hall.hpp"
#include "shared.h"
#include <Arduino.h>
#include "board.h"
#include "logger.hpp"

#ifdef RIGHT_HAND
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 4, 5, 6 }, // segment 0, segment 1, splay thumb
                                                                       { 7, 15, 16 }, // index
                                                                       { 17, 18, 8 },
                                                                       { 3, 9, 10 },
                                                                       { 11, 12, 13 } };
#elif defined(LEFT_HAND)
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 11, 12, 13 },
                                                                       { 3, 9, 10 },
                                                                       { 17, 18, 8 },
                                                                       { 7, 15, 16 },
                                                                       { 4, 5, 6 } };
#else
#error "Left or right hand not selected in board.h"
#endif

int min_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 5000, 5000, 5000 },
                                                               { 5000, 5000, 5000 },
                                                               { 5000, 5000, 5000 },
                                                               { 5000, 5000, 5000 },
                                                               { 5000, 5000, 5000 } };
int max_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 0, 0, 0 },
                                                               { 0, 0, 0 },
                                                               { 0, 0, 0 },
                                                               { 0, 0, 0 },
                                                               { 0, 0, 0 } };

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
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
      int temp_hall_value = analogRead(hall_pins[i][j]);
      if (temp_hall_value > max_hall_value[i][j]){
        temp_hall_value = max_hall_value[i][j] - 1;
      } else if (temp_hall_value < min_hall_value[i][j]) {
        temp_hall_value = min_hall_value[i][j] + 1;
      }
      hall[i][j] = map(temp_hall_value, min_hall_value[i][j], max_hall_value[i][j], 0, 4095);

    }
  }
  return (int**)hall;
}
int** read_hall_calibration() {  // false for min, true for max
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
        int temp_hall_value = analogRead(hall_pins[i][j]);
        if (min_hall_value[i][j] > temp_hall_value) {
          min_hall_value[i][j] = temp_hall_value;
        }
        if (max_hall_value[i][j] < temp_hall_value) {
          max_hall_value[i][j] = temp_hall_value;
        }
    }
  }
  return (int**)hall;
}

void hall_callibration() {
  Serial.println("Please open, then close your hand so that we can calibrate!");
  int start_time = millis();
  //digitalWrite(21, HIGH);
  while (millis() - start_time < 10000) {  // for five seconds
    // we need to read all hall sensor values at the minimum and maximum state
    read_hall_calibration();
    
  }
  //digitalWrite(21, LOW);
  // store in EEPROM?
  Serial.println("Values calibrated!");
}


void print_hall(void) {
  char buffer[8];
  read_hall();
  for (uint8_t i = 0; i < 1; ++i) {
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
  for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
    for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER - 1; ++j) { // don't include splay
      writeUnsignedShortToCharArrayLE(hall[i][j], package_data.bend_angle, bend_count); // should call this 20 times
      bend_count += 2;
      // writeUnsignedShortToCharArrayLE(4096 * i + 410*j, package_data.bend_angle, bend_count); // should call this 20 times
    }
    
    writeUnsignedShortToCharArrayLE(hall[i][2], package_data.splay, splay_count);
    splay_count += 2;
  }
  package_data.battery_lvl[0] = 0;
  package_data.button = 0;
  package_data.joystick[0] = 0;
  package_data.joystick[1] = 0;
}