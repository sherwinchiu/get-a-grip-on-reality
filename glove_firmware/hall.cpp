#include "hall.hpp"

#include <Arduino.h>
#include "board.h"
#include "logger.hpp"

#ifdef RIGHT_HAND
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] 
        = {{ 4, 5, 6},
           { 7,15,16},
           {17,18, 8},
           { 3, 9,10},
           {11,12,13}};
#elif defined(LEFT_HAND)
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] 
        = {{11,12,13},
           { 3, 9,10},
           {17,18, 8},
           { 7,15,16},
           { 4, 5, 6}};
#else
  #error "Left or right hand not selected in board.h"
#endif

int min_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] 
        = {{5000,5000,5000},
           {5000, 5000,5000},
           {5000,5000,5000},
           {5000,5000,5000},
           {5000, 5000,5000}};
int max_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER]
        = {{0,0,0},
           {0,0,0},
           {0,0,0},
           {0,0,0},
           {0,0,0}};

static int hall[5][3];

void init_hall(){
  for(uint8_t i=0;i<NUM_HALL_ROWS;++i){
    for(uint8_t j=0;j<HALL_SENSORS_PER_FINGER;++j){
      pinMode(hall_pins[i][j], INPUT);
    }
  }
}

int** read_hall(){
  for(uint8_t i=0;i<NUM_HALL_ROWS;++i){
    for(uint8_t j=0;j<HALL_SENSORS_PER_FINGER;++j){
      hall[i][j] = analogRead(hall_pins[i][j]);
    }
  }
  return (int**) hall;
}
int** read_hall_calibration(bool min_or_max){ // false for min, true for max
  for(uint8_t i=0;i<NUM_HALL_ROWS;++i){
    for(uint8_t j=0;j<HALL_SENSORS_PER_FINGER;++j){
      if (min_or_max) { // for min
        int temp_hall_value = individual_hall_callibration(analogRead(hall_pins[i][j]));
        if (min_hall_value[i][j] > temp_hall_value) {
          min_hall_value[i][j] = temp_hall_value;
        }
      } else { // for max
        int temp_hall_value = individual_hall_callibration(analogRead(hall_pins[i][j]));
        if (max_hall_value[i][j] < temp_hall_value) {
          max_hall_value[i][j] = temp_hall_value;
        }
      }
    }
  }
  return (int**) hall;
}

void hall_callibration() {
  Serial.println("Please open, then close your hand so that we can calibrate!");
  int start_time = millis();
  while (millis() - start_time < 5000) { // for five seconds
    
    // we need to read all hall sensor values at the minimum and maximum state
    read_hall_calibration(false);
    read_hall_calibration(true);
  }
  // store in EEPROM?
  Serial.println("Values calibrated!");
}

// range of hall sensor is from 1200 - 1800 or 1200 - 300 depending on magnet orientation
// 255 will be 90 degrees finger bed, while 0 is 0 degrees finger bed
int individual_hall_callibration(int hall_reading) {
  int new_reading;
  if (hall_reading >= 1200 && hall_reading <= 1800) { // assume north magnet reading
    new_reading = map(hall_reading, 1200, 1800, 0, 255);
  } else if (hall_reading >= 300 && hall_reading <= 1200) {
    new_reading = map(hall_reading, 1200, 300, 0, 255); 
  }
  return new_reading;
}

void print_hall(void){
  char buffer[8];
  read_hall();
  for(uint8_t i=0;i<1;++i){
    for(uint8_t j=0;j<HALL_SENSORS_PER_FINGER;++j){
      snprintf((char*) buffer, 8, "%*d ", 4, hall[i][j]);
      Serial.print(buffer);
      // Log::print((const char*)buffer);
    }
    Serial.println();
    // Log::println();
  }
  // Serial.println();
}
