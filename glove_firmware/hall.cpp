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
