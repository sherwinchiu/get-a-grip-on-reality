// The goal of this sketch is to test BLE and the IMU, 
// The IMU is IIM-42652 and we're using BLE
#include <EEPROM.h>
#include "hall.hpp"
#include "bluetooth.hpp"
// #include "imu.hpp"
#include "logger.hpp"
#include "servo.hpp"


bool run_callibration = true;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");
  initBluetooth();
  //initImu();
  pinMode(21, OUTPUT);
  init_servos();
  init_hall();

  // while(!isBluetoothConnected());
  // Log::println("Starting");
  // two modes: normal mode or hall calibration mode
  // if (run_callibration) {
  //    // run callibration for hall sensors
  //    hall_callibration();
  // }
}

uint32_t lastSend;
void loop() {
  lastSend = millis();
  bluetoothTask();
  print_hall();
  // read_hall();
  // imuTask();
  while(lastSend + 30 > millis()) delay(1);
}

