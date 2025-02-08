// The goal of this sketch is to test BLE and the IMU, 
// The IMU is IIM-42652 and we're using BLE
#include <EEPROM.h>
#include "bluetooth.hpp"
#include "imu.hpp"
#include "logger.hpp"
#include "hall.hpp"

bool run_callibration = true;

void setup() {
  // initBluetooth();
  // initImu();
  init_hall();
  Serial.begin(115200);

  // while(!isBluetoothConnected());
  // Log::println("Starting");
  // two modes: normal mode or hall calibration mode
  if (run_callibration) {
    // run callibration for hall sensors
    hall_callibration();
  } else {
    // run normally
  }
}

void loop() {
  // bluetoothTask();
  // imuTask();
  print_hall();
  delay(100);
}

