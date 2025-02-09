// The goal of this sketch is to test BLE and the IMU, 
// The IMU is IIM-42652 and we're using BLE

#include "bluetooth.hpp"
#include "imu.hpp"
#include "logger.hpp"

void setup() {
  initBluetooth();
  initImu();

  while(!isBluetoothConnected());
  Log::println("Starting");
}

void loop() {
  bluetoothTask();
  imuTask();
}

