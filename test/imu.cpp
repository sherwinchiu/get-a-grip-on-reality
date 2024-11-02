#include "imu.hpp"

#include <Arduino.h>
#include <Wire.h>
#include "logger.hpp"
#include "RAK12033-IIM42652.h"
IIM42652 IMU;

void initImu(void){
  Wire.begin(40, 39);
  while (!IMU.begin(Wire, 0x68)){
    Log::println("IIM-42652 is not connected.");
    delay(1000);
  }
  IMU.ex_idle();
  IMU.accelerometer_enable();
  IMU.gyroscope_enable();
  IMU.temperature_enable();
}

void imuTask(void){
  IIM42652_axis_t accel_data;
  IIM42652_axis_t gyro_data;
  float temp;

  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  Log::println("Accel task");

  delay(100);

  IMU.get_accel_data(&accel_data);
  IMU.get_gyro_data(&gyro_data);
  IMU.get_temperature(&temp);

  /*
   * ±16 g  : 2048  LSB/g
   * ±8 g   : 4096  LSB/g
   * ±4 g   : 8192  LSB/g
   * ±2 g   : 16384 LSB/g
   */
  acc_x = (float)accel_data.x / 2048;
  acc_y = (float)accel_data.y / 2048;
  acc_z = (float)accel_data.z / 2048;

  Log::print("Accel X: ");
  Log::print(acc_x);
  Log::print("[g]  Y: ");
  Log::print(acc_y);
  Log::print("[g]  Z: ");
  Log::print(acc_z);
  Log::println("[g]");

  // /*
  //  * ±2000 º/s    : 16.4   LSB/(º/s)
  //  * ±1000 º/s    : 32.8   LSB/(º/s)
  //  * ±500  º/s    : 65.5   LSB/(º/s)
  //  * ±250  º/s    : 131    LSB/(º/s)
  //  * ±125  º/s    : 262    LSB/(º/s)
  //  * ±62.5  º/s   : 524.3  LSB/(º/s)
  //  * ±31.25  º/s  : 1048.6 LSB/(º/s)
  //  * ±15.625 º/s  : 2097.2 LSB/(º/s)
  //  */
  // gyro_x = (float)gyro_data.x / 16.4;
  // gyro_y = (float)gyro_data.y / 16.4;
  // gyro_z = (float)gyro_data.z / 16.4;

  // Log::print("Gyro  X:");
  // Log::print(gyro_x);
  // Log::print("º/s  Y: ");
  // Log::print(gyro_y);
  // Log::print("º/s  Z: ");
  // Log::print(gyro_z);
  // Log::println("º/s");

  // Log::print("Temper : ");
  // Log::print(temp);
  // Log::println("[ºC]");
}