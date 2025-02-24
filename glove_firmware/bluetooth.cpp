#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "hall.hpp"
#include "bluetooth.hpp"
#include "logger.hpp"
#include "shared.h"
#include "servo.hpp"

BLEServer *pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pWriteCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

InputData package_data;


// each segment has 

void ServerCallback::onConnect(BLEServer *pServer) {
  deviceConnected = true;
};

void ServerCallback::onDisconnect(BLEServer *pServer) {
  deviceConnected = false;
}
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    digitalWrite(21, HIGH);
    if (rxValue.length() > 0) {
      int servo_pos[NUM_SERVO_ROWS] = {0, 0, 0, 0, 0};
      for (int i = 0; i < rxValue.length(); i += 2) {
        servo_pos[i/2] = map(rxValue[i], 0, 255, 0, 180);
      }
      move_servos(servo_pos);
    }
   // delay(300);
    digitalWrite(21, LOW);
  }
};
void PeripheralCallback::onWrite(BLECharacteristic *pCharacteristic) {
    
}

void initBluetooth(void){
  // // Create the BLE Device
  // // Create the BLE Server
  // pServer = BLEDevice::createServer();
  // pServer->setCallbacks(new ServerCallback());

  // // Create the BLE Service
  // BLEService *pService = pServer->createService(SERVICE_UUID);

  // // Create the BLE Characteristic
  // pCharacteristic = pService->createCharacteristic(
  //                     CHARACTERISTIC_UUID,
  //                     BLECharacteristic::PROPERTY_NOTIFY  // Add notify property
  //                   );
  // pWriteCharacteristic = pService->createCharacteristic(
  //                     WRITE_CHARACTERISTIC_UUID,
  //                     BLECharacteristic::PROPERTY_WRITE  // Add notify property
  //                   );

  // // Set callback for the write characteristic
  // pWriteCharacteristic->setCallbacks(new PeripheralCallback());

  // // Create a BLE Descriptor
  // pCharacteristic->addDescriptor(new BLE2902());

  // // Start the service
  // pService->start();

  // // Start advertising
  // BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  // pAdvertising->addServiceUUID(SERVICE_UUID);
  // pAdvertising->setScanResponse(true);
  // //pAdvertising->setMinInterval();
  // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  // pAdvertising->setMinPreferred(0x12);
  // BLEDevice::startAdvertising();
    // Create the BLE Device
  BLEDevice::init("FYDPGloveRight");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallback());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());

  pWriteCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pWriteCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x06);
  BLEDevice::startAdvertising();
  // Log::println("Waiting a client connection to notify...");
 
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void bluetoothTask(void){

  // if (deviceConnected) {
  //   pTxCharacteristic->setValue(&txValue, 1);
  //   pTxCharacteristic->notify();
  //   txValue++;
  //   delay(10);  // bluetooth stack will go into congestion, if too many packets are sent
  // }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Log::println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  } 
  
  if (deviceConnected) { // periodically send data
    read_hall();
    construct_package();
    if (deviceConnected){
      pCharacteristic->setValue((uint8_t*)&package_data, sizeof(InputData));
      pCharacteristic->notify();
    }
  }
  
}

bool isBluetoothConnected(void){
  return deviceConnected;
}

void transmitMessage(const char* msg, size_t len){
  if (deviceConnected){
      pCharacteristic->setValue((uint8_t*)msg, sizeof(InputData));
      pCharacteristic->notify();
    }
  
}