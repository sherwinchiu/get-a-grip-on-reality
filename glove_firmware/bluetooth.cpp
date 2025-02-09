#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "bluetooth.hpp"
#include "logger.hpp"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

void ServerCallback::onConnect(BLEServer *pServer) {
  deviceConnected = true;
};

void ServerCallback::onDisconnect(BLEServer *pServer) {
  deviceConnected = false;
}

void PeripheralCallback::onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Log::println("*********");
      Log::print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++) {
        Log::print(rxValue[i]);
      }

      Log::println();
      Log::println("*********");
    }
  }

void initBluetooth(void){
  // Create the BLE Device
  BLEDevice::init("FYDPGloveRight");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallback());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new PeripheralCallback());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Log::println("Waiting a client connection to notify...");
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
  if (deviceConnected) {
    char dog = char(analogRead(4));
    transmitMessage(&dog, sizeof(char));
  }
  
}

bool isBluetoothConnected(void){
  return deviceConnected;
}

void transmitMessage(const char* msg, size_t len){
  if (deviceConnected){
    pTxCharacteristic->setValue((uint8_t *)msg, len);
    pTxCharacteristic->notify();
  }
}