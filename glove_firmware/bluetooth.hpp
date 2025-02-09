#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP
#include <BLEServer.h>
// default uuid
// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID 6E400001
// #define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
// #define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define SERVICE_UUID        "7241bbc8-8ed8-4729-85ea-0ffc63248b4f"
#define CHARACTERISTIC_UUID "34797cc3-9e74-42e1-a669-be3cbdbae64d"
#define CHARACTERISTIC_UUID2 "3a8b9eb6-d16c-4075-9178-a5c0380a5815"
#define WRITE_CHARACTERISTIC_UUID "36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4"

class ServerCallback : public BLEServerCallbacks {
public:
  void onConnect(BLEServer *pServer);
  void onDisconnect(BLEServer *pServer);
};

class PeripheralCallback : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *pCharacteristic);
};

void initBluetooth(void);
void bluetoothTask(void);
bool isBluetoothConnected(void);
void transmitMessage(const char* msg, size_t len);
#endif