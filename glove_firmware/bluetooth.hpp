#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP
#include <BLEServer.h>

#define SERVICE_UUID           "7241bbc8-8ed8-4729-85ea-0ffc63248b4f"  // UART service UUID 6E400001
#define CHARACTERISTIC_UUID_RX "36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4"
#define CHARACTERISTIC_UUID_TX "34797cc3-9e74-42e1-a669-be3cbdbae64d"

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