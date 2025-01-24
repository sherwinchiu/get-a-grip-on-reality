#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP
#include <BLEServer.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID 6E400001
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

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