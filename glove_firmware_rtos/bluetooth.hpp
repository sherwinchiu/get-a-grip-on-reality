// =============================================================================
//  bluetooth.hpp  --  BLE GATT server: notify sensor data, receive haptics
// =============================================================================
#ifndef BLUETOOTH_HPP
#define BLUETOOTH_HPP

#include <BLEServer.h>

// 128-bit UUIDs for our custom service + characteristics (unchanged).
#define SERVICE_UUID            "7241bbc8-8ed8-4729-85ea-0ffc63248b4f"
#define CHARACTERISTIC_UUID_TX  "34797cc3-9e74-42e1-a669-be3cbdbae64d" // glove -> host (notify)
#define CHARACTERISTIC_UUID_RX  "36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4" // host -> glove (write/haptics)

// Connection state callbacks: these SET/CLEAR the BLE_CONNECTED_BIT event bit
// instead of poking a shared global directly.
//
//  NOTE: arduino-esp32 core 3.x calls BOTH the one-arg and two-arg forms of
//  onConnect/onDisconnect (see BLEServer.cpp ESP_GATTS_CONNECT_EVT, which calls
//  onConnect(this) then onConnect(this, param)). Overriding just the one-arg
//  form is therefore sufficient -- it runs on every connect/disconnect.
class ServerCallback : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};

// Set by setup() from initImu()'s result. When false, bleTask transmits ONLY
// the original 32-byte InputData (no orientation tail) so old host parsers keep
// working unchanged; when true it transmits the full 44-byte packet.
extern bool g_imuPresent;

// Initialise the BLE stack, service, and characteristics. Call once in setup().
void initBluetooth(void);

// --- Helpers used by the power-on self-test (work without bleTask running) ---
// True if a client is currently connected (reads the event-group bit).
bool bleConnected(void);
// Block up to timeout_ms for a client to connect. Returns true if connected.
bool bleWaitConnected(uint32_t timeout_ms);
// Notify an arbitrary text string on the TX characteristic (for POST reports).
// No-op if not connected. NOTE: during normal operation TX carries InputData;
// only the self-test sends text, before the streaming tasks start.
void bleNotifyText(const char* msg);

// -----------------------------------------------------------------------------
//  bleTask : periodic consumer/transmitter.
//  Waits for a connection (event group), pulls the freshest packet from the
//  sensorMailbox, and notifies it to the host ~33x/sec.
// -----------------------------------------------------------------------------
void bleTask(void* pvParameters);

#endif // BLUETOOTH_HPP
