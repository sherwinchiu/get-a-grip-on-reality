// =============================================================================
//  bluetooth.cpp
// =============================================================================
#include "bluetooth.hpp"

#include <Arduino.h>
#include <stddef.h>          // offsetof
#include <string.h>          // strlen
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "config.h"
#include "shared.h"
#include "rtos_common.hpp"   // sensorMailbox, servoQueue, bleEvents
#include "logger.hpp"

// Whether an IMU was detected at boot (set from setup()). Drives packet length.
bool g_imuPresent = false;

// BLE objects. Created in initBluetooth(), used by the task + callbacks.
static BLEServer*         pServer         = nullptr;
static BLECharacteristic* pTxCharacteristic = nullptr;  // we NOTIFY through this
static BLECharacteristic* pRxCharacteristic = nullptr;  // host WRITES haptics here

// -----------------------------------------------------------------------------
//  CONNECTION STATE CALLBACKS
// -----------------------------------------------------------------------------
//  These run in the BLE host task context. All we do is flip an event-group
//  bit; any task interested in "are we connected?" waits on that bit. Notice we
//  do NOT touch the radio or do slow work here.
// -----------------------------------------------------------------------------
void ServerCallback::onConnect(BLEServer* /*pServer*/) {
    xEventGroupSetBits(bleEvents, BLE_CONNECTED_BIT);
    logf("[BLE] onConnect\n");
}

void ServerCallback::onDisconnect(BLEServer* /*pServer*/) {
    xEventGroupClearBits(bleEvents, BLE_CONNECTED_BIT);
}

// -----------------------------------------------------------------------------
//  RX (WRITE) CALLBACK  --  the host sends us haptic targets
// -----------------------------------------------------------------------------
//  CRITICAL FREERTOS LESSON:
//  This callback runs INSIDE the BLE stack. If we moved servos here (PWM writes,
//  any delay) we'd block the BLE host task and wreck throughput/latency for
//  everyone. Instead we parse the bytes into a ServoCommand and POST it to
//  servoQueue, then return immediately. servoTask does the heavy lifting.
// -----------------------------------------------------------------------------
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() == 0) return;

        ServoCommand cmd = {{0, 0, 0, 0, 0}};
        // One byte per finger (the host sends 0 = not touching, 255 = touching).
        // BINARY haptics: any finger over the threshold -> drive its servo to the
        // ENGAGED angle and hold (stall torque); otherwise RELAXED. No gradient.
        for (int i = 0; i + 1 < (int)rxValue.length() && (i / 2) < NUM_SERVO_ROWS; i += 2) {
            bool touching = (uint8_t)rxValue[i] >= SERVO_ENGAGE_THRESH;
            cmd.pos[i / 2] = touching ? SERVO_ENGAGED_DEG : SERVO_RELAXED_DEG;
        }

        // Hand the command off to the servo task. Timeout 0 == "don't block":
        // if the queue is somehow full, we drop this command rather than stall
        // the BLE stack. The next command will arrive in a few ms anyway.
        xQueueSend(servoQueue, &cmd, 0);
    }
};

void initBluetooth(void) {
#ifdef LEFT_HAND
    BLEDevice::init("GloveLeft");
#else
    BLEDevice::init("GloveRight");
#endif

    // Prefer a large ATT MTU. Our notify packet is 32 bytes (44 with IMU), but the
    // DEFAULT MTU is only 23 -> max notify payload 20 bytes, which would TRUNCATE
    // our packet and the host would silently reject it as "too short". The central
    // (phone) drives the actual MTU exchange, but advertising a large local MTU
    // lets it negotiate up. Android Chrome auto-requests 247; this keeps us ready.
    BLEDevice::setMTU(247);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallback());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    // TX: NOTIFY characteristic. The BLE2902 descriptor lets the client enable
    // notifications (subscribe).
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX: WRITE characteristic. We allow BOTH write-with-response and
    // write-WITHOUT-response: the web app uses the no-response form for haptics so
    // each force update skips the ACK round-trip -> noticeably lower latency / snappier.
    pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pRxCharacteristic->setCallbacks(new RxCallbacks());
    pRxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    // Start advertising so phones/PCs can find us.
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);   // helps iOS connection stability
    pAdvertising->setMaxPreferred(0x06);
    BLEDevice::startAdvertising();

    logf("[BLE] Advertising\n");
}

// -----------------------------------------------------------------------------
//  Self-test helpers. These work BEFORE bleTask is created: connection state is
//  tracked by the BLE stack's callbacks (event-group bit), and notify() can be
//  called directly on the characteristic.
// -----------------------------------------------------------------------------
bool bleConnected(void) {
    // Primary signal: the bit our onConnect callback sets. Belt-and-suspenders:
    // also trust the server's own live connection count, so even if the connect
    // callback never fired/set the bit, we still detect the link and transmit.
    if ((xEventGroupGetBits(bleEvents) & BLE_CONNECTED_BIT) != 0) return true;
    return pServer && pServer->getConnectedCount() > 0;
}

bool bleWaitConnected(uint32_t timeout_ms) {
    EventBits_t bits = xEventGroupWaitBits(
        bleEvents, BLE_CONNECTED_BIT, pdFALSE /*don't clear*/, pdTRUE /*all bits*/,
        pdMS_TO_TICKS(timeout_ms));
    return (bits & BLE_CONNECTED_BIT) != 0;
}

void bleNotifyText(const char* msg) {
    if (!pTxCharacteristic || !bleConnected()) return;
    pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
    pTxCharacteristic->notify();
}

// -----------------------------------------------------------------------------
//  THE TASK : consumer/transmitter
// -----------------------------------------------------------------------------
void bleTask(void* pvParameters) {
    InputData pkg;
    bool wasConnected = false;
    const TickType_t period = pdMS_TO_TICKS(BLE_PERIOD_MS);

    // Diagnostic counters so Serial proves whether we are actually transmitting:
    //   sent  = notify() calls succeeded   miss = mailbox had no fresh packet
    uint32_t sent = 0, miss = 0;
    TickType_t lastReport = xTaskGetTickCount();

    for (;;) {
        // Are we connected? bleConnected() checks BOTH the callback's event bit
        // AND the server's live connection count, so a missed callback can't make
        // us sit silent on an open link.
        bool connected = bleConnected();

        // --- Handle a fresh DISCONNECT: kick advertising back on. ---
        if (!connected && wasConnected) {
            vTaskDelay(pdMS_TO_TICKS(500));   // let the BLE stack settle
            pServer->startAdvertising();
            logf("[BLE] client left -> re-advertising\n");
        }
        // --- Handle a fresh CONNECT. ---
        if (connected && !wasConnected) {
            logf("[BLE] client connected\n");
        }
        wasConnected = connected;

        // --- If connected, transmit the freshest sensor packet. ---
        if (connected) {
            // Pull the latest snapshot from the mailbox. Timeout 0: if the hall
            // task hasn't produced anything new since last time, skip this round
            // rather than waiting.
            if (xQueueReceive(sensorMailbox, &pkg, 0) == pdTRUE) {
                // Overlay the freshest IMU orientation onto the packet. We PEEK
                // (not receive) so the mailbox keeps the value -- the IMU task
                // runs at 100 Hz and overwrites it anyway. If the IMU is
                // disabled the peek fails and pkg keeps its zeroed defaults.
                // Decide the packet LENGTH based on whether we have an IMU:
                //   no IMU  -> send only the original fields (up to .roll) = 32B
                //   has IMU -> overlay orientation and send the full 44B packet
                size_t len = offsetof(InputData, roll);   // 32 bytes (no IMU)
                if (g_imuPresent) {
                    Orientation att;
                    if (xQueuePeek(imuMailbox, &att, 0) == pdTRUE) {
                        pkg.roll  = att.roll;
                        pkg.pitch = att.pitch;
                        pkg.yaw   = att.yaw;
                    }
                    len = sizeof(InputData);              // 44 bytes (with IMU)
                }
                pTxCharacteristic->setValue((uint8_t*)&pkg, len);
                pTxCharacteristic->notify();
                sent++;
            } else {
                miss++;
            }
        }

        // Once per second, report whether we're actually streaming. If `sent`
        // keeps climbing the firmware IS notifying -> any "no data" problem is on
        // the host/web side. If `miss` climbs instead, the hall mailbox is empty
        // (hallTask stalled). If neither moves, we're not reaching this branch
        // (not connected).
        if (xTaskGetTickCount() - lastReport >= pdMS_TO_TICKS(1000)) {
            logf("[BLE] tx: sent=%lu miss=%lu connected=%d\n",
                 (unsigned long)sent, (unsigned long)miss, (int)(wasConnected ? 1 : 0));
            sent = 0;
            miss = 0;
            lastReport = xTaskGetTickCount();
        }

        // Sleep ~30 ms; cheap fixed-rate loop. (vTaskDelay is fine here -- a few
        // ms of jitter on the transmit cadence is harmless.)
        vTaskDelay(period);
    }
}
