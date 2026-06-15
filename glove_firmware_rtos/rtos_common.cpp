// =============================================================================
//  rtos_common.cpp  --  Definitions + one-time creation of the RTOS primitives
// =============================================================================
#include "rtos_common.hpp"

// These are the single, real definitions of the handles declared `extern` in
// the header. Every other .cpp file sees them through rtos_common.hpp.
QueueHandle_t      servoQueue    = nullptr;
QueueHandle_t      sensorMailbox = nullptr;
QueueHandle_t      imuMailbox    = nullptr;
SemaphoreHandle_t  serialMutex   = nullptr;
SemaphoreHandle_t  i2cMutex      = nullptr;
EventGroupHandle_t bleEvents     = nullptr;

volatile uint8_t   g_batteryPercent = 0;

void rtos_init(void) {
    // -------------------------------------------------------------------------
    //  xQueueCreate(uxQueueLength, uxItemSize)
    //    uxQueueLength : how many items can sit in the queue at once
    //    uxItemSize    : size in bytes of ONE item (the queue COPIES items in
    //                    and out, so the data is safe even if the sender's copy
    //                    goes out of scope)
    // -------------------------------------------------------------------------

    // Up to 4 pending haptic commands; each item is one ServoCommand struct.
    servoQueue = xQueueCreate(4, sizeof(ServoCommand));

    // Length 1 => "mailbox". We will write it with xQueueOverwrite so it always
    // holds the most recent InputData snapshot.
    sensorMailbox = xQueueCreate(1, sizeof(InputData));

    // Length-1 mailbox for the fused IMU orientation.
    imuMailbox = xQueueCreate(1, sizeof(Orientation));

    // A mutex is a binary semaphore specialised for "ownership" (it tracks which
    // task holds it and supports priority inheritance to avoid priority
    // inversion -- a nice talking point for an interview).
    serialMutex = xSemaphoreCreateMutex();
    i2cMutex    = xSemaphoreCreateMutex();

    // Event group starts with all bits clear (i.e. "not connected").
    bleEvents = xEventGroupCreate();

    // Defensive check: if the heap was too full to allocate any of these, fail
    // loudly rather than dereferencing a null handle later.
    if (!servoQueue || !sensorMailbox || !imuMailbox || !serialMutex || !i2cMutex || !bleEvents) {
        Serial.println("FATAL: failed to create RTOS objects (out of heap?)");
        while (true) { delay(1000); }   // halt; watchdog/visibility
    }
}
