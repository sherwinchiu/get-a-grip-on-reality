// =============================================================================
//  logger.hpp  --  Thread-safe logging over the Serial UART
// =============================================================================
//
//  In the original firmware, logging pushed bytes over BLE. Here we log to the
//  Serial port instead, because the whole point of this rewrite is to learn
//  FreeRTOS -- and Serial is the perfect example of a SHARED RESOURCE that must
//  be protected by a mutex.
//
//  Multiple tasks (hall, ble, servo, the diagnostics loop) all want to print.
//  Without protection, "Hall: 1234" and "BLE connected" can interleave into
//  "HaBLE conl: 12nected34". `logf()` takes serialMutex so each line is atomic.
// =============================================================================
#ifndef LOGGER_HPP
#define LOGGER_HPP

// Initialise the Serial port. Call once from setup() (before tasks start).
void logInit(unsigned long baud);

// printf-style logging, but mutex-protected so output never interleaves.
// Usage: logf("Hall finger %d = %d\n", finger, value);
void logf(const char* fmt, ...);

#endif // LOGGER_HPP
