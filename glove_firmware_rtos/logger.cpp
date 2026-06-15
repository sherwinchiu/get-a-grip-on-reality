// =============================================================================
//  logger.cpp  --  Implementation of the mutex-guarded logger
// =============================================================================
#include "logger.hpp"

#include <Arduino.h>
#include <stdarg.h>          // va_list / va_start for the printf-style API
#include "config.h"          // PLOT_HALL
#include "rtos_common.hpp"   // gives us serialMutex

void logInit(unsigned long baud) {
    Serial.begin(baud);
}

void logf(const char* fmt, ...) {
#ifdef PLOT_HALL
    // Plotting mode: the Serial port is reserved for clean numeric data, so we
    // drop all text logs. (Comment out PLOT_HALL in config.h to get logs back.)
    (void)fmt;
    return;
#else
    // 1) Format the message into a local buffer FIRST. We do the (potentially
    //    slow) string formatting OUTSIDE the lock so we hold the mutex for as
    //    short a time as possible -- holding a lock longer than necessary is a
    //    classic concurrency mistake.
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // 2) Now grab exclusive access to Serial. xSemaphoreTake blocks this task
    //    until the mutex is free (or the timeout elapses). pdMS_TO_TICKS converts
    //    milliseconds into the scheduler's native "tick" unit.
    //
    //    If serialMutex hasn't been created yet (very early boot), just print
    //    directly -- there are no other tasks running at that point anyway.
    if (serialMutex == nullptr) {
        Serial.print(buf);
        return;
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print(buf);
        // 3) ALWAYS give the mutex back, or every other task blocks forever.
        xSemaphoreGive(serialMutex);
    }
    // If we failed to take it within 50 ms we simply drop the log line rather
    // than block a real-time task. Logging must never jeopardise control loops.
#endif // PLOT_HALL
}
