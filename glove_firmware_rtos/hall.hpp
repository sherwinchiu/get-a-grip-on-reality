// =============================================================================
//  hall.hpp  --  Finger-bend / splay sensing via Hall-effect sensors
// =============================================================================
//
//  5 fingers x 3 sensors each:
//     index 0 -> base segment bend
//     index 1 -> knuckle segment bend
//     index 2 -> splay (sideways spread between fingers)
//
//  The hall TASK samples these on a fixed period, builds an InputData packet,
//  and publishes it to the sensorMailbox queue for the BLE task to transmit.
// =============================================================================
#ifndef HALL_HPP
#define HALL_HPP

#include <stdint.h>   // uint32_t etc. (this header may be parsed before Arduino.h)
#include "shared.h"   // InputData

#define NUM_HALL_ROWS            5   // 5 fingers
#define HALL_SENSORS_PER_FINGER  3   // 2 bend + 1 splay
#define NUM_SAMPLES_AVG          32  // software averaging filter depth
#define IGNORE_SPLAY_FINGER      1   // index finger has no splay sensor wired

// Hardware setup. Call once from setup().
void init_hall(void);

// Take a fresh reading of all sensors into the internal `hall[][]` buffer.
void read_hall(void);

// Sample the RAW (un-mapped) averaged ADC value of every sensor. Used by the
// power-on self-test to check each Hall channel is wired and reading sanely.
void hall_sample_raw(uint16_t out[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER]);

// Interactive min/max calibration: for `duration_ms`, the user opens & closes
// their hand and we record the extreme raw ADC value on every sensor into the
// min/max tables. Call from setup() BEFORE the tasks start (it blocks). Matches
// the original hall_callibration(), but corrected to compare RAW readings.
// Automatically persists the result to flash (NVS) on completion.
void hall_calibrate(uint32_t duration_ms);

// Load previously-saved calibration from flash (NVS). If none is stored, the
// hard-coded default ranges in hall.cpp are kept. Call from setup().
void hall_load_calibration(void);

// Persist the current min/max tables to flash (NVS). Called automatically by
// hall_calibrate(), exposed here in case you want to save explicitly.
void hall_save_calibration(void);

// Forget the saved calibration so the next boot falls back to the hard-coded
// defaults. Useful as a "reset to factory" action.
void hall_clear_calibration(void);

// Build a transmit-ready InputData packet from the latest `hall[][]` values.
// (Replaces the old construct_package(); now writes into a caller-owned struct
//  instead of a global, which is cleaner for passing through a queue.)
void build_package(InputData* out);

// Stream the latest hall values in Arduino Serial Plotter format. Compiled in
// only when PLOT_HALL is defined in config.h; called each cycle by hallTask.
void plot_hall(void);

// -----------------------------------------------------------------------------
//  hallTask : PERIODIC producer task.
//  Runs every HALL_PERIOD_MS using vTaskDelayUntil for jitter-free timing,
//  reads sensors, builds a packet, and overwrites the sensorMailbox.
// -----------------------------------------------------------------------------
void hallTask(void* pvParameters);

#endif // HALL_HPP
