// =============================================================================
//  hall.cpp
// =============================================================================
#include "hall.hpp"

#include <Arduino.h>
#include <sys/types.h>
#include <Preferences.h>     // ESP32 NVS wrapper -- persists calibration to flash
#include "config.h"
#include "rtos_common.hpp"   // sensorMailbox, g_batteryPercent
#include "logger.hpp"
#include "protocol_logic.h"  // pure, unit-tested packing + normalisation

// -----------------------------------------------------------------------------
//  Pin maps + calibration tables (identical to the original firmware).
//  segment 0, segment 1, splay
// -----------------------------------------------------------------------------
#ifdef RIGHT_HAND
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { {  4,  5,  6 }, // thumb
                                                                       {  7, 15, 16 }, // index
                                                                       { 17, 18,  8 }, // middle
                                                                       { 11, 12, 13 }, // ring
                                                                       {  3,  9, 10 } };// pinky
#elif defined(LEFT_HAND)
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { {  3,  9, 10 },
                                                                       { 11, 12, 13 },
                                                                       { 17, 18,  8 },
                                                                       {  7, 15, 16 },
                                                                       {  4,  5,  6 } };
#else
#error "Left or right hand not selected in config.h"
#endif

                                                               // Base, Knuckle, Splay
uint8_t invert_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 0, 1, 0 },   // Thumb
                                                                      { 0, 0, 0 },   // Index
                                                                      { 0, 1, 0 },   // Middle
                                                                      { 0, 1, 0 },   // Ring
                                                                      { 1, 1, 0 } }; // Pinky

uint16_t min_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 1080, 2450, 1050 },
                                                                    { 1500, 1550, 4095 },
                                                                    { 1610, 1970, 1578 },
                                                                    { 1420, 2050, 1480 },
                                                                    { 2250, 2030, 2315 } };

uint16_t max_hall_value[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 1150, 2660, 1650 },
                                                                    { 1620, 1960, 4096 },
                                                                    { 1770, 2550, 1597 },
                                                                    { 1780, 2570, 1530 },
                                                                    { 2680, 2450, 2340 } };

// Internal, processed sensor values [finger][sensor], range 0..4095.
// Only the hall task writes this, and it copies the result into a packet before
// publishing, so no extra locking is needed on it.
static unsigned short hall[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];

void init_hall(void) {
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
            pinMode(hall_pins[i][j], INPUT);
}

// Burst-sample every sensor NUM_SAMPLES_AVG times and average, returning the RAW
// (un-clamped, un-mapped) ADC value per sensor. Shared by the normal read path
// AND the calibration routine so both operate on identical raw numbers.
// (Accumulating as we go avoids the old ~960-byte stack buffer.)
static void read_hall_raw(uint16_t out[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER]) {
    uint32_t acc[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { { 0 } };
    // Interleave: sample every sensor each pass, repeat -- same ordering as the
    // original so the averaging window behaves identically.
    for (uint8_t s = 0; s < NUM_SAMPLES_AVG; ++s)
        for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
            for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
                acc[i][j] += analogRead(hall_pins[i][j]);
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
            out[i][j] = (uint16_t)(acc[i][j] / NUM_SAMPLES_AVG);
}

// Public wrapper so the self-test can read raw ADC values (read_hall_raw is static).
void hall_sample_raw(uint16_t out[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER]) {
    read_hall_raw(out);
}

void read_hall(void) {
    uint16_t raw[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];
    read_hall_raw(raw);

    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i) {
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
            // Clamp -> map to 0..4095 -> invert -> sinusoidal response.
            // (Pure, unit-tested math lives in protocol_logic.h.)
            hall[i][j] = glove::hall_normalize(raw[i][j], min_hall_value[i][j],
                                               max_hall_value[i][j], invert_hall_value[i][j]);
        }
    }
}

void hall_calibrate(uint32_t duration_ms) {
    logf("[CAL] Open, then SLOWLY close your hand to calibrate (%lu s)...\n",
         (unsigned long)(duration_ms / 1000));

    // Reset bounds to the OPPOSITE extremes so the open/close motion fills in the
    // true min and max actually seen this session.
    //   (BUGFIX vs original: the old read_hall_calibration() compared the already
    //    processed/sine-mapped hall[][] value against the raw-range min/max tables
    //    -- mixing units. Here we calibrate on the RAW averaged ADC value, which
    //    is exactly what read_hall() later clamps and maps against.)
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
            min_hall_value[i][j] = 4095;
            max_hall_value[i][j] = 0;
        }

    uint16_t raw[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];
    uint32_t start = millis();
    while (millis() - start < duration_ms) {
        read_hall_raw(raw);
        for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
            for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
                if (raw[i][j] < min_hall_value[i][j]) min_hall_value[i][j] = raw[i][j];
                if (raw[i][j] > max_hall_value[i][j]) max_hall_value[i][j] = raw[i][j];
            }
        vTaskDelay(pdMS_TO_TICKS(5));   // ~yield to other tasks while sampling
    }

    // Safety: a sensor that never moved (unplugged, or held still) leaves
    // min == max, which would divide-by-zero inside map(). Force a 1-count span.
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
            if (max_hall_value[i][j] <= min_hall_value[i][j])
                max_hall_value[i][j] = min_hall_value[i][j] + 1;

    logf("[CAL] done -- captured min/max (raw ADC) per finger:\n");
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        logf("  f%u: base[%u-%u] knuckle[%u-%u] splay[%u-%u]\n", i,
             min_hall_value[i][0], max_hall_value[i][0],
             min_hall_value[i][1], max_hall_value[i][1],
             min_hall_value[i][2], max_hall_value[i][2]);

    hall_save_calibration();   // persist so it survives the next reboot
}

// -----------------------------------------------------------------------------
//  CALIBRATION PERSISTENCE (NVS / flash, via the ESP32 Preferences library)
// -----------------------------------------------------------------------------
//  Preferences stores key/value blobs in a dedicated flash partition that
//  survives reflashing of the app (unless you erase flash). We save the two
//  uint16_t[5][3] tables plus a "valid" marker; on boot we load them back if the
//  marker is set, otherwise the built-in defaults in this file stand.
// -----------------------------------------------------------------------------
static const char* CAL_NS = "hallcal";   // NVS namespace (<=15 chars)

void hall_save_calibration(void) {
    Preferences p;
    p.begin(CAL_NS, /*readOnly=*/false);
    p.putBytes("min", min_hall_value, sizeof(min_hall_value));
    p.putBytes("max", max_hall_value, sizeof(max_hall_value));
    p.putBool("valid", true);
    p.end();
    logf("[CAL] saved to flash (NVS)\n");
}

void hall_load_calibration(void) {
    Preferences p;
    p.begin(CAL_NS, /*readOnly=*/true);
    bool valid = p.getBool("valid", false);
    if (valid) {
        p.getBytes("min", min_hall_value, sizeof(min_hall_value));
        p.getBytes("max", max_hall_value, sizeof(max_hall_value));
        logf("[CAL] loaded saved calibration from flash\n");
    } else {
        logf("[CAL] no saved calibration -- using built-in default ranges\n");
    }
    p.end();
}

void hall_clear_calibration(void) {
    Preferences p;
    p.begin(CAL_NS, /*readOnly=*/false);
    p.clear();                 // wipe this namespace
    p.end();
    logf("[CAL] cleared saved calibration (defaults will be used next boot)\n");
}

void build_package(InputData* out) {
    // Bend + splay packing (incl. the index-finger splay skip) lives in the
    // unit-tested protocol_logic.h, so the firmware-side byte layout is verified
    // to match what the host app / OpenVR driver decode.
    glove::pack_bend(out->bend_angle, hall);
    glove::pack_splay(out->splay, hall);

    out->battery_lvl[0] = g_batteryPercent;   // updated by the charger task (0 if no charger)
    out->button         = 0;
    out->joystick[0]    = 0;
    out->joystick[1]    = 0;
    // Orientation defaults to 0; bleTask overlays the latest IMU value (if any)
    // just before transmitting, so these are only seen if the IMU is disabled.
    out->roll  = 0.0f;
    out->pitch = 0.0f;
    out->yaw   = 0.0f;
}

// -----------------------------------------------------------------------------
//  SERIAL PLOTTER OUTPUT
// -----------------------------------------------------------------------------
//  The Arduino Serial Plotter graphs lines of "label:value" pairs (space or tab
//  separated, one newline-terminated sample per line). We take serialMutex so a
//  sample never interleaves with another task's output. With PLOT_HALL defined,
//  logf() is silenced (see logger.cpp) so the plotter only ever sees numbers.
// -----------------------------------------------------------------------------
void plot_hall(void) {
#ifdef PLOT_HALL
    if (serialMutex == nullptr) return;
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  #if PLOT_HALL_FINGER < 0
    // All fingers: 15 named traces, e.g. "f0s0:1234 f0s1:..."
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
            Serial.printf("f%us%u:%u ", i, j, hall[i][j]);
  #else
    // One finger's 3 sensors: "base:.. knuckle:.. splay:.."
    Serial.printf("base:%u knuckle:%u splay:%u",
                  hall[PLOT_HALL_FINGER][0],
                  hall[PLOT_HALL_FINGER][1],
                  hall[PLOT_HALL_FINGER][2]);
  #endif
    Serial.println();
    xSemaphoreGive(serialMutex);
#endif // PLOT_HALL
}

// -----------------------------------------------------------------------------
//  THE TASK : periodic producer
// -----------------------------------------------------------------------------
void hallTask(void* pvParameters) {
    InputData pkg;

    // vTaskDelayUntil needs a reference "last wake time". We seed it with the
    // current tick count, then it advances by exactly one period each loop.
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(HALL_PERIOD_MS);

    for (;;) {
        read_hall();              // sample + filter the sensors
        build_package(&pkg);      // pack into a BLE-ready struct
        plot_hall();              // stream to Serial Plotter (no-op unless PLOT_HALL)

        // Publish the latest snapshot. xQueueOverwrite ALWAYS succeeds on a
        // length-1 queue: if the BLE task hasn't consumed the previous packet,
        // we simply replace it with this fresher one. Real-time data should be
        // current, not queued up stale.
        xQueueOverwrite(sensorMailbox, &pkg);

        // Sleep until the NEXT period boundary. Unlike vTaskDelay (which delays
        // a fixed amount AFTER the work finishes, letting jitter accumulate),
        // vTaskDelayUntil anchors to an absolute schedule -> rock-steady 50 Hz.
        vTaskDelayUntil(&lastWake, period);
    }
}
