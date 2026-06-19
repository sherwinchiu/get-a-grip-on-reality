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
//  segment 0 (base), segment 1 (knuckle), splay
//
//  ⚠⚠ CRITICAL ADC2-vs-BLE HAZARD ⚠⚠
//  On the ESP32-S3, ADC1 = GPIO1..10 and ADC2 = GPIO11..20. **ADC2 cannot be read
//  reliably while the radio (BLE/Wi-Fi) is on** -- analogRead() on an ADC2 pin
//  returns garbage / stuck-high once BLE is initialised (documented:
//  espressif/arduino-esp32 #3208). Calibration works because it runs BEFORE
//  initBluetooth(); during normal streaming those channels are JUNK.
//
//  Which of THESE pins are ADC2 (broken once BLE is up):
//     thumb  {4,5,6}    -> all ADC1   OK
//     index  {7,15,16}  -> 7 ADC1 ok; 15,16 ADC2 BAD (base ok, so index curl tracks)
//     middle {17,18,8}  -> 17,18 ADC2 BAD; 8 ADC1 ok   (both bend sensors junk)
//     ring   {11,12,13} -> ALL ADC2 BAD                (reads "fully bent" -> false grab!)
//     pinky  {3,9,10}   -> all ADC1   OK
//  FIX = hardware: move the ADC2 sensors onto free ADC1 pins (only GPIO1/GPIO2 are
//  free here, so most need an external I2C ADC e.g. ADS1115), OR accept that
//  middle/ring bend is unreliable while connected. This is NOT fixable in software.
// -----------------------------------------------------------------------------
#ifdef RIGHT_HAND
const static int hall_pins[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER] = { {  4,  5,  6 }, // thumb  ADC1/ADC1/ADC1
                                                                       {  7, 15, 16 }, // index  ADC1/ADC2/ADC2
                                                                       { 17, 18,  8 }, // middle ADC2/ADC2/ADC1
                                                                       { 11, 12, 13 }, // ring   ADC2/ADC2/ADC2  <-- all bad w/ BLE
                                                                       {  3,  9, 10 } };// pinky  ADC1/ADC1/ADC1
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

// NVS namespace for persisted calibration (<=15 chars). Defined up here so the
// calibration routine can also read the PREVIOUS saved ranges to merge with.
static const char* CAL_NS = "hallcal";

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
    logf("\n========================================================\n");
    logf("[CAL] >>> CALIBRATION START (%lu s) <<<\n", (unsigned long)(duration_ms / 1000));
    logf("[CAL] FULLY OPEN your hand, then SLOWLY close it, then open again.\n");
    logf("[CAL] Move every finger through its full range during this window.\n");
    logf("========================================================\n");

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
    int lastShown = -1;
    while (millis() - start < duration_ms) {
        read_hall_raw(raw);
        for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
            for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
                if (raw[i][j] < min_hall_value[i][j]) min_hall_value[i][j] = raw[i][j];
                if (raw[i][j] > max_hall_value[i][j]) max_hall_value[i][j] = raw[i][j];
            }
        // Once-per-second countdown so you can see calibration is alive + timed.
        int remain = (int)((duration_ms - (millis() - start) + 999) / 1000);
        if (remain != lastShown) {
            logf("[CAL]  ...%d s left\n", remain);
            lastShown = remain;
        }
        vTaskDelay(pdMS_TO_TICKS(5));   // ~yield to other tasks while sampling
    }

    // MERGE WITH THE PREVIOUS SAVED CALIBRATION so the stored range only ever
    // GROWS. For each sensor keep the LOWER of (this session's min, last saved
    // min) and the HIGHER of (this session's max, last saved max) -- i.e. the
    // widest range ever seen. This way a better/larger previous calibration is
    // never discarded just because this session's hand motion was smaller, and
    // it also rescues any sensor that didn't move this time.
    {
        Preferences p;
        p.begin(CAL_NS, /*readOnly=*/true);
        if (p.getBool("valid", false)) {
            uint16_t prevMin[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];
            uint16_t prevMax[NUM_HALL_ROWS][HALL_SENSORS_PER_FINGER];
            p.getBytes("min", prevMin, sizeof(prevMin));
            p.getBytes("max", prevMax, sizeof(prevMax));
            for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
                for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j) {
                    if (prevMin[i][j] < min_hall_value[i][j]) min_hall_value[i][j] = prevMin[i][j];
                    if (prevMax[i][j] > max_hall_value[i][j]) max_hall_value[i][j] = prevMax[i][j];
                }
            logf("[CAL] merged with previous saved calibration (kept widest range)\n");
        } else {
            logf("[CAL] no previous calibration to merge -- using this session only\n");
        }
        p.end();
    }

    // Safety: a sensor that never moved (unplugged, or held still) AND had no
    // previous range leaves min == max, which would divide-by-zero inside map().
    // Force a 1-count span.
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        for (uint8_t j = 0; j < HALL_SENSORS_PER_FINGER; ++j)
            if (max_hall_value[i][j] <= min_hall_value[i][j])
                max_hall_value[i][j] = min_hall_value[i][j] + 1;

    static const char* fname[NUM_HALL_ROWS] = { "thumb", "index", "middl", "ring ", "pinky" };
    logf("\n========================================================\n");
    logf("[CAL] >>> CALIBRATION END -- captured config (raw ADC) <<<\n");
    logf("[CAL]        base[min-max]  knuckle[min-max]  splay[min-max]\n");
    for (uint8_t i = 0; i < NUM_HALL_ROWS; ++i)
        logf("[CAL] %s base[%4u-%4u] knuckle[%4u-%4u] splay[%4u-%4u]\n", fname[i],
             min_hall_value[i][0], max_hall_value[i][0],
             min_hall_value[i][1], max_hall_value[i][1],
             min_hall_value[i][2], max_hall_value[i][2]);
    logf("========================================================\n\n");

    hall_save_calibration();   // persist so it survives the next reboot
}

// -----------------------------------------------------------------------------
//  CALIBRATION PERSISTENCE (NVS / flash, via the ESP32 Preferences library)
// -----------------------------------------------------------------------------
//  Preferences stores key/value blobs in a dedicated flash partition that
//  survives reflashing of the app (unless you erase flash). We save the two
//  uint16_t[5][3] tables plus a "valid" marker; on boot we load them back if the
//  marker is set, otherwise the built-in defaults in this file stand.
//  (CAL_NS is defined near the top of this file so hall_calibrate() can use it.)
// -----------------------------------------------------------------------------

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

    // Brief startup delay: creating this (higher-priority) task preempts setup()
    // immediately. Yielding here first lets setup() finish creating ALL tasks and
    // print its [BOOT] lines before we start sampling -- so a problem in the first
    // read is diagnosable (you'll have seen the other tasks come up) instead of
    // silently freezing boot.
    vTaskDelay(pdMS_TO_TICKS(300));
    logf("[hall] task running; high-water=%u words\n",
         (unsigned)uxTaskGetStackHighWaterMark(nullptr));

    // vTaskDelayUntil needs a reference "last wake time". We seed it with the
    // current tick count, then it advances by exactly one period each loop.
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(HALL_PERIOD_MS);

    bool firstPass = true;
    for (;;) {
        read_hall();              // sample + filter the sensors
        if (firstPass) logf("[hall] first read_hall() OK (high-water=%u words)\n",
                            (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        build_package(&pkg);      // pack into a BLE-ready struct
        if (firstPass) { logf("[hall] first build_package() OK -- streaming now\n"); firstPass = false; }
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
