// =============================================================================
//  bq25887.hpp  --  TI BQ25887 2-cell I2C battery charger driver
// =============================================================================
//
//  The BQ25887 is a 2-cell (2S) Li-ion switch-mode charger with USB input, an
//  integrated 16-bit ADC (pack/cell voltages, charge current, die temp) and
//  AUTOMATIC power-path cell balancing. We talk to it over I2C to:
//     * set the charge voltage / current safely for our pack
//     * keep its I2C watchdog fed (so our settings persist)
//     * read back charge state + voltages, and estimate battery %
//
//  All register addresses, scalings and the I2C address (0x6A) below are taken
//  from the datasheet (SLUSD89B, Rev B). Cell balancing is left at the chip's
//  automatic defaults -- it needs no firmware action.
// =============================================================================
#ifndef BQ25887_HPP
#define BQ25887_HPP

#include <stdint.h>

// Decoded charge state, from REG0B CHRG_STAT[2:0].
enum ChargeState {
    CHG_NOT_CHARGING = 0,
    CHG_TRICKLE      = 1,
    CHG_PRECHARGE    = 2,
    CHG_FAST_CC      = 3,   // constant current
    CHG_TAPER_CV     = 4,   // constant voltage
    CHG_TOPOFF       = 5,
    CHG_DONE         = 6,
    CHG_RESERVED     = 7,
};

// A snapshot of what the charger reports.
struct ChargerStatus {
    bool        present;        // device ACKed on the bus
    ChargeState state;
    uint16_t    vbat_mv;        // total pack voltage (both cells)
    uint16_t    vcell_top_mv;   // top cell
    uint16_t    vcell_bot_mv;   // bottom cell
    uint16_t    ichg_ma;        // charge current
    uint8_t     battery_pct;    // rough 0..100 estimate from vbat
    bool        power_good;     // valid input source attached
    bool        fault;          // any of input-OVP / thermal-shutdown / safety-timer
};

// Detect + configure the charger (charge V/I, watchdog, ADC). Returns false if
// the device doesn't respond, so a missing charger never hangs boot.
bool bq_init(void);

// Read status + ADC values into `out`. Returns false on bus error.
bool bq_read_status(ChargerStatus* out);

// Periodic task: services the I2C watchdog, reads status, updates battery %.
void chargerTask(void* pvParameters);

#endif // BQ25887_HPP
