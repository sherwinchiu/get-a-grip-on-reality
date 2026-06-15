// =============================================================================
//  bq25887.cpp  --  BQ25887 charger driver + chargerTask
// =============================================================================
#include "bq25887.hpp"
#include "config.h"

#ifdef ENABLE_CHARGER
#include <Arduino.h>
#include <Wire.h>
#include "rtos_common.hpp"   // i2cMutex, g_batteryPercent
#include "logger.hpp"
#include "protocol_logic.h"  // pure, unit-tested register encoders

// -----------------------------------------------------------------------------
//  Register map (datasheet SLUSD89B). Only the ones we use are named.
// -----------------------------------------------------------------------------
#define REG_VCELL          0x00   // Cell Voltage Regulation Limit (5 mV/LSB, +3.40 V)
#define REG_ICHG           0x01   // Charge Current Limit (50 mA/LSB) + EN_HIZ/EN_ILIM
#define REG_PRECHG_TERM    0x04   // IPRECHG[7:4], ITERM[3:0] (50 mA/LSB, +50 mA)
#define REG_CHG_CTRL1      0x05   // EN_TERM, WATCHDOG[5:4], EN_TIMER, ...
#define REG_CHG_CTRL2      0x06   // ..., EN_CHG (bit3), ...
#define REG_CHG_CTRL3      0x07   // ..., WD_RST (bit6), ...
#define REG_STATUS1        0x0B   // CHRG_STAT[2:0], WD_STAT(3), ...
#define REG_STATUS2        0x0C   // PG_STAT(7), VBUS_STAT[6:4], ...
#define REG_FAULT          0x0E   // VBUS_OVP(7), TSHUT(6), TMR(4)
#define REG_ADC_CTRL       0x15   // ADC_EN(7), ADC_RATE(6), ADC_SAMPLE[5:4]
#define REG_ICHG_ADC       0x19   // 16-bit charge current, 1 mA/LSB (0x19 hi, 0x1A lo)
#define REG_VBAT_ADC       0x1D   // 16-bit pack voltage,    1 mV/LSB (0x1D hi, 0x1E lo)
#define REG_VCELLTOP_ADC   0x1F   // 16-bit top-cell voltage,1 mV/LSB
#define REG_VCELLBOT_ADC   0x26   // 16-bit bottom-cell volt,1 mV/LSB
#define REG_PART_INFO      0x25   // PN[6:3] = 0b0101 for BQ25887

// -----------------------------------------------------------------------------
//  Low-level register access -- each transaction takes the shared I2C mutex so
//  it can't collide with the IMU task on the same bus.
// -----------------------------------------------------------------------------
static bool i2c_lock(void)   { return i2cMutex ? (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) : true; }
static void i2c_unlock(void) { if (i2cMutex) xSemaphoreGive(i2cMutex); }

static bool reg_write(uint8_t reg, uint8_t val) {
    if (!i2c_lock()) return false;
    Wire.beginTransmission(CHARGER_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    bool ok = (Wire.endTransmission() == 0);
    i2c_unlock();
    return ok;
}

static bool reg_read(uint8_t reg, uint8_t* val) {
    if (!i2c_lock()) return false;
    Wire.beginTransmission(CHARGER_I2C_ADDR);
    Wire.write(reg);
    bool ok = (Wire.endTransmission(false) == 0);            // repeated-start
    if (ok && Wire.requestFrom((uint16_t)CHARGER_I2C_ADDR, (uint8_t)1, true) == 1) {
        *val = Wire.read();
    } else {
        ok = false;
    }
    i2c_unlock();
    return ok;
}

// Read a 16-bit big-endian value spanning `reg` (MSB) and reg+1 (LSB).
static bool reg_read16(uint8_t reg, uint16_t* val) {
    if (!i2c_lock()) return false;
    Wire.beginTransmission(CHARGER_I2C_ADDR);
    Wire.write(reg);
    bool ok = (Wire.endTransmission(false) == 0);
    if (ok && Wire.requestFrom((uint16_t)CHARGER_I2C_ADDR, (uint8_t)2, true) == 2) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        *val = ((uint16_t)hi << 8) | lo;
    } else {
        ok = false;
    }
    i2c_unlock();
    return ok;
}

bool bq_init(void) {
    Wire.begin(CHARGER_SDA, CHARGER_SCL);   // safe even if the IMU already began it

    // 1) Confirm it's really a BQ25887 (PN field = 0b0101 in REG25 bits 6:3).
    uint8_t pn;
    if (!reg_read(REG_PART_INFO, &pn)) {
        logf("[BQ] no response at 0x%02X -- charger absent?\n", CHARGER_I2C_ADDR);
        return false;
    }
    if (((pn >> 3) & 0x0F) != 0x05) {
        logf("[BQ] unexpected part id 0x%02X (PN!=0101)\n", pn);
        return false;
    }

    // 2) Charge voltage per cell (REG00). Encoding is unit-tested in protocol_logic.h.
    reg_write(REG_VCELL, glove::bq_vcell_reg(CHARGER_VCELL_MV));

    // 3) Fast-charge current (REG01). Keep EN_ILIM (bit6) set, EN_HIZ clear.
    reg_write(REG_ICHG, 0x40 | glove::bq_ichg_reg(CHARGER_ICHG_MA));

    // 4) Pre-charge + termination currents (REG04): IPRECHG[7:4] | ITERM[3:0].
    reg_write(REG_PRECHG_TERM, glove::bq_prechg_term_reg(CHARGER_IPRECHG_MA, CHARGER_ITERM_MA));

    // 5) I2C watchdog (REG05 bits 5:4): 01 = 40 s if we service it, 00 = disabled.
    uint8_t r5 = 0;
    reg_read(REG_CHG_CTRL1, &r5);
    r5 &= ~0x30;
#ifdef CHARGER_SERVICE_WATCHDOG
    r5 |= 0x10;   // WATCHDOG = 01 (40 s) -- chargerTask must kick it
#else
    r5 |= 0x00;   // WATCHDOG = 00 (disabled) -- settings persist unattended
#endif
    reg_write(REG_CHG_CTRL1, r5);

    // 6) Make sure charging is enabled (REG06 EN_CHG = bit3; default is already 1).
    uint8_t r6 = 0;
    reg_read(REG_CHG_CTRL2, &r6);
    reg_write(REG_CHG_CTRL2, (uint8_t)(r6 | (1 << 3)));

    // 7) Enable the ADC, continuous mode, 15-bit (REG15 ADC_EN = bit7).
    reg_write(REG_ADC_CTRL, 0x80);

    logf("[BQ] BQ25887 configured: %d mV/cell, %d mA charge, term %d mA\n",
         CHARGER_VCELL_MV, CHARGER_ICHG_MA, CHARGER_ITERM_MA);
    return true;
}

bool bq_read_status(ChargerStatus* o) {
    uint8_t s1, s2, fault;
    if (!reg_read(REG_STATUS1, &s1)) { o->present = false; return false; }
    o->present = true;
    reg_read(REG_STATUS2, &s2);
    reg_read(REG_FAULT, &fault);

    o->state      = (ChargeState)(s1 & 0x07);
    o->power_good = (s2 & 0x80) != 0;
    o->fault      = (fault & (0x80 | 0x40 | 0x10)) != 0;   // OVP | thermal | safety-timer

    uint16_t v = 0;
    reg_read16(REG_VBAT_ADC,     &v); o->vbat_mv      = v;
    reg_read16(REG_VCELLTOP_ADC, &v); o->vcell_top_mv = v;
    reg_read16(REG_VCELLBOT_ADC, &v); o->vcell_bot_mv = v;
    reg_read16(REG_ICHG_ADC,     &v); o->ichg_ma      = v;

    // Rough linear state-of-charge from pack voltage. (A real fuel gauge would
    // use a discharge curve + coulomb counting; this is a usable estimate.)
    o->battery_pct = glove::bq_battery_pct(o->vbat_mv, BATT_EMPTY_MV, BATT_FULL_MV);
    return true;
}

static const char* state_str(ChargeState s) {
    switch (s) {
        case CHG_NOT_CHARGING: return "idle";
        case CHG_TRICKLE:      return "trickle";
        case CHG_PRECHARGE:    return "precharge";
        case CHG_FAST_CC:      return "fast-CC";
        case CHG_TAPER_CV:     return "taper-CV";
        case CHG_TOPOFF:       return "top-off";
        case CHG_DONE:         return "done";
        default:               return "?";
    }
}

void chargerTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CHARGER_PERIOD_MS);
    uint32_t tick = 0;

    for (;;) {
#ifdef CHARGER_SERVICE_WATCHDOG
        // Kick the 40 s I2C watchdog every ~20 s so our charge settings persist.
        // (Set WD_RST = bit6 in REG07; it self-clears.)
        if (tick % 20 == 0) {
            uint8_t r7 = 0;
            if (reg_read(REG_CHG_CTRL3, &r7)) reg_write(REG_CHG_CTRL3, (uint8_t)(r7 | 0x40));
        }
#endif
        ChargerStatus st;
        if (bq_read_status(&st)) {
            g_batteryPercent = st.battery_pct;   // rides along in InputData.battery_lvl
            logf("[BQ] %-9s pack=%umV (top=%u bot=%u) Ichg=%umA batt=%u%%%s\n",
                 state_str(st.state), st.vbat_mv, st.vcell_top_mv, st.vcell_bot_mv,
                 st.ichg_ma, st.battery_pct, st.fault ? "  *FAULT*" : "");
        } else {
            logf("[BQ] read failed\n");
        }
        tick++;
        vTaskDelayUntil(&lastWake, period);
    }
}

#else  // ENABLE_CHARGER not defined: no-op stubs so the firmware still links.

bool bq_init(void) { return false; }
bool bq_read_status(ChargerStatus* o) { if (o) o->present = false; return false; }
void chargerTask(void* pvParameters) { vTaskDelete(NULL); }

#endif // ENABLE_CHARGER
