// =============================================================================
//  protocol_logic.h  --  PURE, hardware-independent firmware logic
// =============================================================================
//
//  WHY THIS HEADER EXISTS
//  ----------------------
//  The interesting, bug-prone parts of embedded firmware are usually pure
//  functions: byte packing, fixed-point scaling, register encoding, filter math.
//  But they normally sit tangled up with the hardware (Wire, analogRead, the BLE
//  stack), which can't run on a PC -- so they go untested.
//
//  This header pulls that logic OUT of the .cpp files into dependency-free
//  functions (only <cstdint>/<cmath>). The firmware includes it and calls these
//  (so this IS the real code path, not a copy), and the host unit tests in
//  test/ include the same header and verify it with g++ -- no ESP32 required.
//
//  This is the standard "separate logic from the HAL" pattern for testable
//  embedded code. Everything here is `inline` so it can live in a header.
// =============================================================================
#ifndef PROTOCOL_LOGIC_H
#define PROTOCOL_LOGIC_H

#include <cstdint>
#include <cmath>

namespace glove {

// pi/2 as a literal so we don't depend on Arduino's HALF_PI macro or M_PI.
static constexpr double HALF_PI_D = 1.57079632679489661923;
static constexpr double RAD2DEG   = 57.29577951308232;

// ---------------------------------------------------------------------------
//  BLE PACKET PACKING  (glove -> app)
// ---------------------------------------------------------------------------
//  Store a 16-bit value little-endian (LSB first) at byte offset `idx`. This is
//  exactly what the host app (glove-protocol.js) and the OpenVR driver decode,
//  so the byte order MUST be LSB-then-MSB. (Replaces the old
//  writeUnsignedShortToCharArrayLE.)
inline void pack_u16_le(uint16_t value, uint8_t* buf, int idx) {
    buf[idx]     = (uint8_t)(value & 0xFF);          // LSB at lower address
    buf[idx + 1] = (uint8_t)((value >> 8) & 0xFF);   // MSB next
}

//  The index finger has no splay magnet wired, so its splay slot is skipped when
//  packing -- a subtle layout detail the host MUST agree on. Centralised + tested.
static constexpr int HALL_IGNORE_SPLAY_FINGER = 1;   // 0=thumb .. 4=pinky

//  Pack the bend_angle[20] sub-array: 2 segments per finger (thumb->pinky),
//  little-endian. hall is [finger][sensor]: sensor 0,1 = bend segments, 2 = splay.
inline void pack_bend(uint8_t* bend_angle, const uint16_t hall[5][3]) {
    int bc = 0;
    for (int f = 0; f < 5; ++f) {
        pack_u16_le(hall[f][0], bend_angle, bc); bc += 2;
        pack_u16_le(hall[f][1], bend_angle, bc); bc += 2;
    }
}

//  Pack the splay[8] sub-array: 4 values, SKIPPING the index finger. On the
//  current hardware only the thumb reports a real splay; the rest are pinned to
//  mid-scale (2048). Bit-identical to the firmware's build_package().
inline void pack_splay(uint8_t* splay, const uint16_t hall[5][3]) {
    int sc = 0;
    for (int f = 0; f < 5; ++f) {
        if (f == HALL_IGNORE_SPLAY_FINGER) continue;          // no sensor wired
        uint16_t v = (f > 0) ? (uint16_t)2048 : hall[0][2];   // only thumb is real
        pack_u16_le(v, splay, sc); sc += 2;
    }
}

// ---------------------------------------------------------------------------
//  HALL SENSOR NORMALISATION
// ---------------------------------------------------------------------------
//  Take a raw averaged ADC reading and turn it into a 0..4095 bend value:
//    1) clamp into the calibrated [mn, mx] range
//    2) integer-map to 0..4096 (same formula as Arduino's map())
//    3) optionally invert (magnet orientation)
//    4) apply the linear->sinusoidal response curve (gentler at the start)
//  Bit-identical to the math that used to live inline in read_hall().
inline uint16_t hall_normalize(uint16_t raw, uint16_t mn, uint16_t mx, bool invert) {
    long t = raw < mn ? mn : (raw > mx ? mx : raw);                 // constrain
    long m = (mx == mn) ? 0 : (long)(t - mn) * 4096L / (long)(mx - mn);  // map(...,0,4096)
    if (invert) m = 4095 - m;
    double s = (1.0 - std::cos((double)m * HALF_PI_D / 4095.0)) * 4095.0;
    if (s < 0.0) s = 0.0;
    return (uint16_t)(uint32_t)s;
}

// ---------------------------------------------------------------------------
//  BQ25887 CHARGER REGISTER ENCODING  (datasheet SLUSD89B)
//  Centralising these here means the unit tests guard the values that decide
//  how a real battery gets charged -- the highest-stakes math in the firmware.
// ---------------------------------------------------------------------------
//  REG00 VCELLREG: per-cell charge voltage. offset 3.40 V, 5 mV/step. 4200->0xA0(160)
inline uint8_t bq_vcell_reg(uint16_t mv)        { return (uint8_t)((mv - 3400) / 5); }
//  REG01 ICHG[5:0]: fast-charge current, 50 mA/step. 1500->30
inline uint8_t bq_ichg_reg(uint16_t ma)         { return (uint8_t)((ma / 50) & 0x3F); }
//  REG04 IPRECHG[7:4] / ITERM[3:0]: offset 50 mA, 50 mA/step. 150->2
inline uint8_t bq_iprechg_reg(uint16_t ma)      { return (uint8_t)(((ma - 50) / 50) & 0x0F); }
inline uint8_t bq_iterm_reg(uint16_t ma)        { return (uint8_t)(((ma - 50) / 50) & 0x0F); }
inline uint8_t bq_prechg_term_reg(uint16_t iprechg_ma, uint16_t iterm_ma) {
    return (uint8_t)((bq_iprechg_reg(iprechg_ma) << 4) | bq_iterm_reg(iterm_ma));
}
//  REG0B CHRG_STAT[2:0]: 0 idle,1 trickle,2 precharge,3 fast-CC,4 taper-CV,5 topoff,6 done
inline int bq_decode_chrg_stat(uint8_t reg0b)   { return reg0b & 0x07; }

//  Rough linear state-of-charge from pack voltage, clamped 0..100.
inline uint8_t bq_battery_pct(uint16_t vbat_mv, uint16_t empty_mv, uint16_t full_mv) {
    if (full_mv <= empty_mv) return 0;
    long pct = (long)(vbat_mv - empty_mv) * 100 / (long)(full_mv - empty_mv);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

// ---------------------------------------------------------------------------
//  IMU SENSOR FUSION  (complementary filter)
// ---------------------------------------------------------------------------
//  Absolute tilt from the gravity vector (accelerometer), in degrees.
inline double accel_roll_deg(double ax, double ay, double az)  { (void)ax; return std::atan2(ay, az) * RAD2DEG; }
inline double accel_pitch_deg(double ax, double ay, double az) { return std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD2DEG; }

//  One complementary-filter step: trust the gyro short-term, let the
//  accelerometer slowly correct drift. alpha in [0,1] (e.g. 0.98 = 98% gyro).
inline double comp_filter(double prev_deg, double gyro_dps, double dt, double accel_deg, double alpha) {
    return alpha * (prev_deg + gyro_dps * dt) + (1.0 - alpha) * accel_deg;
}

// ---------------------------------------------------------------------------
//  POWER-ON SELF-TEST thresholds (pure pass/fail predicates, unit-tested)
// ---------------------------------------------------------------------------
//  A connected Hall channel reads somewhere mid-range; a disconnected ADC pin
//  tends to pin near the rails (0 or 4095). Not foolproof, but reliably catches
//  dead/unwired channels at bring-up.
static constexpr uint16_t HALL_RAIL_LOW  = 50;
static constexpr uint16_t HALL_RAIL_HIGH = 4045;
inline bool hall_channel_healthy(uint16_t raw) {
    return raw > HALL_RAIL_LOW && raw < HALL_RAIL_HIGH;
}

//  When stationary the accelerometer should read ~1 g total. A magnitude well
//  off 1 g means bad data, wrong scaling, or the glove wasn't held still.
inline double imu_accel_magnitude_g(double ax, double ay, double az) {
    return std::sqrt(ax * ax + ay * ay + az * az);
}
inline bool imu_accel_ok(double ax, double ay, double az, double tol = 0.20) {
    double m = imu_accel_magnitude_g(ax, ay, az);
    return m > (1.0 - tol) && m < (1.0 + tol);
}

//  2S Li-ion pack voltage sanity: ~6.0 V (near-empty) .. ~8.6 V (full + margin).
inline bool pack_voltage_plausible(uint16_t vbat_mv) {
    return vbat_mv > 5000 && vbat_mv < 8800;
}

} // namespace glove

#endif // PROTOCOL_LOGIC_H
