// =============================================================================
//  test_protocol_logic.cpp  --  host unit tests for the firmware's pure logic
// =============================================================================
//
//  Compiles and runs on a PC with g++ (NO ESP32, no Arduino). It includes the
//  exact same protocol_logic.h the firmware uses, so these tests cover the real
//  code path -- packet packing, hall normalisation, BQ25887 register encoding,
//  and the IMU complementary filter.
//
//  Build & run:
//     g++ -std=c++17 -I.. test_protocol_logic.cpp -o test_logic && ./test_logic
//  (or just run  ./run_tests.sh  in this folder)
// =============================================================================
#include "../protocol_logic.h"

#include <cstdio>
#include <cmath>
#include <cstdint>

using namespace glove;

// ---- tiny test harness (no framework) ----
static int g_checks = 0, g_fails = 0;
#define CHECK(cond, msg) do { \
    ++g_checks; \
    if (!(cond)) { ++g_fails; std::printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)
static bool approx(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

// =============================================================================
int main() {
    std::printf("firmware protocol_logic.h unit tests\n");

    // ---- 1. LE packing (must match host app glove-protocol.js + the driver) ----
    {
        uint8_t buf[4] = {0};
        pack_u16_le(0x0ABC, buf, 0);            // 2748
        CHECK(buf[0] == 0xBC, "LE: LSB at low address");
        CHECK(buf[1] == 0x0A, "LE: MSB at high address");
        pack_u16_le(0xFFFF, buf, 2);
        CHECK(buf[2] == 0xFF && buf[3] == 0xFF, "LE: max value");
    }

    // ---- 2. Build a full 32-byte InputData packet and verify the contract ----
    {
        uint8_t pkt[32] = {0};
        pkt[0] = 87;                            // battery
        uint16_t bend[10] = {100,200,300,400,500,600,700,800,900,1000};
        int bc = 0;
        for (int f = 0; f < 5; ++f) {           // bend_angle[20] starts at byte 1
            pack_u16_le(bend[f*2],   pkt + 1, bc);     bc += 2;
            pack_u16_le(bend[f*2+1], pkt + 1, bc);     bc += 2;
        }
        // decode finger 2's PIP (index 5 in bend[]) back out at its byte offset
        int off = 1 + 5 * 2;                    // byte 11..12
        uint16_t decoded = pkt[off] | (pkt[off+1] << 8);
        CHECK(decoded == 600, "packet: bend value round-trips at correct offset");
        CHECK(pkt[0] == 87, "packet: battery at byte 0");
        CHECK(bc == 20, "packet: bend occupies exactly 20 bytes");
    }

    // ---- 2b. Bend + splay packing (incl. the index-finger splay skip) ----
    {
        // [finger][sensor]: sensor 0,1 = bend segments, 2 = splay
        uint16_t hall[5][3] = {
            { 111,  222,  333 },   // thumb  (splay 333 is the ONLY real splay)
            { 444,  555,  666 },   // index  (splay 666 must be SKIPPED)
            { 777,  888,  999 },   // middle
            {1111, 1222, 1333 },   // ring
            {1444, 1555, 1666 },   // pinky
        };
        uint8_t bend[20] = {0}, splay[8] = {0};
        pack_bend(bend, hall);
        pack_splay(splay, hall);

        auto rd = [](const uint8_t* b, int i){ return (uint16_t)(b[i] | (b[i+1] << 8)); };
        // bend: finger f occupies bytes [f*4 .. f*4+3]
        for (int f = 0; f < 5; ++f) {
            CHECK(rd(bend, f*4)     == hall[f][0], "pack_bend: segment 0 at right offset");
            CHECK(rd(bend, f*4 + 2) == hall[f][1], "pack_bend: segment 1 at right offset");
        }
        // splay: 4 values = thumb(real), middle(2048), ring(2048), pinky(2048)
        CHECK(rd(splay, 0) == 333,  "pack_splay: slot 0 = thumb (real)");
        CHECK(rd(splay, 2) == 2048, "pack_splay: slot 1 = middle (centred, index skipped)");
        CHECK(rd(splay, 4) == 2048, "pack_splay: slot 2 = ring (centred)");
        CHECK(rd(splay, 6) == 2048, "pack_splay: slot 3 = pinky (centred)");
        // the index finger's splay (666) must NOT appear anywhere
        bool has666 = false;
        for (int i = 0; i < 8; i += 2) if (rd(splay, i) == 666) has666 = true;
        CHECK(!has666, "pack_splay: index-finger splay is skipped, not packed");
    }

    // ---- 3. Hall normalisation ----
    {
        CHECK(hall_normalize(1000, 1000, 2000, false) == 0, "hall: at min -> 0");
        uint16_t atMax = hall_normalize(2000, 1000, 2000, false);
        CHECK(atMax >= 4095, "hall: at max -> ~4095");
        CHECK(hall_normalize(500, 1000, 2000, false) == 0, "hall: below min clamps to 0");
        CHECK(hall_normalize(9999, 1000, 2000, false) >= 4095, "hall: above max clamps to max");
        // invert: min should now read high. (At the sine endpoint cos(pi/2) is a
        // hair > 0 in float, so (1-cos)*4095 truncates to 4094 -- a 1-LSB artifact
        // that's real in the firmware too. We assert "reads high", not exactly 4095.)
        CHECK(hall_normalize(1000, 1000, 2000, true) >= 4094, "hall: inverted min -> high");
        // monotonic increasing across the range (non-inverted)
        uint16_t a = hall_normalize(1200, 1000, 2000, false);
        uint16_t b = hall_normalize(1600, 1000, 2000, false);
        CHECK(a < b, "hall: monotonic increasing");
        // degenerate min==max must not divide by zero / crash
        (void)hall_normalize(1500, 1500, 1500, false);
        CHECK(true, "hall: min==max does not crash");
    }

    // ---- 4. BQ25887 register encoding (datasheet SLUSD89B) ----
    {
        CHECK(bq_vcell_reg(4200) == 0xA0, "bq: 4.20V/cell -> 0xA0 (datasheet default)");
        CHECK(bq_vcell_reg(3400) == 0x00, "bq: 3.40V/cell -> 0x00 (offset)");
        CHECK(bq_vcell_reg(4600) == 0xF0, "bq: 4.60V/cell -> 0xF0 (max, 240)");
        CHECK(bq_ichg_reg(1500) == 30,    "bq: 1500mA charge -> 30");
        CHECK(bq_ichg_reg(500)  == 10,    "bq: 500mA charge -> 10");
        CHECK(bq_iterm_reg(150) == 2,     "bq: 150mA term -> 2");
        CHECK(bq_iprechg_reg(150) == 2,   "bq: 150mA prechg -> 2");
        CHECK(bq_prechg_term_reg(150,150) == 0x22, "bq: prechg|term -> 0x22 (reset default)");
        CHECK(bq_decode_chrg_stat(0x00) == 0, "bq: status idle");
        CHECK(bq_decode_chrg_stat(0x03) == 3, "bq: status fast-CC");
        CHECK(bq_decode_chrg_stat(0xF6) == 6, "bq: status done (masks upper bits)");
    }

    // ---- 5. Battery percentage estimate ----
    {
        CHECK(bq_battery_pct(8400, 6000, 8400) == 100, "batt: full -> 100%");
        CHECK(bq_battery_pct(6000, 6000, 8400) == 0,   "batt: empty -> 0%");
        CHECK(bq_battery_pct(7200, 6000, 8400) == 50,  "batt: midpoint -> 50%");
        CHECK(bq_battery_pct(5000, 6000, 8400) == 0,   "batt: below empty clamps to 0");
        CHECK(bq_battery_pct(9000, 6000, 8400) == 100, "batt: above full clamps to 100");
        CHECK(bq_battery_pct(7000, 8000, 8000) == 0,   "batt: degenerate range -> 0 (no div0)");
    }

    // ---- 6. IMU sensor fusion ----
    {
        // hand flat, gravity on +Z: roll and pitch ~ 0
        CHECK(approx(accel_roll_deg(0, 0, 1), 0.0), "imu: flat -> roll 0");
        CHECK(approx(accel_pitch_deg(0, 0, 1), 0.0), "imu: flat -> pitch 0");
        // rolled 90 deg: gravity on +Y -> roll ~ +90
        CHECK(approx(accel_roll_deg(0, 1, 0), 90.0, 0.01), "imu: gravity +Y -> roll +90");
        // pitched: gravity on -X -> pitch ~ +90
        CHECK(approx(accel_pitch_deg(-1, 0, 0), 90.0, 0.01), "imu: gravity -X -> pitch +90");

        // complementary filter: alpha=1 => pure gyro integration
        CHECK(approx(comp_filter(10.0, 5.0, 0.1, 999.0, 1.0), 10.5), "imu: alpha=1 -> gyro only");
        // alpha=0 => snaps to the accelerometer angle
        CHECK(approx(comp_filter(10.0, 5.0, 0.1, 42.0, 0.0), 42.0), "imu: alpha=0 -> accel only");
        // alpha=0.98 blend
        double r = comp_filter(0.0, 0.0, 0.01, 10.0, 0.98);
        CHECK(approx(r, 0.2), "imu: alpha=0.98 blends 2% of accel");
    }

    // ---- 7. Power-on self-test thresholds ----
    {
        // Hall channel health: railed values flagged, mid-range OK.
        CHECK(hall_channel_healthy(2000) == true,  "post: mid-range hall channel healthy");
        CHECK(hall_channel_healthy(0)    == false, "post: 0 (railed low) unhealthy");
        CHECK(hall_channel_healthy(4095) == false, "post: 4095 (railed high) unhealthy");
        CHECK(hall_channel_healthy(50)   == false, "post: at low rail bound unhealthy");
        CHECK(hall_channel_healthy(51)   == true,  "post: just inside low rail healthy");

        // Accelerometer magnitude: ~1 g passes, far off fails.
        CHECK(approx(imu_accel_magnitude_g(0, 0, 1), 1.0), "post: |g| of (0,0,1) == 1");
        CHECK(imu_accel_ok(0, 0, 1.0)  == true,  "post: stationary 1g passes");
        CHECK(imu_accel_ok(0.1, 0, 0.99) == true, "post: slightly tilted ~1g passes");
        CHECK(imu_accel_ok(0, 0, 0.5)  == false, "post: 0.5g (freefall-ish) fails");
        CHECK(imu_accel_ok(2, 2, 2)    == false, "post: 3.4g (shaking) fails");

        // Pack voltage plausibility for a 2S Li-ion.
        CHECK(pack_voltage_plausible(7400) == true,  "post: 7.4V pack plausible");
        CHECK(pack_voltage_plausible(8400) == true,  "post: 8.4V full plausible");
        CHECK(pack_voltage_plausible(3000) == false, "post: 3.0V (1 cell?) implausible");
        CHECK(pack_voltage_plausible(0)    == false, "post: 0V implausible");
        CHECK(pack_voltage_plausible(9000) == false, "post: 9.0V (overvoltage) implausible");
    }

    // ---- summary ----
    std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
    if (g_fails == 0) std::printf("ALL FIRMWARE LOGIC TESTS PASSED\n");
    return g_fails == 0 ? 0 : 1;
}
