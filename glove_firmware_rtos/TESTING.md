# Firmware Testing — how the glove firmware is verified

Two kinds of verification back this firmware. Knowing *why* each exists (and what
it can and can't prove) is exactly the kind of thing an interviewer drills into.

## 1. Host unit tests of the pure logic (`test/`)

**The idea:** the bug-prone parts of embedded firmware are usually *pure* functions —
byte packing, fixed-point scaling, register encoding, filter math — but they normally sit
tangled with hardware (`Wire`, `analogRead`, the BLE stack) that can't run on a PC, so they
go untested. The fix is to **separate the logic from the hardware-abstraction layer (HAL)**:

- `protocol_logic.h` — pure, dependency-free functions (only `<cstdint>`/`<cmath>`).
- The firmware `.cpp` files **call into it** (`hall.cpp`, `bq25887.cpp`, `imu.cpp`), so it's
  the real code path — not a copy that can drift.
- `test/test_protocol_logic.cpp` includes the *same header* and verifies it with `g++`. No
  ESP32, no flashing, runs in milliseconds.

**Run them:**
```sh
cd glove_firmware_rtos/test
./run_tests.sh          # or: g++ -std=c++17 -I.. test_protocol_logic.cpp -o test_logic && ./test_logic
```

**What's covered (37 checks):**
| Area | What's asserted |
|------|-----------------|
| BLE packing | `pack_u16_le` is **little-endian** (LSB first) and a 32-byte InputData packet lands at the right offsets — this is the byte contract the web app and OpenVR driver decode |
| Hall sensor | clamp → integer-map → invert → sinusoidal curve; endpoints, monotonicity, invert, and the degenerate `min==max` (no divide-by-zero) |
| BQ25887 | charge-voltage / current / pre-charge / termination register encodings vs. the datasheet (e.g. 4.20 V/cell → `0xA0`), status decode, and clamped battery-% estimate |
| IMU fusion | accelerometer tilt (`atan2`) and the complementary-filter blend (`alpha=1` → gyro only, `alpha=0` → accel only) |

One test deliberately documents a 1-LSB truncation artifact at the sine endpoint
(`cos(π/2)` is a hair > 0 in float) — the test asserts real behavior rather than hiding it.

## 2. Full-firmware compile check (the integration gate)

The unit tests prove the *logic*; they can't prove the firmware *builds and links* against
the real ESP32 core, BLE, FreeRTOS, and the IMU/charger libraries. That's a separate gate:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --libraries <your Arduino/libraries> glove_firmware_rtos
```
Current build: ~53% flash, ~9% RAM, clean. (Verified on ESP32 core 3.3.7.)

## What is NOT unit-tested (and why that's honest)

The RTOS wiring (task scheduling, queues, mutexes), the live BLE radio, the ADC reads, and
the servo PWM are **integration/hardware concerns** — you can't meaningfully unit-test them
on a host, and pretending otherwise is how you get a green suite and a dead board. Those are
verified by: the compile gate above, the on-device diagnostics (`loop()` prints free heap +
per-task stack high-water marks), and the Serial Plotter / Simulation-Mode paths.

## Cross-project contract test

The web demo (`../mobile-demo/`) has its own test suite that **reads this firmware's
`bluetooth.hpp`/`.cpp` and asserts the UUIDs and device name match** what the app expects —
so the firmware and the Web Bluetooth app can't silently drift apart. Run it with
`cd ../mobile-demo && node --test`.

## Interview soundbite

> *"I split the firmware so the pure logic — packing, register encoding, sensor fusion —
> lives in a HAL-free header the firmware actually calls, and I unit-test that natively with
> g++ in milliseconds. Hardware-dependent parts are covered by a compile gate and on-device
> diagnostics instead. And a cross-project test reads the firmware headers to guarantee the
> BLE contract stays in sync with the host app."*
