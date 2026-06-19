# VR Haptic Glove — System Overview & Interview Prep

A single place to study the whole project end-to-end and rehearse the questions an
interviewer is likely to drill. Deeper detail lives in the per-component docs:

- Firmware architecture → [`glove_firmware_rtos/RTOS_GUIDE.md`](glove_firmware_rtos/RTOS_GUIDE.md)
- Firmware testing → [`glove_firmware_rtos/TESTING.md`](glove_firmware_rtos/TESTING.md)
- Web Bluetooth demo → [`mobile-demo/README.md`](mobile-demo/README.md)

---

## 1. The system, end to end

```
  ┌──────────────────────── THE GLOVE (wearable) ────────────────────────┐
  │  14 Hall-effect sensors      IIM-42652 IMU       5 servos             │
  │  (finger bend + splay)       (accel + gyro)      (force feedback)     │
  │        │                          │                   ▲               │
  │        ▼                          ▼                   │               │
  │  ┌─────────────────────  ESP32-S3 (dual-core)  ─────────────────┐     │
  │  │  FreeRTOS tasks:                                             │     │
  │  │   hallTask(50Hz) ─┐                          servoTask ◄──┐  │     │
  │  │   imuTask(100Hz) ─┼─► sensorMailbox ─► bleTask ─► notify  │  │     │
  │  │   chargerTask(1Hz)┘   (BQ25887 BMS, I2C)        ▲  write ─┘  │     │
  │  └───────────────────────────────────────────────│────────────┘     │
  └──────────────────────────────── BLE ─────────────│──────────────────┘
                                                      │
        ┌─────────────────────────────────────────────┴───────────────┐
        ▼                                                              ▼
  OpenVR driver (C++, SimpleBLE)                     Web Bluetooth demo (this repo)
  FYDP_Driver/ — feeds SteamVR                       mobile-demo/ — 3D hand + grab demo
```

**The contract that ties it together:** a custom BLE GATT service with one *notify*
characteristic (glove → host, the sensor packet) and one *write* characteristic (host →
glove, the force-feedback command). Same UUIDs in the firmware, the OpenVR driver, and the
web app. A unit test in `mobile-demo/` literally reads the firmware headers and asserts they
match, so the three can't drift apart.

| | Value |
|---|---|
| Service | `7241bbc8-8ed8-4729-85ea-0ffc63248b4f` |
| Notify (sensors, glove→host) | `34797cc3-9e74-42e1-a669-be3cbdbae64d` |
| Write (force, host→glove) | `36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4` |
| Device name | `GloveRight` |

**Sensor packet (32 B, little-endian):** `[0]` battery · `[1–20]` bend (5 fingers × 2 joints
× uint16, 0–4095) · `[21–28]` splay (4 × uint16) · `[29]` button · `[30–31]` joystick. With
the IMU enabled the firmware appends `roll/pitch/yaw` float32 (→ 44 B).

**Force packet (10 B):** 2 bytes per finger — engage point + force magnitude.

---

## 2. Key design decisions (and the "why")

- **FreeRTOS over a superloop.** Independent tasks with their own rates (50/100/1 Hz),
  communicating through queues — no busy-waiting, no one slow step starving another. Radio
  pinned to core 0, sensors/control to core 1.
- **Queues for data, mutexes for shared *resources*.** A length-1 "mailbox" carries the
  latest sensor snapshot (drop stale, keep fresh). Mutexes guard the UART and the I2C bus —
  two tasks can't talk on one bus at once.
- **Defer work out of callbacks.** The BLE write callback runs in the BLE stack; it just
  enqueues a `ServoCommand` and returns. A dedicated `servoTask` does the slow PWM. Same
  principle as keeping ISRs short.
- **Complementary filter for orientation.** Gyro for smooth short-term motion, accelerometer
  to cancel long-term drift; one line, runs at 100 Hz, plus a startup gyro-bias calibration.
- **I2C-programmed BMS (BQ25887).** Charge voltage/current set in registers per the
  datasheet; the chip's 40 s I2C watchdog is kept alive by `chargerTask` as a fail-safe (if
  firmware hangs, charging reverts to safe defaults).
- **Logic/HAL separation for testability.** Pure math (packing, register encoding, fusion)
  lives in `protocol_logic.h`, unit-tested natively with g++; hardware paths are
  compile-gated and checked with on-device diagnostics.

---

## 3. Likely grill questions — crisp answers

**RTOS / concurrency**
- *Why FreeRTOS, not just `loop()`?* Independent timing per subsystem, preemptive
  scheduling fills idle time with useful work, and I can pin the radio and the control logic
  to separate cores.
- *Queue vs. mutex — when each?* Queue *transfers ownership of data* (a copy, no shared
  state → no torn reads). Mutex protects a *shared resource that can't be copied* (the UART,
  the I2C bus). Reach for the queue first.
- *Priority inversion?* A low-priority task holding a mutex blocks a high-priority waiter.
  FreeRTOS mutexes do priority inheritance — the holder is temporarily boosted. (Famous
  Mars Pathfinder bug.)
- *How do you size task stacks?* Empirically — `uxTaskGetStackHighWaterMark` is printed in
  `loop()`; watch the low-water mark and leave headroom.
- *`vTaskDelay` vs `vTaskDelayUntil`?* Delay = "wait ~N ms after the work" (jitter
  accumulates). DelayUntil = wake on absolute period boundaries → rock-steady sample rate.

**BLE**
- *Why two characteristics?* Notify = high-rate sensor stream pushed to the host; Write =
  occasional force commands from the host. Different directions, different cadences.
- *How do you avoid congesting the link?* Force writes are rate-limited to ~1 per 9 ms,
  latest-value-only, non-overlapping (wait for the previous write to resolve).
- *Endianness?* Little-endian 16-bit; the firmware packs LSB-first and a unit test asserts
  the host decodes the same way.

**Sensors / fusion / BMS**
- *Why fuse accel + gyro?* Each is bad alone — accel is absolute but noisy under motion and
  can't see yaw; gyro is smooth but drifts. The complementary filter high-passes the gyro
  and low-passes the accel.
- *How does the charger know the cells?* I set per-cell charge voltage (REG00, 5 mV/step)
  and current (REG01, 50 mA/step) over I2C; cell balancing on the BQ25887 is automatic.
- *What if the MCU crashes mid-charge?* The chip's I2C watchdog expires after 40 s and
  reverts to safe register defaults — `chargerTask` kicks it during normal operation.

**Testing / quality**
- *How do you test firmware without hardware?* Separate the pure logic into a HAL-free
  header and unit-test it natively (37 g++ checks, milliseconds). Hardware paths get a
  compile gate + on-device diagnostics. I'm explicit about what *can't* be unit-tested.
- *How do the firmware and host stay in sync?* A cross-project test reads the firmware
  headers and asserts the UUIDs/name match the host app's expectations.

---

## 4. Run / verify everything

```bash
# Firmware compiles for the board:
arduino-cli compile --fqbn esp32:esp32:esp32s3 --libraries <Arduino/libraries> glove_firmware_rtos

# Firmware logic tests (g++):        cd glove_firmware_rtos/test && ./run_tests.sh      # 37 checks
# Web demo protocol tests (Node):    cd mobile-demo && node --test                      # 22 checks
# Web demo (serve, then connect the glove):  cd mobile-demo && python3 -m http.server 8000
```

---

## 5. Honest limitations (good to volunteer before they ask)

- The web demo's grab→force path is real, but the firmware currently uses the force packet's
  *engage* byte directly as the servo angle and ignores the force byte — noted in the code.
- Splay: only the thumb carries a real value on the current hardware; the rest are centred.
- Battery % is a linear voltage estimate, not a coulomb-counting fuel gauge.
- Unit tests cover logic, not the live radio/WebGL/servo path — that's verified on a bench
  with the real glove.
