# Power-On Self-Test (POST) — "first firmware go"

A one-shot hardware bring-up check that runs **separately from the main loop**, the first
time you flash a built glove. After a BLE client connects, it exercises every subsystem
*through its real driver* and reports PASS/FAIL over **both Serial and BLE**, then starts
normal operation. It's the fast way to confirm the whole stack — ADC + Hall, I2C + IMU,
I2C + charger, PWM + servos, and the BLE link itself — is wired and working before you trust
the live data.

## How it works

It runs inside `setup()` **while still single-threaded** — after `initBluetooth()` /
`initImu()` / `bq_init()`, but *before* the FreeRTOS tasks are created. That matters:

- No task contention on the shared I2C/ADC buses, so results are deterministic.
- BLE still connects and notifies because that's handled by the BLE stack's own task +
  callbacks (the connection event-group bit and `characteristic->notify()` don't need our
  `bleTask` to be running yet).

Files: `selftest.{hpp,cpp}` (the sequence) + pass/fail **thresholds** in
`protocol_logic.h` (so they're pure and unit-tested — see `test/`).

## What it checks

| # | Subsystem | What it does | Pass criteria |
|---|-----------|--------------|---------------|
| 1 | **BLE** | Waits up to `SELFTEST_BLE_WAIT_MS` for a client | client connects (no client → SKIP, not fail — e.g. USB-only bench) |
| 2 | **Hall** | Reads raw ADC on all 15 channels, prints them | no channel railed at 0/4095 (`hall_channel_healthy`); index-splay (unwired) excluded |
| 3 | **IMU** | Reads accel + gyro via the driver | gravity magnitude ≈ 1 g while still (`imu_accel_ok`); SKIP if IMU absent |
| 4 | **Charger** | Reads BQ25887 status + pack/cell voltages | no fault & pack voltage plausible for 2S (`pack_voltage_plausible`); SKIP if absent |
| 5 | **Servos** | Sweeps each finger 0°→60°→0°, one at a time | commanded sweep (no position feedback) — **confirm visually**; marked `OK*` |

It finishes with a summary: `RESULT: N passed, M failed, K skipped` and a clear
`>>> ALL CHECKS PASSED <<<` / `>>> SEE FAILURES ABOVE <<<` banner.

## How to run it

1. `RUN_SELFTEST_ON_BOOT` is **on by default** in `config.h` (it's a bring-up tool). Flash
   the firmware.
2. Open **Serial Monitor @ 115200** to watch the report (always available over USB).
3. To also see it over BLE, connect a client during the wait window:
   - a BLE terminal app like **nRF Connect** shows the text lines directly, or
   - the web demo / OpenVR driver (they harmlessly ignore the POST text frames — their
     parsers drop anything that isn't a 32-byte packet — then receive real data once the
     tasks start).
4. Watch the glove during the servo sweep — each finger should flex and relax in turn.

Example Serial output:
```
[POST] ===== POWER-ON SELF-TEST =====
[POST] BLE   : client connected             OK
[POST] HALL  : thumb  base=1523 knuckle=1820 splay=1190
[POST] HALL  : all channels in range        OK
[POST] IMU   : |g|=1.01  gyro=(0,-1,0)dps
[POST] IMU   : gravity vector sane          OK
[POST] BQ    : pack=7912mV (top=3955 bot=3957) batt=79%
[POST] BQ    : status nominal              OK
[POST] SERVO : sweeping each finger -- WATCH THE GLOVE
[POST] RESULT: 5 passed, 0 failed, 0 skipped
[POST] >>> ALL CHECKS PASSED <<<
```

## Turning it off

Comment out `#define RUN_SELFTEST_ON_BOOT` in `config.h` for everyday/production use, so
boots are fast and the servos don't sweep each time. Everything else is unchanged.

## Interview soundbite

> *"I added a power-on self-test that runs before the RTOS tasks start — single-threaded, so
> no bus contention — and walks every subsystem through its real driver: ADC/Hall, I2C/IMU
> with a gravity-magnitude sanity check, I2C/charger, and a commanded servo sweep, reporting
> over Serial and BLE. The pass/fail thresholds are pure functions I unit-test on the host,
> and absent optional hardware reports as SKIP rather than FAIL."*
