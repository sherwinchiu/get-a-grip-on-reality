# Architecture & How It Works

The complete technical explanation of the force-feedback VR glove — both the **architecture**
(how the pieces fit) and the **low-level** detail (bytes, registers, math). Pair this with
[`INTERVIEW_PREP.md`](INTERVIEW_PREP.md) (study questions + talking points) and the deep
per-component docs it links.

---

## 0. What it is, in one breath

A wearable glove (ESP32-S3) reads **14 Hall-effect finger sensors**, fuses an **IIM-42652
IMU**, manages a **2-cell battery via a BQ25887**, and drives **5 servos** for per-finger
force feedback. It streams sensor data over **BLE** and accepts force commands back. Three
*consumers* read that same BLE stream: a **Web Bluetooth demo** (this repo's `mobile-demo/`),
an **OpenVR/SteamVR driver** (`../FYDP_Driver`), and a (parked) native option. **One BLE
byte-contract ties all of them together.**

```
        ┌──────────────── GLOVE (ESP32-S3, FreeRTOS) ────────────────┐
 Hall ──┤ hallTask 50Hz ─┐                                           │
 IMU  ──┤ imuTask 100Hz ─┼─► mailboxes ─► bleTask ──(BLE notify)──┐  │
 BQ25887┤ chargerTask 1Hz┘                          ▲             │  │
 servos◄┤ servoTask  ◄── servoQueue ◄──(BLE write)──┘             │  │
        └─────────────────────────────────────────────────────────│──┘
                                                                   │ BLE GATT
            ┌──────────────────────────────────────────────────────┘
            ▼                         ▼                          ▼
    Web demo (mobile-demo)     OpenVR driver (FYDP_Driver)   [native — parked]
```

---

## 1. The BLE contract — the spine of the whole system

Everything hinges on one custom GATT service. The firmware, the web app, and the driver all
hard-code these, and a unit test cross-checks them so they can't drift.

| | UUID / value |
|---|---|
| Service | `7241bbc8-8ed8-4729-85ea-0ffc63248b4f` |
| **Notify** (glove → host: sensors) | `34797cc3-9e74-42e1-a669-be3cbdbae64d` |
| **Write** (host → glove: force) | `36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4` |
| Advertised name | `GloveRight` / `GloveLeft` |

**Sensor packet (notify), 32 bytes — or 44 with the IMU.** All multi-byte values are
**little-endian** (LSB first), because that's how the firmware packs them and how an
ARM/x86 host naturally reads them.

| Byte(s) | Field | Encoding |
|---|---|---|
| `0` | battery | 0–100 (%) |
| `1–20` | finger bend | 5 fingers × 2 joints (MCP, PIP), thumb→pinky, `uint16` 0–4095 |
| `21–28` | splay | 4 × `uint16` 0–4095 |
| `29` | button | 0/1 |
| `30–31` | joystick X, Y | `uint8` each, 0–255 |
| `32–43` | *(optional)* roll, pitch, yaw | `float32` LE — only present when the firmware IMU is enabled |

**Force packet (write), 10 bytes:** 2 per finger (thumb→pinky) = `[engage, force]`, each 0–255.
*Low-level note:* the current firmware uses the **engage byte directly as the servo angle**
(`0–255 → 0–180°`) and ignores the force byte.

---

## 2. Firmware (`glove_firmware_rtos/`) — ESP32-S3, Arduino core + FreeRTOS

> Deep dive: [`RTOS_GUIDE.md`](glove_firmware_rtos/RTOS_GUIDE.md). Testing:
> [`TESTING.md`](glove_firmware_rtos/TESTING.md). Bring-up: [`SELFTEST.md`](glove_firmware_rtos/SELFTEST.md).

### 2.1 Boot sequence (`setup()`)
1. `logInit()` — Serial up first so early logs show.
2. `rtos_init()` — create all queues/mutexes/event group **before** anything uses them.
3. Init peripherals: `init_servos()`, `init_hall()`.
4. *(optional)* `hall_calibrate()` or `hall_load_calibration()` (NVS).
5. `initBluetooth()` → `initImu()` (returns false if absent) → `bq_init()` (returns false if absent).
6. *(optional)* `run_selftest()` — power-on self-test through the real drivers.
7. Create the tasks; `loop()` becomes a diagnostics heartbeat (heap + per-task stack high-water marks).

### 2.2 Tasks (the "what runs where")
| Task | Core | Priority | Rate | Job |
|---|---|---|---|---|
| `hallTask` | 1 (APP) | 2 | 50 Hz | sample sensors → build packet → publish |
| `imuTask` | 1 | 1 | 100 Hz | read accel+gyro → complementary filter → publish orientation |
| `chargerTask` | 1 | 1 | 1 Hz | service charger watchdog, read voltages, update battery % |
| `bleTask` | 0 (PROTOCOL) | 2 | ~33 Hz | pull latest packet (+orientation) → BLE notify |
| `servoTask` | 1 | 3 | event-driven | block on queue → drive PWM |
| `loopTask` (Arduino) | 1 | 1 | 0.2 Hz | diagnostics |

### 2.3 Inter-task plumbing (the concurrency primitives)
- **`sensorMailbox`** — length-1 queue written with `xQueueOverwrite`: always holds the
  *latest* `InputData` (stale frames dropped, which is what you want for real-time).
- **`servoQueue`** — length-4 queue of `ServoCommand`. The BLE write callback runs in the
  BLE stack, so it just **enqueues** and returns; `servoTask` does the slow PWM. (Defer work
  out of the callback — same idea as keeping an ISR short.)
- **`imuMailbox`** — length-1 `Orientation`, consumed by `bleTask` (peeked, not drained).
- **`bleEvents`** — event group; `BLE_CONNECTED_BIT` set/cleared by the BLE server callbacks.
- **`serialMutex`** — serialises `logf()` so task prints don't interleave.
- **`i2cMutex`** — the IMU (100 Hz) and charger (1 Hz) share one I²C bus; this stops their
  transactions from colliding.

### 2.4 Low-level per module
- **Hall (`hall.cpp`)** — 5×3 ADC channels. Each read: burst **32 samples** and average →
  `hall_normalize(raw, min, max, invert)` = clamp into the calibrated range → integer
  `map(...,0,4096)` → optional invert → sinusoidal response `(1 − cos(x·π/2 / 4095)) · 4095`.
  Calibration min/max persist to flash (NVS `Preferences`). Packing: `pack_bend` (2 LE values
  per finger) + `pack_splay` (skips the index finger — no splay magnet — thumb real, rest 2048).
- **IMU (`imu.cpp`)** — IIM-42652 over I²C @ `0x68`. Accel `±16 g` (2048 LSB/g), gyro
  `±2000 °/s` (16.4 LSB/°/s). Startup **gyro-bias calibration** (avg 300 still samples).
  Per 100 Hz tick: accel tilt via `atan2`; **complementary filter**
  `angle = 0.98·(angle + gyro·dt) + 0.02·accel_angle`; yaw is gyro-integrated (drifts, no mag).
- **Charger (`bq25887.cpp`)** — BQ25887 over I²C @ `0x6A`. Register encodings (datasheet
  SLUSD89B): charge V `REG00 = (mV−3400)/5`, charge I `REG01 = mA/50`, pre/term
  `(mA−50)/50`. Keeps the **40 s I²C watchdog** alive (kicks `WD_RST` ~every 20 s) so settings
  persist; if firmware hangs, the chip safely reverts to defaults. Reads pack/cell voltages +
  charge current from the integrated 16-bit ADC; estimates battery % (linear).
- **BLE (`bluetooth.cpp`)** — Bluedroid. Notify char streams `InputData`; the write char's
  `onWrite` parses 2 bytes/servo → `ServoCommand` → `servoQueue`. `bleTask` chooses the
  notify length: **32 B** normally, **44 B** when an IMU is present (orientation appended).
- **Servo (`servo.cpp`)** — `servoTask` blocks on `servoQueue` (zero CPU when idle), then PWM.

### 2.5 HAL / logic separation (why it's testable)
All the pure, bug-prone math — packing, hall normalization, BQ25887 register encoding, IMU
fusion, self-test thresholds — lives in **`protocol_logic.h`** with no hardware dependency.
The firmware `.cpp` files *call into it* (so it's the real code path), and host tests compile
the same header with **g++** (`test/`, 52 checks). Hardware paths (RTOS, radio, ADC, PWM) are
covered by the compile gate + on-device diagnostics + the self-test.

---

## 3. Web demo (`mobile-demo/`) — Web Bluetooth + Three.js

> Run/deploy + demo-day guide: [`README.md`](mobile-demo/README.md),
> [`DEMO_RUNSHEET.md`](mobile-demo/DEMO_RUNSHEET.md).

### 3.1 File split
- **`glove-protocol.js`** — dependency-free protocol layer (parse/encode, force packing, grab
  logic, constants). **Single source of truth**, imported by both the app and the tests.
- **`index.html`** — the app: UI/CSS, Three.js scene, the `Glove` class, the render loop.
- **`tests/glove-protocol.test.mjs`** — 22 Node tests, incl. a cross-check that reads the
  firmware headers.

### 3.2 Connection lifecycle (the `Glove` class)
`connect()` (must be a user gesture) → `navigator.bluetooth.requestDevice({namePrefix:'Glove'})`
→ `gatt.connect()` → `getPrimaryService` → get notify + write characteristics →
`startNotifications()`. Each notification fires `_onPacket(e)` → `parsePacket(e.target.value)`
(guarded: a short/garbled packet is counted and dropped, never thrown). Robustness: idempotent
listeners (no duplicates across reconnects), teardown on partial-connect failure, graceful
disconnect → idle state, chooser-cancel handled quietly.

### 3.3 Render loop (60 fps)
1. **State source:** the live glove packet when connected, else a calm idle pose.
2. `hand.apply(state, sway)` — drive the 3D hand.
3. `updateGrab(state)` — grab detection → force write.
4. `updateDebug(state, dt)` — push oscilloscope history + update the active data mode (~15 Hz).
5. Render Three.js.

A separate `setInterval(9 ms)` **force-write pump** sends the latest force packet, **non-overlapping**
(waits for the previous write to resolve) — matching the firmware's BLE pacing.

### 3.4 The 3D hand (Three.js)
- **Rig:** a `group` → per-finger `root` (splay pivot) → `mcp` (knuckle pivot) → proximal
  phalanx → `pip` (middle pivot) → distal phalanx + tip. Plus a palm slab, a green wrist
  **cuff**, a tapered **forearm**, and a `graspAnchor` (the object is parented here so it
  always sits in the palm).
- **Joint mapping math:** `mcp.rotation.x = curl_mcp · 1.30 rad`, `pip.rotation.x = curl_pip ·
  1.55 rad`, `root.rotation.z = splay · 0.26 rad`. Whole-hand tilt from the IMU:
  `target = base + clamp(orientation · 0.8°/° , ±0.9 rad)`, smoothed (lerp 0.18). When
  disconnected, a gentle sine **idle sway** keeps it alive.

### 3.5 The 6 data modes (tabbed "Live data")
| Mode | Shows |
|---|---|
| **Telemetry** | battery, per-finger MCP/PIP bend bars (driven by the raw reading → truly 0 at rest), button, joystick (+ XY pad), orientation |
| **Graph** | live **oscilloscope** — 5 finger-curl traces over a ~12 s rolling window, % grid |
| **Force** | grab state, writes/s, per-finger force bars, the raw 10-byte force packet (hex) |
| **Servos** | commanded servo angle per finger (`engage byte → 0–180°`, exactly as the firmware interprets it) |
| **Raw** | hex dump of the actual incoming packet + decoded fields (and "with/without IMU") |
| **BLE** | device name, state, stream Hz, packet count, throughput, dropped packets, the UUIDs |

### 3.6 Force feedback — per-finger collision
Each spawned object carries a **hitbox** (sphere for the ball/mouse, box for the cube). Every
frame, each fingertip's world position is transformed into the object's local space and tested
against that hitbox (`objectContact`), so a finger registers contact **at its own point, only
when it actually touches** — the thumb and each finger contact independently as the hand
closes. Contacting tips are tinted; a **per-finger** force packet (`buildForcePacketPerFinger`)
commands each servo proportional to its contact depth × the object's `force` profile. A "grab"
= thumb + ≥1 other finger in contact. This is more faithful than the OpenVR driver's
pose-only `isGripping` heuristic. (`buildForcePacket`/`graspPoseMet` remain in the protocol
module as the simpler pose-based path + for the tests.)

### 3.7 Why Web Bluetooth
No app install — just a URL in Chrome. Requires a **secure context** (https or localhost) and
is **Chrome/Edge only** (incl. Android Chrome; not Safari/iOS). On the Meta Quest browser
Web Bluetooth is blocked, which is why the in-headset path would need a native app.

---

## 4. How it all stays in sync (testing)
- **Firmware logic:** `glove_firmware_rtos/test/` — g++ host tests of `protocol_logic.h` (52 checks).
- **Web logic:** `mobile-demo/tests/` — Node tests of `glove-protocol.js` (22 checks).
- **Cross-project contract test:** a web test literally **reads the firmware's `bluetooth.hpp`/`.cpp`**
  and asserts the UUIDs + device name match the app — so firmware and app can never silently diverge.
- **Run everything:** `./run_all_tests.sh` (firmware g++ + web Node + optional firmware compile gate).

---

## 5. Repo map
| Path | What |
|---|---|
| `glove_firmware_rtos/` | the firmware (FreeRTOS) + its tests + RTOS_GUIDE / TESTING / SELFTEST |
| `glove_firmware_rtos/protocol_logic.h` | pure, shared, unit-tested firmware logic |
| `mobile-demo/` | the Web Bluetooth demo (`index.html` + `glove-protocol.js` + tests) |
| `mobile-demo/README.md`, `DEMO_RUNSHEET.md` | run/deploy + demo-day guide |
| `../FYDP_Driver/` | the OpenVR/SteamVR C++ driver (separate repo) |
| `INTERVIEW_PREP.md` | study questions + crisp answers + talking points |
| `ARCHITECTURE.md` | this doc — how everything works, architecturally + low-level |
| `run_all_tests.sh` | one command to run every test |
