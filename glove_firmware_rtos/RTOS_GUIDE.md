# FreeRTOS Glove Firmware — Learning Guide

This folder is a FreeRTOS rewrite of the original Arduino `glove_firmware`. It does the
exact same job — read finger Hall sensors, stream them over BLE, drive 5 haptic servos
from BLE commands — but restructured around the RTOS concepts you'll want to talk about
in an embedded interview.

The original superloop is still in `../glove_firmware` so you can diff the two designs.

---

## 1. The big idea: superloop → tasks

**Old design (`../glove_firmware/glove_firmware.ino`):**

```cpp
void loop() {
  lastSend = millis();
  bluetoothTask();
  print_hall();
  while (lastSend + 30 > millis()) delay(1);   // busy-wait padding to ~30 ms
}
```

Everything runs **sequentially on one path**. If a BLE operation takes a while, the Hall
read waits. If the Hall read is slow, BLE stalls. The `while(...) delay(1)` is the CPU
literally spinning, doing nothing useful, to pad the loop to a fixed rate.

**New design:** three independent **tasks**, each an infinite loop doing one job. The
FreeRTOS **scheduler** decides which task runs on which of the ESP32-S3's two cores, and
when. A task only gives up the CPU when it *blocks* (waits for data or sleeps), so the
CPU is never spinning uselessly — when `hallTask` sleeps between samples, `bleTask` or
`servoTask` can run.

> **Interview soundbite:** *"I moved from a cooperative superloop to a preemptive
> multitasking model. Each subsystem became a task communicating through queues, which
> decoupled their timing and let me pin the BLE stack and the control logic to different
> cores."*

---

## 2. What is FreeRTOS (and where is it already)?

FreeRTOS is a tiny real-time **scheduler**. "Real-time" means tasks run with predictable
timing, not necessarily *fast*. You almost certainly already use it: **Arduino on ESP32
runs on top of FreeRTOS.** Your `setup()` and `loop()` actually run inside a FreeRTOS task
called `loopTask` (priority 1, pinned to core 1). So "adding FreeRTOS" here just means we
stop hiding it and start creating our own tasks alongside `loopTask`.

Key pieces the scheduler gives us, all used in this firmware:

| Primitive    | What it is                                  | Where we use it                              |
|--------------|---------------------------------------------|----------------------------------------------|
| **Task**     | An independent thread with its own stack    | `hallTask`, `bleTask`, `servoTask`, `imuTask`, `chargerTask` |
| **Queue**    | Thread-safe FIFO that *copies* items        | `servoQueue`, `sensorMailbox`, `imuMailbox`  |
| **Mutex**    | Exclusive lock on a shared resource         | `serialMutex` (UART), `i2cMutex` (I2C bus)   |
| **Event group** | A set of bits tasks set/clear/wait on    | `bleEvents` (the connection bit)             |
| **`vTaskDelayUntil`** | Sleep until an absolute time point | `hallTask` 50 Hz, `imuTask` 100 Hz cadence   |

---

## 3. Architecture / data flow

```
   ADC pins                                                     phone / PC
      |                                                              ^
      v                                                              |
 +-----------+   InputData (overwrite)   +-----------+   BLE notify  |
 | hallTask  | ========================> | bleTask   | =============='
 | core 1    |     sensorMailbox (len 1) | core 0    |
 | 50 Hz     |                           | ~33 Hz    |
 +-----------+                           +-----------+
                                              ^  ^
                                              |  | reads BLE_CONNECTED_BIT
                                       bleEvents  (event group)
                                              |
                              set/cleared by BLE server callbacks

   phone/PC writes haptic bytes
            |
            v
   +----------------+   ServoCommand    +-----------+   PWM
   | BLE RX callback| ================> | servoTask | =========> 5 servos
   | (BLE host ctx) |    servoQueue     | core 1    |
   +----------------+    (len 4)        +-----------+

   +-----------+   Orientation     +-----------+   xQueuePeek
   | imuTask   | ================> | imuMailbox| ============> bleTask
   | core 1    |   (mailbox,len 1) +-----------+   (overlaid onto the
   | 100 Hz    |   roll/pitch/yaw                   outgoing InputData packet)
   +-----------+
```

`bleTask` PEEKs `imuMailbox` (rather than receiving) so the 100 Hz IMU value stays
available, and copies roll/pitch/yaw into the `InputData` packet right before notifying.
The orientation is appended at the tail of the packet (bytes 32..43, three little-endian
floats), so the existing finger-data byte offsets are unchanged.

Two directions of data, each through its own queue. The two control loops (sensing,
actuation) never call each other directly — they only exchange messages. That's the
decoupling that makes the system robust.

---

## 4. The primitives, explained where they live

### 4.1 Tasks — `xTaskCreatePinnedToCore`

Created in `glove_firmware_rtos.ino`:

```cpp
xTaskCreatePinnedToCore(hallTask, "hall", STACK_HALL, nullptr, PRIO_HALL, &hHall, CORE_APP);
//                       fn        name    stackBYTES  arg      priority   handle  core
```

- **Stack size is in *bytes*** on ESP32 (plain FreeRTOS uses words — classic trap). Each
  task owns a private stack carved from the heap. `hallTask` needs a big one because
  `read_hall()` puts a `32×5×3` sample buffer on its stack.
- **Priority:** higher number wins. `servoTask` is highest (3) because haptics must feel
  instant; `hallTask`/`bleTask` are 2; the Arduino `loopTask` is 1; the idle task is 0.
- **Core pinning:** `CORE_PROTOCOL (0)` for BLE, `CORE_APP (1)` for sensors/servos. The
  BLE controller is timing-sensitive, so we keep our ADC bursts off its core.
- A task is an **infinite loop** (`for(;;)`). It must never return. To stop one, call
  `vTaskDelete(NULL)`.

### 4.2 Queues — passing data safely between tasks

A queue **copies** items in and out, so the sender and receiver never touch the same
memory — no manual locking needed for the payload.

**`servoQueue`** (`rtos_common.cpp`, `xQueueCreate(4, sizeof(ServoCommand))`):
The BLE RX callback runs *inside the BLE stack*. Doing slow work there (driving 5 servos)
would stall BLE for every other operation. So the callback just parses the bytes and
`xQueueSend`s a `ServoCommand`, then returns in microseconds. `servoTask` blocks on
`xQueueReceive(..., portMAX_DELAY)` and does the actual motor moves. This **"defer work
out of the callback/ISR"** pattern is one of the most important RTOS idioms.

**`sensorMailbox`** (`xQueueCreate(1, sizeof(InputData))`):
Length-1 queue written with `xQueueOverwrite` — a **mailbox**. `hallTask` produces ~50
packets/s; `bleTask` consumes the freshest one whenever it's ready. If BLE is briefly
behind, we *overwrite* the stale packet instead of letting old data pile up. For
real-time tracking you want the *latest* value, not a backlog.

> **Why a queue instead of a shared global?** A plain global `InputData package_data` read
> by one task while another writes it can be read **half-updated** (a "torn read") — the
> low byte of a new sample with the high byte of an old one. The queue copies the whole
> struct atomically under the hood, so the reader always sees a consistent snapshot.

### 4.3 Mutex — protecting a shared resource (`serialMutex`)

`Serial` is one physical UART shared by every task. If two tasks print at once, the bytes
interleave into garbage. `logf()` in `logger.cpp` takes the mutex, prints, then gives it
back:

```cpp
vsnprintf(buf, ...);                                 // format OUTSIDE the lock
if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50))) {// acquire exclusive access
    Serial.print(buf);
    xSemaphoreGive(serialMutex);                     // ALWAYS release
}
```

Two details worth saying out loud in an interview:
- We format the string *before* taking the lock, to **hold it for as short a time as
  possible**.
- A FreeRTOS mutex supports **priority inheritance**: if a low-priority task holds the
  lock and a high-priority task wants it, the holder is temporarily bumped up so it
  finishes and releases quickly. This avoids **priority inversion** (the bug that famously
  hit the Mars Pathfinder).

### 4.4 Event group — broadcasting state (`bleEvents`)

A single bit, `BLE_CONNECTED_BIT`, means "a client is connected". The BLE server callbacks
`xEventGroupSetBits` / `ClearBits` it; `bleTask` reads it to decide whether to transmit.
Using an event bit instead of a `bool deviceConnected` global means the state change is
published through a proper synchronization object that other tasks could **block on**
(`xEventGroupWaitBits`) if we wanted them to sleep until connected.

### 4.5 `vTaskDelay` vs `vTaskDelayUntil`

In `hallTask`:

```cpp
TickType_t lastWake = xTaskGetTickCount();
for (;;) {
    read_hall(); build_package(&pkg); xQueueOverwrite(...);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HALL_PERIOD_MS));  // absolute schedule
}
```

- `vTaskDelay(20ms)` sleeps 20 ms **after the work finishes** — so the true period is
  `work_time + 20ms`, and jitter accumulates.
- `vTaskDelayUntil(&lastWake, 20ms)` wakes at **fixed 20 ms boundaries** regardless of how
  long the work took — giving a rock-steady 50 Hz sample rate. Use it for anything that
  needs a precise rate (sensor sampling, control loops).

`pdMS_TO_TICKS(ms)` converts milliseconds to the scheduler's tick count (the default tick
is 1 ms on ESP32, but you should never hard-code that — always convert).

### 4.6 IMU sensor fusion — `imuTask` (the 4th task)

`imuTask` (`imu.cpp`) runs at **100 Hz** and fuses the IIM-42652's accelerometer and
gyroscope into a stable orientation estimate using a **complementary filter**:

```cpp
att.roll  = ALPHA*(att.roll  + gx*dt) + (1-ALPHA)*accel_roll;
att.pitch = ALPHA*(att.pitch + gy*dt) + (1-ALPHA)*accel_pitch;
att.yaw  += gz*dt;   // no magnetometer -> integrate gyro only (drifts slowly)
```

Why fuse at all? Each sensor is bad alone:

- **Accelerometer** measures gravity → an *absolute* roll/pitch with no drift, but it's
  swamped by noise whenever the hand actually moves, and it can't see yaw.
- **Gyroscope** measures angular *rate* → integrate it for a buttery-smooth angle that
  tracks fast motion, but the tiny zero offset integrates into unbounded **drift**.

The complementary filter is a one-line sensor fusion: `ALPHA` (=0.98) high-passes the gyro
(trusted short-term) and low-passes the accelerometer (trusted long-term), giving an angle
that's both smooth *and* drift-free. Two real-world touches in the code:

- **Gyro bias calibration** at boot: `initImu()` averages 300 stationary samples to learn
  the gyro's zero-rate offset and subtracts it, so a still hand reads ~0°/s instead of
  slowly spinning.
- **Fixed timestep:** because `vTaskDelayUntil` paces the loop at exactly 100 Hz, `dt` is
  a known constant — no need to measure elapsed time.

The result is published to `imuMailbox` (same length-1 mailbox pattern as the sensors), so
any consumer can grab the latest tilt. This is the "real-time IMU sensor fusion" running
concurrently with — and fully decoupled from — the BLE and servo work.

> *Why a separate task and not fold it into `hallTask`?* Different rates (100 Hz vs 50 Hz)
> and the fusion math is independent of the finger sampling. Separate tasks keep each loop
> at its own natural rate and make the design read like the block diagram.

> **Interview soundbite:** *"The orientation came from a complementary filter fusing accel
> and gyro at 100 Hz in its own task — gyro for the fast smooth response, accel to cancel
> the long-term drift, plus a startup bias calibration on the gyro."*

### 4.7 Adaptive packet: "no IMU" vs "yes IMU"

`initImu()` reports whether the IMU was actually found; setup() stores that in
`g_imuPresent`. Because the orientation floats live at the **tail** of `InputData`,
`bleTask` simply chooses the notify *length*:

```cpp
size_t len = offsetof(InputData, roll);   // 32 bytes — original packet, no orientation
if (g_imuPresent) { /* overlay roll/pitch/yaw */ len = sizeof(InputData); } // 44 bytes
pTxCharacteristic->setValue((uint8_t*)&pkg, len);
```

So with no IMU the device emits **exactly your original 32-byte packet** (old host parsers
keep working untouched); with an IMU it emits 44 bytes with tilt appended. Same
characteristic/UUID either way — the host just checks the received length.

### 4.8 Serial plotting the Hall sensors

The old `print_hall()` is back as `plot_hall()`, but RTOS-aware. Define `PLOT_HALL` in
`config.h` and it streams values every hall cycle (50 Hz) in **Arduino Serial Plotter**
format (`label:value` pairs). While `PLOT_HALL` is on, `logf()` is silenced so the plotter
only sees clean numbers (no `[BLE]`/`[IMU]`/`[diag]` text corrupting the graph).
`PLOT_HALL_FINGER` picks one finger (0–4) or `-1` for all 15 traces.

```
base:1820 knuckle:2960 splay:1040
base:1815 knuckle:2971 splay:1038
...
```

Open **Tools → Serial Plotter** at 115200 to see live curves as you bend the finger.

### 4.9 Hall min/max calibration (open/close)

`hall_calibrate(ms)` reproduces the original "open then close your hand" routine: for the
window it records the lowest and highest **raw** ADC value each sensor sees, and stores
them as the `min_hall_value`/`max_hall_value` used to scale every later reading. Enable it
by uncommenting `HALL_CALIBRATE_ON_BOOT` in `config.h`.

Two deliberate differences from the original:

- It runs in `setup()` **before the tasks start**, so it's single-threaded — no locking
  needed, and `hallTask` only ever sees finished calibration data.
- **Bug fix:** the original `read_hall_calibration()` compared the *processed*
  (sine-mapped) value against the *raw-range* min/max tables — mixed units. This version
  calibrates on the raw averaged ADC value, which is exactly what `read_hall()` later
  clamps/maps against, plus a guard so an unplugged sensor can't cause a divide-by-zero in
  `map()`.

Calibration now **persists to flash**: `hall_calibrate()` saves the min/max tables to NVS
(via the ESP32 `Preferences` library) when it finishes, and on boot `hall_load_calibration()`
restores them. So the workflow is: enable `HALL_CALIBRATE_ON_BOOT` once, calibrate, then
disable it — the saved ranges load automatically every boot after that.
`hall_clear_calibration()` wipes them back to the built-in defaults.

### 4.10 BQ25887 battery charger + the I2C-bus mutex

`bq25887.*` is an I2C driver for the TI **BQ25887** 2-cell charger, plus `chargerTask`
(1 Hz). On boot, `bq_init()`:

1. Confirms the part (REG25 `PN = 0101`) — returns false if absent so a board without it
   still boots everything else.
2. Programs charge voltage (REG00, 5 mV/step), fast-charge current (REG01, 50 mA/step),
   and pre-charge/termination currents (REG04) — all from `config.h` constants.
3. Sets the I2C watchdog (REG05) and enables the integrated ADC (REG15).

`chargerTask` then, every second: **services the watchdog**, reads the charge state +
pack/cell voltages + charge current from the 16-bit ADCs, estimates battery %, and writes
it into `g_batteryPercent` — which the hall packet builder drops into
`InputData.battery_lvl`, so the phone finally sees a real battery level. All register
values were taken from the datasheet (SLUSD89B); cell balancing is automatic, so firmware
leaves those registers at their defaults.

**The watchdog is itself an RTOS lesson.** The BQ25887 has a 40 s I2C watchdog: if nothing
talks to it for 40 s, it assumes the host died and reverts to safe default settings. We
*keep* it on and have `chargerTask` "kick" it (`WD_RST`) every ~20 s. That's a fail-safe —
if our firmware hangs, charging falls back to the chip's safe defaults rather than running
forever with custom settings. (Set `CHARGER_SERVICE_WATCHDOG` off in `config.h` to disable
the watchdog instead, so settings persist unattended — a deliberate trade-off.)

**Why a second mutex (`i2cMutex`)?** The IMU (`imuTask`, 100 Hz) and the charger
(`chargerTask`, 1 Hz) sit on the **same I2C bus**. The Arduino `Wire` driver isn't safe to
call from two tasks at once — overlapping transactions corrupt each other or wedge the bus.
So both tasks take `i2cMutex` around every bus access. It's the exact same pattern as
`serialMutex` (§4.3), just guarding a different shared peripheral — a clean illustration
that *any* singleton hardware resource shared between tasks needs a lock.

> ⚠️ **Before you charge a real pack:** set `CHARGER_VCELL_MV` / `CHARGER_ICHG_MA` in
> `config.h` to your cell's spec (charge current is usually ≤ 0.5C). The defaults
> (4.20 V/cell, 500 mA) are conservative but are **not** a substitute for checking your
> battery's datasheet. Also confirm `CHARGER_SDA`/`CHARGER_SCL` match your schematic.

> **Interview soundbite:** *"The charger and IMU share an I2C bus across two tasks, so I
> guarded it with a mutex — same reasoning as the UART. And I kept the charger's hardware
> watchdog alive from a periodic task as a fail-safe: if the firmware hangs, charging
> reverts to safe defaults."*

---

## 5. File map

| File              | Role                                                              |
|-------------------|-------------------------------------------------------------------|
| `config.h`        | All tuning knobs: cores, priorities, stack sizes, periods         |
| `shared.h`        | Inter-task message structs (`InputData`, `ServoCommand`)          |
| `rtos_common.*`   | Declares + creates the queues, mutex, event group                 |
| `logger.*`        | Mutex-guarded `logf()` printf-style logging                       |
| `hall.*`          | Sensor read/filter/pack + the periodic `hallTask` producer        |
| `servo.*`         | Servo driver + the event-driven `servoTask` consumer              |
| `bluetooth.*`     | BLE GATT server, callbacks (defer to queues), `bleTask`           |
| `imu.*`           | 100 Hz complementary-filter sensor fusion task (`ENABLE_IMU`)     |
| `bq25887.*`       | BQ25887 charger driver + 1 Hz battery monitor task (`ENABLE_CHARGER`) |
| `*.ino`           | `setup()` wires it all up; `loop()` is a diagnostics heartbeat    |

---

## 6. Build & flash (Arduino IDE)

1. **Board:** *ESP32S3 Dev Module* (install "esp32 by Espressif Systems" boards package).
2. **Libraries:** `ESP32Servo` and `RAK12033-IIM42652` (for the IMU). The BLE and FreeRTOS
   APIs ship with the ESP32 core.
3. Select the right hand in `config.h` (`#define RIGHT_HAND` or `LEFT_HAND`).
4. Open `glove_firmware_rtos.ino`, select your port, **Upload**.
5. Open Serial Monitor at **115200**. Every 5 s you'll see the diagnostics line:

   ```
   [diag] heap=...  stack free (words) hall=... ble=... servo=... imu=...
   ```

   You'll also see the fused IMU orientation a few times a second:

   ```
   [IMU] roll=  12.3  pitch=  -4.1  yaw=   0.8 [deg]
   ```

   Watch the `stack free` numbers — if any trends toward 0, bump that `STACK_*` in
   `config.h`. If they stay large, you can shrink them to save RAM.

> The IMU is **enabled by default** (`#define ENABLE_IMU` in `config.h`) and needs the
> `RAK12033-IIM42652` library. To build without the IMU hardware/library, comment out that
> one `#define` — `initImu()`/`imuTask()` become no-op stubs and nothing else changes. Hold
> the glove **still** for the first ~second after boot so the gyro bias calibration is
> accurate.

---

## 7. Interview talking points (cheat sheet)

- **Why tasks over a superloop?** Decoupled timing, no busy-waiting, the scheduler fills
  idle time with useful work, and I can pin radio vs. application code to separate cores.
- **How do tasks communicate?** Queues (copying, so no torn reads), not shared globals.
  Two directions, two queues; the sensor one is a length-1 *mailbox* so it always holds
  the freshest sample.
- **Why defer work out of the BLE callback?** It runs in the BLE host task; blocking it
  stalls the whole stack. The callback enqueues a command and returns; a dedicated task
  does the slow servo moves. Same principle as keeping ISRs short.
- **Mutex vs. queue?** Mutex protects a *shared resource* (the UART). Queue *transfers
  ownership of data*. I reach for a queue first because it avoids shared state entirely;
  I use a mutex only when a resource genuinely can't be copied.
- **Priority inversion?** A mutex with priority inheritance prevents a low-priority lock
  holder from indefinitely blocking a high-priority waiter.
- **Periodic timing?** `vTaskDelayUntil` for fixed-rate loops (no drift), `vTaskDelay` for
  "wait roughly this long".
- **How do I size stacks?** Empirically, via `uxTaskGetStackHighWaterMark` (printed in
  `loop()`), then leave headroom.
- **Preemptive vs. cooperative?** FreeRTOS here is preemptive with time-slicing between
  equal-priority tasks; a higher-priority task that becomes ready preempts a lower one
  immediately.

---

## 8. Ideas to take it further

- Replace the `sensorMailbox` poll in `bleTask` with a **direct-to-task notification**
  (`xTaskNotifyGive` / `ulTaskNotifyTake`) — the lightest, fastest signalling primitive.
- Add a **software timer** (`xTimerCreate`) for battery sampling instead of a whole task.
- Add the **task watchdog** (`esp_task_wdt`) so a hung task forces a recovery reset.
- Move the Hall ADC reads to **DMA + a callback** so the CPU isn't busy during sampling.
- Profile with the Arduino/ESP-IDF **FreeRTOS trace** tools to see core utilisation.
