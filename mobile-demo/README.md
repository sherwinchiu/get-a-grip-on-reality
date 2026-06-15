# Haptic Glove — Web Bluetooth Demo

A single self-contained `index.html` that connects to the force-feedback VR glove
(ESP32-S3) over **Web Bluetooth**, renders a live 3D articulated hand from the sensor
stream, and writes force-feedback commands back to the glove's servos. Built for a live
interview demo — **Simulation Mode runs with no hardware**, so there's always a fallback.

No build step. Three.js loads from a CDN. Everything is in `index.html`.

---

## Run it

Web Bluetooth requires a **secure context** — you must serve the file over `https` or
`localhost`. Opening the file directly (`file://…`) will **not** work for BLE (Simulation
Mode still works from `file://`, but just serve it — it's one command).

**On your computer (localhost):**
```bash
cd mobile-demo
python3 -m http.server 8000
# then open  http://localhost:8000  in Chrome
```

**From your phone (to connect the real glove on Android Chrome):**
The phone needs a secure origin too. Easiest options:
- **USB + adb reverse** (recommended, counts as localhost on the phone):
  ```bash
  adb reverse tcp:8000 tcp:8000
  # on the phone's Chrome, open  http://localhost:8000
  ```
- **Or** put the file on any `https` host (GitHub Pages, ngrok `https` tunnel, etc.) and
  open that URL on the phone. A plain `http://<laptop-LAN-IP>:8000` will **not** allow
  Web Bluetooth — it's not a secure context.

---

## Connect the glove (important)

1. Power on the glove (it advertises as **`FYDPGloveRight`**).
2. **Do NOT pair the glove in the phone's Android Bluetooth settings.** Web Bluetooth
   claims the device itself through Chrome's in-page chooser, and a pre-existing OS-level
   pairing can interfere with that. If you previously paired it in system settings,
   **remove/"Forget" it there** first.
3. In the app, tap **Connect Glove**. A Chrome device chooser pops up *inside the page* —
   pick **FYDPGloveRight** from that list (not from system settings).
4. On connect, the status pill turns green and Simulation Mode switches off automatically.
   If the link drops, the app falls back to Simulation and you can tap Connect to retry.

**Browser support:** Chrome / Edge only (incl. Android Chrome). Web Bluetooth is **not**
available in Safari / iOS.

---

## Simulation Mode

Toggle **Sim: On/Off** in the top bar. When on (the default at startup), the hand, grab
detection, and force pipeline are driven by *synthetic* data — it even synthesises a real
32-byte packet and runs it through the same parser, so the full visual works with no glove
attached. Build/verify the demo here first; connecting a real glove just swaps the data
source.

---

## Using the demo

- **Spawn** a Ball, Cube, or Mouse — each has its own force profile (editable in the
  `FORCE_PROFILES` config near the top of the script).
- **Grab**: curl the thumb + at least one finger past the threshold while an object is
  spawned → the app writes that object's force profile to the glove so the servos resist.
  Open your hand to release.
- The **Sensor Telemetry** and **Force Output** panels show the live data pipeline: raw
  per-joint bend, splay, battery, button, joystick, orientation (if the firmware IMU is
  on), and the exact 10-byte force packet last sent.

Force writes are rate-limited to ~1 every 9 ms (latest value only) to match the firmware's
BLE expectations and avoid congestion.

---

## Files & tests

| File | Role |
|------|------|
| `index.html` | The app: UI, Three.js hand, BLE connection, render loop |
| `glove-protocol.js` | **Dependency-free** protocol layer — packet parse/encode, force packing, grab logic, sim source. Imported by the app *and* the tests, so there's one source of truth for the firmware byte contract. |
| `tests/glove-protocol.test.mjs` | 22 unit tests (Node built-in runner, no deps) |

Run the tests (Node 18+):
```bash
cd mobile-demo
node --test
```
They cover parse/encode round-trips, **little-endian** decoding, 0–4095 clamping, the
optional 44-byte IMU tail, splay mapping, the 10-byte force packet, grab detection, and the
simulation source — plus a **live cross-check that the demo's UUIDs and device name match
the firmware's `bluetooth.hpp`/`.cpp`** (so the two can't silently drift apart). What unit
tests *can't* prove is the live radio/WebGL/servo path — that's why Simulation Mode exists
as the verifiable, always-works fallback.

## BLE contract (read from `glove_firmware_rtos/`)

- **Service** `7241bbc8-8ed8-4729-85ea-0ffc63248b4f`
- **Notify (glove → app)** `34797cc3-9e74-42e1-a669-be3cbdbae64d`
- **Write (app → glove)** `36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4`
- **Device name** `FYDPGloveRight`

**Input packet — 32 bytes, little-endian 16-bit values:**

| Bytes | Field |
|-------|-------|
| 0     | battery (0–255; firmware sends 0–100) |
| 1–20  | finger bend — 5 fingers × 2 joints (MCP, PIP), thumb→pinky, `uint16` 0–4095 |
| 21–28 | splay — 4 × `uint16` 0–4095 |
| 29    | button (0/1) |
| 30–31 | joystick x, y (0–255) |
| 32–43 | *optional* roll/pitch/yaw `float32` — only present when the firmware's IMU is enabled (packet becomes 44 bytes) |

> Two corrections vs. the original brief: the base packet is **32 bytes (not 33)**, and the
> **joystick is at bytes 30–31** (there's no spare byte 30). The parser reads the optional
> IMU tail when present and ignores it otherwise.

**Force packet — 10 bytes, app → glove:** 2 bytes per finger (thumb→pinky): `[engage%, force%]`,
each 0–255. *Note:* the current firmware uses the **engage byte directly as the servo angle**
(`0–255 → 0–180°`) and ignores the force byte — adjust `buildForcePacket()` if you change
the firmware's force model.

---

## Extending to two gloves

The code is structured for a clean second-glove drop-in but is **wired for one** glove
today. Connection handling, the 3D hand, and the force writer all live in a `Glove` class
that's instantiated once into a `gloves` array. To add the left hand later: construct a
second `new Glove({ role:'left', side:-1 })`, push it into `gloves`, and connect it with a
`namePrefix`/role filter — the hand model already mirrors via its `side` scale.
