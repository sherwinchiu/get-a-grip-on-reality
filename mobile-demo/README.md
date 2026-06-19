# Force-Feedback Glove — Web Bluetooth Demo

A single self-contained `index.html` that connects to the glove (ESP32-S3) over **Web
Bluetooth**, renders a live 3D articulated hand from the sensor stream, lets you grab objects,
and writes **force-feedback** commands back to the servos. No build step; Three.js loads from
a CDN. Architecture + low-level detail: see [`../ARCHITECTURE.md`](../ARCHITECTURE.md).

> This demo is about the **real glove** — there's no simulation mode. When no glove is
> connected it shows a calm idle hand and a "connect your glove" prompt; the moment you
> connect, your fingers (and, with the IMU, your wrist) map live.

## Requirements

- **Chrome or Edge** (incl. **Android Chrome**). Web Bluetooth is **not** available on
  Safari/iOS or the Meta Quest browser.
- A **secure context** — the page must be served over **https** or **localhost**. Opening the
  file directly (`file://…`) will not allow Bluetooth.

## Run / deploy

**On your phone (the real demo) — easiest, no PC needed at showtime:**
1. Drag **`../get-a-grip-demo.zip`** onto **https://app.netlify.com/drop** → you get a
   permanent `https://…netlify.app` link (no account, no git push).
2. Open that link in **Chrome on your phone**.

**On your computer (local dev):**
```bash
cd mobile-demo
python3 -m http.server 8000     # then open http://localhost:8000 in Chrome
```

## Connect the glove

1. Power on the glove (it advertises as **`GloveRight`** / `GloveLeft`).
2. **Do NOT pair it in the phone's system Bluetooth settings** — Web Bluetooth claims the
   device through Chrome's in-page chooser, and an OS-level pairing interferes. "Forget" it
   there if present.
3. Tap **Connect Glove** → pick **GloveRight** from the chooser that pops up *in the page*.
4. Move your fingers → the hand mirrors them. If the firmware IMU is on, tilt your hand and
   the whole hand tilts too. If the link drops, you'll see the "connect your glove" prompt
   again — just tap Connect to reconnect.

## Using the demo

- **Spawn** a Ball, Cube, or Mouse (each has its own force profile, editable in
  `FORCE_PROFILES` in `glove-protocol.js`).
- **Grab**: as you close your hand, each fingertip is **collision-tested against the object's
  hitbox** — a finger resists only when it actually touches, at its own contact point
  (touching tips tint red; thumb + a finger = GRAB). Each servo gets its own force. Open to release.
- **Live data** tabs (the sidebar "modes"): **Telemetry** (bend/battery/button/joystick/orientation),
  **Graph** (live oscilloscope of finger curl), **Force** (force bars + the raw 10-byte packet),
  **Servos** (commanded angles), **Raw** (hex dump + decoded fields), **BLE** (link stats + UUIDs).
- **⟲ reset view** (bottom-right of the 3D stage) snaps the camera back if it gets dragged askew.

Force writes are rate-limited to ~1 every 9 ms (latest value only, non-overlapping) to match
the firmware's BLE pacing.

## Files & tests

| File | Role |
|------|------|
| `index.html` | the app: UI, Three.js hand, BLE connection, render loop, the 6 data modes |
| `glove-protocol.js` | dependency-free protocol layer (parse/encode, force packing, grab logic) — imported by the app **and** the tests, so there's one source of truth for the byte contract |
| `tests/glove-protocol.test.mjs` | 22 Node tests |

```bash
cd mobile-demo && node --test
```
The tests cover parse/encode round-trips, **little-endian** decoding, clamping, the optional
44-byte IMU tail, splay mapping, the force packet, grab detection — plus a **cross-check that
the demo's UUIDs and device name match the firmware's `bluetooth.hpp`/`.cpp`**, so the two
can't silently drift apart. (They can't prove the live radio/WebGL/servo path — that needs the
glove on a bench.)

## BLE contract (mirrors `glove_firmware_rtos/`)

- **Service** `7241bbc8-8ed8-4729-85ea-0ffc63248b4f`
- **Notify (glove → app)** `34797cc3-9e74-42e1-a669-be3cbdbae64d`
- **Write (app → glove)** `36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4`
- **Device name** `GloveRight` / `GloveLeft`

**Input packet — 32 bytes (44 with IMU), little-endian:** `[0]` battery · `[1–20]` bend
(5×2 `uint16` 0–4095) · `[21–28]` splay (4×`uint16`) · `[29]` button · `[30–31]` joystick ·
`[32–43]` *optional* roll/pitch/yaw `float32`.

**Force packet — 10 bytes:** 2 per finger `[engage, force]`, 0–255. *Note:* the firmware uses
the engage byte directly as the servo angle (`0–255 → 0–180°`).

## Extending to two gloves

Wired for one glove today, but built for a clean second: connection, hand, and force logic all
live in a `Glove` class instantiated once into a `gloves` array. To add the left hand: construct
`new Glove({ role:'left', side:-1 })` (the hand model mirrors via its `side` scale), push it,
and connect it by name. See `../ARCHITECTURE.md` §3.
