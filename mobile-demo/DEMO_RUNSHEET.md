# Demo-day run sheet — haptic glove

Everything you need to walk in, plug in, and nail the live demo. Keep this open on your phone.

## 1. Before you leave (prep checklist)

- [ ] **Glove charged.** (If unsure the charger is wired for I²C, that's fine — it charges standalone; see `glove_firmware_rtos/`.)
- [ ] **Firmware flashed** to the ESP32-S3 and confirmed booting (Serial Monitor @ 115200 shows `[BOOT] …`).
  - Optional but reassuring: leave `RUN_SELFTEST_ON_BOOT` on for a bench run, watch the POST report (Hall/IMU/charger/servos), then you *know* the hardware's alive. Turn it off for the actual demo so boot is instant.
  - If the charger isn't I²C-connected, comment out `#define ENABLE_CHARGER` so boot doesn't spend ~2.5 s retrying.
- [ ] **Demo deployed + tested on YOUR phone.** Drag `get-a-grip-demo.zip` onto https://app.netlify.com/drop → get the `https://…netlify.app` link → open it on your phone in **Chrome** and confirm it loads (you'll see the hand + "connect your glove" card).
- [ ] **Save the Netlify link** somewhere one tap away (bookmark / note).
- [ ] **Phone Bluetooth ON**, and the glove **NOT paired in Android system Bluetooth settings** (Web Bluetooth claims it via the in-page chooser; a system pairing interferes — "Forget" it if it's there).
- [ ] Use **Chrome on Android** (not Samsung Internet, not iOS/Safari — those can't do Web Bluetooth).

## 2. The 60-second demo flow

1. Open the Netlify link → "This is a live web app, no install — it talks to my glove over Bluetooth."
2. Power on the glove. Tap **Connect Glove** → pick **GloveRight** from the chooser.
3. **Move your fingers** → the 3D hand mirrors them in real time. (If the IMU's on, tilt your hand — the whole hand tilts too.)
4. Flip the sidebar tabs to show the data pipeline:
   - **Graph** → live oscilloscope of each finger's curl.
   - **Raw** → the actual 32/44-byte BLE packet (hex) + decoded fields. "This is the literal characteristic data."
   - **Telemetry / BLE** → battery, joystick, stream rate, UUIDs.
5. **Spawn a Ball**, close your hand to **grab** it → the GRAB badge lights and the glove's **servos push back** (force feedback). Open to release.
6. Wrap: "Firmware's FreeRTOS on the ESP32-S3; the same unit-tested packet parser runs in the web app and would run in the OpenVR/Unity client — one contract, three consumers."

## 3. If something goes wrong

| Symptom | Fix |
|---|---|
| No **Connect** button / "not available" banner | You're not on the Netlify **https** link (or not Chrome). Don't open the file directly. |
| Chooser shows nothing / can't find glove | Glove powered + advertising? Remove any **system Bluetooth pairing**. Re-tap Connect. |
| Connects, but hand doesn't move | Check the glove booted OK (Serial). Sensors/firmware issue, not the app. The app shows "Dropped" packets in the **BLE** tab if data is malformed. |
| Laggy / drops | BLE congestion — move closer, fewer 2.4 GHz devices around. The app already rate-limits force writes. |
| Camera got dragged askew | Tap **⟲ reset view** (bottom-right of the 3D stage). |
| Glove disconnects | App auto-shows "connect your glove" again — just tap Connect to reconnect. |

## 4. Fallback (if BLE just won't cooperate live)

You can still present strongly without a live link:
- Walk the **6 data modes** and explain the BLE contract from the **Raw**/**BLE** tabs.
- Show the **firmware self-test** (`SELFTEST.md`) + Serial output as proof the hardware works.
- Talk the architecture from `INTERVIEW_PREP.md` (FreeRTOS tasks, queues/mutexes, sensor fusion, the BMS, the cross-project unit tests).

The simulation that *used* to be the fallback was removed (you asked to focus on the real glove), so the live connection is the demo — hence the prep checklist above. Test it on your phone the night before. You've got this. 💚
