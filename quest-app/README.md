# Native Quest VR app — head start

A standalone Meta Quest app (Unity) that connects to the glove over **native Android
Bluetooth** and renders a VR hand + grab game — **no Windows, no SteamVR, no phone bridge**.

> Why native works where the browser doesn't: the **Quest Browser blocks Web Bluetooth**,
> but Quest runs Android (Horizon OS), and a **native app can use the standard Android
> `BluetoothGatt` API** to connect to the glove directly. So native is the way to do
> glove + standalone VR in the headset.

## What's in this folder (done — and yours to explain)

| File | Status |
|------|--------|
| `GloveProtocol.cs` | ✅ **Done.** Exact C# port of the firmware BLE contract — parse packet (bend/splay/battery/button/joystick/IMU, little-endian), grab detection, 10-byte force packet. No Unity dependency, so you can unit-test it too. |
| `QuestGloveController.cs` | ✅ **Skeleton done.** A `MonoBehaviour` that maps parsed packets → finger-joint rotations + wrist tilt, detects grabs, and emits the force packet. Plugin-agnostic. |

These are the parts that are real, correct, and reviewable. The rest is Unity-editor wiring
(below) — which needs the Unity editor + the headset to build and iterate, so it isn't
something that can be produced/verified outside Unity.

## Build steps (in the Unity editor)

1. **Unity** 2022 LTS or newer. Create a 3D (URP) project.
2. **Meta XR SDK** — install the *Meta XR All-in-One SDK* from the Asset Store / Package
   Manager. Use a Building Block or the OVRCameraRig for the VR camera + controllers.
3. **Project settings:** Android platform, Quest as target, IL2CPP / ARM64, and set up the
   Oculus/OpenXR loader (Meta's setup tool does most of this).
4. **BLE plugin** (the one external piece). Quest is Android, so any Android BLE plugin works:
   - Easiest: a Unity Asset Store BLE plugin (e.g. Shatalmic "Bluetooth LE for iOS, tvOS and
     Android"). Subscribe to the **notify** UUID, and use its write method for **force**.
   - Or write a small Kotlin `BluetoothGatt` plugin and bridge via `AndroidJavaProxy`.
5. **Permissions** — add to `AndroidManifest.xml`:
   ```xml
   <uses-permission android:name="android.permission.BLUETOOTH_SCAN" android:usesPermissionFlags="neverForLocation" />
   <uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
   <!-- older Android fallback -->
   <uses-permission android:name="android.permission.BLUETOOTH" android:maxSdkVersion="30" />
   <uses-permission android:name="android.permission.BLUETOOTH_ADMIN" android:maxSdkVersion="30" />
   ```
   Request `BLUETOOTH_CONNECT`/`SCAN` at runtime (Android 12+).
6. **Wire it up:** drop `GloveProtocol.cs` + `QuestGloveController.cs` into `Assets/`. Put the
   controller on your hand rig and assign the 5 MCP + 5 PIP joint Transforms (thumb..pinky)
   and the wrist. Then connect the two hooks:
   - plugin "characteristic changed" → `controller.HandlePacket(bytes)`
   - `controller.SendForce = (bytes) => plugin.Write(Gatt.Service, Gatt.Write, bytes);`
     (rate-limit writes to ~1 per 9 ms in your plugin layer.)
7. **Scan + connect** to the device whose name starts with `FYDPGlove` (`Gatt.DeviceNamePrefix`).
8. **Build & Run** to the Quest (developer mode + USB/adb). Add grabbable objects with
   colliders/rigidbodies; set `objectInHand = true` while one is held to enable force.

## Effort (honest)

- VR scene + hand rig in Unity: known territory, ~days if you know Unity.
- The real effort/risk is the **BLE plugin wiring + Android permissions** — a paid plugin
  makes it ~a day; a custom Kotlin plugin is more.
- Working prototype: ~1–2 weekends if you're comfortable with Unity; longer if learning it.

## Interview talking points

> *"The Quest Browser blocks Web Bluetooth, so for in-headset use I went native: a Unity app
> using Android `BluetoothGatt` for BLE. The packet parsing and force logic are shared with
> my web demo — same byte contract, ported to C# and unit-testable — so the hand mapping and
> grab→force code is identical across web and VR. Finger joints are driven by the two bend
> values per finger, the wrist by the fused IMU, and a grab writes the force packet back."*

This C# parser is a 1:1 port of the firmware contract that's unit-tested on the web side
(`mobile-demo/tests/`), so you can speak to it as verified logic, not guesswork.
