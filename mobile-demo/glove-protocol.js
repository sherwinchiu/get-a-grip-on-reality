/* ============================================================================
   glove-protocol.js  ──  PURE protocol layer for the haptic-glove demo.

   This module has NO dependencies (no Three.js, no DOM, no Web Bluetooth), so
   it runs identically in the browser AND in Node for unit testing. It is the
   single source of truth for the byte-level contract with the ESP32 firmware:

       glove  --(32B / 44B notify packet)-->  parsePacket()
       app    --(10B force packet)--------->  buildForcePacket()

   Every offset, width, and endianness here mirrors the firmware
   (glove_firmware_rtos/) and the OpenVR driver (FYDP_Driver). If you change the
   firmware packet, change it here and the unit tests will tell you what broke.
   ========================================================================== */

/* ---- small math helpers (these were missing before — caused a runtime crash) ---- */
export const clamp   = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
export const clamp01 = (v) => clamp(v, 0, 1);

/* ---- BLE GATT contract (must match the firmware's bluetooth.hpp exactly) ---- */
export const GATT = {
  deviceNamePrefix: 'Glove',                              // advertised name (GloveRight / GloveLeft)
  service: '7241bbc8-8ed8-4729-85ea-0ffc63248b4f',        // custom service
  notify:  '34797cc3-9e74-42e1-a669-be3cbdbae64d',        // glove -> app sensor stream (NOTIFY)
  write:   '36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4',        // app -> glove force commands (WRITE)
};

/* ---- Per-object force profiles. engage/force are 0..1 (scaled to 0..255). ---- */
export const FORCE_PROFILES = {
  ball:  { label: 'Ball',  engage: 0.40, force: 0.72, color: 0x9bff1a },  // firm-ish, engages mid-curl
  cube:  { label: 'Cube',  engage: 0.20, force: 0.95, color: 0xff8a1e },  // firm, engages early
  mouse: { label: 'Mouse', engage: 0.55, force: 0.30, color: 0x34e2e2 },  // light
};

/* ---- Grab detection thresholds (normalised curl 0..1) ---- */
export const GRAB = { thumbThreshold: 0.35, fingerThreshold: 0.40, minOtherFingers: 1 };

/* ---- Sensor packet sizes ---- */
export const PACKET_BASE_BYTES = 32;   // battery + bend + splay + button + joystick
export const PACKET_IMU_BYTES  = 44;   // base + roll/pitch/yaw float32 tail (firmware IMU on)
export const FORCE_BYTES        = 10;  // 2 bytes per finger

/* ============================================================================
   parsePacket(dv)  ──  glove -> app
   Byte map (LITTLE-ENDIAN 16-bit values, exactly how the firmware packs them
   via writeUnsignedShortToCharArrayLE: LSB first):
     [0]      battery (0..255; firmware sends 0..100)
     [1..20]  bend  : 5 fingers x 2 joints (MCP, PIP), thumb->pinky, uint16 0..4095
     [21..28] splay : 4 x uint16 0..4095
     [29]     button (0/1)
     [30..31] joystick x, y (0..255)
     [32..43] OPTIONAL roll/pitch/yaw float32 (only when firmware IMU is enabled)
   Accepts any DataView/ArrayBuffer; returns a normalised state object.
   ========================================================================== */
export function parsePacket(dv) {
  // Allow callers to pass an ArrayBuffer or a DataView.
  if (dv instanceof ArrayBuffer) dv = new DataView(dv);
  if (dv.byteLength < PACKET_BASE_BYTES) {
    throw new RangeError(`packet too short: ${dv.byteLength} < ${PACKET_BASE_BYTES} bytes`);
  }

  const s = { battery: 0, fingers: [], splayRaw: [], button: 0, joy: { x: 0, y: 0 }, orientation: null };
  s.battery = dv.getUint8(0);

  // bend: two joints per finger. getUint16(offset, /*littleEndian=*/true)
  for (let f = 0; f < 5; f++) {
    const base = 1 + f * 4;
    const mcpRaw = dv.getUint16(base,     true);
    const pipRaw = dv.getUint16(base + 2, true);
    s.fingers.push({
      mcp: clamp01(mcpRaw / 4095),   // 0 = straight, 1 = fully bent
      pip: clamp01(pipRaw / 4095),
      mcpRaw, pipRaw,
    });
  }

  // splay: 4 little-endian 16-bit values
  for (let i = 0; i < 4; i++) s.splayRaw.push(dv.getUint16(21 + i * 2, true));

  s.button = dv.getUint8(29);
  s.joy.x  = dv.getUint8(30);
  s.joy.y  = dv.getUint8(31);

  // optional IMU orientation tail (firmware appends 12 bytes when its IMU is on)
  if (dv.byteLength >= PACKET_IMU_BYTES) {
    s.orientation = {
      roll:  dv.getFloat32(32, true),
      pitch: dv.getFloat32(36, true),
      yaw:   dv.getFloat32(40, true),
    };
  }
  return s;
}

/* ----------------------------------------------------------------------------
   splayToFingers(splayRaw)  ──  map 4 raw splay values to per-finger spread,
   mirroring the OpenVR driver's ProcessInput convention. On the current
   hardware only the thumb carries a real value; middle stays centred and the
   pinky is relative to the ring.  Returns [thumb, index, middle, ring, pinky]
   each in -1..1.
   -------------------------------------------------------------------------- */
export function splayToFingers(splayRaw) {
  const n = v => clamp(v / 4095, 0, 1) * 2 - 1;   // 0..4095 -> -1..1
  const thumb = n(splayRaw[0]);
  const index = n(splayRaw[1]);
  const ring  = n(splayRaw[2]);
  const pinky = clamp(ring + n(splayRaw[3]), -1, 1);
  return [thumb, index, 0, ring, pinky];
}

/* ============================================================================
   buildForcePacket(profile)  ──  app -> glove, 10 bytes (2 per finger thumb->pinky)
     byte[2f]   = engage point   (0..255)
     byte[2f+1] = force magnitude (0..255)
   NOTE: the current firmware uses byte[2f] (engage) directly as the servo angle
   (0..255 -> 0..180 deg) and ignores byte[2f+1]. Adjust here if the firmware's
   force model changes.
   ========================================================================== */
export function buildForcePacket(profile) {
  const a = new Uint8Array(FORCE_BYTES);
  const eng = Math.round(clamp01(profile.engage) * 255);
  const frc = Math.round(clamp01(profile.force)  * 255);
  for (let f = 0; f < 5; f++) { a[f * 2] = eng; a[f * 2 + 1] = frc; }
  return a;
}

export const ZERO_FORCE = new Uint8Array(FORCE_BYTES);   // release: relax all servos

/* ----------------------------------------------------------------------------
   buildForcePacketPerFinger(forces)  ──  per-finger force, forces[f] in 0..1
   (thumb..pinky). Each finger gets its OWN byte, so a finger only resists when
   *it* is actually touching the object (collision-driven, not a global grasp).
   -------------------------------------------------------------------------- */
export function buildForcePacketPerFinger(forces) {
  const a = new Uint8Array(FORCE_BYTES);
  for (let f = 0; f < 5; f++) {
    const v = Math.round(clamp01(forces[f] || 0) * 255);
    a[f * 2] = v; a[f * 2 + 1] = v;
  }
  return a;
}

/* ----------------------------------------------------------------------------
   graspPoseMet(state)  ──  is the hand in a grabbing pose? (object-agnostic)
   thumb curled past threshold AND >= minOtherFingers other fingers curled.
   The caller ANDs this with "is an object spawned?".
   -------------------------------------------------------------------------- */
export function graspPoseMet(state, grab = GRAB) {
  const avg = fg => (fg.mcp + fg.pip) / 2;
  const thumb = avg(state.fingers[0]);
  let others = 0;
  for (let f = 1; f < 5; f++) if (avg(state.fingers[f]) > grab.fingerThreshold) others++;
  return thumb > grab.thumbThreshold && others >= grab.minOtherFingers;
}

/* ----------------------------------------------------------------------------
   encodeInputPacket(fields)  ──  pack a notify packet the way the FIRMWARE does
   (little-endian). Used by Simulation Mode and by the unit tests as the inverse
   of parsePacket, so a round-trip is provably lossless.
     fields = { battery, bend:[10], splay:[4], button, joy:{x,y}, orientation? }
   Returns a DataView (32 or 44 bytes).
   -------------------------------------------------------------------------- */
export function encodeInputPacket({ battery = 0, bend = [], splay = [], button = 0,
                                    joy = { x: 0, y: 0 }, orientation = null } = {}) {
  const len = orientation ? PACKET_IMU_BYTES : PACKET_BASE_BYTES;
  const dv = new DataView(new ArrayBuffer(len));
  dv.setUint8(0, battery & 0xff);
  for (let f = 0; f < 5; f++) {
    dv.setUint16(1 + f * 4,     (bend[f * 2]     ?? 0) & 0xffff, true);
    dv.setUint16(1 + f * 4 + 2, (bend[f * 2 + 1] ?? 0) & 0xffff, true);
  }
  for (let i = 0; i < 4; i++) dv.setUint16(21 + i * 2, (splay[i] ?? 0) & 0xffff, true);
  dv.setUint8(29, button & 0xff);
  dv.setUint8(30, joy.x & 0xff);
  dv.setUint8(31, joy.y & 0xff);
  if (orientation) {
    dv.setFloat32(32, orientation.roll  || 0, true);
    dv.setFloat32(36, orientation.pitch || 0, true);
    dv.setFloat32(40, orientation.yaw   || 0, true);
  }
  return dv;
}

/* ----------------------------------------------------------------------------
   synthPacket(t)  ──  Simulation Mode source. Synthesises a REAL firmware-format
   packet from a time-based grasp animation and returns it as a DataView, so the
   exact same parsePacket() pipeline runs with or without a glove attached.
   -------------------------------------------------------------------------- */
export function synthPacket(t) {
  const env = 0.5 - 0.5 * Math.cos(t * 1.8);     // open -> close grasp envelope, 0..1
  const bend = [];
  for (let f = 0; f < 5; f++) {
    const lead = f === 0 ? 0 : 0.12 * f;         // fingers trail the thumb slightly
    const c = clamp01(env - lead);
    bend.push(Math.round(c * 4095 * 0.95), Math.round(c * 4095));   // MCP, PIP
  }
  const splay = [Math.round((0.5 + 0.45 * Math.sin(t * 0.7)) * 4095), 2048, 2048, 2048];
  const joy = {
    x: clamp(Math.round(128 + 90 * Math.cos(t * 0.9)), 0, 255),
    y: clamp(Math.round(128 + 90 * Math.sin(t * 0.9)), 0, 255),
  };
  return encodeInputPacket({ battery: 87, bend, splay, button: env > 0.85 ? 1 : 0, joy });
}
