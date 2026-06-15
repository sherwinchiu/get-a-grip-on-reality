/* ============================================================================
   Unit tests for glove-protocol.js  ──  run with:  node --test
   No dependencies (uses Node's built-in test runner + assert).

   These tests pin the byte-level BLE contract so it can't drift from the
   firmware. They cover: parse/encode round-trips, endianness, clamping, the
   optional IMU tail, splay mapping, force packing, grab detection, the
   simulation source, and a live cross-check against the firmware's own UUIDs.
   ========================================================================== */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import * as P from '../glove-protocol.js';

const __dirname = dirname(fileURLToPath(import.meta.url));

/* ---------- helpers ---------- */
const approx = (a, b, eps = 1e-3) => Math.abs(a - b) <= eps;

/* ===========================================================================
   1. CONSTANTS / CONTRACT
   =========================================================================== */
test('GATT UUIDs are the expected lowercase 128-bit values', () => {
  assert.equal(P.GATT.service, '7241bbc8-8ed8-4729-85ea-0ffc63248b4f');
  assert.equal(P.GATT.notify,  '34797cc3-9e74-42e1-a669-be3cbdbae64d');
  assert.equal(P.GATT.write,   '36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4');
  assert.equal(P.GATT.deviceNamePrefix, 'FYDPGlove');
  for (const u of [P.GATT.service, P.GATT.notify, P.GATT.write]) {
    assert.match(u, /^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/);
  }
});

test('packet size constants', () => {
  assert.equal(P.PACKET_BASE_BYTES, 32);
  assert.equal(P.PACKET_IMU_BYTES, 44);
  assert.equal(P.FORCE_BYTES, 10);
});

test('FORCE_PROFILES are well-formed (engage/force in 0..1)', () => {
  for (const key of ['ball', 'cube', 'mouse']) {
    const p = P.FORCE_PROFILES[key];
    assert.ok(p, `${key} profile exists`);
    assert.ok(p.engage >= 0 && p.engage <= 1, `${key} engage in range`);
    assert.ok(p.force  >= 0 && p.force  <= 1, `${key} force in range`);
  }
});

/* ===========================================================================
   2. PARSE / ENCODE ROUND-TRIP  (encode is the inverse of parse)
   =========================================================================== */
test('round-trip: every field survives encode -> parse', () => {
  const bend  = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]; // 5 fingers x 2 joints
  const splay = [2000, 100, 3000, 4095];
  const dv = P.encodeInputPacket({ battery: 42, bend, splay, button: 1, joy: { x: 200, y: 30 } });
  assert.equal(dv.byteLength, 32);

  const s = P.parsePacket(dv);
  assert.equal(s.battery, 42);
  assert.equal(s.button, 1);
  assert.equal(s.joy.x, 200);
  assert.equal(s.joy.y, 30);
  for (let f = 0; f < 5; f++) {
    assert.equal(s.fingers[f].mcpRaw, bend[f * 2],     `finger ${f} MCP raw`);
    assert.equal(s.fingers[f].pipRaw, bend[f * 2 + 1], `finger ${f} PIP raw`);
  }
  assert.deepEqual(s.splayRaw, splay);
});

test('parsePacket accepts a raw ArrayBuffer too', () => {
  const dv = P.encodeInputPacket({ battery: 7 });
  const s = P.parsePacket(dv.buffer);          // pass ArrayBuffer, not DataView
  assert.equal(s.battery, 7);
});

/* ===========================================================================
   3. ENDIANNESS  (firmware packs LITTLE-endian: LSB first)
   =========================================================================== */
test('16-bit values are little-endian (LSB at lower address)', () => {
  // 0x0ABC = 2748. LE bytes: [0]=0xBC (LSB), [1]=0x0A (MSB)
  const dv = P.encodeInputPacket({ bend: [0x0ABC, 0, 0, 0, 0, 0, 0, 0, 0, 0] });
  assert.equal(dv.getUint8(1), 0xBC, 'low byte first');
  assert.equal(dv.getUint8(2), 0x0A, 'high byte second');
  assert.equal(P.parsePacket(dv).fingers[0].mcpRaw, 0x0ABC);
});

/* ===========================================================================
   4. CLAMPING / NORMALISATION
   =========================================================================== */
test('bend normalises 0 -> 0.0 and 4095 -> 1.0', () => {
  const dv = P.encodeInputPacket({ bend: [0, 4095, 0, 0, 0, 0, 0, 0, 0, 0] });
  const s = P.parsePacket(dv);
  assert.ok(approx(s.fingers[0].mcp, 0.0), 'straight finger -> 0');
  assert.ok(approx(s.fingers[0].pip, 1.0), 'fully bent -> 1');
});

test('clamp / clamp01 helpers', () => {
  assert.equal(P.clamp(5, 0, 10), 5);
  assert.equal(P.clamp(-3, 0, 10), 0);
  assert.equal(P.clamp(99, 0, 10), 10);
  assert.equal(P.clamp01(1.7), 1);
  assert.equal(P.clamp01(-0.2), 0);
});

/* ===========================================================================
   5. OPTIONAL IMU TAIL
   =========================================================================== */
test('44-byte packet parses orientation; 32-byte yields null', () => {
  const withImu = P.encodeInputPacket({ orientation: { roll: 12.5, pitch: -4.25, yaw: 90.5 } });
  assert.equal(withImu.byteLength, 44);
  const s = P.parsePacket(withImu);
  assert.ok(s.orientation, 'orientation present');
  assert.ok(approx(s.orientation.roll, 12.5));
  assert.ok(approx(s.orientation.pitch, -4.25));
  assert.ok(approx(s.orientation.yaw, 90.5));

  const noImu = P.encodeInputPacket({ battery: 1 });
  assert.equal(P.parsePacket(noImu).orientation, null);
});

test('parsePacket rejects a too-short buffer', () => {
  assert.throws(() => P.parsePacket(new DataView(new ArrayBuffer(10))), RangeError);
});

/* ===========================================================================
   6. SPLAY MAPPING  (mirrors the OpenVR driver's ProcessInput convention)
   =========================================================================== */
test('splayToFingers: middle centred, range -1..1, pinky relative to ring', () => {
  const out = P.splayToFingers([4095, 2048, 0, 2048]);
  assert.equal(out.length, 5);
  assert.ok(approx(out[0], 1.0), 'thumb full splay -> +1');
  assert.equal(out[2], 0, 'middle always centred');
  assert.ok(approx(out[3], -1.0), 'ring 0 -> -1');
  // pinky = clamp(ring + own, -1, 1): ring(-1) + own(0 -> 0.0006) -> ~-1, clamped
  assert.ok(out[4] >= -1 && out[4] <= 1);
});

/* ===========================================================================
   7. FORCE PACKET  (app -> glove)
   =========================================================================== */
test('buildForcePacket: 10 bytes, engage/force scaled 0..255 for all 5 fingers', () => {
  const a = P.buildForcePacket(P.FORCE_PROFILES.cube);   // engage 0.20, force 0.95
  assert.equal(a.length, 10);
  const eng = Math.round(0.20 * 255);   // 51
  const frc = Math.round(0.95 * 255);   // 242
  for (let f = 0; f < 5; f++) {
    assert.equal(a[f * 2], eng,     `finger ${f} engage byte`);
    assert.equal(a[f * 2 + 1], frc, `finger ${f} force byte`);
  }
});

test('buildForcePacket clamps out-of-range profiles', () => {
  const a = P.buildForcePacket({ engage: 2.0, force: -1 });
  assert.equal(a[0], 255, 'engage clamped to 255');
  assert.equal(a[1], 0,   'force clamped to 0');
});

test('ZERO_FORCE is 10 zero bytes (release)', () => {
  assert.equal(P.ZERO_FORCE.length, 10);
  assert.ok(P.ZERO_FORCE.every(b => b === 0));
});

/* ===========================================================================
   8. GRAB DETECTION
   =========================================================================== */
function poseState(curls) {   // curls: array of 5 [mcp,pip] pairs
  return { fingers: curls.map(([mcp, pip]) => ({ mcp, pip })) };
}
test('graspPoseMet: open hand is not a grab', () => {
  assert.equal(P.graspPoseMet(poseState([[0,0],[0,0],[0,0],[0,0],[0,0]])), false);
});
test('graspPoseMet: thumb alone is not a grab', () => {
  assert.equal(P.graspPoseMet(poseState([[0.9,0.9],[0,0],[0,0],[0,0],[0,0]])), false);
});
test('graspPoseMet: thumb + one finger past threshold is a grab', () => {
  assert.equal(P.graspPoseMet(poseState([[0.8,0.8],[0.9,0.9],[0,0],[0,0],[0,0]])), true);
});
test('graspPoseMet: just-below thresholds is not a grab', () => {
  const t = P.GRAB.thumbThreshold, f = P.GRAB.fingerThreshold;
  assert.equal(P.graspPoseMet(poseState([[t-0.01,t-0.01],[f,f],[0,0],[0,0],[0,0]])), false);
});

/* ===========================================================================
   9. SIMULATION SOURCE  (must produce valid, parseable firmware packets)
   =========================================================================== */
test('synthPacket produces a parseable 32-byte packet at any t', () => {
  for (const t of [0, 0.5, 1.0, 1.74533, 3.3, 10]) {
    const dv = P.synthPacket(t);
    assert.equal(dv.byteLength, 32);
    const s = P.parsePacket(dv);
    assert.equal(s.battery, 87);
    assert.ok(s.joy.x >= 0 && s.joy.x <= 255);
    assert.ok(s.joy.y >= 0 && s.joy.y <= 255);
    for (const fg of s.fingers) {
      assert.ok(fg.mcp >= 0 && fg.mcp <= 1, 'mcp in 0..1');
      assert.ok(fg.pip >= 0 && fg.pip <= 1, 'pip in 0..1');
    }
  }
});

test('synthPacket sweeps from open to a full grab over its cycle', () => {
  // envelope peaks near t where cos(1.8t) = -1  => 1.8t = pi => t ~= 1.745
  const grabbed = P.parsePacket(P.synthPacket(1.745));
  const open    = P.parsePacket(P.synthPacket(0.0));
  assert.equal(P.graspPoseMet(grabbed), true,  'closed pose grabs');
  assert.equal(P.graspPoseMet(open),    false, 'open pose does not grab');
});

/* ===========================================================================
   10. CROSS-CHECK AGAINST THE ACTUAL FIRMWARE (skips if firmware not present)
   =========================================================================== */
test('demo UUIDs match the firmware bluetooth.hpp', (t) => {
  const fw = join(__dirname, '..', '..', 'glove_firmware_rtos', 'bluetooth.hpp');
  if (!existsSync(fw)) { t.skip('firmware not found at ' + fw); return; }
  const src = readFileSync(fw, 'utf8');
  const grab = (macro) => {
    const m = src.match(new RegExp(macro + '\\s+"([0-9a-fA-F-]+)"'));
    return m ? m[1].toLowerCase() : null;
  };
  assert.equal(grab('SERVICE_UUID'),            P.GATT.service, 'service UUID matches firmware');
  assert.equal(grab('CHARACTERISTIC_UUID_TX'),  P.GATT.notify,  'notify UUID matches firmware');
  assert.equal(grab('CHARACTERISTIC_UUID_RX'),  P.GATT.write,   'write UUID matches firmware');
});

test('demo device-name prefix matches the firmware advertised name', (t) => {
  const fw = join(__dirname, '..', '..', 'glove_firmware_rtos', 'bluetooth.cpp');
  if (!existsSync(fw)) { t.skip('firmware not found'); return; }
  const src = readFileSync(fw, 'utf8');
  const m = src.match(/BLEDevice::init\("([^"]+)"\)/);
  assert.ok(m, 'found BLEDevice::init name');
  assert.ok(m[1].startsWith(P.GATT.deviceNamePrefix), `advertised "${m[1]}" starts with "${P.GATT.deviceNamePrefix}"`);
});
