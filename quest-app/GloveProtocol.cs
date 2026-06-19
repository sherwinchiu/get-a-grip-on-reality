// =============================================================================
//  GloveProtocol.cs  --  C# port of the glove BLE contract (for a Unity/Quest app)
// =============================================================================
//
//  This is a direct, dependency-free port of mobile-demo/glove-protocol.js, which
//  itself mirrors the ESP32 firmware (glove_firmware_rtos). It has NO UnityEngine
//  dependency, so you can unit-test it in plain C#/.NET as well as use it in Unity.
//
//  Byte layout (little-endian 16-bit values), notify characteristic, 32 or 44 B:
//     [0]      battery (0..255; firmware sends 0..100)
//     [1..20]  bend  : 5 fingers x 2 joints (MCP, PIP), thumb->pinky, uint16 0..4095
//     [21..28] splay : 4 x uint16 0..4095
//     [29]     button (0/1)
//     [30..31] joystick x, y (0..255)
//     [32..43] OPTIONAL roll/pitch/yaw float32 (only when firmware IMU is enabled)
//
//  Force characteristic (app -> glove), 10 bytes: 2 per finger (engage, force).
// =============================================================================
using System;

namespace GetAGrip
{
    // --- BLE GATT contract (must match the firmware's bluetooth.hpp exactly) ---
    public static class Gatt
    {
        public const string DeviceNamePrefix = "FYDPGlove";
        public const string Service = "7241bbc8-8ed8-4729-85ea-0ffc63248b4f";
        public const string Notify  = "34797cc3-9e74-42e1-a669-be3cbdbae64d"; // glove -> app
        public const string Write   = "36ade52d-4a4c-4b23-9d64-78e6a3e2cdd4"; // app -> glove
    }

    public struct Finger { public float mcp, pip; public int mcpRaw, pipRaw; } // mcp/pip in 0..1

    public struct GloveState
    {
        public int battery;            // 0..100
        public Finger[] fingers;       // length 5, thumb..pinky
        public int[] splayRaw;         // length 4, 0..4095
        public int button;             // 0/1
        public int joyX, joyY;         // 0..255
        public bool hasOrientation;
        public float roll, pitch, yaw; // degrees (only if hasOrientation)
    }

    public static class GloveProtocol
    {
        public const int PacketBaseBytes = 32;
        public const int PacketImuBytes  = 44;
        public const int ForceBytes      = 10;

        static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);

        // Parse a notify packet into a normalised state. Throws if too short.
        public static GloveState Parse(byte[] d)
        {
            if (d == null || d.Length < PacketBaseBytes)
                throw new ArgumentException($"packet too short: {(d == null ? 0 : d.Length)} < {PacketBaseBytes}");

            var s = new GloveState { fingers = new Finger[5], splayRaw = new int[4] };
            s.battery = d[0];

            for (int f = 0; f < 5; f++)
            {
                int b = 1 + f * 4;
                int mcpRaw = d[b]     | (d[b + 1] << 8);   // little-endian
                int pipRaw = d[b + 2] | (d[b + 3] << 8);
                s.fingers[f] = new Finger {
                    mcpRaw = mcpRaw, pipRaw = pipRaw,
                    mcp = Clamp01(mcpRaw / 4095f),          // 0 straight .. 1 fully bent
                    pip = Clamp01(pipRaw / 4095f),
                };
            }
            for (int i = 0; i < 4; i++) s.splayRaw[i] = d[21 + i * 2] | (d[22 + i * 2] << 8);

            s.button = d[29];
            s.joyX = d[30];
            s.joyY = d[31];

            if (d.Length >= PacketImuBytes)
            {
                // Firmware packs little-endian floats; Quest (ARM) is little-endian,
                // so BitConverter (platform-endian) matches directly.
                s.hasOrientation = true;
                s.roll  = BitConverter.ToSingle(d, 32);
                s.pitch = BitConverter.ToSingle(d, 36);
                s.yaw   = BitConverter.ToSingle(d, 40);
            }
            return s;
        }

        // Map the 4 raw splay values to per-finger spread (-1..1), mirroring the
        // driver/web convention: only the thumb is real; middle stays centred.
        public static float[] SplayToFingers(int[] splayRaw)
        {
            Func<int, float> n = v => (Math.Min(Math.Max(v / 4095f, 0f), 1f) * 2f - 1f);
            float thumb = n(splayRaw[0]);
            float index = n(splayRaw[1]);
            float ring  = n(splayRaw[2]);
            float pinky = Math.Min(Math.Max(ring + n(splayRaw[3]), -1f), 1f);
            return new[] { thumb, index, 0f, ring, pinky };
        }

        // 10-byte force packet: 2 per finger [engage, force], each 0..255.
        // (Firmware currently uses the engage byte directly as the servo angle.)
        public static byte[] BuildForce(float engage01, float force01)
        {
            byte eng = (byte)Math.Round(Clamp01(engage01) * 255f);
            byte frc = (byte)Math.Round(Clamp01(force01)  * 255f);
            var a = new byte[ForceBytes];
            for (int f = 0; f < 5; f++) { a[f * 2] = eng; a[f * 2 + 1] = frc; }
            return a;
        }
        public static readonly byte[] ZeroForce = new byte[ForceBytes];

        // Grab pose: thumb + at least one other finger curled past threshold.
        public static bool GraspPoseMet(GloveState s, float thumbTh = 0.35f, float fingerTh = 0.40f, int minOthers = 1)
        {
            float Avg(Finger f) => (f.mcp + f.pip) * 0.5f;
            int others = 0;
            for (int i = 1; i < 5; i++) if (Avg(s.fingers[i]) > fingerTh) others++;
            return Avg(s.fingers[0]) > thumbTh && others >= minOthers;
        }
    }
}
