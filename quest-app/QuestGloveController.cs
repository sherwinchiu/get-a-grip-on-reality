// =============================================================================
//  QuestGloveController.cs  --  Unity MonoBehaviour that drives a VR hand rig
//  from the glove's BLE stream, and writes force feedback back to the glove.
// =============================================================================
//
//  WHAT THIS DOES (and what you still wire up in the Unity editor):
//    * Parsing/grab/force logic is DONE (GloveProtocol.cs) — fully reusable.
//    * You assign the hand-rig joint Transforms in the Inspector.
//    * You connect ONE BLE plugin (see README) and route its two hooks:
//        - incoming notify bytes  -> HandlePacket(byte[])
//        - this script's SendForce delegate -> the plugin's "write" method
//      This script is intentionally BLE-plugin-agnostic so you can use any
//      Android BLE plugin (Quest is Android, so native BLE works — unlike the
//      Quest Browser, which blocks Web Bluetooth).
//
//  Axis/sign conventions depend on YOUR rig — the TODO comments call out the few
//  numbers to flip if a finger bends the wrong way.
// =============================================================================
using System;
using UnityEngine;
using GetAGrip;

public class QuestGloveController : MonoBehaviour
{
    [Header("Hand rig (assign in Inspector, thumb..pinky order)")]
    public Transform[] mcpJoints = new Transform[5];   // knuckle joints
    public Transform[] pipJoints = new Transform[5];   // middle joints
    public Transform   wrist;                            // optional: tilts with the IMU

    [Header("Kinematics (match the web demo)")]
    public float mcpMaxDeg = 74f;     // full-curl flexion at the knuckle
    public float pipMaxDeg = 89f;     // full-curl flexion at the middle joint
    public float splayMaxDeg = 15f;   // sideways spread
    public float tiltScale = 0.8f;    // model-deg per IMU-deg (wrist tilt)
    public float smooth = 0.25f;      // 0..1 lerp factor per frame

    [Header("Active force profile (engage, force in 0..1)")]
    public float engage = 0.45f;
    public float force  = 0.60f;
    public bool  objectInHand = true; // set true while a grabbable is held

    // Wire these two to your BLE plugin (see README):
    //   plugin "value changed" callback  -> controller.HandlePacket(bytes)
    //   controller.SendForce             -> plugin.Write(serviceUUID, writeUUID, bytes)
    public Action<byte[]> SendForce;

    GloveState _state;
    bool _haveState;
    bool _grabbing;

    // Called by your BLE plugin whenever a notify packet arrives.
    public void HandlePacket(byte[] data)
    {
        try { _state = GloveProtocol.Parse(data); _haveState = true; }
        catch (Exception) { /* short/garbled packet — drop it */ }
    }

    void Update()
    {
        if (!_haveState) return;
        float t = 1f - Mathf.Pow(1f - smooth, Time.deltaTime * 60f); // frame-rate-independent lerp

        float[] splays = GloveProtocol.SplayToFingers(_state.splayRaw);
        for (int f = 0; f < 5; f++)
        {
            var fg = _state.fingers[f];
            // TODO: if a finger bends the wrong way, negate the angle or change the axis (X/Y/Z).
            if (mcpJoints[f]) mcpJoints[f].localRotation = Quaternion.Slerp(mcpJoints[f].localRotation,
                Quaternion.Euler(fg.mcp * mcpMaxDeg, splays[f] * splayMaxDeg, 0f), t);
            if (pipJoints[f]) pipJoints[f].localRotation = Quaternion.Slerp(pipJoints[f].localRotation,
                Quaternion.Euler(fg.pip * pipMaxDeg, 0f, 0f), t);
        }

        // Whole-hand tilt from the fused IMU (only when the firmware streams it).
        if (wrist && _state.hasOrientation)
        {
            var target = Quaternion.Euler(_state.pitch * tiltScale, _state.yaw * tiltScale, -_state.roll * tiltScale);
            wrist.localRotation = Quaternion.Slerp(wrist.localRotation, target, t);
        }

        // Grab -> force feedback (rate-limit your SendForce in the plugin layer to ~1/9ms).
        bool grab = objectInHand && GloveProtocol.GraspPoseMet(_state);
        if (grab != _grabbing)
        {
            _grabbing = grab;
            SendForce?.Invoke(grab ? GloveProtocol.BuildForce(engage, force) : GloveProtocol.ZeroForce);
        }
    }

    public bool IsGrabbing => _grabbing;
    public GloveState State => _state;
}
