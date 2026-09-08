using UnityEngine;
using System;
namespace PyLoN
{
    /// <summary>Unity adapter for the ROS right-handed, SI frame contract.</summary>
    internal static class FrameConversions
    {
        public const float KilonewtonsToNewtons = 1000f;
        public const float TonnesToKilograms = 1000f;
        public static Vector3 UnityVectorToRos(Vector3 value)
        { return new Vector3(value.z, -value.x, value.y); }
        public static Vector3 UnityAxialToRos(Vector3 value)
        { return -UnityVectorToRos(value); }
        public static Quaternion UnityRotationToRos(Quaternion value)
        { return new Quaternion(-value.z, value.x, -value.y, value.w).normalized; }
        public static Vector3 VesselLocalAxialToBody(Vector3 value)
        { return new Vector3(-value.y, value.x, value.z); }
        public static Vector3 WorldToBody(Transform reference, Vector3 world)
        { return new Vector3(Vector3.Dot(world, reference.up), Vector3.Dot(world, -reference.right), Vector3.Dot(world, -reference.forward)); }
        internal static Vector3 PlanetSpinAxisWorld(CelestialBody body)
        {
            if (body == null || !body.rotates || Math.Abs(body.rotationPeriod) < 1e-6)
                return Vector3.zero;
            return body.transform.up.normalized * (float)(2.0 * Math.PI / body.rotationPeriod);
        }
    }
}
