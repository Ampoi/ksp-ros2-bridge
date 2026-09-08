using static PyLoN.JsonPacketWriter;
using System;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    public sealed partial class PyLoNVehicleManager
    {
        private Guid imuVesselId;
        private double imuSampleTime = double.NaN;
        private float nextImuTime;

        public void LateUpdate()
        {
            // Read completed physics samples, independently of active-vessel
            // control and without requiring a sensor PartModule on any craft.
            if (stateClient == null || Time.realtimeSinceStartup < nextImuTime) return;
            nextImuTime = Time.realtimeSinceStartup + 1f / StateRateHz;
            var target = FlightGlobals.ActiveVessel;
            if (target == null || target.packed || target.ReferenceTransform == null)
            {
                imuSampleTime = double.NaN;
                return;
            }
            var now = Planetarium.GetUniversalTime();
            var previous = imuSampleTime;
            imuSampleTime = now;
            if (imuVesselId != target.id)
            {
                imuVesselId = target.id;
                return;
            }
            // Warm up after unpack; avoid duplicate paused samples and time jumps.
            if (double.IsNaN(previous) || now <= previous) return;

            var reference = target.ReferenceTransform;
            // KSP computes perturbation_immediate as acceleration_immediate
            // minus graviticAcceleration, in world axes and m/s^2. This is
            // specific force: +g upward at rest, approximately 0 in free fall.
            Vector3 force = (Vector3)target.perturbation_immediate;
            var acceleration = new Vector3(
                Vector3.Dot(force, reference.up),
                Vector3.Dot(force, -reference.right),
                Vector3.Dot(force, -reference.forward));
            // Angular velocity is axial: include the handedness sign, as
            // in VesselAngularVelocityBody(), not the polar-vector mapping.
            var local = target.angularVelocity;
            var gyro = FrameConversions.VesselLocalAxialToBody(local);
            // Unity physics changes between surface-rotating and inertial
            // frames at the body's rotation threshold. A physical gyro always
            // measures inertial rotation, including the planet's spin on ground.
            if (FlightGlobals.RefFrameIsRotating)
            {
                var spin = FrameConversions.PlanetSpinAxisWorld(target.mainBody);
                gyro += new Vector3(Vector3.Dot(spin, reference.up),
                    Vector3.Dot(spin, -reference.right), Vector3.Dot(spin, -reference.forward));
            }
            if (!FiniteImuVector(acceleration) || !FiniteImuVector(gyro)) return;

            var builder = new StringBuilder(384);
            builder.Append('{');
            AppendString(builder, "type", "pylon_imu", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", target.id.ToString("N"));
            AppendNumber(builder, "universalTime", now);
            AppendVector(builder, "angularVelocity", gyro);
            AppendVector(builder, "linearAcceleration", acceleration);
            builder.Append('}');
            Send(builder.ToString());
        }

        // Physical right-hand spin axis (north), expressed in Unity world axes.
        // This is a direction, not a Unity axial-vector encoding.


        private static bool FiniteImuVector(Vector3 value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }
    }
}
