using System;
using System.Text;
using UnityEngine;
using static PyLoN.JsonPacketWriter;
namespace PyLoN
{
    /// <summary>Optional truth output and its reference origin, independent of control authority.</summary>
    internal sealed class VesselTelemetry
    {
        private const int ProtocolVersion = 1;
        private readonly Func<Vessel> currentVessel;
        private readonly Action<string> Send;
        private Vessel vessel { get { return currentVessel(); } }
        public VesselTelemetry(Func<Vessel> currentVessel, Action<string> send)
        { this.currentVessel = currentVessel; Send = send; }
        private Vector3 WorldVectorToBody(Vector3 value) { return FrameConversions.WorldToBody(vessel.ReferenceTransform, value); }
        private Vector3 VesselAngularVelocityBody() { return FrameConversions.VesselLocalAxialToBody(vessel.angularVelocity); }
        private Vector3 VesselAngularVelocityWorld() { return vessel.ReferenceTransform.rotation * vessel.angularVelocity; }
        private Vector3 BodyForwardWorld() { return vessel.ReferenceTransform.up.normalized; }
        private Vector3 BodyLeftWorld() { return -vessel.ReferenceTransform.right.normalized; }
        private Vector3 BodyUpWorld() { return -vessel.ReferenceTransform.forward.normalized; }
        internal CelestialBody anchorBody;
        internal Vector3d anchorPosition;
        internal Vector3d anchorEast;
        internal Vector3d anchorNorth;
        internal Vector3d anchorUp;
        internal Vector3 previousLinearVelocity;
        internal Vector3 previousAngularVelocity;
        internal double previousTruthTime;
        internal bool derivativeReady;
        internal int originSequence;

        internal void SendGroundTruth()
        {
            if (vessel == null || anchorBody == null) return;
            Vector3d currentEast;
            Vector3d currentNorth;
            Vector3d currentUp;
            TangentBasis(vessel.latitude, vessel.longitude, out currentEast, out currentNorth, out currentUp);
            var bodyFixedPosition = GeodeticPosition(vessel.latitude, vessel.longitude, vessel.altitude, anchorBody.Radius);
            var delta = bodyFixedPosition - anchorPosition;
            var position = ProjectToAnchor(delta);

            var linearVelocity = WorldVectorToAnchor(vessel.srf_velocity, currentEast, currentNorth, currentUp);
            var angularVelocity = -WorldVectorToAnchor(
                VesselAngularVelocityWorld(), currentEast, currentNorth, currentUp);
            // Publish directly from KSP's vessel basis. Angular velocity and
            // torque use axial-vector conversions, including the handedness
            // sign, so they agree with right-handed ROS quaternion derivatives.
            var linearVelocityBody = WorldVectorToBody(vessel.srf_velocity);
            var angularVelocityBody = VesselAngularVelocityBody();
            var planetSpin = FrameConversions.PlanetSpinAxisWorld(anchorBody);
            var frameAngularVelocity = WorldVectorToAnchor(planetSpin, currentEast, currentNorth, currentUp);
            // The ENU truth frame is body-fixed, even when Unity physics is not.
            if (!FlightGlobals.RefFrameIsRotating)
            {
                angularVelocity -= frameAngularVelocity;
                angularVelocityBody -= WorldVectorToBody(planetSpin);
            }
            var now = Planetarium.GetUniversalTime();
            var elapsed = Math.Max(0.0001, now - previousTruthTime);
            var linearAcceleration = derivativeReady
                ? (linearVelocity - previousLinearVelocity) / (float)elapsed
                : Vector3.zero;
            var angularAcceleration = derivativeReady
                ? (angularVelocity - previousAngularVelocity) / (float)elapsed
                : Vector3.zero;
            derivativeReady = true;
            previousTruthTime = now;
            previousLinearVelocity = linearVelocity;
            previousAngularVelocity = angularVelocity;

            var bodyX = WorldDirectionToAnchor(BodyForwardWorld(), currentEast, currentNorth, currentUp);
            var bodyY = WorldDirectionToAnchor(BodyLeftWorld(), currentEast, currentNorth, currentUp);
            var bodyZ = WorldDirectionToAnchor(BodyUpWorld(), currentEast, currentNorth, currentUp);
            var rotation = QuaternionFromBasis(bodyX, bodyY, bodyZ);

            var builder = new StringBuilder(768);
            builder.Append('{');
            AppendString(builder, "type", "pylon_ground_truth", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", vessel.id.ToString("N"));
            AppendString(builder, "vessel", vessel.vesselName);
            AppendNumber(builder, "originSequence", originSequence);
            AppendNumber(builder, "universalTime", Planetarium.GetUniversalTime());
            AppendVector(builder, "position", position);
            AppendQuaternion(builder, "rotation", rotation);
            AppendVector(builder, "linearVelocity", linearVelocity);
            AppendVector(builder, "angularVelocity", angularVelocity);
            AppendVector(builder, "linearVelocityBody", linearVelocityBody);
            AppendVector(builder, "angularVelocityBody", angularVelocityBody);
            AppendVector(builder, "frameAngularVelocity", frameAngularVelocity);
            AppendVector(builder, "linearAcceleration", linearAcceleration);
            AppendVector(builder, "angularAcceleration", angularAcceleration);
            builder.Append('}');
            Send(builder.ToString());
            SendNearbyVessels(now, position, linearVelocity, currentEast, currentNorth, currentUp);
        }
        private void SendNearbyVessels(double now, Vector3 position, Vector3 velocity,
            Vector3d east, Vector3d north, Vector3d up)
        {
            // The observer and every target share one origin and physics sample.
            // Convert CoM differences before adding the absolute origin so Unity
            // floating-origin shifts and large orbital coordinates cancel out.
            var builder = new StringBuilder(2048);
            builder.Append('{');
            AppendString(builder, "type", "pylon_nearby_vessels", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", vessel.id.ToString("N"));
            AppendNumber(builder, "originSequence", originSequence);
            AppendNumber(builder, "universalTime", now);
            AppendVector(builder, "position", position);
            AppendVector(builder, "linearVelocity", velocity);
            Prefix(builder, "vessels", false);
            builder.Append('[');
            var count = 0;
            foreach (var other in FlightGlobals.VesselsLoaded)
            {
                if (other == null || other == vessel || other.packed || other.mainBody != anchorBody)
                    continue;
                var offset = other.CurrentCoM - vessel.CurrentCoM;
                if (offset.sqrMagnitude > 2500.0 * 2500.0) continue;
                // Bound the UDP datagram to well below its 65507-byte limit.
                if (count >= 32) break;
                if (count++ > 0) builder.Append(',');
                var relativePosition = WorldVectorToAnchor(offset, east, north, up);
                var relativeVelocity = WorldVectorToAnchor(other.srf_velocity - vessel.srf_velocity, east, north, up);
                builder.Append('{');
                AppendString(builder, "vesselId", other.id.ToString("N"), true);
                AppendString(builder, "vessel", other.vesselName);
                AppendBoolean(builder, "isDebris", other.vesselType == VesselType.Debris);
                // Add in double precision: casting the large absolute result to
                // Vector3 would quantize away metre/submetre relative motion.
                AppendDoubleVector(builder, "position", new Vector3d(position.x, position.y, position.z) +
                    new Vector3d(relativePosition.x, relativePosition.y, relativePosition.z));
                AppendDoubleVector(builder, "linearVelocity", new Vector3d(velocity.x, velocity.y, velocity.z) +
                    new Vector3d(relativeVelocity.x, relativeVelocity.y, relativeVelocity.z));
                builder.Append('}');
            }
            builder.Append(']').Append('}');
            Send(builder.ToString());
        }
        internal void ResetGroundTruthOrigin()
        {
            if (vessel == null || vessel.mainBody == null)
            {
                return;
            }
            anchorBody = vessel.mainBody;
            anchorPosition = GeodeticPosition(vessel.latitude, vessel.longitude, vessel.altitude, anchorBody.Radius);
            TangentBasis(vessel.latitude, vessel.longitude, out anchorEast, out anchorNorth, out anchorUp);
            previousTruthTime = Planetarium.GetUniversalTime();
            previousLinearVelocity = Vector3.zero;
            previousAngularVelocity = Vector3.zero;
            derivativeReady = false;
            originSequence++;
        }
        private static Vector3d GeodeticPosition(double latitudeDegrees, double longitudeDegrees, double altitude, double radius)
        {
            var latitude = latitudeDegrees * Math.PI / 180.0;
            var longitude = longitudeDegrees * Math.PI / 180.0;
            var distance = radius + altitude;
            var cosLatitude = Math.Cos(latitude);
            return new Vector3d(
                distance * cosLatitude * Math.Cos(longitude),
                distance * cosLatitude * Math.Sin(longitude),
                distance * Math.Sin(latitude));
        }
        private static void TangentBasis(double latitudeDegrees, double longitudeDegrees,
            out Vector3d east, out Vector3d north, out Vector3d up)
        {
            var latitude = latitudeDegrees * Math.PI / 180.0;
            var longitude = longitudeDegrees * Math.PI / 180.0;
            east = new Vector3d(-Math.Sin(longitude), Math.Cos(longitude), 0.0);
            north = new Vector3d(
                -Math.Sin(latitude) * Math.Cos(longitude),
                -Math.Sin(latitude) * Math.Sin(longitude),
                Math.Cos(latitude));
            up = new Vector3d(
                Math.Cos(latitude) * Math.Cos(longitude),
                Math.Cos(latitude) * Math.Sin(longitude),
                Math.Sin(latitude));
        }
        private Vector3 ProjectToAnchor(Vector3d value)
        {
            return new Vector3((float)Vector3d.Dot(value, anchorEast),
                (float)Vector3d.Dot(value, anchorNorth), (float)Vector3d.Dot(value, anchorUp));
        }
        private Vector3 WorldVectorToAnchor(Vector3d world, Vector3d currentEast, Vector3d currentNorth, Vector3d currentUp)
        {
            var eastWorld = vessel.east.normalized;
            var northWorld = vessel.north.normalized;
            var upWorld = vessel.upAxis.normalized;
            var bodyFixed = currentEast * Vector3d.Dot(world, eastWorld) +
                            currentNorth * Vector3d.Dot(world, northWorld) +
                            currentUp * Vector3d.Dot(world, upWorld);
            return ProjectToAnchor(bodyFixed);
        }
        private Vector3 WorldDirectionToAnchor(Vector3 world, Vector3d currentEast, Vector3d currentNorth, Vector3d currentUp)
        {
            return WorldVectorToAnchor(new Vector3d(world.x, world.y, world.z), currentEast, currentNorth, currentUp).normalized;
        }
        private static Quaternion QuaternionFromBasis(Vector3 x, Vector3 y, Vector3 z)
        {
            var m00 = x.x; var m01 = y.x; var m02 = z.x;
            var m10 = x.y; var m11 = y.y; var m12 = z.y;
            var m20 = x.z; var m21 = y.z; var m22 = z.z;
            var trace = m00 + m11 + m22;
            float qx, qy, qz, qw;
            if (trace > 0f)
            {
                var s = Mathf.Sqrt(trace + 1f) * 2f;
                qw = 0.25f * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                var s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
                qw = (m21 - m12) / s; qx = 0.25f * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                var s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
                qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25f * s; qz = (m12 + m21) / s;
            }
            else
            {
                var s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
                qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25f * s;
            }
            return new Quaternion(qx, qy, qz, qw).normalized;
        }
    }
}
