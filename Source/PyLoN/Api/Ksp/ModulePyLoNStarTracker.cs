using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using PyLoN.Domain;
using UnityEngine;

namespace PyLoN
{
    /// <summary>Engineering star-tracker sensor simulation, not image-based star identification.</summary>
    public sealed class ModulePyLoNStarTracker : PartModule
    {
        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Star Tracker")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool trackerEnabled = true;
        [KSPField(guiActive = true, guiName = "Tracking Status")]
        public string trackingStatus = "acquiring";
        [KSPField] public float sampleRateHz = 5f;
        [KSPField] public float fieldOfViewDegrees = 20f;
        [KSPField] public float sunExclusionDegrees = 35f;
        [KSPField] public float bodyLimbMarginDegrees = 5f;
        [KSPField] public float maxAngularRateDegrees = 2f;
        [KSPField] public float acquisitionSeconds = 2f;
        [KSPField] public float noiseArcsec = 20f;
        [KSPField] public float electricChargePerSecond = 0.05f;
        public string udpHost { get { return RuntimeSettings.StateHost; } }
        public int udpPort { get { return RuntimeSettings.StatePort; } }

        private readonly StarTrackerLock trackerLock = new StarTrackerLock();
        private readonly System.Random noise = new System.Random(Guid.NewGuid().GetHashCode());
        private readonly string sessionId = Guid.NewGuid().ToString("N");
        private UdpClient client;
        private float nextSample, lastWarning = -10f;
        private double previousUt = double.NaN;
        private bool powered, wasActive;
        private Quaternion previousOptical;
        private Guid previousVessel;
        private long sequence;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            sampleRateHz = Bound(sampleRateHz, 1, 20, 5);
            fieldOfViewDegrees = Bound(fieldOfViewDegrees, 5, 60, 20);
            sunExclusionDegrees = Bound(sunExclusionDegrees, fieldOfViewDegrees / 2, 90, 35);
            bodyLimbMarginDegrees = Bound(bodyLimbMarginDegrees, 0, 30, 5);
            maxAngularRateDegrees = Bound(maxAngularRateDegrees, .01f, 20, 2);
            acquisitionSeconds = Bound(acquisitionSeconds, .1f, 60, 2);
            noiseArcsec = Bound(noiseArcsec, .1f, 3600, 20);
            electricChargePerSecond = Bound(electricChargePerSecond, .0001f, 10, .05f);
        }

        public void FixedUpdate()
        {
            powered = false;
            if (!HighLogic.LoadedSceneIsFlight || vessel == null || vessel.packed || !trackerEnabled) return;
            double demand = electricChargePerSecond * TimeWarp.fixedDeltaTime;
            powered = demand > 0 && part.RequestResource("ElectricCharge", demand) >= demand * .999;
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            bool active = HighLogic.LoadedSceneIsFlight && vessel != null && vessel == FlightGlobals.ActiveVessel;
            if (!active)
            {
                if (wasActive) SendState("inactive", Quaternion.identity, 0);
                wasActive = false; trackerLock.Reset(); previousUt = double.NaN;
                return;
            }
            wasActive = true;
            if (Time.realtimeSinceStartup < nextSample) return;
            nextSample = Time.realtimeSinceStartup + 1f / sampleRateHz;
            var now = Planetarium.GetUniversalTime();
            if (previousVessel != vessel.id)
            {
                trackerLock.Reset(); previousUt = double.NaN; previousVessel = vessel.id;
            }
            if (now == previousUt) return; // Paused samples must not refresh ROS validity.
            var optical = part.FindModelTransform("StarTrackerOptical");
            if (optical == null || vessel.ReferenceTransform == null || Planetarium.fetch == null)
            {
                trackerLock.Reset(); SendState("sensor_unavailable", Quaternion.identity, 0); return;
            }
            var opticalAttitude = Attitude(optical.forward, -optical.right, optical.up);
            var dt = now - previousUt;
            float rate = double.IsNaN(previousUt) || dt <= 0 || dt > 2 ? float.PositiveInfinity :
                Quaternion.Angle(previousOptical, opticalAttitude) / (float)dt;
            // The sampled optical displacement also covers moving mounts; an
            // inertial gyro bound prevents whole-turn aliasing between samples.
            rate = Mathf.Max(rate, InertialBodyRate());
            previousUt = now; previousOptical = opticalAttitude;
            string reason = !trackerEnabled ? "disabled" : vessel.packed ? "packed" :
                !powered ? "no_power" : rate > maxAngularRateDegrees ? "slew_rate_exceeded" :
                Visibility(optical);
            trackingStatus = trackerLock.Update(now, reason, acquisitionSeconds);
            var result = Quaternion.identity;
            if (trackingStatus == "tracking")
            {
                var reference = vessel.ReferenceTransform;
                result = Attitude(reference.up, -reference.right, -reference.forward);
                // Small independent isotropic attitude error (radians); covariance is sigma^2 I.
                var error = new Vector3(Gaussian(), Gaussian(), Gaussian()) * (noiseArcsec / 3600f);
                result = (result * Quaternion.AngleAxis(error.magnitude, error.normalized)).normalized;
            }
            SendState(trackingStatus, result, float.IsInfinity(rate) ? 0 : rate);
        }

        private float InertialBodyRate()
        {
            var local = vessel.angularVelocity;
            var gyro = FrameConversions.VesselLocalAxialToBody(local);
            var body = vessel.mainBody;
            if (FlightGlobals.RefFrameIsRotating && body != null && body.rotates && Math.Abs(body.rotationPeriod) > 1e-6)
            {
                var spin = body.transform.up.normalized * (float)(2 * Math.PI / body.rotationPeriod);
                var reference = vessel.ReferenceTransform;
                gyro += new Vector3(Vector3.Dot(spin, reference.up),
                    Vector3.Dot(spin, -reference.right), Vector3.Dot(spin, -reference.forward));
            }
            return gyro.magnitude * Mathf.Rad2Deg;
        }

        private string Visibility(Transform optical)
        {
            var origin = optical.position;
            var axis = optical.forward;
            var sun = Planetarium.fetch.Sun;
            if (sun == null) return "sensor_unavailable";
            // Conservative space-only optical model: no atmospheric star availability claim.
            if (vessel.mainBody != null && vessel.mainBody.atmosphere &&
                vessel.altitude < vessel.mainBody.atmosphereDepth) return "atmosphere";
            foreach (var body in FlightGlobals.Bodies)
            {
                if (body == null) continue;
                Vector3d delta = body.position - (Vector3d)origin;
                double margin = body == sun ? sunExclusionDegrees :
                    fieldOfViewDegrees / 2 + bodyLimbMarginDegrees;
                if (StarTrackerLock.Excluded(Vector3.Angle(axis, (Vector3)delta), delta.magnitude,
                    body.Radius, margin)) return body == sun ? "sun_exclusion" : "body_in_fov";
            }
            // Sample the center and two rings across the circular field. Ignore our own
            // housing; include own-vessel fairings/parts and other loaded craft/terrain.
            for (int i = 0; i < 17; i++)
            {
                float angle = i == 0 ? 0 : (i - 1) * Mathf.PI / 4;
                float radius = i == 0 ? 0 : Mathf.Tan(fieldOfViewDegrees * .5f * Mathf.Deg2Rad) * (i <= 8 ? .5f : 1);
                var direction = (axis + radius * (Mathf.Cos(angle) * optical.right + Mathf.Sin(angle) * optical.up)).normalized;
                foreach (var hit in Physics.RaycastAll(origin, direction, 2000f, ~0, QueryTriggerInteraction.Ignore))
                {
                    var hitPart = hit.collider.GetComponentInParent<Part>();
                    if (hitPart == part) continue;
                    // Celestial spheres are checked above; only flight physical geometry.
                    if (hitPart != null || hit.collider.gameObject.layer == 15) return "occluded";
                }
            }
            return "clear";
        }

        // Fixed celestial basis; Planetarium axes follow Unity's rotating reference
        // frame. The (right, forward, up) reflection makes right-handed ROS XYZ.
        internal static Vector3 Inertial(Vector3 world)
        {
            return new Vector3((float)Vector3d.Dot(world, Planetarium.right),
                (float)Vector3d.Dot(world, Planetarium.forward), (float)Vector3d.Dot(world, Planetarium.up));
        }
        internal static Quaternion Attitude(Vector3 x, Vector3 y, Vector3 z)
        {
            return Quaternion.LookRotation(Inertial(z), Inertial(y)).normalized;
        }

        private void SendState(string reason, Quaternion rotation, float rate)
        {
            if (vessel == null) return;
            trackingStatus = reason;
            bool valid = reason == "tracking";
            var sigma = noiseArcsec * Math.PI / (180 * 3600);
            var packet = new StringBuilder(700);
            packet.Append("{\"type\":\"pylon_star_tracker\",\"version\":1,\"vesselId\":\"").Append(vessel.id.ToString("N"));
            packet.Append("\",\"sensorId\":\"").Append(ModulePyLoNSensorId.Resolve(part, "star_tracker"));
            packet.Append("\",\"sessionId\":\"").Append(sessionId).Append("\",\"sequence\":").Append(++sequence);
            packet.Append(",\"universalTime\":").Append(Number(Planetarium.GetUniversalTime()));
            packet.Append(",\"referenceFrame\":\"kerbol_inertial\",\"measuredFrame\":\"base_link\"");
            packet.Append(",\"valid\":").Append(valid ? "true" : "false");
            packet.Append(",\"reason\":\"").Append(reason).Append("\",\"angularRateDegSec\":").Append(Number(rate));
            packet.Append(",\"noiseStdRad\":").Append(Number(sigma));
            packet.Append(",\"orientation\":");
            if (valid) packet.Append('[').Append(Number(rotation.x)).Append(',').Append(Number(rotation.y))
                .Append(',').Append(Number(rotation.z)).Append(',').Append(Number(rotation.w)).Append(']');
            else packet.Append("null");
            packet.Append('}');
            try
            {
                if (client == null) { client = new UdpClient(); client.Connect(udpHost, udpPort); }
                var bytes = Encoding.UTF8.GetBytes(RuntimeSession.Wrap(packet.ToString())); client.Send(bytes, bytes.Length);
            }
            catch (Exception ex)
            {
                if (client != null) client.Close(); client = null;
                if (Time.realtimeSinceStartup - lastWarning > 5)
                { lastWarning = Time.realtimeSinceStartup; Debug.LogWarning("[PyLoN] Star tracker UDP: " + ex.Message); }
            }
        }
        public void OnDestroy()
        {
            if (wasActive) SendState("inactive", Quaternion.identity, 0);
            if (client != null) client.Close(); client = null;
        }
        private float Gaussian()
        {
            return (float)(Math.Sqrt(-2 * Math.Log(1 - noise.NextDouble())) * Math.Cos(2 * Math.PI * noise.NextDouble()));
        }
        private static string Number(double value) { return value.ToString("R", CultureInfo.InvariantCulture); }
        private static float Bound(float value, float min, float max, float fallback)
        { return float.IsNaN(value) || float.IsInfinity(value) ? fallback : Mathf.Clamp(value, min, max); }
    }
}
