using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
namespace PyLoN
{
    /// <summary>Process identity plus a monotonically increasing flight generation.</summary>
    internal static class RuntimeSession
    {
        public static readonly string InstanceId = Guid.NewGuid().ToString("N");
        public static long Generation { get; private set; }
        public static string Epoch { get; private set; } = Guid.NewGuid().ToString("N");
        public static string VesselId { get { return active == null ? "" : active.id.ToString("N"); } }
        public static string VesselName { get { return active == null ? "" : active.vesselName; } }
        public static bool Available { get { return active != null && !active.packed; } }
        public static event Action Changed;
        private static Vessel active;
        private static double previousTime = double.NaN;
        private static int topology;
        private static CelestialBody body;
        private static Vector3d position;
        private static double speed;
        private static bool available;

        public static void Observe()
        {
            var target = HighLogic.LoadedSceneIsFlight ? FlightGlobals.ActiveVessel : null;
            var now = Planetarium.fetch == null ? 0 : Planetarium.GetUniversalTime();
            int nextTopology = 17;
            if (target != null)
                foreach (var part in target.parts)
                    if (part != null) unchecked { nextTopology = nextTopology * 31 + part.persistentId.GetHashCode(); }
            var nextBody = target == null ? null : target.mainBody;
            var nextPosition = nextBody == null ? Vector3d.zero : target.GetWorldPos3D() - nextBody.position;
            var nextSpeed = target == null ? 0 : target.obt_velocity.magnitude;
            bool nextAvailable = target != null && !target.packed;
            double elapsed = now - previousTime;
            // Body-relative coordinates survive floating-origin shifts. Allow physical
            // motion and numerical noise; a discontinuous relocation starts an epoch.
            bool relocated = available && nextAvailable && target == active && nextBody == body && elapsed >= 0 &&
                (nextPosition - position).magnitude > 50.0 + Math.Max(speed, nextSpeed) * elapsed * 2.0;
            bool reset = active != target || topology != nextTopology || body != nextBody || available != nextAvailable || relocated ||
                (!double.IsNaN(previousTime) && now < previousTime);
            active = target; previousTime = now; topology = nextTopology; body = nextBody;
            position = nextPosition; speed = nextSpeed; available = nextAvailable;
            if (reset) Reset();
        }
        public static void Reset()
        {
            Generation++;
            Epoch = Guid.NewGuid().ToString("N");
            if (Changed != null) Changed();
        }
        public static string Wrap(string json)
        {
            Observe();
            if (string.IsNullOrEmpty(json) || json[0] != '{') throw new ArgumentException("Telemetry must be a JSON object");
            return "{\"runtimeInstance\":\"" + InstanceId + "\",\"runtimeGeneration\":" + Generation.ToString(CultureInfo.InvariantCulture) +
                ",\"runtimeEpoch\":\"" + Epoch + "\",\"runtimeVesselId\":\"" + VesselId + "\"," + json.Substring(1);
        }
        public static bool Accept(PyLoNCommandEnvelope command)
        {
            Observe();
            return Available && command != null && command.version == 1 && command.type != null && command.type.StartsWith("pylon_", StringComparison.Ordinal) &&
                command.runtimeInstance == InstanceId && command.runtimeGeneration == Generation &&
                command.runtimeEpoch == Epoch && command.runtimeVesselId == VesselId;
        }
    }

    /// <summary>Flight identity is published even with no LiDAR or truth consumer.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class PyLoNSessionPublisher : MonoBehaviour
    {
        private readonly UdpClient client = new UdpClient();
        private float nextSend;
        public void Update()
        {
            RuntimeSession.Observe();
            if (Time.realtimeSinceStartup < nextSend) return;
            nextSend = Time.realtimeSinceStartup + 0.1f;
            try
            {
                var json = JsonUtility.ToJson(new SessionPacket { type = "pylon_session", version = 1,
                    vesselId = RuntimeSession.VesselId, vesselName = RuntimeSession.VesselName,
                    available = RuntimeSession.Available, universalTime = Planetarium.GetUniversalTime() });
                var bytes = Encoding.UTF8.GetBytes(RuntimeSession.Wrap(json));
                client.Send(bytes, bytes.Length, RuntimeSettings.StateHost, RuntimeSettings.StatePort);
            }
            catch (Exception ex) { Debug.LogWarning("[PyLoN] Session transport: " + ex.Message); }
        }
        public void OnDestroy() { client.Close(); }
        [Serializable] private sealed class SessionPacket
        {
            public string type, vesselId, vesselName;
            public int version;
            public bool available;
            public double universalTime;
        }
    }
}
