using System;
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
        public static bool Accept(PyLoNCommandEnvelope command)
        {
            Observe();
            return Available && command != null && command.version == 1 && command.type != null && command.type.StartsWith("pylon_", StringComparison.Ordinal) &&
                command.runtimeInstance == InstanceId && command.runtimeGeneration == Generation &&
                command.runtimeEpoch == Epoch && command.runtimeVesselId == VesselId;
        }
    }

}
