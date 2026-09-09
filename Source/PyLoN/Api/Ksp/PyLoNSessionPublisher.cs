using System;
using System.Net.Sockets;
using UnityEngine;

namespace PyLoN
{
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
                var bytes = TelemetryPacketCodec.Encode(json);
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
