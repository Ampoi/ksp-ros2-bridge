using System;
using System.Globalization;
using UnityEngine;

namespace KerbalLiDAR
{
    /// <summary>One model producer per Flight scene, including craft with no LiDAR.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed partial class KerbalRosVesselModelManager : MonoBehaviour
    {
        private Vessel vessel;
        private bool activeVesselUrdfEnabled = true;
        private bool udpEnabled = true;
        private bool allowRemoteUrdf;
        private string udpHost = "127.0.0.1";
        private int udpPort = 49010;
        private int maxDatagramBytes = 60000;
        private float activeVesselUrdfRefreshSeconds = 2f;
        private int activeVesselUrdfChunkBytes = 12000;
        private int maxActiveVesselUrdfChunks = 256;

        public void Update()
        {
            var active = FlightGlobals.ActiveVessel;
            if (active != vessel)
            {
                ClearActiveVesselUrdf();
                CloseUdpClient();
                vessel = active;
                LoadModelSettings();
                nextActiveVesselUrdfTime = 0f;
            }
            ProcessActiveVesselUrdf();
        }

        public void OnDestroy()
        {
            ClearActiveVesselUrdf();
            CloseUdpClient();
        }

        private void LoadModelSettings()
        {
            activeVesselUrdfEnabled = udpEnabled = true;
            allowRemoteUrdf = false;
            udpHost = "127.0.0.1";
            udpPort = 49010;
            activeVesselUrdfRefreshSeconds = 2f;
            activeVesselUrdfChunkBytes = 12000;
            maxActiveVesselUrdfChunks = 256;
            maxDatagramBytes = 60000;
            var nodes = GameDatabase.Instance == null ? null : GameDatabase.Instance.GetConfigNodes("KERBAL_ROS2_MODEL");
            if (nodes != null && nodes.Length > 0)
            {
                var node = nodes[0];
                bool parsed;
                if (bool.TryParse(node.GetValue("enabled"), out parsed)) activeVesselUrdfEnabled = parsed;
                if (bool.TryParse(node.GetValue("allowRemoteUrdf"), out parsed)) allowRemoteUrdf = parsed;
                if (!string.IsNullOrEmpty(node.GetValue("udpHost"))) udpHost = node.GetValue("udpHost");
                udpPort = (int)Number(node, "udpPort", 49010, 1, 65535);
                activeVesselUrdfRefreshSeconds = Number(node, "refreshSeconds", 2, 0.5f, 30);
                activeVesselUrdfChunkBytes = (int)Number(node, "chunkBytes", 12000, 128, 48000);
                maxActiveVesselUrdfChunks = (int)Number(node, "maxChunks", 256, 1, 512);
                return;
            }
            if (vessel == null) return;
            foreach (var candidate in vessel.parts)
            {
                foreach (PartModule module in candidate.Modules)
                {
                    var lidar = module as ModuleKerbalLidar;
                    if (lidar == null || !lidar.activeVesselUrdfEnabled || !lidar.udpEnabled) continue;
                    udpHost = lidar.udpHost;
                    udpPort = lidar.udpPort;
                    allowRemoteUrdf = lidar.allowRemoteUrdf;
                    activeVesselUrdfRefreshSeconds = Mathf.Clamp(lidar.activeVesselUrdfRefreshSeconds, 0.5f, 30f);
                    activeVesselUrdfChunkBytes = Mathf.Clamp(lidar.activeVesselUrdfChunkBytes, 128, 48000);
                    maxActiveVesselUrdfChunks = Mathf.Clamp(lidar.maxActiveVesselUrdfChunks, 1, 512);
                    maxDatagramBytes = Mathf.Clamp(lidar.maxDatagramBytes, 1152, 65507);
                    return;
                }
            }
        }

        private static float Number(ConfigNode node, string key, float fallback, float min, float max)
        {
            float value;
            return float.TryParse(node.GetValue(key), NumberStyles.Float, CultureInfo.InvariantCulture, out value)
                && !float.IsNaN(value) && !float.IsInfinity(value) ? Mathf.Clamp(value, min, max) : fallback;
        }
    }
}
