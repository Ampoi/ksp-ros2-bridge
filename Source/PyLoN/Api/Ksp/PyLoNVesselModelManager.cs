using System;
using System.Globalization;
using UnityEngine;

namespace PyLoN
{
    /// <summary>One model producer per Flight scene, including craft with no LiDAR.</summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed partial class PyLoNVesselModelManager : MonoBehaviour
    {
        private Vessel vessel;
        private bool activeVesselUrdfEnabled = true;
        private bool udpEnabled = true;
        private bool allowRemoteUrdf;
        private string udpHost { get { return RuntimeSettings.StateHost; } }
        private int udpPort { get { return RuntimeSettings.StatePort; } }
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
            activeVesselUrdfRefreshSeconds = 2f;
            activeVesselUrdfChunkBytes = 12000;
            maxActiveVesselUrdfChunks = 256;
            maxDatagramBytes = 60000;
            var nodes = GameDatabase.Instance == null ? null : GameDatabase.Instance.GetConfigNodes("PYLON_MODEL");
            if (nodes != null && nodes.Length > 0)
            {
                var node = nodes[0];
                bool parsed;
                if (bool.TryParse(node.GetValue("enabled"), out parsed)) activeVesselUrdfEnabled = parsed;
                if (bool.TryParse(node.GetValue("allowRemoteUrdf"), out parsed)) allowRemoteUrdf = parsed;
                activeVesselUrdfRefreshSeconds = Number(node, "refreshSeconds", 2, 0.5f, 30);
                activeVesselUrdfChunkBytes = (int)Number(node, "chunkBytes", 12000, 128, 48000);
                maxActiveVesselUrdfChunks = (int)Number(node, "maxChunks", 256, 1, 512);
                return;
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
