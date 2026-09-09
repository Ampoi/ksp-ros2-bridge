using System;
using System.Globalization;
using System.Text;

namespace PyLoN
{
    /// <summary>Common v1 telemetry envelope; producers supply only their payload.</summary>
    internal static class TelemetryPacketCodec
    {
        public static byte[] Encode(string json)
        {
            RuntimeSession.Observe();
            if (string.IsNullOrEmpty(json) || json[0] != '{') throw new ArgumentException("Telemetry must be a JSON object");
            var wrapped = "{\"runtimeInstance\":\"" + RuntimeSession.InstanceId + "\",\"runtimeGeneration\":" + RuntimeSession.Generation.ToString(CultureInfo.InvariantCulture) +
                ",\"runtimeEpoch\":\"" + RuntimeSession.Epoch + "\",\"runtimeVesselId\":\"" + RuntimeSession.VesselId + "\"," + json.Substring(1);
            return Encoding.UTF8.GetBytes(wrapped);
        }
    }
}
