using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    public partial class ModulePyLoNRgbCamera : PartModule
    {
        private const int ProtocolVersion = 1;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Camera")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool cameraEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "UDP")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool udpEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Frame Rate Hz", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 15f, stepIncrement = 1f)]
        public float frameRateHz = 5f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Vertical FOV", guiFormat = "F0")]
        [UI_FloatRange(minValue = 20f, maxValue = 120f, stepIncrement = 5f)]
        public float verticalFovDegrees = 60f;


        [KSPField]
        public int imageWidth = 320;

        [KSPField]
        public int imageHeight = 240;

        public string udpHost { get { return RuntimeSettings.StateHost; } }

        public int udpPort { get { return RuntimeSettings.StatePort; } }

        [KSPField]
        public int frameChunkBytes = 12000;

        [KSPField]
        public int maxFrameBytes = 4194304;

        [KSPField]
        public int maxFrameChunks = 512;

        [KSPField]
        public float nearClipMeters = 0.05f;

        [KSPField]
        public float farClipMeters = 20000f;

        [KSPField]
        public bool useGameClipPlanes = true;

        [KSPField]
        public string cameraOriginLocalPosition = "0, 0, -0.2";

        [KSPField]
        public string forwardAxis = "-Z";

        [KSPField]
        public string upAxis = "+Y";

        [KSPField(guiActive = true, guiName = "Image Size")]
        public string imageSizeStatus = "";

        [KSPField(guiActive = true, guiName = "Last Frame Bytes")]
        public int lastFrameBytes;

        [KSPField(guiActive = true, guiName = "Last Capture ms", guiFormat = "F1")]
        public float lastCaptureMs;

        private readonly string sessionId = Guid.NewGuid().ToString("N");
        private long frameSequence;
        private float nextFrameTime;
        private bool flightTopicActive;
        private KspSceneRgbCapture sceneCapture;
        private UdpClient udpClient;
        private IPEndPoint udpEndPoint;
        private string endpointKey;
        private double lastWarningTime = -1000.0;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            NormalizeConfig();
            UpdateCameraGimbal();
            InitializePreview();
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            NormalizeConfig();
        }

        [KSPEvent(guiActive = true, guiName = "Capture RGB Frame Now", active = true)]
        public void CaptureNow()
        {
            if (HighLogic.LoadedSceneIsFlight && udpEnabled &&
                vessel != null && FlightGlobals.ActiveVessel == vessel)
            {
                CaptureFrame(true);
            }
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Resolution: 160 x 120", active = true)]
        public void SetLowResolution()
        {
            SetResolution(160, 120);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Resolution: 320 x 240", active = true)]
        public void SetMediumResolution()
        {
            SetResolution(320, 240);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Resolution: 640 x 480", active = true)]
        public void SetHighResolution()
        {
            SetResolution(640, 480);
        }

        [KSPAction("Toggle RGB Camera")]
        public void ToggleCameraAction(KSPActionParam action)
        {
            cameraEnabled = !cameraEnabled;
        }

        public void OnDestroy()
        {
            DestroyPreview();
            SendInactive();
            DestroyCaptureResources();
            CloseUdpClient();
        }

        private void CaptureFrame(bool sendUdp)
        {
            NormalizeConfig();
            var started = Time.realtimeSinceStartup;
            previewHasFrame = false;
            try
            {
                var root = part != null && part.transform != null ? part.transform : transform;
                Vector3 origin;
                Quaternion rotation;
                ResolveCameraPose(root, out origin, out rotation);

                if (sceneCapture == null)
                {
                    sceneCapture = new KspSceneRgbCapture();
                }
                sceneCapture.Render(
                    imageWidth,
                    imageHeight,
                    origin,
                    rotation,
                    verticalFovDegrees,
                    nearClipMeters,
                    farClipMeters,
                    useGameClipPlanes);
                previewHasFrame = true;
                previewCaptureFailed = false;
                lastCaptureMs = (Time.realtimeSinceStartup - started) * 1000f;
                if (!sendUdp)
                {
                    return;
                }

                var rgb = sceneCapture.ReadTopDownRgb();
                if (rgb.Length > maxFrameBytes)
                {
                    WarnThrottled("RGB frame is " + rgb.Length + " bytes; select a lower resolution.");
                    return;
                }

                SendFrame(rgb, root, origin, rotation);
                lastFrameBytes = rgb.Length;
                lastCaptureMs = (Time.realtimeSinceStartup - started) * 1000f;
                flightTopicActive = true;
            }
            catch (Exception ex)
            {
                // Keep transport failures separate from successful local rendering.
                previewCaptureFailed = !previewHasFrame;
                WarnThrottled("RGB capture failed: " + ex.Message);
            }
        }

        private void SendFrame(byte[] rgb, Transform partTransform, Vector3 origin, Quaternion rotation)
        {
            EnsureUdpClient();
            frameSequence++;
            var chunkCount = (rgb.Length + frameChunkBytes - 1) / frameChunkBytes;
            if (chunkCount <= 0 || chunkCount > maxFrameChunks)
            {
                WarnThrottled("RGB frame needs " + chunkCount + " UDP chunks; select a lower resolution.");
                return;
            }

            var checksum = Sha256Hex(rgb);
            var framePosition = FrameConversions.UnityVectorToRos(partTransform.InverseTransformPoint(origin));
            var frameRotation = FrameConversions.UnityRotationToRos(Quaternion.Inverse(partTransform.rotation) * rotation);
            var resolvedName = ResolveSensorId();
            var universalTime = Planetarium.GetUniversalTime();
            for (var chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++)
            {
                var offset = chunkIndex * frameChunkBytes;
                var count = Math.Min(frameChunkBytes, rgb.Length - offset);
                var builder = new StringBuilder(count * 2);
                builder.Append('{');
                AppendString(builder, "type", "pylon_camera_frame_chunk", true);
                AppendNumber(builder, "version", ProtocolVersion, false);
                AppendString(builder, "source", "rgb_camera", false);
                AppendString(builder, "sessionId", sessionId, false);
                AppendNumber(builder, "sequence", frameSequence, false);
                AppendNumber(builder, "chunkIndex", chunkIndex, false);
                AppendNumber(builder, "chunkCount", chunkCount, false);
                AppendNumber(builder, "frameBytes", rgb.Length, false);
                AppendString(builder, "sha256", checksum, false);
                AppendString(builder, "sensorId", resolvedName, false);
                AppendString(builder, "vessel", vessel != null ? vessel.vesselName : "", false);
                AppendNumber(builder, "partFlightId", part != null ? (long)part.flightID : 0L, false);
                AppendDouble(builder, "universalTime", universalTime, false);
                AppendNumber(builder, "width", imageWidth, false);
                AppendNumber(builder, "height", imageHeight, false);
                AppendNumber(builder, "step", imageWidth * 3, false);
                AppendString(builder, "encoding", "rgb8", false);
                AppendFloat(builder, "verticalFovDeg", verticalFovDegrees, false);
                AppendString(builder, "coordinateFrame", "ros_sensor", false);
                AppendVector(builder, "framePosition", framePosition, false);
                AppendQuaternion(builder, "frameRotation", frameRotation, false);
                AppendString(builder, "data", Convert.ToBase64String(rgb, offset, count), false);
                builder.Append('}');
                var datagram = TelemetryPacketCodec.Encode(builder.ToString());
                udpClient.Send(datagram, datagram.Length, udpEndPoint);
            }
        }

        private void SendInactive()
        {
            if (!flightTopicActive)
            {
                return;
            }
            try
            {
                EnsureUdpClient();
                var builder = new StringBuilder(192);
                builder.Append('{');
                AppendString(builder, "type", "pylon_camera_inactive", true);
                AppendNumber(builder, "version", ProtocolVersion, false);
                AppendString(builder, "source", "rgb_camera", false);
                AppendString(builder, "sensorId", ResolveSensorId(), false);
                AppendNumber(builder, "partFlightId", part != null ? (long)part.flightID : 0L, false);
                builder.Append('}');
                var datagram = TelemetryPacketCodec.Encode(builder.ToString());
                udpClient.Send(datagram, datagram.Length, udpEndPoint);
            }
            catch (Exception ex)
            {
                WarnThrottled("RGB camera inactive notification failed: " + ex.Message);
            }
            flightTopicActive = false;
        }

        private void DestroyCaptureResources()
        {
            previewHasFrame = false;
            if (sceneCapture != null)
            {
                sceneCapture.Dispose();
                sceneCapture = null;
            }
        }

        private void SetResolution(int width, int height)
        {
            imageWidth = width;
            imageHeight = height;
            imageSizeStatus = width.ToString(CultureInfo.InvariantCulture) + " x " + height.ToString(CultureInfo.InvariantCulture);
            DestroyCaptureResources();
        }

        private void NormalizeConfig()
        {
            imageWidth = Mathf.Clamp(imageWidth, 16, 1280);
            imageHeight = Mathf.Clamp(imageHeight, 16, 720);
            frameRateHz = Mathf.Clamp(Mathf.Round(frameRateHz), 1f, 15f);
            verticalFovDegrees = Mathf.Clamp(verticalFovDegrees, 20f, 120f);

            frameChunkBytes = Mathf.Clamp(frameChunkBytes, 512, 36000);
            maxFrameBytes = Mathf.Clamp(maxFrameBytes, 1024, 16777216);
            maxFrameChunks = Mathf.Clamp(maxFrameChunks, 1, 1024);
            nearClipMeters = Mathf.Clamp(nearClipMeters, 0.01f, 10f);
            farClipMeters = Mathf.Max(nearClipMeters + 1f, farClipMeters);
            imageSizeStatus = imageWidth.ToString(CultureInfo.InvariantCulture) + " x " + imageHeight.ToString(CultureInfo.InvariantCulture);
        }

        private string ResolveSensorId()
        {
            return ModulePyLoNSensorId.Resolve(
                part,
                "rgb_camera");
        }

        private void EnsureUdpClient()
        {
            var host = string.IsNullOrEmpty(udpHost) ? "127.0.0.1" : udpHost;
            var key = host + ":" + udpPort.ToString(CultureInfo.InvariantCulture);
            if (udpClient != null && udpEndPoint != null && endpointKey == key) return;
            CloseUdpClient();
            IPAddress address;
            if (!IPAddress.TryParse(host, out address))
            {
                var addresses = Dns.GetHostAddresses(host);
                if (addresses.Length == 0) throw new InvalidOperationException("Could not resolve UDP host '" + host + "'.");
                address = addresses[0];
            }
            udpClient = new UdpClient();
            udpEndPoint = new IPEndPoint(address, udpPort);
            endpointKey = key;
        }

        private void CloseUdpClient()
        {
            if (udpClient != null) udpClient.Close();
            udpClient = null;
            udpEndPoint = null;
            endpointKey = null;
        }

        private void WarnThrottled(string message)
        {
            var now = Time.realtimeSinceStartup;
            if (now - lastWarningTime < 5.0) return;
            lastWarningTime = now;
            Debug.LogWarning("[PyLoN] " + message);
        }

        private static Vector3 AxisToWorld(Transform target, string axis)
        {
            switch ((axis ?? "").Trim().ToUpperInvariant())
            {
                case "+X": case "X": return target.right.normalized;
                case "-X": return (-target.right).normalized;
                case "+Y": case "Y": return target.up.normalized;
                case "-Y": return (-target.up).normalized;
                case "-Z": return (-target.forward).normalized;
                default: return target.forward.normalized;
            }
        }

        private static void Orthonormalize(ref Vector3 forward, ref Vector3 up, Transform fallback)
        {
            if (forward.sqrMagnitude < 0.0001f) forward = fallback.forward;
            if (up.sqrMagnitude < 0.0001f || Mathf.Abs(Vector3.Dot(forward.normalized, up.normalized)) > 0.99f) up = fallback.up;
            if (Mathf.Abs(Vector3.Dot(forward.normalized, up.normalized)) > 0.99f) up = fallback.right;
            forward.Normalize();
            up = (up - Vector3.Project(up, forward)).normalized;
        }

        private static Vector3 ParseVector3(string value, Vector3 fallback)
        {
            var pieces = (value ?? "").Split(',');
            if (pieces.Length != 3) return fallback;
            float x, y, z;
            if (!float.TryParse(pieces[0], NumberStyles.Float, CultureInfo.InvariantCulture, out x)
                || !float.TryParse(pieces[1], NumberStyles.Float, CultureInfo.InvariantCulture, out y)
                || !float.TryParse(pieces[2], NumberStyles.Float, CultureInfo.InvariantCulture, out z)) return fallback;
            return new Vector3(x, y, z);
        }





        private static string Sha256Hex(byte[] payload)
        {
            using (var sha = SHA256.Create())
            {
                var hash = sha.ComputeHash(payload);
                var builder = new StringBuilder(hash.Length * 2);
                foreach (var value in hash) builder.Append(value.ToString("x2", CultureInfo.InvariantCulture));
                return builder.ToString();
            }
        }

        private static void Prefix(StringBuilder builder, string name, bool first)
        {
            if (!first) builder.Append(',');
            JsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendString(StringBuilder builder, string name, string value, bool first)
        {
            Prefix(builder, name, first);
            JsonString(builder, value);
        }

        private static void AppendNumber(StringBuilder builder, string name, long value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendDouble(StringBuilder builder, string name, double value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendFloat(StringBuilder builder, string name, float value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendVector(StringBuilder builder, string name, Vector3 value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append('[');
            builder.Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',');
            builder.Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',');
            builder.Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }

        private static void AppendQuaternion(StringBuilder builder, string name, Quaternion value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append('[');
            builder.Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',');
            builder.Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',');
            builder.Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(',');
            builder.Append(value.w.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }

        private static void JsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            foreach (var character in value ?? "")
            {
                switch (character)
                {
                    case '"': builder.Append("\\\""); break;
                    case '\\': builder.Append("\\\\"); break;
                    case '\n': builder.Append("\\n"); break;
                    case '\r': builder.Append("\\r"); break;
                    case '\t': builder.Append("\\t"); break;
                    default:
                        if (character < 32) builder.Append("\\u").Append(((int)character).ToString("x4", CultureInfo.InvariantCulture));
                        else builder.Append(character);
                        break;
                }
            }
            builder.Append('"');
        }
    }
}
