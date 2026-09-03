using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public class ModuleKerbalRgbCamera : PartModule
    {
        private const int ProtocolVersion = 1;
        private const int MaxPartNameLength = 64;

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

        [KSPField(isPersistant = true)]
        public string partName = "";

        [KSPField]
        public int imageWidth = 320;

        [KSPField]
        public int imageHeight = 240;

        [KSPField]
        public string udpHost = "127.0.0.1";

        [KSPField]
        public int udpPort = 49010;

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
        private float nextNameValidationTime;
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
            AssignUniquePartName(partName, false);
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            NormalizeConfig();
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            if (HighLogic.LoadedSceneIsEditor)
            {
                MaintainUniquePartName();
                return;
            }

            if (!HighLogic.LoadedSceneIsFlight || !cameraEnabled || !udpEnabled)
            {
                return;
            }

            var now = Time.realtimeSinceStartup;
            if (now < nextFrameTime)
            {
                return;
            }

            nextFrameTime = now + 1f / Mathf.Max(1f, frameRateHz);
            CaptureAndSend();
        }

        [KSPEvent(guiActive = true, guiName = "Capture RGB Frame Now", active = true)]
        public void CaptureNow()
        {
            if (HighLogic.LoadedSceneIsFlight && udpEnabled)
            {
                CaptureAndSend();
            }
        }

        [KSPEvent(guiActive = false, guiActiveEditor = false, guiName = "Edit ROS2 Part Name", active = false)]
        public void EditPartName()
        {
            var pendingName = ResolvePartName();
            var dialogId = "kerbal_rgb_camera_part_name_" + (part != null ? part.craftID.ToString(CultureInfo.InvariantCulture) : "0");
            PopupDialog.DismissPopup(dialogId);
            var dialog = new MultiOptionDialog(
                dialogId,
                "Use letters, numbers, and underscores. The name is made unique within this craft.",
                "ROS2 RGB Camera Name",
                HighLogic.UISkin,
                new DialogGUIBase[]
                {
                    new DialogGUITextInput(
                        pendingName,
                        false,
                        MaxPartNameLength,
                        delegate(string value) { pendingName = value; return value; },
                        280f),
                    new DialogGUIButton(
                        "Save",
                        delegate
                        {
                            var assigned = AssignUniquePartName(pendingName, true);
                            ScreenMessages.PostScreenMessage("ROS2 camera name: " + assigned, 3f, ScreenMessageStyle.UPPER_CENTER);
                        },
                        true),
                    new DialogGUIButton("Cancel", delegate { }, true)
                });
            PopupDialog.SpawnPopupDialog(dialog, false, HighLogic.UISkin, true, string.Empty);
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
            SendInactive();
            DestroyCaptureResources();
            CloseUdpClient();
        }

        private void CaptureAndSend()
        {
            NormalizeConfig();
            var started = Time.realtimeSinceStartup;
            try
            {
                var root = part != null && part.transform != null ? part.transform : transform;
                var forward = AxisToWorld(root, forwardAxis);
                var up = AxisToWorld(root, upAxis);
                Orthonormalize(ref forward, ref up, root);
                var origin = root.TransformPoint(ParseVector3(cameraOriginLocalPosition, Vector3.zero));
                var rotation = Quaternion.LookRotation(forward, up);

                if (sceneCapture == null)
                {
                    sceneCapture = new KspSceneRgbCapture();
                }
                var rgb = sceneCapture.Capture(
                    imageWidth,
                    imageHeight,
                    origin,
                    rotation,
                    verticalFovDegrees,
                    nearClipMeters,
                    farClipMeters);
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
            var framePosition = UnityVectorToRos(partTransform.InverseTransformPoint(origin));
            var frameRotation = UnityRotationToRos(Quaternion.Inverse(partTransform.rotation) * rotation);
            var resolvedName = ResolvePartName();
            var universalTime = Planetarium.GetUniversalTime();
            for (var chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++)
            {
                var offset = chunkIndex * frameChunkBytes;
                var count = Math.Min(frameChunkBytes, rgb.Length - offset);
                var builder = new StringBuilder(count * 2);
                builder.Append('{');
                AppendString(builder, "type", "ksp_camera_frame_chunk", true);
                AppendNumber(builder, "version", ProtocolVersion, false);
                AppendString(builder, "source", "rgb_camera", false);
                AppendString(builder, "sessionId", sessionId, false);
                AppendNumber(builder, "sequence", frameSequence, false);
                AppendNumber(builder, "chunkIndex", chunkIndex, false);
                AppendNumber(builder, "chunkCount", chunkCount, false);
                AppendNumber(builder, "frameBytes", rgb.Length, false);
                AppendString(builder, "sha256", checksum, false);
                AppendString(builder, "sensorId", resolvedName, false);
                AppendString(builder, "partName", resolvedName, false);
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
                var datagram = Encoding.UTF8.GetBytes(builder.ToString());
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
                AppendString(builder, "type", "ksp_camera_inactive", true);
                AppendNumber(builder, "version", ProtocolVersion, false);
                AppendString(builder, "source", "rgb_camera", false);
                AppendString(builder, "sensorId", ResolvePartName(), false);
                AppendString(builder, "partName", ResolvePartName(), false);
                AppendNumber(builder, "partFlightId", part != null ? (long)part.flightID : 0L, false);
                builder.Append('}');
                var datagram = Encoding.UTF8.GetBytes(builder.ToString());
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
            udpPort = Mathf.Clamp(udpPort, 1, 65535);
            frameChunkBytes = Mathf.Clamp(frameChunkBytes, 512, 36000);
            maxFrameBytes = Mathf.Clamp(maxFrameBytes, 1024, 16777216);
            maxFrameChunks = Mathf.Clamp(maxFrameChunks, 1, 1024);
            nearClipMeters = Mathf.Clamp(nearClipMeters, 0.01f, 10f);
            farClipMeters = Mathf.Max(nearClipMeters + 1f, farClipMeters);
            partName = string.IsNullOrEmpty(partName) ? "" : partName.Trim();
            imageSizeStatus = imageWidth.ToString(CultureInfo.InvariantCulture) + " x " + imageHeight.ToString(CultureInfo.InvariantCulture);
        }

        private void MaintainUniquePartName()
        {
            if (Time.realtimeSinceStartup < nextNameValidationTime)
            {
                return;
            }
            nextNameValidationTime = Time.realtimeSinceStartup + 0.5f;
            AssignUniquePartName(partName, true);
        }

        private string AssignUniquePartName(string requestedName, bool markEditorModified)
        {
            var baseName = NormalizeName(requestedName);
            if (string.IsNullOrEmpty(baseName))
            {
                baseName = "rgb_camera";
            }
            var used = CollectOtherSensorNames();
            var unique = baseName;
            var suffix = 2;
            while (used.Contains(unique))
            {
                var suffixText = "_" + suffix.ToString(CultureInfo.InvariantCulture);
                var length = Math.Min(baseName.Length, MaxPartNameLength - suffixText.Length);
                unique = baseName.Substring(0, length).TrimEnd('_') + suffixText;
                suffix++;
            }
            var changed = !string.Equals(partName, unique, StringComparison.Ordinal);
            partName = unique;
            if (changed && markEditorModified && HighLogic.LoadedSceneIsEditor
                && EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
            }
            return unique;
        }

        private HashSet<string> CollectOtherSensorNames()
        {
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            IList<Part> parts = null;
            if (HighLogic.LoadedSceneIsEditor && EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                parts = EditorLogic.fetch.ship.parts;
            }
            else if (vessel != null)
            {
                parts = vessel.parts;
            }
            if (parts == null)
            {
                return names;
            }
            foreach (var candidatePart in parts)
            {
                if (candidatePart == null || candidatePart == part)
                {
                    continue;
                }
                foreach (PartModule candidateModule in candidatePart.Modules)
                {
                    var camera = candidateModule as ModuleKerbalRgbCamera;
                    if (camera != null)
                    {
                        var cameraName = NormalizeName(camera.partName);
                        if (!string.IsNullOrEmpty(cameraName)) names.Add(cameraName);
                    }
                    var lidar = candidateModule as ModuleKerbalLidar;
                    if (lidar != null)
                    {
                        var lidarName = NormalizeName(!string.IsNullOrEmpty(lidar.partName) ? lidar.partName : lidar.lidarName);
                        if (!string.IsNullOrEmpty(lidarName)) names.Add(lidarName);
                    }
                }
            }
            return names;
        }

        private string ResolvePartName()
        {
            return ModuleKerbalRosSensorId.Resolve(
                part,
                string.IsNullOrEmpty(partName) ? "rgb_camera" : partName);
        }

        private static string NormalizeName(string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return string.Empty;
            var builder = new StringBuilder(Math.Min(value.Length, MaxPartNameLength));
            var underscore = false;
            for (var index = 0; index < value.Length && builder.Length < MaxPartNameLength; index++)
            {
                var character = value[index];
                var letter = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z');
                var digit = character >= '0' && character <= '9';
                if (letter || digit)
                {
                    builder.Append(char.ToLowerInvariant(character));
                    underscore = false;
                }
                else if (!underscore && builder.Length > 0)
                {
                    builder.Append('_');
                    underscore = true;
                }
            }
            var normalized = builder.ToString().Trim('_');
            if (!string.IsNullOrEmpty(normalized) && char.IsDigit(normalized[0]))
            {
                normalized = "_" + normalized;
            }
            return normalized.Length <= MaxPartNameLength ? normalized : normalized.Substring(0, MaxPartNameLength);
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
            var now = Planetarium.GetUniversalTime();
            if (now - lastWarningTime < 5.0) return;
            lastWarningTime = now;
            Debug.LogWarning("[KerbalLiDAR] " + message);
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

        private static Vector3 UnityVectorToRos(Vector3 value)
        {
            return new Vector3(value.z, -value.x, value.y);
        }

        private static Quaternion UnityRotationToRos(Quaternion value)
        {
            return new Quaternion(-value.z, value.x, -value.y, value.w).normalized;
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
