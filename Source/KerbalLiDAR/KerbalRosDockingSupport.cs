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
    [Serializable]
    public sealed class KerbalRosDockingCommand
    {
        public string type;
        public int version;
        public string name;
        public int action;
        public long sequence;
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class KerbalRosDockingManager : MonoBehaviour
    {
        private const int ProtocolVersion = 1;
        private const int CommandPort = 49011;
        private const int StatePort = 49010;
        private const float StateRateHz = 10f;
        private const float CameraRateHz = 5f;
        private const int ImageWidth = 320;
        private const int ImageHeight = 240;
        private const float VerticalFovDegrees = 60f;
        private const int FrameChunkBytes = 12000;
        private const int ActionSelectCamera = 1;
        private const int ActionStopCamera = 2;
        private const int ActionRelease = 3;

        private sealed class PortEntry
        {
            public ModuleDockingNode Node;
            public Part Part;
            public int ModuleIndex;
            public string Name;
        }

        private static KerbalRosDockingManager instance;
        private readonly List<PortEntry> ports = new List<PortEntry>();
        private readonly Dictionary<string, long> lastSequences = new Dictionary<string, long>();
        private readonly string cameraSessionId = Guid.NewGuid().ToString("N");
        private UdpClient stateClient;
        private IPEndPoint stateEndpoint;
        private Vessel attachedVessel;
        private string selectedCameraName = "";
        private float nextStateTime;
        private float nextManifestTime;
        private float nextCameraTime;
        private long frameSequence;
        private bool cameraTopicActive;
        private bool cameraStoppedExplicitly;
        private Camera captureCamera;
        private GameObject captureCameraObject;
        private RenderTexture renderTexture;
        private Texture2D readbackTexture;
        private float lastWarningTime = -1000f;

        public void Start()
        {
            instance = this;
            stateClient = new UdpClient();
            stateEndpoint = new IPEndPoint(IPAddress.Loopback, StatePort);
            AttachToActiveVessel();
        }

        public void Update()
        {
            AttachToActiveVessel();
            if (attachedVessel == null)
            {
                return;
            }

            var now = Time.realtimeSinceStartup;
            if (now >= nextManifestTime)
            {
                RefreshPorts(false);
                SendManifest();
                nextManifestTime = now + 1f;
            }
            if (now >= nextStateTime)
            {
                SendStates();
                nextStateTime = now + 1f / StateRateHz;
            }
            if (!string.IsNullOrEmpty(selectedCameraName) && now >= nextCameraTime)
            {
                CaptureAndSendSelectedCamera();
                nextCameraTime = now + 1f / CameraRateHz;
            }
        }

        public void OnDestroy()
        {
            SendCameraInactive(selectedCameraName, FindPort(selectedCameraName));
            SendEmptyManifest();
            DestroyCaptureResources();
            if (stateClient != null)
            {
                stateClient.Close();
                stateClient = null;
            }
            if (instance == this)
            {
                instance = null;
            }
        }

        internal static bool TryDispatch(string json, int port)
        {
            if (string.IsNullOrEmpty(json))
            {
                return false;
            }
            KerbalRosCommandEnvelope envelope;
            try
            {
                envelope = JsonUtility.FromJson<KerbalRosCommandEnvelope>(json);
            }
            catch
            {
                return false;
            }
            if (envelope == null || envelope.type != "ksp_docking_port_command")
            {
                return false;
            }
            if (envelope.version != ProtocolVersion || port != CommandPort || instance == null)
            {
                return true;
            }
            try
            {
                instance.ApplyCommand(JsonUtility.FromJson<KerbalRosDockingCommand>(json));
            }
            catch (Exception exception)
            {
                instance.Warn("Dropped invalid docking command: " + exception.Message);
            }
            return true;
        }

        private void AttachToActiveVessel()
        {
            var active = FlightGlobals.ActiveVessel;
            if (active == attachedVessel)
            {
                return;
            }
            SendCameraInactive(selectedCameraName, FindPort(selectedCameraName));
            if (attachedVessel != null)
            {
                SendEmptyManifest();
            }
            attachedVessel = active;
            selectedCameraName = "";
            cameraStoppedExplicitly = false;
            lastSequences.Clear();
            ports.Clear();
            if (attachedVessel != null)
            {
                RefreshPorts(true);
                SendManifest();
            }
        }

        private void RefreshPorts(bool selectFirst)
        {
            var previousSelection = selectedCameraName;
            ports.Clear();
            if (attachedVessel != null && attachedVessel.parts != null)
            {
                foreach (var vesselPart in attachedVessel.parts)
                {
                    if (vesselPart == null)
                    {
                        continue;
                    }
                    for (var moduleIndex = 0; moduleIndex < vesselPart.Modules.Count; moduleIndex++)
                    {
                        var node = vesselPart.Modules[moduleIndex] as ModuleDockingNode;
                        if (node == null)
                        {
                            continue;
                        }
                        ports.Add(new PortEntry
                        {
                            Node = node,
                            Part = vesselPart,
                            ModuleIndex = moduleIndex,
                            Name = PortName(vesselPart, moduleIndex)
                        });
                    }
                }
            }
            ports.Sort(delegate(PortEntry left, PortEntry right)
            {
                var leftId = StablePartId(left.Part);
                var rightId = StablePartId(right.Part);
                var comparison = leftId.CompareTo(rightId);
                return comparison != 0 ? comparison : left.ModuleIndex.CompareTo(right.ModuleIndex);
            });

            if (!string.IsNullOrEmpty(previousSelection) && FindPort(previousSelection) == null)
            {
                SendCameraInactive(previousSelection, null);
                selectedCameraName = "";
                cameraStoppedExplicitly = false;
            }
            if ((selectFirst || (!cameraStoppedExplicitly && string.IsNullOrEmpty(selectedCameraName))) && ports.Count > 0)
            {
                selectedCameraName = ports[0].Name;
                nextCameraTime = 0f;
            }
        }

        private void ApplyCommand(KerbalRosDockingCommand command)
        {
            if (command == null || command.sequence <= 0)
            {
                return;
            }
            var name = KerbalRosMotorNames.Sanitize(command.name, "docking_port");
            var entry = FindPort(name);
            if (entry == null)
            {
                return;
            }
            long previous;
            if (lastSequences.TryGetValue(name, out previous) && command.sequence <= previous)
            {
                return;
            }
            lastSequences[name] = command.sequence;

            if (command.action == ActionSelectCamera)
            {
                if (selectedCameraName != name)
                {
                    SendCameraInactive(selectedCameraName, FindPort(selectedCameraName));
                    selectedCameraName = name;
                    cameraStoppedExplicitly = false;
                    nextCameraTime = 0f;
                }
                return;
            }
            if (command.action == ActionStopCamera)
            {
                if (selectedCameraName == name)
                {
                    SendCameraInactive(name, entry);
                    selectedCameraName = "";
                    cameraStoppedExplicitly = true;
                }
                return;
            }
            if (command.action == ActionRelease)
            {
                Release(entry.Node);
            }
        }

        private static void Release(ModuleDockingNode node)
        {
            if (node == null)
            {
                return;
            }
            var undock = node.Events["Undock"];
            if (undock != null && undock.active)
            {
                node.Undock();
                return;
            }
            var decouple = node.Events["Decouple"];
            if (decouple != null && decouple.active)
            {
                node.Decouple();
            }
        }

        private void SendStates()
        {
            foreach (var entry in ports)
            {
                try
                {
                    var nodeState = entry.Node != null ? entry.Node.state : "Unavailable";
                    var partner = entry.Node != null ? entry.Node.otherNode : null;
                    var partnerPart = partner != null ? partner.part : null;
                    var builder = new StringBuilder(512);
                    builder.Append('{');
                    AppendString(builder, "type", "ksp_docking_port_state", true);
                    AppendNumber(builder, "version", ProtocolVersion, false);
                    AppendString(builder, "name", entry.Name, false);
                    AppendNumber(builder, "partFlightId", (long)entry.Part.flightID, false);
                    AppendNumber(builder, "moduleIndex", entry.ModuleIndex, false);
                    AppendString(builder, "nodeType", entry.Node != null && !string.IsNullOrEmpty(entry.Node.nodeType) ? entry.Node.nodeType : "unknown", false);
                    AppendString(builder, "state", string.IsNullOrEmpty(nodeState) ? "Unknown" : nodeState, false);
                    AppendBoolean(builder, "docked", ContainsState(nodeState, "Docked"), false);
                    AppendBoolean(builder, "acquiring", ContainsState(nodeState, "Acquire"), false);
                    AppendBoolean(builder, "releasable", IsReleasable(entry.Node), false);
                    AppendBoolean(builder, "cameraActive", entry.Name == selectedCameraName, false);
                    AppendString(builder, "partnerName", partnerPart != null ? PortName(partnerPart, ModuleIndex(partnerPart, partner)) : "", false);
                    AppendNumber(builder, "partnerPartFlightId", partnerPart != null ? (long)partnerPart.flightID : 0L, false);
                    builder.Append('}');
                    Send(builder);
                }
                catch (Exception exception)
                {
                    Warn("Docking state send failed: " + exception.Message);
                }
            }
        }

        private void SendManifest()
        {
            var builder = new StringBuilder(ports.Count * 96 + 128);
            builder.Append('{');
            AppendString(builder, "type", "ksp_docking_port_manifest", true);
            AppendNumber(builder, "version", ProtocolVersion, false);
            Prefix(builder, "ports", false);
            builder.Append('[');
            for (var index = 0; index < ports.Count; index++)
            {
                if (index > 0) builder.Append(',');
                builder.Append('{');
                AppendString(builder, "name", ports[index].Name, true);
                AppendNumber(builder, "partFlightId", (long)ports[index].Part.flightID, false);
                AppendNumber(builder, "moduleIndex", ports[index].ModuleIndex, false);
                builder.Append('}');
            }
            builder.Append("]}");
            Send(builder);
        }

        private void SendEmptyManifest()
        {
            Send(new StringBuilder("{\"type\":\"ksp_docking_port_manifest\",\"version\":1,\"ports\":[]}"));
        }

        private void CaptureAndSendSelectedCamera()
        {
            var entry = FindPort(selectedCameraName);
            if (entry == null || entry.Node == null || entry.Part == null)
            {
                return;
            }
            try
            {
                EnsureCaptureResources();
                var sourceCamera = FlightCamera.fetch != null ? FlightCamera.fetch.mainCamera : Camera.main;
                if (sourceCamera == null)
                {
                    return;
                }
                var cameraTransform = entry.Node.nodeTransform;
                if (cameraTransform == null)
                {
                    cameraTransform = entry.Node.controlTransform;
                }
                if (cameraTransform == null)
                {
                    cameraTransform = entry.Part.transform;
                }
                var forward = cameraTransform.forward.normalized;
                var origin = cameraTransform.position + forward * 0.02f;
                var rotation = Quaternion.LookRotation(forward, cameraTransform.up);
                captureCamera.CopyFrom(sourceCamera);
                captureCamera.enabled = false;
                captureCamera.transform.position = origin;
                captureCamera.transform.rotation = rotation;
                captureCamera.fieldOfView = VerticalFovDegrees;
                captureCamera.aspect = (float)ImageWidth / ImageHeight;
                captureCamera.nearClipPlane = 0.05f;
                captureCamera.farClipPlane = 20000f;
                captureCamera.allowHDR = false;
                captureCamera.allowMSAA = false;
                captureCamera.targetTexture = renderTexture;
                captureCamera.Render();

                var previousActive = RenderTexture.active;
                try
                {
                    RenderTexture.active = renderTexture;
                    readbackTexture.ReadPixels(new Rect(0, 0, ImageWidth, ImageHeight), 0, 0, false);
                    readbackTexture.Apply(false, false);
                }
                finally
                {
                    RenderTexture.active = previousActive;
                }
                SendCameraFrame(entry, ReadTopDownRgb(), origin, rotation);
                cameraTopicActive = true;
            }
            catch (Exception exception)
            {
                Warn("Docking camera capture failed: " + exception.Message);
            }
        }

        private void SendCameraFrame(PortEntry entry, byte[] rgb, Vector3 origin, Quaternion rotation)
        {
            frameSequence++;
            var chunkCount = (rgb.Length + FrameChunkBytes - 1) / FrameChunkBytes;
            var checksum = Sha256Hex(rgb);
            var framePosition = UnityVectorToRos(entry.Part.transform.InverseTransformPoint(origin));
            var frameRotation = UnityRotationToRos(Quaternion.Inverse(entry.Part.transform.rotation) * rotation);
            for (var chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++)
            {
                var offset = chunkIndex * FrameChunkBytes;
                var count = Math.Min(FrameChunkBytes, rgb.Length - offset);
                var builder = new StringBuilder(count * 2);
                builder.Append('{');
                AppendString(builder, "type", "ksp_camera_frame_chunk", true);
                AppendNumber(builder, "version", ProtocolVersion, false);
                AppendString(builder, "source", "docking_port", false);
                AppendString(builder, "sessionId", cameraSessionId, false);
                AppendNumber(builder, "sequence", frameSequence, false);
                AppendNumber(builder, "chunkIndex", chunkIndex, false);
                AppendNumber(builder, "chunkCount", chunkCount, false);
                AppendNumber(builder, "frameBytes", rgb.Length, false);
                AppendString(builder, "sha256", checksum, false);
                AppendString(builder, "partName", entry.Name, false);
                AppendNumber(builder, "partFlightId", (long)entry.Part.flightID, false);
                AppendDouble(builder, "universalTime", Planetarium.GetUniversalTime(), false);
                AppendNumber(builder, "width", ImageWidth, false);
                AppendNumber(builder, "height", ImageHeight, false);
                AppendNumber(builder, "step", ImageWidth * 3, false);
                AppendString(builder, "encoding", "rgb8", false);
                AppendFloat(builder, "verticalFovDeg", VerticalFovDegrees, false);
                AppendString(builder, "coordinateFrame", "ros_sensor", false);
                AppendVector(builder, "framePosition", framePosition, false);
                AppendQuaternion(builder, "frameRotation", frameRotation, false);
                AppendString(builder, "data", Convert.ToBase64String(rgb, offset, count), false);
                builder.Append('}');
                Send(builder);
            }
        }

        private void SendCameraInactive(string name, PortEntry entry)
        {
            if (!cameraTopicActive || string.IsNullOrEmpty(name))
            {
                return;
            }
            var builder = new StringBuilder(224);
            builder.Append('{');
            AppendString(builder, "type", "ksp_camera_inactive", true);
            AppendNumber(builder, "version", ProtocolVersion, false);
            AppendString(builder, "source", "docking_port", false);
            AppendString(builder, "partName", name, false);
            AppendNumber(builder, "partFlightId", entry != null && entry.Part != null ? (long)entry.Part.flightID : 0L, false);
            builder.Append('}');
            Send(builder);
            cameraTopicActive = false;
        }

        private byte[] ReadTopDownRgb()
        {
            var pixels = readbackTexture.GetPixels32();
            var rgb = new byte[ImageWidth * ImageHeight * 3];
            var targetIndex = 0;
            for (var targetY = 0; targetY < ImageHeight; targetY++)
            {
                var sourceRow = (ImageHeight - 1 - targetY) * ImageWidth;
                for (var x = 0; x < ImageWidth; x++)
                {
                    var pixel = pixels[sourceRow + x];
                    rgb[targetIndex++] = pixel.r;
                    rgb[targetIndex++] = pixel.g;
                    rgb[targetIndex++] = pixel.b;
                }
            }
            return rgb;
        }

        private void EnsureCaptureResources()
        {
            if (captureCamera != null && renderTexture != null && readbackTexture != null)
            {
                return;
            }
            DestroyCaptureResources();
            captureCameraObject = new GameObject("Kerbal Docking RGB Capture Camera");
            captureCameraObject.hideFlags = HideFlags.HideAndDontSave;
            captureCamera = captureCameraObject.AddComponent<Camera>();
            captureCamera.enabled = false;
            renderTexture = new RenderTexture(ImageWidth, ImageHeight, 24, RenderTextureFormat.ARGB32);
            renderTexture.Create();
            readbackTexture = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);
        }

        private void DestroyCaptureResources()
        {
            if (captureCamera != null) captureCamera.targetTexture = null;
            if (renderTexture != null)
            {
                renderTexture.Release();
                Destroy(renderTexture);
                renderTexture = null;
            }
            if (readbackTexture != null)
            {
                Destroy(readbackTexture);
                readbackTexture = null;
            }
            if (captureCameraObject != null)
            {
                Destroy(captureCameraObject);
                captureCameraObject = null;
                captureCamera = null;
            }
        }

        private PortEntry FindPort(string name)
        {
            if (string.IsNullOrEmpty(name)) return null;
            for (var index = 0; index < ports.Count; index++)
            {
                if (ports[index].Name == name) return ports[index];
            }
            return null;
        }

        private static bool IsReleasable(ModuleDockingNode node)
        {
            if (node == null) return false;
            var undock = node.Events["Undock"];
            var decouple = node.Events["Decouple"];
            return undock != null && undock.active || decouple != null && decouple.active;
        }

        private static bool ContainsState(string state, string value)
        {
            return !string.IsNullOrEmpty(state) && state.IndexOf(value, StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static uint StablePartId(Part part)
        {
            return part == null ? 0u : part.persistentId != 0u ? part.persistentId : part.flightID;
        }

        private static string PortName(Part part, int moduleIndex)
        {
            return "docking_port_" + StablePartId(part).ToString(CultureInfo.InvariantCulture) + "_" + Math.Max(0, moduleIndex).ToString(CultureInfo.InvariantCulture);
        }

        private static int ModuleIndex(Part part, PartModule module)
        {
            if (part == null || module == null) return 0;
            for (var index = 0; index < part.Modules.Count; index++)
            {
                if (ReferenceEquals(part.Modules[index], module)) return index;
            }
            return 0;
        }

        private void Send(StringBuilder builder)
        {
            if (stateClient == null || stateEndpoint == null) return;
            try
            {
                var bytes = Encoding.UTF8.GetBytes(builder.ToString());
                stateClient.Send(bytes, bytes.Length, stateEndpoint);
            }
            catch (Exception exception)
            {
                Warn("Docking UDP send failed: " + exception.Message);
            }
        }

        private void Warn(string message)
        {
            if (Time.realtimeSinceStartup - lastWarningTime < 5f) return;
            lastWarningTime = Time.realtimeSinceStartup;
            Debug.LogWarning("[KerbalLiDAR] " + message);
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

        private static void AppendBoolean(StringBuilder builder, string name, bool value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append(value ? "true" : "false");
        }

        private static void AppendVector(StringBuilder builder, string name, Vector3 value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }

        private static void AppendQuaternion(StringBuilder builder, string name, Quaternion value, bool first)
        {
            Prefix(builder, name, first);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.w.ToString("R", CultureInfo.InvariantCulture)).Append(']');
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
