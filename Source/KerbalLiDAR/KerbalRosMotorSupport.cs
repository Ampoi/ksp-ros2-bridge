using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    [Serializable]
    public sealed class KerbalRosMotorCommand
    {
        public string type;
        public int version;
        public string name;
        public long partFlightId;
        public string mode;
        public bool hasPosition;
        public double position;
        public bool hasVelocity;
        public double velocity;
        public bool hasEffort;
        public double effort;
        public long sequence;
    }

    public interface IKerbalRosMotor
    {
        Part MotorPart { get; }
        string JointName { get; }
        string JointType { get; }
        int CommandUdpPort { get; }
        string StateUdpHost { get; }
        int StateUdpPort { get; }
        float StateRateHz { get; }
        void ApplyRosCommand(KerbalRosMotorCommand command);
        string BuildStatePacket();
    }

    internal static class KerbalRosMotorNames
    {
        public static string Resolve(string configuredName, string fallbackPrefix, Part part)
        {
            var candidate = string.IsNullOrEmpty(configuredName)
                ? fallbackPrefix + "_" + (part == null ? "0" : part.flightID.ToString(CultureInfo.InvariantCulture))
                : configuredName;
            return Sanitize(candidate, fallbackPrefix);
        }

        public static string Sanitize(string value, string fallback)
        {
            var builder = new StringBuilder();
            var previousUnderscore = false;
            var source = (value ?? string.Empty).Trim().ToLowerInvariant();
            for (var index = 0; index < source.Length; index++)
            {
                var character = source[index];
                var accepted = character >= 'a' && character <= 'z' ||
                               character >= '0' && character <= '9' ||
                               character == '_';
                if (!accepted)
                {
                    character = '_';
                }

                if (character == '_')
                {
                    if (previousUnderscore || builder.Length == 0)
                    {
                        previousUnderscore = true;
                        continue;
                    }

                    previousUnderscore = true;
                }
                else
                {
                    previousUnderscore = false;
                }

                builder.Append(character);
            }

            while (builder.Length > 0 && builder[builder.Length - 1] == '_')
            {
                builder.Length--;
            }

            if (builder.Length == 0)
            {
                builder.Append(fallback);
            }

            if (builder[0] >= '0' && builder[0] <= '9')
            {
                builder.Insert(0, '_');
            }

            return builder.ToString();
        }
    }

    internal static class KerbalRosMotorVisuals
    {
        public static Transform EnsureTransform(Part part, string transformName)
        {
            var existing = part == null ? null : part.FindModelTransform(transformName);
            if (existing != null)
            {
                return existing;
            }

            var gameObject = new GameObject(transformName);
            var modelRoot = part.transform.Find("model");
            gameObject.transform.SetParent(modelRoot == null ? part.transform : modelRoot, false);
            gameObject.layer = part.gameObject.layer;
            return gameObject.transform;
        }

        public static void AddStockModel(
            Part part,
            Transform parent,
            string modelPath,
            string position,
            string rotation,
            string scale)
        {
            if (part == null || parent == null || string.IsNullOrEmpty(modelPath))
            {
                return;
            }

            try
            {
                var model = GameDatabase.Instance.GetModel(modelPath.Trim());
                if (model == null)
                {
                    Debug.LogWarning("[KerbalLiDAR] Motor visual model was not found: " + modelPath);
                    return;
                }

                model.name = "motor_visual_" + parent.childCount.ToString(CultureInfo.InvariantCulture);
                model.transform.SetParent(parent, false);
                model.transform.localPosition = ParseVector3(position, Vector3.zero);
                model.transform.localEulerAngles = ParseVector3(rotation, Vector3.zero);
                model.transform.localScale = ParseVector3(scale, Vector3.one);
                SetLayerRecursively(model.transform, part.gameObject.layer);

                var colliders = model.GetComponentsInChildren<Collider>(true);
                for (var index = 0; index < colliders.Length; index++)
                {
                    UnityEngine.Object.Destroy(colliders[index]);
                }
            }
            catch (Exception exception)
            {
                Debug.LogWarning("[KerbalLiDAR] Failed to load motor visual model '" + modelPath + "': " + exception.Message);
            }
        }

        public static GameObject AddIndicator(Part part, Transform parent, PrimitiveType primitiveType, Vector3 position, Vector3 scale, Color color)
        {
            var indicator = GameObject.CreatePrimitive(primitiveType);
            indicator.name = "motor_indicator";
            indicator.transform.SetParent(parent, false);
            indicator.transform.localPosition = position;
            indicator.transform.localRotation = Quaternion.identity;
            indicator.transform.localScale = scale;
            indicator.layer = part.gameObject.layer;

            var collider = indicator.GetComponent<Collider>();
            if (collider != null)
            {
                UnityEngine.Object.Destroy(collider);
            }

            var renderer = indicator.GetComponent<Renderer>();
            if (renderer != null)
            {
                var shader = Shader.Find("KSP/Specular");
                if (shader == null)
                {
                    shader = Shader.Find("Diffuse");
                }

                if (shader != null)
                {
                    renderer.material = new Material(shader);
                    renderer.material.color = color;
                }
            }

            return indicator;
        }

        private static Vector3 ParseVector3(string value, Vector3 fallback)
        {
            if (string.IsNullOrEmpty(value))
            {
                return fallback;
            }

            var values = value.Split(',');
            if (values.Length != 3)
            {
                return fallback;
            }

            float x;
            float y;
            float z;
            if (!float.TryParse(values[0].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out x) ||
                !float.TryParse(values[1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out y) ||
                !float.TryParse(values[2].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out z))
            {
                return fallback;
            }

            return new Vector3(x, y, z);
        }

        private static void SetLayerRecursively(Transform transform, int layer)
        {
            transform.gameObject.layer = layer;
            for (var index = 0; index < transform.childCount; index++)
            {
                SetLayerRecursively(transform.GetChild(index), layer);
            }
        }
    }

    internal static class KerbalRosMotorJson
    {
        public static string State(
            IKerbalRosMotor motor,
            double position,
            double velocity,
            double effort,
            double current,
            double target,
            bool powered,
            bool engaged,
            bool locked)
        {
            var part = motor.MotorPart;
            var vesselName = part != null && part.vessel != null ? part.vessel.vesselName : string.Empty;
            var partFlightId = part == null ? 0u : part.flightID;
            var universalTime = Planetarium.fetch == null ? 0.0 : Planetarium.GetUniversalTime();
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "ksp_motor_state", true);
            AppendNumber(builder, "version", 1, false);
            AppendString(builder, "name", motor.JointName, false);
            AppendString(builder, "jointType", motor.JointType, false);
            AppendString(builder, "vessel", vesselName, false);
            AppendNumber(builder, "partFlightId", partFlightId, false);
            AppendNumber(builder, "universalTime", universalTime, false);
            AppendNumber(builder, "position", position, false);
            AppendNumber(builder, "velocity", velocity, false);
            AppendNumber(builder, "effort", effort, false);
            AppendNumber(builder, "current", current, false);
            AppendNumber(builder, "target", target, false);
            AppendBoolean(builder, "powered", powered, false);
            AppendBoolean(builder, "engaged", engaged, false);
            AppendBoolean(builder, "locked", locked, false);
            builder.Append('}');
            return builder.ToString();
        }

        private static void AppendPrefix(StringBuilder builder, string name, bool first)
        {
            if (!first)
            {
                builder.Append(',');
            }

            AppendJsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendString(StringBuilder builder, string name, string value, bool first)
        {
            AppendPrefix(builder, name, first);
            AppendJsonString(builder, value);
        }

        private static void AppendNumber(StringBuilder builder, string name, double value, bool first)
        {
            AppendPrefix(builder, name, first);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendBoolean(StringBuilder builder, string name, bool value, bool first)
        {
            AppendPrefix(builder, name, first);
            builder.Append(value ? "true" : "false");
        }

        private static void AppendJsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            var source = value ?? string.Empty;
            for (var index = 0; index < source.Length; index++)
            {
                var character = source[index];
                switch (character)
                {
                    case '\\':
                        builder.Append("\\\\");
                        break;
                    case '"':
                        builder.Append("\\\"");
                        break;
                    case '\n':
                        builder.Append("\\n");
                        break;
                    case '\r':
                        builder.Append("\\r");
                        break;
                    case '\t':
                        builder.Append("\\t");
                        break;
                    default:
                        builder.Append(character);
                        break;
                }
            }

            builder.Append('"');
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class KerbalRosMotorUdpManager : MonoBehaviour
    {
        private static readonly List<IKerbalRosMotor> Motors = new List<IKerbalRosMotor>();
        private readonly Dictionary<int, UdpClient> commandClients = new Dictionary<int, UdpClient>();
        private readonly Dictionary<IKerbalRosMotor, float> nextStateTimes = new Dictionary<IKerbalRosMotor, float>();
        private readonly Dictionary<string, IPEndPoint> stateEndpoints = new Dictionary<string, IPEndPoint>();
        private UdpClient stateClient;
        private float lastWarningTime = -1000f;

        internal static void Register(IKerbalRosMotor motor)
        {
            if (motor != null && !Motors.Contains(motor))
            {
                Motors.Add(motor);
            }
        }

        internal static void Unregister(IKerbalRosMotor motor)
        {
            Motors.Remove(motor);
        }

        public void Start()
        {
            stateClient = new UdpClient();
        }

        public void Update()
        {
            EnsureCommandClients();
            ReceiveCommands();
            SendStates();
        }

        public void OnDestroy()
        {
            foreach (var client in commandClients.Values)
            {
                client.Close();
            }

            commandClients.Clear();
            stateEndpoints.Clear();
            nextStateTimes.Clear();
            if (stateClient != null)
            {
                stateClient.Close();
                stateClient = null;
            }

            Motors.Clear();
        }

        private void EnsureCommandClients()
        {
            for (var index = Motors.Count - 1; index >= 0; index--)
            {
                var motor = Motors[index];
                if (motor == null || motor.MotorPart == null)
                {
                    Motors.RemoveAt(index);
                    continue;
                }

                var port = motor.CommandUdpPort;
                if (port <= 0 || port > 65535 || commandClients.ContainsKey(port))
                {
                    continue;
                }

                try
                {
                    var client = new UdpClient(new IPEndPoint(IPAddress.Any, port));
                    client.Client.Blocking = false;
                    commandClients.Add(port, client);
                    Debug.Log("[KerbalLiDAR] ROS motor commands listening on udp://0.0.0.0:" + port.ToString(CultureInfo.InvariantCulture));
                }
                catch (Exception exception)
                {
                    Warn("Could not bind ROS motor command UDP port " + port.ToString(CultureInfo.InvariantCulture) + ": " + exception.Message);
                }
            }
        }

        private void ReceiveCommands()
        {
            foreach (var pair in commandClients)
            {
                while (true)
                {
                    try
                    {
                        IPEndPoint sender = null;
                        var bytes = pair.Value.Receive(ref sender);
                        var json = Encoding.UTF8.GetString(bytes);
                        var command = JsonUtility.FromJson<KerbalRosMotorCommand>(json);
                        if (command == null || command.type != "ksp_motor_command" || command.version != 1)
                        {
                            continue;
                        }

                        Dispatch(command, pair.Key);
                    }
                    catch (SocketException exception)
                    {
                        if (exception.SocketErrorCode != SocketError.WouldBlock &&
                            exception.SocketErrorCode != SocketError.TryAgain)
                        {
                            Warn("ROS motor command UDP receive failed: " + exception.Message);
                        }

                        break;
                    }
                    catch (Exception exception)
                    {
                        Warn("Dropped invalid ROS motor command: " + exception.Message);
                        break;
                    }
                }
            }
        }

        private static void Dispatch(KerbalRosMotorCommand command, int port)
        {
            var sanitizedName = string.IsNullOrEmpty(command.name)
                ? string.Empty
                : KerbalRosMotorNames.Sanitize(command.name, "motor");
            for (var index = 0; index < Motors.Count; index++)
            {
                var motor = Motors[index];
                var part = motor == null ? null : motor.MotorPart;
                if (motor == null || part == null || motor.CommandUdpPort != port)
                {
                    continue;
                }

                var idMatches = command.partFlightId > 0 && command.partFlightId == part.flightID;
                var nameMatches = sanitizedName.Length > 0 && sanitizedName == motor.JointName;
                if (idMatches || nameMatches)
                {
                    motor.ApplyRosCommand(command);
                }
            }
        }

        private void SendStates()
        {
            if (stateClient == null)
            {
                return;
            }

            var now = Time.realtimeSinceStartup;
            for (var index = 0; index < Motors.Count; index++)
            {
                var motor = Motors[index];
                if (motor == null || motor.MotorPart == null || motor.StateRateHz <= 0f)
                {
                    continue;
                }

                float nextTime;
                if (nextStateTimes.TryGetValue(motor, out nextTime) && now < nextTime)
                {
                    continue;
                }

                nextStateTimes[motor] = now + 1f / Mathf.Clamp(motor.StateRateHz, 1f, 60f);
                try
                {
                    var endpoint = ResolveEndpoint(motor.StateUdpHost, motor.StateUdpPort);
                    var bytes = Encoding.UTF8.GetBytes(motor.BuildStatePacket());
                    stateClient.Send(bytes, bytes.Length, endpoint);
                }
                catch (Exception exception)
                {
                    Warn("ROS motor state UDP send failed: " + exception.Message);
                }
            }
        }

        private IPEndPoint ResolveEndpoint(string host, int port)
        {
            var resolvedHost = string.IsNullOrEmpty(host) ? "127.0.0.1" : host.Trim();
            var resolvedPort = Mathf.Clamp(port, 1, 65535);
            var key = resolvedHost + ":" + resolvedPort.ToString(CultureInfo.InvariantCulture);
            IPEndPoint endpoint;
            if (stateEndpoints.TryGetValue(key, out endpoint))
            {
                return endpoint;
            }

            IPAddress address;
            if (!IPAddress.TryParse(resolvedHost, out address))
            {
                var addresses = Dns.GetHostAddresses(resolvedHost);
                if (addresses.Length == 0)
                {
                    throw new InvalidOperationException("Could not resolve UDP host '" + resolvedHost + "'.");
                }

                address = addresses[0];
            }

            endpoint = new IPEndPoint(address, resolvedPort);
            stateEndpoints.Add(key, endpoint);
            return endpoint;
        }

        private void Warn(string message)
        {
            var now = Time.realtimeSinceStartup;
            if (now - lastWarningTime < 5f)
            {
                return;
            }

            lastWarningTime = now;
            Debug.LogWarning("[KerbalLiDAR] " + message);
        }
    }
}
