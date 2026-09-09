using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace PyLoN
{
    [Serializable]
    public sealed class PyLoNMotorCommand
    {
        public string type;
        public int version;
        public string name;
        public string vesselId;
        public string controllerId;
        public string leaseId;
        public long partFlightId;
        public string mode;
        public bool hasEnabled;
        public bool enabled;
        public bool hasPosition;
        public double position;
        public bool hasVelocity;
        public double velocity;
        public bool hasEffort;
        public double effort;
        public double timeoutSeconds;
        public long sequence;
    }

    public interface IPyLoNMotor
    {
        Part MotorPart { get; }
        string JointName { get; }
        string JointType { get; }
        float StateRateHz { get; }
        void ApplyRosCommand(PyLoNMotorCommand command);
        string BuildStatePacket();
    }

    internal static class PyLoNMotorNames
    {
        public static string Resolve(string configuredName, string fallbackPrefix, Part part, PartModule module = null)
        {
            var candidate = string.IsNullOrEmpty(configuredName)
                ? fallbackPrefix + "_" + (part == null
                    ? "0"
                    : (part.persistentId != 0u ? part.persistentId : part.flightID).ToString(CultureInfo.InvariantCulture)) +
                  "_" + PyLoNActuatorNames.ModuleIndex(part, module).ToString(CultureInfo.InvariantCulture)
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

    internal static class PyLoNMotorCollisions
    {
        public static void EnableBetweenConnectedParts(Part motorPart, Part drivenPart, ConfigurableJoint joint)
        {
            if (motorPart == null || drivenPart == null || joint == null)
            {
                return;
            }

            // KSP ignores a same-vessel collider pair unless both owning parts
            // opt in. Unity also suppresses contact between the two rigidbodies
            // connected by a Joint unless enableCollision is set.
            joint.enableCollision = true;
            motorPart.sameVesselCollision = true;
            drivenPart.sameVesselCollision = true;

            // ModuleJointMotor initializes after KSP has built its collider
            // ignore table, so request a refresh for already-loaded vessels.
            motorPart.ResetCollisionIgnores();
        }
    }

    internal static class PyLoNMotorVisuals
    {
        public static Transform EnsureTransform(Part part, string transformName)
        {
            if (part == null || part.transform == null || string.IsNullOrEmpty(transformName))
            {
                return null;
            }

            var existing = part.FindModelTransform(transformName);
            if (existing != null)
            {
                return existing;
            }

            var gameObject = new GameObject(transformName);
            var modelRoot = part.transform.Find("model");
            gameObject.transform.SetParent(modelRoot == null ? part.transform : modelRoot, false);
            gameObject.transform.localPosition = Vector3.zero;
            gameObject.transform.localRotation = Quaternion.identity;
            gameObject.transform.localScale = Vector3.one;
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

            for (var index = 0; index < parent.childCount; index++)
            {
                if (parent.GetChild(index).name.StartsWith("motor_visual", StringComparison.Ordinal))
                {
                    return;
                }
            }

            try
            {
                var model = GameDatabase.Instance.GetModel(modelPath.Trim());
                if (model == null)
                {
                    Debug.LogWarning("[PyLoN] Motor visual model was not found: " + modelPath);
                    return;
                }

                model.name = "motor_visual";
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
                Debug.LogWarning("[PyLoN] Failed to load motor visual model '" + modelPath + "': " + exception.Message);
            }
        }

        public static GameObject AddIndicator(Part part, Transform parent, PrimitiveType primitiveType, Vector3 position, Vector3 scale, Color color)
        {
            if (part == null || parent == null)
            {
                return null;
            }

            var existing = parent.Find("motor_indicator");
            if (existing != null)
            {
                existing.localPosition = position;
                existing.localRotation = Quaternion.identity;
                existing.localScale = scale;
                existing.gameObject.layer = part.gameObject.layer;
                return existing.gameObject;
            }

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

    internal static class PyLoNMotorJson
    {
        public static string State(
            IPyLoNMotor motor,
            double position,
            double velocity,
            double effort,
            double current,
            double target,
            bool powered,
            bool engaged,
            bool locked,
            string commandMode,
            bool commandActive)
        {
            var part = motor.MotorPart;
            var vesselName = part != null && part.vessel != null ? part.vessel.vesselName : string.Empty;
            var partFlightId = part == null ? 0u : part.flightID;
            var universalTime = Planetarium.fetch == null ? 0.0 : Planetarium.GetUniversalTime();
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "pylon_motor_state", true);
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
            AppendString(builder, "commandMode", commandMode, false);
            AppendBoolean(builder, "commandActive", commandActive, false);
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
    public sealed class PyLoNMotorUdpManager : MonoBehaviour
    {
        private static readonly List<IPyLoNMotor> Motors = new List<IPyLoNMotor>();
        private CommandReceiver commands;
        private readonly Dictionary<IPyLoNMotor, float> nextStateTimes = new Dictionary<IPyLoNMotor, float>();
        private readonly Dictionary<string, IPEndPoint> stateEndpoints = new Dictionary<string, IPEndPoint>();
        private UdpClient stateClient;
        private float lastWarningTime = -1000f;

        internal static void Register(IPyLoNMotor motor)
        {
            if (motor != null && !Motors.Contains(motor))
            {
                Motors.Add(motor);
            }
        }

        internal static void Unregister(IPyLoNMotor motor)
        {
            Motors.Remove(motor);
        }

        public void Start()
        {
            stateClient = new UdpClient();
            commands = new CommandReceiver(Dispatch, Warn);
        }

        public void Update()
        {
            UpdateManagedMotors();
            commands.Pump();
            SendStates();
        }

        private static void UpdateManagedMotors()
        {
            var elapsed = Mathf.Max(Time.deltaTime, 0.0001f);
            for (var index = Motors.Count - 1; index >= 0; index--)
            {
                var servo = Motors[index] as ModulePyLoNServo;
                if (servo != null)
                {
                    servo.ManagedUpdate(elapsed);
                    continue;
                }

                var linearMotor = Motors[index] as ModulePyLoNLinearMotor;
                if (linearMotor != null)
                {
                    linearMotor.ManagedUpdate(elapsed);
                }
            }
        }

        public void OnDestroy()
        {
            if (commands != null) commands.Dispose();
            stateEndpoints.Clear();
            nextStateTimes.Clear();
            if (stateClient != null)
            {
                stateClient.Close();
                stateClient = null;
            }

            Motors.Clear();
        }

        private static void Dispatch(PyLoNMotorCommand command)
        {
            if (command.version == 1)
            {
                string rejectionReason;
                if (!PyLoNVehicleManager.TryAcceptExclusiveCommand(
                    command.vesselId, command.controllerId, command.leaseId,
                    command.sequence, out rejectionReason))
                {
                    return;
                }
            }
            var sanitizedName = string.IsNullOrEmpty(command.name)
                ? string.Empty
                : PyLoNMotorNames.Sanitize(command.name, "motor");
            for (var index = 0; index < Motors.Count; index++)
            {
                var motor = Motors[index];
                var part = motor == null ? null : motor.MotorPart;
                if (motor == null || part == null || part.vessel != FlightGlobals.ActiveVessel)
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
                if (motor == null || motor.MotorPart == null || motor.MotorPart.vessel != FlightGlobals.ActiveVessel || motor.StateRateHz <= 0f)
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
                    var endpoint = ResolveEndpoint(RuntimeSettings.StateHost, RuntimeSettings.StatePort);
                    var bytes = TelemetryPacketCodec.Encode(motor.BuildStatePacket());
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
            Debug.LogWarning("[PyLoN] " + message);
        }
    }
}
