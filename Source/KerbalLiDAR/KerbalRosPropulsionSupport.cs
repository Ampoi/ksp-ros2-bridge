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
    public sealed class KerbalRosCommandEnvelope
    {
        public string type;
        public int version;
    }

    [Serializable]
    public sealed class KerbalRosPropulsionModuleCommand
    {
        public string name;
        public long partFlightId;
        public int moduleIndex;
        public string kind;
        public bool hasEnabled;
        public bool enabled;
        public bool hasThrottle;
        public double throttle;
        public bool release;
    }

    [Serializable]
    public sealed class KerbalRosPropulsionCommand
    {
        public string type;
        public int version;
        public KerbalRosPropulsionModuleCommand[] commands;
        public bool hasMainThrottle;
        public double mainThrottle;
        public bool hasRcsCommand;
        public double rcsX;
        public double rcsY;
        public double rcsZ;
        public double rcsPitch;
        public double rcsYaw;
        public double rcsRoll;
        public double timeoutSeconds;
        public long sequence;
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class KerbalRosPropulsionManager : MonoBehaviour
    {
        private const int CommandPort = 49011;
        private const int StatePort = 49010;
        private const float DefaultTimeoutSeconds = 0.5f;
        private const float StateRateHz = 10f;

        private sealed class EngineOverride
        {
            public bool IndependentThrottle;
            public float IndependentThrottlePercentage;
            public float ExpiresAt = float.PositiveInfinity;
            public bool TimedOut;
        }

        private sealed class RcsOverride
        {
            public bool Enabled;
            public float ThrustPercentage;
            public float ExpiresAt = float.PositiveInfinity;
            public bool TimedOut;
        }

        private static KerbalRosPropulsionManager instance;
        private readonly Dictionary<ModuleEngines, EngineOverride> engineOverrides =
            new Dictionary<ModuleEngines, EngineOverride>();
        private readonly Dictionary<ModuleRCS, RcsOverride> rcsOverrides =
            new Dictionary<ModuleRCS, RcsOverride>();
        private UdpClient stateClient;
        private IPEndPoint stateEndpoint;
        private Vessel attachedVessel;
        private float nextStateTime;
        private float mainThrottleExpiresAt;
        private float rcsCommandExpiresAt;
        private float commandedMainThrottle;
        private float commandedRcsX;
        private float commandedRcsY;
        private float commandedRcsZ;
        private float commandedRcsPitch;
        private float commandedRcsYaw;
        private float commandedRcsRoll;
        private bool mainThrottleActive;
        private bool rcsCommandActive;
        private bool rcsActionGroupOverridden;
        private bool rcsActionGroupWasActive;
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
            ApplyTimeouts();
            SendStates();
        }

        public void OnDestroy()
        {
            RestoreAllOverrides();
            DetachFromVessel();
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

            if (envelope == null || envelope.type != "ksp_propulsion_command")
            {
                return false;
            }

            if (envelope.version != 1 || port != CommandPort || instance == null)
            {
                return true;
            }

            try
            {
                var command = JsonUtility.FromJson<KerbalRosPropulsionCommand>(json);
                if (command != null)
                {
                    instance.ApplyCommand(command);
                }
            }
            catch (Exception exception)
            {
                instance.Warn("Dropped invalid ROS propulsion command: " + exception.Message);
            }

            return true;
        }

        private void AttachToActiveVessel()
        {
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == attachedVessel)
            {
                return;
            }

            RestoreAllOverrides();
            DetachFromVessel();
            attachedVessel = vessel;
            if (attachedVessel != null)
            {
                attachedVessel.OnFlyByWire += ApplyFlightControls;
            }
        }

        private void DetachFromVessel()
        {
            RestoreRcsActionGroup();
            if (attachedVessel != null)
            {
                attachedVessel.OnFlyByWire -= ApplyFlightControls;
                attachedVessel = null;
            }

            mainThrottleActive = false;
            rcsCommandActive = false;
        }

        private void ApplyFlightControls(FlightCtrlState state)
        {
            if (state == null)
            {
                return;
            }

            if (mainThrottleActive)
            {
                state.mainThrottle = commandedMainThrottle;
            }

            if (rcsCommandActive)
            {
                state.X = commandedRcsX;
                state.Y = commandedRcsY;
                state.Z = commandedRcsZ;
                state.pitch = commandedRcsPitch;
                state.yaw = commandedRcsYaw;
                state.roll = commandedRcsRoll;
            }
        }

        private void ApplyCommand(KerbalRosPropulsionCommand command)
        {
            AttachToActiveVessel();
            if (attachedVessel == null)
            {
                return;
            }

            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : DefaultTimeoutSeconds;
            var now = Time.realtimeSinceStartup;

            if (command.hasMainThrottle && IsFinite(command.mainThrottle))
            {
                commandedMainThrottle = Mathf.Clamp01((float)command.mainThrottle);
                mainThrottleExpiresAt = now + timeout;
                mainThrottleActive = true;
            }

            if (command.hasRcsCommand &&
                IsFinite(command.rcsX) && IsFinite(command.rcsY) && IsFinite(command.rcsZ) &&
                IsFinite(command.rcsPitch) && IsFinite(command.rcsYaw) && IsFinite(command.rcsRoll))
            {
                if (!rcsActionGroupOverridden)
                {
                    rcsActionGroupWasActive = attachedVessel.ActionGroups[KSPActionGroup.RCS];
                    rcsActionGroupOverridden = true;
                }

                attachedVessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                commandedRcsX = ClampAxis(command.rcsX);
                commandedRcsY = ClampAxis(command.rcsY);
                commandedRcsZ = ClampAxis(command.rcsZ);
                commandedRcsPitch = ClampAxis(command.rcsPitch);
                commandedRcsYaw = ClampAxis(command.rcsYaw);
                commandedRcsRoll = ClampAxis(command.rcsRoll);
                rcsCommandExpiresAt = now + timeout;
                rcsCommandActive = true;
            }

            var moduleCommands = command.commands;
            if (moduleCommands == null)
            {
                return;
            }

            for (var index = 0; index < moduleCommands.Length; index++)
            {
                ApplyModuleCommand(moduleCommands[index], now, timeout);
            }
        }

        private void ApplyModuleCommand(KerbalRosPropulsionModuleCommand command, float now, float timeout)
        {
            if (command == null)
            {
                return;
            }

            var targetAll = string.Equals(command.name, "*", StringComparison.Ordinal);
            var targetName = targetAll
                ? string.Empty
                : KerbalRosMotorNames.Sanitize(command.name, "propulsion");
            var kind = (command.kind ?? string.Empty).Trim().ToLowerInvariant();

            for (var partIndex = 0; partIndex < attachedVessel.parts.Count; partIndex++)
            {
                var candidatePart = attachedVessel.parts[partIndex];
                if (candidatePart == null)
                {
                    continue;
                }

                for (var moduleIndex = 0; moduleIndex < candidatePart.Modules.Count; moduleIndex++)
                {
                    var engine = candidatePart.Modules[moduleIndex] as ModuleEngines;
                    if (engine != null && (kind.Length == 0 || kind == "engine") &&
                        Matches(command, targetAll, targetName, candidatePart, moduleIndex, "engine"))
                    {
                        ApplyEngineCommand(engine, command, now, timeout);
                        continue;
                    }

                    var rcs = candidatePart.Modules[moduleIndex] as ModuleRCS;
                    if (rcs != null && (kind.Length == 0 || kind == "rcs") &&
                        Matches(command, targetAll, targetName, candidatePart, moduleIndex, "rcs"))
                    {
                        ApplyRcsCommand(rcs, command, now, timeout);
                    }
                }
            }
        }

        private static bool Matches(
            KerbalRosPropulsionModuleCommand command,
            bool targetAll,
            string targetName,
            Part candidatePart,
            int moduleIndex,
            string kind)
        {
            if (targetAll)
            {
                return true;
            }

            if (command.partFlightId > 0)
            {
                return command.partFlightId == candidatePart.flightID && command.moduleIndex == moduleIndex;
            }

            return targetName.Length > 0 && targetName == ModuleName(kind, candidatePart, moduleIndex);
        }

        private void ApplyEngineCommand(
            ModuleEngines engine,
            KerbalRosPropulsionModuleCommand command,
            float now,
            float timeout)
        {
            if (command.release)
            {
                RestoreEngine(engine);
                return;
            }

            EngineOverride original;
            if (!engineOverrides.TryGetValue(engine, out original))
            {
                original = new EngineOverride
                {
                    IndependentThrottle = engine.independentThrottle,
                    IndependentThrottlePercentage = engine.independentThrottlePercentage
                };
                engineOverrides.Add(engine, original);
            }

            if (command.hasEnabled)
            {
                if (command.enabled && !engine.EngineIgnited)
                {
                    engine.Activate();
                }
                else if (!command.enabled && engine.EngineIgnited)
                {
                    engine.Shutdown();
                }
            }

            if (command.hasThrottle && IsFinite(command.throttle))
            {
                engine.independentThrottle = true;
                engine.independentThrottlePercentage = Mathf.Clamp01((float)command.throttle) * 100f;
                original.ExpiresAt = now + timeout;
                original.TimedOut = false;
            }
        }

        private void ApplyRcsCommand(
            ModuleRCS rcs,
            KerbalRosPropulsionModuleCommand command,
            float now,
            float timeout)
        {
            if (command.release)
            {
                RestoreRcs(rcs);
                return;
            }

            RcsOverride original;
            if (!rcsOverrides.TryGetValue(rcs, out original))
            {
                original = new RcsOverride
                {
                    Enabled = rcs.rcsEnabled,
                    ThrustPercentage = rcs.thrustPercentage
                };
                rcsOverrides.Add(rcs, original);
            }

            if (command.hasEnabled)
            {
                rcs.rcsEnabled = command.enabled;
            }

            if (command.hasThrottle && IsFinite(command.throttle))
            {
                rcs.thrustPercentage = Mathf.Clamp01((float)command.throttle) * 100f;
                original.ExpiresAt = now + timeout;
                original.TimedOut = false;
            }
        }

        private void ApplyTimeouts()
        {
            var now = Time.realtimeSinceStartup;
            if (mainThrottleActive && now > mainThrottleExpiresAt)
            {
                commandedMainThrottle = 0f;
                mainThrottleActive = false;
                if (attachedVessel != null && attachedVessel.ctrlState != null)
                {
                    attachedVessel.ctrlState.mainThrottle = 0f;
                }
            }

            if (rcsCommandActive && now > rcsCommandExpiresAt)
            {
                commandedRcsX = 0f;
                commandedRcsY = 0f;
                commandedRcsZ = 0f;
                commandedRcsPitch = 0f;
                commandedRcsYaw = 0f;
                commandedRcsRoll = 0f;
                rcsCommandActive = false;
                RestoreRcsActionGroup();
            }

            foreach (var pair in engineOverrides)
            {
                if (pair.Key != null && !pair.Value.TimedOut && now > pair.Value.ExpiresAt)
                {
                    pair.Key.independentThrottle = true;
                    pair.Key.independentThrottlePercentage = 0f;
                    pair.Value.TimedOut = true;
                }
            }

            foreach (var pair in rcsOverrides)
            {
                if (pair.Key != null && !pair.Value.TimedOut && now > pair.Value.ExpiresAt)
                {
                    pair.Key.thrustPercentage = 0f;
                    pair.Value.TimedOut = true;
                }
            }
        }

        private void SendStates()
        {
            if (stateClient == null || attachedVessel == null || Time.realtimeSinceStartup < nextStateTime)
            {
                return;
            }

            nextStateTime = Time.realtimeSinceStartup + 1f / StateRateHz;
            for (var partIndex = 0; partIndex < attachedVessel.parts.Count; partIndex++)
            {
                var candidatePart = attachedVessel.parts[partIndex];
                if (candidatePart == null)
                {
                    continue;
                }

                for (var moduleIndex = 0; moduleIndex < candidatePart.Modules.Count; moduleIndex++)
                {
                    string packet = null;
                    var engine = candidatePart.Modules[moduleIndex] as ModuleEngines;
                    if (engine != null)
                    {
                        packet = BuildEngineState(candidatePart, moduleIndex, engine);
                    }
                    else
                    {
                        var rcs = candidatePart.Modules[moduleIndex] as ModuleRCS;
                        if (rcs != null)
                        {
                            packet = BuildRcsState(candidatePart, moduleIndex, rcs);
                        }
                    }

                    if (packet == null)
                    {
                        continue;
                    }

                    try
                    {
                        var bytes = Encoding.UTF8.GetBytes(packet);
                        stateClient.Send(bytes, bytes.Length, stateEndpoint);
                    }
                    catch (Exception exception)
                    {
                        Warn("ROS propulsion state UDP send failed: " + exception.Message);
                        return;
                    }
                }
            }
        }

        private string BuildEngineState(Part candidatePart, int moduleIndex, ModuleEngines engine)
        {
            EngineOverride overrideState;
            var commandActive = engineOverrides.TryGetValue(engine, out overrideState);
            var builder = BeginState(candidatePart, moduleIndex, "engine");
            AppendString(builder, "engineId", engine.engineID);
            AppendBoolean(builder, "enabled", engine.EngineIgnited);
            AppendBoolean(builder, "operational", engine.isOperational);
            AppendBoolean(builder, "flameout", engine.flameout);
            AppendBoolean(builder, "throttleable", !engine.throttleLocked);
            AppendNumber(builder, "throttle", engine.currentThrottle);
            AppendNumber(builder, "throttleLimit", engine.independentThrottle
                ? engine.independentThrottlePercentage / 100f
                : engine.thrustPercentage / 100f);
            AppendNumber(builder, "thrust", engine.finalThrust);
            AppendNumber(builder, "maxThrust", engine.maxThrust);
            AppendBoolean(builder, "commandActive", commandActive);
            AppendBoolean(builder, "timedOut", commandActive && overrideState.TimedOut);
            builder.Append('}');
            return builder.ToString();
        }

        private string BuildRcsState(Part candidatePart, int moduleIndex, ModuleRCS rcs)
        {
            RcsOverride overrideState;
            var commandActive = rcsOverrides.TryGetValue(rcs, out overrideState);
            var thrust = 0f;
            if (rcs.thrustForces != null)
            {
                for (var index = 0; index < rcs.thrustForces.Length; index++)
                {
                    thrust += Mathf.Abs(rcs.thrustForces[index]);
                }
            }

            var builder = BeginState(candidatePart, moduleIndex, "rcs");
            AppendBoolean(builder, "enabled", rcs.rcsEnabled);
            AppendBoolean(builder, "active", rcs.rcs_active);
            AppendBoolean(builder, "flameout", rcs.flameout);
            AppendNumber(builder, "throttleLimit", rcs.thrustPercentage / 100f);
            AppendNumber(builder, "thrust", thrust);
            AppendNumber(builder, "maxThrust", rcs.thrusterPower);
            AppendNumber(builder, "thrusterCount", rcs.thrusterTransforms == null ? 0 : rcs.thrusterTransforms.Count);
            AppendBoolean(builder, "commandActive", commandActive);
            AppendBoolean(builder, "timedOut", commandActive && overrideState.TimedOut);
            builder.Append('}');
            return builder.ToString();
        }

        private StringBuilder BeginState(Part candidatePart, int moduleIndex, string kind)
        {
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "ksp_propulsion_state", true);
            AppendNumber(builder, "version", 1);
            AppendString(builder, "name", ModuleName(kind, candidatePart, moduleIndex));
            AppendString(builder, "kind", kind);
            AppendString(builder, "vessel", attachedVessel == null ? string.Empty : attachedVessel.vesselName);
            AppendNumber(builder, "partFlightId", candidatePart.flightID);
            AppendNumber(builder, "moduleIndex", moduleIndex);
            AppendNumber(builder, "universalTime", Planetarium.fetch == null ? 0.0 : Planetarium.GetUniversalTime());
            return builder;
        }

        private static string ModuleName(string kind, Part candidatePart, int moduleIndex)
        {
            return KerbalRosActuatorNames.For(kind, candidatePart, moduleIndex);
        }

        private void RestoreAllOverrides()
        {
            var engines = new List<ModuleEngines>(engineOverrides.Keys);
            for (var index = 0; index < engines.Count; index++)
            {
                RestoreEngine(engines[index]);
            }

            var rcsModules = new List<ModuleRCS>(rcsOverrides.Keys);
            for (var index = 0; index < rcsModules.Count; index++)
            {
                RestoreRcs(rcsModules[index]);
            }
        }

        private void RestoreEngine(ModuleEngines engine)
        {
            EngineOverride original;
            if (engine == null || !engineOverrides.TryGetValue(engine, out original))
            {
                return;
            }

            engine.independentThrottle = original.IndependentThrottle;
            engine.independentThrottlePercentage = original.IndependentThrottlePercentage;
            engineOverrides.Remove(engine);
        }

        private void RestoreRcs(ModuleRCS rcs)
        {
            RcsOverride original;
            if (rcs == null || !rcsOverrides.TryGetValue(rcs, out original))
            {
                return;
            }

            rcs.rcsEnabled = original.Enabled;
            rcs.thrustPercentage = original.ThrustPercentage;
            rcsOverrides.Remove(rcs);
        }

        private static float ClampAxis(double value)
        {
            return Mathf.Clamp((float)value, -1f, 1f);
        }

        private void RestoreRcsActionGroup()
        {
            if (!rcsActionGroupOverridden)
            {
                return;
            }

            if (attachedVessel != null)
            {
                attachedVessel.ActionGroups.SetGroup(KSPActionGroup.RCS, rcsActionGroupWasActive);
            }

            rcsActionGroupOverridden = false;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static void AppendPrefix(StringBuilder builder, string name, bool first = false)
        {
            if (!first)
            {
                builder.Append(',');
            }

            AppendJsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendString(StringBuilder builder, string name, string value, bool first = false)
        {
            AppendPrefix(builder, name, first);
            AppendJsonString(builder, value);
        }

        private static void AppendNumber(StringBuilder builder, string name, double value)
        {
            AppendPrefix(builder, name);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendBoolean(StringBuilder builder, string name, bool value)
        {
            AppendPrefix(builder, name);
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
