using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using ModuleWheels;
using UnityEngine;

namespace KerbalLiDAR
{
    [Serializable]
    public sealed class KerbalRosBodyWrenchCommand
    {
        public string type;
        public int version;
        public string frame;
        public double[] force;
        public double[] torque;
        public double timeoutSeconds;
        public long sequence;
    }

    [Serializable]
    public sealed class KerbalRosActuatorCommand
    {
        public string type;
        public int version;
        public string actuatorType;
        public string name;
        public bool enabled;
        public double timeoutSeconds;
        public long sequence;
        public double targetAngularVelocity;
        public double steeringAngle;
        public double maxDriveTorque;
        public double targetThrust;
        public double thrustLimit;
        public bool separate;
    }

    internal static class KerbalRosActuatorNames
    {
        internal static string For(string kind, Part part, int moduleIndex)
        {
            var id = part == null ? 0u : part.persistentId;
            if (id == 0u && part != null)
            {
                id = part.flightID;
            }
            return KerbalRosMotorNames.Sanitize(
                kind + "_" + id.ToString(CultureInfo.InvariantCulture) + "_" +
                Math.Max(0, moduleIndex).ToString(CultureInfo.InvariantCulture),
                kind);
        }

        internal static int ModuleIndex(Part part, PartModule module)
        {
            if (part == null || module == null)
            {
                return 0;
            }
            for (var index = 0; index < part.Modules.Count; index++)
            {
                if (ReferenceEquals(part.Modules[index], module))
                {
                    return index;
                }
            }
            return 0;
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class KerbalRosVehicleManager : MonoBehaviour
    {
        private const int ProtocolVersion = 1;
        private const int CommandPort = 49011;
        private const int StatePort = 49010;
        private const float StateRateHz = 30f;
        private const float DefaultTimeout = 0.5f;

        private sealed class WheelOverride
        {
            public bool Enabled;
            public float TargetAngularVelocity;
            public float SteeringAngleDegrees;
            public float MaxDriveTorque;
            public float ExpiresAt;
        }

        private sealed class EngineOverride
        {
            public bool Enabled;
            public float TargetThrust;
            public float ExpiresAt;
        }

        private sealed class RcsOverride
        {
            public bool Enabled;
            public float ThrustLimit;
            public float ExpiresAt;
        }

        private sealed class OriginalEngineState
        {
            public bool IndependentThrottle;
            public float IndependentThrottlePercentage;
        }

        private sealed class OriginalRcsState
        {
            public bool Enabled;
            public float ThrustPercentage;
        }

        private sealed class OriginalWheelState
        {
            public bool MotorEnabled;
            public float MaxDriveTorque;
        }

        private struct EngineChannel
        {
            public ModuleEngines Engine;
            public Vector3 Force;
            public Vector3 Torque;
            public float MaximumThrust;
        }

        private static KerbalRosVehicleManager instance;
        private readonly Dictionary<string, WheelOverride> wheelOverrides = new Dictionary<string, WheelOverride>();
        private readonly Dictionary<string, EngineOverride> engineOverrides = new Dictionary<string, EngineOverride>();
        private readonly Dictionary<string, RcsOverride> rcsOverrides = new Dictionary<string, RcsOverride>();
        private readonly Dictionary<ModuleEngines, OriginalEngineState> bodyEngineStates =
            new Dictionary<ModuleEngines, OriginalEngineState>();
        private readonly Dictionary<ModuleRCS, OriginalRcsState> directRcsStates =
            new Dictionary<ModuleRCS, OriginalRcsState>();
        private readonly Dictionary<ModuleWheelBase, OriginalWheelState> directWheelStates =
            new Dictionary<ModuleWheelBase, OriginalWheelState>();
        private UdpClient stateClient;
        private IPEndPoint stateEndpoint;
        private Vessel vessel;
        private CelestialBody anchorBody;
        private Vector3d anchorPosition;
        private Vector3d anchorEast;
        private Vector3d anchorNorth;
        private Vector3d anchorUp;
        private Vector3 previousLinearVelocity;
        private Vector3 previousAngularVelocity;
        private double previousTruthTime;
        private bool derivativeReady;
        private int originSequence;
        private float nextStateTime;
        private float nextManifestTime;
        private bool wrenchActive;
        private Vector3 requestedForce;
        private Vector3 requestedTorque;
        private float wrenchExpiresAt;
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
            ExpireCommands();
            ApplyDirectWheelCommands();
            if (vessel == null || Time.realtimeSinceStartup < nextStateTime)
            {
                return;
            }
            nextStateTime = Time.realtimeSinceStartup + 1f / StateRateHz;
            if (Time.realtimeSinceStartup >= nextManifestTime)
            {
                SendActuatorManifest();
                nextManifestTime = Time.realtimeSinceStartup + 1f;
            }
            SendGroundTruth();
            SendActuatorStates();
        }

        public void OnDestroy()
        {
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
            if (envelope == null ||
                (envelope.type != "ksp_body_wrench_command" && envelope.type != "ksp_actuator_command"))
            {
                return false;
            }
            if (port != CommandPort || envelope.version != ProtocolVersion || instance == null)
            {
                return true;
            }
            try
            {
                if (envelope.type == "ksp_body_wrench_command")
                {
                    instance.ApplyBodyWrench(JsonUtility.FromJson<KerbalRosBodyWrenchCommand>(json));
                }
                else
                {
                    instance.ApplyActuatorCommand(JsonUtility.FromJson<KerbalRosActuatorCommand>(json));
                }
            }
            catch (Exception exception)
            {
                instance.Warn("Dropped invalid vehicle command: " + exception.Message);
            }
            return true;
        }

        private void AttachToActiveVessel()
        {
            var active = FlightGlobals.ActiveVessel;
            if (active == vessel && (active == null || active.mainBody == anchorBody))
            {
                return;
            }
            DetachFromVessel();
            vessel = active;
            if (vessel == null)
            {
                return;
            }
            vessel.OnFlyByWire += ApplyFlyByWire;
            ResetGroundTruthOrigin();
        }

        private void DetachFromVessel()
        {
            if (vessel != null)
            {
                vessel.OnFlyByWire -= ApplyFlyByWire;
            }
            RestoreBodyEngineStates();
            RestoreDirectRcsStates();
            RestoreDirectWheelStates();
            wheelOverrides.Clear();
            engineOverrides.Clear();
            rcsOverrides.Clear();
            wrenchActive = false;
            vessel = null;
            anchorBody = null;
            derivativeReady = false;
        }

        private void ResetGroundTruthOrigin()
        {
            if (vessel == null || vessel.mainBody == null)
            {
                return;
            }
            anchorBody = vessel.mainBody;
            anchorPosition = GeodeticPosition(vessel.latitude, vessel.longitude, vessel.altitude, anchorBody.Radius);
            TangentBasis(vessel.latitude, vessel.longitude, out anchorEast, out anchorNorth, out anchorUp);
            previousTruthTime = Planetarium.GetUniversalTime();
            previousLinearVelocity = Vector3.zero;
            previousAngularVelocity = Vector3.zero;
            derivativeReady = false;
            originSequence++;
            nextManifestTime = 0f;
        }

        private void ApplyBodyWrench(KerbalRosBodyWrenchCommand command)
        {
            if (command == null || !VectorIsFinite(command.force) || !VectorIsFinite(command.torque))
            {
                return;
            }
            requestedForce = new Vector3((float)command.force[0], (float)command.force[1], (float)command.force[2]);
            requestedTorque = new Vector3((float)command.torque[0], (float)command.torque[1], (float)command.torque[2]);
            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : DefaultTimeout;
            wrenchExpiresAt = Time.realtimeSinceStartup + timeout;
            wrenchActive = true;
        }

        private void ApplyActuatorCommand(KerbalRosActuatorCommand command)
        {
            if (command == null || string.IsNullOrEmpty(command.name))
            {
                return;
            }
            var name = KerbalRosMotorNames.Sanitize(command.name, "actuator");
            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : DefaultTimeout;
            var expiresAt = Time.realtimeSinceStartup + timeout;
            switch ((command.actuatorType ?? string.Empty).Trim().ToLowerInvariant())
            {
                case "wheel":
                    if (!IsFinite(command.targetAngularVelocity) || !IsFinite(command.steeringAngle) ||
                        !IsFinite(command.maxDriveTorque)) return;
                    wheelOverrides[name] = new WheelOverride
                    {
                        Enabled = command.enabled,
                        TargetAngularVelocity = (float)command.targetAngularVelocity,
                        SteeringAngleDegrees = (float)(command.steeringAngle * Mathf.Rad2Deg),
                        MaxDriveTorque = Mathf.Max(0f, (float)command.maxDriveTorque),
                        ExpiresAt = expiresAt
                    };
                    break;
                case "engine":
                    if (!IsFinite(command.targetThrust)) return;
                    engineOverrides[name] = new EngineOverride
                    {
                        Enabled = command.enabled,
                        TargetThrust = Mathf.Max(0f, (float)command.targetThrust),
                        ExpiresAt = expiresAt
                    };
                    break;
                case "rcs":
                    if (!IsFinite(command.thrustLimit)) return;
                    rcsOverrides[name] = new RcsOverride
                    {
                        Enabled = command.enabled,
                        ThrustLimit = Mathf.Max(0f, (float)command.thrustLimit),
                        ExpiresAt = expiresAt
                    };
                    break;
                case "separation":
                    ApplySeparationCommand(name, command.separate);
                    break;
            }
        }

        private void ApplySeparationCommand(string name, bool separate)
        {
            if (!separate)
            {
                return;
            }
            foreach (var module in SeparationModules())
            {
                var mechanism = SeparationMechanism(module);
                var targetName = KerbalRosActuatorNames.For(
                    mechanism, module.part, KerbalRosActuatorNames.ModuleIndex(module.part, module));
                if (targetName != name || !SeparationAvailable(module))
                {
                    continue;
                }
                SendSeparationState(module);
                var decoupler = module as ModuleDecouplerBase;
                if (decoupler != null)
                {
                    decoupler.Decouple();
                }
                else
                {
                    var fairing = module as ModuleProceduralFairing;
                    if (fairing != null)
                    {
                        fairing.DeployFairing();
                    }
                }
                SendSeparationState(module);
                nextManifestTime = 0f;
                return;
            }
        }

        private void ExpireCommands()
        {
            var now = Time.realtimeSinceStartup;
            if (wrenchActive && now > wrenchExpiresAt)
            {
                wrenchActive = false;
                requestedForce = Vector3.zero;
                requestedTorque = Vector3.zero;
                RestoreBodyEngineStates();
            }
            ExpireWheelOverrides(now);
            ExpireEngineOverrides(now);
            ExpireRcsOverrides(now);
        }

        private static void RemoveExpired<T>(Dictionary<string, T> values, float now, Func<T, float> expiry)
        {
            var expired = new List<string>();
            foreach (var pair in values)
            {
                if (now > expiry(pair.Value)) expired.Add(pair.Key);
            }
            for (var index = 0; index < expired.Count; index++) values.Remove(expired[index]);
        }

        private void ExpireWheelOverrides(float now)
        {
            var expired = ExpiredNames(wheelOverrides, now, delegate(WheelOverride value) { return value.ExpiresAt; });
            for (var index = 0; index < expired.Count; index++)
            {
                var name = expired[index];
                foreach (var wheel in WheelBases())
                {
                    if (KerbalRosActuatorNames.For("wheel", wheel.part,
                        KerbalRosActuatorNames.ModuleIndex(wheel.part, wheel)) != name) continue;
                    OriginalWheelState original;
                    if (directWheelStates.TryGetValue(wheel, out original))
                    {
                        var motor = wheel.part.FindModuleImplementing<ModuleWheelMotor>();
                        if (motor != null) motor.motorEnabled = original.MotorEnabled;
                        if (wheel.Wheel != null)
                        {
                            wheel.Wheel.driveInput = 0f;
                            wheel.Wheel.steerInput = 0f;
                            wheel.Wheel.maxDriveTorque = original.MaxDriveTorque;
                        }
                        directWheelStates.Remove(wheel);
                    }
                }
                wheelOverrides.Remove(name);
            }
        }

        private void ExpireEngineOverrides(float now)
        {
            var expired = ExpiredNames(engineOverrides, now, delegate(EngineOverride value) { return value.ExpiresAt; });
            for (var index = 0; index < expired.Count; index++)
            {
                var name = expired[index];
                engineOverrides.Remove(name);
                if (wrenchActive) continue;
                foreach (var engine in EngineModules())
                {
                    if (KerbalRosActuatorNames.For("engine", engine.part,
                        KerbalRosActuatorNames.ModuleIndex(engine.part, engine)) != name) continue;
                    OriginalEngineState original;
                    if (bodyEngineStates.TryGetValue(engine, out original))
                    {
                        engine.independentThrottle = original.IndependentThrottle;
                        engine.independentThrottlePercentage = original.IndependentThrottlePercentage;
                        bodyEngineStates.Remove(engine);
                    }
                }
            }
        }

        private void ExpireRcsOverrides(float now)
        {
            var expired = ExpiredNames(rcsOverrides, now, delegate(RcsOverride value) { return value.ExpiresAt; });
            for (var index = 0; index < expired.Count; index++)
            {
                var name = expired[index];
                foreach (var rcs in RcsModules())
                {
                    if (KerbalRosActuatorNames.For("rcs", rcs.part,
                        KerbalRosActuatorNames.ModuleIndex(rcs.part, rcs)) != name) continue;
                    OriginalRcsState original;
                    if (directRcsStates.TryGetValue(rcs, out original))
                    {
                        rcs.rcsEnabled = original.Enabled;
                        rcs.thrustPercentage = original.ThrustPercentage;
                        directRcsStates.Remove(rcs);
                    }
                }
                rcsOverrides.Remove(name);
            }
        }

        private static List<string> ExpiredNames<T>(Dictionary<string, T> values, float now, Func<T, float> expiry)
        {
            var result = new List<string>();
            foreach (var pair in values)
            {
                if (now > expiry(pair.Value)) result.Add(pair.Key);
            }
            return result;
        }

        private void ApplyFlyByWire(FlightCtrlState state)
        {
            if (state == null || vessel == null)
            {
                return;
            }
            ApplyDirectEngineAndRcsCommands();
            if (!wrenchActive)
            {
                return;
            }

            var residualForce = requestedForce;
            var residualTorque = requestedTorque;
            AllocateWheels(state, ref residualForce, ref residualTorque);
            AllocateEngines(ref residualForce, ref residualTorque);
            AllocateRcs(state, ref residualForce, ref residualTorque);
            SendWrenchStatus(residualForce, residualTorque);
        }

        private void AllocateWheels(FlightCtrlState state, ref Vector3 force, ref Vector3 torque)
        {
            var maximumForce = 0f;
            var steeringLever = 0f;
            var count = 0;
            foreach (var wheelBase in WheelBases())
            {
                var name = KerbalRosActuatorNames.For("wheel", wheelBase.part,
                    KerbalRosActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
                if (wheelOverrides.ContainsKey(name) || wheelBase.Wheel == null || !wheelBase.Wheel.IsGrounded)
                {
                    continue;
                }
                var motor = wheelBase.part.FindModuleImplementing<ModuleWheelMotor>();
                if (motor == null || !motor.motorEnabled)
                {
                    continue;
                }
                var radius = Mathf.Max(0.01f, wheelBase.Wheel.WheelRadius);
                maximumForce += Mathf.Max(0f, motor.maxTorque) / radius;
                steeringLever += Mathf.Abs(Vector3.Dot(wheelBase.part.transform.position - vessel.CoM, BodyForwardWorld()));
                count++;
            }
            if (maximumForce <= 0.001f || count == 0)
            {
                return;
            }
            steeringLever = Mathf.Max(0.5f, steeringLever / count);
            var drive = Mathf.Clamp(force.x / maximumForce, -1f, 1f);
            var steer = Mathf.Clamp(torque.z / (maximumForce * steeringLever), -1f, 1f);
            state.wheelThrottle = drive;
            state.wheelSteer = steer;
            force.x -= drive * maximumForce;
            torque.z -= steer * maximumForce * steeringLever;
        }

        private void AllocateEngines(ref Vector3 force, ref Vector3 torque)
        {
            var channels = EngineChannels();
            if (channels.Count == 0)
            {
                return;
            }
            var commands = SolveBounded(channels, force, torque);
            var achievedForce = Vector3.zero;
            var achievedTorque = Vector3.zero;
            for (var index = 0; index < channels.Count; index++)
            {
                var channel = channels[index];
                RememberEngineState(channel.Engine);
                channel.Engine.independentThrottle = true;
                channel.Engine.independentThrottlePercentage = commands[index] * 100f;
                achievedForce += channel.Force * commands[index];
                achievedTorque += channel.Torque * commands[index];
            }
            force -= achievedForce;
            torque -= achievedTorque;
        }

        private void AllocateRcs(FlightCtrlState state, ref Vector3 force, ref Vector3 torque)
        {
            var maxForce = 0f;
            var maxTorque = 0f;
            var enabled = false;
            foreach (var rcs in RcsModules())
            {
                var name = KerbalRosActuatorNames.For("rcs", rcs.part,
                    KerbalRosActuatorNames.ModuleIndex(rcs.part, rcs));
                RcsOverride direct;
                if (rcsOverrides.TryGetValue(name, out direct))
                {
                    rcs.rcsEnabled = direct.Enabled;
                    rcs.thrustPercentage = rcs.thrusterPower > 0.001f
                        ? Mathf.Clamp01(direct.ThrustLimit / rcs.thrusterPower) * 100f
                        : 0f;
                }
                if (!rcs.rcsEnabled || rcs.flameout)
                {
                    continue;
                }
                enabled = true;
                var nozzleCount = rcs.thrusterTransforms == null ? 1 : Math.Max(1, rcs.thrusterTransforms.Count);
                maxForce += rcs.thrusterPower * nozzleCount * Mathf.Clamp01(rcs.thrustPercentage / 100f);
                maxTorque += rcs.thrusterPower * nozzleCount *
                    Mathf.Max(0.5f, Vector3.Distance(rcs.part.transform.position, vessel.CoM));
            }
            if (!enabled)
            {
                return;
            }
            vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
            maxForce = Mathf.Max(0.001f, maxForce);
            maxTorque = Mathf.Max(0.001f, maxTorque);
            var x = Mathf.Clamp(force.x / maxForce, -1f, 1f);
            var y = Mathf.Clamp(force.y / maxForce, -1f, 1f);
            var z = Mathf.Clamp(force.z / maxForce, -1f, 1f);
            var roll = Mathf.Clamp(torque.x / maxTorque, -1f, 1f);
            var pitch = Mathf.Clamp(torque.y / maxTorque, -1f, 1f);
            var yaw = Mathf.Clamp(torque.z / maxTorque, -1f, 1f);
            state.Z = x;
            state.X = -y;
            state.Y = z;
            state.roll = roll;
            state.pitch = pitch;
            state.yaw = yaw;
            force -= new Vector3(x, y, z) * maxForce;
            torque -= new Vector3(roll, pitch, yaw) * maxTorque;
        }

        private List<float> SolveBounded(List<EngineChannel> channels, Vector3 force, Vector3 torque)
        {
            var values = new List<float>(channels.Count);
            for (var index = 0; index < channels.Count; index++) values.Add(0f);
            var scale = 1f;
            for (var index = 0; index < channels.Count; index++)
            {
                scale += channels[index].Force.sqrMagnitude + channels[index].Torque.sqrMagnitude;
            }
            var step = 1f / scale;
            for (var iteration = 0; iteration < 48; iteration++)
            {
                var errorForce = -force;
                var errorTorque = -torque;
                for (var index = 0; index < channels.Count; index++)
                {
                    errorForce += channels[index].Force * values[index];
                    errorTorque += channels[index].Torque * values[index];
                }
                for (var index = 0; index < channels.Count; index++)
                {
                    var gradient = Vector3.Dot(channels[index].Force, errorForce) +
                                   Vector3.Dot(channels[index].Torque, errorTorque) + values[index] * 0.0001f;
                    values[index] = Mathf.Clamp01(values[index] - step * gradient);
                }
            }
            return values;
        }

        private List<EngineChannel> EngineChannels()
        {
            var channels = new List<EngineChannel>();
            foreach (var engine in EngineModules())
            {
                var name = KerbalRosActuatorNames.For("engine", engine.part,
                    KerbalRosActuatorNames.ModuleIndex(engine.part, engine));
                if (engineOverrides.ContainsKey(name) || !engine.isOperational || engine.flameout)
                {
                    continue;
                }
                var maxThrust = Mathf.Max(0f, engine.GetMaxThrust());
                if (maxThrust <= 0.001f)
                {
                    continue;
                }
                var direction = Vector3.zero;
                var center = Vector3.zero;
                var transforms = engine.thrustTransforms;
                if (transforms != null && transforms.Count > 0)
                {
                    for (var index = 0; index < transforms.Count; index++)
                    {
                        direction += -transforms[index].forward;
                        center += transforms[index].position;
                    }
                    center /= transforms.Count;
                }
                else
                {
                    direction = BodyForwardWorld();
                    center = engine.part.transform.position;
                }
                direction.Normalize();
                var force = WorldVectorToBody(direction) * maxThrust;
                var offset = WorldVectorToBody(center - vessel.CoM);
                channels.Add(new EngineChannel
                {
                    Engine = engine,
                    Force = force,
                    Torque = Vector3.Cross(offset, force),
                    MaximumThrust = maxThrust
                });
            }
            return channels;
        }

        private void ApplyDirectEngineAndRcsCommands()
        {
            foreach (var engine in EngineModules())
            {
                var name = KerbalRosActuatorNames.For("engine", engine.part,
                    KerbalRosActuatorNames.ModuleIndex(engine.part, engine));
                EngineOverride direct;
                if (!engineOverrides.TryGetValue(name, out direct)) continue;
                RememberEngineState(engine);
                if (direct.Enabled && !engine.EngineIgnited) engine.Activate();
                if (!direct.Enabled && engine.EngineIgnited) engine.Shutdown();
                engine.independentThrottle = true;
                var max = Mathf.Max(0.001f, engine.GetMaxThrust());
                engine.independentThrottlePercentage = direct.Enabled
                    ? Mathf.Clamp01(direct.TargetThrust / max) * 100f
                    : 0f;
            }
            foreach (var rcs in RcsModules())
            {
                var name = KerbalRosActuatorNames.For("rcs", rcs.part,
                    KerbalRosActuatorNames.ModuleIndex(rcs.part, rcs));
                RcsOverride direct;
                if (!rcsOverrides.TryGetValue(name, out direct)) continue;
                if (!directRcsStates.ContainsKey(rcs))
                {
                    directRcsStates[rcs] = new OriginalRcsState
                    {
                        Enabled = rcs.rcsEnabled,
                        ThrustPercentage = rcs.thrustPercentage
                    };
                }
                rcs.rcsEnabled = direct.Enabled;
                rcs.thrustPercentage = direct.Enabled && rcs.thrusterPower > 0.001f
                    ? Mathf.Clamp01(direct.ThrustLimit / rcs.thrusterPower) * 100f
                    : 0f;
            }
        }

        private void ApplyDirectWheelCommands()
        {
            if (vessel == null) return;
            foreach (var wheelBase in WheelBases())
            {
                var controller = wheelBase.Wheel;
                if (controller == null) continue;
                var name = KerbalRosActuatorNames.For("wheel", wheelBase.part,
                    KerbalRosActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
                WheelOverride direct;
                if (!wheelOverrides.TryGetValue(name, out direct)) continue;
                var motor = wheelBase.part.FindModuleImplementing<ModuleWheelMotor>();
                if (!directWheelStates.ContainsKey(wheelBase))
                {
                    directWheelStates[wheelBase] = new OriginalWheelState
                    {
                        MotorEnabled = motor != null && motor.motorEnabled,
                        MaxDriveTorque = controller.maxDriveTorque
                    };
                }
                if (motor != null) motor.motorEnabled = direct.Enabled;
                if (!direct.Enabled)
                {
                    controller.driveInput = 0f;
                    controller.steerInput = 0f;
                    continue;
                }
                var angularVelocity = controller.currentState == null ? 0f : controller.currentState.angularVelocity;
                controller.maxDriveTorque = direct.MaxDriveTorque > 0f
                    ? direct.MaxDriveTorque
                    : motor == null ? controller.maxDriveTorque : motor.maxTorque;
                controller.driveInput = Mathf.Clamp((direct.TargetAngularVelocity - angularVelocity) * 0.1f, -1f, 1f);
                controller.steerInput = controller.maxSteerAngle > 0.001f
                    ? Mathf.Clamp(direct.SteeringAngleDegrees / controller.maxSteerAngle, -1f, 1f)
                    : 0f;
            }
        }

        private void RememberEngineState(ModuleEngines engine)
        {
            if (engine == null || bodyEngineStates.ContainsKey(engine)) return;
            bodyEngineStates[engine] = new OriginalEngineState
            {
                IndependentThrottle = engine.independentThrottle,
                IndependentThrottlePercentage = engine.independentThrottlePercentage
            };
        }

        private void RestoreBodyEngineStates()
        {
            foreach (var pair in bodyEngineStates)
            {
                if (pair.Key == null) continue;
                pair.Key.independentThrottle = pair.Value.IndependentThrottle;
                pair.Key.independentThrottlePercentage = pair.Value.IndependentThrottlePercentage;
            }
            bodyEngineStates.Clear();
        }

        private void RestoreDirectRcsStates()
        {
            foreach (var pair in directRcsStates)
            {
                if (pair.Key == null) continue;
                pair.Key.rcsEnabled = pair.Value.Enabled;
                pair.Key.thrustPercentage = pair.Value.ThrustPercentage;
            }
            directRcsStates.Clear();
        }

        private void RestoreDirectWheelStates()
        {
            foreach (var pair in directWheelStates)
            {
                if (pair.Key == null || pair.Key.Wheel == null) continue;
                var motor = pair.Key.part.FindModuleImplementing<ModuleWheelMotor>();
                if (motor != null) motor.motorEnabled = pair.Value.MotorEnabled;
                pair.Key.Wheel.driveInput = 0f;
                pair.Key.Wheel.steerInput = 0f;
                pair.Key.Wheel.maxDriveTorque = pair.Value.MaxDriveTorque;
            }
            directWheelStates.Clear();
        }

        private void SendGroundTruth()
        {
            if (vessel == null || anchorBody == null || stateClient == null) return;
            Vector3d currentEast;
            Vector3d currentNorth;
            Vector3d currentUp;
            TangentBasis(vessel.latitude, vessel.longitude, out currentEast, out currentNorth, out currentUp);
            var bodyFixedPosition = GeodeticPosition(vessel.latitude, vessel.longitude, vessel.altitude, anchorBody.Radius);
            var delta = bodyFixedPosition - anchorPosition;
            var position = ProjectToAnchor(delta);

            var linearVelocity = WorldVectorToAnchor(vessel.srf_velocity, currentEast, currentNorth, currentUp);
            var angularVelocity = WorldVectorToAnchor(vessel.angularVelocity, currentEast, currentNorth, currentUp);
            var now = Planetarium.GetUniversalTime();
            var elapsed = Math.Max(0.0001, now - previousTruthTime);
            var linearAcceleration = derivativeReady
                ? (linearVelocity - previousLinearVelocity) / (float)elapsed
                : Vector3.zero;
            var angularAcceleration = derivativeReady
                ? (angularVelocity - previousAngularVelocity) / (float)elapsed
                : Vector3.zero;
            derivativeReady = true;
            previousTruthTime = now;
            previousLinearVelocity = linearVelocity;
            previousAngularVelocity = angularVelocity;

            var bodyX = WorldDirectionToAnchor(BodyForwardWorld(), currentEast, currentNorth, currentUp);
            var bodyY = WorldDirectionToAnchor(BodyLeftWorld(), currentEast, currentNorth, currentUp);
            var bodyZ = WorldDirectionToAnchor(BodyUpWorld(), currentEast, currentNorth, currentUp);
            var rotation = QuaternionFromBasis(bodyX, bodyY, bodyZ);

            var builder = new StringBuilder(768);
            builder.Append('{');
            AppendString(builder, "type", "ksp_ground_truth", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", vessel.id.ToString("N"));
            AppendString(builder, "vessel", vessel.vesselName);
            AppendNumber(builder, "originSequence", originSequence);
            AppendNumber(builder, "universalTime", Planetarium.GetUniversalTime());
            AppendVector(builder, "position", position);
            AppendQuaternion(builder, "rotation", rotation);
            AppendVector(builder, "linearVelocity", linearVelocity);
            AppendVector(builder, "angularVelocity", angularVelocity);
            AppendVector(builder, "linearAcceleration", linearAcceleration);
            AppendVector(builder, "angularAcceleration", angularAcceleration);
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendActuatorStates()
        {
            foreach (var wheelBase in WheelBases()) SendWheelState(wheelBase);
            foreach (var engine in EngineModules()) SendEngineState(engine);
            foreach (var rcs in RcsModules()) SendRcsState(rcs);
            foreach (var separation in SeparationModules()) SendSeparationState(separation);
        }

        private void SendActuatorManifest()
        {
            if (vessel == null) return;
            var builder = new StringBuilder(1024);
            builder.Append('{');
            AppendString(builder, "type", "ksp_actuator_manifest", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", vessel.id.ToString("N"));
            Prefix(builder, "actuators", false);
            builder.Append('[');
            var first = true;
            foreach (var wheel in WheelBases())
            {
                AppendManifestActuator(builder, "wheel", KerbalRosActuatorNames.For(
                    "wheel", wheel.part, KerbalRosActuatorNames.ModuleIndex(wheel.part, wheel)), ref first);
            }
            foreach (var engine in EngineModules())
            {
                AppendManifestActuator(builder, "engine", KerbalRosActuatorNames.For(
                    "engine", engine.part, KerbalRosActuatorNames.ModuleIndex(engine.part, engine)), ref first);
            }
            foreach (var rcs in RcsModules())
            {
                AppendManifestActuator(builder, "rcs", KerbalRosActuatorNames.For(
                    "rcs", rcs.part, KerbalRosActuatorNames.ModuleIndex(rcs.part, rcs)), ref first);
            }
            foreach (var motor in MotorModules())
            {
                AppendManifestActuator(builder, "motor", motor.JointName, ref first);
            }
            foreach (var separation in SeparationModules())
            {
                var mechanism = SeparationMechanism(separation);
                AppendManifestActuator(builder, "separation", KerbalRosActuatorNames.For(
                    mechanism, separation.part,
                    KerbalRosActuatorNames.ModuleIndex(separation.part, separation)), ref first);
            }
            builder.Append(']').Append('}');
            Send(builder.ToString());
        }

        private static void AppendManifestActuator(
            StringBuilder builder, string kind, string name, ref bool first)
        {
            if (!first) builder.Append(',');
            first = false;
            builder.Append('{');
            AppendString(builder, "actuatorType", kind, true);
            AppendString(builder, "name", name);
            builder.Append('}');
        }

        private void SendWheelState(ModuleWheelBase wheelBase)
        {
            var controller = wheelBase.Wheel;
            if (controller == null || controller.currentState == null) return;
            var state = controller.currentState;
            var motor = wheelBase.part.FindModuleImplementing<ModuleWheelMotor>();
            var name = KerbalRosActuatorNames.For("wheel", wheelBase.part,
                KerbalRosActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
            var builder = BeginActuatorState("wheel", name, wheelBase.part);
            AppendBoolean(builder, "enabled", motor != null && motor.motorEnabled);
            AppendBoolean(builder, "grounded", state.grounded);
            AppendNumber(builder, "angularPosition", controller.wheelCollider == null ? 0f : controller.wheelCollider.angularPosition);
            AppendNumber(builder, "angularVelocity", state.angularVelocity);
            AppendNumber(builder, "steeringAngle", state.steerAngle * Mathf.Deg2Rad);
            AppendNumber(builder, "driveTorque", state.driveTorque);
            AppendNumber(builder, "brakeTorque", state.brakeTorque);
            AppendNumber(builder, "slip", state.combinedTireSlip);
            AppendNumber(builder, "maxDriveTorque", controller.maxDriveTorque);
            AppendBoolean(builder, "commandActive", wheelOverrides.ContainsKey(name));
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendEngineState(ModuleEngines engine)
        {
            var name = KerbalRosActuatorNames.For("engine", engine.part,
                KerbalRosActuatorNames.ModuleIndex(engine.part, engine));
            var builder = BeginActuatorState("engine", name, engine.part);
            AppendBoolean(builder, "enabled", engine.EngineIgnited);
            AppendBoolean(builder, "operational", engine.isOperational);
            AppendBoolean(builder, "flameout", engine.flameout);
            AppendNumber(builder, "throttle", engine.currentThrottle);
            AppendNumber(builder, "thrust", engine.GetCurrentThrust());
            AppendNumber(builder, "maxThrust", engine.GetMaxThrust());
            AppendBoolean(builder, "commandActive", engineOverrides.ContainsKey(name));
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendRcsState(ModuleRCS rcs)
        {
            var name = KerbalRosActuatorNames.For("rcs", rcs.part,
                KerbalRosActuatorNames.ModuleIndex(rcs.part, rcs));
            var thrust = 0f;
            if (rcs.thrustForces != null)
            {
                for (var index = 0; index < rcs.thrustForces.Length; index++) thrust += Mathf.Abs(rcs.thrustForces[index]);
            }
            var nozzleCount = rcs.thrusterTransforms == null ? 1 : Math.Max(1, rcs.thrusterTransforms.Count);
            var builder = BeginActuatorState("rcs", name, rcs.part);
            AppendBoolean(builder, "enabled", rcs.rcsEnabled);
            AppendBoolean(builder, "active", rcs.rcs_active);
            AppendBoolean(builder, "flameout", rcs.flameout);
            AppendNumber(builder, "thrust", thrust);
            AppendNumber(builder, "maxThrust", rcs.thrusterPower * nozzleCount);
            AppendNumber(builder, "thrustLimit", rcs.thrusterPower * Mathf.Clamp01(rcs.thrustPercentage / 100f));
            AppendBoolean(builder, "commandActive", rcsOverrides.ContainsKey(name));
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendSeparationState(PartModule module)
        {
            if (module == null || module.part == null)
            {
                return;
            }
            var mechanism = SeparationMechanism(module);
            if (string.IsNullOrEmpty(mechanism))
            {
                return;
            }
            var name = KerbalRosActuatorNames.For(
                mechanism, module.part, KerbalRosActuatorNames.ModuleIndex(module.part, module));
            var builder = BeginActuatorState("separation", name, module.part);
            AppendString(builder, "mechanism", mechanism);
            AppendBoolean(builder, "available", SeparationAvailable(module));
            AppendBoolean(builder, "separated", SeparationComplete(module));
            builder.Append('}');
            Send(builder.ToString());
        }

        private static string SeparationMechanism(PartModule module)
        {
            if (module is ModuleDecouplerBase) return "decoupler";
            if (module is ModuleProceduralFairing) return "fairing";
            return string.Empty;
        }

        private static bool SeparationAvailable(PartModule module)
        {
            var decoupler = module as ModuleDecouplerBase;
            if (decoupler != null)
            {
                var decoupleEvent = decoupler.Events != null && decoupler.Events.Contains("Decouple")
                    ? decoupler.Events["Decouple"]
                    : null;
                return !decoupler.isDecoupled && (decoupleEvent == null || decoupleEvent.active);
            }
            var fairing = module as ModuleProceduralFairing;
            var deployEvent = FairingDeployEvent(fairing);
            return deployEvent != null && deployEvent.active;
        }

        private static bool SeparationComplete(PartModule module)
        {
            var decoupler = module as ModuleDecouplerBase;
            if (decoupler != null)
            {
                return decoupler.isDecoupled;
            }
            var fairing = module as ModuleProceduralFairing;
            var deployEvent = FairingDeployEvent(fairing);
            return fairing != null && deployEvent != null && !deployEvent.active;
        }

        private static BaseEvent FairingDeployEvent(ModuleProceduralFairing fairing)
        {
            return fairing == null || fairing.Events == null || !fairing.Events.Contains("DeployFairing")
                ? null
                : fairing.Events["DeployFairing"];
        }

        private void SendWrenchStatus(Vector3 residualForce, Vector3 residualTorque)
        {
            var builder = new StringBuilder(384);
            builder.Append('{');
            AppendString(builder, "type", "ksp_wrench_status", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendVector(builder, "requestedForce", requestedForce);
            AppendVector(builder, "requestedTorque", requestedTorque);
            AppendVector(builder, "residualForce", residualForce);
            AppendVector(builder, "residualTorque", residualTorque);
            builder.Append('}');
            Send(builder.ToString());
        }

        private StringBuilder BeginActuatorState(string kind, string name, Part targetPart)
        {
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "ksp_actuator_state", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "actuatorType", kind);
            AppendString(builder, "name", name);
            AppendString(builder, "vessel", vessel == null ? string.Empty : vessel.vesselName);
            AppendNumber(builder, "partFlightId", targetPart == null ? 0u : targetPart.flightID);
            AppendNumber(builder, "persistentId", targetPart == null ? 0u : targetPart.persistentId);
            AppendNumber(builder, "universalTime", Planetarium.GetUniversalTime());
            return builder;
        }

        private IEnumerable<ModuleWheelBase> WheelBases()
        {
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var targetPart in vessel.parts)
            {
                if (targetPart == null) continue;
                foreach (PartModule module in targetPart.Modules)
                {
                    var wheel = module as ModuleWheelBase;
                    if (wheel != null) yield return wheel;
                }
            }
        }

        private IEnumerable<ModuleEngines> EngineModules()
        {
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var targetPart in vessel.parts)
            {
                if (targetPart == null) continue;
                foreach (PartModule module in targetPart.Modules)
                {
                    var engine = module as ModuleEngines;
                    if (engine != null) yield return engine;
                }
            }
        }

        private IEnumerable<ModuleRCS> RcsModules()
        {
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var targetPart in vessel.parts)
            {
                if (targetPart == null) continue;
                foreach (PartModule module in targetPart.Modules)
                {
                    var rcs = module as ModuleRCS;
                    if (rcs != null) yield return rcs;
                }
            }
        }

        private IEnumerable<IKerbalRosMotor> MotorModules()
        {
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var targetPart in vessel.parts)
            {
                if (targetPart == null) continue;
                foreach (PartModule module in targetPart.Modules)
                {
                    var motor = module as IKerbalRosMotor;
                    if (motor != null) yield return motor;
                }
            }
        }

        private IEnumerable<PartModule> SeparationModules()
        {
            if (vessel == null || vessel.parts == null) yield break;
            foreach (var targetPart in vessel.parts)
            {
                if (targetPart == null) continue;
                foreach (PartModule module in targetPart.Modules)
                {
                    if (module is ModuleDecouplerBase || module is ModuleProceduralFairing)
                    {
                        yield return module;
                    }
                }
            }
        }

        private Vector3 BodyForwardWorld()
        {
            var reference = vessel == null ? null : vessel.ReferenceTransform;
            return reference == null ? Vector3.forward : reference.up.normalized;
        }

        private Vector3 BodyLeftWorld()
        {
            var reference = vessel == null ? null : vessel.ReferenceTransform;
            return reference == null ? Vector3.left : (-reference.right).normalized;
        }

        private Vector3 BodyUpWorld()
        {
            var reference = vessel == null ? null : vessel.ReferenceTransform;
            return reference == null ? Vector3.up : (-reference.forward).normalized;
        }

        private Vector3 WorldVectorToBody(Vector3 world)
        {
            return new Vector3(
                Vector3.Dot(world, BodyForwardWorld()),
                Vector3.Dot(world, BodyLeftWorld()),
                Vector3.Dot(world, BodyUpWorld()));
        }

        private static Vector3d GeodeticPosition(double latitudeDegrees, double longitudeDegrees, double altitude, double radius)
        {
            var latitude = latitudeDegrees * Math.PI / 180.0;
            var longitude = longitudeDegrees * Math.PI / 180.0;
            var distance = radius + altitude;
            var cosLatitude = Math.Cos(latitude);
            return new Vector3d(
                distance * cosLatitude * Math.Cos(longitude),
                distance * cosLatitude * Math.Sin(longitude),
                distance * Math.Sin(latitude));
        }

        private static void TangentBasis(double latitudeDegrees, double longitudeDegrees,
            out Vector3d east, out Vector3d north, out Vector3d up)
        {
            var latitude = latitudeDegrees * Math.PI / 180.0;
            var longitude = longitudeDegrees * Math.PI / 180.0;
            east = new Vector3d(-Math.Sin(longitude), Math.Cos(longitude), 0.0);
            north = new Vector3d(
                -Math.Sin(latitude) * Math.Cos(longitude),
                -Math.Sin(latitude) * Math.Sin(longitude),
                Math.Cos(latitude));
            up = new Vector3d(
                Math.Cos(latitude) * Math.Cos(longitude),
                Math.Cos(latitude) * Math.Sin(longitude),
                Math.Sin(latitude));
        }

        private Vector3 ProjectToAnchor(Vector3d value)
        {
            return new Vector3((float)Vector3d.Dot(value, anchorEast),
                (float)Vector3d.Dot(value, anchorNorth), (float)Vector3d.Dot(value, anchorUp));
        }

        private Vector3 WorldVectorToAnchor(Vector3d world, Vector3d currentEast, Vector3d currentNorth, Vector3d currentUp)
        {
            var eastWorld = vessel.east.normalized;
            var northWorld = vessel.north.normalized;
            var upWorld = vessel.upAxis.normalized;
            var bodyFixed = currentEast * Vector3d.Dot(world, eastWorld) +
                            currentNorth * Vector3d.Dot(world, northWorld) +
                            currentUp * Vector3d.Dot(world, upWorld);
            return ProjectToAnchor(bodyFixed);
        }

        private Vector3 WorldDirectionToAnchor(Vector3 world, Vector3d currentEast, Vector3d currentNorth, Vector3d currentUp)
        {
            return WorldVectorToAnchor(new Vector3d(world.x, world.y, world.z), currentEast, currentNorth, currentUp).normalized;
        }

        private static Quaternion QuaternionFromBasis(Vector3 x, Vector3 y, Vector3 z)
        {
            var m00 = x.x; var m01 = y.x; var m02 = z.x;
            var m10 = x.y; var m11 = y.y; var m12 = z.y;
            var m20 = x.z; var m21 = y.z; var m22 = z.z;
            var trace = m00 + m11 + m22;
            float qx, qy, qz, qw;
            if (trace > 0f)
            {
                var s = Mathf.Sqrt(trace + 1f) * 2f;
                qw = 0.25f * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                var s = Mathf.Sqrt(1f + m00 - m11 - m22) * 2f;
                qw = (m21 - m12) / s; qx = 0.25f * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                var s = Mathf.Sqrt(1f + m11 - m00 - m22) * 2f;
                qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25f * s; qz = (m12 + m21) / s;
            }
            else
            {
                var s = Mathf.Sqrt(1f + m22 - m00 - m11) * 2f;
                qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25f * s;
            }
            return new Quaternion(qx, qy, qz, qw).normalized;
        }

        private void Send(string json)
        {
            if (stateClient == null || stateEndpoint == null) return;
            try
            {
                var bytes = Encoding.UTF8.GetBytes(json);
                stateClient.Send(bytes, bytes.Length, stateEndpoint);
            }
            catch (Exception exception)
            {
                Warn("Vehicle state UDP send failed: " + exception.Message);
            }
        }

        private void Warn(string message)
        {
            if (Time.realtimeSinceStartup - lastWarningTime < 5f) return;
            lastWarningTime = Time.realtimeSinceStartup;
            Debug.LogWarning("[KerbalLiDAR] " + message);
        }

        private static bool VectorIsFinite(double[] values)
        {
            return values != null && values.Length >= 3 &&
                   IsFinite(values[0]) && IsFinite(values[1]) && IsFinite(values[2]);
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        private static void Prefix(StringBuilder builder, string name, bool first)
        {
            if (!first) builder.Append(',');
            JsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendString(StringBuilder builder, string name, string value, bool first = false)
        {
            Prefix(builder, name, first);
            JsonString(builder, value);
        }

        private static void AppendNumber(StringBuilder builder, string name, double value)
        {
            Prefix(builder, name, false);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendBoolean(StringBuilder builder, string name, bool value)
        {
            Prefix(builder, name, false);
            builder.Append(value ? "true" : "false");
        }

        private static void AppendVector(StringBuilder builder, string name, Vector3 value)
        {
            Prefix(builder, name, false);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }

        private static void AppendQuaternion(StringBuilder builder, string name, Quaternion value)
        {
            Prefix(builder, name, false);
            builder.Append('[').Append(value.x.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.y.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.z.ToString("R", CultureInfo.InvariantCulture)).Append(',')
                .Append(value.w.ToString("R", CultureInfo.InvariantCulture)).Append(']');
        }

        private static void JsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            var source = value ?? string.Empty;
            for (var index = 0; index < source.Length; index++)
            {
                var character = source[index];
                if (character == '\\') builder.Append("\\\\");
                else if (character == '"') builder.Append("\\\"");
                else if (character == '\n') builder.Append("\\n");
                else if (character == '\r') builder.Append("\\r");
                else builder.Append(character);
            }
            builder.Append('"');
        }
    }
}
