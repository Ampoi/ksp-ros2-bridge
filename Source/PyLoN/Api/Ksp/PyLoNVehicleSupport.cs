using static PyLoN.JsonPacketWriter;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using PyLoN.Application.Control;
using PyLoN.Domain.Control;
using ModuleWheels;
using UnityEngine;

namespace PyLoN
{
    [Serializable]
    public sealed class PyLoNBodyWrenchCommand
    {
        public string type;
        public int version;
        public string frame;
        public string vesselId;
        public string controllerId;
        public string leaseId;
        public double[] force;
        public double[] torque;
        public double timeoutSeconds;
        public long sequence;
    }

    [Serializable]
    public sealed class PyLoNActuatorCommand
    {
        public string type;
        public int version;
        public string actuatorType;
        public string name;
        public string vesselId;
        public string controllerId;
        public string leaseId;
        public bool enabled;
        public double timeoutSeconds;
        public long sequence;
        public double targetAngularVelocity;
        public double steeringAngle;
        public double maxDriveTorque;
        public double brake;
        public double targetThrust;
        public bool hasGimbalCommand;
        public double gimbalPitch;
        public double gimbalYaw;
        public double gimbalRoll;
        public double thrustLimit;
        public bool separate;
    }

    [Serializable]
    public sealed class PyLoNControlAuthorityCommand
    {
        public string type;
        public int version;
        public string action;
        public string vesselId;
        public string controllerId;
        public string leaseId;
        public int priority;
        public double leaseDurationSeconds;
        public bool suppressSas;
        public long sequence;
    }

    internal static class PyLoNActuatorNames
    {
        internal static string For(string kind, Part part, int moduleIndex)
        {
            var id = part == null ? 0u : part.persistentId;
            if (id == 0u && part != null)
            {
                id = part.flightID;
            }
            return PyLoNIdentityScenario.Resolve(kind, part, moduleIndex, PyLoNMotorNames.Sanitize(
                kind + "_" + id.ToString(CultureInfo.InvariantCulture) + "_" +
                Math.Max(0, moduleIndex).ToString(CultureInfo.InvariantCulture),
                kind));
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
    public sealed partial class PyLoNVehicleManager : MonoBehaviour
    {
        private const int ProtocolVersion = 1;
        private const int ControlProtocolVersion = 1;
        private static int CommandPort { get { return RuntimeSettings.CommandPort; } }
        private static int StatePort { get { return RuntimeSettings.StatePort; } }
        private const float StateRateHz = 30f;
        private const float DefaultTimeout = 0.5f;
        // KSP's flight dynamics use tonnes, kN and kN*m.  The ROS API is SI:
        // kilograms, N and N*m.  Keep the conversion at this adapter boundary
        // so the domain allocator and every wire message remain unambiguous.
        private const float KspForceUnitInNewtons = FrameConversions.KilonewtonsToNewtons;
        private const float NewtonsToKspForceUnit = 1f / KspForceUnitInNewtons;

        private sealed class WheelOverride
        {
            public bool Enabled;
            public float TargetAngularVelocity;
            public float Brake;
            public float SteeringAngleDegrees;
            public float MaxDriveTorque;
            public float ExpiresAt;
        }

        private sealed class EngineOverride
        {
            public bool Enabled;
            public float TargetThrust;
            public bool HasGimbalCommand;
            public Vector3 GimbalInput;
            public long Sequence;
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
            public bool SteeringComponentEnabled;
            public float MaxDriveTorque;
        }

        private struct EngineChannel
        {
            public ModuleEngines Engine;
            public Vector3 Force;
            public Vector3 Torque;
            public float MaximumThrust;
        }

        private static PyLoNVehicleManager instance;

        internal static bool ExclusiveControlActive
        {
            get
            {
                return instance != null && instance.control != null &&
                    instance.control.Authority.Mode != ControlAuthorityMode.Unowned;
            }
        }

        internal static bool TryAcceptExclusiveCommand(
            string vesselId,
            string controllerId,
            string leaseId,
            long sequence,
            out string reason)
        {
            if (instance == null || instance.control == null || !instance.TargetsActiveVessel(vesselId))
            {
                reason = "active_vessel_mismatch";
                return false;
            }
            return instance.control.Authority.AcceptCommand(
                controllerId, leaseId, Time.realtimeSinceStartup, sequence, out reason);
        }
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
        private VesselTelemetry telemetry;
        private float nextStateTime;
        private float nextManifestTime;
        private float nextAuthorityStateTime;
        private bool wrenchActive;
        private Vector3 requestedForce;
        private Vector3 requestedTorque;
        private float wrenchExpiresAt;
        private long activeWrenchSequence;
        private string activeWrenchControllerId = string.Empty;
        private string activeWrenchLeaseId = string.Empty;
        private WrenchValue rawRequestedWrench;
        private WrenchValue requestedWrench;
        private WrenchValue allocatedWrench;
        private bool wrenchFeedbackPending;
        private string wrenchReason = "idle";
        private bool rcsActionGroupOverridden;
        private bool rcsActionGroupWasEnabled;
        private bool sasOverrideActive;
        private bool sasWasEnabled;
        private VehicleControlApplication control;
        private VesselParts parts;
        private ActuatorTelemetry actuatorTelemetry;
        private float lastWarningTime = -1000f;

        private void OnRuntimeSessionChanged()
        {
            if (control == null) return;
            StopAllVehicleControl();
            control.Authority.ReleaseForLifecycleChange("runtime_session_changed");
            if (telemetry != null) telemetry.ResetGroundTruthOrigin();
            nextManifestTime = 0f;
            if (actuatorTelemetry != null) actuatorTelemetry.Reset();
        }

        public void Start()
        {
            parts = new VesselParts(() => vessel);
            actuatorTelemetry = new ActuatorTelemetry(() => vessel, parts, Send,
                (kind, name) => kind == "wheel" ? wheelOverrides.ContainsKey(name) :
                    kind == "engine" ? engineOverrides.ContainsKey(name) : rcsOverrides.ContainsKey(name), AppendGimbalState);
            RuntimeSession.Changed += OnRuntimeSessionChanged;
            instance = this;
            telemetry = new VesselTelemetry(() => vessel, Send);
            control = new VehicleControlApplication(LoadSafetyPolicy());
            control.ResetSafety(Time.realtimeSinceStartup);
            stateClient = new UdpClient();
            stateEndpoint = new IPEndPoint(Dns.GetHostAddresses(RuntimeSettings.StateHost)[0], StatePort);
            AttachToActiveVessel();
        }

        public void Update()
        {
            AttachToActiveVessel();
            ExpireCommands();
            if (wrenchFeedbackPending)
            {
                wrenchFeedbackPending = false;
                SendWrenchStatus(allocatedWrench, MeasurePropulsionWrench(), wrenchReason);
            }
            if (control != null &&
                (control.Authority.Mode == ControlAuthorityMode.EmergencyStop ||
                 (control.Authority.Mode == ControlAuthorityMode.Owned &&
                  control.Authority.SuppressSas)))
            {
                // Ownership is a mode, not a one-shot toggle. Keep SAS
                // suppressed if the player or another mod tries to re-enable it.
                AcquireSasOverride();
            }
            ApplyDirectWheelCommands();
            if (Time.realtimeSinceStartup >= nextAuthorityStateTime)
            {
                SendControlAuthorityState(control == null ? "initializing" : control.Authority.Reason);
                nextAuthorityStateTime = Time.realtimeSinceStartup + 0.1f;
            }
            if (vessel == null || Time.realtimeSinceStartup < nextStateTime)
            {
                return;
            }
            nextStateTime = Time.realtimeSinceStartup + 1f / StateRateHz;
            if (Time.realtimeSinceStartup >= nextManifestTime)
            {
                actuatorTelemetry.PublishManifest();
                nextManifestTime = Time.realtimeSinceStartup + 1f;
            }
            telemetry.SendGroundTruth();
            actuatorTelemetry.PublishStates();
        }

        public void OnDestroy()
        {
            RuntimeSession.Changed -= OnRuntimeSessionChanged;
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

        // Called only after CommandDispatcher has accepted the session envelope.
        internal static bool TryDispatch(string json, PyLoNCommandEnvelope envelope, int port)
        {
            if (envelope == null ||
                (envelope.type != "pylon_body_wrench_command" &&
                 envelope.type != "pylon_actuator_command" &&
                 envelope.type != "pylon_control_authority_command"))
            {
                return false;
            }
            if (port != CommandPort || instance == null)
            {
                return true;
            }
            try
            {
                if (envelope.type == "pylon_control_authority_command" && envelope.version == ControlProtocolVersion)
                {
                    instance.ApplyControlAuthority(JsonUtility.FromJson<PyLoNControlAuthorityCommand>(json));
                }
                else if (envelope.type == "pylon_body_wrench_command" && envelope.version == ControlProtocolVersion)
                {
                    instance.ApplyBodyWrench(JsonUtility.FromJson<PyLoNBodyWrenchCommand>(json));
                }
                else if (envelope.type == "pylon_actuator_command" && envelope.version == ControlProtocolVersion)
                {
                    instance.ApplyActuatorCommand(JsonUtility.FromJson<PyLoNActuatorCommand>(json));
                }
                else
                {
                    instance.SendRejectedWrench(0, "unsupported_control_protocol");
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
            if (active == vessel && (active == null || active.mainBody == telemetry.anchorBody))
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
            telemetry.ResetGroundTruthOrigin();
            nextManifestTime = 0f;
            if (actuatorTelemetry != null) actuatorTelemetry.Reset();
            if (control != null && control.Authority.Mode == ControlAuthorityMode.EmergencyStop)
            {

                AcquireSasOverride();
            }
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
            RestoreRcsActionGroup();
            RestoreSasState();
            wheelOverrides.Clear();
            RestoreGimbals(null);
            engineOverrides.Clear();
            rcsOverrides.Clear();
            wrenchActive = false;
            requestedForce = Vector3.zero;
            requestedTorque = Vector3.zero;
            requestedWrench = WrenchValue.Zero;
            rawRequestedWrench = WrenchValue.Zero;
            allocatedWrench = WrenchValue.Zero;
            wrenchFeedbackPending = false;
            activeWrenchSequence = 0;
            activeWrenchControllerId = string.Empty;
            activeWrenchLeaseId = string.Empty;
            if (control != null)
            {
                control.Authority.ReleaseForLifecycleChange("active_vessel_changed");
                control.ResetSafety(Time.realtimeSinceStartup);
            }
            vessel = null;
            telemetry.anchorBody = null;
            telemetry.derivativeReady = false;
        }



        private void ApplyControlAuthority(PyLoNControlAuthorityCommand command)
        {
            if (command == null || control == null)
            {
                return;
            }
            if (!TargetsActiveVessel(command.vesselId))
            {
                SendControlAuthorityState("active_vessel_mismatch");
                return;
            }

            var now = Time.realtimeSinceStartup;
            var action = (command.action ?? string.Empty).Trim().ToLowerInvariant();
            var previousMode = control.Authority.Mode;
            var previousControllerId = control.Authority.ControllerId;
            var previousLeaseId = control.Authority.LeaseId;
            var accepted = false;
            string reason;
            if (action == "acquire")
            {
                accepted = control.Authority.Acquire(
                    command.controllerId, command.leaseId, command.priority, now,
                    command.leaseDurationSeconds, command.suppressSas,
                    command.sequence, out reason);
            }
            else if (action == "renew")
            {
                accepted = control.Authority.Renew(
                    command.controllerId, command.leaseId, now,
                    command.leaseDurationSeconds, command.suppressSas,
                    command.sequence, out reason);
            }
            else if (action == "release")
            {
                accepted = control.Authority.Release(
                    command.controllerId, command.leaseId, command.sequence, out reason);
            }
            else if (action == "emergency_stop")
            {
                accepted = control.Authority.EmergencyStop(
                    command.controllerId, command.leaseId, command.sequence,
                    "emergency_stop", out reason);
            }
            else if (action == "clear_emergency_stop")
            {
                accepted = control.Authority.ClearEmergencyStop(
                    command.controllerId, command.leaseId, command.sequence, out reason);
            }
            else
            {
                reason = "unsupported_authority_action";
            }

            if (accepted && control.Authority.Mode == ControlAuthorityMode.Owned)
            {
                var ownerChanged = previousMode != ControlAuthorityMode.Owned ||
                    previousControllerId != control.Authority.ControllerId ||
                    previousLeaseId != control.Authority.LeaseId;
                if (ownerChanged)
                {
                    StopAllVehicleControl();
                }

                if (control.Authority.SuppressSas)
                {
                    AcquireSasOverride();
                }
                else
                {
                    RestoreSasState();
                }
            }
            else if (accepted && control.Authority.Mode == ControlAuthorityMode.EmergencyStop)
            {
                StopAllVehicleControl();

                AcquireSasOverride();
            }
            else if (accepted)
            {
                StopAllVehicleControl();
            }
            SendControlAuthorityState(reason);
        }

        private void ApplyBodyWrench(PyLoNBodyWrenchCommand command)
        {
            if (command == null)
            {
                SendRejectedWrench(0, "invalid_wrench", WrenchValue.Zero, string.Empty, string.Empty);
                return;
            }
            if (!string.Equals(command.frame, "base_link", StringComparison.Ordinal))
            {
                SendRejectedWrench(
                    command.sequence, "unsupported_wrench_frame", WrenchValue.Zero,
                    command.controllerId, command.leaseId);
                return;
            }
            if (!VectorIsFinite(command.force) || !VectorIsFinite(command.torque))
            {
                SendRejectedWrench(
                    command.sequence, "invalid_wrench", WrenchValue.Zero,
                    command.controllerId, command.leaseId);
                return;
            }
            var raw = new WrenchValue(
                new Vector3Value(command.force[0], command.force[1], command.force[2]),
                new Vector3Value(command.torque[0], command.torque[1], command.torque[2]));
            if (!TargetsActiveVessel(command.vesselId))
            {
                SendRejectedWrench(
                    command.sequence, "active_vessel_mismatch", raw,
                    command.controllerId, command.leaseId);
                return;
            }
            var rejectionReason = "control_unavailable";
            var filtered = control == null ? null : control.AcceptWrench(
                command.controllerId,
                command.leaseId,
                command.sequence,
                Time.realtimeSinceStartup,
                raw,
                ToDomain(VesselAngularVelocityBody()),
                out rejectionReason);
            if (filtered == null)
            {
                SendRejectedWrench(
                    command.sequence, rejectionReason, raw,
                    command.controllerId, command.leaseId);
                return;
            }
            rawRequestedWrench = raw;
            requestedWrench = filtered.Wrench;
            requestedForce = ToUnity(requestedWrench.Force);
            requestedTorque = ToUnity(requestedWrench.Torque);
            wrenchReason = filtered.Reason;
            activeWrenchSequence = command.sequence;
            activeWrenchControllerId = command.controllerId ?? string.Empty;
            activeWrenchLeaseId = command.leaseId ?? string.Empty;
            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : DefaultTimeout;
            wrenchExpiresAt = Time.realtimeSinceStartup + timeout;
            wrenchActive = true;
        }

        private void ApplyActuatorCommand(PyLoNActuatorCommand command)
        {
            if (command == null || string.IsNullOrEmpty(command.name))
            {
                return;
            }
            if (!TargetsActiveVessel(command.vesselId))
            {
                return;
            }
            var rejectionReason = "control_unavailable";
            if (control == null || !control.Authority.AcceptCommand(
                command.controllerId, command.leaseId, Time.realtimeSinceStartup,
                command.sequence, out rejectionReason))
            {
                SendControlAuthorityState(rejectionReason);
                return;
            }
            var name = PyLoNMotorNames.Sanitize(command.name, "actuator");
            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : DefaultTimeout;
            var expiresAt = Time.realtimeSinceStartup + timeout;
            switch ((command.actuatorType ?? string.Empty).Trim().ToLowerInvariant())
            {
                case "wheel":
                    if (!IsFinite(command.targetAngularVelocity) || !IsFinite(command.steeringAngle) ||
                        !IsFinite(command.maxDriveTorque) || !IsFinite(command.brake) ||
                        command.brake < 0.0 || command.brake > 1.0) return;
                    wheelOverrides[name] = new WheelOverride
                    {
                        Enabled = command.enabled,
                        Brake = (float)command.brake,
                        TargetAngularVelocity = (float)command.targetAngularVelocity,
                        SteeringAngleDegrees = (float)(command.steeringAngle * Mathf.Rad2Deg),
                        MaxDriveTorque = Mathf.Max(
                            0f, (float)command.maxDriveTorque * NewtonsToKspForceUnit),
                        ExpiresAt = expiresAt
                    };
                    break;
                case "engine":
                    if (!IsFinite(command.targetThrust) ||
                        !ValidGimbalAxis(command.gimbalPitch) ||
                        !ValidGimbalAxis(command.gimbalYaw) ||
                        !ValidGimbalAxis(command.gimbalRoll)) return;
                    if (!command.hasGimbalCommand) RestoreGimbals(name);
                    engineOverrides[name] = new EngineOverride
                    {
                        Enabled = command.enabled,
                        HasGimbalCommand = command.hasGimbalCommand,
                        GimbalInput = new Vector3((float)command.gimbalPitch,
                            (float)command.gimbalRoll, (float)command.gimbalYaw),
                        Sequence = command.sequence,
                        TargetThrust = Mathf.Max(
                            0f, (float)command.targetThrust * NewtonsToKspForceUnit),
                        ExpiresAt = expiresAt
                    };
                    break;
                case "rcs":
                    if (!IsFinite(command.thrustLimit)) return;
                    rcsOverrides[name] = new RcsOverride
                    {
                        Enabled = command.enabled,
                        ThrustLimit = Mathf.Max(
                            0f, (float)command.thrustLimit * NewtonsToKspForceUnit),
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
            foreach (var module in parts.Separations())
            {
                var mechanism = VesselParts.SeparationMechanism(module);
                var targetName = PyLoNActuatorNames.For(
                    mechanism, module.part, PyLoNActuatorNames.ModuleIndex(module.part, module));
                if (targetName != name || !VesselParts.SeparationAvailable(module))
                {
                    continue;
                }
                actuatorTelemetry.PublishSeparation(module);
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
                actuatorTelemetry.PublishSeparation(module);
                nextManifestTime = 0f;
            if (actuatorTelemetry != null) actuatorTelemetry.Reset();
                return;
            }
        }

        private void ExpireCommands()
        {
            var now = Time.realtimeSinceStartup;
            if (control != null && control.Authority.Expire(now))
            {
                StopAllVehicleControl();
                SendControlAuthorityState("lease_expired");
            }
            if (wrenchActive && now > wrenchExpiresAt)
            {
                wrenchActive = false;
                requestedForce = Vector3.zero;
                requestedTorque = Vector3.zero;
                requestedWrench = WrenchValue.Zero;
                rawRequestedWrench = WrenchValue.Zero;
                allocatedWrench = WrenchValue.Zero;
                wrenchFeedbackPending = false;
                wrenchReason = "command_timeout";
                RestoreBodyEngineStates();
                RestoreRcsActionGroup();
                SendWrenchStatus(WrenchValue.Zero, MeasurePropulsionWrench(), "command_timeout");
            }
            ExpireWheelOverrides(now);
            ExpireEngineOverrides(now);
            ExpireRcsOverrides(now);
        }

        private void AcquireSasOverride()
        {
            if (vessel == null)
            {
                return;
            }
            if (!sasOverrideActive)
            {
                sasWasEnabled = vessel.ActionGroups[KSPActionGroup.SAS];
                sasOverrideActive = true;
            }
            vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
        }

        private void RestoreSasState()
        {
            if (!sasOverrideActive)
            {
                return;
            }
            if (vessel != null)
            {
                vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, sasWasEnabled);
            }
            sasOverrideActive = false;
            sasWasEnabled = false;
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
                ParkRover();
                foreach (var wheel in parts.Get<ModuleWheelBase>())
                {
                    if (PyLoNActuatorNames.For("wheel", wheel.part,
                        PyLoNActuatorNames.ModuleIndex(wheel.part, wheel)) != name) continue;
                    OriginalWheelState original;
                    if (directWheelStates.TryGetValue(wheel, out original))
                    {
                        var motor = wheel.part.FindModuleImplementing<ModuleWheelMotor>();
                        if (motor != null) motor.motorEnabled = original.MotorEnabled;
                        var steering = wheel.part.FindModuleImplementing<ModuleWheelSteering>();
                        if (steering != null) steering.enabled = original.SteeringComponentEnabled;
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
                RestoreGimbals(name);
                if (wrenchActive) continue;
                foreach (var engine in parts.Get<ModuleEngines>())
                {
                    if (PyLoNActuatorNames.For("engine", engine.part,
                        PyLoNActuatorNames.ModuleIndex(engine.part, engine)) != name) continue;
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
                foreach (var rcs in parts.Get<ModuleRCS>())
                {
                    if (PyLoNActuatorNames.For("rcs", rcs.part,
                        PyLoNActuatorNames.ModuleIndex(rcs.part, rcs)) != name) continue;
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
            if (control != null && control.Authority.Mode != ControlAuthorityMode.Unowned)
            {
                ClearFlightControlState(state);
            }
            if (control != null && control.Authority.Mode == ControlAuthorityMode.EmergencyStop)
            {
                return;
            }
            ApplyDirectEngineAndRcsCommands();
            ApplyGimbalCommands();
            if (!wrenchActive)
            {
                return;
            }

            var residualForce = requestedForce;
            var residualTorque = requestedTorque;
            AllocateWheels(state, ref residualForce, ref residualTorque);
            AllocateEngines(ref residualForce, ref residualTorque);
            AllocateRcs(state, ref residualForce, ref residualTorque);
            allocatedWrench = requestedWrench - new WrenchValue(
                ToDomain(residualForce), ToDomain(residualTorque));
            // FlightCtrlState is consumed by ModuleRCS later in the physics
            // tick. Measure and publish from Update after KSP has applied it.
            wrenchFeedbackPending = true;
        }

        private void AllocateWheels(FlightCtrlState state, ref Vector3 force, ref Vector3 torque)
        {
            var maximumForce = 0f;
            var steeringLever = 0f;
            var count = 0;
            foreach (var wheelBase in parts.Get<ModuleWheelBase>())
            {
                var name = PyLoNActuatorNames.For("wheel", wheelBase.part,
                    PyLoNActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
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
                maximumForce += Mathf.Max(0f, motor.maxTorque) *
                    KspForceUnitInNewtons / radius;
                steeringLever += Mathf.Abs(Vector3.Dot(
                    wheelBase.part.transform.position - vessel.CurrentCoM,
                    BodyForwardWorld()));
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
            var positive = new WrenchValue[6];
            var negative = new WrenchValue[6];
            var hasNozzles = false;
            for (var axis = 0; axis < 6; axis++)
            {
                positive[axis] = EvaluateRcsControl(UnitRcsControl(axis, 1f), ref hasNozzles);
                negative[axis] = EvaluateRcsControl(UnitRcsControl(axis, -1f), ref hasNozzles);
            }
            if (!hasNozzles)
            {
                return;
            }

            AcquireRcsActionGroupOverride();
            var desired = new WrenchValue(ToDomain(force), ToDomain(torque));
            var allocation = RcsControlAllocator.Solve(desired, positive, negative);
            state.X = (float)allocation.Control.X;
            state.Y = (float)allocation.Control.Y;
            state.Z = (float)allocation.Control.Z;
            state.pitch = (float)allocation.Control.Pitch;
            state.yaw = (float)allocation.Control.Yaw;
            state.roll = (float)allocation.Control.Roll;

            // Re-evaluate the combined command through KSP's actual per-nozzle
            // mixer. This captures mixed translation/rotation clamping that a
            // linear channel solve cannot represent.
            var combined = EvaluateRcsControl(allocation.Control, ref hasNozzles);
            force -= ToUnity(combined.Force);
            torque -= ToUnity(combined.Torque);
        }

        private WrenchValue EvaluateRcsControl(RcsControlInput controlInput, ref bool hasNozzles)
        {
            if (vessel == null || vessel.ReferenceTransform == null)
            {
                return WrenchValue.Zero;
            }
            var forceBody = Vector3Value.Zero;
            var torqueBody = Vector3Value.Zero;
            var referenceRotation = vessel.ReferenceTransform.rotation;
            var precisionMode = FlightInputHandler.fetch != null &&
                FlightInputHandler.fetch.precisionMode;
            foreach (var rcs in parts.Get<ModuleRCS>())
            {
                var name = PyLoNActuatorNames.For("rcs", rcs.part,
                    PyLoNActuatorNames.ModuleIndex(rcs.part, rcs));
                if (rcsOverrides.ContainsKey(name) || !rcs.rcsEnabled || rcs.flameout ||
                    rcs.thrusterTransforms == null)
                {
                    continue;
                }
                var maximum = rcs.thrusterPower *
                    Mathf.Clamp01(rcs.thrustPercentage / 100f) * KspForceUnitInNewtons;
                if (maximum <= 0.001f)
                {
                    continue;
                }
                var linearInput = referenceRotation * new Vector3(
                    rcs.enableX ? (float)controlInput.X : 0f,
                    rcs.enableZ ? (float)controlInput.Z : 0f,
                    rcs.enableY ? (float)controlInput.Y : 0f);
                // Match ModuleRCS.Update exactly. KSP converts this local
                // (pitch, roll, yaw) vector through ReferenceTransform before
                // evaluating every nozzle and its moment arm.
                var rotationInput = referenceRotation * new Vector3(
                    rcs.enablePitch ? (float)controlInput.Pitch : 0f,
                    rcs.enableRoll ? (float)controlInput.Roll : 0f,
                    rcs.enableYaw ? (float)controlInput.Yaw : 0f);

                for (var index = 0; index < rcs.thrusterTransforms.Count; index++)
                {
                    var nozzle = rcs.thrusterTransforms[index];
                    if (nozzle == null || !nozzle.gameObject.activeInHierarchy)
                    {
                        continue;
                    }
                    hasNozzles = true;
                    var axis = (rcs.useZaxis ? nozzle.forward : nozzle.up).normalized;
                    var offsetWorld = nozzle.position - vessel.CurrentCoM;
                    var rotationDirection = Vector3.zero;
                    if (rotationInput.sqrMagnitude > 1.0e-10f)
                    {
                        var lever = Vector3.ProjectOnPlane(offsetWorld, rotationInput);
                        if (lever.sqrMagnitude > 1.0e-10f)
                        {
                            rotationDirection = Vector3.Cross(rotationInput, lever.normalized);
                        }
                    }
                    var fraction = Mathf.Max(Vector3.Dot(axis, rotationDirection), 0f) +
                        Mathf.Max(Vector3.Dot(axis, linearInput), 0f);
                    fraction = Mathf.Clamp01(fraction);
                    if (rcs.fullThrust && fraction >= rcs.fullThrustMin)
                    {
                        fraction = 1f;
                    }
                    if (precisionMode)
                    {
                        if (rcs.useLever)
                        {
                            // Matches ModuleRCS.GetLeverDistance: perpendicular
                            // distance from CurrentCoM to the nozzle thrust line.
                            var distance = offsetWorld.magnitude;
                            if (distance > 1.0e-6f)
                            {
                                var cosine = Mathf.Clamp(Vector3.Dot(
                                    -offsetWorld / distance, -axis), -1f, 1f);
                                var leverDistance = distance * Mathf.Sqrt(
                                    Mathf.Max(0f, 1f - cosine * cosine));
                                if (leverDistance > 1f)
                                {
                                    fraction /= leverDistance;
                                }
                            }
                        }
                        else
                        {
                            fraction *= rcs.precisionFactor;
                        }
                    }
                    if (fraction <= 1.0e-6f)
                    {
                        continue;
                    }
                    var forceWorld = -axis * (maximum * fraction);
                    var torqueWorld = Vector3.Cross(offsetWorld, forceWorld);
                    forceBody += ToDomain(WorldVectorToBody(forceWorld));
                    torqueBody += ToDomain(WorldAxialVectorToBody(torqueWorld));
                }
            }
            return new WrenchValue(forceBody, torqueBody);
        }

        private static RcsControlInput UnitRcsControl(int axis, float value)
        {
            var input = new RcsControlInput();
            if (axis == 0) input.X = value;
            else if (axis == 1) input.Y = value;
            else if (axis == 2) input.Z = value;
            else if (axis == 3) input.Pitch = value;
            else if (axis == 4) input.Yaw = value;
            else if (axis == 5) input.Roll = value;
            return input;
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
            foreach (var engine in parts.Get<ModuleEngines>())
            {
                var name = PyLoNActuatorNames.For("engine", engine.part,
                    PyLoNActuatorNames.ModuleIndex(engine.part, engine));
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
                var worldForce = direction * (maxThrust * KspForceUnitInNewtons);
                var worldOffset = center - vessel.CurrentCoM;
                var force = WorldVectorToBody(worldForce);
                channels.Add(new EngineChannel
                {
                    Engine = engine,
                    Force = force,
                    Torque = WorldAxialVectorToBody(Vector3.Cross(worldOffset, worldForce)),
                    MaximumThrust = maxThrust * KspForceUnitInNewtons
                });
            }
            return channels;
        }

        private void ApplyDirectEngineAndRcsCommands()
        {
            foreach (var engine in parts.Get<ModuleEngines>())
            {
                var name = PyLoNActuatorNames.For("engine", engine.part,
                    PyLoNActuatorNames.ModuleIndex(engine.part, engine));
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
            foreach (var rcs in parts.Get<ModuleRCS>())
            {
                var name = PyLoNActuatorNames.For("rcs", rcs.part,
                    PyLoNActuatorNames.ModuleIndex(rcs.part, rcs));
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
            foreach (var wheelBase in parts.Get<ModuleWheelBase>())
            {
                var controller = wheelBase.Wheel;
                if (controller == null) continue;
                var name = PyLoNActuatorNames.For("wheel", wheelBase.part,
                    PyLoNActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
                WheelOverride direct;
                if (!wheelOverrides.TryGetValue(name, out direct)) continue;
                var motor = wheelBase.part.FindModuleImplementing<ModuleWheelMotor>();
                var steering = wheelBase.part.FindModuleImplementing<ModuleWheelSteering>();
                if (!directWheelStates.ContainsKey(wheelBase))
                {
                    directWheelStates[wheelBase] = new OriginalWheelState
                    {
                        MotorEnabled = motor != null && motor.motorEnabled,
                        SteeringComponentEnabled = steering != null && steering.enabled,
                        MaxDriveTorque = controller.maxDriveTorque
                    };
                }
                // Stock FixedUpdate overwrites steerInput from the vessel-wide
                // wheelSteer control. Suspend that component while individual
                // Ackermann angles own the collider; restore it on every exit.
                if (steering != null) steering.enabled = false;
                if (motor != null) motor.motorEnabled = direct.Enabled;
                // Normal commands release the parking group. Timeout/release
                // explicitly leaves it engaged until another command or player.
                if (direct.Brake <= 0f) vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                var brakes = wheelBase.part.FindModuleImplementing<ModuleWheelBrakes>();
                if (brakes != null) brakes.brakeInput = direct.Brake;
                controller.brakeInput = direct.Brake;
                if (!direct.Enabled || direct.Brake > 0f)
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
            if (directWheelStates.Count > 0) ParkRover();
            foreach (var pair in directWheelStates)
            {
                if (pair.Key == null || pair.Key.Wheel == null) continue;
                var motor = pair.Key.part.FindModuleImplementing<ModuleWheelMotor>();
                if (motor != null) motor.motorEnabled = pair.Value.MotorEnabled;
                var steering = pair.Key.part.FindModuleImplementing<ModuleWheelSteering>();
                if (steering != null) steering.enabled = pair.Value.SteeringComponentEnabled;
                pair.Key.Wheel.driveInput = 0f;
                pair.Key.Wheel.steerInput = 0f;
                pair.Key.Wheel.maxDriveTorque = pair.Value.MaxDriveTorque;
            }
            directWheelStates.Clear();
        }





        private void SendRejectedWrench(long sequence, string reason)
        {
            SendRejectedWrench(sequence, reason, WrenchValue.Zero, string.Empty, string.Empty);
        }

        private void SendRejectedWrench(
            long sequence,
            string reason,
            WrenchValue requested,
            string controllerId,
            string leaseId)
        {
            var previousSequence = activeWrenchSequence;
            var previousControllerId = activeWrenchControllerId;
            var previousLeaseId = activeWrenchLeaseId;
            var previousRequested = rawRequestedWrench;
            activeWrenchSequence = sequence;
            activeWrenchControllerId = controllerId ?? string.Empty;
            activeWrenchLeaseId = leaseId ?? string.Empty;
            rawRequestedWrench = requested;
            SendWrenchStatus(WrenchValue.Zero, MeasurePropulsionWrench(), reason, false);
            activeWrenchSequence = previousSequence;
            activeWrenchControllerId = previousControllerId;
            activeWrenchLeaseId = previousLeaseId;
            rawRequestedWrench = previousRequested;
        }

        private void SendWrenchStatus(
            WrenchValue allocated,
            WrenchValue achieved,
            string reason,
            bool accepted = true)
        {
            var allocationResidual = rawRequestedWrench - allocated;
            var trackingResidual = rawRequestedWrench - achieved;
            var forceScale = Math.Max(1.0, rawRequestedWrench.Force.Magnitude);
            var torqueScale = Math.Max(1.0, rawRequestedWrench.Torque.Magnitude);
            var requestedNorm = rawRequestedWrench.WeightedNorm(forceScale, torqueScale);
            var saturationRatio = allocationResidual.WeightedNorm(forceScale, torqueScale) /
                Math.Max(requestedNorm, 1.0e-9);
            var trackingRatio = trackingResidual.WeightedNorm(forceScale, torqueScale) /
                Math.Max(requestedNorm, 1.0e-9);

            var builder = new StringBuilder(1024);
            builder.Append('{');
            AppendString(builder, "type", "pylon_wrench_status", true);
            AppendNumber(builder, "version", ControlProtocolVersion);
            AppendString(builder, "vesselId", ActiveVesselId());
            AppendString(builder, "controllerId", activeWrenchControllerId);
            AppendString(builder, "leaseId", activeWrenchLeaseId);
            AppendNumber(builder, "sequence", activeWrenchSequence);
            AppendBoolean(builder, "accepted", accepted);
            AppendString(builder, "reason", reason ?? string.Empty);
            AppendWrench(builder, "requested", rawRequestedWrench);
            AppendWrench(builder, "allocated", allocated);
            AppendWrench(builder, "achieved", achieved);
            AppendWrench(builder, "allocationResidual", allocationResidual);
            AppendWrench(builder, "trackingResidual", trackingResidual);
            AppendNumber(builder, "saturationRatio", saturationRatio);
            AppendNumber(builder, "trackingErrorRatio", trackingRatio);
            AppendBoolean(builder, "saturated", saturationRatio > 0.1);
            AppendString(builder, "achievedQuality",
                "propulsion_measured_previous_physics_tick_wheels_excluded");
            builder.Append('}');
            Send(builder.ToString());
        }

        private WrenchValue MeasurePropulsionWrench()
        {
            if (vessel == null)
            {
                return WrenchValue.Zero;
            }
            var force = Vector3Value.Zero;
            var torque = Vector3Value.Zero;
            foreach (var engine in parts.Get<ModuleEngines>())
            {
                var thrust = Mathf.Max(0f, engine.GetCurrentThrust()) *
                    KspForceUnitInNewtons;
                var transforms = engine.thrustTransforms;
                if (thrust <= 0f || transforms == null || transforms.Count == 0)
                {
                    continue;
                }
                var perNozzle = thrust / transforms.Count;
                for (var index = 0; index < transforms.Count; index++)
                {
                    var nozzle = transforms[index];
                    if (nozzle == null) continue;
                    var worldForce = -nozzle.forward.normalized * perNozzle;
                    force += ToDomain(WorldVectorToBody(worldForce));
                    torque += ToDomain(WorldAxialVectorToBody(
                        Vector3.Cross(nozzle.position - vessel.CurrentCoM, worldForce)));
                }
            }
            foreach (var rcs in parts.Get<ModuleRCS>())
            {
                if (rcs.thrusterTransforms == null || rcs.thrustForces == null)
                {
                    continue;
                }
                var count = Math.Min(rcs.thrusterTransforms.Count, rcs.thrustForces.Length);
                for (var index = 0; index < count; index++)
                {
                    var nozzle = rcs.thrusterTransforms[index];
                    // KSP stores signed per-nozzle thrust; magnitude is the
                    // achieved force while the transform provides direction.
                    var thrust = Mathf.Abs(rcs.thrustForces[index]) *
                        KspForceUnitInNewtons;
                    if (nozzle == null || thrust <= 0f) continue;
                    var axis = (rcs.useZaxis ? nozzle.forward : nozzle.up).normalized;
                    var worldForce = -axis * thrust;
                    force += ToDomain(WorldVectorToBody(worldForce));
                    torque += ToDomain(WorldAxialVectorToBody(
                        Vector3.Cross(nozzle.position - vessel.CurrentCoM, worldForce)));
                }
            }
            return new WrenchValue(force, torque);
        }

        private void SendControlAuthorityState(string reason)
        {
            if (stateClient == null || control == null)
            {
                return;
            }
            var authority = control.Authority;
            var now = Time.realtimeSinceStartup;
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "pylon_control_authority_state", true);
            AppendNumber(builder, "version", ControlProtocolVersion);
            AppendString(builder, "vesselId", ActiveVesselId());
            AppendString(builder, "vessel", vessel == null ? string.Empty : vessel.vesselName);
            AppendNumber(builder, "state", (int)authority.Mode);
            AppendString(builder, "controllerId", authority.ControllerId);
            AppendString(builder, "leaseId", authority.LeaseId);
            AppendNumber(builder, "priority", authority.Priority);
            AppendNumber(builder, "leaseRemainingSeconds",
                authority.Mode == ControlAuthorityMode.Owned ? Math.Max(0.0, authority.ExpiresAt - now) : 0.0);
            AppendBoolean(builder, "sasSuppressed", sasOverrideActive);
            AppendBoolean(builder, "emergencyStop", authority.Mode == ControlAuthorityMode.EmergencyStop);
            AppendNumber(builder, "lastSequence", authority.LastSequence);
            AppendString(builder, "reason", reason ?? authority.Reason);
            builder.Append('}');
            Send(builder.ToString());
        }

        private bool TargetsActiveVessel(string vesselId)
        {
            return vessel != null && !string.IsNullOrEmpty(vesselId) &&
                string.Equals(vesselId, ActiveVesselId(), StringComparison.OrdinalIgnoreCase);
        }

        private string ActiveVesselId()
        {
            return vessel == null ? string.Empty : vessel.id.ToString("N");
        }

        private void StopAllVehicleControl()
        {
            wrenchActive = false;
            requestedForce = Vector3.zero;
            requestedTorque = Vector3.zero;
            rawRequestedWrench = WrenchValue.Zero;
            requestedWrench = WrenchValue.Zero;
            allocatedWrench = WrenchValue.Zero;
            wrenchFeedbackPending = false;
            wrenchReason = "control_stopped";
            RestoreBodyEngineStates();
            RestoreDirectRcsStates();
            RestoreDirectWheelStates();
            RestoreRcsActionGroup();
            RestoreSasState();
            wheelOverrides.Clear();
            RestoreGimbals(null);
            engineOverrides.Clear();
            rcsOverrides.Clear();
            if (control != null)
            {
                control.ResetSafety(Time.realtimeSinceStartup);
            }
        }

        private static void ClearFlightControlState(FlightCtrlState state)
        {
            state.X = 0f;
            state.Y = 0f;
            state.Z = 0f;
            state.pitch = 0f;
            state.yaw = 0f;
            state.roll = 0f;
            state.mainThrottle = 0f;
            state.wheelThrottle = 0f;
            state.wheelSteer = 0f;
        }

        private void AcquireRcsActionGroupOverride()
        {
            if (vessel == null)
            {
                return;
            }
            if (!rcsActionGroupOverridden)
            {
                rcsActionGroupWasEnabled = vessel.ActionGroups[KSPActionGroup.RCS];
                rcsActionGroupOverridden = true;
            }
            vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
        }

        private void RestoreRcsActionGroup()
        {
            if (!rcsActionGroupOverridden)
            {
                return;
            }
            if (vessel != null)
            {
                vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, rcsActionGroupWasEnabled);
            }
            rcsActionGroupOverridden = false;
            rcsActionGroupWasEnabled = false;
        }

        private static Vector3Value ToDomain(Vector3 value)
        {
            return new Vector3Value(value.x, value.y, value.z);
        }

        private static Vector3 ToUnity(Vector3Value value)
        {
            return new Vector3((float)value.X, (float)value.Y, (float)value.Z);
        }

        private static ControlSafetyPolicy LoadSafetyPolicy()
        {
            var policy = new ControlSafetyPolicy();
            try
            {
                var nodes = GameDatabase.Instance == null
                    ? null
                    : GameDatabase.Instance.GetConfigNodes("PYLON_CONTROL");
                if (nodes == null || nodes.Length == 0)
                {
                    return policy;
                }
                var node = nodes[0];
                policy.MaximumForce = PositiveConfig(node, "maxForceN", policy.MaximumForce);
                policy.MaximumTorque = PositiveConfig(node, "maxTorqueNm", policy.MaximumTorque);
                policy.MaximumAngularSpeed = PositiveConfig(
                    node, "maxAngularSpeedRadSec", policy.MaximumAngularSpeed);
                policy.MaximumForceSlew = PositiveConfig(
                    node, "maxForceSlewNPerSec", policy.MaximumForceSlew);
                policy.MaximumTorqueSlew = PositiveConfig(
                    node, "maxTorqueSlewNmPerSec", policy.MaximumTorqueSlew);
                policy.MaximumContinuousActuation = PositiveConfig(
                    node, "maxContinuousActuationSec", policy.MaximumContinuousActuation);
                policy.ResetIdleDuration = PositiveConfig(
                    node, "continuousResetIdleSec", policy.ResetIdleDuration);
            }
            catch (Exception exception)
            {
                Debug.LogWarning("[PyLoN] Invalid PYLON_CONTROL config: " + exception.Message);
            }
            return policy;
        }

        private static double PositiveConfig(ConfigNode node, string name, double fallback)
        {
            if (node == null)
            {
                return fallback;
            }
            double parsed;
            return double.TryParse(
                node.GetValue(name), NumberStyles.Float, CultureInfo.InvariantCulture, out parsed) &&
                IsFinite(parsed) && parsed > 0.0
                ? parsed
                : fallback;
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

        private Vector3 VesselAngularVelocityWorld()
        {
            if (vessel == null)
            {
                return Vector3.zero;
            }
            var reference = vessel.ReferenceTransform;
            return reference == null
                ? vessel.angularVelocity
                : reference.rotation * vessel.angularVelocity;
        }

        private Vector3 VesselAngularVelocityBody()
        {
            if (vessel == null)
            {
                return Vector3.zero;
            }
            // Angular velocity is an axial vector. Unity -> ROS changes
            // handedness, so it needs the determinant (-1) in addition to
            // the (forward, left, up) permutation used for linear vectors.
            var local = vessel.angularVelocity;
            return FrameConversions.VesselLocalAxialToBody(local);
        }

        private Vector3 WorldAxialVectorToBody(Vector3 world)
        {
            // For reflection M: (M r) x (M F) = det(M) M (r x F).
            // Keep torques consistent with ROS quaternion derivatives.
            return -WorldVectorToBody(world);
        }













        private void Send(string json)
        {
            if (stateClient == null || stateEndpoint == null) return;
            try
            {
                var bytes = TelemetryPacketCodec.Encode(json);
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
            Debug.LogWarning("[PyLoN] " + message);
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






















    }
}
