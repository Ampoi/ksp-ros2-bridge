using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using ModuleWheels;
using static PyLoN.JsonPacketWriter;
namespace PyLoN {
    /// <summary>Read-only actuator state and geometry; control only supplies override status.</summary>
    internal sealed class ActuatorTelemetry {
        private const int ProtocolVersion = 1;
        private const float KspForceUnitInNewtons = FrameConversions.KilonewtonsToNewtons;
        private readonly Func<Vessel> current;
        private Vessel vessel { get { return current(); } }
        private readonly VesselParts parts;
        private readonly Action<string> Send;
        private readonly Func<string, string, bool> overrideActive;
        private readonly Action<StringBuilder, ModuleEngines> AppendGimbalState;
        private Vector3 roverMinimum, roverMaximum;
        private int roverWheelCount;
        private float nextRoverGeometry;
        public ActuatorTelemetry(Func<Vessel> current, VesselParts parts, Action<string> send,
            Func<string, string, bool> overrideActive, Action<StringBuilder, ModuleEngines> appendGimbalState) {
            this.current = current; this.parts = parts; Send = send;
            this.overrideActive = overrideActive; AppendGimbalState = appendGimbalState;
        }
        public void Reset() { nextRoverGeometry = 0; }
        private Transform BodyFrame { get { return vessel.ReferenceTransform != null ? vessel.ReferenceTransform : vessel.transform; } }
        private Vector3 WorldVectorToBody(Vector3 value) { return FrameConversions.WorldToBody(BodyFrame, value); }
        private Vector3 BodyForwardWorld() { return BodyFrame.up; }
        private Vector3 BodyUpWorld() { return -BodyFrame.forward; }
        private string ActiveVesselId() { return vessel == null ? "" : vessel.id.ToString("N"); }
        public void PublishStates()
        {
            UpdateRoverGeometry();
            foreach (var wheelBase in parts.Get<ModuleWheelBase>()) SendWheelState(wheelBase);
            foreach (var engine in parts.Get<ModuleEngines>()) SendEngineState(engine);
            foreach (var rcs in parts.Get<ModuleRCS>()) SendRcsState(rcs);
            foreach (var separation in parts.Separations()) PublishSeparation(separation);
        }

        public void PublishManifest()
        {
            if (vessel == null) return;
            var builder = new StringBuilder(1024);
            builder.Append('{');
            AppendString(builder, "type", "pylon_actuator_manifest", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "vesselId", vessel.id.ToString("N"));
            Prefix(builder, "actuators", false);
            builder.Append('[');
            var first = true;
            foreach (var wheel in parts.Get<ModuleWheelBase>())
            {
                AppendManifestActuator(builder, "wheel", PyLoNActuatorNames.For(
                    "wheel", wheel.part, PyLoNActuatorNames.ModuleIndex(wheel.part, wheel)), ref first);
            }
            foreach (var engine in parts.Get<ModuleEngines>())
            {
                AppendManifestActuator(builder, "engine", PyLoNActuatorNames.For(
                    "engine", engine.part, PyLoNActuatorNames.ModuleIndex(engine.part, engine)), ref first);
            }
            foreach (var rcs in parts.Get<ModuleRCS>())
            {
                AppendManifestActuator(builder, "rcs", PyLoNActuatorNames.For(
                    "rcs", rcs.part, PyLoNActuatorNames.ModuleIndex(rcs.part, rcs)), ref first);
            }
            foreach (var motor in parts.Get<IPyLoNMotor>())
            {
                AppendManifestActuator(builder, "motor", motor.JointName, ref first);
            }
            foreach (var separation in parts.Separations())
            {
                var mechanism = VesselParts.SeparationMechanism(separation);
                AppendManifestActuator(builder, "separation", PyLoNActuatorNames.For(
                    mechanism, separation.part,
                    PyLoNActuatorNames.ModuleIndex(separation.part, separation)), ref first);
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
            var name = PyLoNActuatorNames.For("wheel", wheelBase.part,
                PyLoNActuatorNames.ModuleIndex(wheelBase.part, wheelBase));
            var builder = BeginActuatorState("wheel", name, wheelBase.part);
            AppendBoolean(builder, "enabled", motor != null && motor.motorEnabled);
            AppendWheelGeometry(builder, wheelBase);
            AppendBoolean(builder, "grounded", state.grounded);
            AppendNumber(builder, "angularPosition", controller.wheelCollider == null ? 0f : controller.wheelCollider.angularPosition);
            AppendNumber(builder, "angularVelocity", state.angularVelocity);
            AppendNumber(builder, "steeringAngle", state.steerAngle * Mathf.Deg2Rad);
            AppendNumber(builder, "driveTorque", state.driveTorque * KspForceUnitInNewtons);
            AppendNumber(builder, "brakeTorque", state.brakeTorque * KspForceUnitInNewtons);
            AppendNumber(builder, "slip", state.combinedTireSlip);
            AppendNumber(builder, "maxDriveTorque",
                controller.maxDriveTorque * KspForceUnitInNewtons);
            AppendBoolean(builder, "commandActive", overrideActive("wheel", name));
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendEngineState(ModuleEngines engine)
        {
            var name = PyLoNActuatorNames.For("engine", engine.part,
                PyLoNActuatorNames.ModuleIndex(engine.part, engine));
            var builder = BeginActuatorState("engine", name, engine.part);
            AppendBoolean(builder, "enabled", engine.EngineIgnited);
            AppendBoolean(builder, "operational", engine.isOperational);
            AppendBoolean(builder, "flameout", engine.flameout);
            AppendNumber(builder, "throttle", engine.currentThrottle);
            AppendNumber(builder, "thrust",
                engine.GetCurrentThrust() * KspForceUnitInNewtons);
            AppendNumber(builder, "maxThrust",
                engine.GetMaxThrust() * KspForceUnitInNewtons);
            AppendBoolean(builder, "commandActive", overrideActive("engine", name));
            AppendGimbalState(builder, engine);
            builder.Append('}');
            Send(builder.ToString());
        }

        private void SendRcsState(ModuleRCS rcs)
        {
            var name = PyLoNActuatorNames.For("rcs", rcs.part,
                PyLoNActuatorNames.ModuleIndex(rcs.part, rcs));
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
            AppendNumber(builder, "thrust", thrust * KspForceUnitInNewtons);
            AppendNumber(builder, "maxThrust",
                rcs.thrusterPower * nozzleCount * KspForceUnitInNewtons);
            AppendNumber(builder, "thrustLimit", rcs.thrusterPower *
                Mathf.Clamp01(rcs.thrustPercentage / 100f) * KspForceUnitInNewtons);
            AppendBoolean(builder, "commandActive", overrideActive("rcs", name));
            builder.Append('}');
            Send(builder.ToString());
        }

        public void PublishSeparation(PartModule module)
        {
            if (module == null || module.part == null)
            {
                return;
            }
            var mechanism = VesselParts.SeparationMechanism(module);
            if (string.IsNullOrEmpty(mechanism))
            {
                return;
            }
            var name = PyLoNActuatorNames.For(
                mechanism, module.part, PyLoNActuatorNames.ModuleIndex(module.part, module));
            var builder = BeginActuatorState("separation", name, module.part);
            AppendString(builder, "mechanism", mechanism);
            AppendBoolean(builder, "available", VesselParts.SeparationAvailable(module));
            AppendBoolean(builder, "separated", VesselParts.SeparationComplete(module));
            builder.Append('}');
            Send(builder.ToString());
        }

        private StringBuilder BeginActuatorState(string kind, string name, Part targetPart)
        {
            var builder = new StringBuilder(512);
            builder.Append('{');
            AppendString(builder, "type", "pylon_actuator_state", true);
            AppendNumber(builder, "version", ProtocolVersion);
            AppendString(builder, "actuatorType", kind);
            AppendString(builder, "name", name);
            AppendString(builder, "vessel", vessel == null ? string.Empty : vessel.vesselName);
            AppendNumber(builder, "partFlightId", targetPart == null ? 0u : targetPart.flightID);
            AppendNumber(builder, "persistentId", targetPart == null ? 0u : targetPart.persistentId);
            AppendNumber(builder, "universalTime", Planetarium.GetUniversalTime());
            return builder;
        }

        private void UpdateRoverGeometry()
        {
            if (Time.realtimeSinceStartup < nextRoverGeometry) return;
            nextRoverGeometry = Time.realtimeSinceStartup + 1f;
            roverMinimum = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            roverMaximum = -roverMinimum;
            roverWheelCount = 0;
            foreach (var wheel in parts.Get<ModuleWheelBase>()) roverWheelCount++;
            foreach (var part in vessel.parts)
            {
                // Collider bounds deliberately overestimate rotated mesh bounds.
                foreach (var collider in part.GetComponentsInChildren<Collider>())
                {
                    if (!collider.enabled || collider.isTrigger) continue;
                    var bounds = collider.bounds;
                    for (int corner = 0; corner < 8; corner++)
                    {
                        var point = bounds.center + Vector3.Scale(bounds.extents,
                            new Vector3((corner & 1) == 0 ? -1 : 1,
                                (corner & 2) == 0 ? -1 : 1, (corner & 4) == 0 ? -1 : 1));
                        var local = WorldVectorToBody(point - (Vector3)vessel.CurrentCoM);
                        roverMinimum = Vector3.Min(roverMinimum, local);
                        roverMaximum = Vector3.Max(roverMaximum, local);
                    }
                }
            }
            if (float.IsInfinity(roverMinimum.x)) roverMinimum = roverMaximum = Vector3.zero;
        }

        private void AppendWheelGeometry(StringBuilder builder, ModuleWheelBase wheel)
        {
            var controller = wheel.Wheel;
            var frame = controller.wcTransform;
            var steer = wheel.part.FindModuleImplementing<ModuleWheelSteering>();
            var position = frame == null ? wheel.part.transform.position : frame.position;
            AppendString(builder, "vesselId", ActiveVesselId());
            AppendNumber(builder, "wheelCount", roverWheelCount);
            AppendNumber(builder, "radius", controller.WheelRadius);
            AppendVector(builder, "position", WorldVectorToBody(position - (Vector3)vessel.CurrentCoM));
            // Direct wheel commands bypass ModuleWheelMotor's inversion flags.
            // Signs refer to the physical, unsteered wheel collider frame.
            var forward = frame == null ? Vector3.zero : frame.forward;
            var up = frame == null ? Vector3.zero : frame.up;
            var alignment = Vector3.Dot(forward, BodyForwardWorld());
            AppendNumber(builder, "rollingSign", Mathf.Abs(alignment) > 0.9f ? Mathf.Sign(alignment) : 0f);
            var upAlignment = Vector3.Dot(up, BodyUpWorld());
            AppendNumber(builder, "steeringSign", Mathf.Abs(upAlignment) > 0.9f ? -Mathf.Sign(upAlignment) : 0f);
            AppendBoolean(builder, "steeringEnabled", steer != null && steer.steeringEnabled);
            AppendNumber(builder, "maxSteeringAngle", Mathf.Abs(controller.maxSteerAngle) * Mathf.Deg2Rad);
            AppendVector(builder, "bodyMin", roverMinimum);
            AppendVector(builder, "bodyMax", roverMaximum);
        }
    }
}
