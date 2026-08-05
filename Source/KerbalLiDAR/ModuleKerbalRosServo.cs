using System;
using Expansions.Serenity;
using UnityEngine;

namespace KerbalLiDAR
{
    public sealed class ModuleKerbalRosServo : ModuleRoboticRotationServo, IKerbalRosMotor
    {
        [KSPField(isPersistant = true)]
        public string motorName = "";

        [KSPField]
        public int commandUdpPort = 49011;

        [KSPField]
        public string stateUdpHost = "127.0.0.1";

        [KSPField]
        public int stateUdpPort = 49010;

        [KSPField]
        public float stateRateHz = 20f;

        [KSPField]
        public float commandTimeoutSeconds = 0.5f;

        [KSPField]
        public float ratedEffortNm = 250f;

        [KSPField]
        public float torquePerAmpNm = 40f;

        [KSPField]
        public string movingModelPath = "Squad/Parts/Utility/dockingPortJr/dockingPortJr";

        [KSPField]
        public string movingModelPosition = "0, 0.1875, 0";

        [KSPField]
        public string movingModelRotation = "0, 0, 0";

        [KSPField]
        public string movingModelScale = "1, 1, 1";

        [KSPField(guiActive = true, guiName = "ROS Joint")]
        public string rosJointDisplay = "";

        [KSPField(guiActive = true, guiName = "ROS Command")]
        public string rosCommandState = "Waiting";

        private bool visualReady;
        private bool velocityMode;
        private float commandedVelocityDegrees;
        private float commandExpiresAt;
        private float previousStateAngle;
        private float stateVelocityDegrees;
        private bool stateVelocityInitialized;

        Part IKerbalRosMotor.MotorPart { get { return part; } }
        string IKerbalRosMotor.JointName { get { return JointName; } }
        string IKerbalRosMotor.JointType { get { return "revolute"; } }
        int IKerbalRosMotor.CommandUdpPort { get { return commandUdpPort; } }
        string IKerbalRosMotor.StateUdpHost { get { return stateUdpHost; } }
        int IKerbalRosMotor.StateUdpPort { get { return stateUdpPort; } }
        float IKerbalRosMotor.StateRateHz { get { return stateRateHz; } }

        private string JointName
        {
            get { return KerbalRosMotorNames.Resolve(motorName, "servo", part); }
        }

        public override void OnStartBeforePartAttachJoint(StartState state)
        {
            EnsureVisuals();
            base.OnStartBeforePartAttachJoint(state);
        }

        public override void OnStart(StartState state)
        {
            EnsureVisuals();
            base.OnStart(state);
            rosJointDisplay = JointName;
            KerbalRosMotorUdpManager.Register(this);
        }

        protected override void OnFixedUpdate()
        {
            UpdateVelocityCommand();
            base.OnFixedUpdate();
            UpdateStateVelocity();
        }

        protected override void OnDestroy()
        {
            KerbalRosMotorUdpManager.Unregister(this);
            base.OnDestroy();
        }

        void IKerbalRosMotor.ApplyRosCommand(KerbalRosMotorCommand command)
        {
            if (command == null)
            {
                return;
            }

            if (command.hasEffort && IsFinite(command.effort))
            {
                var percentage = ratedEffortNm > 0.001f
                    ? Mathf.Clamp01((float)Math.Abs(command.effort) / ratedEffortNm) * 100f
                    : 100f;
                servoMotorLimit = percentage;
            }

            if (!servoMotorIsEngaged)
            {
                EngageMotor();
            }

            if (string.Equals(command.mode, "velocity", StringComparison.OrdinalIgnoreCase) &&
                command.hasVelocity && IsFinite(command.velocity))
            {
                if (Math.Abs(command.velocity) < 0.000001)
                {
                    velocityMode = false;
                    targetAngle = currentAngle;
                    previousTargetAngle = targetAngle;
                    rosCommandState = "Velocity hold";
                    return;
                }

                velocityMode = true;
                commandedVelocityDegrees = (float)(command.velocity * Mathf.Rad2Deg);
                commandExpiresAt = Time.realtimeSinceStartup + Mathf.Max(0.05f, commandTimeoutSeconds);
                rosCommandState = "Velocity";
                return;
            }

            velocityMode = false;
            if (command.hasPosition && IsFinite(command.position))
            {
                var degrees = (float)(command.position * Mathf.Rad2Deg);
                targetAngle = Mathf.Clamp(degrees, hardMinMaxLimits.x, hardMinMaxLimits.y);
                previousTargetAngle = targetAngle;
            }

            if (command.hasVelocity && IsFinite(command.velocity) && Math.Abs(command.velocity) > 0.000001)
            {
                traverseVelocity = Mathf.Clamp(
                    (float)Math.Abs(command.velocity * Mathf.Rad2Deg),
                    traverseVelocityLimits.x,
                    traverseVelocityLimits.y);
            }

            rosCommandState = "Position";
        }

        string IKerbalRosMotor.BuildStatePacket()
        {
            var error = Mathf.DeltaAngle(currentAngle, targetAngle);
            var effortNm = Mathf.Clamp(Mathf.Abs(motorOutput) * 1000f, 0f, Mathf.Max(0f, ratedEffortNm));
            if (Mathf.Abs(error) > 0.001f)
            {
                effortNm *= Mathf.Sign(error);
            }

            var currentAmps = torquePerAmpNm > 0.001f ? Mathf.Abs(effortNm) / torquePerAmpNm : 0f;
            return KerbalRosMotorJson.State(
                this,
                currentAngle * Mathf.Deg2Rad,
                stateVelocityDegrees * Mathf.Deg2Rad,
                effortNm,
                currentAmps,
                targetAngle * Mathf.Deg2Rad,
                hasEnoughResources,
                servoMotorIsEngaged,
                servoIsLocked);
        }

        private void UpdateVelocityCommand()
        {
            if (!velocityMode)
            {
                return;
            }

            if (Time.realtimeSinceStartup > commandExpiresAt)
            {
                velocityMode = false;
                targetAngle = currentAngle;
                previousTargetAngle = targetAngle;
                rosCommandState = "Velocity timeout";
                return;
            }

            traverseVelocity = Mathf.Clamp(
                Mathf.Abs(commandedVelocityDegrees),
                traverseVelocityLimits.x,
                traverseVelocityLimits.y);
            var direction = Mathf.Sign(commandedVelocityDegrees);
            targetAngle = direction >= 0f ? hardMinMaxLimits.y : hardMinMaxLimits.x;
            previousTargetAngle = targetAngle;
        }

        private void EnsureVisuals()
        {
            if (visualReady || part == null)
            {
                return;
            }

            var baseTransform = KerbalRosMotorVisuals.EnsureTransform(part, baseTransformName);
            var movingTransform = KerbalRosMotorVisuals.EnsureTransform(part, servoTransformName);
            KerbalRosMotorVisuals.AddStockModel(
                part,
                movingTransform,
                movingModelPath,
                movingModelPosition,
                movingModelRotation,
                movingModelScale);
            KerbalRosMotorVisuals.AddIndicator(
                part,
                movingTransform,
                PrimitiveType.Cube,
                new Vector3(0.24f, 0.27f, 0f),
                new Vector3(0.08f, 0.04f, 0.04f),
                new Color(1f, 0.35f, 0.05f));
            baseTransform.gameObject.layer = part.gameObject.layer;
            visualReady = true;
        }

        private void UpdateStateVelocity()
        {
            if (!stateVelocityInitialized)
            {
                previousStateAngle = currentAngle;
                stateVelocityInitialized = true;
                return;
            }

            var elapsed = Mathf.Max(Time.fixedDeltaTime, 0.0001f);
            var instantaneous = Mathf.DeltaAngle(previousStateAngle, currentAngle) / elapsed;
            stateVelocityDegrees = Mathf.Lerp(stateVelocityDegrees, instantaneous, 0.35f);
            previousStateAngle = currentAngle;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
