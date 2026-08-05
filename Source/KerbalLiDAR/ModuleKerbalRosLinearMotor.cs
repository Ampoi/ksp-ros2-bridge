using System;
using Expansions.Serenity;
using UnityEngine;

namespace KerbalLiDAR
{
    public sealed class ModuleKerbalRosLinearMotor : ModuleRoboticServoPiston, IKerbalRosMotor
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
        public float ratedEffortN = 4000f;

        [KSPField]
        public float forcePerAmpN = 800f;

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
        private float commandedVelocity;
        private float commandExpiresAt;
        private GameObject pistonRod;
        private float previousStateExtension;
        private float stateVelocity;
        private bool stateVelocityInitialized;

        Part IKerbalRosMotor.MotorPart { get { return part; } }
        string IKerbalRosMotor.JointName { get { return JointName; } }
        string IKerbalRosMotor.JointType { get { return "prismatic"; } }
        int IKerbalRosMotor.CommandUdpPort { get { return commandUdpPort; } }
        string IKerbalRosMotor.StateUdpHost { get { return stateUdpHost; } }
        int IKerbalRosMotor.StateUdpPort { get { return stateUdpPort; } }
        float IKerbalRosMotor.StateRateHz { get { return stateRateHz; } }

        private string JointName
        {
            get { return KerbalRosMotorNames.Resolve(motorName, "linear", part); }
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
            UpdatePistonRod();
        }

        protected override void OnDestroy()
        {
            KerbalRosMotorUdpManager.Unregister(this);
            if (pistonRod != null)
            {
                UnityEngine.Object.Destroy(pistonRod);
                pistonRod = null;
            }

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
                var percentage = ratedEffortN > 0.001f
                    ? Mathf.Clamp01((float)Math.Abs(command.effort) / ratedEffortN) * 100f
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
                    targetExtension = currentExtension;
                    rosCommandState = "Velocity hold";
                    return;
                }

                velocityMode = true;
                commandedVelocity = (float)command.velocity;
                commandExpiresAt = Time.realtimeSinceStartup + Mathf.Max(0.05f, commandTimeoutSeconds);
                rosCommandState = "Velocity";
                return;
            }

            velocityMode = false;
            if (command.hasPosition && IsFinite(command.position))
            {
                targetExtension = Mathf.Clamp((float)command.position, hardMinMaxLimits.x, hardMinMaxLimits.y);
            }

            if (command.hasVelocity && IsFinite(command.velocity) && Math.Abs(command.velocity) > 0.000001)
            {
                traverseVelocity = Mathf.Clamp(
                    (float)Math.Abs(command.velocity),
                    traverseVelocityLimits.x,
                    traverseVelocityLimits.y);
            }

            rosCommandState = "Position";
        }

        string IKerbalRosMotor.BuildStatePacket()
        {
            var error = targetExtension - currentExtension;
            var effortN = Mathf.Clamp(Mathf.Abs(motorOutput) * 1000f, 0f, Mathf.Max(0f, ratedEffortN));
            if (Mathf.Abs(error) > 0.0001f)
            {
                effortN *= Mathf.Sign(error);
            }

            var currentAmps = forcePerAmpN > 0.001f ? Mathf.Abs(effortN) / forcePerAmpN : 0f;
            return KerbalRosMotorJson.State(
                this,
                currentExtension,
                stateVelocity,
                effortN,
                currentAmps,
                targetExtension,
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
                targetExtension = currentExtension;
                rosCommandState = "Velocity timeout";
                return;
            }

            traverseVelocity = Mathf.Clamp(
                Mathf.Abs(commandedVelocity),
                traverseVelocityLimits.x,
                traverseVelocityLimits.y);
            targetExtension = commandedVelocity >= 0f ? hardMinMaxLimits.y : hardMinMaxLimits.x;
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
            pistonRod = KerbalRosMotorVisuals.AddIndicator(
                part,
                baseTransform,
                PrimitiveType.Cylinder,
                new Vector3(0f, 0.19f, 0f),
                new Vector3(0.12f, 0.001f, 0.12f),
                new Color(0.72f, 0.75f, 0.78f));
            visualReady = true;
        }

        private void UpdatePistonRod()
        {
            if (pistonRod == null)
            {
                return;
            }

            var extension = Mathf.Max(0.002f, currentExtension);
            pistonRod.transform.localPosition = new Vector3(0f, 0.1875f + extension * 0.5f, 0f);
            pistonRod.transform.localScale = new Vector3(0.12f, extension * 0.5f, 0.12f);
        }

        private void UpdateStateVelocity()
        {
            if (!stateVelocityInitialized)
            {
                previousStateExtension = currentExtension;
                stateVelocityInitialized = true;
                return;
            }

            var elapsed = Mathf.Max(Time.fixedDeltaTime, 0.0001f);
            var instantaneous = (currentExtension - previousStateExtension) / elapsed;
            stateVelocity = Mathf.Lerp(stateVelocity, instantaneous, 0.35f);
            previousStateExtension = currentExtension;
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
