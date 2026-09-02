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

        private bool velocityMode;
        private float commandedVelocity;
        private float commandExpiresAt;
        private bool commandActive;
        private string activeCommandMode = "position";
        private GameObject pistonRod;
        private float previousStateExtension;
        private float stateVelocity;
        private bool stateVelocityInitialized;
        private bool stockServoStarted;
        private bool stockAttachJointStarted;

        Part IKerbalRosMotor.MotorPart { get { return part; } }
        string IKerbalRosMotor.JointName { get { return JointName; } }
        string IKerbalRosMotor.JointType { get { return "prismatic"; } }
        int IKerbalRosMotor.CommandUdpPort { get { return commandUdpPort; } }
        string IKerbalRosMotor.StateUdpHost { get { return stateUdpHost; } }
        int IKerbalRosMotor.StateUdpPort { get { return stateUdpPort; } }
        float IKerbalRosMotor.StateRateHz { get { return stateRateHz; } }

        private string JointName
        {
            get { return KerbalRosMotorNames.Resolve(motorName, "linear", part, this); }
        }

        public override void OnAwake()
        {
            base.OnAwake();
            EnsureVisuals();
        }

        public override void OnStartBeforePartAttachJoint(StartState state)
        {
            EnsureVisuals();
            StartStockAttachJoint(state);
        }

        public override void OnStart(StartState state)
        {
            EnsureVisuals();
            rosJointDisplay = JointName;

            // BaseServo rewires the driven attach node during OnStart. Doing that
            // while a new part is still attached to the editor cursor interrupts
            // EditorLogic.attachPart and leaves its placement ghost active.
            if (!HighLogic.LoadedSceneIsEditor || part == null || part.isAttached)
            {
                StartStockServo(state);
            }

            if (HighLogic.LoadedSceneIsFlight)
            {
                KerbalRosMotorUdpManager.Register(this);
            }
        }

        protected override void Update()
        {
            if (!stockServoStarted && HighLogic.LoadedSceneIsEditor && part != null && part.isAttached)
            {
                StartStockServo(StartState.Editor);
                StartStockAttachJoint(StartState.Editor);
            }

            if (stockServoStarted)
            {
                base.Update();
            }
        }

        protected override void OnFixedUpdate()
        {
            if (!stockServoStarted)
            {
                return;
            }

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

            if (stockServoStarted)
            {
                base.OnDestroy();
            }
        }

        void IKerbalRosMotor.ApplyRosCommand(KerbalRosMotorCommand command)
        {
            if (command == null)
            {
                return;
            }

            if (command.hasEnabled)
            {
                if (command.enabled && !servoMotorIsEngaged)
                {
                    EngageMotor();
                }
                else if (!command.enabled)
                {
                    if (servoMotorIsEngaged) DisengageMotor();
                    velocityMode = false;
                    commandActive = false;
                    targetExtension = currentExtension;
                    rosCommandState = "Disabled";
                    return;
                }
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

            activeCommandMode = string.IsNullOrEmpty(command.mode) ? "position" : command.mode.ToLowerInvariant();
            var timeout = IsFinite(command.timeoutSeconds) && command.timeoutSeconds > 0.0
                ? Mathf.Clamp((float)command.timeoutSeconds, 0.05f, 10f)
                : Mathf.Max(0.05f, commandTimeoutSeconds);
            commandExpiresAt = Time.realtimeSinceStartup + timeout;
            commandActive = true;

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
                servoIsLocked,
                activeCommandMode,
                commandActive);
        }

        private void UpdateVelocityCommand()
        {
            if (commandActive && Time.realtimeSinceStartup > commandExpiresAt)
            {
                commandActive = false;
                velocityMode = false;
                targetExtension = currentExtension;
                if (servoMotorIsEngaged) DisengageMotor();
                rosCommandState = "Command timeout";
                return;
            }

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
            if (part == null)
            {
                return;
            }

            var baseTransform = KerbalRosMotorVisuals.EnsureTransform(part, baseTransformName);
            var movingTransform = KerbalRosMotorVisuals.EnsureTransform(part, servoTransformName);
            if (baseTransform == null || movingTransform == null)
            {
                Debug.LogError("[KerbalLiDAR] ROS linear motor could not create its fixed and moving transforms.");
                return;
            }

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
                new Vector3(0f, 0.1474114f, 0f),
                new Vector3(0.12f, 0.001f, 0.12f),
                new Color(0.72f, 0.75f, 0.78f));
        }

        private void StartStockServo(StartState state)
        {
            if (stockServoStarted)
            {
                return;
            }

            base.OnStart(state);
            stockServoStarted = true;
        }

        private void StartStockAttachJoint(StartState state)
        {
            if (!stockServoStarted || stockAttachJointStarted)
            {
                return;
            }

            base.OnStartBeforePartAttachJoint(state);
            stockAttachJointStarted = true;
        }

        private void UpdatePistonRod()
        {
            if (pistonRod == null)
            {
                return;
            }

            var extension = Mathf.Max(0.002f, currentExtension);
            pistonRod.transform.localPosition = new Vector3(0f, 0.1474114f + extension * 0.5f, 0f);
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
