using System;
using System.Collections.Generic;
using UnityEngine;

namespace KerbalLiDAR
{
    /// <summary>
    /// DLC-independent linear actuator built on the same passive-editor,
    /// flight-joint lifecycle as ModuleKerbalRosServo. In the editor this is
    /// a two-node telescoping housing with an adjustable output attachment.
    /// In flight the joint at the top node is converted into a driven slider.
    /// </summary>
    public sealed partial class ModuleKerbalRosLinearMotor : ModuleJointMotor, IKerbalRosMotor
    {
        [KSPField(isPersistant = true)]
        public string motorName = "";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Target Extension", guiUnits = " m")]
        [UI_FloatRange(minValue = 0f, maxValue = 1.6f, stepIncrement = 0.01f)]
        public float targetExtension;

        [KSPField] public float minExtension;
        [KSPField] public float maxExtension = 1.6f;
        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Travel Speed", guiUnits = " m/s", guiFormat = "F2")]
        [UI_FloatRange(minValue = 0.02f, maxValue = 1f, stepIncrement = 0.01f)]
        public float traverseVelocity = 0.5f;
        [KSPField] public float minTraverseVelocity = 0.02f;
        [KSPField] public float maxTraverseVelocity = 1f;
        [KSPField] public float positionGain = 3f;
        [KSPField] public float extensionTolerance = 0.002f;
        [KSPField] public float jointDamper = 100f;
        [KSPField] public float maxForce = 4000f;
        [KSPField] public string drivenNodeName = "top";
        [KSPField] public string movingTransformName = "LinearRod";
        [KSPField] public string intermediateTransformName = "LinearSleeve";
        [KSPField] public string movingModelPath = "";
        [KSPField] public string movingModelPosition = "0, 0.8024656, 0";
        [KSPField] public string movingModelRotation = "0, 0, 0";
        [KSPField] public string movingModelScale = "0.78, 1, 0.78";
        [KSPField(isPersistant = true)] public bool motorEngaged = true;
        [KSPField(isPersistant = true)] public bool motorLocked;
        [KSPField] public int commandUdpPort = 49011;
        [KSPField] public string stateUdpHost = "127.0.0.1";
        [KSPField] public int stateUdpPort = 49010;
        [KSPField] public float stateRateHz = 20f;
        [KSPField] public float commandTimeoutSeconds = 0.5f;
        [KSPField] public float ratedEffortN = 4000f;
        [KSPField] public float forcePerAmpN = 800f;

        [KSPField(guiActive = true, guiName = "ROS Joint")]
        public string rosJointDisplay = "";

        [KSPField(guiActive = true, guiName = "ROS Command")]
        public string rosCommandState = "Waiting";

        [KSPField(isPersistant = true, guiActive = true, guiName = "Current Extension", guiUnits = " m", guiFormat = "F3")]
        public float currentExtension;

        [KSPField(guiActive = true, guiName = "Motor Status")]
        public string motorStatus = "Waiting for joint";

        private Transform movingTransform;
        private Transform intermediateTransform;
        private Vector3 intermediateRestPosition;
        private Vector3 movingTransformRestPosition;
        private bool movingTransformReady;
        private bool jointInitialized;
        private bool loadedCurrentExtension;
        private Part drivenPart;
        private Vector3 referenceLocalDrivenPosition;
        private float referenceExtension;
        private bool velocityMode;
        private float commandedVelocity;
        private float commandExpiresAt;
        private bool commandActive;
        private string activeCommandMode = "position";
        private float editorAppliedExtension;
        private float previousStateExtension;
        private float stateVelocity;
        private bool stateVelocityInitialized;
        private float effortLimitN;
        private float driveSign = 1f;
        private bool driveSignCalibrated;
        private float previousMeasuredExtension;
        private float lastLogicalSpeed;
        private int lastManagedFrame = -1;

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

        protected override void OnModuleStart(StartState state)
        {
            jointNodeName = drivenNodeName;
            targetExtension = ClampExtension(targetExtension);
            traverseVelocity = Mathf.Clamp(traverseVelocity, minTraverseVelocity, maxTraverseVelocity);
            editorAppliedExtension = targetExtension;
            currentExtension = HighLogic.LoadedSceneIsFlight && loadedCurrentExtension
                ? ClampExtension(currentExtension) : targetExtension;
            effortLimitN = Mathf.Max(0f, Mathf.Min(maxForce, ratedEffortN));
            rosJointDisplay = JointName;
            EnsureVisuals();
            CacheMovingTransform();
            SetVisualExtension(currentExtension);
            CacheDrivenNode();
            UpdateDrivenNode(currentExtension);

            if (HighLogic.LoadedSceneIsFlight)
            {
                KerbalRosMotorUdpManager.Register(this);
            }

            RefreshEventLabels();
        }

        protected override void OnModuleLoad(ConfigNode node)
        {
            loadedCurrentExtension = node.HasValue("currentExtension");
        }

        protected override void OnModuleSave(ConfigNode node)
        {
        }

        protected override void OnJointInit(bool goodSetup)
        {
            jointInitialized = goodSetup && joint != null;
            if (!jointInitialized)
            {
                motorStatus = "No driven joint";
                Debug.LogWarning("[KerbalLiDAR] ROS linear actuator has no physical joint at node '" + drivenNodeName + "': " + JointName);
                return;
            }

            var node = part.FindAttachNode(drivenNodeName);
            drivenPart = node == null ? null : node.attachedPart;
            if (drivenPart == null)
            {
                motorStatus = "Top end is empty";
                Debug.LogWarning("[KerbalLiDAR] ROS linear actuator top end has no attached part: " + JointName);
                return;
            }

            foreach (var slider in pJoint.joints)
            {
                if (slider != null)
                    KerbalRosMotorCollisions.EnableBetweenConnectedParts(part, drivenPart, slider);
            }
            CacheMovingContactPairs();

            referenceLocalDrivenPosition = part.transform.InverseTransformPoint(drivenPart.transform.position);
            // A command may arrive while KSP is still creating the joint.
            // Its reference is the actual loaded pose, not the new target.
            referenceExtension = currentExtension;
            currentExtension = referenceExtension;
            previousMeasuredExtension = currentExtension;
            driveSign = 1f;
            driveSignCalibrated = false;
            ApplyMotorMode();
            motorStatus = "Ready";
        }

        protected override void OnMotorModeChanged(Mode newMode)
        {
            RefreshEventLabels();
        }

        public void Update()
        {
            if (!HighLogic.LoadedSceneIsEditor || part == null)
            {
                return;
            }

            var requestedExtension = ClampExtension(targetExtension);
            targetExtension = requestedExtension;
            var delta = requestedExtension - editorAppliedExtension;
            if (Mathf.Abs(delta) <= 0.0001f)
            {
                return;
            }

            TranslateEditorDrivenBranch(delta);
            editorAppliedExtension = requestedExtension;
            currentExtension = requestedExtension;
            SetVisualExtension(requestedExtension);
            UpdateDrivenNode(requestedExtension);
            if (EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
            }
        }

        internal void ManagedUpdate(float elapsed)
        {
            if (!HighLogic.LoadedSceneIsFlight)
            {
                return;
            }

            if (lastManagedFrame == Time.frameCount)
            {
                return;
            }

            lastManagedFrame = Time.frameCount;
            elapsed = Mathf.Max(elapsed, 0.0001f);
            UpdateVelocityCommand(elapsed);
            EnsureFlightJoint();
            UpdateFlightMotor();
            UpdateStateVelocity(elapsed);
        }

        public void OnDestroy()
        {
            KerbalRosMotorUdpManager.Unregister(this);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Disengage Motor", active = true)]
        public void ToggleMotor()
        {
            motorEngaged = !motorEngaged;
            ApplyMotorMode();
            RefreshEventLabels();
        }

        [KSPEvent(guiActive = true, guiName = "Lock Motor", active = true)]
        public void ToggleLock()
        {
            motorLocked = !motorLocked;
            ApplyMotorMode();
            RefreshEventLabels();
        }

        void IKerbalRosMotor.ApplyRosCommand(KerbalRosMotorCommand command)
        {
            if (command == null)
            {
                return;
            }

            if (command.hasEnabled)
            {
                motorEngaged = command.enabled;
                if (!motorEngaged)
                {
                    velocityMode = false;
                    commandActive = false;
                    targetExtension = currentExtension;
                    ApplyMotorMode();
                    RefreshEventLabels();
                    rosCommandState = "Disabled";
                    return;
                }
            }

            if (command.hasEffort && IsFinite(command.effort))
            {
                effortLimitN = Mathf.Clamp((float)Math.Abs(command.effort), 0f, Mathf.Max(0f, maxForce));
                ConfigureDrive();
            }

            if (!motorEngaged)
            {
                motorEngaged = true;
                ApplyMotorMode();
                RefreshEventLabels();
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
                commandedVelocity = Mathf.Clamp(
                    (float)command.velocity,
                    -Mathf.Max(0f, maxTraverseVelocity),
                    Mathf.Max(0f, maxTraverseVelocity));
                rosCommandState = "Velocity";
                return;
            }

            velocityMode = false;
            if (command.hasPosition && IsFinite(command.position))
            {
                targetExtension = ClampExtension((float)command.position);
            }

            if (command.hasVelocity && IsFinite(command.velocity) && Math.Abs(command.velocity) > 0.000001)
            {
                traverseVelocity = Mathf.Clamp(
                    (float)Math.Abs(command.velocity),
                    Mathf.Max(0f, minTraverseVelocity),
                    Mathf.Max(minTraverseVelocity, maxTraverseVelocity));
            }

            rosCommandState = "Position";
        }

        string IKerbalRosMotor.BuildStatePacket()
        {
            ManagedUpdate(Mathf.Max(Time.deltaTime, 1f / Mathf.Max(1f, stateRateHz)));
            var error = targetExtension - currentExtension;
            var effortN = Mathf.Abs(error) <= extensionTolerance || !motorEngaged
                ? 0f
                : Mathf.Sign(error) * effortLimitN;
            var currentAmps = forcePerAmpN > 0.001f ? Mathf.Abs(effortN) / forcePerAmpN : 0f;
            return KerbalRosMotorJson.State(
                this,
                currentExtension,
                stateVelocity,
                effortN,
                currentAmps,
                targetExtension,
                true,
                motorEngaged,
                motorLocked,
                activeCommandMode,
                commandActive);
        }

        private void UpdateFlightMotor()
        {
            targetExtension = ClampExtension(targetExtension);
            if (!jointInitialized || joint == null || drivenPart == null)
            {
                motorStatus = "No driven joint";
                return;
            }

            EnsureJointConfiguration();
            ExcludeMovingOutputContacts();
            currentExtension = MeasureCurrentExtension();
            CalibrateDriveSign();
            var error = targetExtension - currentExtension;
            var logicalSpeed = 0f;
            if (motorEngaged && !motorLocked && Mathf.Abs(error) > Mathf.Max(0.0001f, extensionTolerance))
            {
                logicalSpeed = Mathf.Clamp(
                    error * Mathf.Max(0.01f, positionGain),
                    -Mathf.Max(0f, traverseVelocity),
                    Mathf.Max(0f, traverseVelocity));
                motorStatus = "Driving " + logicalSpeed.ToString("F3") + " m/s";
            }
            else
            {
                motorStatus = motorLocked ? "Locked" : motorEngaged ? "Holding" : "Disengaged";
            }

            SetLinearMotorSpeed(logicalSpeed * driveSign);
            lastLogicalSpeed = logicalSpeed;
            SetVisualExtension(currentExtension);
            UpdateDrivenNode(currentExtension);
            PreserveDrivenBranchPose();
        }

        private float MeasureCurrentExtension()
        {
            var outputNode = drivenPart.FindAttachNodeByPart(part);
            if (outputNode != null && drivenNode != null)
            {
                // Measure mating faces, not a saved target/reference value.
                // KSP can restore an older craft pose on reload or unpack.
                var outputFace = part.transform.InverseTransformPoint(
                    drivenPart.transform.TransformPoint(outputNode.position));
                return Vector3.Dot(outputFace - contractedNodePosition, Vector3.up);
            }
            var localPosition = part.transform.InverseTransformPoint(drivenPart.transform.position);
            return referenceExtension + Vector3.Dot(localPosition - referenceLocalDrivenPosition, Vector3.up);
        }

        private void CalibrateDriveSign()
        {
            var measuredDelta = currentExtension - previousMeasuredExtension;
            if (!driveSignCalibrated && Mathf.Abs(lastLogicalSpeed) > 0.01f && Mathf.Abs(measuredDelta) > 0.0001f)
            {
                if (Mathf.Sign(measuredDelta) != Mathf.Sign(lastLogicalSpeed))
                {
                    driveSign = -driveSign;
                    Debug.Log("[KerbalLiDAR] ROS linear actuator automatically reversed its physical drive sign: " + JointName);
                }

                driveSignCalibrated = true;
            }

            previousMeasuredExtension = currentExtension;
        }

        private void ApplyMotorMode()
        {
            if (!jointInitialized || joint == null)
            {
                return;
            }

            SetMotorMode(motorLocked ? Mode.Park : motorEngaged ? Mode.Drive : Mode.Neutral);
            EnsureJointConfiguration();
            ConfigureDrive();
            if (!motorEngaged || motorLocked)
            {
                SetLinearMotorSpeed(0f);
            }
        }

        private void UpdateVelocityCommand(float elapsed)
        {
            if (commandActive && Time.realtimeSinceStartup > commandExpiresAt)
            {
                commandActive = false;
                velocityMode = false;
                motorEngaged = false;
                targetExtension = currentExtension;
                rosCommandState = "Command timeout";
                ApplyMotorMode();
                RefreshEventLabels();
                return;
            }

            if (velocityMode)
            {
                targetExtension = ClampExtension(targetExtension + commandedVelocity * elapsed);
            }
        }

        private void UpdateStateVelocity(float elapsed)
        {
            if (!stateVelocityInitialized)
            {
                previousStateExtension = currentExtension;
                stateVelocityInitialized = true;
                return;
            }

            var instantaneous = (currentExtension - previousStateExtension) / elapsed;
            stateVelocity = Mathf.Lerp(stateVelocity, instantaneous, 0.35f);
            previousStateExtension = currentExtension;
        }

        private void TranslateEditorDrivenBranch(float delta)
        {
            var drivenNode = part.FindAttachNode(drivenNodeName);
            if (drivenNode == null || drivenNode.attachedPart == null || drivenNode.attachedPart.parent != part)
            {
                return;
            }

            var translation = part.transform.TransformDirection(Vector3.up).normalized * delta;
            var drivenParts = new List<Part>();
            CollectPartBranch(drivenNode.attachedPart, drivenParts);
            var originalPositions = new Vector3[drivenParts.Count];
            for (var index = 0; index < drivenParts.Count; index++)
            {
                originalPositions[index] = drivenParts[index].transform.position;
            }

            for (var index = 0; index < drivenParts.Count; index++)
            {
                drivenParts[index].transform.position = originalPositions[index] + translation;
            }
        }

        private static void CollectPartBranch(Part root, List<Part> result)
        {
            if (root == null || result.Contains(root))
            {
                return;
            }

            result.Add(root);
            for (var index = 0; index < root.children.Count; index++)
            {
                CollectPartBranch(root.children[index], result);
            }
        }

        private void EnsureVisuals()
        {
            if (part == null)
            {
                return;
            }

            // Native assets include the moving hierarchy in the prefab, so it is
            // present during icon generation, editor cloning and flight loading.
            if (string.IsNullOrEmpty(movingModelPath))
            {
                if (part.FindModelTransform(movingTransformName) == null)
                    Debug.LogError("[KerbalLiDAR] Linear actuator model is missing " + movingTransformName);
                return;
            }

            var target = KerbalRosMotorVisuals.EnsureTransform(part, movingTransformName);
            if (target == null)
            {
                Debug.LogError("[KerbalLiDAR] ROS linear actuator could not create its moving transform.");
                return;
            }

            KerbalRosMotorVisuals.AddStockModel(
                part,
                target,
                movingModelPath,
                movingModelPosition,
                movingModelRotation,
                movingModelScale);
        }

        private void CacheMovingTransform()
        {
            if (movingTransformReady || part == null)
            {
                return;
            }

            movingTransform = part.FindModelTransform(movingTransformName);
            if (movingTransform != null)
            {
                // Native stage origins are zero. An editor clone can already be
                // extended; treating that pose as rest would apply travel twice.
                movingTransformRestPosition = string.IsNullOrEmpty(movingModelPath)
                    ? Vector3.zero : movingTransform.localPosition;
            }

            intermediateTransform = part.FindModelTransform(intermediateTransformName);
            if (intermediateTransform != null)
                intermediateRestPosition = Vector3.zero;
            // Retry if the prefab hierarchy is not available yet.
            movingTransformReady = movingTransform != null;
        }

        private void SetVisualExtension(float extension)
        {
            CacheMovingTransform();
            if (movingTransform != null)
            {
                movingTransform.localPosition = movingTransformRestPosition + Vector3.up * ClampExtension(extension);
            }
            if (intermediateTransform != null)
            {
                intermediateTransform.localPosition = intermediateRestPosition + Vector3.up * (0.5f * ClampExtension(extension));
            }
        }

        private void RefreshEventLabels()
        {
            if (Events == null)
            {
                return;
            }

            Events[nameof(ToggleMotor)].guiName = motorEngaged ? "Disengage Motor" : "Engage Motor";
            Events[nameof(ToggleLock)].guiName = motorLocked ? "Unlock Motor" : "Lock Motor";
        }

        private float ClampExtension(float extension)
        {
            return Mathf.Clamp(extension, minExtension, Mathf.Max(minExtension, maxExtension));
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
