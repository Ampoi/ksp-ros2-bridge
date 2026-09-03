using System;
using System.Collections.Generic;
using UnityEngine;

namespace KerbalLiDAR
{
    /// <summary>
    /// DLC-independent axial position servo built on KSP's working velocity
    /// motor. In the editor it remains a passive two-node cylinder. In flight
    /// it closes a position loop around the top stack joint.
    /// </summary>
    public sealed class ModuleKerbalRosServo : ModuleJointMotor, IKerbalRosMotor
    {
        [KSPField(isPersistant = true)]
        public string motorName = "";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Target Angle", guiUnits = "°")]
        [UI_FloatRange(minValue = -180f, maxValue = 180f, stepIncrement = 1f)]
        public float targetAngle;

        [KSPField] public float minAngle = -180f;
        [KSPField] public float maxAngle = 180f;
        [KSPField] public float traverseVelocity = 45f;
        [KSPField] public float positionGain = 3f;
        [KSPField] public float angleTolerance = 0.25f;
        [KSPField] public float jointDamper = 100f;
        [KSPField] public float maxTorque = 250f;
        [KSPField] public string drivenNodeName = "top";
        [KSPField] public string rotatingTransformName = "port";
        [KSPField(isPersistant = true)] public bool motorEngaged = true;
        [KSPField(isPersistant = true)] public bool motorLocked;
        [KSPField] public int commandUdpPort = 49011;
        [KSPField] public string stateUdpHost = "127.0.0.1";
        [KSPField] public int stateUdpPort = 49010;
        [KSPField] public float stateRateHz = 20f;
        [KSPField] public float commandTimeoutSeconds = 0.5f;
        [KSPField] public float ratedEffortNm = 250f;
        [KSPField] public float torquePerAmpNm = 40f;

        [KSPField(guiActive = true, guiName = "ROS Joint")]
        public string rosJointDisplay = "";

        [KSPField(guiActive = true, guiName = "ROS Command")]
        public string rosCommandState = "Waiting";

        [KSPField(guiActive = true, guiName = "Current Angle", guiUnits = "°", guiFormat = "F1")]
        public float currentAngle;

        [KSPField(guiActive = true, guiName = "Motor Status")]
        public string motorStatus = "Waiting for joint";

        private Transform rotatingTransform;
        private Quaternion rotatingTransformRestRotation = Quaternion.identity;
        private bool rotatingTransformReady;
        private bool jointInitialized;
        private Part drivenPart;
        private Quaternion referenceRelativeRotation = Quaternion.identity;
        private float referenceAngle;
        private bool velocityMode;
        private float commandedVelocityDegrees;
        private float commandExpiresAt;
        private bool commandActive;
        private string activeCommandMode = "position";
        private float editorAppliedAngle;
        private float previousStateAngle;
        private float stateVelocityDegrees;
        private bool stateVelocityInitialized;
        private float effortLimitNm;
        private float driveSign = 1f;
        private bool driveSignCalibrated;
        private float previousMeasuredAngle;
        private float lastLogicalSpeedDegrees;
        private float stalledSince = -1f;
        private float nextDiagnosticTime;
        private int lastManagedFrame = -1;

        Part IKerbalRosMotor.MotorPart { get { return part; } }
        string IKerbalRosMotor.JointName { get { return JointName; } }
        string IKerbalRosMotor.JointType { get { return "revolute"; } }
        int IKerbalRosMotor.CommandUdpPort { get { return commandUdpPort; } }
        string IKerbalRosMotor.StateUdpHost { get { return stateUdpHost; } }
        int IKerbalRosMotor.StateUdpPort { get { return stateUdpPort; } }
        float IKerbalRosMotor.StateRateHz { get { return stateRateHz; } }

        private string JointName
        {
            get { return KerbalRosMotorNames.Resolve(motorName, "servo", part, this); }
        }

        protected override void OnModuleStart(StartState state)
        {
            jointNodeName = drivenNodeName;
            targetAngle = ClampAngle(targetAngle);
            editorAppliedAngle = targetAngle;
            currentAngle = targetAngle;
            effortLimitNm = Mathf.Max(0f, Mathf.Min(maxTorque, ratedEffortNm));
            rosJointDisplay = JointName;
            CacheRotatingTransform();
            SetVisualAngle(targetAngle);

            if (HighLogic.LoadedSceneIsFlight)
            {
                KerbalRosMotorUdpManager.Register(this);
            }

            RefreshEventLabels();
        }

        protected override void OnModuleLoad(ConfigNode node)
        {
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
                Debug.LogWarning("[KerbalLiDAR] ROS axial servo has no physical joint at node '" + drivenNodeName + "': " + JointName);
                return;
            }

            var node = part.FindAttachNode(drivenNodeName);
            drivenPart = node == null ? null : node.attachedPart;
            if (drivenPart == null)
            {
                motorStatus = "Top face is empty";
                Debug.LogWarning("[KerbalLiDAR] ROS axial servo top face has no attached part: " + JointName);
                return;
            }

            KerbalRosMotorCollisions.EnableBetweenConnectedParts(part, drivenPart, joint);

            // Stack-node orientation is the ConfigurableJoint X axis. Free only
            // that twist axis and keep both swing axes rigid.
            joint.angularXMotion = ConfigurableJointMotion.Free;
            joint.angularYMotion = ConfigurableJointMotion.Locked;
            joint.angularZMotion = ConfigurableJointMotion.Locked;
            joint.rotationDriveMode = RotationDriveMode.XYAndZ;

            referenceRelativeRotation = Quaternion.Inverse(part.transform.rotation) * drivenPart.transform.rotation;
            referenceAngle = targetAngle;
            currentAngle = referenceAngle;
            previousMeasuredAngle = currentAngle;
            driveSign = 1f;
            driveSignCalibrated = false;
            ApplyMotorMode();
            motorStatus = "Ready";
            LogJointDiagnostics("initialized");
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

            var requestedAngle = ClampAngle(targetAngle);
            targetAngle = requestedAngle;
            var delta = Mathf.DeltaAngle(editorAppliedAngle, requestedAngle);
            if (Mathf.Abs(delta) <= 0.001f)
            {
                return;
            }

            RotateEditorDrivenBranch(delta);
            editorAppliedAngle = requestedAngle;
            currentAngle = requestedAngle;
            SetVisualAngle(requestedAngle);
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
                    targetAngle = currentAngle;
                    ApplyMotorMode();
                    RefreshEventLabels();
                    rosCommandState = "Disabled";
                    return;
                }
            }

            if (command.hasEffort && IsFinite(command.effort))
            {
                effortLimitNm = Mathf.Clamp((float)Math.Abs(command.effort), 0f, Mathf.Max(0f, maxTorque));
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
                    targetAngle = currentAngle;
                    rosCommandState = "Velocity hold";
                    return;
                }

                velocityMode = true;
                commandedVelocityDegrees = (float)(command.velocity * Mathf.Rad2Deg);
                rosCommandState = "Velocity";
                return;
            }

            velocityMode = false;
            if (command.hasPosition && IsFinite(command.position))
            {
                targetAngle = Mathf.Clamp((float)(command.position * Mathf.Rad2Deg), minAngle, maxAngle);
            }

            if (command.hasVelocity && IsFinite(command.velocity) && Math.Abs(command.velocity) > 0.000001)
            {
                traverseVelocity = Mathf.Clamp((float)Math.Abs(command.velocity * Mathf.Rad2Deg), 0.1f, 360f);
            }

            rosCommandState = "Position";
        }

        string IKerbalRosMotor.BuildStatePacket()
        {
            // State generation is a guaranteed manager callback at stateRateHz.
            // Keep this fallback so motor control does not depend on any
            // particular PartModule/MonoBehaviour update dispatch path.
            ManagedUpdate(Mathf.Max(Time.deltaTime, 1f / Mathf.Max(1f, stateRateHz)));
            var error = Mathf.DeltaAngle(currentAngle, targetAngle);
            var effortNm = Mathf.Abs(error) <= angleTolerance || !motorEngaged
                ? 0f
                : Mathf.Sign(error) * effortLimitNm;
            var currentAmps = torquePerAmpNm > 0.001f ? Mathf.Abs(effortNm) / torquePerAmpNm : 0f;
            return KerbalRosMotorJson.State(
                this,
                currentAngle * Mathf.Deg2Rad,
                stateVelocityDegrees * Mathf.Deg2Rad,
                effortNm,
                currentAmps,
                targetAngle * Mathf.Deg2Rad,
                true,
                motorEngaged,
                motorLocked,
                activeCommandMode,
                commandActive);
        }

        private void UpdateFlightMotor()
        {
            targetAngle = ClampAngle(targetAngle);
            if (!jointInitialized || joint == null || drivenPart == null)
            {
                motorStatus = "No driven joint";
                return;
            }

            EnsureJointConfiguration();
            currentAngle = MeasureCurrentAngle();
            CalibrateDriveSign();
            var error = Mathf.DeltaAngle(currentAngle, targetAngle);
            var logicalSpeedDegrees = 0f;
            if (motorEngaged && !motorLocked && Mathf.Abs(error) > Mathf.Max(0.01f, angleTolerance))
            {
                logicalSpeedDegrees = Mathf.Clamp(error * Mathf.Max(0.01f, positionGain), -traverseVelocity, traverseVelocity);
                motorStatus = "Driving " + logicalSpeedDegrees.ToString("F1") + "°/s";
            }
            else
            {
                motorStatus = motorLocked ? "Locked" : motorEngaged ? "Holding" : "Disengaged";
            }

            SetMotorSpeed(logicalSpeedDegrees * driveSign * Mathf.Deg2Rad);
            lastLogicalSpeedDegrees = logicalSpeedDegrees;
            SetVisualAngle(currentAngle);
            DiagnoseStall(error, logicalSpeedDegrees);
        }

        private float MeasureCurrentAngle()
        {
            var relative = Quaternion.Inverse(part.transform.rotation) * drivenPart.transform.rotation;
            var change = relative * Quaternion.Inverse(referenceRelativeRotation);
            return ClampAngle(referenceAngle + SignedTwistAngleDegrees(change, Vector3.up));
        }

        private void CalibrateDriveSign()
        {
            var measuredDelta = Mathf.DeltaAngle(previousMeasuredAngle, currentAngle);
            if (!driveSignCalibrated && Mathf.Abs(lastLogicalSpeedDegrees) > 1f && Mathf.Abs(measuredDelta) > 0.02f)
            {
                if (Mathf.Sign(measuredDelta) != Mathf.Sign(lastLogicalSpeedDegrees))
                {
                    driveSign = -driveSign;
                    Debug.Log("[KerbalLiDAR] ROS axial servo automatically reversed its physical drive sign: " + JointName);
                }

                driveSignCalibrated = true;
            }

            previousMeasuredAngle = currentAngle;
        }

        private void ApplyMotorMode()
        {
            if (!jointInitialized || joint == null)
            {
                return;
            }

            SetMotorMode(motorLocked ? Mode.Park : motorEngaged ? Mode.Drive : Mode.Neutral);
            ConfigureDrive();
            if (!motorEngaged || motorLocked)
            {
                SetMotorSpeed(0f);
            }
        }

        private void ConfigureDrive()
        {
            if (!jointInitialized || joint == null)
            {
                return;
            }

            var drive = joint.angularXDrive;
            drive.positionSpring = 0f;
            drive.positionDamper = motorEngaged ? Mathf.Max(0f, jointDamper) : 0f;
            drive.maximumForce = motorEngaged ? Mathf.Max(0f, effortLimitNm) : 0f;
            joint.angularXDrive = drive;
        }

        private void EnsureJointConfiguration()
        {
            // KSP reapplies ordinary PartJoint limits when the vessel unpacks
            // and when autostruts are rebuilt. That happens after OnJointInit,
            // so enforce the motor freedom and drive here as well.
            var configurationChanged = joint.angularXMotion != ConfigurableJointMotion.Free ||
                                       joint.angularYMotion != ConfigurableJointMotion.Locked ||
                                       joint.angularZMotion != ConfigurableJointMotion.Locked;
            if (configurationChanged)
            {
                joint.angularXMotion = ConfigurableJointMotion.Free;
                joint.angularYMotion = ConfigurableJointMotion.Locked;
                joint.angularZMotion = ConfigurableJointMotion.Locked;
                joint.rotationDriveMode = RotationDriveMode.XYAndZ;
                Debug.Log("[KerbalLiDAR] ROS axial servo restored motor freedom after KSP joint rebuild: " + JointName);
            }

            var drive = joint.angularXDrive;
            var wantedDamper = motorEngaged ? Mathf.Max(0f, jointDamper) : 0f;
            var wantedForce = motorEngaged ? Mathf.Max(0f, effortLimitNm) : 0f;
            if (configurationChanged ||
                Mathf.Abs(drive.positionSpring) > 0.001f ||
                Mathf.Abs(drive.positionDamper - wantedDamper) > 0.001f ||
                Mathf.Abs(drive.maximumForce - wantedForce) > 0.001f)
            {
                drive.positionSpring = 0f;
                drive.positionDamper = wantedDamper;
                drive.maximumForce = wantedForce;
                joint.angularXDrive = drive;
            }
        }

        private void UpdateVelocityCommand(float elapsed)
        {
            if (commandActive && Time.realtimeSinceStartup > commandExpiresAt)
            {
                commandActive = false;
                velocityMode = false;
                motorEngaged = false;
                targetAngle = currentAngle;
                rosCommandState = "Command timeout";
                ApplyMotorMode();
                RefreshEventLabels();
                return;
            }

            if (!velocityMode)
            {
                return;
            }

            if (Time.realtimeSinceStartup > commandExpiresAt)
            {
                velocityMode = false;
                targetAngle = currentAngle;
                rosCommandState = "Velocity timeout";
                return;
            }

            targetAngle = Mathf.Clamp(targetAngle + commandedVelocityDegrees * elapsed, minAngle, maxAngle);
        }

        private void UpdateStateVelocity(float elapsed)
        {
            if (!stateVelocityInitialized)
            {
                previousStateAngle = currentAngle;
                stateVelocityInitialized = true;
                return;
            }

            var instantaneous = Mathf.DeltaAngle(previousStateAngle, currentAngle) / elapsed;
            stateVelocityDegrees = Mathf.Lerp(stateVelocityDegrees, instantaneous, 0.35f);
            previousStateAngle = currentAngle;
        }

        private void DiagnoseStall(float error, float commandedSpeed)
        {
            if (Mathf.Abs(error) <= 1f || Mathf.Abs(commandedSpeed) <= 1f || Mathf.Abs(stateVelocityDegrees) > 0.05f)
            {
                stalledSince = -1f;
                return;
            }

            if (stalledSince < 0f)
            {
                stalledSince = Time.realtimeSinceStartup;
            }

            if (Time.realtimeSinceStartup - stalledSince >= 1f && Time.realtimeSinceStartup >= nextDiagnosticTime)
            {
                nextDiagnosticTime = Time.realtimeSinceStartup + 2f;
                LogJointDiagnostics("stalled target=" + targetAngle.ToString("F1") + " current=" + currentAngle.ToString("F1") + " cmd=" + commandedSpeed.ToString("F1"));
            }
        }

        private void LogJointDiagnostics(string reason)
        {
            if (joint == null)
            {
                Debug.LogWarning("[KerbalLiDAR] ROS axial servo " + reason + ": Unity joint is null, " + JointName);
                return;
            }

            var drive = joint.angularXDrive;
            Debug.Log(
                "[KerbalLiDAR] ROS axial servo " + reason + ": " + JointName +
                " node=" + drivenNodeName +
                " attached=" + (drivenPart == null ? "null" : drivenPart.name) +
                " axis=" + joint.axis +
                " secondary=" + joint.secondaryAxis +
                " motion=" + joint.angularXMotion + "/" + joint.angularYMotion + "/" + joint.angularZMotion +
                " drive=" + drive.positionSpring.ToString("F1") + "/" + drive.positionDamper.ToString("F1") + "/" + drive.maximumForce.ToString("F1") +
                " targetW=" + joint.targetAngularVelocity +
                " body=" + joint.gameObject.name + "/kinematic=" + (joint.GetComponent<Rigidbody>() == null ? "null" : joint.GetComponent<Rigidbody>().isKinematic.ToString()) +
                " connected=" + (joint.connectedBody == null ? "null" : joint.connectedBody.name));
        }

        private void RotateEditorDrivenBranch(float deltaDegrees)
        {
            var drivenNode = part.FindAttachNode(drivenNodeName);
            if (drivenNode == null || drivenNode.attachedPart == null || drivenNode.attachedPart.parent != part)
            {
                return;
            }

            var pivot = part.transform.TransformPoint(drivenNode.position);
            var axis = part.transform.TransformDirection(Vector3.up).normalized;
            var rotation = Quaternion.AngleAxis(deltaDegrees, axis);
            var drivenParts = new List<Part>();
            CollectPartBranch(drivenNode.attachedPart, drivenParts);

            // KSP can parent editor part transforms so moving an ancestor may
            // immediately move its descendants. Snapshot the whole branch
            // before applying anything; otherwise deeper parts receive the
            // same delta once per ancestor and the assembly fans apart.
            var originalPositions = new Vector3[drivenParts.Count];
            var originalRotations = new Quaternion[drivenParts.Count];
            for (var index = 0; index < drivenParts.Count; index++)
            {
                originalPositions[index] = drivenParts[index].transform.position;
                originalRotations[index] = drivenParts[index].transform.rotation;
            }

            for (var index = 0; index < drivenParts.Count; index++)
            {
                var child = drivenParts[index];
                child.transform.position = pivot + rotation * (originalPositions[index] - pivot);
                child.transform.rotation = rotation * originalRotations[index];
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

        private void CacheRotatingTransform()
        {
            if (rotatingTransformReady || part == null)
            {
                return;
            }

            rotatingTransform = part.FindModelTransform(rotatingTransformName);
            if (rotatingTransform != null)
            {
                rotatingTransformRestRotation = rotatingTransform.localRotation;
            }

            rotatingTransformReady = true;
        }

        private void SetVisualAngle(float angle)
        {
            CacheRotatingTransform();
            if (rotatingTransform != null)
            {
                rotatingTransform.localRotation = rotatingTransformRestRotation * Quaternion.AngleAxis(angle, Vector3.up);
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

        private float ClampAngle(float angle)
        {
            return Mathf.Clamp(Mathf.DeltaAngle(0f, angle), minAngle, maxAngle);
        }

        private static float SignedTwistAngleDegrees(Quaternion rotation, Vector3 axis)
        {
            axis.Normalize();
            var projected = rotation.x * axis.x + rotation.y * axis.y + rotation.z * axis.z;
            var angle = 2f * Mathf.Atan2(projected, rotation.w) * Mathf.Rad2Deg;
            return Mathf.DeltaAngle(0f, angle);
        }

        private static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
