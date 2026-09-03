using System;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar : PartModule
    {
        private const float HemisphereSolidAngleSteradians = 6.2831853f;
        private const float GoldenAngleRadians = 2.3999632f;

        [KSPField]
        public string sensorMode = "2D";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Horizontal Lasers", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 2048f, stepIncrement = 1f)]
        public float horizontalLaserCount = 180f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Vertical Lasers", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 128f, stepIncrement = 1f)]
        public float verticalLaserCount = 1f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Hemisphere Density / sr", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 1024f, stepIncrement = 1f)]
        public float hemisphereDensity = 160f;

        [KSPField]
        public float horizontalFovDegrees = 360f;

        [KSPField]
        public float verticalFovDegrees = 0f;

        [KSPField]
        public float maxDistance = 2000f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Near Range m", guiFormat = "F0")]
        [UI_FloatRange(minValue = 10f, maxValue = 30f, stepIncrement = 5f)]
        public float nearRangeMeters = 30f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Medium Range m", guiFormat = "F0")]
        [UI_FloatRange(minValue = 50f, maxValue = 150f, stepIncrement = 10f)]
        public float mediumRangeMeters = 100f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Long Range m", guiFormat = "F0")]
        [UI_FloatRange(minValue = 150f, maxValue = 250f, stepIncrement = 10f)]
        public float longRangeMeters = 250f;

        [KSPField(isPersistant = true)]
        public string rangeProfile = "medium";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Scan Rate Hz", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 60f, stepIncrement = 1f)]
        public float scanRateHz = 10f;

        [KSPField]
        public string udpHost = "127.0.0.1";

        [KSPField]
        public int udpPort = 49010;

        [KSPField(isPersistant = true)]
        public string partName = "";

        // Kept for loading craft files made before partName was introduced.
        [KSPField(isPersistant = true, guiActive = true, guiName = "Active Vessel URDF")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool activeVesselUrdfEnabled = true;

        [KSPField]
        public float activeVesselUrdfRefreshSeconds = 2f;

        [KSPField]
        public int activeVesselUrdfChunkBytes = 12000;

        [KSPField]
        public int maxActiveVesselUrdfChunks = 256;

        [KSPField]
        public bool allowRemoteUrdf = false;

        [KSPField(isPersistant = true)]
        public string lidarName = "";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "LiDAR")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool lidarEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "UDP")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool udpEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "20 FPS Stream")]
        [UI_Toggle(enabledText = "On", disabledText = "Off")]
        public bool streamAt20Fps = false;

        [KSPField]
        public bool ignoreOwnVessel = true;

        [KSPField]
        public bool includeHitPoints = false;

        [KSPField]
        public bool includeDirections = false;

        [KSPField]
        public bool drawDebugRays = false;

        [KSPField]
        public float debugRaySeconds = 0.05f;

        [KSPField(isPersistant = true)]
        public bool radarLinesVisible = false;

        [KSPField]
        public float radarLineWidth = 0.08f;

        [KSPField]
        public float originOffsetMeters = 0.05f;

        [KSPField]
        public float noHitValue = -1f;

        [KSPField]
        public int maxLaserCount = 4096;

        [KSPField]
        public int maxDatagramBytes = 60000;

        [KSPField]
        public string rayOriginName = "";

        [KSPField]
        public string rayOriginLocalPosition = "0, 0, 0";

        [KSPField]
        public string forwardAxis = "-Y";

        [KSPField]
        public bool align3DToAttachNormal = true;

        [KSPField]
        public string upAxis = "+Z";

        [KSPField]
        public string visualModelObjPath = "";

        [KSPField]
        public string visualModelTexturePath = "";

        [KSPField]
        public float visualModelScale = 1f;

        [KSPField]
        public string visualModelPosition = "0, 0, 0";

        [KSPField]
        public string visualModelRotation = "0, 0, 0";

        [KSPField]
        public bool visualModelDoubleSided = true;

        [KSPField(guiActive = true, guiName = "Last Rays")]
        public int lastRayCount = 0;

        [KSPField(guiActive = true, guiName = "Last Hits")]
        public int lastHitCount = 0;

        [KSPField(guiActive = true, guiName = "Last Scan ms", guiFormat = "F1")]
        public float lastScanMs = 0f;

        [KSPField(guiActive = true, guiActiveEditor = true, guiName = "Ray Budget")]
        public string rayBudget = "";

        private float nextScanTime;
        private bool flightTopicActive;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            NormalizeConfig();
            LoadConfiguredVisualModel();
            UpdateUi();

        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            NormalizeConfig();
            LoadConfiguredVisualModel();
        }

        public override void OnFixedUpdate()
        {
            ProcessFlightStreaming();
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            EnsureConfiguredVisualModel();

            if (HighLogic.LoadedSceneIsFlight)
            {
                ProcessFlightStreaming();
                return;
            }

            if (!HighLogic.LoadedSceneIsEditor)
            {
                return;
            }

            if (radarLinesVisible)
            {
                UpdateRadarPreviewLines();
            }
            else
            {
                ClearRadarLines();
            }
        }

        private void ProcessFlightStreaming()
        {
            if (!HighLogic.LoadedSceneIsFlight)
            {
                return;
            }

            ProcessActiveVesselUrdf();

            if (!lidarEnabled && !radarLinesVisible)
            {
                ClearRadarLines();
                return;
            }

            var effectiveScanRate = streamAt20Fps ? 20f : scanRateHz;
            var interval = effectiveScanRate > 0.01f ? 1f / effectiveScanRate : 1f;
            var now = Time.realtimeSinceStartup;
            if (now < nextScanTime)
            {
                return;
            }

            nextScanTime = now + interval;
            ScanAndSend(lidarEnabled);
        }

        [KSPEvent(guiActive = true, guiName = "Send LiDAR Scan Now", active = true)]
        public void SendScanNow()
        {
            if (HighLogic.LoadedSceneIsFlight)
            {
                ScanAndSend();
            }
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Show Laser Preview", active = true)]
        public void ToggleRadarLines()
        {
            radarLinesVisible = !radarLinesVisible;
            if (!radarLinesVisible)
            {
                ClearRadarLines();
            }
            else if (HighLogic.LoadedSceneIsFlight)
            {
                ScanAndSend(lidarEnabled);
            }
            else if (HighLogic.LoadedSceneIsEditor)
            {
                UpdateRadarPreviewLines();
            }

            UpdateUi();
        }

        [KSPAction("Toggle Laser Preview")]
        public void ToggleRadarLinesAction(KSPActionParam action)
        {
            ToggleRadarLines();
        }

        public void OnDestroy()
        {
            DestroyRadarLines();
            DestroyVisualModel();
            SendFlightTopicInactive();
            ClearActiveVesselUrdf();
            CloseUdpClient();
        }

        private void ScanAndSend()
        {
            ScanAndSend(true);
        }

        private void ScanAndSend(bool sendUdp)
        {
            NormalizeConfig();

            var started = Time.realtimeSinceStartup;
            var originTransform = ResolveOriginTransform();
            var forward = ResolveScanForward(originTransform);
            var up = AxisToWorld(originTransform, upAxis);
            Orthonormalize(ref forward, ref up, originTransform);

            var origin = ResolveScanOrigin(originTransform, forward);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);

            var scan = BuildScanPacket(origin, forward, up, horizontalCount, verticalCount);
            lastScanMs = (Time.realtimeSinceStartup - started) * 1000f;

            if (sendUdp && udpEnabled)
            {
                SendUdp(scan);
                flightTopicActive = true;
            }
        }

        private void SendFlightTopicInactive()
        {
            if (!flightTopicActive)
            {
                return;
            }

            packetBuilder.Length = 0;
            packetBuilder.Append('{');
            AppendProperty(packetBuilder, "type", "ksp_lidar_inactive", true);
            AppendProperty(packetBuilder, "version", JsonVersion, false);
            AppendProperty(packetBuilder, "mode", Is3DMode() ? "3D" : "2D", false);
            AppendProperty(packetBuilder, "sensorId", ResolveSensorId(), false);
            AppendProperty(packetBuilder, "vessel", vessel != null ? vessel.vesselName : "", false);
            AppendProperty(packetBuilder, "partFlightId", part != null ? (long)part.flightID : 0L, false);
            packetBuilder.Append('}');

            SendUdp(packetBuilder.ToString());
            flightTopicActive = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: Low", active = true)]
        public void SetLowPreset()
        {
            if (Is3DMode())
            {
                Set3DRangeProfile("near");
                return;
            }

            SetPreset(90f, 1f);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: Medium", active = true)]
        public void SetMediumPreset()
        {
            if (Is3DMode())
            {
                Set3DRangeProfile("medium");
                return;
            }

            SetPreset(180f, 1f);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: High", active = true)]
        public void SetHighPreset()
        {
            if (Is3DMode())
            {
                Set3DRangeProfile("long");
                return;
            }

            SetPreset(360f, 1f);
        }

        private void SetPreset(float horizontal, float vertical)
        {
            horizontalLaserCount = horizontal;
            verticalLaserCount = vertical;
            NormalizeConfig();
            UpdateUi();
        }

        private void Set3DRangeProfile(string profile)
        {
            rangeProfile = profile;
            NormalizeConfig();
            UpdateUi();
        }

        private string BuildScanPacket(Vector3 origin, Vector3 forward, Vector3 up, int horizontalCount, int verticalCount)
        {
            var useHemisphere = Is3DMode();
            var rayCount = useHemisphere ? horizontalCount : horizontalCount * verticalCount;
            lastRayCount = rayCount;
            lastHitCount = 0;

            var sensorRotation = Quaternion.LookRotation(forward, up);
            var worldToSensorRotation = Quaternion.Inverse(sensorRotation);
            var partTransform = part != null && part.transform != null ? part.transform : transform;
            var sensorPositionInPart = UnityVectorToRos(partTransform.InverseTransformPoint(origin));
            var sensorRotationInPart = UnityRotationToRos(
                Quaternion.Inverse(partTransform.rotation) * sensorRotation
            );

            var ranges = new StringBuilder(lastRayCount * 8);
            var hitMask = new StringBuilder(lastRayCount * 2);
            var points = includeHitPoints ? new StringBuilder(lastRayCount * 20) : null;
            var directions = includeDirections ? new StringBuilder(lastRayCount * 20) : null;

            var right = Vector3.Cross(up, forward).normalized;
            if (right.sqrMagnitude < 0.0001f)
            {
                right = Vector3.right;
            }

            var first = true;
            var rayIndex = 0;
            for (var vertical = 0; vertical < verticalCount; vertical++)
            {
                for (var horizontal = 0; horizontal < horizontalCount; horizontal++)
                {
                    var direction = useHemisphere
                        ? CalculateHemisphereRayDirection(forward, up, right, horizontal, rayCount)
                        : CalculateRayDirection(forward, up, right, horizontal, horizontalCount, vertical, verticalCount);

                    if (!first)
                    {
                        ranges.Append(',');
                        hitMask.Append(',');
                        if (points != null)
                        {
                            points.Append(',');
                        }
                        if (directions != null)
                        {
                            directions.Append(',');
                        }
                    }

                    first = false;

                    RaycastHit hit;
                    var hasHit = TryRaycast(origin, direction, out hit);
                    if (hasHit)
                    {
                        lastHitCount++;
                        AppendFloat(ranges, hit.distance);
                        hitMask.Append('1');
                        if (points != null)
                        {
                            AppendVector3OrNull(
                                points,
                                UnityVectorToRos(worldToSensorRotation * (hit.point - origin)),
                                true
                            );
                        }
                    }
                    else
                    {
                        AppendFloat(ranges, noHitValue);
                        hitMask.Append('0');
                        if (points != null)
                        {
                            AppendVector3OrNull(points, Vector3.zero, false);
                        }
                    }

                    if (directions != null)
                    {
                        AppendVector3OrNull(
                            directions,
                            UnityVectorToRos(worldToSensorRotation * direction),
                            true
                        );
                    }

                    var debugDistance = hasHit ? hit.distance : Mathf.Min(maxDistance, 100f);
                    if (radarLinesVisible)
                    {
                        UpdateRadarLine(rayIndex, origin, origin + direction * debugDistance);
                    }

                    if (drawDebugRays)
                    {
                        Debug.DrawRay(origin, direction * debugDistance, hasHit ? Color.green : Color.red, debugRaySeconds);
                    }

                    rayIndex++;
                }
            }

            if (radarLinesVisible)
            {
                HideUnusedRadarLines(lastRayCount);
            }

            packetBuilder.Length = 0;
            packetBuilder.Append('{');
            AppendProperty(packetBuilder, "type", "ksp_lidar_scan", true);
            AppendProperty(packetBuilder, "version", JsonVersion, false);
            AppendProperty(packetBuilder, "mode", Is3DMode() ? "3D" : "2D", false);
            AppendProperty(packetBuilder, "sensorId", ResolveSensorId(), false);
            AppendProperty(packetBuilder, "vessel", vessel != null ? vessel.vesselName : "", false);
            AppendProperty(packetBuilder, "partFlightId", part != null ? (long)part.flightID : 0L, false);
            AppendProperty(packetBuilder, "universalTime", Planetarium.GetUniversalTime(), false);
            AppendProperty(packetBuilder, "rayCount", rayCount, false);
            AppendProperty(packetBuilder, "horizontalCount", horizontalCount, false);
            AppendProperty(packetBuilder, "verticalCount", verticalCount, false);
            AppendProperty(packetBuilder, "hemisphereDensity", useHemisphere ? hemisphereDensity : 0f, false);
            AppendProperty(packetBuilder, "horizontalFovDeg", useHemisphere ? 360f : horizontalFovDegrees, false);
            AppendProperty(packetBuilder, "verticalFovDeg", useHemisphere ? 180f : 0f, false);
            AppendProperty(packetBuilder, "scanRateHz", streamAt20Fps ? 20f : scanRateHz, false);
            AppendProperty(packetBuilder, "maxDistance", maxDistance, false);
            AppendProperty(packetBuilder, "layout", useHemisphere ? "fibonacci-hemisphere" : "vertical-major", false);
            AppendProperty(packetBuilder, "coordinateFrame", "ros_sensor", false);
            AppendProperty(packetBuilder, "hitCount", lastHitCount, false);
            AppendArrayProperty(packetBuilder, "origin", delegate(StringBuilder builder) { AppendVector3OrNull(builder, Vector3.zero, true); }, false);
            AppendArrayProperty(packetBuilder, "framePosition", delegate(StringBuilder builder) { AppendVector3OrNull(builder, sensorPositionInPart, true); }, false);
            AppendArrayProperty(packetBuilder, "frameRotation", delegate(StringBuilder builder) { AppendQuaternion(builder, sensorRotationInPart); }, false);
            AppendRawArrayProperty(packetBuilder, "ranges", ranges, false);
            AppendRawArrayProperty(packetBuilder, "hitMask", hitMask, false);
            if (points != null)
            {
                AppendRawArrayProperty(packetBuilder, "points", points, false);
            }
            if (directions != null)
            {
                AppendRawArrayProperty(packetBuilder, "directions", directions, false);
            }
            packetBuilder.Append('}');

            return packetBuilder.ToString();
        }

        private Vector3 CalculateRayDirection(Vector3 forward, Vector3 up, Vector3 right, int horizontal, int horizontalCount, int vertical, int verticalCount)
        {
            var yaw = AngleAt(horizontal, horizontalCount, horizontalFovDegrees);
            var pitch = AngleAt(vertical, verticalCount, verticalFovDegrees);
            // LaserScan angles increase counter-clockwise around ROS +Z. Unity's
            // handedness requires the opposite AngleAxis sign for the same order.
            var yawRotation = Quaternion.AngleAxis(-yaw, up);
            var yawedForward = yawRotation * forward;
            var yawedRight = yawRotation * right;
            return (Quaternion.AngleAxis(-pitch, yawedRight) * yawedForward).normalized;
        }

        private Vector3 CalculateHemisphereRayDirection(Vector3 forward, Vector3 up, Vector3 right, int index, int count)
        {
            if (count <= 1)
            {
                return forward.normalized;
            }

            var z = (index + 0.5f) / count;
            var radius = Mathf.Sqrt(Mathf.Max(0f, 1f - z * z));
            var azimuth = index * GoldenAngleRadians;
            return (forward * z + right * (Mathf.Cos(azimuth) * radius) + up * (Mathf.Sin(azimuth) * radius)).normalized;
        }

        private bool TryRaycast(Vector3 origin, Vector3 direction, out RaycastHit selectedHit)
        {
            if (!ignoreOwnVessel)
            {
                return Physics.Raycast(origin, direction, out selectedHit, maxDistance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
            }

            var hits = Physics.RaycastAll(origin, direction, maxDistance, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
            Array.Sort(hits, CompareRaycastHits);

            for (var i = 0; i < hits.Length; i++)
            {
                var hit = hits[i];
                var hitPart = FindPart(hit.collider);
                if (hitPart != null && vessel != null && hitPart.vessel == vessel)
                {
                    continue;
                }

                selectedHit = hit;
                return true;
            }

            selectedHit = default(RaycastHit);
            return false;
        }

        private static int CompareRaycastHits(RaycastHit left, RaycastHit right)
        {
            return left.distance.CompareTo(right.distance);
        }

        private static Part FindPart(Collider collider)
        {
            if (collider == null)
            {
                return null;
            }

            var current = collider.transform;
            while (current != null)
            {
                var hitPart = current.GetComponent<Part>();
                if (hitPart != null)
                {
                    return hitPart;
                }

                current = current.parent;
            }

            return null;
        }

        private Transform ResolveOriginTransform()
        {
            var root = part != null ? part.transform : transform;
            if (!string.IsNullOrEmpty(rayOriginName))
            {
                var child = FindChildRecursive(root, rayOriginName);
                if (child != null)
                {
                    return child;
                }
            }

            return root;
        }

        private Vector3 ResolveScanForward(Transform originTransform)
        {
            if (!Is3DMode() || !align3DToAttachNormal)
            {
                return AxisToWorld(originTransform, forwardAxis);
            }

            // node_attach is configured so its orientation points from the
            // model's mounting face toward the sensor's visible front side.
            if (part != null && part.transform != null && part.srfAttachNode != null)
            {
                var localOutwardNormal = part.srfAttachNode.orientation;
                if (localOutwardNormal.sqrMagnitude >= 0.0001f)
                {
                    return part.transform.TransformDirection(localOutwardNormal).normalized;
                }
            }

            return originTransform.forward.normalized;
        }

        private Vector3 ResolveScanOrigin(Transform originTransform, Vector3 forward)
        {
            var localPosition = ParseConfigVector3(rayOriginLocalPosition, Vector3.zero);
            return originTransform.TransformPoint(localPosition)
                + forward * Mathf.Max(0f, originOffsetMeters);
        }

        private static Transform FindChildRecursive(Transform root, string childName)
        {
            if (root == null)
            {
                return null;
            }

            if (root.name == childName)
            {
                return root;
            }

            for (var i = 0; i < root.childCount; i++)
            {
                var child = FindChildRecursive(root.GetChild(i), childName);
                if (child != null)
                {
                    return child;
                }
            }

            return null;
        }

        private static Vector3 AxisToWorld(Transform transform, string axis)
        {
            var normalizedAxis = (axis ?? "").Trim().ToUpperInvariant();
            switch (normalizedAxis)
            {
                case "+X":
                case "X":
                    return transform.right.normalized;
                case "-X":
                    return (-transform.right).normalized;
                case "+Y":
                case "Y":
                    return transform.up.normalized;
                case "-Y":
                    return (-transform.up).normalized;
                case "-Z":
                    return (-transform.forward).normalized;
                case "+Z":
                case "Z":
                default:
                    return transform.forward.normalized;
            }
        }

        private static void Orthonormalize(ref Vector3 forward, ref Vector3 up, Transform fallback)
        {
            if (forward.sqrMagnitude < 0.0001f)
            {
                forward = fallback.forward;
            }

            if (up.sqrMagnitude < 0.0001f || Mathf.Abs(Vector3.Dot(forward.normalized, up.normalized)) > 0.99f)
            {
                up = fallback.up;
                if (Mathf.Abs(Vector3.Dot(forward.normalized, up.normalized)) > 0.99f)
                {
                    up = fallback.right;
                }
            }

            forward.Normalize();
            up = (up - Vector3.Project(up, forward)).normalized;
        }

        private static float AngleAt(int index, int count, float fov)
        {
            if (count <= 1 || Mathf.Abs(fov) < 0.0001f)
            {
                return 0f;
            }

            if (Mathf.Abs(fov) >= 359.9f)
            {
                return -fov * 0.5f + fov * index / count;
            }

            return -fov * 0.5f + fov * index / (count - 1);
        }

        private int ResolveRayCounts(ref int horizontalCount, ref int verticalCount)
        {
            if (Is3DMode())
            {
                horizontalCount = Mathf.Clamp(Mathf.RoundToInt(hemisphereDensity * HemisphereSolidAngleSteradians), 1, Mathf.Max(1, maxLaserCount));
                verticalCount = 1;
                return horizontalCount;
            }

            horizontalCount = Mathf.Max(1, Mathf.RoundToInt(horizontalLaserCount));
            verticalCount = 1;
            ClampLaserCounts(ref horizontalCount, ref verticalCount);
            return horizontalCount * verticalCount;
        }

        private string FormatRayBudget(int horizontalCount, int verticalCount, int rayCount)
        {
            if (Is3DMode())
            {
                return Mathf.RoundToInt(hemisphereDensity).ToString(CultureInfo.InvariantCulture) + " / sr = " + rayCount.ToString(CultureInfo.InvariantCulture);
            }

            return horizontalCount.ToString(CultureInfo.InvariantCulture) + " x " + verticalCount.ToString(CultureInfo.InvariantCulture) + " = " + rayCount.ToString(CultureInfo.InvariantCulture);
        }

        private void ClampLaserCounts(ref int horizontalCount, ref int verticalCount)
        {
            var maxCount = Mathf.Max(1, maxLaserCount);
            if (horizontalCount * verticalCount <= maxCount)
            {
                return;
            }

            if (horizontalCount > maxCount)
            {
                horizontalCount = maxCount;
                verticalCount = 1;
                return;
            }

            verticalCount = Mathf.Max(1, maxCount / horizontalCount);
        }

        private void NormalizeConfig()
        {
            horizontalLaserCount = Mathf.Clamp(Mathf.Round(horizontalLaserCount), 1f, 2048f);
            verticalLaserCount = Mathf.Clamp(Mathf.Round(verticalLaserCount), 1f, 128f);
            nearRangeMeters = Mathf.Clamp(Mathf.Round(nearRangeMeters / 5f) * 5f, 10f, 30f);
            mediumRangeMeters = Mathf.Clamp(Mathf.Round(mediumRangeMeters / 10f) * 10f, 50f, 150f);
            longRangeMeters = Mathf.Clamp(Mathf.Round(longRangeMeters / 10f) * 10f, 150f, 250f);
            Normalize3DRangeProfile();
            horizontalFovDegrees = Mathf.Clamp(horizontalFovDegrees, 0f, 360f);
            verticalFovDegrees = Mathf.Clamp(verticalFovDegrees, 0f, 180f);
            maxDistance = Mathf.Max(0.1f, maxDistance);
            scanRateHz = Mathf.Clamp(Mathf.Round(scanRateHz), 1f, 60f);
            udpPort = Mathf.Clamp(udpPort, 1, 65535);
            partName = string.IsNullOrEmpty(partName) ? "" : partName.Trim();
            activeVesselUrdfRefreshSeconds = Mathf.Clamp(activeVesselUrdfRefreshSeconds, 0.5f, 30f);
            activeVesselUrdfChunkBytes = Mathf.Clamp(activeVesselUrdfChunkBytes, 128, 48000);
            maxActiveVesselUrdfChunks = Mathf.Clamp(maxActiveVesselUrdfChunks, 1, 512);
            lidarName = string.IsNullOrEmpty(lidarName) ? "" : lidarName.Trim();
            if (string.IsNullOrEmpty(partName) && !string.IsNullOrEmpty(lidarName))
            {
                partName = lidarName;
            }
            maxLaserCount = Mathf.Max(1, maxLaserCount);
            maxDatagramBytes = Mathf.Clamp(maxDatagramBytes, 512, 65000);
            radarLineWidth = Mathf.Clamp(radarLineWidth, 0.001f, 1f);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);
        }

        private void Normalize3DRangeProfile()
        {
            if (!Is3DMode())
            {
                hemisphereDensity = Mathf.Clamp(Mathf.Round(hemisphereDensity), 1f, 1024f);
                return;
            }

            rangeProfile = (rangeProfile ?? "").Trim().ToLowerInvariant();
            switch (rangeProfile)
            {
                case "near":
                    maxDistance = nearRangeMeters;
                    hemisphereDensity = 160f;
                    break;
                case "long":
                    maxDistance = longRangeMeters;
                    hemisphereDensity = 512f;
                    break;
                default:
                    rangeProfile = "medium";
                    maxDistance = mediumRangeMeters;
                    hemisphereDensity = 320f;
                    break;
            }
        }

        private bool Is3DMode()
        {
            return string.Equals(sensorMode, "3D", StringComparison.OrdinalIgnoreCase);
        }

        private void UpdateUi()
        {
            var threeDimensional = Is3DMode();
            Fields["horizontalLaserCount"].guiActive = !threeDimensional;
            Fields["horizontalLaserCount"].guiActiveEditor = !threeDimensional;
            Fields["verticalLaserCount"].guiActive = false;
            Fields["verticalLaserCount"].guiActiveEditor = false;
            Fields["hemisphereDensity"].guiActive = false;
            Fields["hemisphereDensity"].guiActiveEditor = false;
            Fields["nearRangeMeters"].guiActive = threeDimensional && rangeProfile == "near";
            Fields["nearRangeMeters"].guiActiveEditor = threeDimensional && rangeProfile == "near";
            Fields["mediumRangeMeters"].guiActive = threeDimensional && rangeProfile == "medium";
            Fields["mediumRangeMeters"].guiActiveEditor = threeDimensional && rangeProfile == "medium";
            Fields["longRangeMeters"].guiActive = threeDimensional && rangeProfile == "long";
            Fields["longRangeMeters"].guiActiveEditor = threeDimensional && rangeProfile == "long";
            Fields["rayBudget"].guiActive = true;
            Fields["rayBudget"].guiActiveEditor = true;
            Events["SetLowPreset"].guiName = threeDimensional ? "3D Range: Near (10-30 m)" : "LiDAR Preset: Low";
            Events["SetMediumPreset"].guiName = threeDimensional ? "3D Range: Medium (50-150 m)" : "LiDAR Preset: Medium";
            Events["SetHighPreset"].guiName = threeDimensional ? "3D Range: Long (150-250 m)" : "LiDAR Preset: High";
            Events["ToggleRadarLines"].guiName = radarLinesVisible ? "Hide Laser Preview" : "Show Laser Preview";
        }

        private string ResolveSensorId()
        {
            var legacy = !string.IsNullOrEmpty(partName) ? partName : lidarName;
            return ModuleKerbalRosSensorId.Resolve(part, legacy);
        }

    }
}
