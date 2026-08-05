using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public class ModuleKerbalLidar : PartModule
    {
        private const int JsonVersion = 1;
        private const float RadarPreviewDistanceMeters = 100f;
        private const float HemisphereSolidAngleSteradians = 6.2831853f;
        private const float GoldenAngleRadians = 2.3999632f;
        private static readonly Color RadarLineColor = new Color(1f, 0f, 0f, 0.95f);

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

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Scan Rate Hz", guiFormat = "F0")]
        [UI_FloatRange(minValue = 1f, maxValue = 60f, stepIncrement = 1f)]
        public float scanRateHz = 10f;

        [KSPField]
        public string udpHost = "127.0.0.1";

        [KSPField]
        public int udpPort = 49010;

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
        public string forwardAxis = "-Y";

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

        private UdpClient udpClient;
        private IPEndPoint udpEndPoint;
        private string endpointKey;
        private float nextScanTime;
        private double lastUdpWarningTime = -1000.0;
        private readonly StringBuilder packetBuilder = new StringBuilder(8192);
        private readonly List<LineRenderer> radarLineRenderers = new List<LineRenderer>();
        private Material radarLineMaterial;
        private GameObject visualModelObject;
        private Material visualModelMaterial;
        private string loadedVisualModelPath;
        private float nextVisualModelSyncTime;

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
            var forward = AxisToWorld(originTransform, forwardAxis);
            var up = AxisToWorld(originTransform, upAxis);
            Orthonormalize(ref forward, ref up, originTransform);

            var origin = originTransform.position + forward * Mathf.Max(0f, originOffsetMeters);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);

            var scan = BuildScanPacket(origin, forward, up, horizontalCount, verticalCount);
            lastScanMs = (Time.realtimeSinceStartup - started) * 1000f;

            if (sendUdp && udpEnabled)
            {
                SendUdp(scan);
            }
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: Low", active = true)]
        public void SetLowPreset()
        {
            if (Is3DMode())
            {
                SetHemispherePreset(64f);
                return;
            }

            SetPreset(90f, 1f);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: Medium", active = true)]
        public void SetMediumPreset()
        {
            if (Is3DMode())
            {
                SetHemispherePreset(160f);
                return;
            }

            SetPreset(180f, 1f);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "LiDAR Preset: High", active = true)]
        public void SetHighPreset()
        {
            if (Is3DMode())
            {
                SetHemispherePreset(512f);
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

        private void SetHemispherePreset(float density)
        {
            hemisphereDensity = density;
            NormalizeConfig();
            UpdateUi();
        }

        private string BuildScanPacket(Vector3 origin, Vector3 forward, Vector3 up, int horizontalCount, int verticalCount)
        {
            var useHemisphere = Is3DMode();
            var rayCount = useHemisphere ? horizontalCount : horizontalCount * verticalCount;
            lastRayCount = rayCount;
            lastHitCount = 0;

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
                            AppendVector3OrNull(points, hit.point, true);
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
                        AppendVector3OrNull(directions, direction, true);
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
            AppendProperty(packetBuilder, "name", ResolveLidarName(), false);
            AppendProperty(packetBuilder, "lidarName", ResolveLidarName(), false);
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
            AppendProperty(packetBuilder, "hitCount", lastHitCount, false);
            AppendArrayProperty(packetBuilder, "origin", delegate(StringBuilder builder) { AppendVector3OrNull(builder, origin, true); }, false);
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

        private void UpdateRadarPreviewLines()
        {
            NormalizeConfig();

            var originTransform = ResolveOriginTransform();
            var forward = AxisToWorld(originTransform, forwardAxis);
            var up = AxisToWorld(originTransform, upAxis);
            Orthonormalize(ref forward, ref up, originTransform);

            var origin = originTransform.position + forward * Mathf.Max(0f, originOffsetMeters);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);

            var right = Vector3.Cross(up, forward).normalized;
            if (right.sqrMagnitude < 0.0001f)
            {
                right = Vector3.right;
            }

            var previewDistance = Mathf.Min(maxDistance, RadarPreviewDistanceMeters);
            var rayIndex = 0;
            for (var vertical = 0; vertical < verticalCount; vertical++)
            {
                for (var horizontal = 0; horizontal < horizontalCount; horizontal++)
                {
                    var direction = Is3DMode()
                        ? CalculateHemisphereRayDirection(forward, up, right, horizontal, rayCount)
                        : CalculateRayDirection(forward, up, right, horizontal, horizontalCount, vertical, verticalCount);
                    UpdateRadarLine(rayIndex, origin, origin + direction * previewDistance);
                    rayIndex++;
                }
            }

            HideUnusedRadarLines(rayIndex);
        }

        private Vector3 CalculateRayDirection(Vector3 forward, Vector3 up, Vector3 right, int horizontal, int horizontalCount, int vertical, int verticalCount)
        {
            var yaw = AngleAt(horizontal, horizontalCount, horizontalFovDegrees);
            var pitch = AngleAt(vertical, verticalCount, verticalFovDegrees);
            var yawRotation = Quaternion.AngleAxis(yaw, up);
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

        private void LoadConfiguredVisualModel()
        {
            if (string.IsNullOrEmpty(visualModelObjPath))
            {
                return;
            }

            try
            {
                var objPath = ResolveKspRelativePath(visualModelObjPath.Trim());
                if (!File.Exists(objPath))
                {
                    Debug.LogWarning("[KerbalLiDAR] Visual OBJ was not found: " + objPath);
                    return;
                }

                DestroyVisualModel();

                var mesh = LoadObjMesh(objPath, visualModelDoubleSided);
                visualModelObject = new GameObject("KerbalLiDAR Visual Model");
                var parent = part != null && part.transform != null ? part.transform : transform;
                visualModelObject.transform.SetParent(parent, false);
                visualModelObject.transform.localPosition = ParseConfigVector3(visualModelPosition, Vector3.zero);
                visualModelObject.transform.localEulerAngles = ParseConfigVector3(visualModelRotation, Vector3.zero);
                visualModelObject.transform.localScale = Vector3.one * Mathf.Max(0.001f, visualModelScale);
                visualModelObject.layer = parent.gameObject.layer;

                var meshFilter = visualModelObject.AddComponent<MeshFilter>();
                meshFilter.sharedMesh = mesh;

                var meshRenderer = visualModelObject.AddComponent<MeshRenderer>();
                visualModelMaterial = CreateVisualModelMaterial();
                meshRenderer.sharedMaterial = visualModelMaterial;
                loadedVisualModelPath = objPath;
                HideStockModelRenderers();
                ShowVisualModelRenderers();
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[KerbalLiDAR] Failed to load visual OBJ '" + visualModelObjPath + "': " + ex.Message);
            }
        }

        private void EnsureConfiguredVisualModel()
        {
            if (string.IsNullOrEmpty(visualModelObjPath))
            {
                return;
            }

            if (Time.realtimeSinceStartup < nextVisualModelSyncTime)
            {
                return;
            }

            nextVisualModelSyncTime = Time.realtimeSinceStartup + 0.5f;
            var objPath = ResolveKspRelativePath(visualModelObjPath.Trim());
            if (visualModelObject == null || !string.Equals(loadedVisualModelPath, objPath, StringComparison.Ordinal))
            {
                LoadConfiguredVisualModel();
                return;
            }

            HideStockModelRenderers();
            ShowVisualModelRenderers();
        }

        private void HideStockModelRenderers()
        {
            var root = part != null && part.transform != null ? part.transform : transform;
            var renderers = root.GetComponentsInChildren<Renderer>(true);
            for (var i = 0; i < renderers.Length; i++)
            {
                if (renderers[i] is LineRenderer || IsVisualModelRenderer(renderers[i]))
                {
                    continue;
                }

                renderers[i].enabled = false;
            }
        }

        private void ShowVisualModelRenderers()
        {
            if (visualModelObject == null)
            {
                return;
            }

            var renderers = visualModelObject.GetComponentsInChildren<Renderer>(true);
            for (var i = 0; i < renderers.Length; i++)
            {
                if (renderers[i] != null)
                {
                    renderers[i].enabled = true;
                }
            }
        }

        private bool IsVisualModelRenderer(Renderer renderer)
        {
            if (renderer == null || visualModelObject == null)
            {
                return false;
            }

            var current = renderer.transform;
            while (current != null)
            {
                if (current == visualModelObject.transform)
                {
                    return true;
                }

                current = current.parent;
            }

            return false;
        }

        private void DestroyVisualModel()
        {
            if (visualModelObject != null)
            {
                Destroy(visualModelObject);
                visualModelObject = null;
            }

            if (visualModelMaterial != null)
            {
                Destroy(visualModelMaterial);
                visualModelMaterial = null;
            }

            loadedVisualModelPath = null;
        }

        private Material CreateVisualModelMaterial()
        {
            var shader = Shader.Find("KSP/Diffuse");
            if (shader == null)
            {
                shader = Shader.Find("Diffuse");
            }

            var material = new Material(shader);
            var texture = LoadVisualModelTexture();
            if (texture != null)
            {
                material.mainTexture = texture;
            }

            return material;
        }

        private Texture2D LoadVisualModelTexture()
        {
            if (string.IsNullOrEmpty(visualModelTexturePath) || GameDatabase.Instance == null)
            {
                return null;
            }

            var texturePath = NormalizeGameDatabaseTexturePath(visualModelTexturePath);
            return string.IsNullOrEmpty(texturePath) ? null : GameDatabase.Instance.GetTexture(texturePath, false);
        }

        private static string NormalizeGameDatabaseTexturePath(string path)
        {
            var normalized = (path ?? "").Trim().Replace('\\', '/');
            const string gameDataPrefix = "GameData/";
            if (normalized.StartsWith(gameDataPrefix, StringComparison.OrdinalIgnoreCase))
            {
                normalized = normalized.Substring(gameDataPrefix.Length);
            }

            var slashIndex = normalized.LastIndexOf('/');
            var dotIndex = normalized.LastIndexOf('.');
            if (dotIndex > slashIndex)
            {
                normalized = normalized.Substring(0, dotIndex);
            }

            return normalized;
        }

        private static Mesh LoadObjMesh(string path, bool doubleSided)
        {
            var sourcePositions = new List<Vector3>();
            var sourceUvs = new List<Vector2>();
            var sourceNormals = new List<Vector3>();
            var meshPositions = new List<Vector3>();
            var meshUvs = new List<Vector2>();
            var meshNormals = new List<Vector3>();
            var triangles = new List<int>();
            var vertexMap = new Dictionary<string, int>();
            var missingNormals = false;

            var lines = File.ReadAllLines(path);
            for (var lineIndex = 0; lineIndex < lines.Length; lineIndex++)
            {
                var line = lines[lineIndex].Trim();
                if (line.Length == 0 || line.StartsWith("#", StringComparison.Ordinal))
                {
                    continue;
                }

                var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0)
                {
                    continue;
                }

                switch (parts[0])
                {
                    case "v":
                        if (parts.Length >= 4)
                        {
                            sourcePositions.Add(new Vector3(ParseObjFloat(parts[1]), ParseObjFloat(parts[2]), ParseObjFloat(parts[3])));
                        }
                        break;
                    case "vt":
                        if (parts.Length >= 3)
                        {
                            sourceUvs.Add(new Vector2(ParseObjFloat(parts[1]), ParseObjFloat(parts[2])));
                        }
                        break;
                    case "vn":
                        if (parts.Length >= 4)
                        {
                            sourceNormals.Add(new Vector3(ParseObjFloat(parts[1]), ParseObjFloat(parts[2]), ParseObjFloat(parts[3])).normalized);
                        }
                        break;
                    case "f":
                        if (parts.Length >= 4)
                        {
                            var faceIndices = new int[parts.Length - 1];
                            for (var i = 1; i < parts.Length; i++)
                            {
                                faceIndices[i - 1] = ResolveObjVertex(parts[i], sourcePositions, sourceUvs, sourceNormals, meshPositions, meshUvs, meshNormals, vertexMap, ref missingNormals);
                            }

                            for (var i = 1; i < faceIndices.Length - 1; i++)
                            {
                                AddTriangle(triangles, faceIndices[0], faceIndices[i], faceIndices[i + 1], doubleSided);
                            }
                        }
                        break;
                }
            }

            if (meshPositions.Count == 0 || triangles.Count == 0)
            {
                throw new InvalidOperationException("OBJ did not contain any mesh geometry.");
            }

            var mesh = new Mesh();
            mesh.name = Path.GetFileNameWithoutExtension(path);
            mesh.vertices = meshPositions.ToArray();
            mesh.triangles = triangles.ToArray();
            if (meshUvs.Count == meshPositions.Count)
            {
                mesh.uv = meshUvs.ToArray();
            }
            if (!missingNormals && meshNormals.Count == meshPositions.Count)
            {
                mesh.normals = meshNormals.ToArray();
            }
            else
            {
                mesh.RecalculateNormals();
            }
            mesh.RecalculateBounds();
            return mesh;
        }

        private static int ResolveObjVertex(string token, List<Vector3> sourcePositions, List<Vector2> sourceUvs, List<Vector3> sourceNormals, List<Vector3> meshPositions, List<Vector2> meshUvs, List<Vector3> meshNormals, Dictionary<string, int> vertexMap, ref bool missingNormals)
        {
            int existingIndex;
            if (vertexMap.TryGetValue(token, out existingIndex))
            {
                return existingIndex;
            }

            var indices = token.Split('/');
            var positionIndex = ParseObjIndex(indices.Length > 0 ? indices[0] : "", sourcePositions.Count);
            var uvIndex = indices.Length > 1 && indices[1].Length > 0 ? ParseObjIndex(indices[1], sourceUvs.Count) : -1;
            var normalIndex = indices.Length > 2 && indices[2].Length > 0 ? ParseObjIndex(indices[2], sourceNormals.Count) : -1;

            var meshIndex = meshPositions.Count;
            meshPositions.Add(sourcePositions[positionIndex]);
            meshUvs.Add(uvIndex >= 0 ? sourceUvs[uvIndex] : Vector2.zero);
            if (normalIndex >= 0)
            {
                meshNormals.Add(sourceNormals[normalIndex]);
            }
            else
            {
                meshNormals.Add(Vector3.zero);
                missingNormals = true;
            }

            vertexMap[token] = meshIndex;
            return meshIndex;
        }

        private static void AddTriangle(List<int> triangles, int a, int b, int c, bool doubleSided)
        {
            triangles.Add(a);
            triangles.Add(b);
            triangles.Add(c);

            if (!doubleSided)
            {
                return;
            }

            triangles.Add(c);
            triangles.Add(b);
            triangles.Add(a);
        }

        private static int ParseObjIndex(string value, int count)
        {
            var index = int.Parse(value, NumberStyles.Integer, CultureInfo.InvariantCulture);
            var resolved = index > 0 ? index - 1 : count + index;
            if (resolved < 0 || resolved >= count)
            {
                throw new InvalidOperationException("OBJ index is out of range.");
            }

            return resolved;
        }

        private static float ParseObjFloat(string value)
        {
            return float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private static Vector3 ParseConfigVector3(string value, Vector3 fallback)
        {
            if (string.IsNullOrEmpty(value))
            {
                return fallback;
            }

            var parts = value.Split(new[] { ',', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 3)
            {
                return fallback;
            }

            float x;
            float y;
            float z;
            if (!float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out x) ||
                !float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out y) ||
                !float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out z))
            {
                return fallback;
            }

            return new Vector3(x, y, z);
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
            hemisphereDensity = Mathf.Clamp(Mathf.Round(hemisphereDensity), 1f, 1024f);
            horizontalFovDegrees = Mathf.Clamp(horizontalFovDegrees, 0f, 360f);
            verticalFovDegrees = Mathf.Clamp(verticalFovDegrees, 0f, 180f);
            maxDistance = Mathf.Max(0.1f, maxDistance);
            scanRateHz = Mathf.Clamp(Mathf.Round(scanRateHz), 1f, 60f);
            udpPort = Mathf.Clamp(udpPort, 1, 65535);
            lidarName = string.IsNullOrEmpty(lidarName) ? "" : lidarName.Trim();
            maxLaserCount = Mathf.Max(1, maxLaserCount);
            maxDatagramBytes = Mathf.Clamp(maxDatagramBytes, 512, 65000);
            radarLineWidth = Mathf.Clamp(radarLineWidth, 0.001f, 1f);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);
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
            Fields["hemisphereDensity"].guiActive = threeDimensional;
            Fields["hemisphereDensity"].guiActiveEditor = threeDimensional;
            Fields["rayBudget"].guiActive = true;
            Fields["rayBudget"].guiActiveEditor = true;
            Events["ToggleRadarLines"].guiName = radarLinesVisible ? "Hide Laser Preview" : "Show Laser Preview";
        }

        private string ResolveLidarName()
        {
            if (!string.IsNullOrEmpty(lidarName))
            {
                return lidarName;
            }

            var partTitle = part != null && part.partInfo != null && !string.IsNullOrEmpty(part.partInfo.title)
                ? part.partInfo.title
                : "lidar";
            var partFlightId = part != null ? ((long)part.flightID).ToString(CultureInfo.InvariantCulture) : "0";
            return partTitle + "_" + partFlightId;
        }

        private string ResolveKspRelativePath(string path)
        {
            if (Path.IsPathRooted(path))
            {
                return path;
            }

            return Path.Combine(GetKspRootPath(), path);
        }

        private string GetKspRootPath()
        {
            try
            {
                if (!string.IsNullOrEmpty(KSPUtil.ApplicationRootPath))
                {
                    return KSPUtil.ApplicationRootPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
                }
            }
            catch
            {
            }

            return Directory.GetCurrentDirectory();
        }

        private void UpdateRadarLine(int index, Vector3 start, Vector3 end)
        {
            var line = EnsureRadarLineRenderer(index);
            var width = Mathf.Max(0.001f, radarLineWidth);
            var lineTransform = line.transform;

            if (!line.gameObject.activeSelf)
            {
                line.gameObject.SetActive(true);
            }

            line.enabled = true;
            line.useWorldSpace = false;
            line.material = GetRadarLineMaterial();
            line.startWidth = width;
            line.endWidth = width;
            line.startColor = RadarLineColor;
            line.endColor = RadarLineColor;
            line.SetPosition(0, lineTransform.InverseTransformPoint(start));
            line.SetPosition(1, lineTransform.InverseTransformPoint(end));
        }

        private LineRenderer EnsureRadarLineRenderer(int index)
        {
            while (radarLineRenderers.Count <= index)
            {
                var lineObject = new GameObject("KerbalLiDAR Radar Line");
                if (part != null && part.transform != null)
                {
                    lineObject.transform.SetParent(part.transform, false);
                    lineObject.layer = part.gameObject.layer;
                }

                var line = lineObject.AddComponent<LineRenderer>();
                line.useWorldSpace = false;
                line.positionCount = 2;
                line.material = GetRadarLineMaterial();
                line.startColor = RadarLineColor;
                line.endColor = RadarLineColor;
                line.startWidth = Mathf.Max(0.001f, radarLineWidth);
                line.endWidth = Mathf.Max(0.001f, radarLineWidth);
                line.gameObject.SetActive(false);

                radarLineRenderers.Add(line);
            }

            return radarLineRenderers[index];
        }

        private Material GetRadarLineMaterial()
        {
            if (radarLineMaterial != null)
            {
                return radarLineMaterial;
            }

            var shader = FindFirstShader(
                "KSP/Particles/Additive",
                "KSP/Alpha/Unlit Transparent",
                "KSP/Alpha/Translucent",
                "Particles/Additive",
                "Unlit/Color",
                "KSP/Diffuse",
                "Diffuse");

            radarLineMaterial = new Material(shader);
            radarLineMaterial.color = RadarLineColor;
            radarLineMaterial.renderQueue = 3000;
            if (radarLineMaterial.HasProperty("_Color"))
            {
                radarLineMaterial.SetColor("_Color", RadarLineColor);
            }
            if (radarLineMaterial.HasProperty("_TintColor"))
            {
                radarLineMaterial.SetColor("_TintColor", RadarLineColor);
            }

            return radarLineMaterial;
        }

        private static Shader FindFirstShader(params string[] shaderNames)
        {
            for (var i = 0; i < shaderNames.Length; i++)
            {
                var shader = Shader.Find(shaderNames[i]);
                if (shader != null)
                {
                    return shader;
                }
            }

            throw new InvalidOperationException("No compatible Unity shader was found for KerbalLiDAR radar lines.");
        }

        private void HideUnusedRadarLines(int activeCount)
        {
            for (var i = activeCount; i < radarLineRenderers.Count; i++)
            {
                var line = radarLineRenderers[i];
                if (line != null && line.gameObject.activeSelf)
                {
                    line.gameObject.SetActive(false);
                }
            }
        }

        private void ClearRadarLines()
        {
            HideUnusedRadarLines(0);
        }

        private void DestroyRadarLines()
        {
            for (var i = 0; i < radarLineRenderers.Count; i++)
            {
                var line = radarLineRenderers[i];
                if (line != null)
                {
                    Destroy(line.gameObject);
                }
            }

            radarLineRenderers.Clear();
            if (radarLineMaterial != null)
            {
                Destroy(radarLineMaterial);
                radarLineMaterial = null;
            }
        }

        private void SendUdp(string payload)
        {
            try
            {
                EnsureUdpClient();

                var bytes = Encoding.UTF8.GetBytes(payload);
                if (bytes.Length > maxDatagramBytes)
                {
                    WarnUdpThrottled("LiDAR UDP packet is " + bytes.Length + " bytes; reduce laser count or disable optional arrays.");
                    return;
                }

                udpClient.Send(bytes, bytes.Length, udpEndPoint);
            }
            catch (Exception ex)
            {
                WarnUdpThrottled("LiDAR UDP send failed: " + ex.Message);
            }
        }

        private void EnsureUdpClient()
        {
            var host = string.IsNullOrEmpty(udpHost) ? "127.0.0.1" : udpHost;
            var key = host + ":" + udpPort.ToString(CultureInfo.InvariantCulture);
            if (udpClient != null && udpEndPoint != null && endpointKey == key)
            {
                return;
            }

            CloseUdpClient();
            udpClient = new UdpClient();
            udpEndPoint = new IPEndPoint(ResolveAddress(host), udpPort);
            endpointKey = key;
        }

        private static IPAddress ResolveAddress(string host)
        {
            IPAddress address;
            if (IPAddress.TryParse(host, out address))
            {
                return address;
            }

            var addresses = Dns.GetHostAddresses(host);
            if (addresses.Length == 0)
            {
                throw new InvalidOperationException("Could not resolve UDP host '" + host + "'.");
            }

            return addresses[0];
        }

        private void CloseUdpClient()
        {
            if (udpClient != null)
            {
                udpClient.Close();
                udpClient = null;
            }

            udpEndPoint = null;
            endpointKey = null;
        }

        private void WarnUdpThrottled(string message)
        {
            var now = Planetarium.GetUniversalTime();
            if (now - lastUdpWarningTime < 5.0)
            {
                return;
            }

            lastUdpWarningTime = now;
            Debug.LogWarning("[KerbalLiDAR] " + message);
        }

        private static void AppendProperty(StringBuilder builder, string name, string value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            AppendJsonString(builder, value);
        }

        private static void AppendProperty(StringBuilder builder, string name, int value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, long value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, double value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendProperty(StringBuilder builder, string name, float value, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            AppendFloat(builder, value);
        }

        private static void AppendArrayProperty(StringBuilder builder, string name, Action<StringBuilder> appendArrayBody, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append('[');
            appendArrayBody(builder);
            builder.Append(']');
        }

        private static void AppendRawArrayProperty(StringBuilder builder, string name, StringBuilder rawBody, bool first)
        {
            AppendPropertyPrefix(builder, name, first);
            builder.Append('[');
            builder.Append(rawBody);
            builder.Append(']');
        }

        private static void AppendPropertyPrefix(StringBuilder builder, string name, bool first)
        {
            if (!first)
            {
                builder.Append(',');
            }

            AppendJsonString(builder, name);
            builder.Append(':');
        }

        private static void AppendFloat(StringBuilder builder, float value)
        {
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static void AppendVector3OrNull(StringBuilder builder, Vector3 value, bool hasValue)
        {
            if (!hasValue)
            {
                builder.Append("null,null,null");
                return;
            }

            AppendFloat(builder, value.x);
            builder.Append(',');
            AppendFloat(builder, value.y);
            builder.Append(',');
            AppendFloat(builder, value.z);
        }

        private static void AppendJsonString(StringBuilder builder, string value)
        {
            builder.Append('"');
            if (value != null)
            {
                for (var i = 0; i < value.Length; i++)
                {
                    var c = value[i];
                    switch (c)
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
                            if (c < 32)
                            {
                                builder.Append("\\u");
                                builder.Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                            }
                            else
                            {
                                builder.Append(c);
                            }
                            break;
                    }
                }
            }

            builder.Append('"');
        }
    }
}
