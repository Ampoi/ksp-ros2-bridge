using System;
using System.Collections.Generic;
using UnityEngine;

namespace KerbalLiDAR
{
    /// <summary>
    /// Renders KSP's stacked flight cameras from a sensor pose into one RGB image.
    /// KSP splits the visible scene across galaxy, scaled-space, and local
    /// near/far cameras, so copying FlightCamera.mainCamera alone is incomplete.
    /// </summary>
    internal sealed class KspSceneRgbCapture
    {
        private sealed class SourceCamera
        {
            public Camera Camera;
            public bool IsScaledSpace;
            public bool IsGalaxy;
            public Camera SavedSettings;
            public Vector3 LocalPosition;
            public Quaternion LocalRotation;
            public Vector3 LocalScale;
            public bool WasEnabled;
            public Rect PixelRect;
            public RenderTexture OriginalTarget;
        }

        private readonly List<SourceCamera> sources = new List<SourceCamera>(6);
        private readonly List<SourceCamera> poseOrder = new List<SourceCamera>(6);
        private readonly List<Camera> settingsCameras = new List<Camera>(6);
        private readonly List<GameObject> settingsCameraObjects = new List<GameObject>(6);
        private readonly List<Behaviour> suspendedTemporalEffects = new List<Behaviour>(4);
        private static bool captureInProgress;
        private bool antialiasingGuardRetained;
        private RenderTexture renderTexture;
        private Texture2D readbackTexture;
        private RenderTexture outputTexture;
        private int renderWidth;
        private int renderHeight;
        private int width;
        private int height;

        // Unity's texture orientation is already correct for GUI.DrawTexture;
        // only the ROS wire format needs the top-down row conversion below.
        public Texture PreviewTexture { get { return readbackTexture; } }

        public byte[] Capture(
            int imageWidth,
            int imageHeight,
            Vector3 origin,
            Quaternion rotation,
            float verticalFovDegrees,
            float nearClipMeters,
            float farClipMeters,
            bool useGameClipPlanes = true)
        {
            Render(imageWidth, imageHeight, origin, rotation, verticalFovDegrees, nearClipMeters, farClipMeters, useGameClipPlanes);
            return ReadTopDownRgb();
        }

        public void Render(
            int imageWidth,
            int imageHeight,
            Vector3 origin,
            Quaternion rotation,
            float verticalFovDegrees,
            float nearClipMeters,
            float farClipMeters,
            bool useGameClipPlanes = true)
        {
            // KSP/visual mods can leave Camera.current populated even in a normal
            // PartModule.OnUpdate. It is not a reliable rendering-state guard.
            // Our own synchronous render guard still rejects callback recursion.
            if (captureInProgress)
            {
                throw new InvalidOperationException("RGB capture must run outside camera rendering callbacks.");
            }
            var started = Time.realtimeSinceStartup;
            if (!antialiasingGuardRetained)
            {
                RgbCaptureAntialiasingGuard.Retain();
                antialiasingGuardRetained = true;
            }
            EnsureResources(imageWidth, imageHeight);
            CollectSourceCameras();
            if (sources.Count == 0)
            {
                throw new InvalidOperationException("KSP scene cameras are unavailable.");
            }

            var previousActive = RenderTexture.active;
            var savedCount = 0;
            captureInProgress = true;
            try
            {
                // Keep the original camera identities: EVE/Scatterer attach hooks,
                // image effects and command buffers to them, not to Camera copies.
                // Save the entire stack before changing any transforms (some share
                // a hierarchy), then move all cameras before invoking MOD hooks.
                for (var index = 0; index < sources.Count; index++)
                {
                    var source = sources[index];
                    source.SavedSettings = GetSettingsCamera(index);
                    source.SavedSettings.CopyFrom(source.Camera);
                    source.SavedSettings.enabled = false;
                    source.LocalPosition = source.Camera.transform.localPosition;
                    source.LocalRotation = source.Camera.transform.localRotation;
                    source.LocalScale = source.Camera.transform.localScale;
                    source.WasEnabled = source.Camera.enabled;
                    source.PixelRect = source.Camera.pixelRect;
                    source.OriginalTarget = source.Camera.targetTexture;
                    savedCount++;
                }
                // A scenery camera can be a child of the main camera. Pose
                // parents first so moving the parent cannot displace an already
                // configured child; rendering below still uses camera depth.
                poseOrder.Clear();
                poseOrder.AddRange(sources);
                poseOrder.Sort(delegate(SourceCamera left, SourceCamera right)
                {
                    return TransformDepth(left.Camera.transform).CompareTo(TransformDepth(right.Camera.transform));
                });
                for (var index = 0; index < poseOrder.Count; index++)
                {
                    SuspendTemporalEffects(poseOrder[index].Camera);
                    ConfigureSource(poseOrder[index], origin, rotation, verticalFovDegrees,
                        nearClipMeters, farClipMeters, useGameClipPlanes);
                }

                // Some KSP flight cameras clear depth only because another camera
                // normally supplies their background. Clear both buffers here so a
                // missing/disabled layer can never leave a transparent stale frame.
                RenderTexture.active = renderTexture;
                GL.Clear(true, true, Color.black);

                for (var index = 0; index < sources.Count; index++)
                {
                    var camera = sources[index].Camera;
                    if (camera.farClipPlane > camera.nearClipPlane)
                    {
                        camera.Render();
                    }
                }

                // Resize only after MOD camera hooks have finished. Their shared
                // screen/depth buffers must never see the sensor output size.
                Graphics.Blit(renderTexture, outputTexture);
                RenderTexture.active = outputTexture;
                readbackTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
                readbackTexture.Apply(false, false);
            }
            finally
            {
                try
                {
                    for (var index = savedCount - 1; index >= 0; index--)
                    {
                        var source = sources[index];
                        if (source.Camera == null) continue;
                        source.Camera.CopyFrom(source.SavedSettings);
                        // Restore the real screen viewport explicitly: an inactive
                        // backup Camera is not a screen-sized render target.
                        source.Camera.targetTexture = source.OriginalTarget;
                        source.Camera.pixelRect = source.PixelRect;
                        source.Camera.enabled = source.WasEnabled;
                    }
                    // CopyFrom may copy transforms too. Restore local transforms
                    // after all copies so parent/child camera rigs retain scale.
                    for (var index = 0; index < savedCount; index++)
                    {
                        var source = sources[index];
                        if (source.Camera == null) continue;
                        source.Camera.transform.localPosition = source.LocalPosition;
                        source.Camera.transform.localRotation = source.LocalRotation;
                        source.Camera.transform.localScale = source.LocalScale;
                    }
                }
                finally
                {
                    try
                    {
                        RestoreTemporalEffects();
                    }
                    finally
                    {
                        RenderTexture.active = previousActive;
                        captureInProgress = false;
                        GL.InvalidateState();
                        RgbCaptureAntialiasingGuard.RecordCapture(Time.realtimeSinceStartup - started);
                    }
                }
            }
        }

        private void SuspendTemporalEffects(Camera camera)
        {
            // Scatterer TAA writes its screen history and advances jitter on
            // EVERY OnPreCull, including manual Camera.Render. CopyFrom cannot
            // restore those textures: the next game frame blends the sensor view
            // into the screen and flashes at the sensor frame rate.
            // The installed Scatterer TAA has no OnEnable/OnDisable handlers, so
            // suspending only this component preserves history without resetting
            // it. Leave EVE, atmospheric scattering and other camera hooks active.
            // Match the full type name to keep Scatterer an optional dependency.
            foreach (var effect in camera.GetComponents<Behaviour>())
            {
                if (effect != null && effect.enabled
                    && effect.GetType().FullName == "Scatterer.TemporalAntiAliasing")
                {
                    RgbCaptureAntialiasingGuard.ObserveEffect(camera, effect);
                    suspendedTemporalEffects.Add(effect);
                    effect.enabled = false;
                }
            }
        }

        private void RestoreTemporalEffects()
        {
            for (var index = suspendedTemporalEffects.Count - 1; index >= 0; index--)
            {
                var effect = suspendedTemporalEffects[index];
                if (effect != null) effect.enabled = true;
            }
            suspendedTemporalEffects.Clear();
        }

        public void Dispose()
        {
            if (antialiasingGuardRetained)
            {
                RgbCaptureAntialiasingGuard.Release();
                antialiasingGuardRetained = false;
            }
            for (var index = 0; index < settingsCameras.Count; index++)
            {
                if (settingsCameras[index] != null)
                {
                    settingsCameras[index].targetTexture = null;
                }
            }
            if (renderTexture != null)
            {
                renderTexture.Release();
                UnityEngine.Object.Destroy(renderTexture);
                renderTexture = null;
            }
            if (outputTexture != null)
            {
                outputTexture.Release();
                UnityEngine.Object.Destroy(outputTexture);
                outputTexture = null;
            }
            if (readbackTexture != null)
            {
                UnityEngine.Object.Destroy(readbackTexture);
                readbackTexture = null;
            }
            for (var index = 0; index < settingsCameraObjects.Count; index++)
            {
                if (settingsCameraObjects[index] != null)
                {
                    UnityEngine.Object.Destroy(settingsCameraObjects[index]);
                }
            }
            settingsCameras.Clear();
            settingsCameraObjects.Clear();
            sources.Clear();
            poseOrder.Clear();
            width = 0;
            height = 0;
        }

        private void EnsureResources(int imageWidth, int imageHeight)
        {
            // Scatterer.DepthToDistanceCommandBuffer shares a depth texture
            // across the flight stack. Its OnPostRender reallocates it whenever
            // activeTexture changes size, leaving stale command buffers behind.
            // Keep the intermediate target screen-sized and stable even when
            // only the requested sensor output resolution changes.
            var gameWidth = Mathf.Max(1, Screen.width);
            var gameHeight = Mathf.Max(1, Screen.height);
            if (renderTexture == null || renderWidth != gameWidth || renderHeight != gameHeight)
            {
                if (renderTexture != null)
                {
                    renderTexture.Release();
                    UnityEngine.Object.Destroy(renderTexture);
                }
                renderWidth = gameWidth;
                renderHeight = gameHeight;
                renderTexture = new RenderTexture(renderWidth, renderHeight, 24, RenderTextureFormat.ARGB32);
                renderTexture.name = "Kerbal composite RGB " + renderWidth + "x" + renderHeight;
                renderTexture.filterMode = FilterMode.Bilinear;
                renderTexture.Create();
            }

            if (outputTexture != null && readbackTexture != null
                && width == imageWidth && height == imageHeight)
            {
                return;
            }
            if (outputTexture != null)
            {
                outputTexture.Release();
                UnityEngine.Object.Destroy(outputTexture);
            }
            if (readbackTexture != null) UnityEngine.Object.Destroy(readbackTexture);
            width = imageWidth;
            height = imageHeight;
            outputTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
            outputTexture.name = "Kerbal RGB output " + width + "x" + height;
            outputTexture.Create();
            readbackTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        }

        private void CollectSourceCameras()
        {
            sources.Clear();
            var seen = new HashSet<Camera>();

            if (HighLogic.LoadedSceneIsEditor)
            {
                // The main editor camera draws parts/workers only. Its sceneryCam
                // child supplies the hangar, ground and sky. Exclude markerCam
                // and unrelated UI/icon cameras from the sensor image.
                if (EditorLogic.fetch != null)
                {
                    var editorCamera = EditorLogic.fetch.editorCamera;
                    if (editorCamera != null)
                    {
                        foreach (var child in editorCamera.GetComponentsInChildren<Camera>())
                        {
                            if (child.name == "sceneryCam")
                                AddSource(child, false, false, seen);
                        }
                        AddSource(editorCamera, false, false, seen);
                    }
                }
                SortSourcesByDepth();
                return;
            }

            var scaledCamera = ScaledCamera.Instance;
            if (scaledCamera != null)
            {
                AddSource(scaledCamera.galaxyCamera, false, true, seen);
                AddSource(scaledCamera.cam, true, false, seen);
            }

            var flightCamera = FlightCamera.fetch;
            if (flightCamera != null && flightCamera.cameras != null)
            {
                for (var index = 0; index < flightCamera.cameras.Length; index++)
                {
                    AddSource(flightCamera.cameras[index], false, false, seen);
                }
            }
            if (flightCamera != null)
            {
                AddSource(flightCamera.mainCamera, false, false, seen);
            }
            // Camera.main can be an IVA/UI camera; only use KSP's flight stack.

            SortSourcesByDepth();
        }

        private static int TransformDepth(Transform transform)
        {
            var depth = 0;
            for (var parent = transform.parent; parent != null; parent = parent.parent) depth++;
            return depth;
        }

        private void SortSourcesByDepth()
        {
            sources.Sort(delegate(SourceCamera left, SourceCamera right)
            {
                return left.Camera.depth.CompareTo(right.Camera.depth);
            });
        }

        private void AddSource(
            Camera source,
            bool isScaledSpace,
            bool isGalaxy,
            HashSet<Camera> seen)
        {
            if (source == null || !source.enabled || !source.gameObject.activeInHierarchy || seen.Contains(source))
            {
                return;
            }
            seen.Add(source);
            sources.Add(new SourceCamera
            {
                Camera = source,
                IsScaledSpace = isScaledSpace,
                IsGalaxy = isGalaxy
            });
        }

        private Camera GetSettingsCamera(int index)
        {
            while (settingsCameras.Count <= index)
            {
                var cameraObject = new GameObject("Kerbal RGB Camera Settings Backup");
                cameraObject.SetActive(false);
                cameraObject.hideFlags = HideFlags.HideAndDontSave;
                var camera = cameraObject.AddComponent<Camera>();
                camera.enabled = false;
                settingsCameraObjects.Add(cameraObject);
                settingsCameras.Add(camera);
            }
            return settingsCameras[index];
        }

        private void ConfigureSource(
            SourceCamera source,
            Vector3 origin,
            Quaternion rotation,
            float verticalFovDegrees,
            float nearClipMeters,
            float farClipMeters,
            bool useGameClipPlanes)
        {
            var captureCamera = source.Camera;
            captureCamera.enabled = false;
            captureCamera.targetTexture = renderTexture;
            captureCamera.rect = new Rect(0f, 0f, 1f, 1f);
            captureCamera.ResetProjectionMatrix();
            captureCamera.ResetWorldToCameraMatrix();
            captureCamera.ResetCullingMatrix();
            captureCamera.fieldOfView = verticalFovDegrees;
            // Keep sensor intrinsics independent of the intermediate screen size.
            // The final blit restores the requested image aspect ratio.
            captureCamera.aspect = (float)width / height;
            captureCamera.transform.rotation = rotation;

            if (source.IsGalaxy)
            {
                // Galaxy geometry follows the stock galaxy camera and only needs
                // the sensor's viewing rotation; translating it would add parallax.
                captureCamera.transform.position = source.SavedSettings.transform.position;
            }
            else if (source.IsScaledSpace)
            {
                captureCamera.transform.position = ScaledSpace.LocalToScaledSpace(origin);
            }
            else
            {
                captureCamera.transform.position = origin;
                if (!useGameClipPlanes)
                {
                    captureCamera.nearClipPlane = Mathf.Max(nearClipMeters, source.SavedSettings.nearClipPlane);
                    captureCamera.farClipPlane = Mathf.Min(farClipMeters, source.SavedSettings.farClipPlane);
                }
            }
        }

        public byte[] ReadTopDownRgb()
        {
            var pixels = readbackTexture.GetPixels32();
            var rgb = new byte[width * height * 3];
            var targetIndex = 0;
            for (var targetY = 0; targetY < height; targetY++)
            {
                var sourceRow = (height - 1 - targetY) * width;
                for (var x = 0; x < width; x++)
                {
                    var pixel = pixels[sourceRow + x];
                    rgb[targetIndex++] = pixel.r;
                    rgb[targetIndex++] = pixel.g;
                    rgb[targetIndex++] = pixel.b;
                }
            }
            return rgb;
        }
    }
}
