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
        }

        private readonly List<SourceCamera> sources = new List<SourceCamera>(6);
        private readonly List<Camera> captureCameras = new List<Camera>(6);
        private readonly List<GameObject> captureCameraObjects = new List<GameObject>(6);
        private RenderTexture renderTexture;
        private Texture2D readbackTexture;
        private int width;
        private int height;

        public byte[] Capture(
            int imageWidth,
            int imageHeight,
            Vector3 origin,
            Quaternion rotation,
            float verticalFovDegrees,
            float nearClipMeters,
            float farClipMeters)
        {
            EnsureResources(imageWidth, imageHeight);
            CollectSourceCameras();
            if (sources.Count == 0)
            {
                throw new InvalidOperationException("KSP flight cameras are unavailable.");
            }

            var previousActive = RenderTexture.active;
            try
            {
                // Some KSP flight cameras clear depth only because another camera
                // normally supplies their background. Clear both buffers here so a
                // missing/disabled layer can never leave a transparent stale frame.
                RenderTexture.active = renderTexture;
                GL.Clear(true, true, Color.black);

                for (var index = 0; index < sources.Count; index++)
                {
                    RenderSource(
                        sources[index],
                        GetCaptureCamera(index),
                        origin,
                        rotation,
                        verticalFovDegrees,
                        nearClipMeters,
                        farClipMeters);
                }

                RenderTexture.active = renderTexture;
                readbackTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
                readbackTexture.Apply(false, false);
            }
            finally
            {
                RenderTexture.active = previousActive;
            }

            return ReadTopDownRgb();
        }

        public void Dispose()
        {
            for (var index = 0; index < captureCameras.Count; index++)
            {
                if (captureCameras[index] != null)
                {
                    captureCameras[index].targetTexture = null;
                }
            }
            if (renderTexture != null)
            {
                renderTexture.Release();
                UnityEngine.Object.Destroy(renderTexture);
                renderTexture = null;
            }
            if (readbackTexture != null)
            {
                UnityEngine.Object.Destroy(readbackTexture);
                readbackTexture = null;
            }
            for (var index = 0; index < captureCameraObjects.Count; index++)
            {
                if (captureCameraObjects[index] != null)
                {
                    UnityEngine.Object.Destroy(captureCameraObjects[index]);
                }
            }
            captureCameras.Clear();
            captureCameraObjects.Clear();
            sources.Clear();
            width = 0;
            height = 0;
        }

        private void EnsureResources(int imageWidth, int imageHeight)
        {
            if (renderTexture != null && readbackTexture != null
                && width == imageWidth && height == imageHeight)
            {
                return;
            }

            Dispose();
            width = imageWidth;
            height = imageHeight;
            renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            renderTexture.name = "Kerbal composite RGB " + width + "x" + height;
            renderTexture.Create();
            readbackTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        }

        private void CollectSourceCameras()
        {
            sources.Clear();
            var seen = new HashSet<Camera>();

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
            AddSource(Camera.main, false, false, seen);

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

        private Camera GetCaptureCamera(int index)
        {
            while (captureCameras.Count <= index)
            {
                var cameraObject = new GameObject("Kerbal Composite RGB Capture Camera");
                cameraObject.hideFlags = HideFlags.HideAndDontSave;
                var camera = cameraObject.AddComponent<Camera>();
                camera.enabled = false;
                captureCameraObjects.Add(cameraObject);
                captureCameras.Add(camera);
            }
            return captureCameras[index];
        }

        private void RenderSource(
            SourceCamera source,
            Camera captureCamera,
            Vector3 origin,
            Quaternion rotation,
            float verticalFovDegrees,
            float nearClipMeters,
            float farClipMeters)
        {
            captureCamera.CopyFrom(source.Camera);
            captureCamera.enabled = false;
            captureCamera.targetTexture = renderTexture;
            captureCamera.rect = new Rect(0f, 0f, 1f, 1f);
            captureCamera.ResetProjectionMatrix();
            captureCamera.fieldOfView = verticalFovDegrees;
            captureCamera.aspect = (float)width / height;
            captureCamera.transform.rotation = rotation;

            if (source.IsGalaxy)
            {
                // Galaxy geometry follows the stock galaxy camera and only needs
                // the sensor's viewing rotation; translating it would add parallax.
                captureCamera.transform.position = source.Camera.transform.position;
            }
            else if (source.IsScaledSpace)
            {
                captureCamera.transform.position = ScaledSpace.LocalToScaledSpace(origin);
            }
            else
            {
                captureCamera.transform.position = origin;
                captureCamera.nearClipPlane = Mathf.Max(nearClipMeters, source.Camera.nearClipPlane);
                captureCamera.farClipPlane = Mathf.Min(farClipMeters, source.Camera.farClipPlane);
                if (captureCamera.farClipPlane <= captureCamera.nearClipPlane)
                {
                    return;
                }
            }

            captureCamera.Render();
        }

        private byte[] ReadTopDownRgb()
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
