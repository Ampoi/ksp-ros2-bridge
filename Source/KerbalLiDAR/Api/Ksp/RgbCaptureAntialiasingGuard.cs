using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace KerbalLiDAR
{
    // Scatterer bases its TAA quality switch on Time.deltaTime. A synchronous
    // sensor readback can make only the next game frame miss that threshold.
    // Discount our measured work, without changing time or saved MOD settings.
    internal static class RgbCaptureAntialiasingGuard
    {
        private static int users;
        private static int preparedFrame = -1;
        private static readonly List<RgbCaptureAntialiasingCameraGuard> cameraGuards = new List<RgbCaptureAntialiasingCameraGuard>();
        private static int captureFrame = -2;
        private static float captureSeconds;
        private static int previousCaptureFrame = -2;
        private static float previousCaptureSeconds;
        private static object settings;
        private static FieldInfo thresholdField;
        private static object overriddenSettings;
        private static FieldInfo overriddenField;
        private static int originalThreshold;
        private static Camera overriddenCamera;

        internal static void Retain()
        {
            users++;
        }

        internal static void Release()
        {
            if (--users != 0) return;
            RestoreThreshold();
            foreach (var guard in cameraGuards)
                if (guard != null) UnityEngine.Object.Destroy(guard);
            cameraGuards.Clear();
            preparedFrame = -1;
            settings = null;
            thresholdField = null;
            captureFrame = -2;
            captureSeconds = 0f;
            previousCaptureFrame = -2;
            previousCaptureSeconds = 0f;
        }

        internal static void ObserveEffect(Camera camera, Behaviour effect)
        {
            // Optional dependency; only bind the known field with its
            // expected type. No Scatterer or a different API means no override.
            var type = effect.GetType().Assembly.GetType("Scatterer.Scatterer");
            var instanceProperty = type == null ? null : type.GetProperty("Instance", BindingFlags.Public | BindingFlags.Static);
            var settingsField = type == null ? null : type.GetField("mainSettings");
            if (instanceProperty == null || settingsField == null) return;
            var instance = instanceProperty.GetValue(null, null);
            if (instance == null) return;
            var value = settingsField.GetValue(instance);
            if (value == null) return;
            var field = value.GetType().GetField("disableTaaBelowFrameRateThreshold");
            if (field == null || field.FieldType != typeof(int)) return;
            settings = value;
            thresholdField = field;
            if (camera.GetComponent<RgbCaptureAntialiasingCameraGuard>() == null)
                cameraGuards.Add(camera.gameObject.AddComponent<RgbCaptureAntialiasingCameraGuard>());
        }

        internal static void RecordCapture(float seconds)
        {
            if (captureFrame != Time.frameCount)
            {
                // Another sensor can already have captured in this Update.
                // Time.deltaTime still describes the preceding frame's work.
                previousCaptureFrame = captureFrame;
                previousCaptureSeconds = captureSeconds;
                captureSeconds = 0f;
            }
            captureFrame = Time.frameCount;
            captureSeconds += Mathf.Max(0f, seconds);
        }

        internal static bool ShouldPreserve(float deltaTime, float unscaledDeltaTime,
            float timeScale, float sensorSeconds, float threshold)
        {
            if (threshold <= 0f || deltaTime <= 0f || timeScale <= 0f || sensorSeconds <= 0f)
                return false;
            var gameSeconds = Mathf.Max(0.001f, unscaledDeltaTime - sensorSeconds) * timeScale;
            return 1f / deltaTime < threshold && 1f / gameSeconds >= threshold;
        }

        internal static void PrepareFrame()
        {
            if (preparedFrame == Time.frameCount) return;
            preparedFrame = Time.frameCount;
            RestoreThreshold();
            var sensorSeconds = Time.frameCount == captureFrame + 1 ? captureSeconds
                : Time.frameCount == previousCaptureFrame + 1 ? previousCaptureSeconds : 0f;
            if (users == 0 || sensorSeconds <= 0f || settings == null || thresholdField == null) return;
            var threshold = (int)thresholdField.GetValue(settings);
            if (!ShouldPreserve(Time.deltaTime, Time.unscaledDeltaTime, Time.timeScale, sensorSeconds, threshold)) return;

            // All these cameras must see the compensated threshold in OnPreCull.
            // Restore at the final camera's OnPreRender, before image rendering
            // and OnGUI (where Scatterer's settings can be saved).
            Camera lastCamera = null;
            foreach (var guard in cameraGuards)
            {
                if (guard == null) continue;
                var camera = guard.GetComponent<Camera>();
                if (camera == null || !camera.enabled || !camera.gameObject.activeInHierarchy || camera.targetTexture != null) continue;
                foreach (var effect in camera.GetComponents<Behaviour>())
                    if (effect != null && effect.enabled && effect.GetType().FullName == "Scatterer.TemporalAntiAliasing"
                        && (lastCamera == null || camera.depth > lastCamera.depth)) lastCamera = camera;
            }
            if (lastCamera == null) return;
            overriddenSettings = settings;
            overriddenField = thresholdField;
            originalThreshold = threshold;
            overriddenCamera = lastCamera;
            thresholdField.SetValue(settings, 0);
        }

        internal static void AfterCamera(Camera camera)
        {
            if (camera == overriddenCamera) RestoreThreshold();
        }

        private static void RestoreThreshold()
        {
            if (overriddenSettings == null) return;
            overriddenField.SetValue(overriddenSettings, originalThreshold);
            overriddenSettings = null;
            overriddenField = null;
            overriddenCamera = null;
        }
    }

    // KSP runs global pre-cull callbacks after Scatterer's quality decision.
    // LateUpdate precedes the whole camera stack, independent of component order.
    internal sealed class RgbCaptureAntialiasingCameraGuard : MonoBehaviour
    {
        public void LateUpdate() { RgbCaptureAntialiasingGuard.PrepareFrame(); }
        public void OnPreRender() { RgbCaptureAntialiasingGuard.AfterCamera(GetComponent<Camera>()); }
        public void OnPostRender() { RgbCaptureAntialiasingGuard.AfterCamera(GetComponent<Camera>()); }
        public void OnDisable() { RgbCaptureAntialiasingGuard.AfterCamera(GetComponent<Camera>()); }
        public void OnDestroy() { RgbCaptureAntialiasingGuard.AfterCamera(GetComponent<Camera>()); }
    }
}
