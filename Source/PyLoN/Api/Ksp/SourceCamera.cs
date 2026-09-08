using UnityEngine;
namespace PyLoN
{
        internal sealed class SourceCamera
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

            public void Save(Camera backup)
            {
                SavedSettings = backup;
                SavedSettings.CopyFrom(Camera);
                SavedSettings.enabled = false;
                LocalPosition = Camera.transform.localPosition;
                LocalRotation = Camera.transform.localRotation;
                LocalScale = Camera.transform.localScale;
                WasEnabled = Camera.enabled;
                PixelRect = Camera.pixelRect;
                OriginalTarget = Camera.targetTexture;
            }
            public void RestoreSettings()
            {
                if (Camera == null) return;
                try { Camera.CopyFrom(SavedSettings); }
                finally
                {
                    Camera.targetTexture = OriginalTarget;
                    Camera.pixelRect = PixelRect;
                    Camera.enabled = WasEnabled;
                }
            }
            public void RestorePose()
            {
                if (Camera == null) return;
                Camera.transform.localPosition = LocalPosition;
                Camera.transform.localRotation = LocalRotation;
                Camera.transform.localScale = LocalScale;
            }
        }
}
