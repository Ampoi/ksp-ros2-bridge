using UnityEngine;

namespace PyLoN
{
    public partial class ModulePyLoNRgbCamera
    {
        private bool previewVisible;
        private bool previewHasFrame;
        private bool previewCaptureFailed;
        private bool previewUiHidden;
        private bool previewInputLocked;
        private Rect previewWindow = new Rect(80f, 80f, 344f, 320f);

        private string PreviewLockId { get { return "PyLoNRgbPreview_" + GetInstanceID(); } }

        private bool IsPreviewSceneAvailable()
        {
            return HighLogic.LoadedSceneIsEditor
                || (HighLogic.LoadedSceneIsFlight && vessel != null && FlightGlobals.ActiveVessel == vessel);
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "Show RGB Preview", active = true)]
        public void TogglePreview()
        {
            if (previewVisible)
            {
                ClosePreview();
                return;
            }
            if (!IsPreviewSceneAvailable())
            {
                return;
            }

            previewVisible = true;
            previewCaptureFailed = false;
            Events[nameof(TogglePreview)].guiName = "Hide RGB Preview";
            nextFrameTime = 0f;
        }

        private void InitializePreview()
        {
            GameEvents.onHideUI.Remove(HidePreviewUi);
            GameEvents.onShowUI.Remove(ShowPreviewUi);
            GameEvents.onHideUI.Add(HidePreviewUi);
            GameEvents.onShowUI.Add(ShowPreviewUi);
        }

        private void HidePreviewUi()
        {
            previewUiHidden = true;
            ReleasePreviewInput();
        }

        private void ShowPreviewUi()
        {
            previewUiHidden = false;
        }

        private void ClosePreview()
        {
            previewVisible = false;
            ReleasePreviewInput();
            if (Events[nameof(TogglePreview)] != null)
            {
                Events[nameof(TogglePreview)].guiName = "Show RGB Preview";
            }
        }

        private void DestroyPreview()
        {
            ClosePreview();
            GameEvents.onHideUI.Remove(HidePreviewUi);
            GameEvents.onShowUI.Remove(ShowPreviewUi);
        }

        public void OnDisable()
        {
            ClosePreview();
            DestroyCaptureResources();
        }

        public void OnGUI()
        {
            if (!previewVisible || previewUiHidden || !IsPreviewSceneAvailable())
            {
                ReleasePreviewInput();
                return;
            }

            var previousSkin = GUI.skin;
            try
            {
                GUI.skin = HighLogic.Skin;
                var imageAspect = (float)Mathf.Max(1, imageWidth) / Mathf.Max(1, imageHeight);
                var imageDisplayHeight = Mathf.Min(240f, 320f / imageAspect);
                previewWindow.width = Mathf.Min(344f, Screen.width);
                previewWindow.height = Mathf.Min(imageDisplayHeight + 104f, Screen.height);
                previewWindow.x = Mathf.Clamp(previewWindow.x, 0f, Mathf.Max(0f, Screen.width - previewWindow.width));
                previewWindow.y = Mathf.Clamp(previewWindow.y, 0f, Mathf.Max(0f, Screen.height - previewWindow.height));
                previewWindow = GUI.Window(GetInstanceID(), previewWindow, DrawPreviewWindow, "RGB Camera Preview");
            }
            finally
            {
                GUI.skin = previousSkin;
            }

            var mouse = Event.current.mousePosition;
            if (previewVisible && previewWindow.Contains(mouse))
            {
                if (!previewInputLocked)
                {
                    var controls = HighLogic.LoadedSceneIsEditor
                        ? ControlTypes.EDITOR_PAD_PICK_PLACE | ControlTypes.EDITOR_PAD_PICK_COPY
                            | ControlTypes.EDITOR_GIZMO_TOOLS | ControlTypes.CAMERACONTROLS
                        : ControlTypes.ALL_SHIP_CONTROLS | ControlTypes.CAMERACONTROLS;
                    InputLockManager.SetControlLock(controls, PreviewLockId);
                    previewInputLocked = true;
                }
            }
            else
            {
                ReleasePreviewInput();
            }
        }

        private void DrawPreviewWindow(int windowId)
        {
            if (GUI.Button(new Rect(previewWindow.width - 30f, 2f, 26f, 22f), "X"))
            {
                ClosePreview();
            }

            GUI.Label(new Rect(12f, 26f, previewWindow.width - 24f, 22f), ResolveSensorId());
            var imageRect = new Rect(12f, 50f, previewWindow.width - 24f, previewWindow.height - 104f);
            GUI.DrawTexture(imageRect, Texture2D.blackTexture);
            if (!cameraEnabled)
            {
                GUI.Label(imageRect, "Camera is off. Enable Camera to preview.");
            }
            else if (previewHasFrame && sceneCapture != null && sceneCapture.PreviewTexture != null)
            {
                GUI.DrawTexture(imageRect, sceneCapture.PreviewTexture, ScaleMode.ScaleToFit, false);
            }
            else
            {
                GUI.Label(imageRect, previewCaptureFailed ? "Capture failed. Retrying..." : "Waiting for camera frame...");
            }

            GUI.Label(new Rect(12f, previewWindow.height - 50f, previewWindow.width - 24f, 22f),
                imageSizeStatus + "  |  " + frameRateHz.ToString("F0") + " Hz  |  FOV " + verticalFovDegrees.ToString("F0") + "°");
            GUI.Label(new Rect(12f, previewWindow.height - 28f, previewWindow.width - 24f, 22f),
                HighLogic.LoadedSceneIsEditor ? "Editor preview (no UDP)"
                    : udpEnabled ? "UDP: On" : "UDP: Off (local preview)");
            GUI.DragWindow(new Rect(0f, 0f, previewWindow.width - 34f, 24f));
        }

        private void ReleasePreviewInput()
        {
            if (!previewInputLocked) return;
            InputLockManager.RemoveControlLock(PreviewLockId);
            previewInputLocked = false;
        }
    }
}
