using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalRgbCamera
    {
        // KSP Part.Update only dispatches PartModule.OnUpdate in Flight.
        // Unity Update also runs in VAB/SPH, where there is no active vessel.
        public void Update()
        {
            if (HighLogic.LoadedSceneIsEditor && isEnabled)
                UpdateCameraFrame();
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            // Keep the two callbacks exclusive, even if an editor tool invokes
            // OnUpdate explicitly. Flight retains KSP's pause/activation gating.
            if (!HighLogic.LoadedSceneIsEditor)
                UpdateCameraFrame();
        }

        private void UpdateCameraFrame()
        {
            UpdateCameraGimbal();
            if (!IsPreviewSceneAvailable())
            {
                ClosePreview();
                SendInactive();
                DestroyCaptureResources();
                return;
            }

            // Editor previews never publish frames into the active flight topics.
            var sendUdp = HighLogic.LoadedSceneIsFlight && udpEnabled;
            if (!cameraEnabled || !sendUdp)
            {
                SendInactive();
            }
            if (!cameraEnabled || (!sendUdp && !previewVisible))
            {
                DestroyCaptureResources();
                return;
            }

            var now = Time.realtimeSinceStartup;
            if (now < nextFrameTime)
            {
                return;
            }

            nextFrameTime = now + 1f / Mathf.Max(1f, frameRateHz);
            CaptureFrame(sendUdp);
        }

    }
}
