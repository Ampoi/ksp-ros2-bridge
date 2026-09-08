using UnityEngine;

namespace PyLoN
{
    public partial class ModulePyLoNRgbCamera
    {
        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true,
            guiName = "Camera Tilt", guiUnits = " deg", guiFormat = "F0")]
        [UI_FloatRange(minValue = 0f, maxValue = 180f, stepIncrement = 1f)]
        public float cameraTiltDegrees = 90f;

        [KSPField]
        public string cameraPivotTransformName = "CameraPivot";

        [KSPField]
        public string cameraOpticalTransformName = "CameraOptical";

        private Transform cameraPivot;
        private Transform cameraOptical;
        private Transform cameraModel;

        private void UpdateCameraGimbal()
        {
            if (part == null) return;
            if (cameraModel == null)
            {
                // KSP renames the imported model root, but preserves its children.
                var mount = part.FindModelTransform("CameraMount");
                if (mount != null && mount.parent != part.transform)
                    cameraModel = mount.parent;
            }
            // Old craft persist the spotlight's nonzero surface node. The new
            // model's origin is its mount plane, so align it to the loaded node
            // without moving the part, its children, or the player's offsets.
            if (cameraModel != null && part.srfAttachNode != null)
                cameraModel.position = part.transform.TransformPoint(part.srfAttachNode.position);

            cameraTiltDegrees = float.IsNaN(cameraTiltDegrees) || float.IsInfinity(cameraTiltDegrees)
                ? 90f : Mathf.Clamp(cameraTiltDegrees, 0f, 180f);
            if (cameraPivot == null && part != null && !string.IsNullOrEmpty(cameraPivotTransformName))
            {
                cameraPivot = part.FindModelTransform(cameraPivotTransformName);
            }
            if (cameraOptical == null && part != null && !string.IsNullOrEmpty(cameraOpticalTransformName))
                cameraOptical = part.FindModelTransform(cameraOpticalTransformName);
            if (cameraPivot != null)
                // Absolute authored neutral prevents an editor/symmetry clone from
                // treating its already-tilted transform as another neutral pose.
                cameraPivot.localRotation = Quaternion.AngleAxis(-cameraTiltDegrees, Vector3.right);
        }

        // Rendering and UDP extrinsics must always use the same moving optical frame.
        private void ResolveCameraPose(Transform root, out Vector3 origin, out Quaternion rotation)
        {
            UpdateCameraGimbal();
            if (cameraOptical != null)
            {
                origin = cameraOptical.position;
                rotation = cameraOptical.rotation;
                return;
            }
            var forward = AxisToWorld(root, forwardAxis);
            var up = AxisToWorld(root, upAxis);
            Orthonormalize(ref forward, ref up, root);
            origin = root.TransformPoint(ParseVector3(cameraOriginLocalPosition, Vector3.zero));
            rotation = Quaternion.LookRotation(forward, up);
        }
    }
}
