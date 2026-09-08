using UnityEngine;

namespace KerbalLiDAR
{
    public sealed partial class ModuleKerbalRosLinearMotor
    {
        private AttachNode drivenNode;
        private Vector3 contractedNodePosition;

        private void CacheDrivenNode()
        {
            drivenNode = part.FindAttachNode(drivenNodeName);
            if (drivenNode != null)
                contractedNodePosition = drivenNode.originalPosition;
        }

        private void UpdateDrivenNode(float extension)
        {
            if (drivenNode != null)
                drivenNode.position = contractedNodePosition + Vector3.up * ClampExtension(extension);
        }

        private void ConfigureDrive()
        {
            EnsureJointConfiguration();
        }

        private void EnsureJointConfiguration()
        {
            if (!jointInitialized || pJoint == null || pJoint.joints == null)
                return;

            var count = 0;
            foreach (var slider in pJoint.joints)
                if (slider != null) count++;
            if (count == 0) return;

            // Ordinary KSP stack connections have THREE ConfigurableJoints.
            // Freeing only PartJoint.Joint leaves two stock springs resisting
            // translation. Restore every joint AND its drive after unpack and
            // KSP SetJointLimits calls, not just when the motor mode changes.
            foreach (var slider in pJoint.joints)
            {
                if (slider == null) continue;
                slider.xMotion = motorLocked ? ConfigurableJointMotion.Locked : ConfigurableJointMotion.Free;
                slider.yMotion = ConfigurableJointMotion.Locked;
                slider.zMotion = ConfigurableJointMotion.Locked;
                slider.angularXMotion = ConfigurableJointMotion.Locked;
                slider.angularYMotion = ConfigurableJointMotion.Locked;
                slider.angularZMotion = ConfigurableJointMotion.Locked;
                var drive = slider.xDrive;
                drive.positionSpring = 0f;
                drive.positionDamper = motorEngaged ? Mathf.Max(0f, jointDamper) / count : 0f;
                // KSP rigidbody mass is in tonnes: Unity force is kN here.
                // Split the effort budget across the parallel constraints.
                drive.maximumForce = motorEngaged ? Mathf.Max(0f, effortLimitN) * 0.001f / count : 0f;
                slider.xDrive = drive;
            }
        }

        private void SetLinearMotorSpeed(float speed)
        {
            if (pJoint == null || pJoint.joints == null) return;
            foreach (var slider in pJoint.joints)
            {
                if (slider != null)
                    slider.targetVelocity = new Vector3(speed, 0f, 0f);
            }
        }
    }
}
