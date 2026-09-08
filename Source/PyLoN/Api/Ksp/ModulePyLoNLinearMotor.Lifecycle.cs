using UnityEngine;

namespace PyLoN
{
    public sealed partial class ModulePyLoNLinearMotor
    {
        private float nextJointRetry;

        private void PreserveDrivenBranchPose()
        {
            if (drivenPart == null || drivenPart.parent != part || part.vessel == null || part.vessel.packed)
                return;
            PreservePartPose(drivenPart, part.vessel.rootPart);
        }

        private static void PreservePartPose(Part child, Part root)
        {
            if (child == null || root == null) return;
            // Stock ModuleJointMotor stores rotations, but never slider travel.
            // Keep KSP's saved/packed vessel coordinates aligned with the real
            // output branch so reloading does not return the payload to rest.
            child.UpdateOrgPosAndRot(root);
            foreach (var descendant in child.children)
                PreservePartPose(descendant, root);
        }

        private void EnsureFlightJoint()
        {
            if (part == null || part.vessel == null || part.vessel.packed || !part.started)
                return;

            if (jointInitialized && joint != null && drivenPart != null &&
                part.FindAttachNode(drivenNodeName)?.attachedPart == drivenPart)
                return;

            if (Time.realtimeSinceStartup < nextJointRetry)
                return;
            nextJointRetry = Time.realtimeSinceStartup + 0.5f;

            var output = part.FindAttachNode(drivenNodeName)?.attachedPart;
            if (output == null || !output.started)
                return;

            // Stock cargo/accessory parts (e.g. the small fireworks launcher)
            // normally inherit their parent's Rigidbody and have no attachJoint.
            // A driven output needs its own body. Use KSP's promotion API so
            // masses, child joints, collision ownership and velocities are updated.
            if (output.physicalSignificance == Part.PhysicalSignificance.NONE)
            {
                output.PromoteToPhysicalPart();
                output.CreateAttachJoint(output.attachMode);
                Debug.Log("[PyLoN] Promoted linear actuator output to a physical part: " + output.name);
            }

            var connection = output == part.parent ? part.attachJoint : output.attachJoint;
            if (connection == null || connection.joints == null || connection.joints.Count == 0 ||
                connection.joints[0] == null)
                return;

            // ModuleJointMotor only waits for this part's started flag. Its
            // initial attempt can precede the child's joint creation, so retry
            // only once the real stack connection exists and physics is active.
            InitJoint();
        }
    }
}
