using System.Collections.Generic;
using UnityEngine;

namespace PyLoN
{
    public sealed partial class ModulePyLoNLinearMotor
    {
        private Collider[] movingColliders = new Collider[0];
        private Collider[] outputColliders = new Collider[0];

        private void CacheMovingContactPairs()
        {
            var moving = new HashSet<Collider>();
            foreach (var stage in new[] { movingTransform, intermediateTransform })
                if (stage != null)
                    foreach (var collider in stage.GetComponentsInChildren<Collider>(true))
                        moving.Add(collider);
            movingColliders = new List<Collider>(moving).ToArray();

            var output = new HashSet<Collider>();
            var branch = new List<Part>();
            // With the actuator mounted upside down, the output is the parent.
            // Do not walk back through this actuator into its fixed-side branch.
            CollectOutputContactParts(drivenPart, part, branch);
            foreach (var member in branch)
                foreach (var collider in member.GetComponentsInChildren<Collider>(true))
                    if (collider.attachedRigidbody != part.rb)
                        output.Add(collider);
            outputColliders = new List<Collider>(output).ToArray();
            ExcludeMovingOutputContacts();
        }

        private static void CollectOutputContactParts(Part member, Part actuator, List<Part> branch)
        {
            if (member == null || member == actuator) return;
            branch.Add(member);
            foreach (var child in member.children)
                CollectOutputContactParts(child, actuator, branch);
        }

        private void ExcludeMovingOutputContacts()
        {
            // Animated stages still belong to the fixed-side Rigidbody. Their
            // previous-frame pose otherwise blocks the payload during retraction.
            // Ignore only these contact pairs: raycasts and housing/world contacts
            // remain available. KSP may rebuild its ignore table after unpack.
            foreach (var moving in movingColliders)
                foreach (var output in outputColliders)
                    if (moving != null && output != null && moving != output &&
                        moving.enabled && output.enabled &&
                        !Physics.GetIgnoreCollision(moving, output))
                        Physics.IgnoreCollision(moving, output, true);
        }
    }
}
