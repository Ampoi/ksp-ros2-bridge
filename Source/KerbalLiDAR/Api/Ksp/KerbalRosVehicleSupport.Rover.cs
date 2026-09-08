using System;
using System.Text;
using UnityEngine;
using ModuleWheels;

namespace KerbalLiDAR
{
    public sealed partial class KerbalRosVehicleManager
    {
        internal static string RuntimeEpoch = Guid.NewGuid().ToString("N");
        private Vector3 roverMinimum, roverMaximum;
        private int roverWheelCount;
        private float nextRoverGeometry;

        private void ParkRover()
        {
            if (vessel == null) return;
            vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
            foreach (var wheel in WheelBases())
            {
                if (wheel.Wheel == null) continue;
                wheel.Wheel.driveInput = 0f;
                wheel.Wheel.brakeInput = 1f;
                var brake = wheel.part.FindModuleImplementing<ModuleWheelBrakes>();
                if (brake != null) brake.brakeInput = 1f;
            }
        }

        private void UpdateRoverGeometry()
        {
            if (Time.realtimeSinceStartup < nextRoverGeometry) return;
            nextRoverGeometry = Time.realtimeSinceStartup + 1f;
            roverMinimum = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            roverMaximum = -roverMinimum;
            roverWheelCount = 0;
            foreach (var wheel in WheelBases()) roverWheelCount++;
            foreach (var part in vessel.parts)
            {
                // Collider bounds deliberately overestimate rotated mesh bounds.
                foreach (var collider in part.GetComponentsInChildren<Collider>())
                {
                    if (!collider.enabled || collider.isTrigger) continue;
                    var bounds = collider.bounds;
                    for (int corner = 0; corner < 8; corner++)
                    {
                        var point = bounds.center + Vector3.Scale(bounds.extents,
                            new Vector3((corner & 1) == 0 ? -1 : 1,
                                (corner & 2) == 0 ? -1 : 1, (corner & 4) == 0 ? -1 : 1));
                        var local = WorldVectorToBody(point - (Vector3)vessel.CurrentCoM);
                        roverMinimum = Vector3.Min(roverMinimum, local);
                        roverMaximum = Vector3.Max(roverMaximum, local);
                    }
                }
            }
            if (float.IsInfinity(roverMinimum.x)) roverMinimum = roverMaximum = Vector3.zero;
        }

        private void AppendWheelGeometry(StringBuilder builder, ModuleWheelBase wheel)
        {
            var controller = wheel.Wheel;
            var frame = controller.wcTransform;
            var steer = wheel.part.FindModuleImplementing<ModuleWheelSteering>();
            var position = frame == null ? wheel.part.transform.position : frame.position;
            AppendString(builder, "vesselId", ActiveVesselId());
            AppendNumber(builder, "wheelCount", roverWheelCount);
            AppendNumber(builder, "radius", controller.WheelRadius);
            AppendVector(builder, "position", WorldVectorToBody(position - (Vector3)vessel.CurrentCoM));
            // Direct wheel commands bypass ModuleWheelMotor's inversion flags.
            // Signs refer to the physical, unsteered wheel collider frame.
            var forward = frame == null ? Vector3.zero : frame.forward;
            var up = frame == null ? Vector3.zero : frame.up;
            var alignment = Vector3.Dot(forward, BodyForwardWorld());
            AppendNumber(builder, "rollingSign", Mathf.Abs(alignment) > 0.9f ? Mathf.Sign(alignment) : 0f);
            var upAlignment = Vector3.Dot(up, BodyUpWorld());
            AppendNumber(builder, "steeringSign", Mathf.Abs(upAlignment) > 0.9f ? -Mathf.Sign(upAlignment) : 0f);
            AppendBoolean(builder, "steeringEnabled", steer != null && steer.steeringEnabled);
            AppendNumber(builder, "maxSteeringAngle", Mathf.Abs(controller.maxSteerAngle) * Mathf.Deg2Rad);
            AppendVector(builder, "bodyMin", roverMinimum);
            AppendVector(builder, "bodyMax", roverMaximum);
        }
    }
}
