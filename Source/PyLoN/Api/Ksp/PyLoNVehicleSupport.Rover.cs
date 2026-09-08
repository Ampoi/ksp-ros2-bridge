using UnityEngine;
using ModuleWheels;
namespace PyLoN { public sealed partial class PyLoNVehicleManager {
        private void ParkRover()
        {
            if (vessel == null) return;
            vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
            foreach (var wheel in parts.Get<ModuleWheelBase>())
            {
                if (wheel.Wheel == null) continue;
                wheel.Wheel.driveInput = 0f;
                wheel.Wheel.brakeInput = 1f;
                var brake = wheel.part.FindModuleImplementing<ModuleWheelBrakes>();
                if (brake != null) brake.brakeInput = 1f;
            }
        }

    }
}
