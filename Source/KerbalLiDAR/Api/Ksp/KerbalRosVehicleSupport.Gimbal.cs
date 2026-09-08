using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public sealed partial class KerbalRosVehicleManager
    {
        private sealed class GimbalOverride
        {
            public bool WasEnabled;
            public string Owner;
            public EngineOverride Command;
        }

        private readonly Dictionary<ModuleGimbal, GimbalOverride> gimbalOverrides =
            new Dictionary<ModuleGimbal, GimbalOverride>();

        private static bool ValidGimbalAxis(double value)
        {
            return IsFinite(value) && value >= -1.0 && value <= 1.0;
        }

        private static IEnumerable<ModuleGimbal> EngineGimbals(ModuleEngines engine)
        {
            if (engine == null || engine.part == null) yield break;
            foreach (PartModule module in engine.part.Modules)
            {
                var gimbal = module as ModuleGimbal;
                if (gimbal == null || gimbal.gimbalTransforms == null) continue;
                // Match the nozzle hierarchy, not every gimbal on the same part.
                // This also handles multiple engine modes sharing one nozzle.
                var matches = false;
                foreach (var pivot in gimbal.gimbalTransforms)
                {
                    if (pivot == null || engine.thrustTransforms == null) continue;
                    foreach (var nozzle in engine.thrustTransforms)
                    {
                        if (nozzle != null && (nozzle == pivot || nozzle.IsChildOf(pivot)))
                            matches = true;
                    }
                }
                if (matches) yield return gimbal;
            }
        }

        private void ApplyGimbalCommands()
        {
            if (vessel == null || vessel.ctrlState == null) return;
            var selected = new Dictionary<ModuleGimbal, GimbalOverride>();
            foreach (var engine in EngineModules())
            {
                var name = KerbalRosActuatorNames.For("engine", engine.part,
                    KerbalRosActuatorNames.ModuleIndex(engine.part, engine));
                EngineOverride command;
                if (!engineOverrides.TryGetValue(name, out command) ||
                    !command.HasGimbalCommand || Time.realtimeSinceStartup > command.ExpiresAt) continue;
                foreach (var gimbal in EngineGimbals(engine))
                {
                    GimbalOverride other;
                    // Shared hardware receives one command per physics tick.
                    // The newest accepted sequence wins across engine modes.
                    if (selected.TryGetValue(gimbal, out other) &&
                        other.Command.Sequence >= command.Sequence) continue;
                    selected[gimbal] = new GimbalOverride { Owner = name, Command = command };
                }
            }
            foreach (var gimbal in new List<ModuleGimbal>(gimbalOverrides.Keys))
            {
                if (!selected.ContainsKey(gimbal)) RestoreGimbal(gimbal);
            }
            foreach (var pair in selected)
            {
                var gimbal = pair.Key;
                GimbalOverride original;
                if (!gimbalOverrides.TryGetValue(gimbal, out original))
                {
                    original = new GimbalOverride { WasEnabled = gimbal.enabled };
                    gimbalOverrides.Add(gimbal, original);
                }
                original.Owner = pair.Value.Owner;
                original.Command = pair.Value.Command;
                // Suppress Unity's automatic call so the stock module cannot
                // overwrite the individual input with vessel-wide steering.
                gimbal.enabled = false;
                if (!original.WasEnabled) continue;
                var state = vessel.ctrlState;
                var pitch = state.pitch;
                var yaw = state.yaw;
                var roll = state.roll;
                try
                {
                    state.pitch = original.Command.GimbalInput.x;
                    state.roll = original.Command.GimbalInput.y;
                    state.yaw = original.Command.GimbalInput.z;
                    // Stock code retains response speed, limiter, lock, axis
                    // toggles, CoM geometry, and engine/nozzle transforms.
                    gimbal.FixedUpdate();
                }
                finally
                {
                    state.pitch = pitch;
                    state.yaw = yaw;
                    state.roll = roll;
                }
            }
        }

        private void RestoreGimbals(string owner)
        {
            foreach (var gimbal in new List<ModuleGimbal>(gimbalOverrides.Keys))
            {
                if (owner == null || gimbalOverrides[gimbal].Owner == owner)
                    RestoreGimbal(gimbal);
            }
        }

        private void RestoreGimbal(ModuleGimbal gimbal)
        {
            GimbalOverride original;
            if (!gimbalOverrides.TryGetValue(gimbal, out original)) return;
            if (gimbal != null)
            {
                // Neutralize stale deflection immediately on timeout or lease
                // loss, including when the vessel no longer receives callbacks.
                if (gimbal.gimbalTransforms != null && gimbal.initRots != null)
                {
                    for (var index = 0; index < gimbal.gimbalTransforms.Count &&
                        index < gimbal.initRots.Count; index++)
                    {
                        if (gimbal.gimbalTransforms[index] != null)
                            gimbal.gimbalTransforms[index].localRotation = gimbal.initRots[index];
                    }
                }
                gimbal.actuation = Vector3.zero;
                gimbal.actuationLocal = Vector3.zero;
                if (gimbal.oldActuationLocal != null)
                    System.Array.Clear(gimbal.oldActuationLocal, 0, gimbal.oldActuationLocal.Length);
                gimbal.enabled = original.WasEnabled;
            }
            gimbalOverrides.Remove(gimbal);
        }

        private void AppendGimbalState(StringBuilder builder, ModuleEngines engine)
        {
            var available = false;
            GimbalOverride active = null;
            foreach (var gimbal in EngineGimbals(engine))
            {
                available = true;
                GimbalOverride candidate;
                if (gimbalOverrides.TryGetValue(gimbal, out candidate) &&
                    (active == null || candidate.Command.Sequence > active.Command.Sequence))
                    active = candidate;
            }
            AppendBoolean(builder, "gimbalAvailable", available);
            AppendBoolean(builder, "gimbalCommandActive", active != null);
            var input = active == null ? Vector3.zero : active.Command.GimbalInput;
            AppendNumber(builder, "gimbalPitch", input.x);
            AppendNumber(builder, "gimbalYaw", input.z);
            AppendNumber(builder, "gimbalRoll", input.y);
        }
    }
}
