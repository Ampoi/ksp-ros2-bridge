using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using KerbalLiDAR.Domain;

namespace KerbalLiDAR
{
    public sealed class ModuleKerbalRosSensorId : PartModule
    {
        private const float ValidationInterval = 0.5f;

        [KSPField(isPersistant = true, guiActiveEditor = true, guiName = "ROS2 Sensor ID")]
        public string sensorId = "";

        [KSPField]
        public string sensorKind = "sensor";

        private float nextValidationTime;

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            var migrated = LegacyName();
            AssignUnique(string.IsNullOrEmpty(sensorId) ? migrated : sensorId, false);
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            if (!HighLogic.LoadedSceneIsEditor || Time.realtimeSinceStartup < nextValidationTime)
            {
                return;
            }

            nextValidationTime = Time.realtimeSinceStartup + ValidationInterval;
            AssignUnique(sensorId, true);
        }

        [KSPEvent(guiActive = false, guiActiveEditor = true, guiName = "Edit ROS2 Sensor ID", active = true)]
        public void EditSensorId()
        {
            var pending = Resolve(part, sensorKind);
            var dialogId = "kerbal_ros_sensor_id_" +
                (part == null ? "0" : part.craftID.ToString(CultureInfo.InvariantCulture));
            PopupDialog.DismissPopup(dialogId);
            var dialog = new MultiOptionDialog(
                dialogId,
                "Use letters, numbers, and underscores. IDs are unique within this craft.",
                "ROS2 Sensor ID",
                HighLogic.UISkin,
                new DialogGUIBase[]
                {
                    new DialogGUITextInput(
                        pending,
                        false,
                        SensorIdentity.MaxLength,
                        delegate(string value) { pending = value; return value; },
                        280f),
                    new DialogGUIButton(
                        "Save",
                        delegate
                        {
                            var assigned = AssignUnique(pending, true);
                            ScreenMessages.PostScreenMessage(
                                "ROS2 sensor ID: " + assigned,
                                3f,
                                ScreenMessageStyle.UPPER_CENTER);
                        },
                        true),
                    new DialogGUIButton("Cancel", delegate { }, true)
                });
            PopupDialog.SpawnPopupDialog(dialog, false, HighLogic.UISkin, true, string.Empty);
        }

        public static string Resolve(Part target, string fallback)
        {
            if (target != null)
            {
                var identity = target.FindModuleImplementing<ModuleKerbalRosSensorId>();
                if (identity != null && !string.IsNullOrEmpty(identity.sensorId))
                {
                    return Normalize(identity.sensorId);
                }
            }

            var normalized = Normalize(fallback);
            return string.IsNullOrEmpty(normalized) ? "sensor" : normalized;
        }

        public static string Normalize(string value)
        {
            return SensorIdentity.Normalize(value);
        }

        private string AssignUnique(string requested, bool markModified)
        {
            var baseId = Normalize(requested);
            if (string.IsNullOrEmpty(baseId))
            {
                baseId = DefaultId();
            }

            var used = OtherSensorIds();
            var unique = baseId;
            var suffix = 2;
            while (used.Contains(unique))
            {
                var suffixText = "_" + suffix.ToString(CultureInfo.InvariantCulture);
                var length = Math.Max(1, Math.Min(baseId.Length, SensorIdentity.MaxLength - suffixText.Length));
                unique = baseId.Substring(0, length).TrimEnd('_') + suffixText;
                suffix++;
            }

            var changed = !string.Equals(sensorId, unique, StringComparison.Ordinal);
            sensorId = unique;
            if (changed && markModified && HighLogic.LoadedSceneIsEditor &&
                EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
            }
            return unique;
        }

        private string DefaultId()
        {
            var kind = Normalize(sensorKind);
            if (string.IsNullOrEmpty(kind))
            {
                kind = "sensor";
            }
            return kind + "_" + Guid.NewGuid().ToString("N").Substring(0, 8);
        }

        private string LegacyName()
        {
            if (part == null)
            {
                return string.Empty;
            }
            var lidar = part.FindModuleImplementing<ModuleKerbalLidar>();
            if (lidar != null)
            {
                return !string.IsNullOrEmpty(lidar.partName) ? lidar.partName : lidar.lidarName;
            }
            var camera = part.FindModuleImplementing<ModuleKerbalRgbCamera>();
            return camera == null ? string.Empty : camera.partName;
        }

        private HashSet<string> OtherSensorIds()
        {
            var ids = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            IList<Part> parts = null;
            if (HighLogic.LoadedSceneIsEditor && EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                parts = EditorLogic.fetch.ship.parts;
            }
            else if (vessel != null)
            {
                parts = vessel.parts;
            }
            if (parts == null)
            {
                return ids;
            }
            for (var index = 0; index < parts.Count; index++)
            {
                var candidate = parts[index];
                if (candidate == null || candidate == part)
                {
                    continue;
                }
                var identity = candidate.FindModuleImplementing<ModuleKerbalRosSensorId>();
                var value = identity == null ? string.Empty : Normalize(identity.sensorId);
                if (!string.IsNullOrEmpty(value))
                {
                    ids.Add(value);
                }
            }
            return ids;
        }
    }
}
