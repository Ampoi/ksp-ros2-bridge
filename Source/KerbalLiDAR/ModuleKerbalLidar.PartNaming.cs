using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private const int MaxPartNameLength = 64;
        private const float PartNameValidationIntervalSeconds = 0.5f;

        private float nextPartNameValidationTime;

        [KSPEvent(guiActive = false, guiActiveEditor = false, guiName = "Edit ROS2 Part Name", active = false)]
        public void EditPartName()
        {
            var pendingName = ResolvePartName();
            var dialogId = PartNameDialogId();
            PopupDialog.DismissPopup(dialogId);

            var dialog = new MultiOptionDialog(
                dialogId,
                "Use letters, numbers, and underscores. The name is made unique within this craft.",
                "ROS2 Part Name",
                HighLogic.UISkin,
                new DialogGUIBase[]
                {
                    new DialogGUITextInput(
                        pendingName,
                        false,
                        MaxPartNameLength,
                        delegate(string value)
                        {
                            pendingName = value;
                            return value;
                        },
                        280f),
                    new DialogGUIButton(
                        "Save",
                        delegate
                        {
                            var assignedName = AssignUniquePartName(pendingName, true);
                            ScreenMessages.PostScreenMessage(
                                "ROS2 part name: " + assignedName,
                                3f,
                                ScreenMessageStyle.UPPER_CENTER);
                        },
                        true),
                    new DialogGUIButton("Cancel", delegate { }, true)
                });

            PopupDialog.SpawnPopupDialog(dialog, false, HighLogic.UISkin, true, string.Empty);
        }

        private void MaintainUniquePartName()
        {
            var now = Time.realtimeSinceStartup;
            if (now < nextPartNameValidationTime)
            {
                return;
            }

            nextPartNameValidationTime = now + PartNameValidationIntervalSeconds;
            EnsureUniquePartName(true);
        }

        private void EnsureUniquePartName(bool markEditorModified)
        {
            AssignUniquePartName(partName, markEditorModified);
        }

        private string AssignUniquePartName(string requestedName, bool markEditorModified)
        {
            var baseName = NormalizePartNameToken(requestedName);
            if (string.IsNullOrEmpty(baseName))
            {
                baseName = DefaultPartNameBase();
            }

            var usedNames = CollectOtherPartNames();
            var uniqueName = baseName;
            var suffix = 2;
            while (usedNames.Contains(uniqueName))
            {
                var suffixText = "_" + suffix.ToString(CultureInfo.InvariantCulture);
                var baseLength = Math.Min(baseName.Length, MaxPartNameLength - suffixText.Length);
                uniqueName = baseName.Substring(0, baseLength).TrimEnd('_') + suffixText;
                suffix++;
            }

            var changed = !string.Equals(partName, uniqueName, StringComparison.Ordinal)
                || !string.Equals(lidarName, uniqueName, StringComparison.Ordinal);
            partName = uniqueName;
            lidarName = uniqueName;
            if (changed && markEditorModified)
            {
                MarkEditorShipModified();
            }

            return uniqueName;
        }

        private static void MarkEditorShipModified()
        {
            if (HighLogic.LoadedSceneIsEditor && EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
            }
        }

        private HashSet<string> CollectOtherPartNames()
        {
            var usedNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            IList<Part> craftParts = null;

            if (HighLogic.LoadedSceneIsEditor && EditorLogic.fetch != null && EditorLogic.fetch.ship != null)
            {
                craftParts = EditorLogic.fetch.ship.parts;
            }
            else if (vessel != null)
            {
                craftParts = vessel.parts;
            }

            if (craftParts == null)
            {
                return usedNames;
            }

            for (var partIndex = 0; partIndex < craftParts.Count; partIndex++)
            {
                var candidatePart = craftParts[partIndex];
                if (candidatePart == null || candidatePart == part)
                {
                    continue;
                }

                for (var moduleIndex = 0; moduleIndex < candidatePart.Modules.Count; moduleIndex++)
                {
                    var camera = candidatePart.Modules[moduleIndex] as ModuleKerbalRgbCamera;
                    if (camera != null)
                    {
                        var cameraName = NormalizePartNameToken(camera.partName);
                        if (!string.IsNullOrEmpty(cameraName))
                        {
                            usedNames.Add(cameraName);
                        }
                        continue;
                    }

                    var module = candidatePart.Modules[moduleIndex] as ModuleKerbalLidar;
                    if (module == null)
                    {
                        continue;
                    }

                    var candidateName = NormalizePartNameToken(
                        !string.IsNullOrEmpty(module.partName) ? module.partName : module.lidarName);
                    if (!string.IsNullOrEmpty(candidateName))
                    {
                        usedNames.Add(candidateName);
                    }
                }
            }

            return usedNames;
        }

        private string DefaultPartNameBase()
        {
            var configuredName = part != null && part.partInfo != null
                ? part.partInfo.name
                : sensorMode + "_lidar";
            var normalized = NormalizePartNameToken(configuredName);
            return string.IsNullOrEmpty(normalized) ? "lidar" : normalized;
        }

        private static string NormalizePartNameToken(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
            {
                return string.Empty;
            }

            var builder = new StringBuilder(Math.Min(value.Length, MaxPartNameLength));
            var previousWasUnderscore = false;
            for (var index = 0; index < value.Length && builder.Length < MaxPartNameLength; index++)
            {
                var character = value[index];
                var isAsciiLetter = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z');
                var isDigit = character >= '0' && character <= '9';
                if (isAsciiLetter || isDigit)
                {
                    builder.Append(char.ToLowerInvariant(character));
                    previousWasUnderscore = false;
                }
                else if (!previousWasUnderscore && builder.Length > 0)
                {
                    builder.Append('_');
                    previousWasUnderscore = true;
                }
            }

            var normalized = builder.ToString().Trim('_');
            if (string.IsNullOrEmpty(normalized))
            {
                return string.Empty;
            }

            if (normalized[0] >= '0' && normalized[0] <= '9')
            {
                normalized = "_" + normalized;
                if (normalized.Length > MaxPartNameLength)
                {
                    normalized = normalized.Substring(0, MaxPartNameLength);
                }
            }

            return normalized;
        }

        private string PartNameDialogId()
        {
            var craftId = part != null ? part.craftID.ToString(CultureInfo.InvariantCulture) : "0";
            return "kerbal_lidar_part_name_" + craftId;
        }
    }
}
