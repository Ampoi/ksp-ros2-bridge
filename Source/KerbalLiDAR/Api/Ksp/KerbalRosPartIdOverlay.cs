using System.Collections.Generic;
using KSP.UI.Screens;
using ModuleWheels;
using UnityEngine;

namespace KerbalLiDAR
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public sealed class KerbalRosPartIdOverlay : MonoBehaviour
    {
        private readonly List<Rect> occupiedLabels = new List<Rect>();
        private readonly Dictionary<Part, List<string>> partIds = new Dictionary<Part, List<string>>();
        private ApplicationLauncherButton button;
        private Texture2D icon;
        private GUIStyle labelStyle;
        private bool hovered;
        private bool pinned;
        private bool uiHidden;

        private void CollectPartIds(Vessel vessel)
        {
            if (vessel == null || vessel.parts == null) return;
            foreach (var part in vessel.parts)
            {
                if (part == null) continue;
                for (var index = 0; index < part.Modules.Count; index++)
                {
                    var module = part.Modules[index];
                    string kind = null;
                    string id = null;
                    var sensor = module as ModuleKerbalRosSensorId;
                    var motor = module as IKerbalRosMotor;
                    if (sensor != null) { kind = sensor.sensorKind; id = ModuleKerbalRosSensorId.Resolve(part, kind); }
                    else if (motor != null) { kind = "servo"; id = motor.JointName; }
                    else if (module is ModuleWheelBase) kind = "wheel";
                    else if (module is ModuleEngines) kind = "engine";
                    else if (module is ModuleRCS) kind = "rcs";
                    else if (module is ModuleDecouplerBase) kind = "decoupler";
                    else if (module is ModuleProceduralFairing) kind = "fairing";
                    else if (module is ModuleDockingNode) kind = "docking_port";
                    if (kind == null) continue;
                    if (id == null) id = KerbalRosActuatorNames.For(kind, part, index);
                    AddPartId(part, id);
                }
            }
        }

        public void Start()
        {
            GameEvents.onGUIApplicationLauncherReady.Add(AddButton);
            GameEvents.onGUIApplicationLauncherDestroyed.Add(RemoveButton);
            GameEvents.onHideUI.Add(HideUi);
            GameEvents.onShowUI.Add(ShowUi);
            if (ApplicationLauncher.Ready) AddButton();
        }

        private void AddButton()
        {
            if (button != null || ApplicationLauncher.Instance == null) return;
            if (icon == null) icon = CreateIcon();
            button = ApplicationLauncher.Instance.AddModApplication(
                () => pinned = true, () => pinned = false,
                () => hovered = true, () => hovered = false,
                null, () => hovered = false,
                ApplicationLauncher.AppScenes.FLIGHT, icon);
        }

        private void RemoveButton()
        {
            hovered = false;
            pinned = false;
            if (button != null && ApplicationLauncher.Instance != null)
                ApplicationLauncher.Instance.RemoveModApplication(button);
            button = null;
        }

        private void HideUi()
        {
            uiHidden = true;
            hovered = false;
        }

        private void ShowUi() { uiHidden = false; }

        public void OnDestroy()
        {
            GameEvents.onGUIApplicationLauncherReady.Remove(AddButton);
            GameEvents.onGUIApplicationLauncherDestroyed.Remove(RemoveButton);
            GameEvents.onHideUI.Remove(HideUi);
            GameEvents.onShowUI.Remove(ShowUi);
            RemoveButton();
            if (icon != null) Destroy(icon);
        }

        public void OnGUI()
        {
            // Labels are passive: do not consume mouse events or lock flight controls.
            if (Event.current.type != EventType.Repaint || uiHidden || (!hovered && !pinned)
                || !HighLogic.LoadedSceneIsFlight || MapView.MapIsEnabled) return;

            var vessel = FlightGlobals.ActiveVessel;
            var camera = FlightCamera.fetch != null ? FlightCamera.fetch.mainCamera : null;
            if (vessel == null || camera == null || !camera.isActiveAndEnabled) return;

            if (labelStyle == null)
            {
                labelStyle = new GUIStyle(HighLogic.Skin.box)
                {
                    fontSize = 12,
                    alignment = TextAnchor.MiddleLeft,
                    padding = new RectOffset(6, 6, 3, 3),
                    wordWrap = false,
                    richText = false
                };
                labelStyle.normal.textColor = Color.white;
            }

            occupiedLabels.Clear();
            partIds.Clear();
            CollectPartIds(vessel);
            // Read the current vessel each repaint so staging, docking, and vessel
            // switching cannot leave labels attached to stale parts.
            for (var index = 0; index < vessel.parts.Count; index++)
            {
                var part = vessel.parts[index];
                if (part == null || part.transform == null) continue;
                List<string> ids;
                if (!partIds.TryGetValue(part, out ids)) continue;
                var screen = camera.WorldToScreenPoint(part.transform.position);
                if (screen.z <= 0f || screen.x < 0f || screen.x > Screen.width
                    || screen.y < 0f || screen.y > Screen.height) continue;

                var content = new GUIContent(string.Join("\n", ids));
                var size = labelStyle.CalcSize(content);
                var anchor = new Vector2(screen.x, Screen.height - screen.y);
                var rect = PlaceLabel(anchor, size);
                occupiedLabels.Add(rect);
                DrawLeader(anchor, new Vector2(
                    Mathf.Clamp(anchor.x, rect.xMin, rect.xMax),
                    Mathf.Clamp(anchor.y, rect.yMin, rect.yMax)));
                GUI.Label(rect, content, labelStyle);
            }

            if (hovered)
            {
                GUI.Label(new Rect(12f, 60f, 410f, 28f),
                    pinned ? "ROS2 IDs pinned - click ID to unpin" : "ROS2 IDs - click ID to pin", labelStyle);
            }
        }

        private void AddPartId(Part part, string id)
        {
            if (part == null || string.IsNullOrEmpty(id)) return;
            List<string> ids;
            if (!partIds.TryGetValue(part, out ids))
            {
                ids = new List<string>();
                partIds.Add(part, ids);
            }
            if (!ids.Contains(id)) ids.Add(id);
        }

        private Rect PlaceLabel(Vector2 anchor, Vector2 size)
        {
            var initial = ClampLabel(new Rect(anchor.x + 8f, anchor.y - size.y * 0.5f, size.x, size.y));
            // Try nearby rows on either side before accepting an overlap on a
            // crowded screen. Leader lines preserve the part-to-label association.
            var rows = Mathf.CeilToInt(Screen.height / (size.y + 3f));
            for (var row = 0; row <= rows; row++)
            {
                for (var side = 0; side < 2; side++)
                {
                    for (var direction = -1; direction <= 1; direction += 2)
                    {
                        var candidate = ClampLabel(new Rect(
                            side == 0 ? anchor.x + 8f : anchor.x - size.x - 8f,
                            initial.y + direction * row * (size.y + 3f), size.x, size.y));
                        var overlaps = false;
                        for (var index = 0; index < occupiedLabels.Count; index++)
                        {
                            if (!candidate.Overlaps(occupiedLabels[index])) continue;
                            overlaps = true;
                            break;
                        }
                        if (!overlaps) return candidate;
                    }
                }
            }
            return initial;
        }

        private static Rect ClampLabel(Rect rect)
        {
            rect.x = Mathf.Clamp(rect.x, 0f, Mathf.Max(0f, Screen.width - rect.width));
            rect.y = Mathf.Clamp(rect.y, 0f, Mathf.Max(0f, Screen.height - rect.height));
            return rect;
        }

        private static void DrawLeader(Vector2 start, Vector2 end)
        {
            var previousMatrix = GUI.matrix;
            var previousColor = GUI.color;
            try
            {
                GUI.color = new Color(0.65f, 0.9f, 1f, 0.8f);
                GUI.DrawTexture(new Rect(start.x - 2f, start.y - 2f, 4f, 4f), Texture2D.whiteTexture);
                var delta = end - start;
                GUIUtility.RotateAroundPivot(Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg, start);
                GUI.DrawTexture(new Rect(start.x, start.y, delta.magnitude, 1f), Texture2D.whiteTexture);
            }
            finally
            {
                GUI.matrix = previousMatrix;
                GUI.color = previousColor;
            }
        }

        private static Texture2D CreateIcon()
        {
            // A small pixel-font "ID" icon, with no external texture dependency.
            var glyph = new[] { "111001110", "010001001", "010001001", "010001001", "111001110" };
            var texture = new Texture2D(38, 38, TextureFormat.RGBA32, false);
            var pixels = new Color32[38 * 38];
            for (var y = 0; y < 38; y++)
            {
                for (var x = 0; x < 38; x++)
                {
                    var column = (x - 5) / 3;
                    var row = (y - 11) / 3;
                    var ink = x >= 5 && x < 32 && y >= 11 && y < 26 && glyph[row][column] == '1';
                    pixels[y * 38 + x] = ink
                        ? new Color32(180, 235, 255, 255) : new Color32(25, 35, 42, 255);
                }
            }
            texture.SetPixels32(pixels);
            texture.Apply(false, true);
            return texture;
        }
    }
}
