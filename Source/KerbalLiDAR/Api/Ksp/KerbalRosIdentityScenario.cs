using System;
using System.Collections.Generic;
using System.Globalization;

namespace KerbalLiDAR
{
    /// <summary>Stock module aliases belong to the save, never to global part configs.</summary>
    [KSPScenario(ScenarioCreationOptions.AddToAllGames, GameScenes.FLIGHT, GameScenes.EDITOR, GameScenes.SPACECENTER)]
    public sealed class KerbalRosIdentityScenario : ScenarioModule
    {
        internal static KerbalRosIdentityScenario Instance;
        private readonly Dictionary<string, string> aliases = new Dictionary<string, string>();

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            Instance = this;
            aliases.Clear();
            foreach (var item in node.GetNodes("IDENTITY"))
            {
                var key = item.GetValue("key");
                var id = item.GetValue("id");
                if (!string.IsNullOrEmpty(key) && !string.IsNullOrEmpty(id)) aliases[key] = id;
            }
        }

        public override void OnSave(ConfigNode node)
        {
            base.OnSave(node);
            foreach (var pair in aliases)
            {
                var item = node.AddNode("IDENTITY");
                item.AddValue("key", pair.Key);
                item.AddValue("id", pair.Value);
            }
        }

        public void OnDestroy() { if (Instance == this) Instance = null; }

        private static string Key(string kind, Part part, int index)
        {
            return part.persistentId.ToString(CultureInfo.InvariantCulture) + ":" + kind + ":" + index.ToString(CultureInfo.InvariantCulture);
        }

        internal static string Resolve(string kind, Part part, int index, string fallback)
        {
            string value;
            return Instance != null && part != null && part.persistentId != 0 &&
                Instance.aliases.TryGetValue(Key(kind, part, index), out value) ? value : fallback;
        }

    }
}
