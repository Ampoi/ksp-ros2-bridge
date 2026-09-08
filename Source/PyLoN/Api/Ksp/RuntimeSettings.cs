using System;
using System.Globalization;
namespace PyLoN
{
    /// <summary>One transport destination for every sensor and vessel service.</summary>
    internal static class RuntimeSettings
    {
        private static ConfigNode Node
        {
            get
            {
                var nodes = GameDatabase.Instance == null ? null : GameDatabase.Instance.GetConfigNodes("PYLON_TRANSPORT");
                if (nodes != null && nodes.Length > 1) throw new InvalidOperationException("Only one PYLON_TRANSPORT configuration is allowed.");
                return nodes != null && nodes.Length == 1 ? nodes[0] : null;
            }
        }
        public static string StateHost { get { return Node == null ? "127.0.0.1" : Node.GetValue("stateHost") ?? "127.0.0.1"; } }
        public static int StatePort { get { return Port("statePort", 49010); } }
        public static int CommandPort { get { return Port("commandPort", 49011); } }
        private static int Port(string key, int fallback)
        {
            var node = Node;
            if (node == null || !node.HasValue(key)) return fallback;
            int value;
            if (!int.TryParse(node.GetValue(key), NumberStyles.Integer, CultureInfo.InvariantCulture, out value) || value < 1 || value > 65535)
                throw new InvalidOperationException("Invalid PYLON_TRANSPORT " + key);
            return value;
        }
    }
}
