using System;
using System.Globalization;
using UnityEngine;

namespace PyLoN
{
    public partial class ModulePyLoNLidar
    {
        [KSPField]
        public string iridescentTransformName = "";

        private void EnsureNativeOpticalCoating()
        {
            if (part == null || string.IsNullOrEmpty(iridescentTransformName)) return;
            var dome = part.FindModelTransform(iridescentTransformName);
            if (dome != null && dome.GetComponent<Renderer>() != null &&
                dome.GetComponent<LidarIridescentCoating>() == null)
            {
                dome.gameObject.AddComponent<LidarIridescentCoating>();
            }
        }

        private static Vector3 ParseConfigVector3(string value, Vector3 fallback)
        {
            if (string.IsNullOrEmpty(value))
            {
                return fallback;
            }

            var parts = value.Split(new[] { ',', ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 3)
            {
                return fallback;
            }

            float x;
            float y;
            float z;
            if (!float.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out x) ||
                !float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out y) ||
                !float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out z))
            {
                return fallback;
            }

            return new Vector3(x, y, z);
        }

    }
}
