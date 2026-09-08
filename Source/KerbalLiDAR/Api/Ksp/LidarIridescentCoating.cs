using UnityEngine;

namespace KerbalLiDAR
{
    /// <summary>
    /// View-dependent optical coating for the Blender hemisphere's analytic UVs.
    /// Uses KSP/Specular, so no platform-specific Unity shader bundle is needed.
    /// Updates only when visible to a camera, without changing shared materials.
    /// </summary>
    public sealed class LidarIridescentCoating : MonoBehaviour
    {
        private const int Width = 64;
        private const int Height = 32;
        private const float Radius = 0.078f;
        private static readonly Vector3 Center = new Vector3(0, 0, 0.099f);
        private static readonly int MainTexture = Shader.PropertyToID("_MainTex");
        private static readonly Vector3[] Normals = CreateNormals();
        private static readonly Color32[] Palette = CreatePalette();
        private Renderer targetRenderer;
        private Texture2D coatingTexture;
        private MaterialPropertyBlock properties;
        private Color32[] pixels;
        private Vector3 previousView;
        private bool previousOrthographic;
        private bool hasView;

        private static Vector3[] CreateNormals()
        {
            var result = new Vector3[Width * Height];
            for (var y = 0; y < Height; y++)
            {
                var theta = (y + 0.5f) / Height * Mathf.PI * 0.5f;
                for (var x = 0; x < Width; x++)
                {
                    var phi = (x + 0.5f) / Width * Mathf.PI * 2f;
                    // MU export maps Blender (x,y,z) to Unity (-x,y,z).
                    result[y * Width + x] = new Vector3(-Mathf.Sin(theta) * Mathf.Cos(phi),
                        Mathf.Sin(theta) * Mathf.Sin(phi), Mathf.Cos(theta));
                }
            }
            return result;
        }

        private static Color32[] CreatePalette()
        {
            var stops = new[] { 0f, 0.26f, 0.5f, 0.72f, 0.9f, 1f };
            var colors = new[] {
                new Color(0.025f, 0.10f, 0.31f), new Color(0.015f, 0.34f, 0.42f),
                new Color(0.10f, 0.055f, 0.32f), new Color(0.39f, 0.075f, 0.24f),
                new Color(0.39f, 0.23f, 0.065f), new Color(0.10f, 0.35f, 0.32f)
            };
            var result = new Color32[256];
            var segment = 0;
            for (var i = 0; i < result.Length; i++)
            {
                var angle = i / 255f;
                while (segment < stops.Length - 2 && angle > stops[segment + 1]) segment++;
                var t = Mathf.InverseLerp(stops[segment], stops[segment + 1], angle);
                var color = Color.Lerp(colors[segment], colors[segment + 1], t).gamma;
                color.a = 0.92f; // KSP/Specular reads alpha as gloss, not opacity.
                result[i] = color;
            }
            return result;
        }

        public void OnWillRenderObject()
        {
            var camera = Camera.current;
            if (camera == null) return;
            if (targetRenderer == null) targetRenderer = GetComponent<Renderer>();
            if (targetRenderer == null) return;
            var orthographic = camera.orthographic;
            var view = orthographic
                ? transform.InverseTransformDirection(-camera.transform.forward).normalized
                : transform.InverseTransformPoint(camera.transform.position) - Center;
            // Stable views allocate and upload nothing. Each camera is evaluated
            // separately, including RGB sensor cameras and editor thumbnails.
            if (hasView && previousOrthographic == orthographic &&
                (view - previousView).sqrMagnitude < 0.00000001f) return;
            if (coatingTexture == null)
            {
                coatingTexture = new Texture2D(Width, Height, TextureFormat.RGBA32, false);
                coatingTexture.name = "LiDAR incidence coating";
                coatingTexture.wrapModeU = TextureWrapMode.Repeat;
                coatingTexture.wrapModeV = TextureWrapMode.Clamp;
                coatingTexture.filterMode = FilterMode.Bilinear;
                properties = new MaterialPropertyBlock();
                pixels = new Color32[Width * Height];
            }
            for (var i = 0; i < pixels.Length; i++)
            {
                var normal = Normals[i];
                var direction = orthographic ? view : (view - normal * Radius).normalized;
                var incidence = 1f - Mathf.Clamp01(Vector3.Dot(normal, direction));
                pixels[i] = Palette[Mathf.Clamp(Mathf.RoundToInt(incidence * 255f), 0, 255)];
            }
            coatingTexture.SetPixels32(pixels);
            coatingTexture.Apply(false, false);
            targetRenderer.GetPropertyBlock(properties);
            properties.SetTexture(MainTexture, coatingTexture);
            targetRenderer.SetPropertyBlock(properties);
            previousView = view;
            previousOrthographic = orthographic;
            hasView = true;
        }

        public void OnDisable()
        {
            if (targetRenderer != null && properties != null)
            {
                targetRenderer.GetPropertyBlock(properties);
                // Preserve KSP's highlighting/thermal properties.
                var material = targetRenderer.sharedMaterial;
                if (material != null) properties.SetTexture(MainTexture, material.mainTexture);
                targetRenderer.SetPropertyBlock(properties);
            }
            hasView = false;
        }

        public void OnDestroy()
        {
            OnDisable();
            if (coatingTexture != null) Destroy(coatingTexture);
        }
    }
}
