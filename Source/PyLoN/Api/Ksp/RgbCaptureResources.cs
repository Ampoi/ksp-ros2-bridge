using System;
using UnityEngine;
namespace PyLoN
{
    internal sealed class RgbCaptureResources : IDisposable
    {
        internal RenderTexture renderTexture, outputTexture;
        internal Texture2D readbackTexture;
        internal int renderWidth, renderHeight, width, height;
        public void Ensure(int imageWidth, int imageHeight)
        {
            // Scatterer.DepthToDistanceCommandBuffer shares a depth texture
            // across the flight stack. Its OnPostRender reallocates it whenever
            // activeTexture changes size, leaving stale command buffers behind.
            // Keep the intermediate target screen-sized and stable even when
            // only the requested sensor output resolution changes.
            var gameWidth = Mathf.Max(1, Screen.width);
            var gameHeight = Mathf.Max(1, Screen.height);
            if (renderTexture == null || renderWidth != gameWidth || renderHeight != gameHeight)
            {
                if (renderTexture != null)
                {
                    renderTexture.Release();
                    UnityEngine.Object.Destroy(renderTexture);
                }
                renderWidth = gameWidth;
                renderHeight = gameHeight;
                renderTexture = new RenderTexture(renderWidth, renderHeight, 24, RenderTextureFormat.ARGB32);
                renderTexture.name = "PyLoN composite RGB " + renderWidth + "x" + renderHeight;
                renderTexture.filterMode = FilterMode.Bilinear;
                renderTexture.Create();
            }

            if (outputTexture != null && readbackTexture != null
                && width == imageWidth && height == imageHeight)
            {
                return;
            }
            if (outputTexture != null)
            {
                outputTexture.Release();
                UnityEngine.Object.Destroy(outputTexture);
            }
            if (readbackTexture != null) UnityEngine.Object.Destroy(readbackTexture);
            width = imageWidth;
            height = imageHeight;
            outputTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
            outputTexture.name = "PyLoN RGB output " + width + "x" + height;
            outputTexture.Create();
            readbackTexture = new Texture2D(width, height, TextureFormat.RGB24, false);
        }
        public byte[] ReadTopDownRgb()
        {
            var pixels = readbackTexture.GetPixels32();
            var rgb = new byte[width * height * 3];
            var targetIndex = 0;
            for (var targetY = 0; targetY < height; targetY++)
            {
                var sourceRow = (height - 1 - targetY) * width;
                for (var x = 0; x < width; x++)
                {
                    var pixel = pixels[sourceRow + x];
                    rgb[targetIndex++] = pixel.r;
                    rgb[targetIndex++] = pixel.g;
                    rgb[targetIndex++] = pixel.b;
                }
            }
            return rgb;
        }
        public void Dispose()
        {
            if (renderTexture != null)
            {
                renderTexture.Release();
                UnityEngine.Object.Destroy(renderTexture);
                renderTexture = null;
            }
            if (outputTexture != null)
            {
                outputTexture.Release();
                UnityEngine.Object.Destroy(outputTexture);
                outputTexture = null;
            }
            if (readbackTexture != null)
            {
                UnityEngine.Object.Destroy(readbackTexture);
                readbackTexture = null;
            }
            width = height = renderWidth = renderHeight = 0;
        }
    }
}
