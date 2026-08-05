using System;
using System.Collections.Generic;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private const float RadarPreviewDistanceMeters = 100f;
        private static readonly Color RadarLineColor = new Color(1f, 0f, 0f, 0.95f);

        private readonly List<LineRenderer> radarLineRenderers = new List<LineRenderer>();
        private Material radarLineMaterial;

        private void UpdateRadarPreviewLines()
        {
            NormalizeConfig();

            var originTransform = ResolveOriginTransform();
            var forward = AxisToWorld(originTransform, forwardAxis);
            var up = AxisToWorld(originTransform, upAxis);
            Orthonormalize(ref forward, ref up, originTransform);

            var origin = originTransform.position + forward * Mathf.Max(0f, originOffsetMeters);
            var horizontalCount = 1;
            var verticalCount = 1;
            var rayCount = ResolveRayCounts(ref horizontalCount, ref verticalCount);
            rayBudget = FormatRayBudget(horizontalCount, verticalCount, rayCount);

            var right = Vector3.Cross(up, forward).normalized;
            if (right.sqrMagnitude < 0.0001f)
            {
                right = Vector3.right;
            }

            var previewDistance = Mathf.Min(maxDistance, RadarPreviewDistanceMeters);
            var rayIndex = 0;
            for (var vertical = 0; vertical < verticalCount; vertical++)
            {
                for (var horizontal = 0; horizontal < horizontalCount; horizontal++)
                {
                    var direction = Is3DMode()
                        ? CalculateHemisphereRayDirection(forward, up, right, horizontal, rayCount)
                        : CalculateRayDirection(forward, up, right, horizontal, horizontalCount, vertical, verticalCount);
                    UpdateRadarLine(rayIndex, origin, origin + direction * previewDistance);
                    rayIndex++;
                }
            }

            HideUnusedRadarLines(rayIndex);
        }

        private void UpdateRadarLine(int index, Vector3 start, Vector3 end)
        {
            var line = EnsureRadarLineRenderer(index);
            var width = Mathf.Max(0.001f, radarLineWidth);
            var lineTransform = line.transform;

            if (!line.gameObject.activeSelf)
            {
                line.gameObject.SetActive(true);
            }

            line.enabled = true;
            line.useWorldSpace = false;
            line.material = GetRadarLineMaterial();
            line.startWidth = width;
            line.endWidth = width;
            line.startColor = RadarLineColor;
            line.endColor = RadarLineColor;
            line.SetPosition(0, lineTransform.InverseTransformPoint(start));
            line.SetPosition(1, lineTransform.InverseTransformPoint(end));
        }

        private LineRenderer EnsureRadarLineRenderer(int index)
        {
            while (radarLineRenderers.Count <= index)
            {
                var lineObject = new GameObject("KerbalLiDAR Radar Line");
                if (part != null && part.transform != null)
                {
                    lineObject.transform.SetParent(part.transform, false);
                    lineObject.layer = part.gameObject.layer;
                }

                var line = lineObject.AddComponent<LineRenderer>();
                line.useWorldSpace = false;
                line.positionCount = 2;
                line.material = GetRadarLineMaterial();
                line.startColor = RadarLineColor;
                line.endColor = RadarLineColor;
                line.startWidth = Mathf.Max(0.001f, radarLineWidth);
                line.endWidth = Mathf.Max(0.001f, radarLineWidth);
                line.gameObject.SetActive(false);

                radarLineRenderers.Add(line);
            }

            return radarLineRenderers[index];
        }

        private Material GetRadarLineMaterial()
        {
            if (radarLineMaterial != null)
            {
                return radarLineMaterial;
            }

            var shader = FindFirstShader(
                "KSP/Particles/Additive",
                "KSP/Alpha/Unlit Transparent",
                "KSP/Alpha/Translucent",
                "Particles/Additive",
                "Unlit/Color",
                "KSP/Diffuse",
                "Diffuse");

            radarLineMaterial = new Material(shader);
            radarLineMaterial.color = RadarLineColor;
            radarLineMaterial.renderQueue = 3000;
            if (radarLineMaterial.HasProperty("_Color"))
            {
                radarLineMaterial.SetColor("_Color", RadarLineColor);
            }
            if (radarLineMaterial.HasProperty("_TintColor"))
            {
                radarLineMaterial.SetColor("_TintColor", RadarLineColor);
            }

            return radarLineMaterial;
        }

        private static Shader FindFirstShader(params string[] shaderNames)
        {
            for (var i = 0; i < shaderNames.Length; i++)
            {
                var shader = Shader.Find(shaderNames[i]);
                if (shader != null)
                {
                    return shader;
                }
            }

            throw new InvalidOperationException("No compatible Unity shader was found for KerbalLiDAR radar lines.");
        }

        private void HideUnusedRadarLines(int activeCount)
        {
            for (var i = activeCount; i < radarLineRenderers.Count; i++)
            {
                var line = radarLineRenderers[i];
                if (line != null && line.gameObject.activeSelf)
                {
                    line.gameObject.SetActive(false);
                }
            }
        }

        private void ClearRadarLines()
        {
            HideUnusedRadarLines(0);
        }

        private void DestroyRadarLines()
        {
            for (var i = 0; i < radarLineRenderers.Count; i++)
            {
                var line = radarLineRenderers[i];
                if (line != null)
                {
                    Destroy(line.gameObject);
                }
            }

            radarLineRenderers.Clear();
            if (radarLineMaterial != null)
            {
                Destroy(radarLineMaterial);
                radarLineMaterial = null;
            }
        }
    }
}
