using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private GameObject visualModelObject;
        private Material visualModelMaterial;
        private string loadedVisualModelPath;
        private float nextVisualModelSyncTime;

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

        private void LoadConfiguredVisualModel()
        {
            EnsureNativeOpticalCoating();
            if (string.IsNullOrEmpty(visualModelObjPath))
            {
                return;
            }

            try
            {
                var objPath = ResolveKspRelativePath(visualModelObjPath.Trim());
                if (!File.Exists(objPath))
                {
                    Debug.LogWarning("[KerbalLiDAR] Visual OBJ was not found: " + objPath);
                    return;
                }

                DestroyVisualModel();

                var mesh = LoadObjMesh(objPath, visualModelDoubleSided);
                visualModelObject = new GameObject("KerbalLiDAR Visual Model");
                var parent = part != null && part.transform != null ? part.transform : transform;
                visualModelObject.transform.SetParent(parent, false);
                visualModelObject.transform.localPosition = ParseConfigVector3(visualModelPosition, Vector3.zero);
                visualModelObject.transform.localEulerAngles = ParseConfigVector3(visualModelRotation, Vector3.zero);
                visualModelObject.transform.localScale = Vector3.one * Mathf.Max(0.001f, visualModelScale);
                visualModelObject.layer = parent.gameObject.layer;

                var meshFilter = visualModelObject.AddComponent<MeshFilter>();
                meshFilter.sharedMesh = mesh;

                var meshRenderer = visualModelObject.AddComponent<MeshRenderer>();
                visualModelMaterial = CreateVisualModelMaterial();
                meshRenderer.sharedMaterial = visualModelMaterial;
                loadedVisualModelPath = objPath;
                HideStockModelRenderers();
                ShowVisualModelRenderers();
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[KerbalLiDAR] Failed to load visual OBJ '" + visualModelObjPath + "': " + ex.Message);
            }
        }

        private void EnsureConfiguredVisualModel()
        {
            if (Time.realtimeSinceStartup < nextVisualModelSyncTime)
            {
                return;
            }

            nextVisualModelSyncTime = Time.realtimeSinceStartup + 0.5f;
            EnsureNativeOpticalCoating();
            if (string.IsNullOrEmpty(visualModelObjPath)) return;
            var objPath = ResolveKspRelativePath(visualModelObjPath.Trim());
            if (visualModelObject == null || !string.Equals(loadedVisualModelPath, objPath, StringComparison.Ordinal))
            {
                LoadConfiguredVisualModel();
                return;
            }

            HideStockModelRenderers();
            ShowVisualModelRenderers();
        }

        private void HideStockModelRenderers()
        {
            var root = part != null && part.transform != null ? part.transform : transform;
            var renderers = root.GetComponentsInChildren<Renderer>(true);
            for (var i = 0; i < renderers.Length; i++)
            {
                if (renderers[i] is LineRenderer || IsVisualModelRenderer(renderers[i]))
                {
                    continue;
                }

                renderers[i].enabled = false;
            }
        }

        private void ShowVisualModelRenderers()
        {
            if (visualModelObject == null)
            {
                return;
            }

            var renderers = visualModelObject.GetComponentsInChildren<Renderer>(true);
            for (var i = 0; i < renderers.Length; i++)
            {
                if (renderers[i] != null)
                {
                    renderers[i].enabled = true;
                }
            }
        }

        private bool IsVisualModelRenderer(Renderer renderer)
        {
            if (renderer == null || visualModelObject == null)
            {
                return false;
            }

            var current = renderer.transform;
            while (current != null)
            {
                if (current == visualModelObject.transform)
                {
                    return true;
                }

                current = current.parent;
            }

            return false;
        }

        private void DestroyVisualModel()
        {
            if (visualModelObject != null)
            {
                Destroy(visualModelObject);
                visualModelObject = null;
            }

            if (visualModelMaterial != null)
            {
                Destroy(visualModelMaterial);
                visualModelMaterial = null;
            }

            loadedVisualModelPath = null;
        }

        private Material CreateVisualModelMaterial()
        {
            var shader = Shader.Find("KSP/Diffuse");
            if (shader == null)
            {
                shader = Shader.Find("Diffuse");
            }

            var material = new Material(shader);
            var texture = LoadVisualModelTexture();
            if (texture != null)
            {
                material.mainTexture = texture;
            }

            return material;
        }

        private Texture2D LoadVisualModelTexture()
        {
            if (string.IsNullOrEmpty(visualModelTexturePath) || GameDatabase.Instance == null)
            {
                return null;
            }

            var texturePath = NormalizeGameDatabaseTexturePath(visualModelTexturePath);
            return string.IsNullOrEmpty(texturePath) ? null : GameDatabase.Instance.GetTexture(texturePath, false);
        }

        private static string NormalizeGameDatabaseTexturePath(string path)
        {
            var normalized = (path ?? "").Trim().Replace('\\', '/');
            const string gameDataPrefix = "GameData/";
            if (normalized.StartsWith(gameDataPrefix, StringComparison.OrdinalIgnoreCase))
            {
                normalized = normalized.Substring(gameDataPrefix.Length);
            }

            var slashIndex = normalized.LastIndexOf('/');
            var dotIndex = normalized.LastIndexOf('.');
            if (dotIndex > slashIndex)
            {
                normalized = normalized.Substring(0, dotIndex);
            }

            return normalized;
        }

        private static Mesh LoadObjMesh(string path, bool doubleSided)
        {
            var sourcePositions = new List<Vector3>();
            var sourceUvs = new List<Vector2>();
            var sourceNormals = new List<Vector3>();
            var meshPositions = new List<Vector3>();
            var meshUvs = new List<Vector2>();
            var meshNormals = new List<Vector3>();
            var triangles = new List<int>();
            var vertexMap = new Dictionary<string, int>();
            var missingNormals = false;

            var lines = File.ReadAllLines(path);
            for (var lineIndex = 0; lineIndex < lines.Length; lineIndex++)
            {
                var line = lines[lineIndex].Trim();
                if (line.Length == 0 || line.StartsWith("#", StringComparison.Ordinal))
                {
                    continue;
                }

                var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0)
                {
                    continue;
                }

                switch (parts[0])
                {
                    case "v":
                        if (parts.Length >= 4)
                        {
                            sourcePositions.Add(new Vector3(ParseObjFloat(parts[1]), ParseObjFloat(parts[2]), ParseObjFloat(parts[3])));
                        }
                        break;
                    case "vt":
                        if (parts.Length >= 3)
                        {
                            sourceUvs.Add(new Vector2(ParseObjFloat(parts[1]), ParseObjFloat(parts[2])));
                        }
                        break;
                    case "vn":
                        if (parts.Length >= 4)
                        {
                            sourceNormals.Add(new Vector3(ParseObjFloat(parts[1]), ParseObjFloat(parts[2]), ParseObjFloat(parts[3])).normalized);
                        }
                        break;
                    case "f":
                        if (parts.Length >= 4)
                        {
                            var faceIndices = new int[parts.Length - 1];
                            for (var i = 1; i < parts.Length; i++)
                            {
                                faceIndices[i - 1] = ResolveObjVertex(parts[i], sourcePositions, sourceUvs, sourceNormals, meshPositions, meshUvs, meshNormals, vertexMap, ref missingNormals);
                            }

                            for (var i = 1; i < faceIndices.Length - 1; i++)
                            {
                                AddTriangle(triangles, faceIndices[0], faceIndices[i], faceIndices[i + 1], doubleSided);
                            }
                        }
                        break;
                }
            }

            if (meshPositions.Count == 0 || triangles.Count == 0)
            {
                throw new InvalidOperationException("OBJ did not contain any mesh geometry.");
            }

            var mesh = new Mesh();
            mesh.name = Path.GetFileNameWithoutExtension(path);
            mesh.vertices = meshPositions.ToArray();
            mesh.triangles = triangles.ToArray();
            if (meshUvs.Count == meshPositions.Count)
            {
                mesh.uv = meshUvs.ToArray();
            }
            if (!missingNormals && meshNormals.Count == meshPositions.Count)
            {
                mesh.normals = meshNormals.ToArray();
            }
            else
            {
                mesh.RecalculateNormals();
            }
            mesh.RecalculateBounds();
            return mesh;
        }

        private static int ResolveObjVertex(string token, List<Vector3> sourcePositions, List<Vector2> sourceUvs, List<Vector3> sourceNormals, List<Vector3> meshPositions, List<Vector2> meshUvs, List<Vector3> meshNormals, Dictionary<string, int> vertexMap, ref bool missingNormals)
        {
            int existingIndex;
            if (vertexMap.TryGetValue(token, out existingIndex))
            {
                return existingIndex;
            }

            var indices = token.Split('/');
            var positionIndex = ParseObjIndex(indices.Length > 0 ? indices[0] : "", sourcePositions.Count);
            var uvIndex = indices.Length > 1 && indices[1].Length > 0 ? ParseObjIndex(indices[1], sourceUvs.Count) : -1;
            var normalIndex = indices.Length > 2 && indices[2].Length > 0 ? ParseObjIndex(indices[2], sourceNormals.Count) : -1;

            var meshIndex = meshPositions.Count;
            meshPositions.Add(sourcePositions[positionIndex]);
            meshUvs.Add(uvIndex >= 0 ? sourceUvs[uvIndex] : Vector2.zero);
            if (normalIndex >= 0)
            {
                meshNormals.Add(sourceNormals[normalIndex]);
            }
            else
            {
                meshNormals.Add(Vector3.zero);
                missingNormals = true;
            }

            vertexMap[token] = meshIndex;
            return meshIndex;
        }

        private static void AddTriangle(List<int> triangles, int a, int b, int c, bool doubleSided)
        {
            triangles.Add(a);
            triangles.Add(b);
            triangles.Add(c);

            if (!doubleSided)
            {
                return;
            }

            triangles.Add(c);
            triangles.Add(b);
            triangles.Add(a);
        }

        private static int ParseObjIndex(string value, int count)
        {
            var index = int.Parse(value, NumberStyles.Integer, CultureInfo.InvariantCulture);
            var resolved = index > 0 ? index - 1 : count + index;
            if (resolved < 0 || resolved >= count)
            {
                throw new InvalidOperationException("OBJ index is out of range.");
            }

            return resolved;
        }

        private static float ParseObjFloat(string value)
        {
            return float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
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

        private string ResolveKspRelativePath(string path)
        {
            if (Path.IsPathRooted(path))
            {
                return path;
            }

            return Path.Combine(GetKspRootPath(), path);
        }

        private string GetKspRootPath()
        {
            try
            {
                if (!string.IsNullOrEmpty(KSPUtil.ApplicationRootPath))
                {
                    return KSPUtil.ApplicationRootPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
                }
            }
            catch
            {
            }

            return Directory.GetCurrentDirectory();
        }
    }
}
