using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Net;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private const string VesselProxyType = "ksp_active_vessel_proxy";
        private const string VesselProxyChunkType = "ksp_vessel_urdf_chunk";
        private const string VesselProxyClearType = "ksp_vessel_urdf_clear";
        private const int VesselProxyVersion = 1;

        private string vesselUrdfSessionId = Guid.NewGuid().ToString("N");
        private float nextActiveVesselUrdfTime;
        private bool activeVesselUrdfAnnounced;
        private bool remoteUrdfWarningShown;

        private struct ProxyBounds
        {
            public Vector3 Center;
            public Vector3 Size;
        }

        private enum ProxyGeometryType
        {
            Box,
            Cylinder,
            Sphere
        }

        private struct ProxyGeometry
        {
            public ProxyGeometryType Type;
            public Vector3 Center;
            public Quaternion Rotation;
            public Vector3 Size;
            public float Radius;
            public float Length;
        }

        private void ProcessActiveVesselUrdf()
        {
            if (!ShouldPublishActiveVesselUrdf())
            {
                ClearActiveVesselUrdf();
                return;
            }

            var now = Time.realtimeSinceStartup;
            if (now < nextActiveVesselUrdfTime)
            {
                return;
            }

            nextActiveVesselUrdfTime = now + activeVesselUrdfRefreshSeconds;
            if (!IsUrdfDestinationAllowed())
            {
                if (!remoteUrdfWarningShown)
                {
                    remoteUrdfWarningShown = true;
                    WarnUdpThrottled(
                        "Active vessel URDF is restricted to loopback. Set allowRemoteUrdf = true only on a trusted network."
                    );
                }
                ClearActiveVesselUrdf();
                return;
            }

            remoteUrdfWarningShown = false;
            try
            {
                SendActiveVesselUrdf();
            }
            catch (Exception ex)
            {
                WarnUdpThrottled("Active vessel URDF generation failed: " + ex.Message);
            }
        }

        private bool ShouldPublishActiveVesselUrdf()
        {
            if (!activeVesselUrdfEnabled || !udpEnabled || vessel == null || vessel.parts == null)
            {
                return false;
            }

            if (FlightGlobals.ActiveVessel != vessel)
            {
                return false;
            }

            foreach (var vesselPart in vessel.parts)
            {
                if (vesselPart == null || vesselPart.Modules == null)
                {
                    continue;
                }

                foreach (PartModule module in vesselPart.Modules)
                {
                    var candidate = module as ModuleKerbalLidar;
                    if (candidate == null || !candidate.activeVesselUrdfEnabled || !candidate.udpEnabled)
                    {
                        continue;
                    }

                    return ReferenceEquals(candidate, this);
                }
            }

            return false;
        }

        private bool IsUrdfDestinationAllowed()
        {
            if (allowRemoteUrdf)
            {
                return true;
            }

            var host = string.IsNullOrEmpty(udpHost) ? "127.0.0.1" : udpHost;
            return IPAddress.IsLoopback(ResolveAddress(host));
        }

        private void SendActiveVesselUrdf()
        {
            string modelId;
            var bundle = BuildActiveVesselProxyBundle(out modelId);
            if (string.IsNullOrEmpty(bundle))
            {
                return;
            }

            var compressed = CompressUtf8(bundle);
            var compressedSha256 = Sha256Hex(compressed);
            var packetOverheadBytes = 1024;
            var safeRawChunkBytes = Math.Max(64, (maxDatagramBytes - packetOverheadBytes) * 3 / 4);
            var chunkBytes = Math.Max(64, Math.Min(activeVesselUrdfChunkBytes, safeRawChunkBytes));
            var chunkCount = (compressed.Length + chunkBytes - 1) / chunkBytes;
            if (chunkCount <= 0 || chunkCount > maxActiveVesselUrdfChunks)
            {
                WarnUdpThrottled(
                    "Active vessel URDF requires " + chunkCount.ToString(CultureInfo.InvariantCulture)
                    + " chunks; limit is " + maxActiveVesselUrdfChunks.ToString(CultureInfo.InvariantCulture) + "."
                );
                return;
            }

            for (var chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++)
            {
                var offset = chunkIndex * chunkBytes;
                var count = Math.Min(chunkBytes, compressed.Length - offset);
                var payload = Convert.ToBase64String(compressed, offset, count);
                var packet = new StringBuilder(payload.Length + 512);
                packet.Append('{');
                AppendProperty(packet, "type", VesselProxyChunkType, true);
                AppendProperty(packet, "version", VesselProxyVersion, false);
                AppendProperty(packet, "sessionId", vesselUrdfSessionId, false);
                AppendProperty(packet, "modelId", modelId, false);
                AppendProperty(packet, "chunkIndex", chunkIndex, false);
                AppendProperty(packet, "chunkCount", chunkCount, false);
                AppendProperty(packet, "encoding", "gzip+base64", false);
                AppendProperty(packet, "sha256", compressedSha256, false);
                AppendProperty(packet, "expiresAfterSec", Mathf.Max(3f, activeVesselUrdfRefreshSeconds * 3f), false);
                AppendProperty(packet, "sentAtUt", Planetarium.GetUniversalTime(), false);
                AppendProperty(packet, "data", payload, false);
                packet.Append('}');
                SendUdp(packet.ToString());
            }

            activeVesselUrdfAnnounced = true;
        }

        private string BuildActiveVesselProxyBundle(out string modelId)
        {
            modelId = "";
            var orderedParts = OrderedVesselParts();
            if (orderedParts.Count == 0)
            {
                return "";
            }

            var linkNames = new Dictionary<Part, string>();
            var namePrefix = "ksp_" + vesselUrdfSessionId.Substring(0, 8);
            for (var index = 0; index < orderedParts.Count; index++)
            {
                linkNames[orderedParts[index]] = namePrefix + "_link_" + index.ToString("D4", CultureInfo.InvariantCulture);
            }

            var urdf = BuildProxyUrdf(orderedParts, linkNames, namePrefix);
            modelId = Sha256Hex(Encoding.UTF8.GetBytes(urdf));

            var bundle = new StringBuilder(urdf.Length + orderedParts.Count * 96 + 512);
            bundle.Append('{');
            AppendProperty(bundle, "type", VesselProxyType, true);
            AppendProperty(bundle, "version", VesselProxyVersion, false);
            AppendProperty(bundle, "sessionId", vesselUrdfSessionId, false);
            AppendProperty(bundle, "modelId", modelId, false);
            AppendProperty(bundle, "geometryPolicy", "primitive_proxy_only", false);
            AppendProperty(bundle, "persistencePolicy", "memory_only", false);
            AppendProperty(bundle, "rootFrame", linkNames[orderedParts[0]], false);
            AppendProperty(bundle, "urdf", urdf, false);
            AppendPropertyPrefix(bundle, "partFrames", false);
            bundle.Append('[');
            for (var index = 0; index < orderedParts.Count; index++)
            {
                if (index > 0)
                {
                    bundle.Append(',');
                }

                bundle.Append('{');
                AppendProperty(bundle, "partFlightId", (long)orderedParts[index].flightID, true);
                AppendProperty(bundle, "frame", linkNames[orderedParts[index]], false);
                bundle.Append('}');
            }
            bundle.Append(']');
            bundle.Append('}');
            return bundle.ToString();
        }

        private List<Part> OrderedVesselParts()
        {
            var ordered = new List<Part>();
            if (vessel == null || vessel.parts == null)
            {
                return ordered;
            }

            var root = vessel.rootPart;
            if (root != null)
            {
                ordered.Add(root);
            }

            foreach (var vesselPart in vessel.parts)
            {
                if (vesselPart != null && vesselPart != root)
                {
                    ordered.Add(vesselPart);
                }
            }

            return ordered;
        }

        private static string BuildProxyUrdf(
            IList<Part> orderedParts,
            IDictionary<Part, string> linkNames,
            string namePrefix
        )
        {
            var urdf = new StringBuilder(orderedParts.Count * 768 + 256);
            urdf.Append("<?xml version=\"1.0\"?>\n");
            urdf.Append("<!-- Runtime-only primitive proxy. Contains no KSP meshes, textures, part names, or asset paths. -->\n");
            urdf.Append("<robot name=\"").Append(namePrefix).Append("_active_vessel\">\n");

            foreach (var vesselPart in orderedParts)
            {
                AppendProxyLink(urdf, vesselPart, linkNames[vesselPart]);
            }

            var root = orderedParts[0];
            for (var index = 1; index < orderedParts.Count; index++)
            {
                var child = orderedParts[index];
                var parent = child.parent;
                if (parent == null || !linkNames.ContainsKey(parent))
                {
                    parent = root;
                }

                AppendFixedJoint(
                    urdf,
                    namePrefix + "_joint_" + index.ToString("D4", CultureInfo.InvariantCulture),
                    linkNames[parent],
                    linkNames[child],
                    parent.transform.InverseTransformPoint(child.transform.position),
                    Quaternion.Inverse(parent.transform.rotation) * child.transform.rotation
                );
            }

            urdf.Append("</robot>\n");
            return urdf.ToString();
        }

        private static void AppendProxyLink(StringBuilder urdf, Part vesselPart, string linkName)
        {
            var geometries = BuildProxyGeometries(vesselPart);
            var bounds = EstimateProxyBounds(geometries);
            var rosCenter = UnityVectorToRos(bounds.Center);
            var rosSize = UnitySizeToRos(bounds.Size);
            var massKg = Math.Max(0.001f, vesselPart.mass * 1000f);
            var ixx = massKg * (rosSize.y * rosSize.y + rosSize.z * rosSize.z) / 12f;
            var iyy = massKg * (rosSize.x * rosSize.x + rosSize.z * rosSize.z) / 12f;
            var izz = massKg * (rosSize.x * rosSize.x + rosSize.y * rosSize.y) / 12f;

            urdf.Append("  <link name=\"").Append(linkName).Append("\">\n");
            urdf.Append("    <inertial>\n      <origin xyz=\"");
            AppendVector(urdf, rosCenter);
            urdf.Append("\" rpy=\"0 0 0\"/>\n      <mass value=\"");
            AppendUrdfFloat(urdf, massKg);
            urdf.Append("\"/>\n      <inertia ixx=\"");
            AppendUrdfFloat(urdf, ixx);
            urdf.Append("\" ixy=\"0\" ixz=\"0\" iyy=\"");
            AppendUrdfFloat(urdf, iyy);
            urdf.Append("\" iyz=\"0\" izz=\"");
            AppendUrdfFloat(urdf, izz);
            urdf.Append("\"/>\n    </inertial>\n");
            AppendProxyGeometries(urdf, "visual", geometries);
            AppendProxyGeometries(urdf, "collision", geometries);
            urdf.Append("  </link>\n");
        }

        private static void AppendProxyGeometries(
            StringBuilder urdf,
            string element,
            IList<ProxyGeometry> geometries
        )
        {
            foreach (var geometry in geometries)
            {
                var rosCenter = UnityVectorToRos(geometry.Center);
                var rosRpy = RosQuaternionToRpy(UnityRotationToRos(geometry.Rotation));
                urdf.Append("    <").Append(element).Append(">\n      <origin xyz=\"");
                AppendVector(urdf, rosCenter);
                urdf.Append("\" rpy=\"");
                AppendVector(urdf, rosRpy);
                urdf.Append("\"/>\n      <geometry>");
                switch (geometry.Type)
                {
                    case ProxyGeometryType.Cylinder:
                        urdf.Append("<cylinder radius=\"");
                        AppendUrdfFloat(urdf, geometry.Radius);
                        urdf.Append("\" length=\"");
                        AppendUrdfFloat(urdf, geometry.Length);
                        urdf.Append("\"/>");
                        break;
                    case ProxyGeometryType.Sphere:
                        urdf.Append("<sphere radius=\"");
                        AppendUrdfFloat(urdf, geometry.Radius);
                        urdf.Append("\"/>");
                        break;
                    default:
                        urdf.Append("<box size=\"");
                        AppendVector(urdf, UnitySizeToRos(geometry.Size));
                        urdf.Append("\"/>");
                        break;
                }
                urdf.Append("</geometry>\n    </").Append(element).Append(">\n");
            }
        }

        private static void AppendFixedJoint(
            StringBuilder urdf,
            string jointName,
            string parentLink,
            string childLink,
            Vector3 unityPosition,
            Quaternion unityRotation
        )
        {
            var rosPosition = UnityVectorToRos(unityPosition);
            var rosRpy = RosQuaternionToRpy(UnityRotationToRos(unityRotation));
            urdf.Append("  <joint name=\"").Append(jointName).Append("\" type=\"fixed\">\n");
            urdf.Append("    <parent link=\"").Append(parentLink).Append("\"/>\n");
            urdf.Append("    <child link=\"").Append(childLink).Append("\"/>\n");
            urdf.Append("    <origin xyz=\"");
            AppendVector(urdf, rosPosition);
            urdf.Append("\" rpy=\"");
            AppendVector(urdf, rosRpy);
            urdf.Append("\"/>\n  </joint>\n");
        }

        private static List<ProxyGeometry> BuildProxyGeometries(Part vesselPart)
        {
            const int maximumGeometriesPerPart = 48;
            var geometries = new List<ProxyGeometry>();
            var colliders = vesselPart.GetComponentsInChildren<Collider>();
            foreach (var collider in colliders)
            {
                if (
                    collider == null
                    || !collider.enabled
                    || FindPart(collider) != vesselPart
                    || geometries.Count >= maximumGeometriesPerPart
                )
                {
                    continue;
                }

                var box = collider as BoxCollider;
                if (box != null)
                {
                    geometries.Add(CreateBoxGeometry(vesselPart.transform, box.transform, box.center, box.size));
                    continue;
                }

                var sphere = collider as SphereCollider;
                if (sphere != null)
                {
                    geometries.Add(CreateSphereGeometry(vesselPart.transform, sphere.transform, sphere.center, sphere.radius));
                    continue;
                }

                var capsule = collider as CapsuleCollider;
                if (capsule != null)
                {
                    AppendCapsuleGeometries(geometries, vesselPart.transform, capsule, maximumGeometriesPerPart);
                    continue;
                }

                var mesh = collider as MeshCollider;
                if (mesh != null && mesh.sharedMesh != null)
                {
                    AppendMeshProxyGeometry(geometries, vesselPart.transform, mesh, maximumGeometriesPerPart);
                }
            }

            if (geometries.Count == 0)
            {
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Box,
                    Center = Vector3.zero,
                    Rotation = Quaternion.identity,
                    Size = Vector3.one * 0.25f
                });
            }

            return geometries;
        }

        private static ProxyGeometry CreateBoxGeometry(
            Transform partTransform,
            Transform geometryTransform,
            Vector3 localCenter,
            Vector3 localSize
        )
        {
            return new ProxyGeometry
            {
                Type = ProxyGeometryType.Box,
                Center = partTransform.InverseTransformPoint(geometryTransform.TransformPoint(localCenter)),
                Rotation = RelativeRotation(partTransform, geometryTransform),
                Size = MaxVector(
                    Vector3.Scale(AbsVector(localSize), RelativeScale(partTransform, geometryTransform)),
                    Vector3.one * 0.01f
                )
            };
        }

        private static ProxyGeometry CreateSphereGeometry(
            Transform partTransform,
            Transform geometryTransform,
            Vector3 localCenter,
            float localRadius
        )
        {
            var scale = RelativeScale(partTransform, geometryTransform);
            return new ProxyGeometry
            {
                Type = ProxyGeometryType.Sphere,
                Center = partTransform.InverseTransformPoint(geometryTransform.TransformPoint(localCenter)),
                Rotation = Quaternion.identity,
                Radius = Mathf.Max(0.01f, Mathf.Abs(localRadius) * Mathf.Max(scale.x, Mathf.Max(scale.y, scale.z)))
            };
        }

        private static void AppendCapsuleGeometries(
            IList<ProxyGeometry> geometries,
            Transform partTransform,
            CapsuleCollider capsule,
            int maximumGeometries
        )
        {
            var scale = RelativeScale(partTransform, capsule.transform);
            var axis = capsule.direction == 0 ? Vector3.right : capsule.direction == 1 ? Vector3.up : Vector3.forward;
            var axisScale = capsule.direction == 0 ? scale.x : capsule.direction == 1 ? scale.y : scale.z;
            var radialScale = capsule.direction == 0
                ? Mathf.Max(scale.y, scale.z)
                : capsule.direction == 1 ? Mathf.Max(scale.x, scale.z) : Mathf.Max(scale.x, scale.y);
            var radius = Mathf.Max(0.01f, Mathf.Abs(capsule.radius) * radialScale);
            var totalLength = Mathf.Max(radius * 2f, Mathf.Abs(capsule.height) * axisScale);
            var cylinderLength = totalLength - radius * 2f;
            var center = partTransform.InverseTransformPoint(capsule.transform.TransformPoint(capsule.center));
            var colliderRotation = RelativeRotation(partTransform, capsule.transform);
            var axisInPart = (colliderRotation * axis).normalized;

            if (cylinderLength > 0.01f && geometries.Count < maximumGeometries)
            {
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Cylinder,
                    Center = center,
                    Rotation = colliderRotation * Quaternion.FromToRotation(Vector3.forward, axis),
                    Radius = radius,
                    Length = cylinderLength
                });
            }

            var capOffset = axisInPart * (cylinderLength * 0.5f);
            if (geometries.Count < maximumGeometries)
            {
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Sphere,
                    Center = center + capOffset,
                    Rotation = Quaternion.identity,
                    Radius = radius
                });
            }
            if (cylinderLength > 0.01f && geometries.Count < maximumGeometries)
            {
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Sphere,
                    Center = center - capOffset,
                    Rotation = Quaternion.identity,
                    Radius = radius
                });
            }
        }

        private static void AppendMeshProxyGeometry(
            IList<ProxyGeometry> geometries,
            Transform partTransform,
            MeshCollider mesh,
            int maximumGeometries
        )
        {
            if (geometries.Count >= maximumGeometries)
            {
                return;
            }

            var bounds = mesh.sharedMesh.bounds;
            var size = Vector3.Scale(AbsVector(bounds.size), RelativeScale(partTransform, mesh.transform));
            var center = partTransform.InverseTransformPoint(mesh.transform.TransformPoint(bounds.center));
            var rotation = RelativeRotation(partTransform, mesh.transform);
            int cylinderAxis;
            var geometryType = SelectMeshProxyGeometry(mesh.sharedMesh, size, out cylinderAxis);
            if (geometryType == ProxyGeometryType.Sphere)
            {
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Sphere,
                    Center = center,
                    Rotation = Quaternion.identity,
                    Radius = Mathf.Max(0.01f, Mathf.Max(size.x, Mathf.Max(size.y, size.z)) * 0.5f)
                });
                return;
            }

            if (geometryType == ProxyGeometryType.Cylinder)
            {
                var diameter = cylinderAxis == 0
                    ? (size.y + size.z) * 0.5f
                    : cylinderAxis == 1 ? (size.x + size.z) * 0.5f : (size.x + size.y) * 0.5f;
                var length = cylinderAxis == 0 ? size.x : cylinderAxis == 1 ? size.y : size.z;
                var axis = cylinderAxis == 0 ? Vector3.right : cylinderAxis == 1 ? Vector3.up : Vector3.forward;
                geometries.Add(new ProxyGeometry
                {
                    Type = ProxyGeometryType.Cylinder,
                    Center = center,
                    Rotation = rotation * Quaternion.FromToRotation(Vector3.forward, axis),
                    Radius = Mathf.Max(0.01f, diameter * 0.5f),
                    Length = Mathf.Max(0.01f, length)
                });
                return;
            }

            geometries.Add(new ProxyGeometry
            {
                Type = ProxyGeometryType.Box,
                Center = center,
                Rotation = rotation,
                Size = MaxVector(size, Vector3.one * 0.01f)
            });
        }

        private static ProxyGeometryType SelectMeshProxyGeometry(
            Mesh mesh,
            Vector3 scaledSize,
            out int cylinderAxis
        )
        {
            cylinderAxis = -1;
            float boxError;
            float sphereError;
            Vector3 cylinderErrors;
            if (!TryMeasureMeshSurface(mesh, out boxError, out sphereError, out cylinderErrors))
            {
                cylinderAxis = SimilarRadialAxes(scaledSize);
                return cylinderAxis >= 0 ? ProxyGeometryType.Cylinder : ProxyGeometryType.Box;
            }

            var bestType = ProxyGeometryType.Box;
            var bestError = boxError;
            var largestSize = Mathf.Max(scaledSize.x, Mathf.Max(scaledSize.y, scaledSize.z));
            var smallestSize = Mathf.Min(scaledSize.x, Mathf.Min(scaledSize.y, scaledSize.z));
            var sphereAspectPenalty = largestSize <= 0.001f
                ? 1f
                : (largestSize - smallestSize) / largestSize * 0.5f;
            var adjustedSphereError = sphereError + sphereAspectPenalty;
            if (adjustedSphereError + 0.02f < bestError)
            {
                bestType = ProxyGeometryType.Sphere;
                bestError = adjustedSphereError;
            }

            for (var axis = 0; axis < 3; axis++)
            {
                var firstRadialSize = axis == 0 ? scaledSize.y : scaledSize.x;
                var secondRadialSize = axis == 2 ? scaledSize.y : scaledSize.z;
                var radialMaximum = Mathf.Max(0.001f, Mathf.Max(firstRadialSize, secondRadialSize));
                var radialPenalty = Mathf.Abs(firstRadialSize - secondRadialSize) / radialMaximum * 0.5f;
                var surfaceError = axis == 0
                    ? cylinderErrors.x
                    : axis == 1 ? cylinderErrors.y : cylinderErrors.z;
                var adjustedCylinderError = surfaceError + radialPenalty;
                if (adjustedCylinderError + 0.02f < bestError)
                {
                    bestType = ProxyGeometryType.Cylinder;
                    bestError = adjustedCylinderError;
                    cylinderAxis = axis;
                }
            }

            return bestType;
        }

        private static bool TryMeasureMeshSurface(
            Mesh mesh,
            out float boxError,
            out float sphereError,
            out Vector3 cylinderErrors
        )
        {
            boxError = 0f;
            sphereError = 0f;
            cylinderErrors = Vector3.zero;
            try
            {
                var vertices = mesh.vertices;
                var triangles = mesh.triangles;
                var bounds = mesh.bounds;
                var extents = bounds.extents;
                if (
                    vertices == null
                    || triangles == null
                    || triangles.Length < 3
                    || extents.x <= 0.0001f
                    || extents.y <= 0.0001f
                    || extents.z <= 0.0001f
                )
                {
                    return false;
                }

                const int maximumSamples = 384;
                var triangleCount = triangles.Length / 3;
                var sampleStep = Math.Max(1, (triangleCount + maximumSamples - 1) / maximumSamples);
                var sampleCount = 0;
                for (var triangle = 0; triangle < triangleCount; triangle += sampleStep)
                {
                    var triangleOffset = triangle * 3;
                    var firstIndex = triangles[triangleOffset];
                    var secondIndex = triangles[triangleOffset + 1];
                    var thirdIndex = triangles[triangleOffset + 2];
                    if (
                        firstIndex < 0 || firstIndex >= vertices.Length
                        || secondIndex < 0 || secondIndex >= vertices.Length
                        || thirdIndex < 0 || thirdIndex >= vertices.Length
                    )
                    {
                        continue;
                    }

                    var point = (vertices[firstIndex] + vertices[secondIndex] + vertices[thirdIndex]) / 3f;
                    var normalized = point - bounds.center;
                    var x = Mathf.Abs(normalized.x / extents.x);
                    var y = Mathf.Abs(normalized.y / extents.y);
                    var z = Mathf.Abs(normalized.z / extents.z);
                    boxError += Mathf.Min(Mathf.Abs(1f - x), Mathf.Min(Mathf.Abs(1f - y), Mathf.Abs(1f - z)));
                    sphereError += Mathf.Abs(Mathf.Sqrt(x * x + y * y + z * z) - 1f);
                    cylinderErrors.x += CylinderSurfaceError(x, y, z);
                    cylinderErrors.y += CylinderSurfaceError(y, x, z);
                    cylinderErrors.z += CylinderSurfaceError(z, x, y);
                    sampleCount++;
                }

                if (sampleCount < 4)
                {
                    return false;
                }

                boxError /= sampleCount;
                sphereError /= sampleCount;
                cylinderErrors /= sampleCount;
                return true;
            }
            catch (UnityException)
            {
                // Some imported collider meshes are not CPU-readable. Bounds-based selection remains available.
                return false;
            }
        }

        private static float CylinderSurfaceError(float axial, float firstRadial, float secondRadial)
        {
            var barrelError = Mathf.Abs(Mathf.Sqrt(firstRadial * firstRadial + secondRadial * secondRadial) - 1f);
            var capError = Mathf.Abs(axial - 1f);
            return Mathf.Min(barrelError, capError);
        }

        private static int SimilarRadialAxes(Vector3 size)
        {
            const float radialTolerance = 0.12f;
            const float axialDifference = 0.18f;
            if (Similar(size.y, size.z, radialTolerance) && !Similar(size.x, (size.y + size.z) * 0.5f, axialDifference))
            {
                return 0;
            }
            if (Similar(size.x, size.z, radialTolerance) && !Similar(size.y, (size.x + size.z) * 0.5f, axialDifference))
            {
                return 1;
            }
            if (Similar(size.x, size.y, radialTolerance) && !Similar(size.z, (size.x + size.y) * 0.5f, axialDifference))
            {
                return 2;
            }
            return -1;
        }

        private static bool Similar(float left, float right, float tolerance)
        {
            var scale = Mathf.Max(0.001f, Mathf.Max(Mathf.Abs(left), Mathf.Abs(right)));
            return Mathf.Abs(left - right) / scale <= tolerance;
        }

        private static Vector3 RelativeScale(Transform partTransform, Transform geometryTransform)
        {
            return new Vector3(
                partTransform.InverseTransformVector(geometryTransform.TransformVector(Vector3.right)).magnitude,
                partTransform.InverseTransformVector(geometryTransform.TransformVector(Vector3.up)).magnitude,
                partTransform.InverseTransformVector(geometryTransform.TransformVector(Vector3.forward)).magnitude
            );
        }

        private static Quaternion RelativeRotation(Transform partTransform, Transform geometryTransform)
        {
            return (Quaternion.Inverse(partTransform.rotation) * geometryTransform.rotation).normalized;
        }

        private static Vector3 AbsVector(Vector3 value)
        {
            return new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
        }

        private static Vector3 MaxVector(Vector3 left, Vector3 right)
        {
            return new Vector3(Mathf.Max(left.x, right.x), Mathf.Max(left.y, right.y), Mathf.Max(left.z, right.z));
        }

        private static ProxyBounds EstimateProxyBounds(IList<ProxyGeometry> geometries)
        {
            var hasBounds = false;
            var minimum = Vector3.zero;
            var maximum = Vector3.zero;
            foreach (var geometry in geometries)
            {
                var size = geometry.Type == ProxyGeometryType.Box
                    ? geometry.Size
                    : geometry.Type == ProxyGeometryType.Sphere
                        ? Vector3.one * geometry.Radius * 2f
                        : new Vector3(geometry.Radius * 2f, geometry.Radius * 2f, geometry.Length);
                var extents = size * 0.5f;
                for (var corner = 0; corner < 8; corner++)
                {
                    var offset = new Vector3(
                        (corner & 1) == 0 ? -extents.x : extents.x,
                        (corner & 2) == 0 ? -extents.y : extents.y,
                        (corner & 4) == 0 ? -extents.z : extents.z
                    );
                    var local = geometry.Center + geometry.Rotation * offset;
                    if (!hasBounds)
                    {
                        minimum = local;
                        maximum = local;
                        hasBounds = true;
                    }
                    else
                    {
                        minimum = Vector3.Min(minimum, local);
                        maximum = Vector3.Max(maximum, local);
                    }
                }
            }

            return new ProxyBounds
            {
                Center = QuantizeVector((minimum + maximum) * 0.5f, 0.05f, -20f, 20f),
                Size = QuantizeVector(maximum - minimum, 0.1f, 0.1f, 20f)
            };
        }

        private static Vector3 QuantizeVector(Vector3 value, float step, float minimum, float maximum)
        {
            return new Vector3(
                Mathf.Clamp(Mathf.Round(value.x / step) * step, minimum, maximum),
                Mathf.Clamp(Mathf.Round(value.y / step) * step, minimum, maximum),
                Mathf.Clamp(Mathf.Round(value.z / step) * step, minimum, maximum)
            );
        }

        private static Vector3 UnityVectorToRos(Vector3 value)
        {
            return new Vector3(value.z, -value.x, value.y);
        }

        private static Vector3 UnitySizeToRos(Vector3 value)
        {
            return new Vector3(Mathf.Abs(value.z), Mathf.Abs(value.x), Mathf.Abs(value.y));
        }

        private static Quaternion UnityRotationToRos(Quaternion value)
        {
            return new Quaternion(-value.z, value.x, -value.y, value.w).normalized;
        }

        private static Vector3 RosQuaternionToRpy(Quaternion value)
        {
            var sinRollCosPitch = 2f * (value.w * value.x + value.y * value.z);
            var cosRollCosPitch = 1f - 2f * (value.x * value.x + value.y * value.y);
            var roll = Mathf.Atan2(sinRollCosPitch, cosRollCosPitch);

            var sinPitch = 2f * (value.w * value.y - value.z * value.x);
            var pitch = Mathf.Abs(sinPitch) >= 1f
                ? Mathf.Sign(sinPitch) * Mathf.PI * 0.5f
                : Mathf.Asin(sinPitch);

            var sinYawCosPitch = 2f * (value.w * value.z + value.x * value.y);
            var cosYawCosPitch = 1f - 2f * (value.y * value.y + value.z * value.z);
            var yaw = Mathf.Atan2(sinYawCosPitch, cosYawCosPitch);
            return new Vector3(roll, pitch, yaw);
        }

        private static void AppendVector(StringBuilder builder, Vector3 value)
        {
            AppendUrdfFloat(builder, value.x);
            builder.Append(' ');
            AppendUrdfFloat(builder, value.y);
            builder.Append(' ');
            AppendUrdfFloat(builder, value.z);
        }

        private static void AppendUrdfFloat(StringBuilder builder, float value)
        {
            builder.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }

        private static byte[] CompressUtf8(string value)
        {
            var input = Encoding.UTF8.GetBytes(value);
            using (var output = new MemoryStream())
            {
                using (var gzip = new GZipStream(output, CompressionLevel.Optimal, true))
                {
                    gzip.Write(input, 0, input.Length);
                }
                return output.ToArray();
            }
        }

        private static string Sha256Hex(byte[] value)
        {
            using (var sha256 = SHA256.Create())
            {
                var digest = sha256.ComputeHash(value);
                var hex = new StringBuilder(digest.Length * 2);
                foreach (var item in digest)
                {
                    hex.Append(item.ToString("x2", CultureInfo.InvariantCulture));
                }
                return hex.ToString();
            }
        }

        private void ClearActiveVesselUrdf()
        {
            if (!activeVesselUrdfAnnounced)
            {
                return;
            }

            try
            {
                var packet = new StringBuilder(192);
                packet.Append('{');
                AppendProperty(packet, "type", VesselProxyClearType, true);
                AppendProperty(packet, "version", VesselProxyVersion, false);
                AppendProperty(packet, "sessionId", vesselUrdfSessionId, false);
                packet.Append('}');
                SendUdp(packet.ToString());
            }
            catch (Exception ex)
            {
                WarnUdpThrottled("Active vessel URDF clear failed: " + ex.Message);
            }

            activeVesselUrdfAnnounced = false;
            vesselUrdfSessionId = Guid.NewGuid().ToString("N");
        }
    }
}
