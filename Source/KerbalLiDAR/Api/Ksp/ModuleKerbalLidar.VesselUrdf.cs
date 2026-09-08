using UnityEngine;
namespace KerbalLiDAR
{
    public partial class ModuleKerbalLidar
    {
        private static Vector3 UnityVectorToRos(Vector3 value)
        { return KerbalRosVesselModelManager.UnityVectorToRos(value); }
        private static Quaternion UnityRotationToRos(Quaternion value)
        { return KerbalRosVesselModelManager.UnityRotationToRos(value); }
    }
}
