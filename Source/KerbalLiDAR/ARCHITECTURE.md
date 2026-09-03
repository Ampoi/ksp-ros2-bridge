# KerbalLiDAR architecture

The mod follows an inward-dependency rule:

- `Domain/` contains deterministic rules and value-oriented logic. It must not
  reference KSP (`Assembly-CSharp`) or Unity types.
- `Api/Ksp/` is the KSP adapter layer. `PartModule` implementations, scene and
  vessel access, Unity rendering/physics, lifecycle callbacks, and KSP UI live
  here.
- ROS 2 conversion is implemented in `Ros2/ksp_lidar_bridge`; the UDP packet is
  the boundary contract between the KSP adapter and that process.

New behavior should be expressed in `Domain/` when it can operate on primitive
values. The KSP adapter translates KSP objects into those values and applies the
result. This keeps game lifecycle concerns out of the reusable rules while the
assembly remains a single KSP plugin.

## LiDAR identity contract

`ModuleKerbalRosSensorId.sensorId` is the canonical sensor identity. LiDAR scan
and inactive packets emit `sensorId`; `partName` and `lidarName` are read only to
migrate old craft files. The bridge publishes LiDAR data as:

`/ksp_vessel/lidar_2d/<sensor_id>/scan`

`/ksp_vessel/lidar_3d/<sensor_id>/points`

## Active-vessel TF contract

The runtime proxy bundle includes the root part pose relative to the vessel
`base_link` origin (the vessel center of mass). The bridge publishes
`base_link -> ksp_<session>_link_0000`, the URDF fixed-joint tree, and each
part-to-sensor transform. Ground truth supplies
`ground_truth_enu -> base_link`, keeping world, vessel, proxy-part, and sensor
frames in one connected TF tree.
