# KerbalLiDAR architecture

The mod follows an inward-dependency rule:

- `Domain/` contains deterministic rules and value-oriented logic, including
  control authority, RCS allocation, and last-resort safety limits. It must not
  reference KSP (`Assembly-CSharp`) or Unity types.
- `Application/` composes domain rules into KSP-side use cases without owning
  Unity lifecycle or UDP serialization.
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
`base_link -> ksp_<vessel-id-prefix>_link_0000` as a dynamic CoM-relative edge.
The URDF fixed-joint tree and each part-to-sensor transform use `/tf_static`.
Ground truth supplies
`ground_truth_enu -> base_link`, keeping world, vessel, proxy-part, and sensor
frames in one connected TF tree. Sensor frames are derived from configurable
Sensor IDs rather than ephemeral part frame names.

## Vehicle-control authority

The `ControlAuthority` aggregate inside the KSP process is the source of truth
for controller ownership. Every formal command targets the active vessel ID and
an acknowledged lease. SAS suppression, emergency stop, timeouts, angular-rate
limits, command slew, and continuous-actuation limits execute in this process so
they do not depend on ROS callback health.

## Production build boundary

The project explicitly compiles `Domain`, `Application`, `Api/Ksp`, and
`Properties/AssemblyInfo.cs`. Debug controllers and local development code are
excluded. Production code must not import files or build targets from the
ignored `Development/` tree. Local probes, evidence and authoring assets stay
there; only distributable models belong under `GameData/`.
