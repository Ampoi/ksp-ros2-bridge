# Monorepo architecture

This repository is organized around bounded contexts rather than deployment
technology. Dependencies point inward: adapters may depend on application and
domain code, while domain code never imports KSP, Unity, ROS2, or demo code.

| Area | Role | May depend on |
|---|---|---|
| `Source/KerbalLiDAR/Domain` | Pure allocation, authority, safety, and value rules | .NET base library |
| `Source/KerbalLiDAR/Application` | KSP-side control use cases | KSP domain |
| `Source/KerbalLiDAR/Api/Ksp` | Unity/KSP and UDP adapters | KSP application/domain |
| `Ros2/ksp_ros2_interfaces` | Versioned ROS2 integration contracts | ROS message packages |
| `Ros2/ksp_lidar_bridge/domain` | Packet validation and time-alignment rules | Python standard library |
| `Ros2/ksp_lidar_bridge/udp_bridge.py` | UDP, ROS2, QoS, TF, and lifecycle adapter | bridge domain + interfaces |
| `Ros2/ksp_vehicle_control/domain` | Reusable 6DoF control law | Python standard library |
| `Ros2/ksp_vehicle_control/application` | Lease/control workflows | vehicle-control domain |
| `Ros2/ksp_vehicle_control/adapters` | ROS2 controller nodes | application/domain + interfaces |
| `Ros2/ksp_nav2_bringup` | Nav2/SLAM integration adapter | public ROS2 contracts |
| `Demo` | Runnable examples and experiment configuration | public ROS2 packages only |
| `build.sh` / `build.ps1` / `sync.sh` | Reproducible production build and installation | production sources only |
| `Development` (local, ignored) | Debug helpers, probes, evidence, asset authoring | excluded from Git and production compilation |

## Vehicle-control aggregate

The KSP process is the authority boundary for the active vessel. A controller
must acquire a time-bounded lease for the exact KSP `vessel_id`. Commands carry
`vessel_id`, `controller_id`, `lease_id`, and a monotonic per-lease sequence.
KSP rejects stale, wrong-vessel, non-owner, and emergency-stopped commands.

SAS ownership, actuator overrides, command timeout, angular-speed limiting,
wrench slew limiting, continuous-actuation limiting, and emergency stop are
applied inside KSP. They therefore remain effective if the ROS graph stalls.

RCS allocation is based on each currently enabled nozzle's KSP transform,
thrust direction, position relative to center of mass, and enabled translation
or rotation axes. The result remains an allocation request to KSP's normalized
flight-control channels; feedback distinguishes requested, allocated, measured,
and residual wrench instead of claiming exact force realization.

## Frame and lifecycle contract

- `ground_truth_enu -> base_link` is dynamic and stamped from KSP universal time.
- `base_link -> ksp_<vessel-id-prefix>_link_0000` is dynamic because center of
  mass can move.
- fixed proxy joints and sensor mount extrinsics are published on `/tf_static`.
- sensor frames use configured Sensor IDs and remain stable across proxy refreshes.
- `/ksp_vessel/lifecycle` distinguishes unavailable, active, changed, and stale
  states and provides the exact commandable `vessel_id`.
- both world-frame `/ground_truth/twist` and body-frame
  `/ground_truth/twist_body` are published; reusable controllers own the
  world-to-body conversion.

## Non-production areas

`Demo/` is intentionally not a source of reusable control or bridge behavior.
The debris-orbit demo publishes a `ControlSetpoint`; `ksp_vehicle_control`
implements the controller. Nav2 owns only its planar adapter. `Development/`
is local-only and ignored, including its C# helpers, probes, evidence and asset
authoring sources. Legacy `Tools/` and root `dev_*.sh` entrypoints are also
ignored. Production builds work from a clean clone without these directories.
The C# project explicitly includes only its production layers and excludes
debug controllers. `sync.sh` builds and installs only the production output.
