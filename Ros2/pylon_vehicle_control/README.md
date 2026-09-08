# pylon_vehicle_control

Reusable high-level vehicle control for KSP ROS2. The package accepts a
world-frame `ControlSetpoint`, tracks Ground Truth pose/twist, acquires the exact
active vessel through the control-authority lease API, and publishes bounded
body-frame `BodyWrenchCommand` messages.

All force, torque, and controller gains are expressed against SI N/N·m at the
ROS boundary. The KSP adapter owns the conversion from KSP's kN/kN·m flight
units; callers must not pre-scale commands by 1000.

```bash
ros2 run pylon_vehicle_control setpoint_controller --ros-args \
  -p controller_id:=my_controller \
  -p setpoint_topic:=/my_controller/setpoint
```

The node does not infer a vessel from a display name. It waits for
`/ksp_vessel/lifecycle`, acquires the reported `vessel_id`, waits for an owned
authority acknowledgement, and releases the lease on shutdown. SAS suppression
is part of the lease rather than a side effect of individual torque commands.

For moving setpoints, `extrapolate_setpoint: true` advances the desired position
from `ControlSetpoint.header.stamp` to the current pose sample using the desired
linear velocity. Use the bridge's mapped KSP timestamp from the Ground Truth
pose header for both samples; do not mix it with a fresh wall-clock timestamp.
Offsets larger than `setpoint_timeout_sec` stop commands and release the lease.
The default is `false` for existing stationary-setpoint publishers. The debris
orbit demo enables this option to compensate for sample age at orbital speeds.

## Contributing to PyLoN

See the [contributor guide](../../docs/contributing/index.md) for build, validation, and source organization.
