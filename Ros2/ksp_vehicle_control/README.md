# ksp_vehicle_control

Reusable high-level vehicle control for KSP ROS2. The package accepts a
world-frame `ControlSetpoint`, tracks Ground Truth pose/twist, acquires the exact
active vessel through the control-authority lease API, and publishes bounded
body-frame `BodyWrenchCommand` messages.

The pure control law lives in `domain/`, lease workflow in `application/`, and
ROS2 node in `adapters/ros2/`. Demo-specific target detection and Nav2 behavior
do not belong here.

```bash
ros2 run ksp_vehicle_control setpoint_controller --ros-args \
  -p controller_id:=my_controller \
  -p setpoint_topic:=/my_controller/setpoint
```

The node does not infer a vessel from a display name. It waits for
`/ksp_vessel/lifecycle`, acquires the reported `vessel_id`, waits for an owned
authority acknowledgement, and releases the lease on shutdown. SAS suppression
is part of the lease rather than a side effect of individual torque commands.
