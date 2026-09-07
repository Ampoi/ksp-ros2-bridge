# Target minus observer, expressed in header.frame_id WORLD axes, not body axes.
# Velocity is d(relative_position)/dt in those axes, in m/s.
# Valid only for this observer and the matching VesselLifecycle epoch.
# The frame may be startup IMU axes; no absolute world pose is implied.
std_msgs/Header header
string observer_vessel_id
uint64 origin_sequence
string target_id
string source
geometry_msgs/Vector3 relative_position
geometry_msgs/Vector3 relative_velocity
uint32 observations
