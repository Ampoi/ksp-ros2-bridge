# Orientation-only measurement; never supplies a position estimate.
# header.frame_id = kerbol_inertial: fixed KSP celestial axes (right, forward, up).
std_msgs/Header header
string vessel_id
string sensor_id
string session_id
uint64 sequence
float64 universal_time
# Quaternion rotates this frame's vectors into header.frame_id (ROS FLU body).
string measured_frame_id
bool valid
# tracking, acquiring, disabled, no_power, atmosphere, sun_exclusion,
# body_in_fov, occluded, slew_rate_exceeded, packed, sensor_unavailable,
# invalid_time, inactive, stale
string reason
# All zeros when invalid: do not normalize or use an invalid quaternion.
geometry_msgs/Quaternion orientation
# Small angle covariance in measured frame (rad^2), row-major; [0]=-1 if invalid.
float64[9] orientation_covariance
float64 angular_rate_deg_sec
