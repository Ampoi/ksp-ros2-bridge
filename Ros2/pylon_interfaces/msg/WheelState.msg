std_msgs/Header header
string id
string name
bool enabled
bool grounded
float64 angular_position
float64 angular_velocity
float64 steering_angle
float64 drive_torque
float64 brake_torque
float64 slip
float64 max_drive_torque
# Physical geometry in base_link, independent of world pose. Zero radius means
# an older plugin without geometry support; autonomous driving must reject it.
string vessel_id
uint32 wheel_count
float64 radius
geometry_msgs/Point position
float64 rolling_sign
float64 steering_sign
bool steering_enabled
float64 max_steering_angle
geometry_msgs/Point body_min
geometry_msgs/Point body_max
