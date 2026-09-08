uint8 MODE_IDLE=0
uint8 MODE_DETUMBLE=1
uint8 MODE_ATTITUDE_HOLD=2
uint8 MODE_SIX_DOF=3

std_msgs/Header header
uint8 mode
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity
