std_msgs/Header header
string vessel_id
string controller_id
string lease_id
uint64 sequence
string id
bool enabled
float64 target_thrust
float64 timeout_sec
# Opt in to engine-local stock TVC control; false releases an earlier override.
bool has_gimbal_command
# KSP pitch/yaw/roll inputs, each in [-1, 1]. Stock limits and locks apply.
float64 gimbal_pitch
float64 gimbal_yaw
float64 gimbal_roll
