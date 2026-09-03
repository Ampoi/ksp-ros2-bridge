std_msgs/Header header

string vessel_id
string controller_id
string lease_id
uint64 sequence
bool accepted
string reason

geometry_msgs/Wrench requested
geometry_msgs/Wrench allocated
geometry_msgs/Wrench achieved
geometry_msgs/Wrench allocation_residual
geometry_msgs/Wrench tracking_residual
float64 saturation_ratio
float64 tracking_error_ratio
bool saturated

# `achieved` is reconstructed from the preceding KSP physics tick's engine and
# per-nozzle RCS thrust. Wheel contact forces are not observable and are excluded.
string achieved_quality
