# One synchronous snapshot, using the active vessel's pylon_ground_truth_enu origin.
# Only loaded, unpacked vessels within 2500 m on the same celestial body.
std_msgs/Header header
string observer_vessel_id
uint64 origin_sequence
geometry_msgs/Point observer_position
geometry_msgs/Vector3 observer_linear_velocity
NearbyVessel[] vessels
