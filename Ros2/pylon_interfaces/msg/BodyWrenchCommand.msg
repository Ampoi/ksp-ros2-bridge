std_msgs/Header header

# The command is accepted only while this lease owns the active vessel.
string vessel_id
string controller_id
string lease_id
uint64 sequence

geometry_msgs/Wrench wrench
float64 timeout_sec
