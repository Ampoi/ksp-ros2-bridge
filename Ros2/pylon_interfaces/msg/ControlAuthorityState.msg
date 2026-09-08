uint8 STATE_UNOWNED=0
uint8 STATE_OWNED=1
uint8 STATE_EMERGENCY_STOP=2

std_msgs/Header header
uint8 state
string vessel_id
string vessel_name
string controller_id
string lease_id
int32 priority
float64 lease_remaining_sec
bool sas_suppressed
bool emergency_stop
uint64 last_sequence
string reason
