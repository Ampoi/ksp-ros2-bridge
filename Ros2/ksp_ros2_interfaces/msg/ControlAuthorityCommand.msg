uint8 ACTION_ACQUIRE=1
uint8 ACTION_RENEW=2
uint8 ACTION_RELEASE=3
uint8 ACTION_EMERGENCY_STOP=4
uint8 ACTION_CLEAR_EMERGENCY_STOP=5

std_msgs/Header header
uint8 action
string vessel_id
string controller_id
string lease_id
int32 priority
float64 lease_duration_sec
bool suppress_sas
uint64 sequence
