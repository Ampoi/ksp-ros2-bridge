uint8 STATE_UNAVAILABLE=0
uint8 STATE_ACTIVE=1
uint8 STATE_CHANGED=2
uint8 STATE_STALE=3

std_msgs/Header header
uint8 state
string vessel_id
string vessel_name
uint64 generation
uint64 origin_sequence
string world_frame
string body_frame
bool model_ready
string model_id
string reason
