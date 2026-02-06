# Block execution status

std_msgs/Header header

string block_id

uint8 state
uint8 STATE_UNKNOWN=0
uint8 STATE_PENDING=1
uint8 STATE_RUNNING=2
uint8 STATE_PAUSED=3
uint8 STATE_COMPLETED=4
uint8 STATE_FAILED=5

float32 progress    # [0..1]
string message
