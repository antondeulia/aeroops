std_msgs/Header header

uint8 IDLE=0
uint8 LOADED=1
uint8 RUNNING=2
uint8 PAUSED=3
uint8 COMPLETED=4
uint8 FAILED=5
uint8 ABORTED=6

uint8 status
string mission_id
uint32 current_waypoint
uint32 total_waypoints
string message