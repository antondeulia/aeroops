std_msgs/Header header

uint8 UNKNOWN=0
uint8 DISCONNECTED=1
uint8 CONNECTED=2
uint8 ARMED=3
uint8 FLYING=4
uint8 LANDING=5
uint8 ERROR=6

uint8 state
string mode
float64 battery_percent
float64 latitude
float64 longitude
float64 altitude_m
float64 velocity_mps
