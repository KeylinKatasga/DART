builtin_interfaces/Time stamp
# /command is a request; /message/tx is published only after serial write succeeds.
# This confirms host write completion, not an MCU acknowledgement.
# Wire units: deg, deg/s and deg/s^2. Convert planner radians before publishing.
float32 yaw
float32 yaw_vel
float32 yaw_acc
float32 pitch
float32 pitch_vel
float32 pitch_acc
uint8 enable_lock
uint8 enable_shoot
int32 autoaim_living_waterfall_flag

