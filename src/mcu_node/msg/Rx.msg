builtin_interfaces/Time stamp
uint64 sequence
# The upstream field is named system_time_point_ms, but the current C board sends microseconds.
uint64 device_sample_time_us
# IMU values are preserved exactly as received; this bridge performs no unit conversion.
float32 imu_yaw
float32 imu_pitch
float32 imu_roll
float32 imu_yaw_vel
float32 imu_pitch_vel
float32 imu_roll_vel
float32 imu_yaw_acc
float32 imu_pitch_acc
float32 imu_roll_acc
float32 imu_quat_w
float32 imu_quat_x
float32 imu_quat_y
float32 imu_quat_z
int16 order
int16 remaining_time_s
int8 sentry_behavior_mode
uint8 colour
# dart_fin_v1 fields. angle1 starts at the upper-left fin and continues clockwise
# for the X-layout: upper-left, upper-right, lower-right, lower-left.
float32 vehicle_x_vel
float32 vehicle_y_vel
float32 angle1
float32 angle2
float32 angle3
float32 angle4
