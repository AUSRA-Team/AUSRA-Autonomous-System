string   robot_name
uint8    state
# State constants: 0=IDLE 1=NAVIGATING 2=DEGRADED 3=ESTOP 4=LOST(fleet-only) 5=EXPLORING
float32  battery_pct
float32  pose_x
float32  pose_y
float32  pose_yaw
string   active_task_id
string[] degraded_systems
builtin_interfaces/Time stamp
