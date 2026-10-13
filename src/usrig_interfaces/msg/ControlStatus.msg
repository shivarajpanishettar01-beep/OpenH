# Internal: robot_control -> robot_state_cpp, merged into /robot/state (mode, flags, source, expert).
# Published at the control rate (125 Hz); recorded for audit. Constants as in RobotState.
std_msgs/Header header
float64 force_setpoint_n
uint8 mode
uint16 safety_flags
uint8 action_source
bool action_is_expert
