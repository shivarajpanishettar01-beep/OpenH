# One RTDE sample of the UR7e, published at 500 Hz by robot_state.
# Every message derived from the same sample (/joint_states, /robot/tcp_pose, /robot/wrench)
# carries the same header.stamp.

uint8 MODE_IDLE=0
uint8 MODE_MOVING=1
uint8 MODE_DESCENT=2
uint8 MODE_TELEOP=3
uint8 MODE_PERTURB=4
uint8 MODE_RETRACT=5
uint8 MODE_FAULT=6
uint8 MODE_FREEDRIVE=7          # arm guided by hand (UR freedrive input held), nothing commanded

uint16 FLAG_ENVELOPE=1
uint16 FLAG_TILT_CONE=2
uint16 FLAG_FORCE_MAX=4
uint16 FLAG_DEADMAN=8
uint16 FLAG_SPEED=16

uint8 SOURCE_TELEOP=0
uint8 SOURCE_SCRIPTED_TASK=1
uint8 SOURCE_TRANSIT=2
uint8 SOURCE_PERTURBATION=3
uint8 SOURCE_NONE=4
uint8 SOURCE_KINESTHETIC=5      # motion by the operator's hand in freedrive (expert demonstration)

std_msgs/Header header            # PC system clock, stamped once per RTDE sample
float64 controller_time_s         # RTDE timestamp from the UR controller
float64[6] q
float64[6] qd
geometry_msgs/Pose tcp_base
geometry_msgs/Pose tcp_phantom    # identity until phantom registration exists
float64[6] wrench                 # base frame, as reported by the controller
float64 force_setpoint_n          # NaN when robot_control is not commanding (no setpoint exists)
uint8 mode                        # IDLE MOVING DESCENT TELEOP PERTURB RETRACT FAULT FREEDRIVE
uint16 safety_flags               # envelope, tilt-cone, force-max, deadman, speed
uint8 action_source               # TELEOP SCRIPTED_TASK TRANSIT PERTURBATION NONE KINESTHETIC
bool action_is_expert
