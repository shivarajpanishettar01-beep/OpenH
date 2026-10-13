"""Per-sample labels of /robot/state that robot_control does not own (pure, shared rule).

robot_state_cpp implements the same rule in C++ (``label_sample``); keep the two in step.

* robot_control status younger than ``CONTROL_STATUS_MAX_AGE_S``: its mode / flags / source / expert /
  force setpoint are used (a status from a stopped robot_control is never carried forward).
* otherwise, freedrive: the configurable digital input wired to the foot pedal (set to "Freedrive" in the
  UR installation) is held and the robot is RUNNING -> FREEDRIVE / KINESTHETIC / expert.
* otherwise nobody commands the robot: IDLE / NONE / not expert.
Without robot_control there is no force setpoint: NaN, not 0.
"""
import math

CONTROL_STATUS_MAX_AGE_S = 0.05      # robot_control publishes at 125 Hz (8 ms)
CONFIGURABLE_INPUT_BIT0 = 8          # actual_digital_input_bits: 0-7 standard, 8-15 configurable, 16-17 tool
ROBOT_MODE_RUNNING = 7               # RTDE robot_mode

MODE_IDLE, MODE_FREEDRIVE = 0, 7
SOURCE_NONE, SOURCE_KINESTHETIC = 4, 5


def freedrive_engaged(freedrive_input, di_bits, robot_mode):
    """True if the pedal input (configurable input 0-7, None = not configured) is held and the arm runs."""
    if freedrive_input is None or di_bits is None or robot_mode != ROBOT_MODE_RUNNING:
        return False
    return bool((int(di_bits) >> (CONFIGURABLE_INPUT_BIT0 + int(freedrive_input))) & 1)


def label_sample(control, control_age_s, freedrive_input, di_bits, robot_mode):
    """(force_setpoint_n, mode, safety_flags, action_source, action_is_expert) for one RTDE sample.
    ``control``: robot_control's tuple in that order, or None."""
    if control is not None and control_age_s is not None and control_age_s <= CONTROL_STATUS_MAX_AGE_S:
        return tuple(control)
    if freedrive_engaged(freedrive_input, di_bits, robot_mode):
        return (math.nan, MODE_FREEDRIVE, 0, SOURCE_KINESTHETIC, True)
    return (math.nan, MODE_IDLE, 0, SOURCE_NONE, False)
