"""Per-sample labels without robot_control: freedrive detection (labels.py, mirrored in robot_state_cpp)."""
import math

from usrig_robot.labels import label_sample

CTL = (4.0, 3, 0, 0, True)           # robot_control: TELEOP, teleop, expert, 4 N
PEDAL_ON_INPUT_2 = 1 << (8 + 2)      # configurable input 2 = bit 10


def test_fresh_control_status_wins():
    assert label_sample(CTL, 0.01, 2, PEDAL_ON_INPUT_2, 7) == CTL


def test_stale_control_status_is_not_carried_forward():
    fsp, mode, flags, src, expert = label_sample(CTL, 0.2, None, 0, 7)
    assert math.isnan(fsp) and (mode, src, expert) == (0, 4, False)


def test_freedrive_needs_pedal_bit_and_running_robot():
    fsp, mode, flags, src, expert = label_sample(None, None, 2, PEDAL_ON_INPUT_2, 7)
    assert math.isnan(fsp) and (mode, src, expert) == (7, 5, True)
    assert label_sample(None, None, 2, PEDAL_ON_INPUT_2, 5)[3] == 4      # robot not RUNNING
    assert label_sample(None, None, 3, PEDAL_ON_INPUT_2, 7)[3] == 4      # another input
    assert label_sample(None, None, 2, 1 << 2, 7)[3] == 4                # standard input 2, not configurable 2
    assert label_sample(None, None, None, PEDAL_ON_INPUT_2, 7)[3] == 4   # freedrive not configured
