"""pose_monitor: live TCP pose, its change from a reference, and the labels robot_state puts on it.

  ros2 run usrig_robot pose_monitor [--rate 10]

Read-only (subscribes to /robot/state; never commands anything). Run it in its own terminal while you
call a topic or service, teleop, run a check tool or guide the arm in freedrive: the change from the
reference shows in the base frame and in the probe frame at the reference (x along the array, y across,
z into tissue). Enter = set the reference to the current pose; q + Enter = quit.
"""
import argparse
import select
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from usrig_common import geometry
from usrig_common import motion_check as mc
from usrig_interfaces.msg import RobotState

MODES = ['IDLE', 'MOVING', 'DESCENT', 'TELEOP', 'PERTURB', 'RETRACT', 'FAULT', 'FREEDRIVE']
SOURCES = ['teleop', 'scripted_task', 'transit', 'perturbation', 'none', 'kinesthetic']
FLAGS = {1: 'envelope', 2: 'tilt', 4: 'force_max', 8: 'deadman', 16: 'speed'}
CLEAR = '\033[2J\033[H'


def render(m, T, T_ref, speed_mm_s, age_s):
    """The screen for one state message (pure, testable)."""
    d = mc.pose_delta(T_ref, T)
    mode = MODES[m.mode] if m.mode < len(MODES) else str(m.mode)
    src = SOURCES[m.action_source] if m.action_source < len(SOURCES) else str(m.action_source)
    flags = [n for b, n in FLAGS.items() if m.safety_flags & b]
    f_axis = float(T[:3, 2] @ np.asarray(m.wrench[:3]))
    stale = f'   <<< STALE {age_s:.1f} s' if age_s > 0.5 else ''
    fsp = 'none' if np.isnan(m.force_setpoint_n) else f'{m.force_setpoint_n:.1f} N'
    ph = geometry.pose_from_msg(m.tcp_phantom)
    return '\n'.join([
        f'pose_monitor  {time.strftime("%H:%M:%S")}   (Enter = reset reference, q + Enter = quit){stale}',
        f'mode {mode}   source {src}   expert {m.action_is_expert}   flags {", ".join(flags) or "none"}',
        f'TCP base     {mc.fmt_pose(T)}',
        f'TCP phantom  {mc.fmt_pose(ph)}',
        f'speed {speed_mm_s:6.1f} mm/s   probe-axis force {f_axis:+6.2f} N   force setpoint {fsp}',
        'change from reference:',
        mc.fmt_delta(d)])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=10.0)
    a, _ = ap.parse_known_args(argv)
    rclpy.init()
    n = rclpy.create_node('pose_monitor')
    lock = threading.Lock()
    st = {'m': None, 't': None, 'hist': []}

    def cb(m):
        with lock:
            st['m'], st['t'] = m, time.monotonic()
            st['hist'] = (st['hist'] + [(st['t'], geometry.pose_from_msg(m.tcp_base)[:3, 3])])[-50:]
    n.create_subscription(RobotState, '/robot/state', cb,
                          QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    T_ref = None
    stdin_open = sys.stdin is not None and not sys.stdin.closed
    try:
        while rclpy.ok():
            if stdin_open and select.select([sys.stdin], [], [], 1.0 / a.rate)[0]:
                raw = sys.stdin.readline()
                if raw == '':                  # EOF (no terminal, e.g. started from a launch file)
                    stdin_open = False
                    continue
                if raw.strip().lower() == 'q':
                    break
                T_ref = None
            elif not stdin_open:
                time.sleep(1.0 / a.rate)
            with lock:
                m, t, hist = st['m'], st['t'], list(st['hist'])
            if m is None:
                print(CLEAR + 'pose_monitor: waiting for /robot/state (robot.launch.py running?)', flush=True)
                continue
            T = geometry.pose_from_msg(m.tcp_base)
            if T_ref is None:
                T_ref = T.copy()
            sp = 0.0
            if len(hist) > 5 and hist[-1][0] > hist[0][0]:
                sp = np.linalg.norm(hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0]) * 1000.0
            print(CLEAR + render(m, T, T_ref, sp, time.monotonic() - t), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
