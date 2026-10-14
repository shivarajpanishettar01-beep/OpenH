"""teleop_check: move the probe along ONE probe-frame axis through the teleop path and report the pose change.

  ros2 run usrig_teleop teleop_check --axis slide_along --speed 5 --seconds 2 [--back] [--yes]
  axes: slide_along (vx, W/S)  slide_across (vy, A/D)  beam (vz)  tilt (wx, R/F)  rock (wy, Q/E)  rotate (wz, Z/C)
  speed: mm/s for slides, deg/s for rotations (negative = the other direction)

Uses exactly what the keyboard uses: /robot/set_teleop, then /teleop/twist (probe frame, 100 Hz) into
robot_control, which filters it (speed, accel, box, cone) and commands the arm. Needs robot_control running
(robot.launch.py control:=true) and NO other /teleop/twist publisher (stop keyboard_teleop). Prints the TCP
pose before and after, the change in the base and probe frames, the executed command and the safety flags
seen, and PASS / FAIL for direction, amount, other axes, and (rotations) pivot at the probe face.
Teleop is disabled again at the end, also on Ctrl+C or error.
"""
import argparse
import threading
import time

import numpy as np
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool

from usrig_common import motion_check as mc
from usrig_common.timeutil import now_ns, stamp_from_ns
from usrig_interfaces.msg import RobotState

MAX_TRAVEL_MM = 50.0        # this tool refuses bigger test motions (robot_control limits still apply)
MAX_ANGLE_DEG = 20.0
FLAGS = {1: 'envelope', 2: 'tilt', 4: 'force_max', 8: 'deadman', 16: 'speed'}


class Recorder:
    """Safety flags and modes seen on /robot/state, and the executed command, during the motion."""

    def __init__(self, node):
        self.lock = threading.Lock()
        self.active = False
        self.reset()
        node.create_subscription(RobotState, '/robot/state', self._state, 50)
        node.create_subscription(TwistStamped, '/robot/cmd_executed', self._cmd, 50)

    def reset(self):
        with self.lock:
            self.flags, self.modes, self.cmd_max = 0, set(), np.zeros(6)

    def _state(self, m):
        if self.active:
            with self.lock:
                self.flags |= int(m.safety_flags)
                self.modes.add(int(m.mode))

    def _cmd(self, m):
        if self.active:
            t = m.twist
            v = np.abs([t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z])
            with self.lock:
                self.cmd_max = np.fmax(self.cmd_max, v)


def check_ready(node):
    s = node.state()
    age_s = (now_ns() - (s.header.stamp.sec * 10**9 + s.header.stamp.nanosec)) * 1e-9
    if abs(age_s) > 0.5:
        raise RuntimeError(f'/robot/state is {age_s:.1f} s old (robot_state running? clock?)')
    if s.mode == RobotState.MODE_FAULT:
        raise RuntimeError('robot in FAULT: acknowledge (K / {"cmd": "ack_fault"}) first')
    n = node.count_publishers('/teleop/twist')
    if n:
        raise RuntimeError(f'{n} node(s) already publish /teleop/twist (keyboard_teleop / spacemouse): stop them first')


def run_axis(node, cli, pub, rec, axis, speed, seconds, rate_hz=100.0):
    """One timed motion; returns (T_before, T_after)."""
    i, rot = mc.AXES[axis]
    tw = np.zeros(6)
    tw[i + (3 if rot else 0)] = np.radians(speed) if rot else speed / 1000.0
    rec.reset()                                      # each motion reports only its own flags
    T0 = node.averaged_tcp_pose()
    for _ in range(int(0.1 * rate_hz)):              # input alive before teleop is enabled (no deadman blip)
        send(pub, np.zeros(6))
        time.sleep(1.0 / rate_hz)
    r = node.call(cli, SetBool.Request(data=True))
    if not r.success:
        raise RuntimeError(f'set_teleop refused: {r.message}')
    rec.active = True
    try:
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            send(pub, tw)
            time.sleep(1.0 / rate_hz)
    finally:
        for _ in range(int(0.5 * rate_hz)):          # explicit zero, then let robot_control hold
            send(pub, np.zeros(6))
            time.sleep(1.0 / rate_hz)
        rec.active = False
        node.call(cli, SetBool.Request(data=False))
    time.sleep(0.5)
    return T0, node.averaged_tcp_pose()


def send(pub, tw):
    m = TwistStamped()
    m.header.stamp = stamp_from_ns(now_ns())
    m.header.frame_id = 'probe'
    (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
     m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = map(float, tw)
    pub.publish(m)


def report(axis, speed, seconds, T0, T1, rec, label):
    _, rot = mc.AXES[axis]
    expected = speed * seconds
    d = mc.pose_delta(T0, T1)
    print(f'\n--- {label}: {axis} ({mc.KEYS[axis]}) {speed:+g} {"deg/s" if rot else "mm/s"} for {seconds:g} s')
    print(f'before {mc.fmt_pose(T0)}\nafter  {mc.fmt_pose(T1)}\n{mc.fmt_delta(d)}')
    with rec.lock:
        flags = [n for b, n in FLAGS.items() if rec.flags & b]
        cm = rec.cmd_max.copy()
    print(f'executed command peak: |v| [{cm[0] * 1000:.1f} {cm[1] * 1000:.1f} {cm[2] * 1000:.1f}] mm/s  '
          f'|w| [{np.degrees(cm[3]):.1f} {np.degrees(cm[4]):.1f} {np.degrees(cm[5]):.1f}] deg/s;  '
          f'safety flags seen: {", ".join(flags) or "none"}')
    ok, rows = mc.evaluate_axis(d, axis, expected)
    limits = [f for f in flags if f != 'deadman']
    rows.append(('no limit hit', not limits, f'robot_control limited the command: {", ".join(limits)} '
                                             '(speed above the cap, or at the box / cone)' if limits else 'none'))
    if 'deadman' in flags:
        print('note: deadman flag seen (input gap > deadman_timeout_s: a loaded PC or network?)')
    return mc.print_table(f'teleop_check {label}', rows) and ok


def main(argv=None, ask=input, node=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--axis', required=True)
    ap.add_argument('--speed', type=float, required=True, help='mm/s (slides) or deg/s (rotations); sign = direction')
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--back', action='store_true', help='return with the opposite motion afterwards')
    ap.add_argument('--yes', action='store_true', help='skip the confirmation (scripts, simulation)')
    a, _ = ap.parse_known_args(argv)
    axis = mc.axis_name(a.axis)
    _, rot = mc.AXES[axis]
    total = abs(a.speed * a.seconds)
    if a.seconds <= 0 or a.speed == 0:
        raise SystemExit('speed and seconds must be non-zero')
    if total > (MAX_ANGLE_DEG if rot else MAX_TRAVEL_MM):
        raise SystemExit(f'{total:.1f} {"deg" if rot else "mm"} is more than this check allows '
                         f'({MAX_ANGLE_DEG if rot else MAX_TRAVEL_MM}); use a smaller speed or time')

    def body(n):
        check_ready(n)
        cli = n.create_client(SetBool, '/robot/set_teleop')
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/robot/set_teleop not available: start robot.launch.py control:=true')
        rec = Recorder(n)
        pub = n.create_publisher(TwistStamped, '/teleop/twist', 10)
        print(f'TCP now {mc.fmt_pose(n.averaged_tcp_pose())}')
        what = (f'teleop_check will move the probe {a.speed * a.seconds:+.1f} {"deg" if rot else "mm"} '
                f'along probe {axis} ({mc.KEYS[axis]}) at {abs(a.speed):g} {"deg/s" if rot else "mm/s"}'
                f'{" and back" if a.back else ""}.')
        if not mc.confirm(what, ask, a.yes):
            print('not confirmed: nothing moved')
            return False
        T0, T1 = run_axis(n, cli, pub, rec, axis, a.speed, a.seconds)
        ok = report(axis, a.speed, a.seconds, T0, T1, rec, 'out')
        if a.back:
            T2, T3 = run_axis(n, cli, pub, rec, axis, -a.speed, a.seconds)
            ok = report(axis, -a.speed, a.seconds, T2, T3, rec, 'back') and ok
            print(f'net change start -> end:\n{mc.fmt_delta(mc.pose_delta(T0, T3))}')
        return ok

    if node is not None:
        return body(node)
    from usrig_common.tool_node import run_tool
    out = {}
    run_tool(lambda n: out.update(ok=body(n)), 'teleop_check')
    raise SystemExit(0 if out.get('ok') else 1)


if __name__ == '__main__':
    main()
