"""freedrive_check: guided check of freedrive collection on the real arm. Never commands the robot.

  ros2 run usrig_freedrive freedrive_check [--still-s 2] [--drift-s 3]

Needs robot_state (robot.launch.py, control:=false) and rig.yaml freedrive.input set. Four steps, each
started with Enter; /robot/state is recorded during each step:
  1. pedal released, hands off      -> labelled IDLE (not kinesthetic), arm still (< 0.5 mm)
  2. hold the pedal, move the probe  -> labelled FREEDRIVE / kinesthetic / expert while it moves; every
     about 5 cm and back, release      sample that moves (> 5 mm/s) is labelled kinesthetic; pose change printed
  3. pedal released, hands off      -> locked again: IDLE, still (< 0.5 mm)
  4. pedal HELD, hand off the handle -> gravity compensation: drift < 2 mm in 3 s (payload / CoG right);
     (keep the hand next to it)         skip with s if you do not want to let go
Prints the pose change of each step and a PASS / FAIL table.
"""
import argparse
import threading
import time

import numpy as np

from usrig_common import config, geometry
from usrig_common import motion_check as mc
from usrig_interfaces.msg import ControlStatus, RobotState

STILL_MM, STILL_DEG = 0.5, 0.2
DRIFT_MM = 2.0
MOVING_MM_S = 5.0
BRAKE_GRACE_S = 0.15       # after the pedal is released the arm takes a moment to stop: not 'unlabelled motion'


class Buffer:

    def __init__(self, node):
        self.lock = threading.Lock()
        self.rows = []              # (t_s, T, mode, source, expert)
        self.on = False
        self.ctl_t = None
        node.create_subscription(RobotState, '/robot/state', self._state, 200)
        node.create_subscription(ControlStatus, '/robot/control_status', self._ctl, 10)

    def _state(self, m):
        if self.on:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            with self.lock:
                self.rows.append((t, geometry.pose_from_msg(m.tcp_base), int(m.mode), int(m.action_source),
                                  bool(m.action_is_expert)))

    def _ctl(self, m):
        self.ctl_t = time.monotonic()

    def record(self, until):
        """Samples stamped between the start and the end of the step (queued older samples are dropped)."""
        with self.lock:
            self.rows = []
        t0 = time.time()
        self.on = True
        try:
            while not until():
                time.sleep(0.02)
            t1 = time.time()
            time.sleep(0.1)                    # samples still in flight
        finally:
            self.on = False
        with self.lock:
            return [r for r in self.rows if t0 <= r[0] <= t1]


def summarize(rows):
    if len(rows) < 2:
        raise RuntimeError('no /robot/state samples during the step (robot_state running?)')
    t = np.array([r[0] for r in rows])
    P = np.array([r[1][:3, 3] for r in rows]) * 1000.0
    step = 10
    v = np.linalg.norm(P[step:] - P[:-step], axis=1) / np.maximum(t[step:] - t[:-step], 1e-6) if len(P) > step else np.zeros(1)
    moving = np.zeros(len(rows), bool)
    moving[step:] = v > MOVING_MM_S
    fd = np.array([r[2] == RobotState.MODE_FREEDRIVE and r[3] == RobotState.SOURCE_KINESTHETIC and r[4] for r in rows])
    last_fd = -np.inf
    braking = np.zeros(len(rows), bool)
    for k in range(len(rows)):
        if fd[k]:
            last_fd = t[k]
        braking[k] = t[k] - last_fd <= BRAKE_GRACE_S
    return {'n': len(rows), 'delta': mc.pose_delta(rows[0][1], rows[-1][1]),
            'path_mm': float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))),
            'max_mm_s': float(v.max()), 'fd_share': float(fd.mean()),
            'moving_not_fd': int(np.sum(moving & ~fd & ~braking)), 'moving': int(moving.sum()),
            'spread_mm': float(np.max(np.linalg.norm(P - P[0], axis=1)))}


def step(buf, ask, prompt, seconds=None, stop_prompt=None):
    """Enter starts; then either a fixed time, or until Enter again (stop_prompt)."""
    r = ask(prompt).strip().lower()
    if r == 's':
        return None
    if seconds is not None:
        t_end = time.monotonic() + seconds
        return summarize(buf.record(lambda: time.monotonic() >= t_end))
    done = threading.Event()
    th = threading.Thread(target=lambda: (ask(stop_prompt), done.set()), daemon=True)
    th.start()
    return summarize(buf.record(done.is_set))


def show(name, s):
    print(f'\n--- {name}: {s["n"]} samples, freedrive-labelled {s["fd_share"] * 100:.0f} %, '
          f'path {s["path_mm"]:.1f} mm, max speed {s["max_mm_s"]:.1f} mm/s\n{mc.fmt_delta(s["delta"])}')


def main(argv=None, ask=input, node=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--still-s', type=float, default=2.0)
    ap.add_argument('--drift-s', type=float, default=3.0)
    a, _ = ap.parse_known_args(argv)

    def body(n):
        if config.get_path(n.rig, 'freedrive.input') is None:
            raise RuntimeError('rig.yaml freedrive.input is null: set the pedal input (FREEDRIVE_COLLECTION_GUIDE.md)')
        buf = Buffer(n)
        n.state()
        time.sleep(0.5)
        if buf.ctl_t is not None and time.monotonic() - buf.ctl_t < 1.0:
            raise RuntimeError('robot_control is running: stop it (control:=false) for freedrive')
        vmax = config.get_path(n.rig, 'limits.max_linear_speed_mm_s')
        rows = []
        s1 = step(buf, ask, '\n1/4  Pedal RELEASED, hands off the arm. Enter to record '
                            f'{a.still_s:g} s: ', a.still_s)
        show('1 released', s1)
        rows += [('1 released: labelled IDLE', s1['fd_share'] == 0.0, f'{s1["fd_share"] * 100:.0f} % freedrive-labelled'),
                 ('1 released: arm still', s1['spread_mm'] <= STILL_MM and s1['delta']['angle_deg'] <= STILL_DEG,
                  f'{s1["spread_mm"]:.2f} mm, {s1["delta"]["angle_deg"]:.2f} deg')]
        s2 = step(buf, ask, '\n2/4  Enter, then HOLD the pedal, move the probe ~5 cm and back by the handle, '
                            'release the pedal: ', None, '     ... press Enter when you have released the pedal: ')
        show('2 guided', s2)
        rows += [('2 guided: moved by hand', s2['path_mm'] >= 10.0, f'path {s2["path_mm"]:.1f} mm (>= 10)'),
                 ('2 guided: labelled kinesthetic', s2['fd_share'] > 0.0, f'{s2["fd_share"] * 100:.0f} % of samples'),
                 ('2 guided: no unlabelled motion', s2['moving_not_fd'] == 0,
                  f'{s2["moving_not_fd"]} of {s2["moving"]} moving samples not labelled kinesthetic '
                  '(pedal input / freedrive.input mismatch if > 0)')]
        if vmax is not None:
            rows.append(('2 guided: hand speed', s2['max_mm_s'] <= float(vmax),
                         f'max {s2["max_mm_s"]:.1f} mm/s (limit {float(vmax):g})'))
        s3 = step(buf, ask, f'\n3/4  Pedal RELEASED, hands off. Enter to record {a.still_s:g} s: ', a.still_s)
        show('3 released again', s3)
        rows += [('3 released: locked, IDLE', s3['fd_share'] == 0.0 and s3['spread_mm'] <= STILL_MM,
                  f'{s3["fd_share"] * 100:.0f} % freedrive-labelled, moved {s3["spread_mm"]:.2f} mm')]
        s4 = step(buf, ask, f'\n4/4  HOLD the pedal, let go of the handle (hand next to it). Enter to record '
                            f'{a.drift_s:g} s (s = skip): ', a.drift_s)
        if s4 is None:
            rows.append(('4 gravity compensation', True, 'skipped by the operator'))
        else:
            show('4 pedal held, hands off', s4)
            rows += [('4 gravity compensation (payload)', s4['spread_mm'] <= DRIFT_MM,
                      f'drift {s4["spread_mm"]:.2f} mm in {a.drift_s:g} s (limit {DRIFT_MM}); larger = payload / CoG wrong'),
                     ('4 labelled kinesthetic', s4['fd_share'] > 0.9, f'{s4["fd_share"] * 100:.0f} %')]
        return mc.print_table('freedrive_check', rows)

    if node is not None:
        return body(node)
    from usrig_common.tool_node import run_tool
    out = {}
    run_tool(lambda n: out.update(ok=body(n)), 'freedrive_check')
    raise SystemExit(0 if out.get('ok') else 1)


if __name__ == '__main__':
    main()
