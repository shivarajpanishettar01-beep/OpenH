"""robot_control against a kinematic fake robot (no URSim, no hardware).

The fake backend integrates the commanded twist into the TCP pose; a fake F/T reports a spring force
when the probe tip is below a phantom surface; in force mode it regulates penetration to the setpoint.
Limits here are TEST values for this simulator only (rig.yaml keeps its nulls).
"""
import json
import sys
import threading
import time

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

import usrig_robot.robot_control as rc
from usrig_common import geometry
from usrig_robot.rtde_reader import RobotSample

S = 1e-9
RIG = {
    'robot': {'ip': 'fake', 'control_backend': 'rtde', 'control_rate_hz': 125.0, 'tcp_setting': [0, 0, 0, 0, 0, 0],
              'payload_kg': 0.5, 'payload_cog_m': [0, 0, 0.05], 'max_state_age_s': 0.05, 'rt_priority': 0},
    'probe': {'tcp_already_includes_probe': True},
    'limits': {'max_contact_force_n': 20.0, 'force_band_n': [2.0, 10.0], 'force_setpoint_default_n': 4.0,
               'tilt_cone_deg': 30.0, 'tilt_reference_axis_base': [0, 0, -1],
               'workspace_box_base_m': {'min': [0.2, -0.3, 0.0], 'max': [0.6, 0.3, 0.4]},
               'phantom_box_m': {'pmin': [-0.1, -0.1, 0.0], 'pmax': [0.1, 0.1, 0.2]}, 'local_tilt_cone_deg': 10.0,
               'max_linear_speed_mm_s': 50.0, 'max_angular_speed_deg_s': 30.0,
               'max_linear_accel_mm_s2': 500.0, 'max_angular_accel_deg_s2': 300.0, 'stop_decel_mm_s2': 1000.0,
               'deadman_timeout_s': 0.2, 'contact_detect_threshold_n': 1.0,
               'contact_lost_threshold_n': 0.5, 'contact_lost_time_s': 0.1},
    'motion': {'transit_speed_mm_s': 40.0, 'descent_speed_mm_s': 10.0, 'descent_max_travel_mm': 60.0,
               'retract_distance_mm': 30.0, 'approach_offset_mm': 50, 'transit_safe_height_mm': 80.0, 'position_gain_per_s': 4.0,
               'rotation_gain_per_s': 4.0},
    'rtde': {'force_mode_limits': [0.1] * 6},
}
DOWN = np.diag([1.0, -1.0, -1.0])        # probe z (beam) pointing down: probe x = base x, probe y = base -y


class FakeRobot:
    """Both the RTDE state source and the control backend."""
    name = 'fake'

    def __init__(self, p0, surface_z=None, k=1000.0):
        self.T = np.eye(4)
        self.T[:3, :3] = DOWN
        self.T[:3, 3] = p0
        self.v = np.zeros(3)
        self.w = np.zeros(3)
        self.force_on, self.fsp = False, 0.0
        self.surface_z, self.k = surface_z, k
        self.extra_force = np.zeros(3)
        self.lock = threading.Lock()
        self.t_last = time.monotonic()
        self.robot_ip = 'fake'
        self.protective = False

    # ---- state side
    def _integrate(self):
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now
        v = self.v.copy()
        if self.force_on:
            # force mode along the probe axis: move so the spring force approaches the setpoint
            # (with no surface the measured force stays 0, so it keeps moving: like the real controller)
            z = self.T[:3, 2]
            v = v - z * (v @ z) + z * np.clip((self.fsp - self._f_axis()) * 0.002, -0.02, 0.02)
        self.T[:3, 3] += v * dt
        self.T[:3, :3] = Rotation.from_rotvec(self.w * dt).as_matrix() @ self.T[:3, :3]

    def _force_base(self):
        f = self.extra_force.copy()
        if self.surface_z is not None and self.T[2, 3] < self.surface_z:
            f[2] += self.k * (self.surface_z - self.T[2, 3])      # phantom pushes up
        return f

    def _f_axis(self):
        return abs(float(self.T[:3, 2] @ self._force_base()))

    def get_latest(self):
        with self.lock:
            self._integrate()
            return RobotSample(stamp_ns=time.time_ns(), controller_time_s=time.monotonic(),
                               q=np.zeros(6), qd=np.zeros(6), tcp_pose=np.array(geometry.ur_from_pose(self.T)),
                               tcp_speed=np.concatenate([self.v, self.w]),
                               wrench=np.concatenate([self._force_base(), np.zeros(3)]))

    # ---- backend side
    def connect(self):
        pass

    def close(self):
        pass

    def kick(self):
        pass

    def command(self, T_tcp, v_base, w_base, accel, force_on, fsp, dt):
        with self.lock:
            self._integrate()
            self.v, self.w = np.asarray(v_base, float), np.asarray(w_base, float)
            self.force_on, self.fsp = force_on, fsp

    def stop(self):
        with self.lock:
            self._integrate()
            self.v, self.w = np.zeros(3), np.zeros(3)
            self.force_on = False

    def zero_ft(self):
        pass

    def protective_stop(self):
        self.protective = True
        self.stop()


@pytest.fixture
def sim(monkeypatch):
    import rclpy
    rclpy.init()
    robots = []

    def make(p0=(0.4, 0.0, 0.2), surface_z=None):
        robot = FakeRobot(np.array(p0, float), surface_z)
        robots.append(robot)
        monkeypatch.setattr(rc, 'make_backend', lambda rig, log=print, ip=None: robot)
        status = rc.ControlStatus()
        node = rc.RobotControlNode(robot, status, sys.modules[__name__].RIG)
        node.__dict__.setdefault('events', [])
        return node, robot

    # Record events from the node's first control tick: the loop starts inside the constructor (a robot
    # placed outside the box FAULTs at once), so wrapping node._event after construction missed events.
    orig = rc.RobotControlNode._event

    def rec(self, kind, value, stamp_ns=None):
        self.__dict__.setdefault('events', []).append((kind, value))
        orig(self, kind, value, stamp_ns)
    monkeypatch.setattr(rc.RobotControlNode, '_event', rec)
    yield make
    rclpy.shutdown()


def twist(node, v=(0, 0, 0), w=(0, 0, 0)):
    from geometry_msgs.msg import TwistStamped
    m = TwistStamped()
    m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v)
    m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = map(float, w)
    node._on_twist(m)


def drive(node, seconds, v=(0, 0, 0), w=(0, 0, 0)):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        twist(node, v, w)
        time.sleep(0.01)


def cmd(node, payload):
    from std_msgs.msg import String
    node._on_command(String(data=json.dumps(payload)))


def motion_events(node):
    return [v for k, v in node.events if k == 'motion']


def test_move_to_pose_reaches_target_and_reports(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    T = robot.T.copy()
    T[:3, 3] = [0.45, 0.05, 0.25]
    geometry.fill_pose_msg(req.target.pose, T)
    res = node._srv_move(req, MoveToPose.Response())
    assert res.success, res.message
    assert np.linalg.norm(robot.T[:3, 3] - [0.45, 0.05, 0.25]) < 0.001
    assert motion_events(node)[-1]['ok'] and motion_events(node)[-1]['motion'] == 'LinearMove'
    node.destroy_node()


def test_target_outside_envelope_refused_before_moving(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    p0 = robot.T[:3, 3].copy()
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    T = robot.T.copy()
    T[:3, 3] = [0.45, 0.0, 0.48]          # above the box top (0.4)
    geometry.fill_pose_msg(req.target.pose, T)
    req.speed_mm_s = 40.0
    t0 = time.monotonic()
    res = node._srv_move(req, MoveToPose.Response())
    assert not res.success and 'refused before moving' in res.message and 'envelope' in res.message
    assert time.monotonic() - t0 < 0.5
    time.sleep(0.2)
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4
    T[:3, 3] = [0.45, 0.0, 0.2]
    T[:3, :3] = Rotation.from_euler('x', 45, degrees=True).as_matrix() @ DOWN    # outside the 30 deg cone
    geometry.fill_pose_msg(req.target.pose, T)
    res = node._srv_move(req, MoveToPose.Response())
    assert not res.success and 'tilt_cone' in res.message
    node.destroy_node()


def test_stalled_move_times_out_instead_of_hanging(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    robot.command = lambda *a, **k: None            # the arm does not follow (e.g. a stuck controller)
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    T = robot.T.copy()
    T[:3, 3] = [0.45, 0.0, 0.25]
    geometry.fill_pose_msg(req.target.pose, T)
    req.speed_mm_s = 40.0
    node._move_timeout = lambda T, speed: 1.0       # keep the test short
    t0 = time.monotonic()
    res = node._srv_move(req, MoveToPose.Response())
    assert not res.success and res.message == 'timeout' and time.monotonic() - t0 < 2.0
    assert motion_events(node)[-1] == {'motion': 'LinearMove', 'ok': False, 'message': 'timeout',
                                       'duration_s': motion_events(node)[-1]['duration_s']}
    node.destroy_node()


def test_run_path_outside_envelope_refused_before_moving(sim, monkeypatch):
    from usrig_interfaces.srv import RunPath
    node, robot = sim(surface_z=0.19)
    monkeypatch.setattr(rc.RobotControlNode, '_phantom_T', lambda self: np.eye(4))
    node.in_contact = True
    req = RunPath.Request()
    for x in (0.4, 0.5, 0.7):                       # last waypoint beyond the box (x max 0.6)
        T = robot.T.copy()
        T[:3, 3] = [x, 0.0, 0.19]
        req.waypoints.append(geometry.fill_pose_msg(PoseStamped().pose, T))
    req.speed_mm_s = 10.0
    res = node._srv_run_path(req, RunPath.Response())
    assert not res.success and 'refused before moving' in res.message
    assert not any(k == 'phase' and v.get('phase') == 'search' for k, v in node.events)
    assert motion_events(node)[-1]['motion'] == 'PathFollow' and not motion_events(node)[-1]['ok']
    node.destroy_node()


def test_teleop_probe_frame_directions_and_deadman(sim):
    node, robot = sim()
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    p0 = robot.T[:3, 3].copy()
    drive(node, 0.6, v=(0.02, 0, 0))                 # probe +x = base +x when pointing down
    d = robot.T[:3, 3] - p0
    assert d[0] > 0.005 and abs(d[1]) < 0.001
    p1 = robot.T[:3, 3].copy()
    drive(node, 0.6, v=(0, 0.02, 0))                 # probe +y = base -y when pointing down
    d = robot.T[:3, 3] - p1
    assert d[1] < -0.005 and abs(d[0]) < 0.002
    p2 = robot.T[:3, 3].copy()
    drive(node, 0.6, v=(0.02, 0.02, 0))              # diagonal
    d = robot.T[:3, 3] - p2
    assert d[0] > 0.004 and d[1] < -0.004
    # deadman: stop sending twists; the robot must stop within the timeout (+ decel)
    time.sleep(0.2 + 0.15)
    p3 = robot.T[:3, 3].copy()
    time.sleep(0.3)
    assert np.linalg.norm(robot.T[:3, 3] - p3) < 1e-4
    assert node.status.safety_flags & rc.safety.FLAG_DEADMAN
    node.destroy_node()


def test_hard_envelope_fault_ack_and_recover_inward_only(sim):
    node, robot = sim(p0=(0.65, 0.0, 0.2))          # outside the box (x max 0.6)
    time.sleep(0.1)
    assert node.status.mode == rc.ST.MODE_FAULT
    assert any(k == 'phase' and v.get('phase') == 'fault' for k, v in node.events)
    drive(node, 0.3, v=(0.02, 0, 0))                 # nothing moves while in FAULT
    assert robot.T[0, 3] == pytest.approx(0.65, abs=1e-6)
    cmd(node, {'cmd': 'ack_fault'})
    assert node.status.mode != rc.ST.MODE_FAULT
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    drive(node, 0.4, v=(0.02, 0, 0))                 # further out: blocked
    assert robot.T[0, 3] <= 0.65 + 1e-4
    drive(node, 1.0, v=(-0.02, 0, 0))                # back inside: allowed
    assert robot.T[0, 3] < 0.64
    node.destroy_node()


def test_force_ceiling_faults(sim):
    node, robot = sim()
    time.sleep(0.1)
    robot.extra_force = np.array([0.0, 0.0, 25.0])   # above max_contact_force_n = 20
    time.sleep(0.1)
    assert node.status.mode == rc.ST.MODE_FAULT
    assert node.status.safety_flags & rc.safety.FLAG_FORCE_MAX
    cmd(node, {'cmd': 'ack_fault'})                  # refused while still violated
    assert node.status.mode == rc.ST.MODE_FAULT
    robot.extra_force[:] = 0
    cmd(node, {'cmd': 'ack_fault'})
    assert node.status.mode != rc.ST.MODE_FAULT
    node.destroy_node()


def test_descend_then_path_then_cancel(sim):
    from geometry_msgs.msg import Pose
    from std_srvs.srv import Trigger
    from usrig_interfaces.srv import DescendToContact, RunPath
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    res = node._srv_descend(DescendToContact.Request(), DescendToContact.Response())
    assert res.success, res.message
    assert node.in_contact and abs(robot.T[2, 3] - 0.03) < 0.003
    # constant-speed path in the phantom frame (= base here: registration is identity)
    node._phantom_T = lambda: np.eye(4)
    req = RunPath.Request()
    for x in (0.40, 0.46):
        T = robot.T.copy()
        T[:3, 3] = [x, 0.0, robot.T[2, 3]]
        req.waypoints.append(geometry.fill_pose_msg(Pose(), T))
    req.speed_mm_s = 15.0
    res = node._srv_run_path(req, RunPath.Response())
    assert res.success, res.message
    assert abs(robot.T[0, 3] - 0.46) < 0.003
    assert res.speed_error_mean_pct < 10.0, res.speed_error_mean_pct
    # cancel a path: retract while it runs -> run_path reports not ok ('preempted'), events recorded
    req2 = RunPath.Request()
    req2.waypoints = list(reversed(req.waypoints))
    req2.speed_mm_s = 10.0
    out = {}
    th = threading.Thread(target=lambda: out.update(r=node._srv_run_path(req2, RunPath.Response())))
    th.start()
    time.sleep(1.0)
    r_ret = node._srv_retract(Trigger.Request(), Trigger.Response())
    th.join(10)
    assert not out['r'].success and out['r'].message == 'preempted'
    assert r_ret.success
    ev = motion_events(node)
    assert {'PathFollow', 'LinearMove', 'Descent'} <= {e['motion'] for e in ev}
    assert any(e['motion'] == 'PathFollow' and not e['ok'] and e['message'] == 'preempted' for e in ev)
    node.destroy_node()


# ---- Known gaps from the 13 Oct control review (docs/control_review.md). strict xfail: when a gap is
# fixed these start passing, the suite fails, and the marker must be removed.

def test_lift_perturbation_stays_in_force_band(sim):
    from usrig_interfaces.srv import DescendToContact
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    seen = []
    node._set_motion(rc.Perturbation(node, 'lift', [-10.0, 0, 0], 1.0, node.force_setpoint))
    t_end = time.monotonic() + 0.8
    while time.monotonic() < t_end:
        seen.append(node.force_setpoint)
        time.sleep(0.02)
    lo, hi = RIG['limits']['force_band_n']
    assert min(seen) >= 0.0 and max(seen) <= hi, f'force setpoint range {min(seen)}..{max(seen)} N'
    assert min(seen) == 0.0                      # the lift reached zero force (lift-off staged)
    node.destroy_node()


def test_contact_loss_is_detected(sim):
    from usrig_interfaces.srv import DescendToContact
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    node._set_motion(rc.Teleop(node, True))
    z0 = robot.T[2, 3]
    robot.surface_z = None                       # phantom gone / probe slid off the edge
    t_end = time.monotonic() + 1.5
    while time.monotonic() < t_end:
        twist(node)                              # operator present, no keys pressed
        time.sleep(0.01)
    events = [v for k, v in node.events if k == 'phase' and v.get('phase') == 'contact_lost']
    assert events and z0 - robot.T[2, 3] < 0.005, f'moved {1000 * (z0 - robot.T[2, 3]):.1f} mm down, events {events}'
    assert not node.in_contact and isinstance(node.motion, rc.Idle)
    # operator re-establishes contact
    robot.surface_z = robot.T[2, 3] - 0.004              # phantom back, 4 mm below the probe
    cmd(node, {'cmd': 'recontact'})
    t_end = time.monotonic() + 5
    while not node.in_contact and time.monotonic() < t_end:
        time.sleep(0.05)
    assert node.in_contact and any(k == 'phase' and v.get('phase') == 'recontact' for k, v in node.events)
    node.destroy_node()


def test_operator_pause_independent_of_service_hold(sim):
    node, robot = sim()
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    cmd(node, {'cmd': 'hold'})                       # operator presses P
    p0 = robot.T[:3, 3].copy()
    drive(node, 0.4, v=(0.02, 0, 0))
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4
    from std_srvs.srv import SetBool
    node._srv_teleop(SetBool.Request(data=True), SetBool.Response())   # service releases its hold only
    drive(node, 0.4, v=(0.02, 0, 0))
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4    # still paused by the operator
    cmd(node, {'cmd': 'hold'})                       # operator resumes
    drive(node, 0.4, v=(0.02, 0, 0))
    assert robot.T[0, 3] - p0[0] > 0.003
    node.destroy_node()


def test_path_aborts_on_contact_loss(sim):
    from geometry_msgs.msg import Pose
    from usrig_interfaces.srv import DescendToContact, RunPath
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    node._phantom_T = lambda: np.eye(4)
    req = RunPath.Request()
    for x in (0.40, 0.50):
        T = robot.T.copy()
        T[:3, 3] = [x, 0.0, robot.T[2, 3]]
        req.waypoints.append(geometry.fill_pose_msg(Pose(), T))
    req.speed_mm_s = 15.0
    out = {}
    th = threading.Thread(target=lambda: out.update(r=node._srv_run_path(req, RunPath.Response())))
    th.start()
    time.sleep(1.0)
    robot.surface_z = None                               # probe slides off the phantom edge
    th.join(10)
    assert not out['r'].success and out['r'].message == 'contact lost'
    node.destroy_node()


def test_design_requires_tcp_at_probe_face():
    assert rc.design_requirements(RIG) == []
    bad = json.loads(json.dumps(RIG))
    bad['probe']['tcp_already_includes_probe'] = False
    assert 'probe face' in rc.design_requirements(bad)[0]


def test_coordinated_move_finishes_position_and_rotation_together(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    T = robot.T.copy()
    T[:3, 3] += [0.06, 0.0, 0.0]
    T[:3, :3] = Rotation.from_euler('z', 40, degrees=True).as_matrix() @ T[:3, :3]
    fr = []
    p0, R0 = robot.T[:3, 3].copy(), robot.T[:3, :3].copy()
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    geometry.fill_pose_msg(req.target.pose, T)
    th = threading.Thread(target=lambda: node._srv_move(req, MoveToPose.Response()))
    th.start()
    while th.is_alive():
        pos = np.linalg.norm(robot.T[:3, 3] - p0) / 0.06
        rot = np.degrees(Rotation.from_matrix(robot.T[:3, :3] @ R0.T).magnitude()) / 40
        fr.append((pos, rot))
        time.sleep(0.05)
    mid = [abs(p - r) for p, r in fr if 0.2 < p < 0.8]
    assert mid and max(mid) < 0.15, f'position and rotation progress diverge by {max(mid):.2f}'
    node.destroy_node()


def test_phantom_moves_go_via_safe_height(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim(p0=(0.30, 0.0, 0.03))            # near the phantom surface (phantom frame = base)
    node._phantom_T = lambda: np.eye(4)
    T = robot.T.copy()
    T[:3, 3] = [0.45, 0.10, 0.04]                      # another target near the surface
    req = MoveToPose.Request()
    req.target.header.frame_id = 'phantom'
    geometry.fill_pose_msg(req.target.pose, T)
    zs = []
    th = threading.Thread(target=lambda: zs.append(node._srv_move(req, MoveToPose.Response())))
    th.start()
    traj = []
    while th.is_alive():
        traj.append(robot.T[:3, 3].copy())
        time.sleep(0.03)
    assert zs[0].success, zs[0].message
    traj = np.array(traj)
    crossing = traj[(traj[:, 0] > 0.33) & (traj[:, 0] < 0.42)]   # the lateral part of the move
    assert len(crossing) and crossing[:, 2].min() > 0.08 - 0.003, crossing[:, 2].min()
    assert np.linalg.norm(robot.T[:3, 3] - [0.45, 0.10, 0.04]) < 0.001
    node.destroy_node()


def test_phantom_move_refuses_without_safe_height(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    node._phantom_T = lambda: np.eye(4)
    node.rig = json.loads(json.dumps(RIG))
    node.rig['motion']['transit_safe_height_mm'] = None
    req = MoveToPose.Request()
    req.target.header.frame_id = 'phantom'
    geometry.fill_pose_msg(req.target.pose, robot.T)
    res = node._srv_move(req, MoveToPose.Response())
    assert not res.success and 'transit_safe_height_mm' in res.message
    node.destroy_node()


def test_phantom_box_and_local_tilt_cone(sim, monkeypatch):
    # registration: phantom origin at (0.4, 0, 0) in base, axes aligned
    T_bp = np.eye(4)
    T_bp[:3, 3] = [0.4, 0.0, 0.0]
    monkeypatch.setattr(rc.RobotControlNode, '_phantom_T', lambda self: T_bp)
    node, robot = sim(p0=(0.45, 0.0, 0.10))
    assert node.lim.extra_boxes, 'phantom box not active'
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    flags = []
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:                  # towards phantom x max (0.4 + 0.1 = 0.5 base)
        twist(node, v=(0.03, 0, 0))
        flags.append(node.status.safety_flags)
        time.sleep(0.01)
    assert robot.T[0, 3] <= 0.5 + 0.004 and robot.T[0, 3] > 0.48
    assert any(f & rc.safety.FLAG_ENVELOPE for f in flags)
    # D10: local cone 10 deg around straight down; tilting further is refused, rotation about the axis is not
    cmd(node, {'cmd': 'set_local_tilt_axis', 'axis_base': [0, 0, -1]})
    drive(node, 2.0, w=(0.2, 0, 0))                  # tilt about probe x
    tilt = geometry.tilt_deg(robot.T, [0, 0, -1])
    assert tilt <= 10.0 + 1.0, tilt
    cmd(node, {'cmd': 'clear_local_tilt_axis'})
    drive(node, 1.0, w=(0.2, 0, 0))
    assert geometry.tilt_deg(robot.T, [0, 0, -1]) > tilt + 3      # global 30 deg cone allows more
    node.destroy_node()


def test_registration_without_phantom_box_refuses_motion(sim, monkeypatch):
    monkeypatch.setattr(rc.RobotControlNode, '_phantom_T', lambda self: np.eye(4))
    rig = json.loads(json.dumps(RIG))
    rig['limits']['phantom_box_m'] = {'pmin': None, 'pmax': None}
    monkeypatch.setattr(sys.modules[__name__], 'RIG', rig)
    node, robot = sim()
    m = node._set_motion(rc.Teleop(node, False))
    assert m.done.is_set() and 'phantom_box_m' in m.message
    node.destroy_node()


def test_two_twist_publishers_refuse_teleop(sim):
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import SetBool
    node, robot = sim()
    import rclpy
    a, b = rclpy.create_node('kbd_fake'), rclpy.create_node('sm_fake')
    a.create_publisher(TwistStamped, '/teleop/twist', 10)
    b.create_publisher(TwistStamped, '/teleop/twist', 10)
    t_end = time.monotonic() + 3
    while node.count_publishers('/teleop/twist') < 2 and time.monotonic() < t_end:
        time.sleep(0.05)
    res = node._srv_teleop(SetBool.Request(data=True), SetBool.Response())
    assert not res.success and 'publish /teleop/twist' in res.message
    a.destroy_node()
    b.destroy_node()
    node.destroy_node()


def test_tick_stats_counts_late_ticks():
    st = rc.TickStats(0.008, 0.008)
    for t in (0.0, 0.008, 0.016, 0.040, 0.048):
        st.add(t)
    sm = st.summary()
    assert sm['ticks'] == 4 and sm['late'] == 1 and abs(sm['max_ms'] - 24.0) < 1e-6
    st.reset()
    assert st.summary() is None


def test_control_timing_event_while_recording(sim):
    node, robot = sim()
    node.timing_report_s = 0.3
    node.episode_id = 'ep_test'
    time.sleep(1.0)
    ev = [v for k, v in node.events if k == 'control_timing']
    assert ev and ev[-1]['ticks'] > 20 and abs(ev[-1]['period_ms'] - 8.0) < 1e-6
    node.destroy_node()


# ---- Stage D (teleoperation) acceptance tests

def test_stale_robot_state_faults_teleop_and_ack_does_not_resume(sim):
    node, robot = sim()
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    drive(node, 0.3, v=(0.02, 0, 0))
    fresh = robot.get_latest
    import dataclasses
    robot.get_latest = lambda: dataclasses.replace(fresh(), stamp_ns=time.time_ns() - 10**9)   # 1 s old
    time.sleep(0.1)
    assert node.status.mode == rc.ST.MODE_FAULT
    assert any(k == 'phase' and v.get('reason') == 'robot state stale' for k, v in node.events)
    robot.get_latest = fresh
    p = robot.T[:3, 3].copy()
    drive(node, 0.3, v=(0.02, 0, 0))                 # input keeps arriving: nothing moves in FAULT
    assert np.linalg.norm(robot.T[:3, 3] - p) < 1e-4
    cmd(node, {'cmd': 'ack_fault'})
    assert node.status.mode != rc.ST.MODE_FAULT
    assert isinstance(node.motion, rc.Idle)          # the previous teleop motion is not restarted
    drive(node, 0.3, v=(0.02, 0, 0))
    assert np.linalg.norm(robot.T[:3, 3] - p) < 1e-4
    node.destroy_node()


def test_over_limit_request_is_clamped_and_executed_differs_from_requested(sim):
    node, robot = sim()
    node._set_motion(rc.Teleop(node, False))
    node.hold = False
    executed = []
    node.pub_cmd.publish = lambda m: executed.append(m.twist.linear.x)
    drive(node, 0.6, v=(0.2, 0, 0))                  # requested 200 mm/s, limit 50 mm/s
    assert max(executed) <= 0.050 + 1e-9 and max(executed) > 0.045
    assert node.status.safety_flags & rc.safety.FLAG_SPEED
    assert np.linalg.norm(robot.v) <= 0.050 + 1e-9
    node.destroy_node()


def test_teleop_twists_ignored_while_a_path_runs_and_set_teleop_takes_over(sim):
    from geometry_msgs.msg import Pose
    from std_srvs.srv import SetBool
    from usrig_interfaces.srv import DescendToContact, RunPath
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    node._phantom_T = lambda: np.eye(4)
    req = RunPath.Request()
    for x in (0.40, 0.44):
        T = robot.T.copy()
        T[:3, 3] = [x, 0.0, robot.T[2, 3]]
        req.waypoints.append(geometry.fill_pose_msg(Pose(), T))
    req.speed_mm_s = 10.0
    out = {}
    th = threading.Thread(target=lambda: out.update(r=node._srv_run_path(req, RunPath.Response())))
    th.start()
    drive(node, 1.0, v=(0, 0.03, 0))                 # sideways keyboard input during the path
    assert abs(robot.T[1, 3]) < 0.002                # ignored: one motion authority at a time
    r = node._srv_teleop(SetBool.Request(data=True), SetBool.Response())   # explicit operator takeover
    th.join(10)
    assert r.success and not out['r'].success and out['r'].message == 'preempted'
    assert isinstance(node.motion, rc.Teleop)
    node.destroy_node()


def test_motion_services_refused_while_faulted(sim):
    from std_srvs.srv import SetBool
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim(p0=(0.65, 0.0, 0.2))           # outside the box -> FAULT
    time.sleep(0.1)
    assert node.status.mode == rc.ST.MODE_FAULT
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    T = robot.T.copy()
    T[:3, 3] = [0.5, 0.0, 0.2]
    geometry.fill_pose_msg(req.target.pose, T)
    res = node._srv_move(req, MoveToPose.Response())
    assert not res.success and 'FAULT' in res.message
    r = node._srv_teleop(SetBool.Request(data=True), SetBool.Response())
    assert not r.success and 'FAULT' in r.message
    node.destroy_node()


def test_status_keeps_publishing_while_state_is_stale(sim):
    import dataclasses
    node, robot = sim()
    published = []
    node.pub_status.publish = lambda m: published.append(m.mode)
    fresh = robot.get_latest
    robot.get_latest = lambda: dataclasses.replace(fresh(), stamp_ns=time.time_ns() - 10**9)
    time.sleep(0.2)
    assert len(published) > 10 and set(published) == {rc.ST.MODE_IDLE}
    node.destroy_node()


# ---- 14 Oct 2026 review fixes

def test_lift_setpoint_restored_when_perturbation_is_cut_short(sim):
    """A lift that ends early (preempted, FAULT) must not leave the lifted setpoint behind: the next
    in-contact hold or recontact would otherwise press at 0 N."""
    from usrig_interfaces.srv import DescendToContact
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    base = node.force_setpoint
    # preempted by another motion
    m = node._set_motion(rc.Perturbation(node, 'lift', [-10.0, 0, 0], 5.0, base))
    time.sleep(0.05)
    assert node.force_setpoint == 0.0
    node._set_motion(rc.Teleop(node, True))
    assert m.done.is_set() and m.message == 'preempted'
    assert node.force_setpoint == base
    # ended by a FAULT
    m = node._set_motion(rc.Perturbation(node, 'lift', [-3.0, 0, 0], 5.0, base))
    time.sleep(0.05)
    assert node.force_setpoint == pytest.approx(base - 3.0)
    node._fault(0, 'test fault')
    time.sleep(0.05)
    assert m.done.is_set() and node.force_setpoint == base
    node.destroy_node()


def test_held_teleop_is_not_labelled_expert(sim):
    """In-contact hold after a descent (episode flow) and the operator's pause are not expert actions."""
    from std_srvs.srv import SetBool
    from usrig_interfaces.srv import DescendToContact
    node, robot = sim(p0=(0.40, 0.0, 0.05), surface_z=0.03)
    assert node._srv_descend(DescendToContact.Request(), DescendToContact.Response()).success
    twist(node)
    time.sleep(0.05)
    snap = node.status.snapshot()                  # (fsp, mode, flags, source, expert)
    assert snap[1] == rc.ST.MODE_TELEOP and snap[3] == rc.ST.SOURCE_NONE and snap[4] is False
    assert node._srv_teleop(SetBool.Request(data=True), SetBool.Response()).success
    for _ in range(5):
        twist(node)
        time.sleep(0.01)
    snap = node.status.snapshot()
    assert snap[3] == rc.ST.SOURCE_TELEOP and snap[4] is True
    cmd(node, {'cmd': 'hold'})                     # operator P
    time.sleep(0.05)
    assert node.status.snapshot()[4] is False
    cmd(node, {'cmd': 'hold'})
    node._srv_teleop(SetBool.Request(data=False), SetBool.Response())   # success hold
    time.sleep(0.05)
    snap = node.status.snapshot()
    assert snap[1] == rc.ST.MODE_TELEOP and snap[3] == rc.ST.SOURCE_NONE and snap[4] is False
    node.destroy_node()


def test_ack_refused_while_state_is_stale(sim):
    import dataclasses
    node, robot = sim(p0=(0.65, 0.0, 0.2))           # outside the box -> FAULT
    time.sleep(0.1)
    assert node.status.mode == rc.ST.MODE_FAULT
    robot.T[0, 3] = 0.5                              # back inside, but the state stream is stale
    fresh = robot.get_latest
    robot.get_latest = lambda: dataclasses.replace(fresh(), stamp_ns=time.time_ns() - 10**9)
    cmd(node, {'cmd': 'ack_fault'})
    assert node.status.mode == rc.ST.MODE_FAULT
    robot.get_latest = fresh
    cmd(node, {'cmd': 'ack_fault'})
    assert node.status.mode != rc.ST.MODE_FAULT
    node.destroy_node()


def test_run_path_outside_local_tilt_cone_refused_before_moving(sim, monkeypatch):
    """A fan wider than the local cone (D10) is refused, not silently clipped to the cone."""
    from usrig_interfaces.srv import RunPath
    node, robot = sim(surface_z=0.19)
    monkeypatch.setattr(rc.RobotControlNode, '_phantom_T', lambda self: np.eye(4))
    node.in_contact = True
    cmd(node, {'cmd': 'set_local_tilt_axis', 'axis_base': [0, 0, -1]})   # cone 10 deg around straight down
    req = RunPath.Request()
    for ang in (-15.0, 0.0, 15.0):                   # fan +-15 deg about probe x
        T = robot.T.copy()
        T[:3, :3] = robot.T[:3, :3] @ Rotation.from_euler('x', ang, degrees=True).as_matrix()
        req.waypoints.append(geometry.fill_pose_msg(PoseStamped().pose, T))
    req.speed_mm_s = 10.0
    res = node._srv_run_path(req, RunPath.Response())
    assert not res.success and 'refused before moving' in res.message and 'tilt_cone' in res.message
    node.destroy_node()


def test_move_clears_previous_local_tilt_cone(sim):
    from usrig_interfaces.srv import MoveToPose
    node, robot = sim()
    cmd(node, {'cmd': 'set_local_tilt_axis', 'axis_base': [0, 0, -1]})
    T = robot.T.copy()
    T[:3, :3] = Rotation.from_euler('x', 20, degrees=True).as_matrix() @ T[:3, :3]   # 20 deg: outside 10
    req = MoveToPose.Request()
    req.target.header.frame_id = 'base'
    geometry.fill_pose_msg(req.target.pose, T)
    res = node._srv_move(req, MoveToPose.Response())
    assert res.success, res.message
    assert node.lim.local_axis is None
    node.destroy_node()


def _teleop_on(node):
    from std_srvs.srv import SetBool
    assert node._srv_teleop(SetBool.Request(data=True), SetBool.Response()).success
    drive(node, 0.1, v=(0.002, 0, 0))
    snap = node.status.snapshot()
    assert snap[3] == rc.ST.SOURCE_TELEOP and snap[4] is True


def test_fault_samples_are_not_labelled_expert(sim):
    """A FAULT used to keep the last motion's labels (TELEOP / expert) for every sample of the stop."""
    node, robot = sim()
    _teleop_on(node)
    robot.extra_force = np.array([0.0, 0.0, 25.0])   # above max_contact_force_n -> FAULT
    drive(node, 0.1)
    fsp, mode, flags, src, expert = node.status.snapshot()
    assert mode == rc.ST.MODE_FAULT
    assert src == rc.ST.SOURCE_NONE and expert is False and fsp == 0.0
    node.destroy_node()


def test_cmd_executed_keeps_coming_during_fault(sim):
    """No /robot/cmd_executed during a FAULT made a converter hold the last teleop velocity as the action."""
    node, robot = sim()
    _teleop_on(node)
    sent = []
    orig = node.pub_cmd.publish
    node.pub_cmd.publish = lambda m: (sent.append(m), orig(m))
    robot.extra_force = np.array([0.0, 0.0, 25.0])
    time.sleep(0.05)
    assert node.status.mode == rc.ST.MODE_FAULT
    n0 = len(sent)
    time.sleep(0.2)
    during = sent[n0:]
    assert len(during) >= 10
    assert all(m.twist.linear.x == m.twist.linear.y == m.twist.linear.z == 0.0 for m in during)
    node.destroy_node()


def test_deadman_zero_is_not_an_expert_action(sim):
    """Lost teleop input (no twist within the deadman time): the zero command is not the operator's action."""
    node, robot = sim()
    _teleop_on(node)
    time.sleep(0.4)                                    # deadman_timeout_s = 0.2 in this RIG
    fsp, mode, flags, src, expert = node.status.snapshot()
    assert flags & rc.safety.FLAG_DEADMAN
    assert src == rc.ST.SOURCE_NONE and expert is False
    node.destroy_node()
