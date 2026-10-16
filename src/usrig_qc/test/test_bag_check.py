"""bag_check on synthetic test-1 bags: the robot data must be READ (pose, joints, F/T), not only present,
and the derived topics must carry the same sample as /robot/state."""
from pathlib import Path

import numpy as np
import yaml

from usrig_qc.bag_check import check
from usrig_qc.episode_qc import _typestore

REPO = Path(__file__).resolve().parents[3]
SCFG = yaml.safe_load((REPO / 'src/usrig_bringup/config/session.yaml').read_text())
RIG = yaml.safe_load((REPO / 'src/usrig_bringup/config/rig.yaml').read_text())
T0 = 1_760_000_000_000_000_000
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def write_bag(path, n=1000, wrench=None, q=None, pose_offset_m=0.0, js_offset=0.0, origin=False, nan_at=None):
    """Robot topics as robot_state_cpp publishes them (500 Hz, one stamp per sample), arm moving 100 mm in x."""
    from rosbags.rosbag2 import StoragePlugin, Writer
    ts = _typestore()
    T = ts.types
    Header, Time = T['std_msgs/msg/Header'], T['builtin_interfaces/msg/Time']
    Pose, Point, Quat = T['geometry_msgs/msg/Pose'], T['geometry_msgs/msg/Point'], T['geometry_msgs/msg/Quaternion']
    PoseSt, WrSt, Wr, V3 = (T['geometry_msgs/msg/PoseStamped'], T['geometry_msgs/msg/WrenchStamped'],
                            T['geometry_msgs/msg/Wrench'], T['geometry_msgs/msg/Vector3'])
    JS, RS = T['sensor_msgs/msg/JointState'], T['usrig_interfaces/msg/RobotState']
    rng = np.random.default_rng(0)
    msgs = []
    for k in range(n):
        ns = T0 + k * 2_000_000
        h = Header(stamp=Time(sec=ns // 10**9, nanosec=ns % 10**9), frame_id='base')
        x = 0.0 if origin else 0.4 + 0.1 * k / n
        qk = np.array([0.1, -1.2, 1.5, -1.9, -1.57, 0.0]) + 0.001 * k / n if q is None else np.array(q, float)
        wk = np.array([0.3, -0.2, -5.0, 0.01, 0.02, 0.0]) + rng.normal(0, 0.05, 6) if wrench is None \
            else np.array(wrench, float)
        if nan_at == k:
            wk = wk.copy()
            wk[2] = np.nan
        ori = Quat(x=1.0, y=0.0, z=0.0, w=0.0)
        pose = Pose(position=Point(x=x, y=-0.3 if not origin else 0.0, z=0.25 if not origin else 0.0), orientation=ori)
        msgs.append(('/robot/state', ns, RS(header=h, controller_time_s=k * 0.002, q=qk, qd=np.zeros(6),
                                            tcp_base=pose, tcp_phantom=pose, wrench=wk, force_setpoint_n=np.nan,
                                            mode=0, safety_flags=0, action_source=4, action_is_expert=False)))
        p2 = Pose(position=Point(x=x + pose_offset_m, y=pose.position.y, z=pose.position.z), orientation=ori)
        msgs.append(('/robot/tcp_pose', ns, PoseSt(header=h, pose=p2)))
        msgs.append(('/robot/wrench', ns, WrSt(header=h, wrench=Wr(force=V3(x=wk[0], y=wk[1], z=wk[2]),
                                                                   torque=V3(x=wk[3], y=wk[4], z=wk[5])))))
        msgs.append(('/joint_states', ns, JS(header=h, name=JOINTS, position=qk + js_offset, velocity=np.zeros(6),
                                             effort=np.zeros(0))))
    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {}
        for topic, ns, m in msgs:
            if topic not in conns:
                conns[topic] = w.add_connection(topic, m.__msgtype__, typestore=ts)
            w.write(conns[topic], ns, ts.serialize_cdr(m, m.__msgtype__))


def robot_checks(tmp_path, **kw):
    bag = tmp_path / 'bag'
    tmp_path.mkdir(parents=True, exist_ok=True)
    write_bag(bag, **kw)
    rows, checks, summary = check(bag, ['robot'], SCFG, RIG)
    return {name: (ok, detail) for name, ok, detail in checks}, summary


def failed(c):
    return sorted(k for k, (ok, _) in c.items() if not ok)


def test_good_robot_bag_passes_and_summarises_motion_and_force(tmp_path):
    c, s = robot_checks(tmp_path)
    assert failed(c) == []
    assert 95 < s['moved_mm'] < 105 and 4.5 < s['force_n']['median'] < 5.5


def test_all_zero_wrench_means_the_ft_reading_is_not_captured(tmp_path):
    c, _ = robot_checks(tmp_path, wrench=[0, 0, 0, 0, 0, 0])
    assert failed(c) == ['robot: force / torque read (F/T live)']


def test_constant_wrench_is_not_a_live_sensor(tmp_path):
    c, _ = robot_checks(tmp_path, wrench=[1, 2, 3, 0, 0, 0])
    assert failed(c) == ['robot: force / torque read (F/T live)']


def test_joints_all_zero_are_not_read(tmp_path):
    c, _ = robot_checks(tmp_path, q=[0, 0, 0, 0, 0, 0])
    assert failed(c) == ['robot: joint angles read']


def test_tcp_at_the_base_origin_is_not_a_real_pose(tmp_path):
    c, _ = robot_checks(tmp_path, origin=True)
    assert 'robot: TCP pose read (not at the base origin)' in failed(c)


def test_derived_topics_must_carry_the_same_sample_as_robot_state(tmp_path):
    c, _ = robot_checks(tmp_path, pose_offset_m=0.01)
    assert failed(c) == ['robot: /robot/tcp_pose = /robot/state']
    c, _ = robot_checks(tmp_path / 'b', js_offset=0.01)
    assert failed(c) == ['robot: /joint_states = /robot/state']


def test_nan_in_a_sample_fails(tmp_path):
    c, _ = robot_checks(tmp_path, nan_at=500)
    assert 'robot: values finite (q, qd, TCP pose, wrench)' in failed(c)
