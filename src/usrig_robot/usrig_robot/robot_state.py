"""robot_state: UR7e state at 500 Hz over read-only RTDE.

Publishes /robot/state, /joint_states, /robot/tcp_pose, /robot/wrench, all with the same stamp
per RTDE sample. No forward kinematics: the TCP pose is the controller's actual_TCP_pose.
RTDE receive is read-only, so this can run next to whatever program is on the robot.
"""
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from usrig_common import config, geometry
from usrig_common.session import read_json
from usrig_common.timeutil import stamp_from_ns
from usrig_interfaces.msg import RobotState

from .rtde_reader import RtdeReader

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class ControlStatus:
    """Fields of RobotState owned by robot_control. Defaults describe a robot nobody commands."""

    def __init__(self):
        self.lock = threading.Lock()
        self.force_setpoint_n = 0.0
        self.mode = RobotState.MODE_IDLE
        self.safety_flags = 0
        self.action_source = RobotState.SOURCE_NONE
        self.action_is_expert = False

    def snapshot(self):
        with self.lock:
            return (self.force_setpoint_n, self.mode, self.safety_flags,
                    self.action_source, self.action_is_expert)


class RobotStateNode(Node):

    def __init__(self, control_status=None, **kwargs):
        super().__init__('robot_state', **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot_ip', '')
        self.declare_parameter('frequency_hz', 500.0)
        self.declare_parameter('session_json', '')
        self.declare_parameter('base_frame', 'base')

        cfg_dir = self.get_parameter('config_dir').value or None
        rig = config.load_config('rig', cfg_dir)
        ip = self.get_parameter('robot_ip').value or config.get_path(rig, 'robot.ip')
        if not ip:
            raise RuntimeError('robot.ip is null in rig.yaml and no robot_ip parameter was given')
        self.base_frame = self.get_parameter('base_frame').value
        self.status = control_status or ControlStatus()

        # Phantom registration (identity until P5 writes it into session.json).
        self.T_phantom_base = np.eye(4)
        sj = self.get_parameter('session_json').value
        if sj:
            reg = read_json(sj).get('phantom_registration', {})
            if 'T_base_phantom' in reg:
                self.T_phantom_base = geometry.inv(np.array(reg['T_base_phantom']))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_state = self.create_publisher(RobotState, '/robot/state', qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/robot/tcp_pose', qos)
        self.pub_wrench = self.create_publisher(WrenchStamped, '/robot/wrench', qos)

        self.latest = None
        self.latest_lock = threading.Lock()
        self.listeners = []          # callables(sample) run in the RTDE thread (robot_control)

        self.reader = RtdeReader(ip, self.get_parameter('frequency_hz').value, self._on_sample)
        self.reader.start()
        self.create_timer(1.0, self._log_stats)
        self.get_logger().info(f'RTDE receive connected to {ip}')

    def _on_sample(self, s):
        with self.latest_lock:
            self.latest = s
        stamp = stamp_from_ns(s.stamp_ns)
        T = geometry.pose_from_ur(s.tcp_pose)
        fsp, mode, flags, src, expert = self.status.snapshot()

        m = RobotState()
        m.header.stamp = stamp
        m.header.frame_id = self.base_frame
        m.controller_time_s = s.controller_time_s
        m.q = s.q.tolist()
        m.qd = s.qd.tolist()
        geometry.fill_pose_msg(m.tcp_base, T)
        geometry.fill_pose_msg(m.tcp_phantom, self.T_phantom_base @ T)
        m.wrench = s.wrench.tolist()
        m.force_setpoint_n = float(fsp)
        m.mode = mode
        m.safety_flags = flags
        m.action_source = src
        m.action_is_expert = bool(expert)
        self.pub_state.publish(m)

        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = self.base_frame
        js.name = JOINT_NAMES
        js.position = s.q.tolist()
        js.velocity = s.qd.tolist()
        self.pub_js.publish(js)

        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.tcp_base
        self.pub_pose.publish(ps)

        w = WrenchStamped()
        w.header = m.header
        (w.wrench.force.x, w.wrench.force.y, w.wrench.force.z,
         w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z) = map(float, s.wrench)
        self.pub_wrench.publish(w)

        for fn in self.listeners:
            fn(s)

    def get_latest(self):
        with self.latest_lock:
            return self.latest

    def _log_stats(self):
        st = self.reader.take_stats()
        if st.stamp_minus_ctrl:
            d = np.asarray(st.stamp_minus_ctrl)
            d = (d - np.median(d)) * 1e3
            jit = f'jitter(stamp-ctrl) p50={np.median(np.abs(d)):.3f} ms p99={np.percentile(np.abs(d), 99):.3f} ms'
        else:
            jit = 'no samples'
        self.get_logger().info(
            f'RTDE {st.samples} Hz, repeated ctrl time {st.repeated_controller_time}, '
            f'gaps {st.controller_gaps}, {jit}')

    def destroy_node(self):
        self.reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotStateNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[robot_state] {e}', file=sys.stderr)
        sys.exit(2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
