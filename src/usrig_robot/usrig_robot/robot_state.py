"""robot_state: UR7e state at 500 Hz over read-only RTDE.

Publishes /robot/state, /joint_states, /robot/tcp_pose, /robot/wrench, all with the same stamp
per RTDE sample. Labels (mode, source, expert, force setpoint) follow labels.py: robot_control's
status when embedded in robot_control, else freedrive detection, else IDLE; standalone it also
publishes the NaN "no command" marker on /robot/cmd_executed (as robot_state_cpp). No forward kinematics: the TCP pose is the controller's actual_TCP_pose.
RTDE receive is read-only, so this can run next to whatever program is on the robot.
"""
import sys
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from usrig_common import config, geometry
from usrig_common.session import read_json
from usrig_common.timeutil import stamp_from_ns
from usrig_interfaces.msg import RobotState

from .labels import label_sample
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

    def __init__(self, control_status=None, node_name='robot_state', publish_topics=True, **kwargs):
        super().__init__(node_name, **kwargs)
        self.publish_topics = publish_topics
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
        self.robot_ip = ip
        self.base_frame = self.get_parameter('base_frame').value
        self.embedded = control_status is not None      # inside robot_control: its status is always current
        self.status = control_status or ControlStatus()
        fd = config.get_path(rig, 'freedrive.input')
        self.freedrive_input = None if fd is None else int(fd)
        self._nocmd_n = 0

        # Phantom registration (identity until P5 writes it into session.json).
        self.T_phantom_base = np.eye(4)
        sj = self.get_parameter('session_json').value
        if sj:
            reg = read_json(sj).get('phantom_registration', {})
            if 'T_base_phantom' in reg:
                self.T_phantom_base = geometry.inv(np.array(reg['T_base_phantom']))

        # Depth 500 = 1 s at 500 Hz: a briefly busy subscriber (the bag recorder) does not lose samples.
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_state = self.create_publisher(RobotState, '/robot/state', qos)
        self.pub_js = self.create_publisher(JointState, '/joint_states', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/robot/tcp_pose', qos)
        self.pub_wrench = self.create_publisher(WrenchStamped, '/robot/wrench', qos)
        self.pub_nocmd = None if self.embedded else self.create_publisher(TwistStamped, '/robot/cmd_executed', 10)

        self.latest = None
        self.latest_lock = threading.Lock()
        self.listeners = []          # callables(sample) run in the RTDE thread (robot_control)

        # The RTDE thread only stamps and queues samples; a separate thread builds and publishes the
        # messages, so publishing load cannot make the reader miss controller samples.
        import queue
        self._q = queue.Queue(maxsize=5000)
        self.queue_overflows = 0
        self._pub_stop = threading.Event()
        self._pub_thread = threading.Thread(target=self._publish_loop, name='robot_state_pub', daemon=True)
        if self.publish_topics:
            self._pub_thread.start()
        self.reader = RtdeReader(ip, self.get_parameter('frequency_hz').value, self._enqueue,
                                 rt_priority=config.get_path(rig, 'robot.rt_priority') or 0,
                                 log=self.get_logger().info)
        self.reader.start()
        self.create_timer(1.0, self._log_stats)
        self.get_logger().info(f'RTDE receive connected to {ip}')

    def _enqueue(self, s):
        with self.latest_lock:
            self.latest = s
        for fn in self.listeners:
            fn(s)
        if not self.publish_topics:
            return
        try:
            self._q.put_nowait(s)
        except Exception:  # queue.Full
            self.queue_overflows += 1

    def _publish_loop(self):
        import queue
        while not self._pub_stop.is_set():
            try:
                s = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            self._publish(s)

    def _publish(self, s):
        stamp = stamp_from_ns(s.stamp_ns)
        T = geometry.pose_from_ur(s.tcp_pose)
        fsp, mode, flags, src, expert = label_sample(self.status.snapshot() if self.embedded else None, 0.0,
                                                     self.freedrive_input, s.di_bits, s.robot_mode)

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
        if self.pub_nocmd is not None:
            self._nocmd_n += 1
            if self._nocmd_n % 4 == 0:              # 125 Hz "no command" marker
                t = TwistStamped()
                t.header.stamp = stamp
                t.header.frame_id = 'none'
                nan = float('nan')
                (t.twist.linear.x, t.twist.linear.y, t.twist.linear.z,
                 t.twist.angular.x, t.twist.angular.y, t.twist.angular.z) = (nan,) * 6
                self.pub_nocmd.publish(t)

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
            f'gaps {st.controller_gaps}, {jit}, publish queue {self._q.qsize()} '
            f'(overflows {self.queue_overflows})')

    def destroy_node(self):
        self.reader.stop()
        self._pub_stop.set()
        if self._pub_thread.is_alive():
            self._pub_thread.join(timeout=1.0)
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
