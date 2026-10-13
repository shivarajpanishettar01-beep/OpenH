"""Small ROS helper for the interactive CLI tools (registration, waypoints, hand-eye)."""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from usrig_common import config, geometry
from usrig_interfaces.msg import RobotState


class ToolNode(Node):

    def __init__(self, name, config_dir=None):
        super().__init__(name)
        self.scfg = config.load_config('session', config_dir)
        self.rig = config.load_config('rig', config_dir)
        self._state = None
        self._lock = threading.Lock()
        self.create_subscription(RobotState, '/robot/state', self._on_state, 10)
        self._ex = MultiThreadedExecutor(num_threads=2)
        self._ex.add_node(self)
        threading.Thread(target=self._ex.spin, daemon=True).start()

    def _on_state(self, m):
        with self._lock:
            self._state = m

    def state(self, timeout=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._lock:
                if self._state is not None:
                    return self._state
            time.sleep(0.01)
        raise RuntimeError('no /robot/state (is robot_state running?)')

    def tip_pose_base(self):
        """Probe-tip pose in the base frame. Never applies the probe offset twice."""
        s = self.state()
        T = geometry.pose_from_msg(s.tcp_base)
        inc = config.get_path(self.rig, 'probe.tcp_already_includes_probe')
        if inc is None:
            raise config.UnsetLimitError(['probe.tcp_already_includes_probe'], 'rig.yaml')
        if inc:
            return T
        off = config.get_path(self.rig, 'probe.tip_offset_flange_m')
        if off is None:
            raise config.UnsetLimitError(['probe.tip_offset_flange_m'], 'rig.yaml')
        return T @ geometry.pose_from_ur(off)

    def averaged_tip_position(self, seconds=0.5):
        pts = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            pts.append(self.tip_pose_base()[:3, 3])
            time.sleep(0.01)
        return np.mean(pts, axis=0)

    def call(self, client, req, timeout=30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{client.srv_name} not available')
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'{client.srv_name} timed out')
            time.sleep(0.01)
        return fut.result()


def run_tool(fn, name):
    rclpy.init()
    node = None
    try:
        node = ToolNode(name)
        fn(node)
    except (RuntimeError, config.UnsetLimitError, ValueError) as e:
        print(f'[{name}] {e}')
        raise SystemExit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
