"""Publishes rig calibrations on /tf_static from session.json and rig.yaml:
  base -> phantom          (phantom registration, when present)
  flange -> wrist_camera  (eye-in-hand calibration wrist_cam_cali_mtx = T_flange_camera, when present)
  base -> tpv_camera      (eye-to-hand calibration tpv_cam_cali_mtx = T_base_camera, when present)
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from usrig_common import config, geometry
from usrig_common.session import read_json
from usrig_common.timeutil import now_stamp
from usrig_common.rosutil import ignore_sigint


def _tf(parent, child, T):
    t = TransformStamped()
    t.header.stamp = now_stamp()
    t.header.frame_id = parent
    t.child_frame_id = child
    p, q = geometry.pq_from_pose(np.asarray(T, float))
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = map(float, q)
    return t


class RigStaticTf(Node):

    def __init__(self):
        super().__init__('rig_static_tf')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('session_json', '')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('flange_frame', 'flange')
        self.declare_parameter('camera_frame', 'wrist_camera')
        self.declare_parameter('tpv_frame', 'tpv_camera')
        self.br = StaticTransformBroadcaster(self)
        self.publish()
        # register_phantom announces a new registration: publish it now (it was read only at start-up)
        from std_msgs.msg import String
        self.create_subscription(String, '/operator/command', self._on_cmd, 10)

    def _on_cmd(self, msg):
        import json
        try:
            if json.loads(msg.data).get('cmd') == 'reload_registration':
                self.publish()
        except (ValueError, AttributeError):
            pass

    def publish(self):
        rig = config.load_config('rig', self.get_parameter('config_dir').value or None)
        sj = self.get_parameter('session_json').value
        sess = read_json(sj) if sj else {}
        tfs = []
        reg = sess.get('phantom_registration', {})
        if 'T_base_phantom' in reg:
            tfs.append(_tf(self.get_parameter('base_frame').value, 'phantom', reg['T_base_phantom']))
        cam = sess.get('wrist_cam_cali_mtx') or config.get_path(rig, 'calibration.wrist_cam_cali_mtx')
        if cam is not None:
            tfs.append(_tf(self.get_parameter('flange_frame').value, self.get_parameter('camera_frame').value, cam))
        tpv = sess.get('tpv_cam_cali_mtx') or config.get_path(rig, 'calibration.tpv_cam_cali_mtx')
        if tpv is not None:
            tfs.append(_tf(self.get_parameter('base_frame').value, self.get_parameter('tpv_frame').value, tpv))
        if tfs:
            self.br.sendTransform(tfs)
        self.tfs = tfs
        self.get_logger().info(f'published {len(tfs)} static transform(s)')


def main(args=None):
    rclpy.init(args=args)
    n = RigStaticTf()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
