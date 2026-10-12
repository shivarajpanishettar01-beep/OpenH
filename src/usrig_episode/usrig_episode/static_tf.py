"""Publishes rig calibrations on /tf_static from session.json and rig.yaml:
  base -> phantom          (phantom registration, when present)
  tcp  -> wrist_camera     (eye-in-hand calibration wrist_cam_cali_mtx, when present)
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from usrig_common import config, geometry
from usrig_common.session import read_json
from usrig_common.timeutil import now_stamp


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
        self.declare_parameter('tcp_frame', 'tcp')
        self.declare_parameter('camera_frame', 'wrist_camera')
        rig = config.load_config('rig', self.get_parameter('config_dir').value or None)
        sj = self.get_parameter('session_json').value
        sess = read_json(sj) if sj else {}
        tfs = []
        reg = sess.get('phantom_registration', {})
        if 'T_base_phantom' in reg:
            tfs.append(_tf(self.get_parameter('base_frame').value, 'phantom', reg['T_base_phantom']))
        cam = sess.get('wrist_cam_cali_mtx') or config.get_path(rig, 'calibration.wrist_cam_cali_mtx')
        if cam is not None:
            tfs.append(_tf(self.get_parameter('tcp_frame').value, self.get_parameter('camera_frame').value, cam))
        self.br = StaticTransformBroadcaster(self)
        if tfs:
            self.br.sendTransform(tfs)
        self.get_logger().info(f'published {len(tfs)} static transform(s)')


def main(args=None):
    rclpy.init(args=args)
    n = RigStaticTf()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
