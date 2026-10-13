"""spacemouse_teleop: 3Dconnexion SpaceMouse -> /teleop/twist (D1), hold-to-move (D2).

Reads sensor_msgs/Joy from the ROS spacenav driver (`ros2 run spacenav spacenav_node`). While the
enable button is held, each mapped axis gives a proportional probe-frame velocity (full deflection =
the keymap.yaml speed for that axis; fine button scales it down); outside the deadband only.
Releasing the enable button gives zero. Pushing / pulling the cap along `force_axis` sends
force_step commands (compress / decompress) at up to `force_rate_n_s`.

If no Joy message arrives for `joy_timeout_s` (driver stopped, device unplugged), the node sends one
zero twist and stops publishing, so robot_control's deadman stops the robot.
Keyboard commands (outcomes, view reached, ...) still come from keyboard_teleop with publish_twist: false.
"""
import json
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from usrig_common import config
from usrig_common.timeutil import now_ns, stamp_from_ns

from .keyboard_teleop import AXIS_INDEX, check_speeds


def map_joy(axes, buttons, sm, speeds, fine_scale):
    """Joy axes / buttons -> (twist[6], force_fraction). Pure function (tested)."""
    out = np.zeros(6)
    enable = sm['enable_button']
    if enable is None or len(buttons) <= enable or not buttons[enable]:
        return out, 0.0
    fine = sm.get('fine_button') is not None and len(buttons) > sm['fine_button'] and buttons[sm['fine_button']]
    db = float(sm['deadband'])

    def shaped(i, sign):
        a = float(axes[i]) * sign if i < len(axes) else 0.0
        a = float(np.clip(a, -1.0, 1.0))
        if abs(a) <= db:
            return 0.0
        return np.sign(a) * (abs(a) - db) / (1.0 - db)     # continuous at the deadband edge
    for name, (idx, sign) in sm['axes'].items():
        k = AXIS_INDEX[name]
        out[k] = shaped(int(idx), float(sign)) * speeds[k]
    if fine:
        out *= fine_scale
    fi, fs = sm['force_axis']
    return out, shaped(int(fi), float(fs))


class SpaceMouseTeleop(Node):

    def __init__(self):
        super().__init__('spacemouse_teleop')
        self.declare_parameter('config_dir', '')
        cfg_dir = self.get_parameter('config_dir').value or None
        km = config.load_config('keymap', cfg_dir)
        rig = config.load_config('rig', cfg_dir)
        self.speeds = check_speeds(km, rig)
        self.sm = km['spacemouse']
        self.fine_scale = float(km.get('fine_scale', 0.25))
        self.force_rate = self.sm.get('force_rate_n_s')
        self.rate = float(km.get('publish_rate_hz', 100.0))
        self.lock = threading.Lock()
        self.joy = None
        self.joy_t = 0.0
        self.lost = True
        self._zero_sent = False
        self.pub_twist = self.create_publisher(TwistStamped, '/teleop/twist', 10)
        self.pub_cmd = self.create_publisher(String, '/operator/command', 50)
        self.create_subscription(Joy, self.sm['topic'], self._on_joy, 10)
        self.create_timer(1.0 / self.rate, self._publish)
        if self.force_rate is None:
            self.get_logger().warn('spacemouse.force_rate_n_s is null: force nudges from the cap are off')

    def _on_joy(self, m):
        with self.lock:
            self.joy = (list(m.axes), list(m.buttons))
            self.joy_t = time.monotonic()

    def _send(self, payload):
        self.pub_cmd.publish(String(data=json.dumps(payload)))

    def _twist(self, v):
        m = TwistStamped()
        m.header.stamp = stamp_from_ns(now_ns())
        m.header.frame_id = 'probe'
        (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
         m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = map(float, v)
        self.pub_twist.publish(m)

    def _publish(self):
        with self.lock:
            joy, age = self.joy, time.monotonic() - self.joy_t
        stale = joy is None or age > float(self.sm['joy_timeout_s'])
        if stale:
            if not self.lost:
                self.get_logger().error('SpaceMouse input lost: twist publishing stopped (deadman stops the robot)')
                self._send({'cmd': 'teleop_input_lost', 'device': 'spacemouse'})
            self.lost = True
            if not self._zero_sent:
                self._twist(np.zeros(6))
                self._zero_sent = True
            return
        if self.lost:
            self._send({'cmd': 'teleop_input_back', 'device': 'spacemouse'})
        self.lost, self._zero_sent = False, False
        v, f = map_joy(joy[0], joy[1], self.sm, self.speeds, self.fine_scale)
        self._twist(v)
        if f and self.force_rate is not None:
            self._send({'cmd': 'force_step', 'delta': float(f * self.force_rate / self.rate)})


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpaceMouseTeleop()
        rclpy.spin(node)
    except (config.UnsetLimitError, ValueError, KeyError) as e:
        print(f'[spacemouse_teleop] {e}', file=sys.stderr)
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
