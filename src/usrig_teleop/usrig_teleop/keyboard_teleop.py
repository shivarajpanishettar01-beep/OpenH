"""keyboard_teleop: evdev key-down / key-up (several keys at once) -> /teleop/twist at 100 Hz.

Motion keys set a target probe-frame velocity, ramped over ~0.25 s. Command keys publish JSON
on /operator/command. Every raw key event is also published there ({"cmd": "key", ...}) so the
bag keeps an audit trail; raw keys are never used as the recorded action.

Needs read access to /dev/input/event* (add the user to the 'input' group).
Speeds come from keymap.yaml and are checked against the rig.yaml limits at start-up.
"""
import json
import select
import sys
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from usrig_common import config
from usrig_common.timeutil import now_ns, stamp_from_ns

from .ramp import TwistRamp

AXIS_INDEX = {'vx': 0, 'vy': 1, 'vz': 2, 'wx': 3, 'wy': 4, 'wz': 5}


def find_keyboard(evdev, path=None):
    if path:
        return evdev.InputDevice(path)
    for p in evdev.list_devices():
        d = evdev.InputDevice(p)
        keys = d.capabilities().get(evdev.ecodes.EV_KEY, [])
        if evdev.ecodes.KEY_W in keys and evdev.ecodes.KEY_SPACE in keys:
            return d
    raise RuntimeError('no keyboard found in /dev/input (is the user in the input group?)')


def check_speeds(km, rig):
    """keymap speeds must stay inside rig.yaml limits; returns per-axis max list."""
    sp = km['speeds']
    lin = [sp['slide_along_mm_s'], sp['slide_across_mm_s'], 0.0]
    ang = [sp['tilt_deg_s'], sp['rock_deg_s'], sp['rotate_deg_s']]
    if any(v is None for v in lin + ang):
        raise config.UnsetLimitError([k for k, v in sp.items() if v is None], 'keymap.yaml speeds')
    vmax = config.get_path(rig, 'limits.max_linear_speed_mm_s')
    wmax = config.get_path(rig, 'limits.max_angular_speed_deg_s')
    if vmax is None or wmax is None:
        raise config.UnsetLimitError(['limits.max_linear_speed_mm_s', 'limits.max_angular_speed_deg_s'],
                                     'rig.yaml')
    bad = [v for v in lin if v > vmax] + [w for w in ang if w > wmax]
    if bad:
        raise ValueError(f'keymap speeds {bad} exceed rig.yaml limits ({vmax} mm/s, {wmax} deg/s)')
    import math
    return [v / 1000.0 for v in lin] + [math.radians(w) for w in ang]


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('config_dir', '')
        cfg_dir = self.get_parameter('config_dir').value or None
        km = config.load_config('keymap', cfg_dir)
        rig = config.load_config('rig', cfg_dir)
        speeds = check_speeds(km, rig)

        import evdev
        self.evdev = evdev
        self.dev = find_keyboard(evdev, km.get('device'))
        self.get_logger().info(f'reading keys from {self.dev.path} ({self.dev.name})')

        self.motion_keys = {}
        for key, spec in km['motion'].items():
            axis, sign = spec.split(':')
            self.motion_keys[key.upper()] = (AXIS_INDEX[axis], float(sign))
        self.commands = {k.upper(): v for k, v in km['commands'].items()}
        self.fine_keys = {k.upper() for k in km.get('fine_keys', ['LEFTSHIFT', 'RIGHTSHIFT'])}
        self.force_step = float(km['force_step_n']) if km.get('force_step_n') is not None else None
        self.ramp = TwistRamp(self.motion_keys, speeds, km.get('fine_scale', 0.25), km.get('ramp_s', 0.25))

        self.held = set()
        self.lock = threading.Lock()
        self.pub_twist = self.create_publisher(TwistStamped, '/teleop/twist', 10)
        self.pub_cmd = self.create_publisher(String, '/operator/command', 50)
        self.rate = float(km.get('publish_rate_hz', 100.0))
        self.create_timer(1.0 / self.rate, self._publish)
        threading.Thread(target=self._read_keys, daemon=True).start()

    def _send(self, payload):
        m = String()
        m.data = json.dumps(payload)
        self.pub_cmd.publish(m)

    def _read_keys(self):
        ec = self.evdev.ecodes
        while rclpy.ok():
            r, _, _ = select.select([self.dev.fd], [], [], 0.1)
            if not r:
                continue
            for ev in self.dev.read():
                if ev.type != ec.EV_KEY or ev.value == 2:      # ignore autorepeat
                    continue
                name = ec.KEY.get(ev.code, str(ev.code))
                name = (name[0] if isinstance(name, list) else name).replace('KEY_', '')
                down = ev.value == 1
                t_ns = int(ev.sec * 1e9 + ev.usec * 1e3)
                self._send({'cmd': 'key', 'key': name, 'down': down, 't_ns': t_ns})
                with self.lock:
                    (self.held.add if down else self.held.discard)(name)
                if down and name in self.commands:
                    self._command(self.commands[name], name)

    def _command(self, action, key):
        fine = bool(self.held & self.fine_keys)
        if action in ('force_up', 'force_down'):
            if self.force_step is None:
                self.get_logger().warn('force_step_n is null in keymap.yaml')
                return
            step = self.force_step * (0.25 if fine else 1.0) * (1 if action == 'force_up' else -1)
            self._send({'cmd': 'force_step', 'delta': step})
        elif action.startswith('outcome_'):
            self._send({'cmd': 'outcome', 'value': action[len('outcome_'):]})
        else:
            self._send({'cmd': action})

    def _publish(self):
        with self.lock:
            held = set(self.held)
        out = self.ramp.step(held, bool(held & self.fine_keys), 1.0 / self.rate)
        m = TwistStamped()
        m.header.stamp = stamp_from_ns(now_ns())
        m.header.frame_id = 'probe'
        (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
         m.twist.angular.x, m.twist.angular.y, m.twist.angular.z) = map(float, out)
        self.pub_twist.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except (config.UnsetLimitError, ValueError, RuntimeError) as e:
        print(f'[keyboard_teleop] {e}', file=sys.stderr)
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
