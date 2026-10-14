"""keyboard_teleop: evdev key-down / key-up (several keys at once) -> /teleop/twist at 100 Hz.

Motion keys set a target probe-frame velocity, ramped over ~0.25 s. Command keys publish JSON
on /operator/command. Every raw key event is also published there ({"cmd": "key", ...}) so the
bag keeps an audit trail; raw keys are never used as the recorded action.

Needs read access to /dev/input/event* (add the user to the 'input' group).

evdev reads the keyboard device directly, independent of window focus: keys typed into any window
on this PC reach the robot while teleop is enabled. Use a dedicated USB keyboard / keypad for the
operator and set keymap.yaml `device:` to its /dev/input/by-id/... path so no other keyboard is read.

Input failures are safe: if the device is lost (unplugged, read error) all keys are released, one
zero twist is sent and publishing STOPS, so robot_control's deadman stops the robot; the node keeps
trying to reopen the device. A kernel event overflow (SYN_DROPPED) resynchronises the held keys from
the device instead of trusting possibly-lost key-ups.
Speeds come from keymap.yaml and are checked against the rig.yaml limits at start-up.
"""
import json
import select
import sys
import threading
import time

import rclpy

from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from usrig_common import config
from usrig_common.timeutil import now_ns, stamp_from_ns
from usrig_common.rosutil import ignore_sigint

from .ramp import KeyState, TwistRamp

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


def check_device(km, publish_twist):
    """Refuse to drive the robot from 'whatever keyboard is found' (keymap device: null).

    evdev ignores window focus, and the hold-to-move enable is Ctrl, so everyday shortcuts typed into any
    window on the PC are motion keys: Ctrl+C / Ctrl+Z rotate, Ctrl+S / Ctrl+W / Ctrl+A / Ctrl+D slide,
    Ctrl+R / Ctrl+F / Ctrl+Q / Ctrl+E tilt and rock. Returns a warning (str) for the commands-only mode
    (no motion), raises ValueError when this node would publish /teleop/twist."""
    if km.get('device'):
        return None
    why = ('keymap.yaml device is null: evdev reads every keyboard regardless of window focus, so shortcuts '
           'typed in a terminal (Ctrl+C, Ctrl+Z, Ctrl+S, ...) are Ctrl + motion keys and outcome / ack keys '
           'act on the episode')
    if publish_twist and not km.get('allow_any_keyboard', False):
        raise ValueError(f'{why}. Set device: to the operator keyboard\'s /dev/input/by-id/...-event-kbd path '
                         '(or allow_any_keyboard: true for a bench test without the arm in reach)')
    return why


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
        self.declare_parameter('publish_twist', True)     # launch sets false when the SpaceMouse drives
        cfg_dir = self.get_parameter('config_dir').value or None
        km = config.load_config('keymap', cfg_dir)
        rig = config.load_config('rig', cfg_dir)
        publish_twist = bool(km.get('publish_twist', True)) and bool(self.get_parameter('publish_twist').value)
        # commands only (SpaceMouse drives, or freedrive collection): no motion, so no speeds needed
        speeds = check_speeds(km, rig) if publish_twist else [0.0] * 6
        device_warning = check_device(km, publish_twist)

        import evdev
        self.evdev = evdev
        self.device_path = km.get('device')
        self.dev = find_keyboard(evdev, self.device_path)
        self.device_path = self.device_path or self.dev.path
        self.get_logger().info(f'reading keys from {self.dev.path} ({self.dev.name})')
        if device_warning:
            self.get_logger().warn(device_warning)

        self.motion_keys = {}
        for key, spec in km['motion'].items():
            axis, sign = spec.split(':')
            self.motion_keys[key.upper()] = (AXIS_INDEX[axis], float(sign))
        self.commands = {k.upper(): v for k, v in km['commands'].items()}
        self.fine_keys = {k.upper() for k in km.get('fine_keys', ['LEFTSHIFT', 'RIGHTSHIFT'])}
        self.force_step = float(km['force_step_n']) if km.get('force_step_n') is not None else None
        self.enable_keys = {k.upper() for k in km.get('enable_keys') or []}
        self.publish_twist = publish_twist
        self.ramp = TwistRamp(self.motion_keys, speeds, km.get('fine_scale', 0.25), km.get('ramp_s', 0.25),
                              enable_keys=self.enable_keys)

        self.keys = KeyState()
        self.lock = threading.Lock()
        self._zero_sent = False
        # with the SpaceMouse driving, this node only sends commands (one twist publisher only)
        self.pub_twist = self.create_publisher(TwistStamped, '/teleop/twist', 10) if self.publish_twist else None
        self.pub_cmd = self.create_publisher(String, '/operator/command', 50)
        self.rate = float(km.get('publish_rate_hz', 100.0))
        self.create_timer(1.0 / self.rate, self._publish)
        threading.Thread(target=self._read_keys, daemon=True).start()

    def _send(self, payload):
        m = String()
        m.data = json.dumps(payload)
        self.pub_cmd.publish(m)

    def _key_name(self, code):
        name = self.evdev.ecodes.KEY.get(code, str(code))
        return (name[0] if isinstance(name, list) else name).replace('KEY_', '')

    def _active_names(self):
        return [self._key_name(c) for c in self.dev.active_keys()]

    def _read_keys(self):
        ec = self.evdev.ecodes
        while rclpy.ok():
            try:
                r, _, _ = select.select([self.dev.fd], [], [], 0.1)
                if not r:
                    continue
                for ev in self.dev.read():
                    if ev.type == ec.EV_SYN and ev.code == ec.SYN_DROPPED:
                        with self.lock:
                            self.keys.resync(self._active_names())
                        self.get_logger().warn('keyboard events dropped by the kernel: key state resynchronised')
                        continue
                    if ev.type != ec.EV_KEY or ev.value == 2:      # ignore autorepeat
                        continue
                    name = self._key_name(ev.code)
                    down = ev.value == 1
                    t_ns = int(ev.sec * 1e9 + ev.usec * 1e3)
                    self._send({'cmd': 'key', 'key': name, 'down': down, 't_ns': t_ns})
                    with self.lock:
                        self.keys.key(name, down)
                    if down and name in self.commands:
                        self._command(self.commands[name], name)
            except (OSError, ValueError) as e:
                with self.lock:
                    self.keys.device_lost()
                self.get_logger().error(f'keyboard lost ({e}): all keys released, twist publishing stopped '
                                        '(robot_control deadman stops the robot); reconnecting')
                self._send({'cmd': 'teleop_input_lost', 'error': str(e)})
                self._reconnect()

    def _reconnect(self):
        while rclpy.ok():
            time.sleep(1.0)
            try:
                self.dev = find_keyboard(self.evdev, self.device_path)
                with self.lock:
                    self.keys.device_back(self._active_names())
                self._zero_sent = False
                self.get_logger().info(f'keyboard back: {self.dev.path}')
                self._send({'cmd': 'teleop_input_back'})
                return
            except (OSError, RuntimeError):
                continue

    def _command(self, action, key):
        with self.lock:
            fine = bool(self.keys.held & self.fine_keys)
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
        if self.pub_twist is None:
            return
        with self.lock:
            held, lost = set(self.keys.held), self.keys.lost
        if lost:
            # One zero twist, then silence: robot_control's deadman must see the input as gone.
            if not self._zero_sent:
                self.ramp.out[:] = 0.0
                self._send_twist(self.ramp.out)
                self._zero_sent = True
            return
        out = self.ramp.step(held, bool(held & self.fine_keys), 1.0 / self.rate)
        self._send_twist(out)

    def _send_twist(self, out):
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
