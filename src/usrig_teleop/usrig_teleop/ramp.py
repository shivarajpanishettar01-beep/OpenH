"""Key state -> smooth probe-frame twist. Pure logic, no evdev, no ROS."""
import numpy as np

AXES = ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')


class TwistRamp:
    """Each held key sets a target on one axis; the output moves towards the target
    linearly, reaching it in ``ramp_s``. Releasing all keys ramps to zero the same way.
    """

    def __init__(self, motion_keys, speeds, fine_scale, ramp_s):
        # motion_keys: {key_name: (axis_index, sign)}; speeds: per-axis max (m/s or rad/s)
        self.motion_keys = motion_keys
        self.speeds = np.asarray(speeds, float)
        self.fine_scale = float(fine_scale)
        self.ramp_s = float(ramp_s)
        self.out = np.zeros(6)

    def target(self, held, fine):
        t = np.zeros(6)
        for k in held:
            if k in self.motion_keys:
                axis, sign = self.motion_keys[k]
                t[axis] += sign
        t = np.clip(t, -1, 1) * self.speeds
        return t * (self.fine_scale if fine else 1.0)

    def step(self, held, fine, dt):
        tgt = self.target(held, fine)
        max_step = self.speeds * (dt / self.ramp_s)
        self.out = self.out + np.clip(tgt - self.out, -max_step, max_step)
        self.out[np.abs(self.out) < 1e-12] = 0.0
        return self.out.copy()


class KeyState:
    """Held keys from evdev-style events, with the two failure cases handled safely.

    * SYN_DROPPED (kernel buffer overflow, key-ups may be lost): resynchronise from the device's
      actual key state (`active_keys()` names), never keep a key that may have been released.
    * device lost (unplugged / read error): clear every key and mark the input as lost; the
      publisher then stops sending twists so robot_control's deadman stops the robot.
    """

    def __init__(self):
        self.held = set()
        self.lost = False

    def key(self, name, down):
        (self.held.add if down else self.held.discard)(name)

    def resync(self, active_names):
        self.held = set(active_names)

    def device_lost(self):
        self.held.clear()
        self.lost = True

    def device_back(self, active_names=()):
        self.held = set(active_names)
        self.lost = False
