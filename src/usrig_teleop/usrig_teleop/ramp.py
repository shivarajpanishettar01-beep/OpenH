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
