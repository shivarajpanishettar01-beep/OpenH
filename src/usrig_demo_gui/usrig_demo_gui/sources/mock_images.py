"""Synthetic frames for the mock source. Purely illustrative, not a physical simulation.

B-mode: log-compressed speckle over a simple tissue map (skin, fascia lines, one vessel that
splits during the sweep, a bit of posterior enhancement). Coupling loss darkens the image and
adds reverberation lines; out-of-plane blurs the vessel.
Wrist RGB / depth: a top-down view of the phantom block with the probe in the frame.

Static layers are computed once; per frame only the moving parts are redrawn (~5 ms each).
"""
from __future__ import annotations

import numpy as np


def _blur(a, sigma, axis):
    if sigma <= 0:
        return a
    r = int(3 * sigma)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    return np.apply_along_axis(lambda v: np.convolve(v, k, mode='same'), axis, a)


class BModeGenerator:
    def __init__(self, depth_mm=40.0, width_mm=38.0, pixel_mm=0.1, n_fields=8, seed=1):
        self.pixel_mm = pixel_mm
        self.h = int(round(depth_mm / pixel_mm))
        self.w = int(round(width_mm / pixel_mm))
        rng = np.random.default_rng(seed)
        fields = []
        for _ in range(n_fields):
            re = _blur(_blur(rng.standard_normal((self.h, self.w)), 1.0, 0), 2.2, 1)
            im = _blur(_blur(rng.standard_normal((self.h, self.w)), 1.0, 0), 2.2, 1)
            env = np.hypot(re, im)
            fields.append((env / env.mean()).astype(np.float32))
        self.fields = fields
        z = (np.arange(self.h) * pixel_mm)[:, None].astype(np.float32)
        x = (np.arange(self.w) * pixel_mm)[None, :].astype(np.float32)
        self.z, self.x = z, x
        wave = 0.6 * np.sin(x / 6.0) + 0.3 * np.sin(x / 2.3 + 1.0)
        echo = np.full((self.h, self.w), 0.30, np.float32)
        echo += 0.10 * np.sin(z / 3.1 + x / 9.0)                                   # soft texture
        echo += 1.6 * np.exp(-((z - 1.2) / 0.5) ** 2)                              # skin line
        echo += 0.9 * np.exp(-((z - 7.5 - wave) / 0.35) ** 2)                      # fascia
        echo += 0.7 * np.exp(-((z - 27.0 - 0.7 * wave) / 0.5) ** 2)
        echo *= 1.0 + 0.25 * (z > 7.5 + wave)
        echo *= np.exp(-0.012 * z)                                                 # residual attenuation
        self.static = echo.astype(np.float32)
        rev = sum(np.exp(-((z - k * 2.2) / 0.25) ** 2) * 0.8 / k for k in range(1, 6))
        self.reverb = (rev * (0.6 + 0.4 * self.fields[0])).astype(np.float32)
        self.air = (0.08 * self.fields[1] * np.exp(-z / 25.0)).astype(np.float32)
        self.last_vessels = []

    def _px(self, mm):
        return int(round(mm / self.pixel_mm))

    def frame(self, t, lateral_mm=0.0, sweep=0.0, coupling=1.0, plane=1.0, compression_mm=0.0):
        """sweep 0..1 along the vessel; coupling 0..1; plane 0..1 (1 = in plane)."""
        echo = self.static.copy()
        cx = 19.0 + lateral_mm
        rz = max(4.2 - 0.25 * compression_mm, 2.0)
        split = float(np.clip((sweep - 0.35) / 0.5, 0, 1)) * 5.5
        vessels = [(cx - split, 17.0, 4.2 - 0.8 * split / 5.5, rz)]
        if split > 0.3:
            vessels.append((cx + split, 17.5, 2.6 + 0.6 * split / 5.5, rz * 0.75))
        self.last_vessels = vessels
        contrast = 0.25 + 0.75 * plane                                            # out of plane: blurry
        for vx, vz, vrx, vrz in vessels:
            c0, c1 = max(self._px(vx - vrx - 1.5), 0), min(self._px(vx + vrx + 1.5), self.w)
            r0, r1 = max(self._px(vz - vrz - 1.5), 0), min(self._px(vz + vrz + 1.5), self.h)
            if c1 <= c0 or r1 <= r0:
                continue
            # posterior enhancement below the lumen
            e0, e1 = max(self._px(vx - 0.9 * vrx), 0), min(self._px(vx + 0.9 * vrx), self.w)
            if e1 > e0:
                echo[self._px(vz + vrz):, e0:e1] *= 1 + 0.25 * contrast
            xs, zs = self.x[:, c0:c1], self.z[r0:r1]
            d = np.sqrt(((xs - vx) / vrx) ** 2 + ((zs - vz) / vrz) ** 2)
            win = echo[r0:r1, c0:c1]
            win[d < 1.0] *= 1 - 0.93 * contrast
            win += 1.3 * contrast * np.exp(-((d - 1.0) * vrx / 0.45) ** 2)

        # speckle: cross-fade between fields as the probe moves (elevational decorrelation)
        pos = (sweep * 5.0 + t * 0.15) % len(self.fields)
        i = int(pos)
        f = pos - i
        sp = self.fields[i] * (1 - f) + self.fields[(i + 1) % len(self.fields)] * f
        shift = self._px(lateral_mm)
        if shift:
            sp = np.roll(sp, shift, axis=1)
        env = echo * sp
        if coupling < 1.0:
            env *= coupling * coupling
            env += (1 - coupling) * (self.air + self.reverb)
        db = 20 * np.log10(env + 1e-4)
        img = np.clip((db + 30.0) * (255 / 40.0), 0, 255)
        return img.astype(np.uint8)


class WristViewGenerator:
    """Top-down wrist camera view; probe fixed in the image, phantom moves under it."""

    def __init__(self, w=320, h=240, seed=2):
        self.w, self.h = w, h
        rng = np.random.default_rng(seed)
        u = np.arange(w)[None, :].astype(np.float32)
        v = np.arange(h)[:, None].astype(np.float32)
        self.f_px = 160.0                      # focal length in pixels (illustrative)
        vig = 1 - 0.7 * (((u - w / 2) / w) ** 2 + ((v - h / 2) / h) ** 2)
        tt = _blur(_blur(rng.standard_normal((h, w)), 2, 0), 2, 1) * 6
        pt = _blur(_blur(rng.standard_normal((h, w)), 1.5, 0), 1.5, 1) * 4
        table = np.stack([120 + tt + 4, 120 + tt + 4, 120 + tt + 8], -1) * vig[..., None]
        shade = (1.0 - 0.10 * (v / h))[..., None]
        ph = np.stack([214 + pt, 160 + pt, 128 + pt], -1) * shade * vig[..., None]
        self.table = np.clip(table, 0, 255).astype(np.uint8)
        self.ph = np.clip(ph, 0, 255).astype(np.uint8)
        # probe: trapezoid from the top edge (camera sits behind the probe) down to the tip
        top_hw, tip_hw, tip_v = 46, 20, 0.64 * h
        half = top_hw + (tip_hw - top_hw) * np.clip(v / tip_v, 0, 1)
        body = (np.abs(u - w / 2) < half) & (v < tip_v)
        face = (np.abs(u - w / 2) < half - 5) & (v < tip_v - 5)
        self.probe_mask = body
        probe = np.zeros((h, w, 3), np.float32)
        probe[body] = (48, 50, 54)
        probe[face] = (70, 74, 80)
        probe += (np.exp(-((u - w / 2 + 10) / 6) ** 2) * 18 * face)[..., None]    # highlight
        self.probe = np.clip(probe, 0, 255).astype(np.uint8)
        gel = np.exp(-(((u - w / 2) / 42) ** 2 + ((v - tip_v) / 9) ** 2))
        self.gel = (gel[..., None] * np.array([45, 45, 50])).astype(np.uint16)
        self.table_depth_noise = (0.0006 * tt).astype(np.float32)
        self.probe_depth = (0.07 + 0.08 * v / h + 0 * u).astype(np.float32)
        self.shadow = (u > w / 2 + half) & (u < w / 2 + half + 10) & (v > 4) & (v < tip_v + 6)

    def _rect(self, cam_h_top, offset_xy_m):
        s = self.f_px / max(cam_h_top, 0.03)
        cx = self.w * 0.5 - offset_xy_m[1] * s
        cy = self.h * 0.64 - offset_xy_m[0] * s
        hw, hh = 0.10 * s, 0.06 * s            # phantom 0.20 x 0.12 m
        c0, c1 = int(np.clip(cx - hw, 0, self.w)), int(np.clip(cx + hw, 0, self.w))
        r0, r1 = int(np.clip(cy - hh, 0, self.h)), int(np.clip(cy + hh, 0, self.h))
        return r0, r1, c0, c1

    def rgb(self, cam_h_top, offset_xy_m, contact):
        img = self.table.copy()
        r0, r1, c0, c1 = self._rect(cam_h_top, offset_xy_m)
        img[r0:r1, c0:c1] = self.ph[r0:r1, c0:c1]
        img[self.probe_mask] = self.probe[self.probe_mask]
        if contact:
            img = np.minimum(img.astype(np.uint16) + self.gel, 255).astype(np.uint8)
        return img

    def depth(self, cam_h_top, offset_xy_m):
        """Depth in metres; 0 where invalid (as RealSense reports)."""
        d = cam_h_top + 0.06 + self.table_depth_noise
        r0, r1, c0, c1 = self._rect(cam_h_top, offset_xy_m)
        d[r0:r1, c0:c1] = cam_h_top + 0.3 * self.table_depth_noise[r0:r1, c0:c1]
        d[self.probe_mask] = self.probe_depth[self.probe_mask]
        d[self.shadow] = 0.0                                                         # occlusion holes
        return d
