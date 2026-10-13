"""Loads one episode for the annotation tool: B-mode, wrist colour (downscaled), probe-axis force,
events. Reads the MCAP bag with rosbags (no ROS graph needed)."""
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class EpisodeData:
    ep_dir: Path
    episode: dict
    t0_ns: int
    t1_ns: int
    us_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    us: list = field(default_factory=list)
    cam_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    cam: list = field(default_factory=list)
    force_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    force: np.ndarray = field(default_factory=lambda: np.zeros(0))
    events: list = field(default_factory=list)

    def frame_at(self, kind, t_ns):
        ts, frames = (self.us_t, self.us) if kind == 'us' else (self.cam_t, self.cam)
        if not len(ts):
            return None
        i = int(np.searchsorted(ts, t_ns, side='right')) - 1
        return frames[max(i, 0)]


def _stamp(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def load_episode(ep_dir, color_topic, cam_width=320, progress=None):
    import cv2
    from rosbags.highlevel import AnyReader
    from usrig_qc.episode_qc import _typestore
    ep_dir = Path(ep_dir)
    episode = json.loads((ep_dir / 'episode.json').read_text())
    us_t, us, cam_t, cam, f_t, f, events = [], [], [], [], [], [], []
    topics = {'/micrus/bmode', color_topic, '/robot/state', '/episode/events'}
    with AnyReader([ep_dir], default_typestore=_typestore()) as r:
        conns = [c for c in r.connections if c.topic in topics]
        total = sum(c.msgcount for c in conns) or 1
        for n, (c, _, raw) in enumerate(r.messages(connections=conns)):
            if progress and n % 2000 == 0:
                progress(n / total)
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/micrus/bmode':
                us_t.append(_stamp(m))
                us.append(np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width].copy())
            elif c.topic == color_topic:
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                img = img.reshape(m.height, m.width, 3)
                if m.encoding == 'bgr8':
                    img = img[:, :, ::-1]
                h = int(round(m.height * cam_width / m.width))
                cam_t.append(_stamp(m))
                cam.append(cv2.resize(img, (cam_width, h), interpolation=cv2.INTER_AREA))
            elif c.topic == '/robot/state':
                o = m.tcp_base.orientation
                x, y, z, w = o.x, o.y, o.z, o.w
                axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
                f_t.append(_stamp(m))
                f.append(float(axis @ np.asarray(m.wrench[:3])))
            else:
                events.append((_stamp(m), m.kind, json.loads(m.value or '{}')))
    t0 = episode.get('t_start_ns') or (us_t[0] if us_t else 0)
    t1 = episode.get('t_end_ns') or (us_t[-1] if us_t else t0)
    o = np.argsort(us_t)
    oc = np.argsort(cam_t)
    return EpisodeData(ep_dir, episode, int(t0), int(t1), np.asarray(us_t, np.int64)[o], [us[i] for i in o],
                       np.asarray(cam_t, np.int64)[oc], [cam[i] for i in oc],
                       np.asarray(f_t, np.int64), np.asarray(f), sorted(events, key=lambda e: e[0]))
