#!/usr/bin/env python3
"""Lossless wrist depth export, aligned to the dataset steps written by bag_to_lerobot.

  python depth_export.py --dataset ~/datasets/us_v21 [--topic /wrist/d415/aligned_depth_to_color/image_raw]

For each episode in meta/micrus_sources.jsonl, step k is at start_unix + k / fps (the
converter's rule); the depth frame used is the latest one stamped at or before that time
(+ time_offset_s), the same sample-and-hold rule as the images. Writes
  depth/episode_XXXXXX.npz   depth uint16 [n, H, W] in mm, stamp_ns int64 [n] (real header stamps)
  meta/depth_index.jsonl     episode_index, file, shape, units, topic
"""
import argparse
import json
from pathlib import Path

import numpy as np


def read_jsonl(p):
    return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--topic', default='/wrist/d415/aligned_depth_to_color/image_raw')
    ap.add_argument('--time-offset-s', type=float, default=0.0, help='PLACEHOLDER until P9c')
    a = ap.parse_args()
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    ds = Path(a.dataset).expanduser()
    info = json.loads((ds / 'meta' / 'info.json').read_text())
    lengths = {e['episode_index']: e['length'] for e in read_jsonl(ds / 'meta' / 'episodes.jsonl')}
    (ds / 'depth').mkdir(exist_ok=True)
    index = []
    for src in read_jsonl(ds / 'meta' / 'micrus_sources.jsonl'):
        ep = src['episode_index']
        n = lengths[ep]
        t_steps = (float(src['start_unix']) + np.arange(n) / float(src['fps'])) * 1e9
        stamps, frames = [], []
        with AnyReader([Path(src['bag'])], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
            conns = [c for c in r.connections if c.topic == a.topic]
            if not conns:
                raise SystemExit(f'{src["bag"]}: no {a.topic}')
            for c, _, raw in r.messages(connections=conns):
                m = r.deserialize(raw, c.msgtype)
                stamps.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec + int(a.time_offset_s * 1e9))
                frames.append(np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width].copy())
        stamps = np.asarray(stamps, np.int64)
        order = np.argsort(stamps)
        stamps = stamps[order]
        idx = np.searchsorted(stamps, t_steps, side='right') - 1
        if np.any(idx < 0):
            raise SystemExit(f'episode {ep}: depth starts after the first step')
        depth = np.stack([frames[order[i]] for i in idx])
        f = f'depth/episode_{ep:06d}.npz'
        np.savez_compressed(ds / f, depth=depth, stamp_ns=stamps[idx], step_time_ns=t_steps.astype(np.int64))
        index.append({'episode_index': ep, 'file': f, 'shape': list(depth.shape), 'units': 'mm',
                      'dtype': 'uint16', 'topic': a.topic, 'repeated_frames': int(n - len(np.unique(idx)))})
        print(f'episode {ep}: {n} steps, {len(np.unique(idx))} distinct depth frames -> {f}')
    (ds / 'meta' / 'depth_index.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in index))
    info.setdefault('extras', {})['depth'] = 'depth/episode_XXXXXX.npz, see meta/depth_index.jsonl'
    (ds / 'meta' / 'info.json').write_text(json.dumps(info, indent=4))


if __name__ == '__main__':
    main()
