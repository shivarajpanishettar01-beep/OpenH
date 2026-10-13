"""Headless operator view: a compact terminal status refreshed once per second.

  ros2 run usrig_qc status_console

For collection without the GUI (the GUI is for demos). It subscribes only to low-rate topics
(/qc/status 2 Hz, /episode/events, /robot/control_status 125 Hz raw-counted, /operator/command) and
never to images or the 500 Hz robot stream, so it costs almost nothing.
"""
import json
import shutil
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from usrig_interfaces.msg import ControlStatus, EpisodeEvent

MODES = ['IDLE', 'MOVING', 'DESCENT', 'TELEOP', 'PERTURB', 'RETRACT', 'FAULT']
SOURCES = ['teleop', 'scripted_task', 'transit', 'perturbation', 'none']
FLAGS = {1: 'envelope', 2: 'tilt', 4: 'force_max', 8: 'deadman', 16: 'speed'}
CLEAR = '\033[2J\033[H'


def main():
    rclpy.init()
    n = rclpy.create_node('status_console')
    be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    st = {'qc': None, 'ctl': None, 'events': [], 'rec': 'not recording', 'ep': None, 't0': None}

    def on_event(m):
        v = json.loads(m.value or '{}')
        if m.kind == 'start':
            st.update(rec='RECORDING', ep=m.episode_id, t0=time.time())
        elif m.kind == 'end':
            st.update(rec='not recording', t0=None)
        label = v.get('phase') or v.get('outcome') or ''
        st['events'] = (st['events'] + [f'{time.strftime("%H:%M:%S")} {m.kind} {label}'])[-6:]

    n.create_subscription(String, '/qc/status', lambda m: st.update(qc=json.loads(m.data)), be)
    n.create_subscription(ControlStatus, '/robot/control_status', lambda m: st.update(ctl=m), be)
    n.create_subscription(EpisodeEvent, '/episode/events', on_event, QoSProfile(depth=50))
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.monotonic() - last < 1.0:
                continue
            last = time.monotonic()
            w = shutil.get_terminal_size((100, 30)).columns
            lines = [f'usrig status  {time.strftime("%H:%M:%S")}'.ljust(w, '-')]
            el = f'  {time.time() - st["t0"]:.0f} s' if st['t0'] else ''
            lines.append(f'episode: {st["ep"] or "-"}   {st["rec"]}{el}')
            c = st['ctl']
            if c is None:
                lines.append('robot_control: no status (not running)')
            else:
                flags = [v for b, v in FLAGS.items() if c.safety_flags & b]
                mode = MODES[c.mode] if c.mode < len(MODES) else str(c.mode)
                warn = '   <<< FAULT: acknowledge with K' if mode == 'FAULT' else ''
                lines.append(f'robot: {mode}  source {SOURCES[c.action_source] if c.action_source < 5 else c.action_source}'
                             f'  expert {c.action_is_expert}  force setpoint {c.force_setpoint_n:.1f} N'
                             f'  flags {",".join(flags) or "none"}{warn}')
            q = st['qc']
            if q is None:
                lines.append('qc_monitor: no status (not running)')
            else:
                lines.append(f'micrus gaps {q.get("micrus_frame_gaps")}  coupling {q.get("coupling_ok")}  '
                             f'force {q.get("force_axis_n") if q.get("force_axis_n") is None else round(q["force_axis_n"], 2)} N '
                             f'in band {q.get("force_in_band")}  disk {q.get("free_disk_gb")} GB  USB {q.get("wrist_usb")}')
                for topic, s in sorted((q.get('topics') or {}).items()):
                    lines.append(f'  {topic:45s} {s.get("hz")!s:>7} Hz  age {s.get("stamp_age_ms")!s:>8} ms  max gap {s.get("max_gap_ms")!s:>8} ms')
            lines.append('events:')
            lines += [f'  {e}' for e in st['events']] or ['  -']
            print(CLEAR + '\n'.join(lines), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
