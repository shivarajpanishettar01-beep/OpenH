"""Mock data source: scripted episodes, no ROS, no hardware.

Each scenario is a list of timeline segments. The probe tip position, tilt and "plane quality"
are interpolated along the timeline; joint angles come from a small IK, the wrench from a spring
model of the phantom, and the images from mock_images. Everything shown in mock mode is
synthetic and the header says so.
"""
from __future__ import annotations

import datetime as _dt
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .. import kinematics as kin
from .. import metrics as M
from ..frames import ToolCalibration, tool_frames
from ..models import (CameraFrame, ContactState, DashboardState, ForceTorque, Health, Metric, NodeHealth,
                      Outcome, PlaneState, RecordingState, RecordingStatus, RobotState, ScanMetrics,
                      StreamStatus, SystemState, TaskEvent, TaskPhase, TaskState, UltrasoundFrame)
from .base import DataSource, expected_text, rate_health
from .mock_images import BModeGenerator, WristViewGenerator

# Scene (metres, robot base frame). Phantom top surface at TOP, sweep along +y.
X0, Y0, TOP = 0.42, 0.10, 0.06
PEN = 0.0022                    # indentation in contact -> ~4.4 N
K_N_PER_MM = 2.0                # phantom stiffness (illustrative)
SWEEP_LEN = 0.06
PHANTOM = dict(center=(X0, Y0 + 0.03), size=(0.12, 0.20, TOP))   # x, y extent and height
HOME_Z = TOP + 0.14

A, C, V, S, R, RC = (TaskPhase.APPROACH, TaskPhase.CONTACT, TaskPhase.VIEW_ACQUISITION,
                     TaskPhase.SWEEP, TaskPhase.REACQUISITION, TaskPhase.RECOVERY)
OK, WARN, ERR, INFO = Health.OK, Health.WARN, Health.DOWN, Health.UNKNOWN


def P(dx=0.0, dy=0.0, dz=0.0):
    """Position relative to the sweep start on the phantom surface; None keeps that coordinate."""
    return tuple(None if d is None else b + d for b, d in zip((X0, Y0, TOP), (dx, dy, dz)))


@dataclass
class Seg:
    name: str
    dur: float
    phase: Optional[TaskPhase]
    to: Optional[Tuple[float, float, float]] = None
    tilt_deg: Optional[float] = None
    plane: Optional[float] = None
    linear: bool = False
    mode: str = 'TELEOP'
    wiggle: float = 0.0
    events: List[Tuple[float, str, Health]] = field(default_factory=list)
    outcome: Optional[Outcome] = None          # outcome from the start of this segment
    rec: str = 'rec'                           # rec | finalizing | idle
    episode: bool = True                       # False after the episode ended (retract, idle)


def _prefix(view_s=5.0, view_plane=1.0):
    return [
        Seg('approach', 4.0, A, to=P(0, 0, 0.02), plane=0.0, mode='MOVING', tilt_deg=0.0),
        Seg('contact', 3.0, C, to=P(0, 0, -PEN), plane=0.45, mode='DESCENT',
            events=[(2.5, 'Contact detected', INFO)]),
        Seg('view', view_s, V, to=P(0.003, 0, -PEN), plane=view_plane, wiggle=1.0,
            events=[(view_s - 0.8, 'Target view reached', OK)] if view_plane > 0.9 else []),
    ]


def _suffix():
    return [
        Seg('finalize', 1.5, None, mode='RETRACT', to=P(0.003, None, 0.03), rec='finalizing', episode=False),
        Seg('retract', 2.5, None, mode='MOVING', to=(X0, Y0, HOME_Z), rec='idle', episode=False),
        Seg('wait', 3.0, None, mode='IDLE', rec='idle', episode=False),
    ]


def _success():
    return _prefix() + [
        Seg('sweep', 12.0, S, to=P(0.003, SWEEP_LEN, -PEN), linear=True, mode='MOVING',
            events=[(0.0, 'Sweep started', INFO)]),
        Seg('reacq', 3.5, R, to=P(0.003, SWEEP_LEN / 2, -PEN)),
        Seg('hold', 2.0, R, mode='IDLE', outcome=Outcome.SUCCESS, events=[(0.0, 'Outcome: success', OK)]),
    ] + _suffix()


def _recovery():
    return _prefix() + [
        Seg('sweep_a', 4.0, S, to=P(0.003, 0.02, -PEN), linear=True, mode='MOVING',
            events=[(0.0, 'Sweep started', INFO)]),
        Seg('perturb', 0.6, RC, to=P(0.003, 0.021, 0.006), tilt_deg=4.0, plane=0.2, mode='PERTURB',
            events=[(0.0, 'Staged perturbation: lift (FM-02)', WARN), (0.35, 'Contact lost', WARN)]),
        Seg('redescend', 2.4, RC, to=P(0.003, 0.021, -PEN), tilt_deg=1.5, plane=0.5,
            events=[(1.9, 'Contact regained', OK)]),
        Seg('reacq', 3.0, R, tilt_deg=0.0, plane=1.0, wiggle=0.6, events=[(2.4, 'View reacquired', OK)]),
        Seg('sweep_b', 6.0, S, to=P(0.003, SWEEP_LEN, -PEN), linear=True, mode='MOVING'),
        Seg('hold', 1.5, S, mode='IDLE', outcome=Outcome.SUCCESS,
            events=[(0.0, 'Outcome: success (recovered)', OK)]),
    ] + _suffix()


def _failure():
    return _prefix() + [
        Seg('sweep_a', 3.0, S, to=P(0.003, 0.015, -PEN), linear=True, mode='MOVING',
            events=[(0.0, 'Sweep started', INFO)]),
        Seg('drift', 2.0, S, to=P(0.003, 0.022, -PEN), tilt_deg=9.0, plane=0.25, mode='MOVING',
            events=[(1.0, 'Out of plane', WARN)]),
        Seg('liftoff', 0.8, RC, to=P(0.004, 0.023, 0.008), events=[(0.35, 'Contact lost', WARN)]),
        Seg('redescend', 3.0, RC, to=P(0.012, 0.023, -PEN), tilt_deg=7.0, plane=0.3,
            events=[(2.5, 'Contact regained', OK)]),
        Seg('reacq', 9.2, R, to=P(0.010, 0.026, -PEN), tilt_deg=6.0, plane=0.35, wiggle=1.0),
    ] + [Seg('finalize', 1.5, None, mode='RETRACT', to=P(0.010, None, 0.03), rec='finalizing', episode=False,
             outcome=Outcome.FAILURE, events=[(0.0, 'Cap reached (30 s) without the target view', ERR)])] \
      + _suffix()[1:]


def _aborted():
    return _prefix(view_s=3.0, view_plane=0.7) + [
        Seg('finalize', 1.5, None, mode='RETRACT', to=P(0.003, None, 0.03), rec='finalizing', episode=False,
            outcome=Outcome.ABORTED, events=[(0.0, 'Discarded by operator', ERR)]),
    ] + _suffix()[1:]


def _idle():
    return [Seg('idle', 3600.0, None, to=(X0, Y0, HOME_Z), mode='IDLE', rec='idle', episode=False)]


SCENARIOS = {
    'normal': (_success, 0.0),
    'idle': (_idle, 0.0),
    'degraded': (_success, 8.0),
    'disconnected': (_idle, 0.0),
    'recovery': (_recovery, 0.0),
    'success': (_success, 21.0),     # starts close to the end so the outcome shows quickly
    'failure': (_failure, 0.0),
    'aborted': (_aborted, 0.0),
}


def _smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class Timeline:
    def __init__(self, segs: List[Seg]):
        self.segs = segs
        self.total = sum(s.dur for s in segs)
        # end state of every segment: pos, tilt, plane
        st = {'pos': np.array([X0, Y0, HOME_Z]), 'tilt': 0.0, 'plane': 0.0}
        self.starts = []
        t = 0.0
        for s in segs:
            self.starts.append((t, dict(st)))
            pos = st['pos'].copy()
            if s.to is not None:
                for i, v in enumerate(s.to):
                    if v is not None:
                        pos[i] = v
            st = {'pos': pos,
                  'tilt': st['tilt'] if s.tilt_deg is None else s.tilt_deg,
                  'plane': st['plane'] if s.plane is None else s.plane}
            t += s.dur
        self.ends = [self.starts[i + 1][1] for i in range(len(segs) - 1)] + [st]
        self.outcomes = []                         # outcome set by this or an earlier segment
        last = None
        for s in segs:
            last = s.outcome or last
            self.outcomes.append(last)
        self.events = [(t0 + dt, txt, lvl) for (t0, _), s in zip(self.starts, segs) for dt, txt, lvl in s.events]
        ep_segs = [i for i, s in enumerate(segs) if s.episode]
        self.episode_end = (self.starts[ep_segs[-1]][0] + segs[ep_segs[-1]].dur) if ep_segs else 0.0

    def at(self, t):
        t = min(max(t, 0.0), self.total - 1e-9)
        for i, ((t0, a), s) in enumerate(zip(self.starts, self.segs)):
            if t < t0 + s.dur or i == len(self.segs) - 1:
                b = self.ends[i]
                u = (t - t0) / s.dur
                w = u if s.linear else _smooth(u)
                pos = a['pos'] + (b['pos'] - a['pos']) * w
                tilt = a['tilt'] + (b['tilt'] - a['tilt']) * w
                plane = a['plane'] + (b['plane'] - a['plane']) * w
                if s.wiggle:
                    k = s.wiggle * (1 - u)
                    pos = pos + np.array([0.0015 * k * math.sin(2 * math.pi * t / 1.7), 0, 0])
                    tilt += 2.0 * k * math.sin(2 * math.pi * t / 2.3)
                outcome = self.outcomes[i] or (Outcome.IN_PROGRESS if s.episode else None)
                return dict(seg=s, pos=pos, tilt=tilt, plane=plane, outcome=outcome)
        raise RuntimeError('unreachable')


class MockSource(DataSource):
    label = 'mock'
    is_mock = True
    scenarios = list(SCENARIOS)

    def __init__(self, cfg: dict, scenario='normal', seed=0):
        self.cfg = cfg
        self.mcfg = cfg.get('mock', {})
        self.rng = np.random.default_rng(seed)
        self.us_gen = BModeGenerator()
        self.cam_gen = WristViewGenerator()
        self.session_id = self.mcfg.get('session_id', 'mock_session').format(
            date=_dt.date.today().strftime('%Y%m%d'))
        self.data_root = cfg.get('data_root', '~/data')
        self.cap_s = float(cfg.get('episode_cap_s') or 30.0)
        self.window_s = float(cfg.get('force_window_s', 10))
        # stand-in rig calibration (config mock.rig); the frames go through the same code as on ROS
        self.cal = ToolCalibration.from_rig(self.mcfg.get('rig') or {})
        if self.cal.T_flange_camera is not None:
            self.cal.camera_source = 'mock'
        self.set_scenario(scenario)

    # ------------------------------------------------------------ control of the mock itself
    def set_scenario(self, name):
        if name not in SCENARIOS:
            raise ValueError(f'unknown scenario {name!r}; one of {list(SCENARIOS)}')
        self.scenario = name
        make, self.offset = SCENARIOS[name]
        self.tl = Timeline(make())
        self.t0 = time.monotonic()
        self.q = kin.ik_point_down((X0, Y0, HOME_Z), (0, 0, -1), kin.Q_SEED)
        n = int(self.window_s * 100)
        self.ft_t = deque(maxlen=n)
        self.ft_v = deque(maxlen=n)
        self.last_ft_t = None
        self.stream_hz = {'us': 27.4, 'rgb': 30.0, 'depth': 30.0}     # what the simulated devices publish
        if name == 'degraded':
            self.stream_hz.update(us=18.0, depth=9.0)
        self.last_gen = {k: -1.0 for k in ('us', 'rgb', 'depth')}
        self.frames = {'us': None, 'rgb': None, 'depth': None}
        self.ep_base = 7

    def _clock(self):
        return time.monotonic() - self.t0 + self.offset

    # ------------------------------------------------------------ physics-ish helpers
    def _wrench(self, t_loc):
        st = self.tl.at(t_loc)
        st2 = self.tl.at(t_loc + 0.01)
        pen = TOP - st['pos'][2]
        vy = (st2['pos'][1] - st['pos'][1]) / 0.01
        n = self.rng.normal(0, 1, 6)
        fz = K_N_PER_MM * pen * 1000 if pen > 0 else 0.0
        fz *= 1 + 0.03 * math.sin(2 * math.pi * 1.3 * t_loc)          # breathing of the gel layer
        fy = -0.35 * fz * math.tanh(vy / 0.002)
        fx = 0.12 * fz * math.sin(2 * math.pi * 0.4 * t_loc)
        w = np.array([fx + 0.05 * n[0], fy + 0.05 * n[1], fz + 0.06 * n[2] + 0.15,
                      -fy * 0.03 + 0.004 * n[3], fx * 0.03 + 0.004 * n[4], 0.003 * n[5]])
        return w

    def _due(self, key, now):
        """Render a new frame at most at the simulated stream rate. The GUI may show fewer frames
        (like the real dashboard with keep-last-1 QoS); the reported rate is the stream's, not the GUI's."""
        if now - self.last_gen[key] >= 1.0 / self.stream_hz[key]:
            self.last_gen[key] = now
            return True
        return False

    def _hz(self, key):
        return self.stream_hz[key] * (1 + self.rng.normal(0, 0.004))

    # ------------------------------------------------------------ snapshot
    def snapshot(self) -> DashboardState:
        sc = self.scenario
        clock = self._clock()
        loop = int(clock // self.tl.total)
        t_loc = clock % self.tl.total
        now = time.monotonic()
        st = self.tl.at(t_loc)
        seg: Seg = st['seg']
        ds = DashboardState(source_label=f'mock: {sc}', is_mock=True)

        if sc == 'disconnected':
            return self._disconnected(ds)

        # robot
        tilt = math.radians(st['tilt'])
        zdir = np.array([0.0, math.sin(tilt), -math.cos(tilt)])
        self.q = kin.ik_point_down(st['pos'], zdir, self.q, iters=12)
        # what the controller would report as actual_TCP_pose: flange (tool0) x tcp_setting
        T_flange = kin.urdf_frames(self.q)['tool0']
        T = T_flange @ self.cal.T_flange_tcp if self.cal.T_flange_tcp is not None else T_flange
        fr = tool_frames(T, self.cal)
        ds.robot = RobotState(connected=True, joints_rad=self.q.copy(), tcp_pos_m=T[:3, 3].copy(),
                              tcp_rotvec=kin.rotvec_from_matrix(T[:3, :3]), mode=seg.mode,
                              T_flange=fr.flange, T_tip=fr.tip, T_camera=fr.camera,
                              frames_missing=self.cal.missing())

        # wrench, 100 Hz samples since the last snapshot
        if self.last_ft_t is None or clock - self.last_ft_t > 1.0 or clock < self.last_ft_t:
            self.last_ft_t = clock - 0.01
        ts = np.arange(self.last_ft_t + 0.01, clock + 1e-9, 0.01)
        for tt in ts:
            self.ft_t.append(tt)
            self.ft_v.append(self._wrench(tt % self.tl.total))
        if len(ts):
            self.last_ft_t = ts[-1]
        if self.ft_t:
            t_arr = np.array(self.ft_t)
            ds.ft = ForceTorque(t=t_arr - t_arr[-1], values=np.array(self.ft_v))
        fz = float(ds.ft.latest()[2]) if ds.ft.latest() is not None else None

        thr = float(self.mcfg.get('contact_force_threshold_n', 1.0))
        in_contact = fz is not None and fz > thr
        ds.contact = ContactState.IN_CONTACT if in_contact else ContactState.NO_CONTACT
        if not in_contact:
            ds.plane = PlaneState.UNKNOWN
        else:
            ds.plane = PlaneState.IN_PLANE if st['plane'] > 0.75 else PlaneState.OUT_OF_PLANE

        # ultrasound
        pen = TOP - st['pos'][2]
        if self._due('us', now) or self.frames['us'] is None:
            lateral = -(st['pos'][0] - (X0 + 0.003)) * 1000
            sweep = float(np.clip((st['pos'][1] - Y0) / SWEEP_LEN, 0, 1))
            coupling = float(np.clip(pen / 0.0008, 0, 1))
            plane_q = st['plane'] if in_contact else 0.5
            self.frames['us'] = self.us_gen.frame(clock, lateral_mm=lateral, sweep=sweep, coupling=coupling,
                                                  plane=plane_q, compression_mm=max(pen, 0) * 1000)
        img = self.frames['us']
        ds.ultrasound = UltrasoundFrame(status=StreamStatus.LIVE, image=img, pixel_mm=(0.1, 0.1),
                                        fps=self._hz('us'), frame_index=int(clock * self.stream_hz['us']),
                                        gain=None, depth_mm=40.0, freq_mhz=None)

        # wrist cameras
        cam_h = 0.12 + (st['pos'][2] - TOP)
        off = (st['pos'][0] - PHANTOM['center'][0], st['pos'][1] - PHANTOM['center'][1])
        if self._due('rgb', now) or self.frames['rgb'] is None:
            self.frames['rgb'] = self.cam_gen.rgb(cam_h, off, in_contact)
        if self._due('depth', now) or self.frames['depth'] is None:
            self.frames['depth'] = self.cam_gen.depth(cam_h, off)
        ds.rgb = CameraFrame(status=StreamStatus.LIVE, image=self.frames['rgb'], fps=self._hz('rgb'))
        ds.depth = CameraFrame(status=StreamStatus.LIVE, image=self.frames['depth'],
                               fps=self._hz('depth'), depth_range_m=(0.0, 0.5))

        # metrics
        ds.metrics = self._metrics(fz, in_contact, st, img)

        # task + recording
        ep_idx = self.ep_base + loop
        events = [TaskEvent(t, txt, lvl) for t, txt, lvl in self.tl.events if t <= t_loc]
        ep_time = min(t_loc, self.tl.episode_end)
        if sc == 'idle':
            ds.task = TaskState()
            ds.recording = RecordingState(status=RecordingStatus.IDLE, session_id=self.session_id,
                                          total_s=self.cap_s, output_path=None)
        else:
            ds.task = TaskState(phase=seg.phase if seg.episode else None, outcome=st['outcome'],
                                progress=min(ep_time / self.cap_s, 1.0), events=events[-4:],
                                detail='Scan the vessel bifurcation in a transverse view.')
            rec = {'rec': RecordingStatus.RECORDING, 'finalizing': RecordingStatus.FINALIZING,
                   'idle': RecordingStatus.IDLE}[seg.rec]
            ds.recording = RecordingState(
                status=rec, session_id=self.session_id, episode_id=f'ep_{ep_idx:04d}',
                elapsed_s=ep_time if rec != RecordingStatus.IDLE else None, total_s=self.cap_s,
                output_path=f'{self.data_root}/{self.session_id}/ep_{ep_idx:04d}/')

        # nodes
        ds.nodes = self._nodes(ds.recording.status == RecordingStatus.RECORDING)
        if sc == 'degraded':
            ds.recording.warnings = ['micrus frame gaps: 14 in this episode (QC)',
                                     'wrist depth: 61 % valid pixels',
                                     'd415 depth stream below expected rate']

        # system
        if any(n.health in (Health.WARN, Health.DOWN) for n in ds.nodes):
            ds.system = SystemState.DEGRADED
        elif sc == 'idle' or seg.mode == 'IDLE' and not seg.episode:
            ds.system = SystemState.IDLE
        else:
            ds.system = SystemState.RUNNING
        return ds

    def _metrics(self, fz, in_contact, st, img):
        band = self.mcfg.get('force_band_n')
        floor = self.mcfg.get('coupling_floor')
        items = []
        lvl = Health.UNKNOWN
        if fz is not None and band and in_contact:
            lvl = Health.OK if band[0] <= fz <= band[1] else Health.WARN
        items.append(Metric('Contact force', abs(fz) if fz is not None else None, 'N', '{:.2f}',
                            'F/T, probe axis', lvl, 'Force along the probe axis (tool z).'))
        ang = abs(st['tilt']) + abs(self.rng.normal(0, 0.15))
        items.append(Metric('Probe-tissue angle', ang if in_contact else None, 'deg', '{:.1f}', 'mock',
                            Health.OK if ang < 5 else Health.WARN,
                            'Angle between probe axis and phantom normal. Needs phantom registration on '
                            'the real rig; n/a when not in contact.'))
        score = float(np.clip(st['plane'] + self.rng.normal(0, 0.02), 0, 1)) if in_contact else None
        items.append(Metric('In-plane score', score, '', '{:.2f}', 'mock',
                            Health.OK if (score or 0) > 0.75 else Health.WARN,
                            'Placeholder 0-1 score. No estimator exists in the repo yet; n/a when not in contact.'))
        r0 = int(31 / 0.1)
        snr = M.speckle_snr(img, (r0, r0 + 60, 100, 280))
        items.append(Metric('Speckle SNR', snr, '', '{:.2f}', 'image ROI', Health.UNKNOWN,
                            'mean/std of displayed intensity, ROI 31-37 mm depth. Relative indicator only.'))
        vx, vz = self.us_gen.last_vessels[0][:2] if self.us_gen.last_vessels else (19.0, 17.0)
        ci, cz = int(vx / 0.1), int(vz / 0.1)
        c = M.cnr(img, (cz - 15, cz + 15, ci - 15, ci + 15), (cz - 30, cz + 30, 20, 70))
        items.append(Metric('CNR (vessel)', c, '', '{:.2f}', 'mock ROI', Health.UNKNOWN,
                            'Vessel lumen vs background. The mock knows where the vessel is; the real rig '
                            'needs a segmentation or a fixed ROI.'))
        cm = M.coupling_mean(img)
        q, ql = None, Health.UNKNOWN
        if cm is not None and floor is not None:
            q, ql = ('Good', Health.OK) if cm > floor else ('Low coupling', Health.WARN)
        return ScanMetrics(items=items, image_quality=q, image_quality_level=ql)

    def _nodes(self, recording):
        out = []
        j = lambda x, s: x + self.rng.normal(0, s)
        measured = {'/micrus/bmode': self._hz('us'),
                    '/wrist/d415/color/image_raw': self._hz('rgb'),
                    '/wrist/d415/aligned_depth_to_color/image_raw': self._hz('depth')}
        nominal = {'/robot/state': j(500.0, 0.6), '/robot/control_status': j(125.0, 0.2), '/qc/status': 2.0}
        for n in self.cfg.get('nodes', []):
            topic = n.get('topic', '')
            rate = measured.get(topic, nominal.get(topic))
            if n.get('node') == 'rosbag2_recorder':
                h = Health.OK if recording else Health.UNKNOWN
                out.append(NodeHealth(n['label'], topic, None, 'recording' if recording else 'idle', h))
                continue
            exp = n.get('expected_hz')
            h = rate_health(rate, exp)
            out.append(NodeHealth(n['label'], topic, rate, expected_text(exp), h))
        return out

    def _disconnected(self, ds):
        ds.system = SystemState.DISCONNECTED
        ds.robot = RobotState(connected=False)
        ds.ultrasound = UltrasoundFrame(status=StreamStatus.DISCONNECTED, depth_mm=None)
        ds.rgb = CameraFrame(status=StreamStatus.DISCONNECTED)
        ds.depth = CameraFrame(status=StreamStatus.DISCONNECTED)
        ds.metrics = ScanMetrics(items=[
            Metric('Contact force', None, 'N', note='No F/T data.'),
            Metric('Probe-tissue angle', None, 'deg', note='No robot state.'),
            Metric('In-plane score', None, note='No image.'),
            Metric('Speckle SNR', None, note='No image.'),
            Metric('CNR (vessel)', None, note='No image.')])
        ds.recording = RecordingState(status=RecordingStatus.UNKNOWN, session_id=self.session_id)
        ds.nodes = [NodeHealth(n['label'], n.get('topic', ''), None, expected_text(n.get('expected_hz')), Health.DOWN)
                    for n in self.cfg.get('nodes', [])]
        return ds

