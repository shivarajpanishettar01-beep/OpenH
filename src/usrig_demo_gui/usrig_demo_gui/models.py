"""Data structures shown by the demo GUI.

Every panel reads one of these and nothing else, so a data source (mock now, ROS 2 later) only
has to fill a DashboardState. None always means "no value / not available", never zero.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------- state categories

class SystemState(Enum):
    RUNNING = 'Running'
    IDLE = 'Idle'
    DEGRADED = 'Degraded'
    DISCONNECTED = 'Disconnected'


class ContactState(Enum):
    IN_CONTACT = 'In Contact'
    NO_CONTACT = 'No Contact'
    UNKNOWN = 'Unknown'


class PlaneState(Enum):
    IN_PLANE = 'In Plane'
    OUT_OF_PLANE = 'Out of Plane'
    UNKNOWN = 'Unknown'


class TaskPhase(Enum):
    APPROACH = 'Approach'
    CONTACT = 'Contact'
    VIEW_ACQUISITION = 'View Acquisition'
    SWEEP = 'Sweep'
    REACQUISITION = 'Reacquisition'
    RECOVERY = 'Recovery'


# Phases drawn on the progress strip, in order. RECOVERY is shown as a separate tag, not a step.
PHASE_STEPS = [TaskPhase.APPROACH, TaskPhase.CONTACT, TaskPhase.VIEW_ACQUISITION,
               TaskPhase.SWEEP, TaskPhase.REACQUISITION]

PHASE_TEXT = {
    TaskPhase.APPROACH: 'Moving to the approach pose above the phantom.',
    TaskPhase.CONTACT: 'Descending until the probe touches the phantom; force settles in the band.',
    TaskPhase.VIEW_ACQUISITION: 'Adjusting the probe to reach the target view.',
    TaskPhase.SWEEP: 'Constant-speed sweep along the planned path.',
    TaskPhase.REACQUISITION: 'Returning to the target view after a disturbance or at the end of the sweep.',
    TaskPhase.RECOVERY: 'A disturbance was detected; the probe is being brought back into contact.',
}


class Outcome(Enum):
    IN_PROGRESS = 'In Progress'
    SUCCESS = 'Success'
    FAILURE = 'Failure'
    ABORTED = 'Aborted'


class StreamStatus(Enum):
    LIVE = 'live'
    LOADING = 'loading'           # source up, no frame yet
    NO_IMAGE = 'no image'         # node up, frames empty or undecodable
    DISCONNECTED = 'disconnected'


class Health(Enum):
    OK = 'ok'
    WARN = 'warn'
    DOWN = 'down'
    UNKNOWN = 'unknown'


class RecordingStatus(Enum):
    RECORDING = 'Recording'
    FINALIZING = 'Finalizing'
    IDLE = 'Not recording'
    UNKNOWN = 'Unknown'


# ---------------------------------------------------------------- data

@dataclass
class RobotState:
    connected: bool = False
    joints_rad: Optional[np.ndarray] = None          # (6,)
    tcp_pos_m: Optional[np.ndarray] = None           # (3,) base frame
    tcp_rotvec: Optional[np.ndarray] = None          # (3,) axis-angle, rad (UR pendant convention)
    mode: Optional[str] = None                       # IDLE MOVING DESCENT TELEOP PERTURB RETRACT FAULT
    safety_flags: List[str] = field(default_factory=list)
    # Tracked tool frames in the base frame (4x4), from /robot/tcp_pose + calibration (frames.py).
    # None = not available (no data or calibration still null).
    T_flange: Optional[np.ndarray] = None            # end effector (tool0 / flange)
    T_tip: Optional[np.ndarray] = None               # probe tip
    T_camera: Optional[np.ndarray] = None            # wrist camera (colour optical frame)
    frames_missing: dict = field(default_factory=dict)   # frame -> rig.yaml entry still needed


@dataclass
class UltrasoundFrame:
    status: StreamStatus = StreamStatus.DISCONNECTED
    image: Optional[np.ndarray] = None               # uint8, rows = depth, cols = lateral
    pixel_mm: Tuple[float, float] = (0.1, 0.1)       # (lateral, axial)
    fps: Optional[float] = None
    frame_index: Optional[int] = None
    gain: Optional[float] = None                     # None = scanner default / not reported
    depth_mm: Optional[float] = None
    freq_mhz: Optional[float] = None


@dataclass
class CameraFrame:
    status: StreamStatus = StreamStatus.DISCONNECTED
    image: Optional[np.ndarray] = None               # rgb: (h, w, 3) uint8; depth: (h, w) float32 metres
    fps: Optional[float] = None
    depth_range_m: Tuple[float, float] = (0.0, 1.0)  # display range for depth


FT_LABELS = ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')
FT_UNITS = ('N', 'N', 'N', 'Nm', 'Nm', 'Nm')


@dataclass
class ForceTorque:
    """Tool-frame wrench history. t is seconds relative to the newest sample (<= 0)."""
    t: Optional[np.ndarray] = None                   # (n,)
    values: Optional[np.ndarray] = None              # (n, 6)
    frame: str = 'tool'

    def latest(self):
        if self.values is None or len(self.values) == 0:
            return None
        return self.values[-1]


@dataclass
class Metric:
    name: str
    value: Optional[float]                           # None = unavailable (shown as n/a, not 0)
    unit: str = ''
    fmt: str = '{:.2f}'
    source: str = ''                                 # where the number comes from
    level: Health = Health.UNKNOWN                   # colour of the value
    note: str = ''                                   # tooltip: definition / why unavailable


@dataclass
class ScanMetrics:
    items: List[Metric] = field(default_factory=list)
    image_quality: Optional[str] = None              # e.g. Good / Low coupling; None = unknown
    image_quality_level: Health = Health.UNKNOWN


@dataclass
class TaskEvent:
    t_s: float                                       # seconds since episode start
    text: str
    level: Health = Health.WARN


@dataclass
class TaskState:
    phase: Optional[TaskPhase] = None                # None = no episode running
    outcome: Optional[Outcome] = None                # None = no episode
    progress: Optional[float] = None                 # 0..1 through the episode
    events: List[TaskEvent] = field(default_factory=list)
    detail: str = ''                                 # extra line, e.g. instruction text


@dataclass
class RecordingState:
    status: RecordingStatus = RecordingStatus.UNKNOWN
    session_id: Optional[str] = None
    episode_id: Optional[str] = None
    elapsed_s: Optional[float] = None
    total_s: Optional[float] = None                  # episode cap, if any
    output_path: Optional[str] = None
    warnings: List[str] = field(default_factory=list)


@dataclass
class NodeHealth:
    name: str
    topic: str = ''
    rate_hz: Optional[float] = None
    expected: str = ''                               # text shown in the tooltip, e.g. "26-29 Hz"
    health: Health = Health.UNKNOWN


@dataclass
class DashboardState:
    source_label: str = ''
    is_mock: bool = True
    system: SystemState = SystemState.DISCONNECTED
    contact: ContactState = ContactState.UNKNOWN
    plane: PlaneState = PlaneState.UNKNOWN
    robot: RobotState = field(default_factory=RobotState)
    ultrasound: UltrasoundFrame = field(default_factory=UltrasoundFrame)
    rgb: CameraFrame = field(default_factory=CameraFrame)
    depth: CameraFrame = field(default_factory=CameraFrame)
    ft: ForceTorque = field(default_factory=ForceTorque)
    metrics: ScanMetrics = field(default_factory=ScanMetrics)
    task: TaskState = field(default_factory=TaskState)
    recording: RecordingState = field(default_factory=RecordingState)
    nodes: List[NodeHealth] = field(default_factory=list)
    # Demo only: set while a real collection session runs (collection_guard.py). The window then
    # shows a paused screen and the source holds no display subscriptions.
    paused_reason: Optional[str] = None
