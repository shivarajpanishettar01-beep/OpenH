"""Trajectory map for the robot view: how the tool moved from the rest position to the scan.

Input per GUI frame: the tracked tool frames (flange, probe tip, wrist camera; frames.py, from
/robot/tcp_pose + calibration), the contact state and the task phase (/episode/events).

A *run* starts when the robot leaves a rest: stationary for `rest_s` while no episode phase is
active. The rest pose is the first point of the run, so the map shows the whole motion
rest -> approach -> contact -> scan -> retract. The previous run stays on screen while the robot
is parked and is replaced when it moves off again.

Per sample: time, the three positions (any may be None if not calibrated), contact (True / False
/ None) and phase. Markers are placed at the run start and at every phase change; a marker that
falls within `merge_m` of the previous one is merged into it ("Contact / View acq.") so labels
do not overlap.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .frames import ToolFrames

SHORT = {'Approach': 'Approach', 'Contact': 'Contact', 'View Acquisition': 'View acq.', 'Sweep': 'Sweep',
         'Reacquisition': 'Reacq.', 'Recovery': 'Recovery'}


# Labels written next to markers in the robot view. The full phase sequence is on the scan
# timeline, so the map names only the points that orient the reader; other changes are dots.
MAP_LABELS = ('Rest', 'Start', 'Contact', 'Recovery', 'End')


def map_label(label: str) -> str:
    return ' / '.join(p for p in label.split(' / ') if p in MAP_LABELS)


@dataclass
class Marker:
    pos: np.ndarray
    label: str


def _p(T):
    return None if T is None else np.asarray(T)[:3, 3].copy()


class TrajectoryMap:
    def __init__(self, rest_s=1.0, rest_tol_m=0.0005, min_step_m=0.0005, max_s=180.0, merge_m=0.006):
        self.rest_s, self.rest_tol_m = float(rest_s), float(rest_tol_m)
        self.min_step_m, self.max_s, self.merge_m = float(min_step_m), float(max_s), float(merge_m)
        self.clear()
        self._still_ref = None
        self._still_t = None
        self.at_rest = False
        self.rest_pose = None          # ToolFrames positions of the last detected rest

    def clear(self):
        self.t: List[float] = []
        self.flange_p: List[Optional[np.ndarray]] = []
        self.tip_p: List[Optional[np.ndarray]] = []
        self.camera_p: List[Optional[np.ndarray]] = []
        self.contact_s: List[Optional[bool]] = []
        self.phase_s: List[Optional[str]] = []
        self.markers: List[Marker] = []
        self._last_phase = None
        self._had_phase = False

    def __len__(self):
        return len(self.t)

    # ------------------------------------------------------------------ input
    @staticmethod
    def _ref(fl, tp, cm):
        return tp if tp is not None else (fl if fl is not None else cm)

    def update(self, t: float, frames: ToolFrames, contact: Optional[bool], phase: Optional[str]) -> bool:
        """phase: phase name ('Approach', ...) or None when no episode is running. Returns True if changed."""
        fl, tp, cm = _p(frames.flange), _p(frames.tip), _p(frames.camera)
        ref = self._ref(fl, tp, cm)
        if ref is None:
            return False
        contact = None if contact is None else bool(contact)
        changed = False

        # rest detection: stationary for rest_s with no episode phase (time inside an episode, e.g. a
        # hold at the end of the scan, does not count towards the rest)
        if phase is not None and self._still_ref is not None:
            self._still_t = t
        if self._still_ref is None or np.linalg.norm(ref - self._still_ref) > self.rest_tol_m:
            moving_off = self.at_rest
            self._still_ref, self._still_t = ref, t
            if moving_off:                                  # leaving a rest: start a new run there
                self.at_rest = False
                self.clear()
                r = self.rest_pose
                self._append(self._still_t, r['flange'], r['tip'], r['camera'], r['contact'], None)
                self.markers.append(Marker(self._ref(r['flange'], r['tip'], r['camera']), 'Rest'))
                changed = True
        elif phase is None and t - self._still_t >= self.rest_s and not self.at_rest:
            self.at_rest = True
            self.rest_pose = dict(flange=fl, tip=tp, camera=cm, contact=contact)

        # sample
        last = self._ref(self.flange_p[-1], self.tip_p[-1], self.camera_p[-1]) if self.t else None
        if last is None or np.linalg.norm(ref - last) >= self.min_step_m or contact != self.contact_s[-1] \
                or phase != self.phase_s[-1]:
            if not self.t:
                self.markers.append(Marker(ref, 'Start'))
            self._append(t, fl, tp, cm, contact, phase)
            changed = True

        # phase markers
        if phase != self._last_phase:
            if phase is not None:
                self._mark(ref, SHORT.get(phase, phase))
                self._had_phase = True
            elif self._had_phase:
                self._mark(ref, 'End')
            self._last_phase = phase
            changed = True

        while self.t and t - self.t[0] > self.max_s:
            for lst in (self.t, self.flange_p, self.tip_p, self.camera_p, self.contact_s, self.phase_s):
                lst.pop(0)
            changed = True
        return changed

    def _append(self, t, fl, tp, cm, contact, phase):
        self.t.append(t)
        self.flange_p.append(fl)
        self.tip_p.append(tp)
        self.camera_p.append(cm)
        self.contact_s.append(contact)
        self.phase_s.append(phase)

    def _mark(self, pos, label):
        if self.markers and np.linalg.norm(self.markers[-1].pos - pos) < self.merge_m:
            if label not in self.markers[-1].label.split(' / '):
                self.markers[-1].label += ' / ' + label
        else:
            self.markers.append(Marker(pos, label))

    # ------------------------------------------------------------------ output
    @staticmethod
    def _arr(lst):
        pts = [p for p in lst if p is not None]
        return np.array(pts) if pts else np.zeros((0, 3))

    def flange(self) -> np.ndarray:
        return self._arr(self.flange_p)

    def camera(self) -> np.ndarray:
        return self._arr(self.camera_p)

    def tip(self) -> np.ndarray:
        return self._arr(self.tip_p)

    def tip_in_air(self) -> np.ndarray:
        """Per tip point: True where the probe was known to be off the phantom (unknown is not air)."""
        return np.array([c is False for p, c in zip(self.tip_p, self.contact_s) if p is not None], bool)

    def path_length_m(self, which='tip') -> float:
        p = {'tip': self.tip, 'flange': self.flange, 'camera': self.camera}[which]()
        return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0
