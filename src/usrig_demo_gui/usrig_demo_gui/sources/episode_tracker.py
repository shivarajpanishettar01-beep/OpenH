"""Turns /episode/events (EpisodeEvent) into task phase, outcome, recording state and an event log.

Pure Python, no ROS imports, so it can be tested and reused for bag replay.
Event kinds published by episode_manager / robot_control (see usrig_interfaces/msg/EpisodeEvent.msg):
start | end | phase | view_reached | perturbation | outcome | gel | note | key | motion | pause

Phase names (value.phase), checked against the publishers in the repo:
  episode_manager: free_air, free_air_end, search (Stage 2 view search), hold, perturbation, recovery,
                   onset, recovery_start, recovery_end (operator keys)
  robot_control:   approach, contact, search + path_length_m (a path run = the sweep), retract,
                   contact_lost, recontact, fault, fault_ack, zero_ft
robot_control's path run is told apart from the Stage 2 search by its path_length_m field and looked
up as 'search_path' in phase_map.
"""
from __future__ import annotations

import json
from typing import Dict, List, Optional

from ..models import Health, Outcome, RecordingStatus, TaskEvent, TaskPhase

FINALIZE_S = 2.0          # show "Finalizing" this long after an end event


def _phase(name: Optional[str]) -> Optional[TaskPhase]:
    if not name:
        return None
    try:
        return TaskPhase[name.upper()]
    except KeyError:
        return None


def _outcome(name: Optional[str]) -> Optional[Outcome]:
    if not name:
        return None
    try:
        return Outcome[name.upper()]
    except KeyError:
        return None


class EpisodeTracker:
    def __init__(self, phase_map: Dict[str, str], outcome_map: Dict[str, str]):
        self.phase_map = {k: _phase(v) for k, v in (phase_map or {}).items()}
        self.outcome_map = {k: _outcome(v) for k, v in (outcome_map or {}).items()}
        self.episode_id: Optional[str] = None
        self.t0_ns: Optional[int] = None
        self.t_end_ns: Optional[int] = None
        self.phase: Optional[TaskPhase] = None
        self.outcome: Optional[Outcome] = None
        self.info: dict = {}
        self.events: List[TaskEvent] = []
        self.active = False

    def _new_episode(self, eid, t_ns):
        self.episode_id, self.t0_ns, self.t_end_ns = eid, t_ns, None
        self.phase, self.outcome, self.info, self.events = None, Outcome.IN_PROGRESS, {}, []
        self.active = True

    def _log(self, t_ns, text, level=Health.UNKNOWN):
        t = (t_ns - self.t0_ns) * 1e-9 if self.t0_ns else 0.0
        self.events.append(TaskEvent(t, text, level))
        self.events = self.events[-20:]

    def handle(self, kind: str, value: str, stamp_ns: int, episode_id: str = ''):
        try:
            v = json.loads(value) if value else {}
        except ValueError:
            v = {'raw': value}
        if not isinstance(v, dict):
            v = {'value': v}
        if episode_id and episode_id != self.episode_id and kind != 'end':
            self._new_episode(episode_id, stamp_ns)          # first event of a new episode
        if kind == 'start':
            if not self.active:
                self._new_episode(episode_id or self.episode_id, stamp_ns)
            self.info = v
            self._log(stamp_ns, 'Episode started')
        elif kind == 'phase':
            name = v.get('phase')
            if name == 'search' and 'path_length_m' in v:
                name = 'search_path'
            ph = self.phase_map.get(name)
            if ph is not None:
                self.phase = ph
            if name in ('perturbation', 'onset'):
                self._log(stamp_ns, f'Disturbance ({name})', Health.WARN)
            elif name in ('recovery_start', 'recovery_end', 'recovery'):
                self._log(stamp_ns, name.replace('_', ' ').capitalize(), Health.WARN if name != 'recovery_end' else Health.OK)
            elif name == 'contact_lost':
                self._log(stamp_ns, 'Contact lost', Health.WARN)
            elif name == 'recontact':
                self._log(stamp_ns, 'Contact regained', Health.OK)
            elif name == 'fault':
                self._log(stamp_ns, f'Fault: {v.get("reason", "")}'[:80], Health.DOWN)
            elif name == 'search_path':
                self._log(stamp_ns, 'Path run started')
        elif kind == 'pause':
            self._log(stamp_ns, 'Paused' if v.get('state') == 'paused' else 'Resumed', Health.WARN)
        elif kind == 'perturbation':
            self._log(stamp_ns, f'Perturbation: {v.get("kind", "?")}', Health.WARN)
        elif kind == 'view_reached':
            self._log(stamp_ns, 'View reached', Health.OK)
        elif kind == 'note':
            self._log(stamp_ns, f'Note: {v.get("text", "")}'[:80])
        elif kind in ('outcome', 'end'):
            oc = self.outcome_map.get(v.get('outcome'))
            if oc is not None and oc != self.outcome:
                self.outcome = oc
                lvl = {Outcome.SUCCESS: Health.OK, Outcome.FAILURE: Health.DOWN}.get(oc, Health.WARN)
                self._log(stamp_ns, f'Outcome: {v.get("outcome")}', lvl)
            if kind == 'end':
                self.active = False
                self.t_end_ns = stamp_ns
                self.phase = None
                if self.outcome == Outcome.IN_PROGRESS:
                    self.outcome = None                       # ended without an outcome yet

    def elapsed_s(self, now_ns: int) -> Optional[float]:
        if self.t0_ns is None:
            return None
        end = now_ns if self.active else (self.t_end_ns or now_ns)
        return max((end - self.t0_ns) * 1e-9, 0.0)

    def recording_status(self, now_ns: int, recorder_up: Optional[bool]) -> RecordingStatus:
        if self.active:
            return RecordingStatus.RECORDING
        if self.t_end_ns and (now_ns - self.t_end_ns) * 1e-9 < FINALIZE_S:
            return RecordingStatus.FINALIZING
        if recorder_up is None:
            return RecordingStatus.UNKNOWN
        return RecordingStatus.RECORDING if recorder_up else RecordingStatus.IDLE
