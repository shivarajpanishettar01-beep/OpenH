"""Demo only: the demo GUI stays off while a real dataset is being collected.

The demo GUI subscribes to the 500 Hz robot stream and every image stream at 30 Hz, about one CPU
core (docs/data_integrity.md). During collection that load belongs to the capture processes, and
the operator window is the image-free operator console. So in ROS mode the GUI checks, read-only,
whether episode_manager is running a real (non-sim) session and, while it is, drops every display
subscription and shows a paused screen instead.

Rule (pure function, tested without ROS):
  episode_manager not in the ROS graph            -> allowed (no collection)
  episode_manager up, /episode/status session.sim -> allowed (simulation, data goes to ~/data_sim)
  episode_manager up, anything else               -> paused (real session, or sim flag not known yet)
"""
from __future__ import annotations

from typing import Optional

EPISODE_MANAGER = 'episode_manager'
STATUS_STALE_S = 5.0          # /episode/status is published at 2 Hz; older than this counts as missing


def collection_lock(nodes_up: Optional[set], status: Optional[dict], status_age_s: Optional[float]) -> Optional[str]:
    """Reason the demo GUI must stay paused, or None when it may show live data.

    nodes_up: node names from the ROS graph (None = graph not read yet, which also pauses).
    status: last /episode/status payload (dict) or None; status_age_s: its age in seconds.
    """
    if nodes_up is None:
        return 'Checking for a running collection session.'
    if EPISODE_MANAGER not in nodes_up:
        return None
    fresh = status is not None and status_age_s is not None and status_age_s < STATUS_STALE_S
    session = ((status or {}).get('session') or {}) if fresh else {}
    if session.get('sim') is True:
        return None
    sid = session.get('session_id')
    if fresh and session.get('sim') is False:
        return f'Data collection in progress (session {sid}).' if sid else 'Data collection in progress.'
    return 'episode_manager is running and its session is not known to be a simulation.'
