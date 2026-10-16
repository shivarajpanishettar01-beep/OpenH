"""Data source interface. The GUI calls snapshot() on its timer and draws what it gets.

A source must never block in snapshot() and must never command the robot.
"""
from __future__ import annotations

from typing import List

from ..config import expected_band
from ..models import DashboardState, Health


class DataSource:
    label = 'source'
    is_mock = False
    scenarios: List[str] = []

    def start(self):
        pass

    def stop(self):
        pass

    def set_scenario(self, name: str):
        pass

    def snapshot(self) -> DashboardState:
        raise NotImplementedError


def rate_health(rate, expected_hz, node_up=True) -> Health:
    """Shared rule for node rows: down if missing, warn if outside the expected band."""
    if not node_up:
        return Health.DOWN
    band = expected_band(expected_hz)
    if band is None:
        return Health.OK
    if rate is None or rate <= 0:
        return Health.DOWN
    lo, hi = band
    return Health.OK if lo <= rate <= hi * 1.05 else Health.WARN


def expected_text(exp):
    """Tooltip text for an expected_hz entry."""
    if exp is None:
        return 'event-driven'
    if isinstance(exp, (int, float)):
        return f'~{exp:g} Hz'
    return f'{exp[0]:g}-{exp[1]:g} Hz'
