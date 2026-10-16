"""Loads config/demo_gui.yaml (installed share dir first, then the source tree)."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml


def default_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('usrig_demo_gui')) / 'config' / 'demo_gui.yaml'
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / 'config' / 'demo_gui.yaml'


def load(path: Optional[str] = None) -> dict:
    p = Path(path).expanduser() if path else default_path()
    return yaml.safe_load(p.read_text())


def expected_band(v):
    """expected_hz entry -> (lo, hi) or None."""
    if v is None:
        return None
    if isinstance(v, (int, float)):
        return float(v) * 0.9, float(v) * 1.1
    return float(v[0]), float(v[1])
